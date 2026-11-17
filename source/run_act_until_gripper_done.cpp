#include "bt_action/run_act_until_gripper_done.hpp"

#include <cstring>
#include <sstream>

namespace {
// Native POD prefix of the driver's 248-byte ActSocketInferenceData payload.
// Decode with memcpy: protobuf bytes need not be aligned. Reject other layouts.
struct alignas(8) ActDataPrefix {
  std::uint32_t struct_size, valid;
  std::int32_t enabled, ready, connected, last_error;
  std::uint64_t inference_count, action_count;
  std::int32_t send_enabled, run_state;
  std::uint64_t sequence, generation, send_generation;
};
static_assert(sizeof(ActDataPrefix) == 72);
constexpr std::size_t kActDataSize = 248;
// The PI0.5 driver extends phase values while preserving its 256-byte NRT ABI.
struct alignas(8) Pi05DataPrefix {
  std::uint32_t struct_size, valid;
  std::int32_t ready, connected, update_enabled, send_enabled, phase, last_error;
  std::uint64_t generation;
};
static_assert(sizeof(Pi05DataPrefix) == 40);
constexpr std::size_t kPi05DataSize = 256;
constexpr std::int32_t kWaitingClose = 0x41430001, kWaitingOpen = 0x41430002;
constexpr std::int32_t kCompleted = 0x41430003, kTimedOut = 0x41430004;
constexpr std::int32_t kFeedbackFault = 0x41430005, kInferenceFault = 0x41430006;
constexpr std::int32_t kGraspFailed = 0x41430007;

bool isPi05Name(const std::string& name) {
  return name == "Pi05Inference" || name.rfind("Pi05Inference#", 0) == 0;
}

std::int32_t pi05RunState(std::int32_t phase) {
  switch (phase) {
    case 1: case 2: case 4: return kWaitingClose;  // connecting, warming, executing
    case 5: return kInferenceFault;
    case 6: return kWaitingOpen;
    case 7: return kCompleted;
    case 8: return kGraspFailed;
    default: return 0;
  }
}

bool isActName(const std::string& name) {
  return name == "ActSocketInference" || name == "act_socket_inference" ||
         name.rfind("ActSocketInference#", 0) == 0;
}
}  // namespace

BT::PortsList RunActUntilGripperDone::providedPorts() {
  return {
      BT::InputPort<std::vector<int>>("robot_sel", "0", "Select one inference command model"),
      BT::InputPort<std::string>("subsystem_name", "", "Inference instance; empty selects the only instance"),
      BT::InputPort<int>("gripper_model", "Gripper feedback model index (required)"),
      BT::InputPort<double>("closed_mm", 5.0, "Closed when opening is at most this value"),
      BT::InputPort<double>("finish_opening_mm", 50.0, "Final opening after a close in this run"),
      BT::InputPort<double>("tolerance_mm", 2.0, "Final opening tolerance"),
      BT::InputPort<int>("closed_stable_ms", 200, "Closed feedback hold time"),
      BT::InputPort<int>("stable_ms", 500, "Final opening hold time"),
      BT::InputPort<int>("timeout_ms", 120000, "Task and driver run time limit"),
      BT::InputPort<int>("feedback_timeout_ms", 1000, "Maximum age of advancing RT feedback"),
      BT::InputPort<int>("command_timeout_ms", 2000, "Start/stop acknowledgement timeout"),
      BT::OutputPort<std::string>("phase"),
      BT::OutputPort<std::string>("error"),
      BT::OutputPort<std::string>("result", "completed, grasp_failed, fault, running or canceled"),
      BT::OutputPort<double>("opening_mm"),
  };
}

nlohmann::json RunActUntilGripperDone::command(bool enable) const {
  bt_manager_cmd::CmdDynBuilder builder;
  builder.init_models(static_cast<std::size_t>(model_count_))
      .select_models(std::vector<int>{command_model_})
      .set_event(pi05_ ? (enable ? "SetPi05Inference" : "StopPi05GripperRun") : "SetActSocketInference")
      .add_field({{"update_enable", enable ? 1 : 0}, {"send_enable", enable ? 1 : 0}});
  if (pi05_) builder.add_field({{"expected_generation", std::to_string(baseline_generation_ + (enable ? 0U : 1U))}});
  if (!subsystem_name_.empty()) builder.add_field({{"subsystem_name", subsystem_name_}});
  if (enable) builder.add_field({{"run_timeout_ms", timeout_ms_}, {"expected_gripper_model", gripper_model_},
      {"finish_on_gripper", 1}, {"closed_mm", closed_mm_}, {"finish_opening_mm", finish_mm_},
      {"tolerance_mm", tolerance_mm_}, {"closed_stable_ms", closed_stable_ms_},
      {"stable_ms", stable_ms_}, {"feedback_timeout_ms", feedback_timeout_ms_}});
  return builder.build();
}

bool RunActUntilGripperDone::readActHealth(const BTManager::NrtSnapshot& snapshot,
                                          ActHealth& health, std::string& error) const {
  if (snapshot.received_at == Clock::time_point{} || !snapshot.data.has_head() ||
      Clock::now() - snapshot.received_at > std::chrono::milliseconds(feedback_timeout_ms_)) {
    error = "fresh inference subsystem NRT status is required";
    return false;
  }
  if (snapshot.advanced_at == Clock::time_point{} ||
      Clock::now() - snapshot.advanced_at > std::chrono::milliseconds(feedback_timeout_ms_)) {
    error = "inference subsystem NRT status stopped advancing";
    return false;
  }
  const overall_system_nrtstate::Subsystem* selected = nullptr;
  for (const auto& subsystem : snapshot.data.subsystem()) {
    if (subsystem_name_.empty() ? !(pi05_ ? isPi05Name(subsystem.subsystem_name()) : isActName(subsystem.subsystem_name()))
                                : subsystem.subsystem_name() != subsystem_name_) continue;
    if (selected) {
      error = "ambiguous inference subsystem NRT status; specify a unique subsystem_name";
      return false;
    }
    selected = &subsystem;
  }
  const auto payload_size = pi05_ ? kPi05DataSize : kActDataSize;
  if (!selected || selected->state() != 1 || selected->data().size() != payload_size) {
    error = "inference subsystem NRT status is missing or its payload length is incompatible";
    return false;
  }
  const auto bit = [](std::int32_t value) { return value == 0 || value == 1; };
  if (pi05_) {
    Pi05DataPrefix data{};
    std::memcpy(&data, selected->data().data(), sizeof(data));
    if (data.struct_size != kPi05DataSize || !bit(data.ready) || !bit(data.connected) ||
        !bit(data.update_enabled) || !bit(data.send_enabled) || data.phase < 0 || data.phase > 8) {
      error = "PI0.5 subsystem NRT status has an incompatible payload format";
      return false;
    }
    health = {selected->subsystem_name(), data.update_enabled, data.ready, data.send_enabled,
              data.last_error, pi05RunState(data.phase), data.generation, 0};
    return true;
  }
  ActDataPrefix data{};
  std::memcpy(&data, selected->data().data(), sizeof(data));
  if (data.struct_size != kActDataSize || !bit(data.enabled) || !bit(data.ready) ||
      !bit(data.send_enabled) || !bit(data.connected)) {
    error = "ACT subsystem NRT status has an incompatible payload format";
    return false;
  }
  health = {selected->subsystem_name(), data.enabled, data.ready, data.send_enabled,
            data.last_error, data.run_state, data.generation, data.send_generation};
  return true;
}

RunActUntilGripperDone::HealthCheck RunActUntilGripperDone::checkActHealth(std::string& error) {
  const auto snapshot = G_BT_MANAGER()->get_nrt_snapshot();
  ActHealth health;
  if (!readActHealth(snapshot, health, error)) return HealthCheck::Failed;
  if (snapshot.regression_count != nrt_regression_baseline_) {
    error = "inference subsystem NRT counter moved backwards";
    return HealthCheck::Failed;
  }
  const bool terminal = health.run_state >= kCompleted && health.run_state <= kGraspFailed;
  const auto expected_generation = baseline_generation_ + (!pi05_ && (baseline_generation_ & 1U) ? 2U : 1U);
  const bool this_run = pi05_ ? health.generation == expected_generation + 1 : health_confirmed_
      ? (health.generation == active_generation_ || health.generation == active_generation_ + 1)
      : (health.generation == expected_generation || health.generation == expected_generation + 1);
  if (terminal && this_run) {
    // A terminal marker can precede the gate snapshot by one publication. Wait for both.
    if (health.enabled || health.send_enabled) return HealthCheck::Waiting;
    if (health.run_state == kCompleted) return HealthCheck::Completed;
    if (health.run_state == kGraspFailed) {
      error = pi05_ ? "PI0.5 grasp failed: gripper opening is below 5 mm" :
                       "ACT grasp failed: gripper opening is below 5 mm";
      LOG_WARN("[RunActUntilGripperDone] grasp_failed confirmed; both ACT gates closed; generation={}",
               health.generation);
      return HealthCheck::GraspFailed;
    }
    if (pi05_) {
      error = "PI0.5 driver faulted; query ShowPi05InferenceData for its stop reason";
      return HealthCheck::Stopped;
    }
    error = health.run_state == kTimedOut ? "ACT subsystem run timed out" :
            health.run_state == kFeedbackFault ? "ACT subsystem gripper feedback faulted" :
                                                "ACT subsystem inference or action faulted";
    return HealthCheck::Stopped;
  }
  if (!health_confirmed_) {
    // A start acknowledgement can arrive before updateData publishes the new run.
    // Old cached status must neither fail the new run nor authorize completion.
    if (health.generation < expected_generation || (!pi05_ && !(health.generation & 1U))) {
      if (Clock::now() < health_deadline_) return HealthCheck::Waiting;
      error = "ACT subsystem did not publish the started run before the status timeout";
      return HealthCheck::Failed;
    }
    if (pi05_ && health.generation != expected_generation) {
      error = "PI0.5 subsystem generation changed before the start was confirmed";
      return HealthCheck::Failed;
    }
  } else if (health.generation != active_generation_ ||
             health.send_generation != active_send_generation_) {
    error = "inference subsystem run changed during gripper monitoring";
    return HealthCheck::Failed;
  }
  if (!health.ready || !health.enabled || !health.send_enabled || health.last_error != 0) {
    error = "ACT subsystem stopped or faulted (last_error=" + std::to_string(health.last_error) + ")";
    return HealthCheck::Failed;
  }
  if (health.run_state != kWaitingClose && health.run_state != kWaitingOpen) {
    error = "inference driver did not arm gripper completion; check driver and command mappings";
    return HealthCheck::Failed;
  }
  if (!health_confirmed_) {
    health_confirmed_ = true;
    active_generation_ = health.generation;
    active_send_generation_ = health.send_generation;
  }
  phase_ = health.run_state == kWaitingOpen ? Phase::WaitingOpen : Phase::WaitingClose;
  setOutput("phase", std::string(health.run_state == kWaitingOpen ? "waiting_open" : "waiting_close"));
  return HealthCheck::Healthy;
}

bool RunActUntilGripperDone::validFeedback(const BTManager::RtSnapshot& snapshot, std::string* error) const {
  const auto& rt = snapshot.data;
  const auto reject = [&](const std::string& reason) {
    if (error) *error = reason;
    return false;
  };
  if (snapshot.received_at == Clock::time_point{} ||
      Clock::now() - snapshot.received_at > std::chrono::milliseconds(feedback_timeout_ms_) ||
      !rt.has_head()) return reject("RT feedback is missing or stale");
  if (snapshot.advanced_at == Clock::time_point{} ||
      Clock::now() - snapshot.advanced_at > std::chrono::milliseconds(feedback_timeout_ms_))
    return reject("RT feedback stopped advancing");
  if (!rt.system_running_state()) return reject("RT reports control system not running");
  if (gripper_model_ < 0 || gripper_model_ >= rt.model_size() ||
      command_model_ < 0 || command_model_ >= rt.model_size()) return reject("RT model index is unavailable");
  const auto& model = rt.model(gripper_model_);
  if (model.joint_size() != 1) return reject("RT gripper model must contain exactly one joint");
  if (model.error_code() != 0)
    return reject("RT gripper model fault: " + std::to_string(model.error_code()));
  const auto& arm = rt.model(command_model_);
  if (arm.error_code() != 0) return reject("RT arm model fault: " + std::to_string(arm.error_code()));
  if (arm.joint_size() == 0) return reject("RT arm model contains no joints");
  for (int i = 0; i < arm.joint_size(); ++i) {
    const auto& axis = arm.joint(i);
    if (!axis.is_enabled() || axis.error_code() != 0)
      return reject("RT arm joint " + std::to_string(i) + " disabled or faulted (error=" +
                    std::to_string(axis.error_code()) + ")");
  }
  const auto& joint = model.joint(0);
  const double opening = joint.position() * 1000.0;
  if (!joint.is_enabled() || joint.error_code() != 0)
    return reject("RT gripper joint disabled or faulted (error=" + std::to_string(joint.error_code()) + ")");
  if (!std::isfinite(opening) || opening < 0.0 || opening > 65.0 + tolerance_mm_)
    return reject("RT gripper opening is invalid: " + std::to_string(opening));
  return true;
}

std::string RunActUntilGripperDone::diagnosticContext() const {
  const auto* manager = G_BT_MANAGER();
  if (!manager) return "manager unavailable";
  const auto now = Clock::now();
  const auto age = [&](const auto& snapshot) -> long long {
    if (snapshot.received_at == Clock::time_point{}) return -1;
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - snapshot.received_at).count();
  };
  const auto rt = manager->get_rt_snapshot();
  const auto nrt = manager->get_nrt_snapshot();
  std::ostringstream out;
  out << "phase=" << static_cast<int>(phase_) << " instance=" << subsystem_name_
      << " rt_age_ms=" << age(rt) << " rt_frame=" << rt.data.head().frame_id()
      << " rt_stamp=" << rt.data.head().timestamp()
      << " nrt_age_ms=" << age(nrt) << " nrt_frame=" << nrt.data.head().frame_id()
      << " nrt_stamp=" << nrt.data.head().timestamp()
      << " active_generation=" << active_generation_ << " baseline_generation=" << baseline_generation_;
  out << " rt_progress_age_ms=" << (rt.advanced_at == Clock::time_point{} ? -1 :
      std::chrono::duration_cast<std::chrono::milliseconds>(now - rt.advanced_at).count())
      << " nrt_progress_age_ms=" << (nrt.advanced_at == Clock::time_point{} ? -1 :
      std::chrono::duration_cast<std::chrono::milliseconds>(now - nrt.advanced_at).count());
  ActHealth health;
  std::string error;
  if (readActHealth(nrt, health, error))
    out << " generation=" << health.generation << " run_state=" << health.run_state
        << " send_generation=" << health.send_generation << " ready=" << health.ready
        << " enabled=" << health.enabled << " send_enabled=" << health.send_enabled
        << " last_error=" << health.last_error;
  return out.str();
}

BT::NodeStatus RunActUntilGripperDone::stopUnconfirmed(const std::string& detail) {
  // A rejected stop must not replace the first failure or be resent immediately.
  owns_run_ = false;
  const auto message = failure_reason_ + "; ACT stop unconfirmed: " + detail +
                       "; subsystem supervision/run timeout remains active";
  setOutput("phase", std::string("stop_unconfirmed"));
  setOutput("error", message);
  LOG_ERROR("[RunActUntilGripperDone] {}", message);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RunActUntilGripperDone::failBeforeStart(const std::string& error) {
  failure_reason_ = error;
  setOutput("result", std::string("fault"));
  setOutput("phase", std::string("failed"));
  setOutput("error", error);
  LOG_ERROR("[RunActUntilGripperDone] start rejected: node={} error={}; {}",
            name(), error, diagnosticContext());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RunActUntilGripperDone::onStart() {
  auto* manager = G_BT_MANAGER();
  phase_ = Phase::Starting;
  failure_reason_.clear();
  // Clear a previous recoverable result before any validation can fail.
  setOutput("result", std::string("fault"));
  setOutput("error", std::string{});
  setOutput("phase", std::string("starting"));
  std::vector<int> selected;
  const bool inputs = getInput("robot_sel", selected) && selected.size() == 1 &&
      getInput("subsystem_name", subsystem_name_) && getInput("gripper_model", gripper_model_) &&
      getInput("closed_mm", closed_mm_) && getInput("finish_opening_mm", finish_mm_) &&
      getInput("tolerance_mm", tolerance_mm_) && getInput("closed_stable_ms", closed_stable_ms_) &&
      getInput("stable_ms", stable_ms_) && getInput("timeout_ms", timeout_ms_) &&
      getInput("feedback_timeout_ms", feedback_timeout_ms_) &&
      getInput("command_timeout_ms", command_timeout_ms_);
  if (!inputs)
    return failBeforeStart("invalid or missing ACT inputs; robot_sel must select exactly one model");
  if (!manager) return failBeforeStart("BT manager unavailable");
  if (manager->is_stopping()) return failBeforeStart("BT manager stopping");
  if (!manager->is_conn_codeit()) return failBeforeStart("Codeit driver disconnected");
  if (!std::isfinite(closed_mm_) || !std::isfinite(finish_mm_) || !std::isfinite(tolerance_mm_) ||
      closed_mm_ < 0.0 || tolerance_mm_ <= 0.0 || finish_mm_ > 65.0 ||
      finish_mm_ - tolerance_mm_ <= closed_mm_ + tolerance_mm_ ||
      stable_ms_ <= 0 || closed_stable_ms_ <= 0 || timeout_ms_ <= stable_ms_ ||
      feedback_timeout_ms_ <= 0 || command_timeout_ms_ <= 0) {
    std::ostringstream settings;
    settings << "invalid ACT completion/timeout settings: closed_mm=" << closed_mm_
             << " finish_opening_mm=" << finish_mm_ << " tolerance_mm=" << tolerance_mm_
             << " closed_stable_ms=" << closed_stable_ms_ << " stable_ms=" << stable_ms_
             << " timeout_ms=" << timeout_ms_ << " feedback_timeout_ms=" << feedback_timeout_ms_
             << " command_timeout_ms=" << command_timeout_ms_;
    return failBeforeStart(settings.str());
  }
  command_model_ = selected.front();
  model_count_ = manager->get_model_size();
  const auto snapshot = manager->get_rt_snapshot();
  if (command_model_ < 0 || command_model_ >= model_count_)
    return failBeforeStart("ACT command model index is unavailable: " + std::to_string(command_model_));
  if (gripper_model_ == command_model_)
    return failBeforeStart("ACT command model and gripper feedback model must differ");
  std::string feedback_error;
  if (!validFeedback(snapshot, &feedback_error)) return failBeforeStart(feedback_error);
  const auto nrt = manager->get_nrt_snapshot();
  ActHealth health;
  std::string health_error;
  if (!readActHealth(nrt, health, health_error) || !health.ready) {
    return failBeforeStart(health_error.empty() ? std::string("ACT subsystem is not ready") : health_error);
  }
  if (pi05_ && (health.enabled || health.send_enabled || !health.generation))
    return failBeforeStart("PI0.5 supervised start requires idle gates and a published generation");
  subsystem_name_ = health.name;
  baseline_generation_ = health.generation;
  health_confirmed_ = false;
  nrt_regression_baseline_ = nrt.regression_count;
  rt_regression_baseline_ = snapshot.regression_count;
  deadline_ = Clock::now() + std::chrono::milliseconds(timeout_ms_);
  phase_ = Phase::Starting;
  owns_run_ = true;  // 请求失败或超时也可能已经改变驱动，必须尝试停止。
  setOutput("result", std::string("running"));
  if (!request_.start(manager->get_reqrep(), "main.handler.req.dyncmd", command(true),
                      std::chrono::milliseconds(command_timeout_ms_), manager->tickNotifier()))
    return stopWithResult(false, "ACT start request could not be created");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RunActUntilGripperDone::stopWithResult(bool success, const std::string& error) {
  setOutput("result", std::string("fault"));
  failure_reason_ = error;
  LOG_ERROR("[RunActUntilGripperDone] first failure: {}; {}", error, diagnosticContext());
  request_.cancel();
  stop_success_ = success;
  phase_ = Phase::Stopping;
  setOutput("phase", std::string("stopping"));
  setOutput("error", error);
  auto* manager = G_BT_MANAGER();
  if (!manager || !request_.start(manager->get_reqrep(), "main.handler.req.dyncmd", command(false),
                                  std::chrono::milliseconds(command_timeout_ms_), manager->tickNotifier())) {
    return stopUnconfirmed("stop request could not be created");
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RunActUntilGripperDone::onRunning() {
  auto* manager = G_BT_MANAGER();
  if (phase_ == Phase::Stopping) {
    const auto stop_message = request_.message();
    const auto result = request_.tick("RunActUntilGripperDone stop");
    if (result == BT::NodeStatus::RUNNING) return result;
    if (result == BT::NodeStatus::FAILURE) {
      return stopUnconfirmed(stop_message.empty() ? "request rejected or timed out" : stop_message);
    }
    owns_run_ = false;
    setOutput("phase", std::string(stop_success_ ? "done" : "failed"));
    return stop_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  if (!manager || manager->is_stopping() || !manager->is_conn_codeit())
    return stopWithResult(false, "driver disconnected or manager stopping");
  if (Clock::now() >= deadline_) return stopWithResult(false, "ACT task timed out");
  const auto snapshot = manager->get_rt_snapshot();
  std::string feedback_error;
  if (!validFeedback(snapshot, &feedback_error)) return stopWithResult(false, feedback_error);
  if (snapshot.regression_count != rt_regression_baseline_)
    return stopWithResult(false, "RT feedback counter moved backwards");
  setOutput("opening_mm", snapshot.data.model(gripper_model_).joint(0).position() * 1000.0);
  if (phase_ == Phase::Starting) {
    const auto result = request_.tick("RunActUntilGripperDone start");
    if (result == BT::NodeStatus::RUNNING) return result;
    if (result == BT::NodeStatus::FAILURE) return stopWithResult(false, "ACT start was rejected or timed out");
    // Completion edges are owned by the subsystem. RT/NRT freshness is recorded
    // at reception, independently of this tree's tick period and startup wait.
    phase_ = Phase::WaitingClose;
    health_deadline_ = Clock::now() + std::chrono::milliseconds(command_timeout_ms_);
    setOutput("phase", std::string("waiting_status"));
    return BT::NodeStatus::RUNNING;
  }
  std::string health_error;
  const auto health = checkActHealth(health_error);
  if (health == HealthCheck::Completed || health == HealthCheck::GraspFailed || health == HealthCheck::Stopped) {
    if (health == HealthCheck::Stopped)
      LOG_ERROR("[RunActUntilGripperDone] ACT terminal fault: {}; {}", health_error, diagnosticContext());
    owns_run_ = false;
    setOutput("result", std::string(health == HealthCheck::Completed ? "completed" :
                                   health == HealthCheck::GraspFailed ? "grasp_failed" : "fault"));
    setOutput("phase", std::string(health == HealthCheck::Completed ? "done" :
                                  health == HealthCheck::GraspFailed ? "grasp_failed" : "failed"));
    setOutput("error", health_error);
    return health == HealthCheck::Completed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  if (health == HealthCheck::Failed) return stopWithResult(false, health_error);
  if (health == HealthCheck::Waiting) return BT::NodeStatus::RUNNING;
  return BT::NodeStatus::RUNNING;
}

void RunActUntilGripperDone::stopBounded() noexcept {
  if (!owns_run_) return;
  try {
    auto* manager = G_BT_MANAGER();
    if (manager) {
      const auto result = ReqRepJson::call_sync(manager->get_reqrep(), "main.handler.req.dyncmd",
                                               command(false), std::chrono::milliseconds(500));
      if (!result.ok) LOG_ERROR("[RunActUntilGripperDone] stop unconfirmed: {}", result.message);
    }
  } catch (...) {
    // 生命周期退出不得抛异常；驱动的 run_timeout_ms 负责失联兜底。
  }
  owns_run_ = false;
}

void RunActUntilGripperDone::onHalted() {
  setOutput("result", std::string("canceled"));
  request_.cancel();
  stopBounded();
}
