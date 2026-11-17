#include "bt_action/move_absj_joint_target.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <set>
#include <iomanip>
#include <locale>
#include <sstream>

namespace {
bool identifier(const std::string& text) {
  if (text.empty() || text.size() > 128) return false;
  const auto start = static_cast<unsigned char>(text.front());
  if (!(std::isalpha(start) || start == '_' || start >= 128)) return false;
  return std::all_of(text.begin(), text.end(), [](unsigned char ch) {
    return std::isalnum(ch) || ch == '_' || ch >= 128;
  });
}

template <typename Snapshot>
bool fresh(const Snapshot& snapshot, int timeout_ms) {
  const auto now = std::chrono::steady_clock::now();
  const auto limit = std::chrono::milliseconds(timeout_ms);
  return snapshot.data.has_head() && snapshot.received_at != decltype(now){} &&
      snapshot.advanced_at != decltype(now){} && now - snapshot.received_at <= limit &&
      now - snapshot.advanced_at <= limit;
}
}  // namespace

BT::PortsList MoveAbsJJointTarget::providedPorts() {
  return {
      BT::InputPort<std::vector<int>>("robot_sel", "0;1", "Robot selection; use 0 for the ACT arm"),
      BT::InputPort<std::string>("jointtarget", "P0", "Named joint target in the selected model"),
      BT::InputPort<std::string>("speed", "v100", "Move speed name"),
      BT::InputPort<std::string>("jointtarget_values", "", "Optional joint values separated by semicolons; rad or m"),
      BT::InputPort<bool>("skip_if_at_target", false, "Succeed without motion if already stably at target"),
      BT::InputPort<std::string>("feedback_subsystem", "", "Optional YSEG2 subsystem actual_pos feedback (metres)"),
      BT::InputPort<int>("timeout_ms", 60000, "Total idle, command and arrival timeout"),
      BT::InputPort<int>("feedback_timeout_ms", 1000, "Maximum RT/NRT feedback age"),
      BT::InputPort<int>("idle_stable_ms", 200, "Idle and stationary observations before submission"),
      BT::InputPort<int>("stable_ms", 200, "Stationary target observations after acknowledgement"),
      BT::InputPort<double>("position_tolerance_rad", 0.01, "Joint target position tolerance"),
      BT::InputPort<double>("velocity_tolerance_rad_s", 0.01, "Stationary joint velocity tolerance"),
      BT::OutputPort<std::string>("phase"),
      BT::OutputPort<std::string>("error"),
  };
}

bool MoveAbsJJointTarget::resolveTargets(const BTManager::RtSnapshot& rt,
    const BTManager::NrtSnapshot& nrt, std::vector<Target>& targets, std::string& error) const {
  targets.clear();
  for (const auto index : selected_) {
    if (index < 0 || index >= rt.data.model_size() || index >= nrt.data.model_size()) {
      error = "selected model is missing from RT/NRT feedback";
      return false;
    }
    const auto& arm = rt.data.model(index);
    const auto& model = nrt.data.model(index);
    if (arm.joint_size() == 0 || (!arm.model_name().empty() && !model.model_name().empty() &&
                                arm.model_name() != model.model_name())) {
      error = "selected model has no joints or RT/NRT model identities differ";
      return false;
    }
    Target target{index, {}};
    if (!jointtarget_values_.empty()) {
      if (jointtarget_values_.size() != static_cast<std::size_t>(arm.joint_size())) {
        error = "explicit joint target dimension differs from the selected model";
        return false;
      }
      target.joints = jointtarget_values_;
      targets.push_back(std::move(target));
      continue;
    }
    int matches = 0;
    for (const auto& point : model.teach_points()) {
      if (point.point_name() != jointtarget_) continue;
      ++matches;
      // PointData stores ten joint slots; only the selected model DOF is used.
      if (point.jointtarget_size() >= arm.joint_size())
        target.joints.assign(point.jointtarget().begin(),
                             point.jointtarget().begin() + arm.joint_size());
    }
    for (const auto& variable : model.matrix_variables()) {
      if (variable.matrixvar_name() != jointtarget_) continue;
      ++matches;
      if (variable.data_size() == arm.joint_size())
        target.joints.assign(variable.data().begin(), variable.data().end());
    }
    if (matches != 1 || target.joints.size() != static_cast<std::size_t>(arm.joint_size()) ||
        !std::all_of(target.joints.begin(), target.joints.end(),
                     [](double value) { return std::isfinite(value); })) {
      error = "jointtarget '" + jointtarget_ + "' is missing, ambiguous or has invalid joint data in model " +
              std::to_string(index);
      return false;
    }
    targets.push_back(std::move(target));
  }
  return true;
}

bool MoveAbsJJointTarget::readFeedback(const BTManager::RtSnapshot& rt,
    const BTManager::NrtSnapshot& nrt, std::string& error) {
  if (!fresh(rt, feedback_timeout_ms_) || !fresh(nrt, feedback_timeout_ms_)) {
    error = "RT/NRT feedback is missing, stale or stopped advancing";
    return false;
  }
  if (rt.regression_count != rt_regressions_ || nrt.regression_count != nrt_regressions_) {
    error = "RT/NRT feedback counter moved backwards";
    return false;
  }
  if (!rt.data.system_running_state() || G_BT_MANAGER()->get_model_size() != model_count_) {
    error = "control system stopped or model layout changed";
    return false;
  }
  std::vector<Target> current;
  if (!resolveTargets(rt, nrt, current, error)) return false;
  for (std::size_t i = 0; i < current.size(); ++i) {
    if (current[i].joints != targets_[i].joints) {
      error = "jointtarget changed during the move";
      return false;
    }
    const auto& arm = rt.data.model(current[i].model);
    if (arm.error_code() != 0) {
      error = "selected model fault: " + arm.error_msg();
      return false;
    }
    for (const auto& joint : arm.joint()) {
      if (!joint.is_enabled() || joint.error_code() != 0 ||
          !std::isfinite(joint.position()) || !std::isfinite(joint.velocity())) {
        error = "selected joint is disabled, faulted or has non-finite feedback";
        return false;
      }
    }
  }
  if (!feedback_subsystem_.empty()) {
    const overall_system_nrtstate::Subsystem* physical = nullptr;
    for (const auto& subsystem : nrt.data.subsystem()) {
      if (subsystem.subsystem_name() != feedback_subsystem_) continue;
      if (physical) { error = "ambiguous physical gripper feedback"; return false; }
      physical = &subsystem;
    }
    if (!physical || physical->state() != 1 || physical->data().size() != sizeof(double)) {
      error = "physical gripper subsystem feedback is missing or incompatible";
      return false;
    }
    std::memcpy(&physical_position_, physical->data().data(), sizeof(physical_position_));
    if (!std::isfinite(physical_position_) || physical_position_ < 0 || physical_position_ > 0.070) {
      error = "physical gripper opening is invalid";
      return false;
    }
  }
  return true;
}

bool MoveAbsJJointTarget::stationary(const overall_system_rtstate::SystemRtState& rt,
                                    bool at_target) const {
  for (const auto& target : targets_) {
    const auto& model = rt.model(target.model);
    // Empty/missing function names do not establish that the old ACT plan ended.
    if (model.current_func_name() != "none" && model.current_func_name() != "NotRunExecute")
      return false;
    for (int i = 0; i < model.joint_size(); ++i) {
      const auto& joint = model.joint(i);
      const auto position = feedback_subsystem_.empty() ? joint.position() : physical_position_;
      if (std::abs(joint.velocity()) > velocity_tolerance_ ||
          (at_target && std::abs(position - target.joints[i]) > position_tolerance_))
        return false;
    }
  }
  return true;
}

BT::NodeStatus MoveAbsJJointTarget::fail(const std::string& error) {
  request_.cancel();
  stable_since_ = target_since_ = {};
  setOutput("phase", std::string("failed"));
  setOutput("error", error);
  LOG_ERROR("[MoveAbsJJointTarget] {}: target={} speed={}", error, jointtarget_, speed_);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveAbsJJointTarget::onStart() {
  auto* manager = G_BT_MANAGER();
  request_.cancel();
  targets_.clear();
  selected_.clear();
  jointtarget_values_.clear();
  stable_since_ = target_since_ = last_frame_at_ = reply_frame_at_ = {};
  last_nrt_frame_at_ = {};
  phase_ = Phase::WaitingIdle;
  setOutput("phase", std::string("waiting_idle"));
  setOutput("error", std::string{});
  std::string values_text;
  if (!getInput("robot_sel", selected_) || !getInput("jointtarget", jointtarget_) ||
      !getInput("speed", speed_) || !getInput("timeout_ms", timeout_ms_) ||
      !getInput("jointtarget_values", values_text) ||
      !getInput("skip_if_at_target", skip_if_at_target_) ||
      !getInput("feedback_subsystem", feedback_subsystem_) ||
      !getInput("feedback_timeout_ms", feedback_timeout_ms_) ||
      !getInput("idle_stable_ms", idle_stable_ms_) || !getInput("stable_ms", stable_ms_) ||
      !getInput("position_tolerance_rad", position_tolerance_) ||
      !getInput("velocity_tolerance_rad_s", velocity_tolerance_) ||
      selected_.empty() || !identifier(jointtarget_) || !identifier(speed_) ||
      timeout_ms_ <= 0 || feedback_timeout_ms_ <= 0 || idle_stable_ms_ <= 0 || stable_ms_ <= 0 ||
      timeout_ms_ <= idle_stable_ms_ || timeout_ms_ <= stable_ms_ ||
      !std::isfinite(position_tolerance_) || position_tolerance_ <= 0 ||
      !std::isfinite(velocity_tolerance_) || velocity_tolerance_ <= 0 ||
      std::set<int>(selected_.begin(), selected_.end()).size() != selected_.size())
    return fail("invalid move inputs");
  if (!values_text.empty()) {
    try { jointtarget_values_ = BT::convertFromString<std::vector<double>>(values_text); }
    catch (const std::exception&) { return fail("invalid explicit joint target values"); }
  }
  if (jointtarget_values_.size() > 10 ||
      !std::all_of(jointtarget_values_.begin(), jointtarget_values_.end(),
                   [](double value) { return std::isfinite(value); }))
    return fail("explicit joint target exceeds ten slots or contains a non-finite value");
  if (!manager || manager->is_stopping() || !manager->is_conn_codeit())
    return fail("driver unavailable or manager stopping");
  model_count_ = manager->get_model_size();
  const auto rt = manager->get_rt_snapshot();
  const auto nrt = manager->get_nrt_snapshot();
  rt_regressions_ = rt.regression_count;
  nrt_regressions_ = nrt.regression_count;
  std::string error;
  if (!resolveTargets(rt, nrt, targets_, error) || !readFeedback(rt, nrt, error)) return fail(error);
  if (!feedback_subsystem_.empty()) {
    const auto suffix = "#" + std::to_string(selected_.front());
    if (selected_.size() != 1 || targets_.front().joints.size() != 1 ||
        feedback_subsystem_.rfind("TwoFingersGripperSerialYSEG2#", 0) != 0 ||
        feedback_subsystem_.size() <= suffix.size() ||
        feedback_subsystem_.compare(feedback_subsystem_.size() - suffix.size(), suffix.size(), suffix) != 0)
      return fail("physical feedback requires a YSEG2 subsystem mounted on the selected single-axis model");
  }
  deadline_ = Clock::now() + std::chrono::milliseconds(timeout_ms_);
  return onRunning();
}

BT::NodeStatus MoveAbsJJointTarget::onRunning() {
  auto* manager = G_BT_MANAGER();
  if (!manager || manager->is_stopping() || !manager->is_conn_codeit())
    return fail("driver disconnected or manager stopping; submitted motion is not canceled");
  if (Clock::now() >= deadline_)
    return fail("move timed out while " + std::string(phase_ == Phase::WaitingIdle ? "waiting for idle" :
        phase_ == Phase::WaitingReply ? "waiting for command reply; execution unconfirmed" :
                                     "waiting for stable target arrival"));
  const auto rt = manager->get_rt_snapshot();
  const auto nrt = manager->get_nrt_snapshot();
  std::string error;
  if (!readFeedback(rt, nrt, error)) return fail(error);
  const auto status = manager->get_gateway_status();
  if (status == 2) return fail("gateway reports an error");

  if (phase_ == Phase::WaitingReply) {
    const auto detail = request_.message();
    const auto result = request_.tick("MoveAbsJJointTarget");
    if (result == BT::NodeStatus::FAILURE)
      return fail("move request failed: " + (detail.empty() ? "rejected or timed out" : detail));
    if (result == BT::NodeStatus::RUNNING) return result;
    phase_ = Phase::Settling;
    reply_frame_at_ = Clock::now();
    stable_since_ = last_frame_at_ = {};
    last_nrt_frame_at_ = {};
    setOutput("phase", std::string("waiting_arrival"));
    return BT::NodeStatus::RUNNING;
  }

  const bool at_target = phase_ == Phase::Settling;
  if (status != 0 || !stationary(rt.data, at_target)) {
    stable_since_ = target_since_ = {};
    last_frame_at_ = rt.advanced_at;
    last_nrt_frame_at_ = nrt.advanced_at;
    return BT::NodeStatus::RUNNING;
  }
  // Cached pre-ack frames and duplicate frames cannot authorize completion.
  if (rt.advanced_at == last_frame_at_ || (at_target && rt.advanced_at <= reply_frame_at_))
    return BT::NodeStatus::RUNNING;
  if (!feedback_subsystem_.empty() &&
      (nrt.advanced_at == last_nrt_frame_at_ || (at_target && nrt.advanced_at <= reply_frame_at_)))
    return BT::NodeStatus::RUNNING;
  last_frame_at_ = rt.advanced_at;
  last_nrt_frame_at_ = nrt.advanced_at;
  const auto observed_at = feedback_subsystem_.empty() ? rt.advanced_at :
      std::min(rt.advanced_at, nrt.advanced_at);
  if (stable_since_ == Clock::time_point{}) stable_since_ = observed_at;
  if (!at_target && skip_if_at_target_) {
    if (stationary(rt.data, true)) {
      if (target_since_ == Clock::time_point{}) target_since_ = observed_at;
    } else target_since_ = {};
  }
  const auto hold = std::chrono::milliseconds(at_target ? stable_ms_ : idle_stable_ms_);
  if (observed_at - stable_since_ < hold) return BT::NodeStatus::RUNNING;
  if (at_target) {
    setOutput("phase", std::string("done"));
    LOG_INFO("[MoveAbsJJointTarget] move complete: target={} speed={}", jointtarget_, speed_);
    return BT::NodeStatus::SUCCESS;
  }

  if (skip_if_at_target_ && stationary(rt.data, true)) {
    // Use a separate target hold: an old idle hold away from home cannot skip motion.
    if (observed_at - target_since_ < std::chrono::milliseconds(stable_ms_))
      return BT::NodeStatus::RUNNING;
    setOutput("phase", std::string("already_at_target"));
    LOG_INFO("[MoveAbsJJointTarget] already at target; skipped motion: {}", jointtarget_);
    return BT::NodeStatus::SUCCESS;
  }
  target_since_ = {};

  bt_manager_cmd::CmdDynBuilder builder;
  builder.init_models(static_cast<std::size_t>(model_count_)).select_models(selected_)
      .set_event(jointtarget_values_.empty() ? "MoveAbsJJointTarget" : "MoveAbsJVar");
  if (jointtarget_values_.empty()) builder.add_field({{"jointtarget", jointtarget_}});
  else {
    std::ostringstream values;
    values.imbue(std::locale::classic());
    values << '{' << std::setprecision(17);
    // Codeit jointtarget_value uses ten slots, including for a one-axis gripper.
    // Match the proven ACT gripper command format, padding unused slots only.
    for (std::size_t i = 0; i < 10; ++i) {
      if (i) values << ',';
      values << (i < jointtarget_values_.size() ? jointtarget_values_[i] : 0.0);
    }
    values << '}';
    builder.add_field({{"jointtarget_value", values.str()}});
  }
  const auto command = builder.add_field({{"speed", speed_}}).build();
  phase_ = Phase::WaitingReply;
  stable_since_ = {};
  setOutput("phase", std::string("moving"));
  const auto remaining = std::max(std::chrono::milliseconds(1),
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - Clock::now()));
  if (!request_.start(manager->get_reqrep(), "main.handler.req.dyncmd", command, remaining,
                      manager->tickNotifier())) return fail("move request could not be created");
  LOG_INFO("[MoveAbsJJointTarget] move submitted once: target={} speed={}", jointtarget_, speed_);
  return BT::NodeStatus::RUNNING;
}

void MoveAbsJJointTarget::onHalted() {
  request_.cancel();
  stable_since_ = target_since_ = {};
  setOutput("phase", std::string("halted"));
  // Local cancellation never claims to revoke an already submitted MoveAbsJ.
}
