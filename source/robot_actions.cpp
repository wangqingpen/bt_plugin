#include "bt_action/robot_actions.hpp"
#include "bt_manager.hpp"
#include "bt_json_request.hpp"
#include "auto_registration.hpp"
#include <behaviortree_cpp/behavior_tree.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>

namespace robot_bt {
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
namespace {
constexpr const char* contextKey = "_robot_task_context";

// One Codeit argument token, including UTF-8 names, but never another command.
bool argumentToken(const std::string& value) {
  if (value.empty() || value.size() > 256) return false;
  for (unsigned char ch : value)
    if (ch <= 32 || ch == 127 || std::string("{}[]|;=\\\"'`").find(ch) != std::string::npos) return false;
  return true;
}
std::string numbers(const std::vector<double>& values) {
  std::ostringstream out; out.imbue(std::locale::classic());
  out << '{' << std::setprecision(17);
  for (size_t i = 0; i < values.size(); ++i) { if (i) out << ','; out << values[i]; }
  out << '}'; return out.str();
}
std::string number(double value) { auto text = numbers({value}); return text.substr(1, text.size() - 2); }
std::string combine(const std::vector<std::string>& lanes) {
  std::string result = "{";
  for (size_t i = 0; i < lanes.size(); ++i) { if (i) result += "||"; result += lanes[i]; }
  return result + "}";
}
template<class T> T input(const BT::TreeNode& node, const char* key) {
  T value{}; auto result = node.getInput(key, value);
  if (!result) throw std::runtime_error(std::string("invalid port ") + key + ": " + result.error());
  return value;
}
void positive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0) throw std::runtime_error(std::string("must be finite and positive: ") + name);
}
template<class Snapshot> bool fresh(const Snapshot& snapshot, int timeout_ms) {
  const auto now = Clock::now(); const auto limit = std::chrono::milliseconds(timeout_ms);
  return snapshot.received_at.time_since_epoch().count() != 0 && snapshot.advanced_at.time_since_epoch().count() != 0 &&
         now - snapshot.received_at <= limit && now - snapshot.advanced_at <= limit;
}
void validateValues(std::vector<double>& values, bool cartesian, int dof) {
  if (!std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); })) throw std::runtime_error("non-finite target");
  if (cartesian) {
    if (values.size() != 7) throw std::runtime_error("Cartesian target must contain seven values");
    double norm = 0; for (int i = 3; i < 7; ++i) norm += values[i] * values[i];
    if (std::abs(norm - 1) > 0.01) throw std::runtime_error("Cartesian target quaternion is not unit length");
  } else {
    if (values.size() != 10 && (dof < 1 || dof > 10 || values.size() != static_cast<size_t>(dof)))
      throw std::runtime_error("joint target must contain live DOF or ten values");
    values.resize(10, 0.0);
  }
}
int liveDof(int model) {
  const auto rt = G_BT_MANAGER()->get_rt_snapshot();
  return model < rt.data.model_size() ? rt.data.model(model).joint_size() : 0;
}
void readTarget(const BT::TreeNode& node, Command& command) {
  command.target_name = input<std::string>(node, "target");
  const auto literal = input<std::string>(node, "target_values");
  if (command.target_name.empty() == literal.empty()) throw std::runtime_error("provide exactly one of target and target_values");
  if (!command.target_name.empty()) {
    if (!argumentToken(command.target_name)) throw std::runtime_error("invalid point name token");
    return; // The executing Codeit instance resolves its own point.
  }
  std::istringstream in(literal); in.imbue(std::locale::classic()); std::string field;
  while (std::getline(in, field, ',')) {
    std::istringstream scalar(field); scalar.imbue(std::locale::classic()); double value;
    if (!(scalar >> value)) throw std::runtime_error("invalid target_values CSV");
    scalar >> std::ws;
    if (!scalar.eof()) throw std::runtime_error("invalid target_values token");
    command.target_values.push_back(value);
  }
  if (literal.back() == ',') throw std::runtime_error("trailing comma in target_values");
  validateValues(command.target_values, command.kind == Kind::Line, liveDof(command.model));
}
std::string describe(const Command& command) {
  return "uid=" + std::to_string(command.node_uid) + " name=" + command.node_name + " model=" + std::to_string(command.model);
}
// ret.ok only checks the outer envelope. Inspect the completed command and its
// per-model driver results as well; an accepted request is not a completed frame.
std::string payloadError(const Json& payload) {
  if (payload.is_array()) {
    for (const auto& entry : payload) { auto error = payloadError(entry); if (!error.empty()) return error; }
  } else if (payload.is_object()) {
    if (payload.contains("return_code")) {
      const auto& code = payload.at("return_code");
      if (!code.is_number_integer()) return "invalid driver return_code: " + payload.dump();
      if (code.get<std::int64_t>() < 0) return "Codeit subcommand rejected: " + payload.dump();
    }
    for (auto it = payload.begin(); it != payload.end(); ++it) {
      if (it.value().is_object() || it.value().is_array()) {
        auto error = payloadError(it.value()); if (!error.empty()) return error;
      }
    }
  }
  return {};
}
std::string rawResultError(const ReqJsonResult& response) {
  if (!response.ok) return "request failed: " + response.message + " response=" + response.body.dump();
  const auto& body = response.body;
  if (!body.is_object() || !body.contains("total") || !body["total"].is_number_integer() ||
      body["total"].get<std::int64_t>() != 1 || !body.contains("failed") || !body["failed"].is_number_integer() ||
      !body.contains("results") || !body["results"].is_array() || body["results"].size() != 1)
    return "missing completed raw-command result: " + body.dump();
  if (body["failed"].get<std::int64_t>() != 0) return "Codeit frame failed: " + body.dump();
  const auto& result = body["results"][0];
  if (!result.is_object() || !result.contains("command_code") || !result["command_code"].is_number_integer())
    return "invalid command result: " + body.dump();
  if (result["command_code"].get<std::int64_t>() < 0) return "Codeit command rejected: " + result.dump();
  return result.contains("payload") ? payloadError(result["payload"]) : std::string{};
}
} // namespace

struct Context {
  int model_count{}, request_timeout_ms{100000000};
  bool aborted{};
  std::string error;
  const void* owner{};
  std::uint64_t frame_sequence{};
};

namespace {
std::shared_ptr<Context> context(const BT::TreeNode& node) {
  if (!node.config().blackboard) throw std::runtime_error("RobotTask blackboard missing");
  auto result = node.config().blackboard->get<std::shared_ptr<Context>>(contextKey);
  if (!result) throw std::runtime_error("place robot nodes under one RobotTask");
  return result;
}
bool connected(const Context& ctx, std::string& error) {
  auto* manager = G_BT_MANAGER();
  if (ctx.aborted) { error = ctx.error; return false; }
  if (!manager || manager->is_stopping() || !manager->is_conn_codeit()) {
    error = "Codeit disconnected or plugin stopping"; return false;
  }
  if (manager->get_model_size() != ctx.model_count) { error = "model topology changed during task"; return false; }
  return true;
}
void fault(const std::shared_ptr<Context>& ctx, const std::string& error) {
  // End only this tree's scheduling. A local failure/halt never emits a
  // controller command; an already-submitted Codeit action may keep executing.
  if (ctx) { ctx->aborted = true; ctx->error = error; }
  LOG_ERROR("[RobotTask] {}", error);
}
std::shared_ptr<Context> makeContext(int timeout_ms) {
  auto* manager = G_BT_MANAGER();
  if (!manager || !manager->is_conn_codeit()) throw std::runtime_error("Codeit disconnected");
  if (timeout_ms <= 0) throw std::runtime_error("request_timeout_ms must be positive");
  auto ctx = std::make_shared<Context>();
  ctx->model_count = manager->get_model_size(); ctx->request_timeout_ms = timeout_ms;
  if (ctx->model_count < 1) throw std::runtime_error("live model count unavailable");
  std::string error; if (!connected(*ctx, error)) throw std::runtime_error(error);
  return ctx;
}
std::vector<double> conditionTarget(const Command& command, int feedback_ms) {
  auto values = command.target_values;
  if (!command.target_name.empty()) {
    const auto nrt = G_BT_MANAGER()->get_nrt_snapshot();
    if (!fresh(nrt, feedback_ms) || command.model >= nrt.data.model_size()) throw std::runtime_error("RobotAtTarget requires fresh NRT taught points");
    int matches = 0;
    for (const auto& point : nrt.data.model(command.model).teach_points()) {
      if (point.point_name() != command.target_name) continue;
      ++matches;
      const auto& data = command.kind == Kind::Line ? point.robottarget() : point.jointtarget();
      values.assign(data.begin(), data.end());
    }
    if (matches != 1) throw std::runtime_error("RobotAtTarget needs one live point: " + command.target_name);
  }
  validateValues(values, command.kind == Kind::Line, liveDof(command.model));
  return values;
}
} // namespace

std::vector<Frame> compileBatch(const std::vector<Command>& commands, int count) {
  if (count < 1 || commands.empty()) throw std::runtime_error("empty command batch or model topology");
  std::vector<std::string> first(count, "NotRunExecute"), insert = first, run = first;
  std::set<int> used; bool blend = false;
  for (const auto& command : commands) {
    const int model = command.model;
    if (model < 0 || model >= count || !used.insert(model).second) throw std::runtime_error("RobotSync requires one direct action per distinct model");
    if (command.kind == Kind::Joint || command.kind == Kind::Line) {
      if (!argumentToken(command.speed)) throw std::runtime_error("invalid speed token");
      const bool named = !command.target_name.empty();
      if (named == !command.target_values.empty() || (named && !argumentToken(command.target_name))) throw std::runtime_error("invalid or ambiguous target");
      const auto target = named ? command.target_name : numbers(command.target_values);
      if (command.kind == Kind::Joint) run[model] = "MoveAbsJ --jointtarget" + std::string(named ? "=" : "_value=") + target + " --speed=" + command.speed;
      else {
        blend = true; first[model] = "MoveBlend --type=first_insert";
        insert[model] = "MoveBlend --type=insert_line --robottarget" + std::string(named ? "=" : "_value=") + target +
            " --ratio=" + number(command.ratio) + " --speed=" + command.speed +
            " --zone=" + numbers({command.zone_position, command.zone_orientation}) + " --is_optimal=" + (command.optimal ? "1" : "0");
        run[model] = "MoveBlend --type=start";
      }
    } else if (command.kind == Kind::Dout) run[model] = "SetHrcsiDout --do_num=" + std::to_string(command.output) + " --value=" + std::to_string(command.value);
    else run[model] = "Sleep --count=" + std::to_string(command.msec);
  }
  std::vector<Frame> result;
  if (blend) { result.push_back({combine(first)}); result.push_back({combine(insert)}); }
  result.push_back({combine(run)}); return result;
}

class Runner {
public:
  Runner(std::shared_ptr<Context> ctx, std::vector<Command> commands)
      : ctx_(std::move(ctx)), frames_(compileBatch(commands, ctx_->model_count)) {
    timeout_ms_ = commands.front().timeout_ms;
    for (const auto& command : commands) {
      timeout_ms_ = std::min(timeout_ms_, command.timeout_ms);
      if (!nodes_.empty()) nodes_ += "; ";
      nodes_ += describe(command);
    }
    for (const auto& command : commands) {
      if (command.kind == Kind::Wait && command.msec >= timeout_ms_)
        throw std::runtime_error("RobotSync wait exceeds the batch's shortest timeout_ms");
    }
  }
  ~Runner() { request_.cancel(); release(); }
  BT::NodeStatus status() const { return status_; }
  const std::string& error() const { return error_; }
  BT::NodeStatus tick() {
    if (status_ != BT::NodeStatus::RUNNING) return status_;
    try {
      std::string error; if (!connected(*ctx_, error)) return fail(error);
      if (ctx_->owner && ctx_->owner != this) return status_;
      ctx_->owner = this;
      auto* manager = G_BT_MANAGER();
      if (!pending_) {
        const auto& command = frames_[index_].command;
        const auto payload = Json{{"raw_commands", Json::array({{{"async", false}, {"timeout", timeout_ms_}, {"command", command}}})}};
        sequence_ = ++ctx_->frame_sequence; sent_at_ = Clock::now();
        LOG_INFO("[RobotAction] submit frame={} part={}/{} nodes=[{}] timeout_ms={} command={}", sequence_, index_ + 1, frames_.size(), nodes_, timeout_ms_, command);
        if (!request_.start(manager->get_reqrep(), "main.handler.req.rawcmd", payload,
            std::chrono::milliseconds(timeout_ms_) + std::chrono::milliseconds(2000), manager->tickNotifier())) return fail("request creation failed");
        pending_ = true; return status_;
      }
      ReqJsonResult response;
      const auto state = request_.tick("RobotAction", &response);
      if (state == BT::NodeStatus::RUNNING) return status_;
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - sent_at_).count();
      LOG_INFO("[RobotAction] reply frame={} part={}/{} nodes=[{}] elapsed_ms={} response={}", sequence_, index_ + 1, frames_.size(), nodes_, elapsed, response.body.dump());
      if (state == BT::NodeStatus::FAILURE) return fail("request failed: " + response.message + " response=" + response.body.dump());
      const auto rejected = rawResultError(response); if (!rejected.empty()) return fail(rejected);
      ++index_; pending_ = false;
      if (index_ == frames_.size()) { release(); status_ = BT::NodeStatus::SUCCESS; }
      return status_;
    } catch (const std::exception& e) { return fail(e.what()); }
  }
  void halt() { if (status_ == BT::NodeStatus::RUNNING) fail("action halted"); }
private:
  void release() { if (ctx_ && ctx_->owner == this) ctx_->owner = nullptr; }
  BT::NodeStatus fail(const std::string& error) {
    request_.cancel(); error_ = "nodes=[" + nodes_ + "] part=" + std::to_string(index_ + 1) + ": " + error;
    fault(ctx_, error_); release(); status_ = BT::NodeStatus::FAILURE; return status_;
  }
  std::shared_ptr<Context> ctx_;
  std::vector<Frame> frames_;
  size_t index_{};
  bool pending_{};
  int timeout_ms_{};
  std::uint64_t sequence_{};
  Clock::time_point sent_at_{};
  std::string nodes_, error_;
  BT::NodeStatus status_{BT::NodeStatus::RUNNING};
  BtJsonRequest request_;
};
} // namespace robot_bt

using namespace robot_bt;
BT::PortsList RobotTask::providedPorts() {
  return {BT::InputPort<int>("request_timeout_ms", 100000000, "Per-frame Codeit timeout in ms; graphical programming default"),
          // Keep accepting existing saved XML without reintroducing its former
          // behavior. The compatibility value is deliberately never read.
          BT::InputPort<int>("hrcsi_stop_model", -1, "Deprecated compatibility port; ignored. No controller stop commands are sent"),
          BT::OutputPort<std::string>("error")};
}
BT::NodeStatus RobotTask::tick() {
  try {
    if (!context_) {
      setOutput("error", std::string{});
      context_ = makeContext(input<int>(*this, "request_timeout_ms"));
      BT::applyRecursiveVisitor(child_node_, [this](BT::TreeNode* node) {
        if (dynamic_cast<RobotTask*>(node)) throw std::runtime_error("nested RobotTask is not supported");
        static_cast<const BT::TreeNode&>(*node).config().blackboard->set(contextKey, context_);
      });
    }
    std::string error; if (!connected(*context_, error)) throw std::runtime_error(error);
    const auto result = child_node_->executeTick();
    if (context_->aborted) throw std::runtime_error(context_->error);
    if (result != BT::NodeStatus::RUNNING) { resetChild(); context_.reset(); }
    return result;
  } catch (const std::exception& e) {
    fault(context_, e.what()); setOutput("error", std::string(e.what())); resetChild(); context_.reset(); return BT::NodeStatus::FAILURE;
  }
}
void RobotTask::halt() {
  if (context_) fault(context_, "task halted");
  BT::DecoratorNode::halt(); context_.reset();
}
BT::PortsList RobotCommand::commonPorts() {
  return {BT::InputPort<int>("model", 0, "Codeit model index"),
          BT::InputPort<int>("timeout_ms", 0, "Per-frame timeout in ms; 0 inherits RobotTask request_timeout_ms"), BT::OutputPort<std::string>("error")};
}
BT::PortsList RobotCommand::motionPorts() {
  auto ports = commonPorts();
  BT::PortsList motion = {BT::InputPort<std::string>("target", std::string{}, "Codeit point name; resolved by Codeit when executing"),
      BT::InputPort<std::string>("target_values", std::string{}, "Explicit numerical CSV; cannot be combined with target"),
      BT::InputPort<std::string>("speed", "v5", "Codeit speed variable")};
  ports.insert(motion.begin(), motion.end()); return ports;
}
BT::PortsList RobotMoveJ::providedPorts() { return motionPorts(); }
BT::PortsList RobotMoveL::providedPorts() {
  auto ports = motionPorts();
  BT::PortsList line = {BT::InputPort<double>("ratio", 4, "Codeit blend ratio"), BT::InputPort<double>("zone_position", 0, "Position blend zone"),
      BT::InputPort<double>("zone_orientation", 0, "Orientation blend zone"), BT::InputPort<bool>("optimal", false, "Codeit optimization flag")};
  ports.insert(line.begin(), line.end()); return ports;
}
BT::PortsList HrcsiSetDO::providedPorts() {
  auto ports = commonPorts(); ports.insert(BT::InputPort<int>("do_num")); ports.insert(BT::InputPort<int>("value")); return ports;
}
BT::PortsList RobotWait::providedPorts() {
  auto ports = commonPorts(); ports.insert(BT::InputPort<int>("msec", 100, "Codeit Sleep duration in milliseconds")); return ports;
}
RobotCommand::RobotCommand(const std::string& name, const BT::NodeConfig& config, Kind kind)
    : BT::StatefulActionNode(name, config), kind_(kind) {}
RobotCommand::~RobotCommand() = default;
Command RobotCommand::prepare() {
  const auto ctx = context(*this); std::string error;
  if (!connected(*ctx, error)) throw std::runtime_error(error);
  Command command; command.kind = kind_; command.node_uid = UID(); command.node_name = name();
  command.model = input<int>(*this, "model"); command.timeout_ms = input<int>(*this, "timeout_ms");
  if (command.timeout_ms == 0) command.timeout_ms = ctx->request_timeout_ms;
  if (command.model < 0 || command.model >= ctx->model_count || command.timeout_ms <= 0) throw std::runtime_error(describe(command) + ": invalid model or timeout_ms");
  if (kind_ == Kind::Joint || kind_ == Kind::Line) {
    readTarget(*this, command); command.speed = input<std::string>(*this, "speed");
    if (!argumentToken(command.speed)) throw std::runtime_error("invalid speed token");
    if (kind_ == Kind::Line) {
      command.ratio = input<double>(*this, "ratio"); command.zone_position = input<double>(*this, "zone_position");
      command.zone_orientation = input<double>(*this, "zone_orientation"); command.optimal = input<bool>(*this, "optimal");
      positive(command.ratio, "ratio");
      if (!std::isfinite(command.zone_position) || command.zone_position < 0 || !std::isfinite(command.zone_orientation) || command.zone_orientation < 0)
        throw std::runtime_error("invalid blend zones");
    }
  } else if (kind_ == Kind::Dout) {
    command.output = input<int>(*this, "do_num"); command.value = input<int>(*this, "value");
    // Codeit validates its controller's actual routing and supported DO range.
    if (command.output < 0 || (command.value != 0 && command.value != 1)) throw std::runtime_error("invalid DO number or value");
  } else {
    command.msec = input<int>(*this, "msec");
    if (command.msec < 0 || command.msec >= command.timeout_ms) throw std::runtime_error("wait must be nonnegative and shorter than timeout_ms");
  }
  return command;
}
void RobotCommand::joinBatch(const std::shared_ptr<Runner>& runner) {
  runner_ = runner; batched_ = true; setOutput("error", std::string{});
}
BT::NodeStatus RobotCommand::onStart() {
  if (batched_ && runner_ && runner_->status() == BT::NodeStatus::RUNNING) return BT::NodeStatus::RUNNING;
  batched_ = false; runner_.reset(); setOutput("error", std::string{});
  try { runner_ = std::make_shared<Runner>(context(*this), std::vector<Command>{prepare()}); return onRunning(); }
  catch (const std::exception& e) {
    const auto error = "uid=" + std::to_string(UID()) + " name=" + name() + ": " + e.what();
    try { fault(context(*this), error); } catch (...) {}
    setOutput("error", error); return BT::NodeStatus::FAILURE;
  }
}
BT::NodeStatus RobotCommand::onRunning() {
  const auto result = batched_ ? runner_->status() : runner_->tick();
  if (result == BT::NodeStatus::FAILURE) setOutput("error", runner_->error()); return result;
}
void RobotCommand::onHalted() { if (runner_) runner_->halt(); runner_.reset(); batched_ = false; }
BT::PortsList RobotSync::providedPorts() { return {BT::OutputPort<std::string>("error")}; }
BT::NodeStatus RobotSync::tick() {
  try {
    if (!runner_) {
      setOutput("error", std::string{}); std::vector<Command> commands; std::vector<RobotCommand*> actions;
      for (auto* child : children_nodes_) {
        auto* action = dynamic_cast<RobotCommand*>(child);
        if (!action) throw std::runtime_error("RobotSync direct children must be RobotMoveJ/L, HrcsiSetDO or RobotWait");
        actions.push_back(action); commands.push_back(action->prepare());
      }
      runner_ = std::make_shared<Runner>(context(*this), std::move(commands));
      for (auto* action : actions) { action->joinBatch(runner_); action->executeTick(); }
    }
    const auto result = runner_->tick();
    for (auto* child : children_nodes_) child->executeTick();
    if (result != BT::NodeStatus::RUNNING) {
      if (result == BT::NodeStatus::FAILURE) setOutput("error", runner_->error());
      haltChildren(); runner_.reset();
    }
    return result;
  } catch (const std::exception& e) {
    const auto error = "RobotSync uid=" + std::to_string(UID()) + " name=" + name() + ": " + e.what();
    try { fault(context(*this), error); } catch (...) {}
    setOutput("error", error); haltChildren(); runner_.reset(); return BT::NodeStatus::FAILURE;
  }
}
void RobotSync::halt() { if (runner_) runner_->halt(); BT::ControlNode::halt(); runner_.reset(); }

BT::PortsList RobotAtTarget::providedPorts() {
  auto ports = RobotCommand::motionPorts(); ports.erase("speed"); ports.erase("timeout_ms");
  BT::PortsList condition = {
      BT::InputPort<std::string>("kind", "joint", "joint or cartesian"),
      BT::InputPort<int>("feedback_timeout_ms", 1500, "Maximum receive and advance age for this condition"),
      BT::InputPort<double>("joint_tolerance", 0.01, "Joint tolerance in rad"),
      BT::InputPort<double>("translation_tolerance", 0.001, "Position tolerance in m"),
      BT::InputPort<double>("orientation_tolerance", 0.01, "Orientation tolerance in rad"),
      BT::InputPort<double>("velocity_tolerance", 0.01, "Stationary joint velocity tolerance")};
  ports.insert(condition.begin(), condition.end()); return ports;
}
BT::NodeStatus RobotAtTarget::tick() {
  try {
    setOutput("error", std::string{}); const auto ctx = context(*this); std::string error;
    if (!connected(*ctx, error)) throw std::runtime_error(error);
    const auto kind = input<std::string>(*this, "kind");
    if (kind != "joint" && kind != "cartesian") throw std::runtime_error("kind must be joint or cartesian");
    Command command; command.kind = kind == "joint" ? Kind::Joint : Kind::Line; command.model = input<int>(*this, "model");
    const auto feedback_ms = input<int>(*this, "feedback_timeout_ms");
    if (command.model < 0 || command.model >= ctx->model_count || feedback_ms <= 0) throw std::runtime_error("invalid condition model or feedback timeout");
    readTarget(*this, command); const auto values = conditionTarget(command, feedback_ms);
    const double jt = input<double>(*this, "joint_tolerance"), pt = input<double>(*this, "translation_tolerance"),
                 ot = input<double>(*this, "orientation_tolerance"), vt = input<double>(*this, "velocity_tolerance");
    positive(jt, "joint_tolerance"); positive(pt, "translation_tolerance"); positive(ot, "orientation_tolerance"); positive(vt, "velocity_tolerance");
    const auto snapshot = G_BT_MANAGER()->get_rt_snapshot();
    if (!fresh(snapshot, feedback_ms) || command.model >= snapshot.data.model_size()) throw std::runtime_error("RobotAtTarget requires fresh RT feedback");
    const auto& model = snapshot.data.model(command.model);
    if (model.error_code() != 0 || model.joint_size() < 1 || model.joint_size() > 10) throw std::runtime_error("invalid or faulted condition model");
    bool arrived = model.current_func_name() == "none" || model.current_func_name() == "NotRunExecute";
    for (int i = 0; i < model.joint_size(); ++i) {
      const auto& joint = model.joint(i);
      if (!std::isfinite(joint.position()) || !std::isfinite(joint.velocity()) || joint.error_code() != 0) throw std::runtime_error("invalid joint feedback");
      arrived = arrived && std::abs(joint.velocity()) <= vt;
      if (command.kind == Kind::Joint) arrived = arrived && std::abs(joint.position() - values[i]) <= jt;
    }
    if (command.kind == Kind::Line) {
      const auto& pose = model.current_point().robottarget();
      if (pose.size() != 7) throw std::runtime_error("Cartesian feedback must contain seven values");
      double distance = 0, dot = 0, norm = 0, goal = 0;
      for (int i = 0; i < 7; ++i) {
        if (!std::isfinite(pose[i])) throw std::runtime_error("invalid Cartesian feedback");
        if (i < 3) distance += (pose[i] - values[i]) * (pose[i] - values[i]);
        else { dot += pose[i] * values[i]; norm += pose[i] * pose[i]; goal += values[i] * values[i]; }
      }
      if (norm < 0.99 || norm > 1.01) throw std::runtime_error("invalid Cartesian feedback quaternion");
      arrived = arrived && std::sqrt(distance) <= pt && 2 * std::acos(std::min(1.0, std::abs(dot) / std::sqrt(norm * goal))) <= ot;
    }
    return arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  } catch (const std::exception& e) {
    const auto error = "RobotAtTarget uid=" + std::to_string(UID()) + " name=" + name() + ": " + e.what();
    try { fault(context(*this), error); } catch (...) {}
    setOutput("error", error); return BT::NodeStatus::FAILURE;
  }
}

REGISTER_NODE(RobotTask, "RobotTask");
REGISTER_NODE(RobotMoveJ, "RobotMoveJ");
REGISTER_NODE(RobotMoveL, "RobotMoveL");
REGISTER_NODE(HrcsiSetDO, "HrcsiSetDO");
REGISTER_NODE(RobotWait, "RobotWait");
REGISTER_NODE(RobotSync, "RobotSync");
REGISTER_NODE(RobotAtTarget, "RobotAtTarget");
