// All requests use the in-memory Bus below. No sockets, Codeit process,
// controller, IO, or production gateway is accessed by this test executable.
#include "bt_action/robot_actions.hpp"
#include "bt_manager.hpp"
#include "bt_json_request.hpp"
#include "auto_registration.hpp"
#include <behaviortree_cpp/xml_parsing.h>
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
namespace {
void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
Json success() {
  return {{"request_code", 0}, {"request_message", "ok"}, {"request_source", "gateway"}, {"total", 1}, {"failed", 0},
          {"results", Json::array({{{"index", 0}, {"command_code", 0}, {"command_message", "ok"},
              {"payload", Json::array({{{"return_code", 0}, {"subcmd_index", 0}, {"return_message", "completed"}}})}}})}};
}
class Request final : public IRequest {
public:
  bool wait(uint32_t) override { return done(); }
  const HttpResponse* response() const noexcept override { return done() ? &response_ : nullptr; }
  const void* response_data() const noexcept override { return response_.data(); }
  size_t response_size() const noexcept override { return response_.size(); }
  ReqStatus result() const noexcept override { return status_; }
  void on_complete(Completion cb, std::shared_ptr<void> ctx) override {
    callback_ = cb; context_ = std::move(ctx); if (done() && callback_) callback_(this, context_.get());
  }
  bool cancel() override { canceled = true; status_ = ReqStatus::Canceled; return true; }
  bool done() const noexcept override { return status_ != ReqStatus::Pending; }
  void complete(Json body = success()) {
    const auto text = body.dump(); response_.body.assign(text.begin(), text.end()); status_ = ReqStatus::Ok;
    if (callback_) callback_(this, context_.get());
  }
  void expire() {
    status_ = ReqStatus::Timeout;
    if (callback_) callback_(this, context_.get());
  }
  bool canceled{};
private:
  std::atomic<ReqStatus> status_{ReqStatus::Pending}; HttpResponse response_;
  Completion callback_ = nullptr; std::shared_ptr<void> context_;
};
struct Bus final : IReqRep {
  std::shared_ptr<IRequest> request(const char* service, const void* data, size_t size) override {
    const std::string name = service;
    ++calls;
    if (name != "main.handler.req.rawcmd") ++forbidden_commands;
    require(name == "main.handler.req.rawcmd", "robot nodes must never send dynamic controller commands");
    auto payload = Json::parse(std::string(static_cast<const char*>(data), size));
    require(payload.at("raw_commands").size() == 1, "one frame per request");
    require(payload["raw_commands"][0]["async"] == false, "must wait for synchronous Codeit result");
    const auto command = payload["raw_commands"][0]["command"].get<std::string>();
    if (command.find("{Stop") != std::string::npos || command.find("||Stop") != std::string::npos) ++forbidden_commands;
    require(command.find("{Stop") == std::string::npos && command.find("||Stop") == std::string::npos,
            "robot nodes must never dispatch Stop or StopHrcsiRobot");
    payloads.push_back(payload); services.push_back(name); last = std::make_shared<Request>();
    return last;
  }
  bool register_handler(const char*, PluginLifetimeHandle, const std::shared_ptr<IReqHandler>&) override { return false; }
  void unregister_handler(const char*, const std::shared_ptr<IReqHandler>&) override {}
  void unregister_owner(PluginLifetimeHandle) override {}
  std::string command() const { return payloads.back().at("raw_commands")[0].at("command"); }
  std::vector<Json> payloads; std::vector<std::string> services; std::shared_ptr<Request> last;
  std::atomic<size_t> calls{}, forbidden_commands{};
};
struct Feedback {
  overall_system_rtstate::SystemRtState rt; overall_system_nrtstate::SystemNrtState nrt;
  static std::int64_t frame;
  explicit Feedback(int count = 3) {
    rt.set_system_running_state(true);
    for (int m = 0; m < count; ++m) {
      auto* model = rt.add_model(); model->set_model_name("model" + std::to_string(m)); model->set_current_func_name("none");
      nrt.add_model()->set_model_name(model->model_name());
      for (int i = 0; i < (m == 2 ? 1 : 6); ++i) model->add_joint()->set_is_enabled(false);
      for (double v : {0., 0., 0., 0., 0., 0., 1.}) {
        model->mutable_current_point()->mutable_wobj()->add_data(v);
        model->mutable_current_point()->add_robottarget(v);
      }
    }
    G_BT_MANAGER()->set_conn_codeit(true); G_BT_MANAGER()->set_gateway_status(0); publish();
  }
  void publish() {
    ++frame; rt.mutable_head()->set_frame_id(frame); rt.mutable_head()->set_timestamp(frame * 2);
    nrt.mutable_head()->set_frame_id(frame); nrt.mutable_head()->set_timestamp(frame * 2);
    auto bytes = rt.SerializeAsString(); require(G_BT_MANAGER()->update_rt(bytes.data(), bytes.size()), "RT publish");
    bytes = nrt.SerializeAsString(); require(G_BT_MANAGER()->update_nrt(bytes.data(), bytes.size()), "NRT publish");
  }
};
std::int64_t Feedback::frame{};
BT::Tree tree(BT::BehaviorTreeFactory& factory, const std::string& children, const std::string& attributes = "") {
  return factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><RobotTask error=\"{task_error}\" " + attributes +
      ">" + children + "</RobotTask></BehaviorTree></root>");
}
std::vector<std::string> run(BT::Tree& task, Bus& bus) {
  std::vector<std::string> frames; auto status = BT::NodeStatus::IDLE; auto seen = bus.payloads.size();
  for (int tick = 0; tick < 10000 && status != BT::NodeStatus::SUCCESS; ++tick) {
    status = task.tickExactlyOnce(); require(status != BT::NodeStatus::FAILURE, "tree unexpectedly failed");
    if (bus.payloads.size() > seen) {
      require(bus.payloads.size() == seen + 1 && bus.services.back() == "main.handler.req.rawcmd", "unexpected or duplicate request");
      frames.push_back(bus.command()); bus.last->complete(); ++seen;
    }
  }
  require(status == BT::NodeStatus::SUCCESS, "tree did not finish"); return frames;
}
void expectRejected(BT::BehaviorTreeFactory& factory, Bus& bus, const Json& response, const std::string& reason) {
  auto task = tree(factory, R"(<Fallback><RobotMoveJ name="failed_move" target="MISSING"/><HrcsiSetDO do_num="4" value="1"/></Fallback>)", "hrcsi_stop_model=\"0\"");
  const auto before = bus.payloads.size(); require(task.tickExactlyOnce() == BT::NodeStatus::RUNNING, "rejection setup");
  bus.last->complete(response); require(task.tickExactlyOnce() == BT::NodeStatus::FAILURE, reason);
  require(bus.payloads.size() == before + 1, "rejection emitted a stop or ran fallback IO");
  require(!task.rootBlackboard()->get<std::string>("task_error").empty(), "failure must be visible");
}
void cases(Bus& bus, BT::BehaviorTreeFactory& factory) {
  Feedback feedback;
  // Disabled axes, empty work-object names, no NRT point library and no JSON
  // files must not prevent the graphical program's first DO from being sent.
  auto io = tree(factory, R"(<HrcsiSetDO model="0" do_num="1" value="0"/>)");
  require(run(io, bus) == std::vector<std::string>{"{SetHrcsiDout --do_num=1 --value=0||NotRunExecute||NotRunExecute}"}, "first DO was blocked");
  require(bus.payloads.back()["raw_commands"][0]["timeout"] == 100000000, "graphical timeout default");
  auto sequence = tree(factory, R"(<Sequence><RobotWait msec="30"/><RobotWait msec="{delay}"/><RobotWait msec="10"/></Sequence>)");
  sequence.rootBlackboard()->set("delay", 45); auto frames = run(sequence, bus);
  require(frames.size() == 3 && frames[0].find("count=30") != std::string::npos && frames[1].find("count=45") != std::string::npos, "reorder/add/blackboard edit");
  sequence.rootBlackboard()->set("delay", 55); require(run(sequence, bus)[1].find("count=55") != std::string::npos, "ports not re-read on next run");
  auto repeat = tree(factory, R"(<Repeat num_cycles="3"><RobotWait msec="7"/></Repeat>)"); require(run(repeat, bus).size() == 3, "Repeat reuse");
  auto named = tree(factory, R"(<RobotMoveJ name="named" model="1" target="new_point"/>)");
  auto before = bus.payloads.size(); require(named.tickExactlyOnce() == BT::NodeStatus::RUNNING, "named submit");
  require(bus.command() == "{NotRunExecute||MoveAbsJ --jointtarget=new_point --speed=v5||NotRunExecute}", "named target or model changed");
  auto pending = bus.last;
  for (int i = 0; i < 10; ++i) require(named.tickExactlyOnce() == BT::NodeStatus::RUNNING, "pending request advanced");
  require(bus.payloads.size() == before + 1, "pending frame resent");
  pending->complete(); require(named.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "completed Codeit result ignored");
  auto sync = tree(factory, R"(<RobotSync><RobotMoveJ model="0" target="JR3" speed="v40"/><RobotMoveL model="1" target="LR7-wobj" ratio="2" zone_position="0.5" zone_orientation="0.25" optimal="true"/></RobotSync>)");
  const auto mixed = run(sync, bus);
  require(mixed.size() == 3 && mixed[0] == "{NotRunExecute||MoveBlend --type=first_insert||NotRunExecute}" &&
      mixed[1] == "{NotRunExecute||MoveBlend --type=insert_line --robottarget=LR7-wobj --ratio=2 --speed=v5 --zone={0.5,0.25} --is_optimal=1||NotRunExecute}" &&
      mixed[2] == "{MoveAbsJ --jointtarget=JR3 --speed=v40||MoveBlend --type=start||NotRunExecute}", "mixed J/L synchronization changed");
  for (const auto& xml : {R"(<RobotSync><RobotWait/><RobotWait/></RobotSync>)",
      R"(<RobotSync><RobotWait model="0" msec="0" timeout_ms="5"/><RobotWait model="1" msec="10" timeout_ms="100"/></RobotSync>)",
      R"(<RobotSync><RobotWait/><RobotMoveJ model="1" target="bad||point"/></RobotSync>)",
      R"(<RobotMoveJ target="JR3" target_values="0,0,0,0,0,0"/>)", R"(<RobotMoveJ/>)",
      R"(<RobotMoveL target_values="0,0,0,0,0,0,0"/>)", R"(<RobotMoveJ target_values="0,0,0,0,0,0,"/>)",
      R"(<RobotMoveJ target="JR3" speed="v5 --speed=v100"/>)", R"(<HrcsiSetDO model="3" do_num="1" value="0"/>)",
      R"(<RobotSync><Sequence><RobotWait/></Sequence></RobotSync>)"}) {
    auto invalid = tree(factory, xml); before = bus.payloads.size();
    require(invalid.tickExactlyOnce() == BT::NodeStatus::FAILURE && bus.payloads.size() == before, "invalid batch sent a partial frame");
  }
  auto literal = tree(factory, R"(<RobotMoveJ target_values="1,2,3,4,5,6"/>)");
  require(run(literal, bus)[0] == "{MoveAbsJ --jointtarget_value={1,2,3,4,5,6,0,0,0,0} --speed=v5||NotRunExecute||NotRunExecute}", "explicit joint values");
  auto routing = tree(factory, R"(<HrcsiSetDO model="2" do_num="32" value="1"/>)");
  require(run(routing, bus)[0] == "{NotRunExecute||NotRunExecute||SetHrcsiDout --do_num=32 --value=1}", "IO still has fixed profile routing");
  auto timeout = tree(factory, R"(<Sequence><RobotWait msec="0"/><RobotWait msec="0" timeout_ms="70000"/></Sequence>)", "request_timeout_ms=\"80000\"");
  before = bus.payloads.size(); run(timeout, bus);
  require(bus.payloads[before]["raw_commands"][0]["timeout"] == 80000 && bus.payloads[before+1]["raw_commands"][0]["timeout"] == 70000, "timeout inheritance or override");
  Json bad = success(); bad["failed"] = 1; expectRejected(factory, bus, bad, "failed count ignored");
  bad = success(); bad["results"][0]["command_code"] = -17; expectRejected(factory, bus, bad, "command error ignored");
  bad = success(); bad["results"][0]["payload"][0]["return_code"] = -23; bad["results"][0]["payload"][0]["return_message"] = "point not found";
  expectRejected(factory, bus, bad, "subcommand error ignored");
  expectRejected(factory, bus, Json{{"request_code", 0}, {"request_message", "accepted"}}, "acceptance treated as completion");
  bad = success(); bad["results"] = Json::array(); expectRejected(factory, bus, bad, "missing results accepted");
  bad = success(); bad["request_code"] = -1; expectRejected(factory, bus, bad, "request rejection ignored");
  auto halted = tree(factory, R"(<Sequence><RobotWait msec="10"/><HrcsiSetDO do_num="4" value="1"/></Sequence>)", "hrcsi_stop_model=\"0\"");
  halted.tickExactlyOnce(); pending = bus.last; before = bus.payloads.size(); halted.haltTree();
  require(pending->canceled && bus.payloads.size() == before, "halting with a legacy stop route must only cancel the local request");
  halted.haltTree(); require(bus.payloads.size() == before, "repeated halt emitted a command");
  before = bus.payloads.size(); pending->complete(); require(bus.payloads.size() == before, "late completion dispatched next action");
  auto disconnected = tree(factory, R"(<RobotWait msec="0"/>)"); disconnected.tickExactlyOnce(); pending = bus.last;
  G_BT_MANAGER()->set_conn_codeit(false); before = bus.payloads.size();
  require(disconnected.tickExactlyOnce() == BT::NodeStatus::FAILURE && pending->canceled && bus.payloads.size() == before, "disconnect did not cancel");
  G_BT_MANAGER()->set_conn_codeit(true);
  auto timed = tree(factory, R"(<Sequence><RobotWait msec="0" timeout_ms="1"/><HrcsiSetDO do_num="4" value="1"/></Sequence>)", "hrcsi_stop_model=\"0\"");
  timed.tickExactlyOnce(); pending = bus.last; before = bus.payloads.size();
  std::this_thread::sleep_for(std::chrono::milliseconds(2010));
  require(timed.tickExactlyOnce() == BT::NodeStatus::FAILURE && pending->canceled && bus.payloads.size() == before,
          "local timeout must cancel without sending a stop or the next action");
  auto bus_timeout = tree(factory, R"(<Sequence><RobotMoveJ target="JR3"/><HrcsiSetDO do_num="4" value="1"/></Sequence>)", "hrcsi_stop_model=\"0\"");
  bus_timeout.tickExactlyOnce(); pending = bus.last; before = bus.payloads.size(); pending->expire();
  require(bus_timeout.tickExactlyOnce() == BT::NodeStatus::FAILURE && bus.payloads.size() == before,
          "service-bus timeout must not dispatch a stop or the next action");
  pending->complete(); require(bus.payloads.size() == before, "late bus reply emitted a command");
  auto subtree = factory.createTreeFromText(R"(<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main"><RobotTask><SubTree ID="Reusable" delay="17"/></RobotTask></BehaviorTree><BehaviorTree ID="Reusable"><RobotWait msec="{delay}"/></BehaviorTree></root>)");
  require(run(subtree, bus)[0].find("count=17") != std::string::npos, "SubTree context mapping");
  auto parallel = tree(factory, R"(<Parallel success_count="2" failure_count="1"><RobotWait model="0" msec="11"/><RobotWait model="1" msec="22"/></Parallel>)");
  require(run(parallel, bus).size() == 2, "Parallel request arbitration");
  feedback.publish();
  auto condition = tree(factory, R"(<Fallback><RobotAtTarget target_values="1,0,0,0,0,0"/><RobotMoveJ target="JR3"/></Fallback>)");
  require(run(condition, bus).size() == 1, "explicit false condition prevented fallback");
  feedback.rt.mutable_model(0)->mutable_joint(0)->set_position(1); feedback.publish();
  require(run(condition, bus).empty(), "explicit true condition did not skip action");
  auto stale = tree(factory, R"(<Fallback><RobotAtTarget target_values="1,0,0,0,0,0" feedback_timeout_ms="1"/><RobotMoveJ target="JR3"/></Fallback>)");
  std::this_thread::sleep_for(std::chrono::milliseconds(4)); before = bus.payloads.size();
  require(stale.tickExactlyOnce() == BT::NodeStatus::FAILURE && bus.payloads.size() == before, "invalid explicit condition authorized fallback");
  Feedback single(1); auto one = tree(factory, R"(<RobotMoveJ target="single_point"/>)");
  require(run(one, bus) == std::vector<std::string>{"{MoveAbsJ --jointtarget=single_point --speed=v5}"}, "model count hardcoded");
  // The extended response API must retain the old envelope-only tick contract
  // for unrelated nodes which do not request raw-command validation.
  BtJsonRequest legacy;
  require(legacy.start(&bus, "main.handler.req.rawcmd",
      {{"raw_commands", Json::array({{{"async", false}, {"timeout", 10}, {"command", "{NotRunExecute||NotRunExecute||NotRunExecute}"}}})}},
      std::chrono::milliseconds(10)), "legacy request creation");
  bus.last->complete({{"request_code", 0}, {"request_message", "ok"}});
  require(legacy.tick("legacy") == BT::NodeStatus::SUCCESS, "legacy request API changed");
  std::cout << "PASS: no Stop/StopHrcsiRobot on error, local or bus timeout, disconnect, halt and late replies; graphical frames, reusable trees and legacy request API preserved\n";
}

void managerStopCase(Bus& bus) {
  Feedback feedback;
  const auto path = std::filesystem::absolute("../bt_trees/no-stop-manager.xml");
  std::ofstream(path) << R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><RobotTask hrcsi_stop_model="0"><Sequence><RobotWait msec="1000"/><HrcsiSetDO do_num="4" value="1"/></Sequence></RobotTask></BehaviorTree></root>)";
  require(G_BT_MANAGER()->loadTree(path.string()).success, "manager load");
  const auto before = bus.calls.load();
  require(G_BT_MANAGER()->start(1).success, "manager start");
  for (int i = 0; i < 1000 && bus.calls.load() == before; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // stop() joins the worker before inspecting its request vector or pointer.
  // This is the same manager entry point used by the webpage stop endpoint.
  G_BT_MANAGER()->stop();
  require(bus.calls.load() == before + 1 && bus.last->canceled,
          "webpage/manager stop must only cancel its outstanding local request");
  bus.last->complete();
  require(bus.calls.load() == before + 1 && bus.forbidden_commands.load() == 0,
          "manager stop or late reply emitted a controller command");
  std::cout << "PASS: webpage stop manager path cancels locally without any controller stop request\n";
}
} // namespace

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  const auto fixtures = std::filesystem::absolute(argv[1]), output = std::filesystem::absolute(argv[2]);
  const auto original = std::filesystem::current_path();
  const auto temp = std::filesystem::temp_directory_path() / ("graphical-bt-test-" + std::to_string(Clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(temp / "bin");
  std::filesystem::create_directories(temp / "bt_trees");
  std::filesystem::current_path(temp / "bin");
  Bus bus; BTManager::Construct(nullptr, &bus, false); int result = 0;
  try {
    require(G_BT_MANAGER()->init(), "manager init"); BT::BehaviorTreeFactory factory; NodeRegistry::instance().registerAll(factory);
    cases(bus, factory);
    managerStopCase(bus);
    Feedback feedback;
    auto full = factory.createTreeFromFile((fixtures / "huashu-editable.xml").string());
    const auto frames = run(full, bus);
    Json expected; std::ifstream(fixtures / "reference_frames.json") >> expected;
    require(frames.size() == expected.size(), "frame count differs from graphical source");
    Json dump = Json::array();
    for (size_t i = 0; i < frames.size(); ++i) {
      require(frames[i] == expected[i].at("command").get<std::string>(), "graphical frame mismatch at " + std::to_string(i + 1) + ": " + frames[i]);
      dump.push_back({{"command", frames[i]}});
    }
    std::ofstream(output) << dump.dump(2) << '\n';
    std::filesystem::copy_file(temp / "bt_trees/bt_nodes.xml", output.parent_path() / "graphical-node-models.xml", std::filesystem::copy_options::overwrite_existing);
    std::cout << "PASS: all " << frames.size() << " frames exactly match original graphical commands with v5; no robot endpoints used\n";
    if (std::filesystem::exists(fixtures / "huashu-start-to-jr7-v5.xml")) {
      auto small = factory.createTreeFromFile((fixtures / "huashu-start-to-jr7-v5.xml").string());
      require(run(small, bus) == std::vector<std::string>(frames.begin(), frames.begin() + 29), "JR7 test tree differs from original prefix");
      std::cout << "PASS: small JR7 test tree matches the original first 29 frames\n";
    }
    if (std::filesystem::exists(fixtures / "editor-roundtrip.xml")) {
      auto saved = factory.createTreeFromFile((fixtures / "editor-roundtrip.xml").string());
      require(run(saved, bus) == frames, "actual editor roundtrip changed dispatch");
      auto edited = factory.createTreeFromFile((fixtures / "editor-edit-test.xml").string());
      auto changed = frames;
      std::swap(changed[0], changed[1]);
      const std::string original_move = "MoveAbsJ --jointtarget=JR3 --speed=v5";
      auto first_move = std::find_if(changed.begin(), changed.end(), [&](const auto& frame) { return frame.find(original_move) != std::string::npos; });
      require(first_move != changed.end(), "initial move fixture not found");
      first_move->replace(first_move->find(original_move), original_move.size(), "MoveAbsJ --jointtarget=OFFLINE_EDIT_POINT --speed=v10");
      require(run(edited, bus) == changed, "actual editor edits did not change the corresponding native frames");
      std::cout << "PASS: deployed editor save preserves frames; drag reorder and point/speed edits change native dispatch\n";
    }
    require(bus.forbidden_commands.load() == 0, "at least one forbidden stop/dynamic request was attempted");
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; result = 1; }
  BTManager::Destruct(); std::filesystem::current_path(original); std::filesystem::remove_all(temp); return result;
}
