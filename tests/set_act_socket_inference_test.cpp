#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "auto_registration.hpp"
#include "bt_manager.hpp"

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

class FakeRequest final : public IRequest {
public:
  bool wait(uint32_t) override { return done(); }
  const HttpResponse* response() const noexcept override { return done() ? &response_ : nullptr; }
  const void* response_data() const noexcept override { return response_.data(); }
  size_t response_size() const noexcept override { return response_.size(); }
  ReqStatus result() const noexcept override { return status_; }
  void on_complete(Completion callback, std::shared_ptr<void> context) override {
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback_ = callback;
      context_ = std::move(context);
    }
    callback_cv_.notify_all();
    if (complete_on_registration) complete(completion_code);
  }
  bool waitForCallback() {
    std::unique_lock<std::mutex> lock(callback_mutex_);
    return callback_cv_.wait_for(lock, std::chrono::seconds(2), [this] { return callback_ != nullptr; });
  }
  bool cancel() override {
    canceled = true;
    status_ = ReqStatus::Canceled;
    return true;
  }
  bool done() const noexcept override { return status_ != ReqStatus::Pending; }

  void complete(int code) {
    const auto text = nlohmann::json{{"request_code", code},
                                    {"request_message", code == 0 ? "ok" : "rejected"}}.dump();
    response_.body.assign(text.begin(), text.end());
    status_ = ReqStatus::Ok;
    if (callback_) callback_(this, context_.get());
  }

  std::atomic_bool canceled{false};
  bool complete_on_registration{false};
  int completion_code{0};

private:
  std::atomic<ReqStatus> status_{ReqStatus::Pending};
  HttpResponse response_;
  Completion callback_{nullptr};
  std::shared_ptr<void> context_;
  std::mutex callback_mutex_;
  std::condition_variable callback_cv_;
};

class FakeReqRep final : public IReqRep {
public:
  std::shared_ptr<IRequest> request(const char* service, const void* data, size_t size) override {
    std::lock_guard<std::mutex> lock(request_mutex_);
    require(std::string(service) == "main.handler.req.dyncmd", "wrong request service");
    ++calls;
    payload = nlohmann::json::parse(std::string(static_cast<const char*>(data), size));
    if (reject_creation) return {};
    last_request = std::make_shared<FakeRequest>();
    require(payload.value("event_name", "") != "ShowActSocketInferenceData",
            "health checks must read subsystem status without sending commands");
    last_request->complete_on_registration = complete_immediately;
    request_cv_.notify_all();
    return last_request;
  }
  std::shared_ptr<FakeRequest> waitForRequestAfter(int previous_calls,
      std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
    std::unique_lock<std::mutex> lock(request_mutex_);
    if (!request_cv_.wait_for(lock, timeout,
                              [this, previous_calls] { return calls > previous_calls; })) return {};
    return last_request;
  }
  bool register_handler(const char*, PluginLifetimeHandle,
                        const std::shared_ptr<IReqHandler>&) override { return false; }
  void unregister_handler(const char*, const std::shared_ptr<IReqHandler>&) override {}
  void unregister_owner(PluginLifetimeHandle) override {}

  std::atomic<int> calls{0};
  bool reject_creation{false};
  bool complete_immediately{false};
  nlohmann::json payload;
  std::shared_ptr<FakeRequest> last_request;

private:
  std::mutex request_mutex_;
  std::condition_variable request_cv_;
};

#include "move_absj_joint_target_cases.hpp"
#include "grasp_retry_cases.hpp"

void runSchedulerTests(FakeReqRep& reqrep) {
  using namespace std::chrono_literals;
  struct Probe {
    std::mutex mutex;
    std::condition_variable cv;
    int visits{0};
    void visit() {
      std::lock_guard<std::mutex> lock(mutex);
      ++visits;
      cv.notify_all();
    }
    bool wait(int expected, std::chrono::milliseconds timeout = 500ms) {
      std::unique_lock<std::mutex> lock(mutex);
      return cv.wait_for(lock, timeout, [this, expected] { return visits >= expected; });
    }
  };
  auto probe = std::make_shared<Probe>();
  NodeRegistry::instance().registerNode("SchedulerProbe", [probe](BT::BehaviorTreeFactory& factory) {
    factory.registerSimpleAction("SchedulerProbe", [probe](BT::TreeNode&) {
      probe->visit();
      return BT::NodeStatus::SUCCESS;
    });
  }, {}, "Action");

  auto* manager = G_BT_MANAGER();
  const auto original_directory = std::filesystem::current_path();
  const auto directory = std::filesystem::temp_directory_path() /
      ("bt-scheduler-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory / "work");
  std::filesystem::create_directories(directory / "bt_trees");
  struct Cleanup {
    BTManager* manager;
    std::filesystem::path original, temporary;
    ~Cleanup() {
      manager->stop();
      manager->clearMonitorCallbacks();
      std::filesystem::current_path(original);
      std::filesystem::remove_all(temporary);
    }
  } cleanup{manager, original_directory, directory};
  std::filesystem::current_path(directory / "work");
  require(manager->init(), "scheduler manager initialization failed");
  manager->set_model_size(2);
  manager->set_conn_codeit(true);
  const auto tree_path = directory / "sequence.xml";
  {
    std::ofstream xml(tree_path);
    xml << R"(<root BTCPP_format="4"><BehaviorTree ID="Main">
      <RunOnce then_skip="false"><Sequence>
        <SetActSocketInference send_enable="0"/><SchedulerProbe/>
      </Sequence></RunOnce></BehaviorTree></root>)";
  }
  auto start = [&] {
    const int previous_calls = reqrep.calls;
    require(manager->loadTree(tree_path.string()).success, "scheduler load failed");
    const auto result = manager->start(5000);
    require(result.success, result.error.c_str());
    auto request = reqrep.waitForRequestAfter(previous_calls);
    require(request && request->waitForCallback(), "scheduler did not submit its request");
    return request;
  };

  // A five-second fallback makes a missed completion notification observable.
  auto request = start();
  require(!probe->wait(1, 50ms), "next node ran before request completion");
  const auto completed_at = std::chrono::steady_clock::now();
  request->complete(0);
  require(probe->wait(1), "completion did not wake the manager before the fallback tick");
  std::cout << "Completion-to-next-node latency: "
            << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - completed_at).count()
            << " ms (fallback tick: 5000 ms)\n";
  manager->stop();

  // Completion inside onStart must not deadlock or be lost before wait_for.
  reqrep.complete_immediately = true;
  start();
  require(probe->wait(2), "synchronous completion notification was lost");
  manager->stop();
  reqrep.complete_immediately = false;

  request = start();
  manager->pause();
  require(manager->step() == BT::NodeStatus::RUNNING, "paused request did not remain pending");
  request->complete(0);
  require(!probe->wait(3, 100ms), "completion bypassed pause");
  manager->resume();
  require(probe->wait(3), "resume did not consume the completed request promptly");
  manager->stop();

  // A canceled old request must not finish the replacement tree's request.
  auto old_request = start();
  const int previous_calls = reqrep.calls;
  require(manager->loadTree(tree_path.string()).success, "replacement tree load failed");
  require(old_request->canceled, "load did not halt the previous tree");
  require(!reqrep.waitForRequestAfter(previous_calls, 100ms), "load executed a replacement without start");
  old_request->complete(0);
  require(!probe->wait(4, 100ms), "old completion started an unarmed replacement");
  require(manager->start(5000).success, "explicit replacement start failed");
  auto replacement = reqrep.waitForRequestAfter(previous_calls);
  require(replacement && replacement->waitForCallback(), "explicit start did not wake the manager");
  require(!probe->wait(4, 100ms), "old completion affected a replacement tree");
  replacement->complete(0);
  require(probe->wait(4), "replacement completion did not wake the manager");
  manager->stop();

  auto failure = std::make_shared<Probe>();
  manager->setMonitorCallbacks({}, [failure](std::uint16_t, std::uint8_t status, std::int64_t) {
    if (status == static_cast<std::uint8_t>(BT::NodeStatus::FAILURE)) failure->visit();
  });
  request = start();
  request->complete(-1);
  require(failure->wait(1), "failure did not wake the manager");
  require(!probe->wait(5, 50ms), "failure advanced to the next node");
  manager->stop();

  std::function<void()> late_notification;
  {
    BTManager temporary(nullptr, nullptr);
    late_notification = temporary.tickNotifier();
  }
  late_notification();
}

void runLifecycleTests(FakeReqRep& reqrep) {
  using namespace std::chrono_literals;
  auto* manager = G_BT_MANAGER();
  const auto original = std::filesystem::current_path();
  const auto temporary = std::filesystem::temp_directory_path() /
      ("bt-lifecycle-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(temporary / "work");
  std::filesystem::create_directories(temporary / "bt_trees");
  struct Cleanup {
    BTManager* manager;
    std::filesystem::path original, temporary;
    ~Cleanup() {
      manager->stop();
      manager->clearMonitorCallbacks();
      std::filesystem::current_path(original);
      std::filesystem::remove_all(temporary);
    }
  } cleanup{manager, original, temporary};
  std::filesystem::current_path(temporary / "work");
  require(manager->init(), "lifecycle manager initialization failed");
  manager->set_model_size(2);
  manager->set_conn_codeit(true);
  auto visits = std::make_shared<std::atomic<int>>(0);
  NodeRegistry::instance().registerNode("LifecycleProbe", [visits](BT::BehaviorTreeFactory& factory) {
    factory.registerSimpleAction("LifecycleProbe", [visits](BT::TreeNode&) {
      ++*visits;
      return BT::NodeStatus::SUCCESS;
    });
  }, {}, "Action");
  auto throw_mode = std::make_shared<std::atomic<int>>(0);
  NodeRegistry::instance().registerNode("LifecycleThrow", [throw_mode](BT::BehaviorTreeFactory& factory) {
    factory.registerSimpleAction("LifecycleThrow", [throw_mode](BT::TreeNode&) {
      if (*throw_mode == 1) throw std::runtime_error("simulated tick failure");
      if (*throw_mode == 2) throw 42;
      return BT::NodeStatus::SUCCESS;
    });
  }, {}, "Action");
  std::atomic<int> snapshots{0};
  manager->setMonitorCallbacks([&](std::string xml) {
    require(xml.find("LifecycleProbe") != std::string::npos, "loaded tree snapshot missing probe");
    ++snapshots;
  }, {});
  const auto path = temporary / "sequence.xml";
  const auto once_path = temporary / "once.xml";
  const auto invalid_path = temporary / "invalid.xml";
  const auto throwing_path = temporary / "throwing.xml";
  const std::string body = "<Sequence><SetActSocketInference send_enable=\"0\"/><LifecycleProbe/></Sequence>";
  {
    std::ofstream(path) << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">" << body << "</BehaviorTree></root>";
    std::ofstream(once_path) << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><RunOnce then_skip=\"false\">"
                             << body << "</RunOnce></BehaviorTree></root>";
    std::ofstream(invalid_path) << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><MissingNode/></BehaviorTree></root>";
    std::ofstream(throwing_path) << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
        "<Sequence><SetActSocketInference send_enable=\"0\"/><LifecycleProbe/><LifecycleThrow/>"
        "</Sequence></BehaviorTree></root>";
  }
  auto wait_idle = [&] {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (manager->is_running() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(1ms);
    require(!manager->is_running(), "terminal result did not disarm execution");
  };
  auto explicit_start = [&] {
    const int before = reqrep.calls;
    const auto result = manager->start(5);
    require(result.success, result.error.c_str());
    auto request = reqrep.waitForRequestAfter(before);
    require(request && request->waitForCallback(), "explicit start did not send its request");
    return request;
  };
  auto no_new_request = [&](const char* reason) {
    const int before = reqrep.calls;
    require(!reqrep.waitForRequestAfter(before, 100ms), reason);
  };

  require(!manager->start().success, "start accepted without a loaded file");
  const auto load = manager->loadTree(path.string());
  require(load.success && snapshots == 1, "load must synchronously publish an idle tree");
  require(!manager->is_running() && reqrep.calls == 0, "load started execution");
  manager->resume();
  require(manager->step() == BT::NodeStatus::IDLE, "step bypassed start permission");
  no_new_request("load/resume ticked an unarmed tree");

  auto request = explicit_start();
  const int active_calls = reqrep.calls;
  require(manager->start(5).success, "repeat active start was rejected");
  require(reqrep.calls == active_calls, "repeat start sent another request");
  request->complete(0);
  wait_idle();
  require(*visits == 1, "completed sequence did not reach the next node");
  no_new_request("completed plain sequence restarted automatically");
  manager->resume();
  manager->tickNotifier()();
  request->complete(0);
  require(manager->step() == BT::NodeStatus::IDLE, "step restarted a completed run");
  no_new_request("resume/late notification restarted a completed run");

  request = explicit_start();
  request->complete(0);
  wait_idle();
  require(*visits == 2, "explicit start failed to create another run");

  // Loading during an active request cancels it, publishes the new tree and
  // waits. Late completion and resume cannot authorize the replacement.
  auto old_request = explicit_start();
  const int before_load = reqrep.calls;
  require(manager->loadTree(path.string()).success, "active replacement load failed");
  require(old_request->canceled && !manager->is_running(), "load did not stop the previous run");
  old_request->complete(0);
  manager->resume();
  require(manager->step() == BT::NodeStatus::IDLE, "replacement step bypassed start");
  no_new_request("replacement started from a stale completion");
  require(reqrep.calls == before_load, "load sent a replacement command");
  request = explicit_start();
  request->complete(-1);
  wait_idle();
  no_new_request("failed sequence automatically retried");

  // An explicit start resets the RunOnce latch; loading it does not run it.
  require(manager->loadTree(once_path.string()).success, "RunOnce load failed");
  no_new_request("RunOnce executed during load");
  const int visits_before_once = *visits;
  for (int i = 0; i < 2; ++i) {
    request = explicit_start();
    request->complete(0);
    wait_idle();
  }
  require(*visits == visits_before_once + 2, "explicit restart did not reset RunOnce");

  request = explicit_start();
  const auto invalid = manager->loadTree(invalid_path.string());
  require(!invalid.success && !invalid.error.empty() && request->canceled,
          "invalid load must reject and halt the preceding run");
  require(!manager->is_running() && !manager->start().success, "invalid load retained an executable old tree");
  manager->resume();
  require(manager->step() == BT::NodeStatus::IDLE, "invalid load allowed a tick");
  no_new_request("invalid load or late notification executed a command");

  // Concurrent loads are serialized, and both leave execution disarmed.
  BTManager::OperationResult first, second;
  std::thread one([&] { first = manager->loadTree(path.string()); });
  std::thread two([&] { second = manager->loadTree(path.string()); });
  one.join();
  two.join();
  require(first.success && second.success && !manager->is_running(), "concurrent loads were not serialized");
  no_new_request("concurrent loads executed a command");
  request = explicit_start();
  manager->stop();
  require(request->canceled && !manager->is_running(), "explicit stop did not halt the run");
  manager->tickNotifier()();
  no_new_request("stopped tree restarted from a notification");
  request = explicit_start();
  request->complete(0);
  wait_idle();

  // Exceptions in both the worker and single-step path must end the run.
  require(manager->loadTree(throwing_path.string()).success, "throwing tree load failed");
  for (int mode : {1, 2}) {
    *throw_mode = mode;
    request = explicit_start();
    request->complete(0);
    wait_idle();
    manager->resume();
    manager->tickNotifier()();
    require(manager->step() == BT::NodeStatus::IDLE, "exception left execution armed");
    no_new_request("exception automatically retried the tree");
  }
  request = explicit_start();
  manager->pause();
  require(manager->step() == BT::NodeStatus::RUNNING, "throwing tree did not pause at its pending request");
  request->complete(0);
  require(manager->step() == BT::NodeStatus::FAILURE, "step did not contain a tick exception");
  require(!manager->is_running(), "step exception did not disarm execution");
  no_new_request("step exception automatically retried the tree");
  *throw_mode = 0;
  request = explicit_start();
  request->complete(0);
  wait_idle();
}

BT::Tree makeTree(BT::BehaviorTreeFactory& factory, const std::string& attributes = "") {
  return factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
      "<SetActSocketInference " + attributes + "/></BehaviorTree></root>");
}

BT::Tree makeOnceTree(BT::BehaviorTreeFactory& factory, const std::string& attributes = "") {
  return factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
      "<RunActSocketInferenceOnce " + attributes + "/></BehaviorTree></root>");
}

void runOnceTests(FakeReqRep& reqrep) {
  auto* manager = G_BT_MANAGER();
  manager->set_model_size(2);
  manager->set_conn_codeit(true);
  BT::BehaviorTreeFactory factory;
  NodeRegistry::instance().registerAll(factory);
  require(manager->dump_nodes_xml().find("ID=\"RunActSocketInferenceOnce\"") != std::string::npos,
          "one-shot node is missing from generated node models");

  auto defaults = makeOnceTree(factory);
  require(defaults.tickExactlyOnce() == BT::NodeStatus::RUNNING, "one-shot request did not start");
  require(reqrep.calls == 1, "one activation must send exactly one request");
  require(reqrep.payload == nlohmann::json{
      {"event_name", "RunActSocketInferenceOnce"}, {"user_action", "Click"},
      {"sub_commands", {{{"selected", true}, {"timeout_ms", 10000}},
                        {{"selected", false}}}}},
      "one-shot default command must select only one model without extra Set commands");
  for (int i = 0; i < 3; ++i) {
    require(defaults.tickExactlyOnce() == BT::NodeStatus::RUNNING, "pending one-shot must wait");
    require(reqrep.calls == 1, "pending tick repeated the one-shot command");
  }
  reqrep.last_request->complete(0);
  require(defaults.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "one-shot success was lost");

  // 实际示例树在管理器持续 tick 已完成根节点时也只能触发一次。
  auto example = factory.createTreeFromFile(
      (std::filesystem::path(__FILE__).parent_path() / "trees/run_act_socket_inference_once.xml").string());
  require(example.tickExactlyOnce() == BT::NodeStatus::RUNNING, "one-shot example did not start");
  const int example_calls = reqrep.calls;
  reqrep.last_request->complete(0);
  for (int i = 0; i < 5; ++i) {
    require(example.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "one-shot example lost its completed result");
    require(reqrep.calls == example_calls, "completed example tree repeated the one-shot command");
  }

  auto custom = makeOnceTree(factory,
      "robot_sel=\"1\" timeout_ms=\"30000\" subsystem_name=\"ActSocketInference#1#0\"");
  require(custom.tickExactlyOnce() == BT::NodeStatus::RUNNING, "custom one-shot did not start");
  require(reqrep.payload["sub_commands"][0] == nlohmann::json{{"selected", false}},
          "one-shot parameters leaked to an unselected model");
  require(reqrep.payload["sub_commands"][1] == nlohmann::json{
      {"selected", true}, {"timeout_ms", 30000}, {"subsystem_name", "ActSocketInference#1#0"}},
      "one-shot timeout or subsystem name was not forwarded");
  reqrep.last_request->complete(-2);
  require(custom.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot driver error was ignored");

  for (const char* attributes : {
           "timeout_ms=\"0\"", "timeout_ms=\"-1\"", "timeout_ms=\"{missing_timeout}\"",
           "subsystem_name=\"{missing_name}\"", "robot_sel=\"-1\"", "robot_sel=\"2\"",
           "robot_sel=\"0;1\"", "robot_sel=\"0;0\"", "robot_sel=\"\""}) {
    const int calls = reqrep.calls;
    auto tree = makeOnceTree(factory, attributes);
    require(tree.tickExactlyOnce() == BT::NodeStatus::FAILURE, "invalid one-shot input was accepted");
    require(reqrep.calls == calls, "invalid one-shot input sent a command");
  }

  manager->set_model_size(0);
  auto no_models = makeOnceTree(factory);
  const int before_no_models = reqrep.calls;
  require(no_models.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot accepted missing model state");
  require(reqrep.calls == before_no_models, "missing model state sent a one-shot command");
  manager->set_model_size(1);

  auto halted = makeOnceTree(factory);
  require(halted.tickExactlyOnce() == BT::NodeStatus::RUNNING, "single-model one-shot was rejected");
  require(reqrep.payload["sub_commands"].size() == 1, "one-shot model count was cached");
  auto halted_request = reqrep.last_request;
  halted.haltTree();
  require(halted_request->canceled, "one-shot halt did not cancel the request");
  require(halted.tickExactlyOnce() == BT::NodeStatus::RUNNING, "one-shot did not restart after halt");
  halted_request->complete(0);
  require(halted.tickExactlyOnce() == BT::NodeStatus::RUNNING, "late completion affected a new one-shot request");
  reqrep.last_request->complete(0);
  require(halted.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "restarted one-shot did not finish");

  auto disconnected = makeOnceTree(factory);
  require(disconnected.tickExactlyOnce() == BT::NodeStatus::RUNNING, "disconnect test did not start");
  auto disconnected_request = reqrep.last_request;
  manager->set_conn_codeit(false);
  require(disconnected.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot ignored disconnect");
  require(disconnected_request->canceled, "one-shot disconnect did not cancel the request");
  auto offline = makeOnceTree(factory);
  const int before_offline = reqrep.calls;
  require(offline.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot accepted an offline driver");
  require(reqrep.calls == before_offline, "one-shot sent a request to an offline driver");
  manager->set_conn_codeit(true);

  reqrep.reject_creation = true;
  auto rejected = makeOnceTree(factory);
  require(rejected.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot ignored request creation error");
  reqrep.reject_creation = false;

  auto timeout = makeOnceTree(factory, "timeout_ms=\"1\"");
  require(timeout.tickExactlyOnce() == BT::NodeStatus::RUNNING, "timeout test did not start");
  auto timeout_request = reqrep.last_request;
  const int before_timeout = reqrep.calls;
  std::this_thread::sleep_for(std::chrono::milliseconds(2100));
  require(timeout.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot ignored request timeout");
  require(timeout_request->canceled && reqrep.calls == before_timeout,
          "one-shot timeout must cancel without retrying");

  auto stopping = makeOnceTree(factory);
  require(stopping.tickExactlyOnce() == BT::NodeStatus::RUNNING, "shutdown test did not start");
  auto stopping_request = reqrep.last_request;
  manager->beginShutdown();
  require(stopping.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot ignored shutdown");
  require(stopping_request->canceled, "one-shot shutdown did not cancel the request");
  const int before_shutdown = reqrep.calls;
  auto stopped = makeOnceTree(factory);
  require(stopped.tickExactlyOnce() == BT::NodeStatus::FAILURE, "one-shot accepted shutdown state");
  require(reqrep.calls == before_shutdown, "one-shot shutdown sent a command");
}

void runPi05Tests(FakeReqRep& reqrep, const char* path) {
  auto* manager = G_BT_MANAGER();
  manager->set_conn_codeit(true);
  BT::BehaviorTreeFactory factory;
  NodeRegistry::instance().registerAll(factory);
  auto full_tree = factory.createTreeFromFile(path);
  require(full_tree.rootNode() != nullptr, "PI0.5 XML did not load");
  std::int64_t frame = 0;
  auto publish = [&](std::uint64_t generation, std::int32_t phase, bool active) {
    overall_system_rtstate::SystemRtState rt;
    rt.mutable_head()->set_frame_id(++frame);
    rt.mutable_head()->set_timestamp(frame * 2);
    rt.set_system_running_state(true);
    rt.add_model()->add_joint()->set_is_enabled(true);
    auto* joint = rt.add_model()->add_joint();
    joint->set_position(0.0625);
    joint->set_is_enabled(true);
    auto bytes = rt.SerializeAsString();
    require(manager->update_rt(bytes.data(), bytes.size()), "PI0.5 RT update failed");

    overall_system_nrtstate::SystemNrtState nrt;
    nrt.mutable_head()->set_frame_id(frame);
    nrt.mutable_head()->set_timestamp(frame * 50);
    auto* sub = nrt.add_subsystem();
    sub->set_subsystem_name("Pi05Inference#0#0");
    sub->set_state(1);
    std::array<char, 256> payload{};
    const auto field = [&](std::size_t offset, const auto& value) {
      std::memcpy(payload.data() + offset, &value, sizeof(value));
    };
    field(0, std::uint32_t(256));
    field(8, std::int32_t(1));
    field(16, std::int32_t(active));
    field(20, std::int32_t(active));
    field(24, phase);
    field(28, std::int32_t(phase == 5));
    field(32, generation);
    sub->set_data(payload.data(), payload.size());
    bytes = nrt.SerializeAsString();
    require(manager->update_nrt(bytes.data(), bytes.size()), "PI0.5 NRT update failed");
  };
  auto node = [&] {
    return factory.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
        "<RunPi05UntilGripperDone gripper_model=\"1\" result=\"{result}\" phase=\"{phase}\"/>"
        "</BehaviorTree></root>");
  };
  auto start = [&](BT::Tree& tree, const char* generation) {
    require(tree.tickExactlyOnce() == BT::NodeStatus::RUNNING, "PI0.5 did not start");
    require(reqrep.payload["event_name"] == "SetPi05Inference", "wrong PI0.5 start event");
    const auto& request = reqrep.payload["sub_commands"][0];
    require(request["expected_generation"] == generation && request["finish_on_gripper"] == 1 &&
            request["expected_gripper_model"] == 1 && request["subsystem_name"] == "Pi05Inference#0#0",
            "PI0.5 supervision was not scoped to the published generation");
    reqrep.last_request->complete(0);
    require(tree.tickExactlyOnce() == BT::NodeStatus::RUNNING, "PI0.5 start acknowledgement failed");
  };

  publish(3, 0, false);
  auto success = node();
  start(success, "3");
  publish(4, 2, true);
  require(success.tickExactlyOnce() == BT::NodeStatus::RUNNING, "even active PI0.5 generation rejected");
  publish(4, 6, true);
  require(success.tickExactlyOnce() == BT::NodeStatus::RUNNING, "PI0.5 release wait rejected");
  require(success.rootBlackboard()->get<std::string>("phase") == "waiting_open", "release phase missing");
  publish(5, 7, false);
  require(success.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "PI0.5 completion failed");

  auto empty = node();
  start(empty, "5");
  require(empty.tickExactlyOnce() == BT::NodeStatus::RUNNING, "old completion authorized a new attempt");
  publish(7, 8, false);
  require(empty.tickExactlyOnce() == BT::NodeStatus::FAILURE, "PI0.5 empty grasp accepted as success");
  require(empty.rootBlackboard()->get<std::string>("result") == "grasp_failed", "PI0.5 empty grasp is not recoverable");

  auto fault = node();
  start(fault, "7");
  publish(9, 5, false);
  require(fault.tickExactlyOnce() == BT::NodeStatus::FAILURE, "PI0.5 driver fault was ignored");
  require(fault.rootBlackboard()->get<std::string>("result") == "fault", "driver fault incorrectly allows retry");

  auto canceled = node();
  start(canceled, "9");
  publish(10, 4, true);
  require(canceled.tickExactlyOnce() == BT::NodeStatus::RUNNING, "PI0.5 active status missing");
  reqrep.complete_immediately = true;
  canceled.haltTree();
  reqrep.complete_immediately = false;
  require(reqrep.payload["event_name"] == "StopPi05GripperRun" &&
          reqrep.payload["sub_commands"][0]["expected_generation"] == "10", "PI0.5 halt was not scoped");
}

void runGripperTests(FakeReqRep& reqrep) {
  using namespace std::chrono_literals;
  constexpr std::int32_t waiting_close = 0x41430001, waiting_open = 0x41430002;
  constexpr std::int32_t completed = 0x41430003, timed_out = 0x41430004;
  auto* manager = G_BT_MANAGER();
  manager->set_conn_codeit(true);
  BT::BehaviorTreeFactory factory;
  NodeRegistry::instance().registerAll(factory);
  require(manager->dump_nodes_xml().find("ID=\"RunActUntilGripperDone\"") != std::string::npos,
          "gripper node missing from node models");
  std::int64_t frame = 1, nrt_frame = 1;
  std::uint64_t generation = 2, send_generation = 2;
  std::int32_t state = 0;
  bool running = false, sending = false, ready = true, duplicate = false, auto_nrt = true;
  int error = 0;
  std::size_t payload_size = 248;
  std::uint32_t reported_size = 248;
  auto publishAct = [&](bool advance = true) {
    overall_system_nrtstate::SystemNrtState nrt;
    if (advance) ++nrt_frame;
    nrt.mutable_head()->set_frame_id(nrt_frame);
    nrt.mutable_head()->set_timestamp(nrt_frame * 50);
    auto* subsystem = nrt.add_subsystem();
    subsystem->set_subsystem_name("ActSocketInference#2");
    subsystem->set_state(1);
    std::array<char, 248> bytes{};
    auto field = [&](std::size_t offset, const auto& value) {
      std::memcpy(bytes.data() + offset, &value, sizeof(value));
    };
    field(0, reported_size);
    field(8, std::int32_t(running));
    field(12, std::int32_t(ready));
    field(16, std::int32_t(1));
    field(20, std::int32_t(error));
    field(40, std::int32_t(sending));
    field(44, state);
    field(56, generation);
    field(64, send_generation);
    std::string data(bytes.data(), bytes.size());
    data.resize(payload_size);
    subsystem->set_data(data);
    if (duplicate) {
      auto* other = nrt.add_subsystem();
      *other = *subsystem;
      other->set_subsystem_name("ActSocketInference#3");
    }
    const auto message = nrt.SerializeAsString();
    require(manager->update_nrt(message.data(), message.size()), "NRT update failed");
  };
  auto publish = [&](double mm = 50, bool advance = true, bool enabled = true) {
    overall_system_rtstate::SystemRtState rt;
    if (advance) ++frame;
    rt.mutable_head()->set_frame_id(frame);
    rt.mutable_head()->set_timestamp(frame * 2);
    rt.set_system_running_state(true);
    rt.add_model()->add_joint()->set_is_enabled(true);
    auto* joint = rt.add_model()->add_joint();
    joint->set_position(mm / 1000.0);
    joint->set_is_enabled(enabled);
    const auto message = rt.SerializeAsString();
    require(manager->update_rt(message.data(), message.size()), "RT update failed");
    if (auto_nrt) publishAct();
  };
  auto tree = [&](const std::string& extra = "") {
    return factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
      "<RunActUntilGripperDone gripper_model=\"1\" closed_stable_ms=\"10\" stable_ms=\"15\" "
      "error=\"{act_test_error}\" phase=\"{act_test_phase}\" "
      "result=\"{act_test_result}\" " +
      extra + "/></BehaviorTree></root>");
  };
  auto arm = [&] {
    generation += (generation & 1U) ? 2 : 1;
    send_generation += (send_generation & 1U) ? 2 : 1;
    running = sending = true;
    state = waiting_close;
    publish();
  };
  auto start = [&](BT::Tree& t) {
    require(t.tickExactlyOnce() == BT::NodeStatus::RUNNING, "gripper task did not start");
    const auto& command = reqrep.payload["sub_commands"][0];
    require(command["update_enable"] == 1 && command["send_enable"] == 1 &&
            command["finish_on_gripper"] == 1 && command["expected_gripper_model"] == 1 &&
            command["closed_stable_ms"] == 10 && command["stable_ms"] == 15 &&
            command["closed_mm"] == 5 && command["finish_opening_mm"] == 50 &&
            command["tolerance_mm"] == 2 && command["feedback_timeout_ms"] == 1000 &&
            command["run_timeout_ms"] == 120000 && command["subsystem_name"] == "ActSocketInference#2",
            "supervised completion conditions were not forwarded");
    reqrep.last_request->complete(0);
    require(t.tickExactlyOnce() == BT::NodeStatus::RUNNING, "start acknowledgement lost");
    arm();
    const int calls = reqrep.calls;
    require(t.tickExactlyOnce() == BT::NodeStatus::RUNNING, "active status was rejected");
    require(reqrep.calls == calls, "status reading sent a command");
  };
  auto finish = [&] {
    running = sending = false;
    if (generation & 1U) ++generation;
    if (send_generation & 1U) ++send_generation;
    state = completed;
    publish();
  };
  auto stopFailure = [&](BT::Tree& t) {
    require(reqrep.payload["sub_commands"][0]["update_enable"] == 0 &&
            reqrep.payload["sub_commands"][0]["send_enable"] == 0, "failure cleanup changed");
    reqrep.last_request->complete(0);
    require(t.tickExactlyOnce() == BT::NodeStatus::FAILURE, "cleanup failure result lost");
    finish();
    state = 0;
    publishAct();
  };

  publish();
  auto normal = tree();
  start(normal);
  const int started_calls = reqrep.calls;
  // RT targets alone cannot end ACT; completion is owned by the subsystem.
  for (double opening : {0.0, 65.0, 65.0}) {
    publish(opening);
    require(normal.tickExactlyOnce() == BT::NodeStatus::RUNNING, "RT-only completion accepted");
  }
  state = waiting_open;
  publish();
  require(normal.tickExactlyOnce() == BT::NodeStatus::RUNNING, "waiting-open status rejected");
  // Terminal intent is insufficient until BOTH gates are confirmed closed.
  state = completed;
  publish();
  require(normal.tickExactlyOnce() == BT::NodeStatus::RUNNING, "open gates authorized success");
  running = false;
  publish();
  require(normal.tickExactlyOnce() == BT::NodeStatus::RUNNING, "open send gate authorized success");
  finish();
  require(normal.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "local driver completion was not accepted");
  normal.haltTree();
  require(reqrep.calls == started_calls, "normal completion/halt sent a blocked stop command");

  publish();
  auto faulted = tree();
  start(faulted);
  const int fault_calls = reqrep.calls;
  finish();
  state = timed_out;
  publish();
  require(faulted.tickExactlyOnce() == BT::NodeStatus::FAILURE, "driver timeout reported success");
  require(reqrep.calls == fault_calls, "already stopped driver triggered another stop request");

  publish();
  auto grasp_failed = tree();
  start(grasp_failed);
  const int grasp_calls = reqrep.calls;
  state = 0x41430007;
  publish();
  require(grasp_failed.tickExactlyOnce() == BT::NodeStatus::RUNNING,
          "grasp failure authorized recovery before gate closure");
  finish();
  state = 0x41430007;
  publish();
  require(grasp_failed.tickExactlyOnce() == BT::NodeStatus::FAILURE,
          "grasp failure was not reported as task failure");
  require(grasp_failed.rootBlackboard()->get<std::string>("act_test_result") == "grasp_failed",
          "recoverable grasp result was not published");
  require(reqrep.calls == grasp_calls, "locally stopped grasp failure sent a blocked stop");

  // A short run may complete before its active status is observed.
  publish();
  auto short_run = tree();
  require(short_run.tickExactlyOnce() == BT::NodeStatus::RUNNING, "short run start failed");
  reqrep.last_request->complete(0);
  short_run.tickExactlyOnce();
  arm();
  finish();
  const int short_calls = reqrep.calls;
  require(short_run.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "short run completion missed");
  require(reqrep.calls == short_calls, "short run sent stop");

  // Inference may delay the first active NRT payload. Advancing RT must still be
  // tracked while cached NRT status is being replaced; this is not a feedback gap.
  publish();
  auto delayed_status = tree("feedback_timeout_ms=\"30\" command_timeout_ms=\"300\"");
  require(delayed_status.tickExactlyOnce() == BT::NodeStatus::RUNNING, "delayed status start failed");
  reqrep.last_request->complete(0);
  require(delayed_status.tickExactlyOnce() == BT::NodeStatus::RUNNING, "delayed status ack failed");
  const int delayed_calls = reqrep.calls;
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(10ms);
    publish(); // Fresh RT and NRT envelopes, but the ACT payload is still cached.
    require(delayed_status.tickExactlyOnce() == BT::NodeStatus::RUNNING, "cached status failed prematurely");
    require(reqrep.calls == delayed_calls, "waiting for active NRT triggered a stop");
  }
  arm();
  require(delayed_status.tickExactlyOnce() == BT::NodeStatus::RUNNING, "delayed active status rejected");
  require(reqrep.calls == delayed_calls, "fresh RT was mistaken for a gap during NRT startup");
  finish();
  require(delayed_status.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "delayed status completion missed");

  // Production defaults: 1000 ms feedback limit with a 1000 ms tree tick.
  // Receiver callbacks continue between ticks; the interval between polls is
  // never evidence of a transport gap.
  publish();
  auto slow_tick = tree();
  start(slow_tick);
  const int slow_calls = reqrep.calls;
  for (int i = 0; i < 12; ++i) {
    std::this_thread::sleep_for(100ms);
    publish();
  }
  require(slow_tick.tickExactlyOnce() == BT::NodeStatus::RUNNING, "slow tick rejected fresh feedback");
  require(reqrep.calls == slow_calls, "tree tick period was mistaken for feedback timeout");
  finish();
  require(slow_tick.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "slow tick completion missed");

  // Cached terminal state of the preceding odd generation cannot finish a new start.
  generation += 1;
  state = waiting_close;
  running = sending = true;
  publish();
  auto cached = tree();
  require(cached.tickExactlyOnce() == BT::NodeStatus::RUNNING, "cached task failed");
  reqrep.last_request->complete(0);
  cached.tickExactlyOnce();
  finish();
  require(cached.tickExactlyOnce() == BT::NodeStatus::RUNNING, "old terminal completed the new run");
  arm();
  require(cached.tickExactlyOnce() == BT::NodeStatus::RUNNING, "new generation failed");
  finish();
  require(cached.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "new terminal completion missed");

  // Legacy drivers must fail explicitly rather than falsely promise automatic stopping.
  publish();
  auto legacy = tree();
  start(legacy);
  state = 0;
  publish();
  require(legacy.tickExactlyOnce() == BT::NodeStatus::RUNNING, "legacy failure did not clean up");
  stopFailure(legacy);

  auto rejected = [&](BT::Tree& t) {
    const int before = reqrep.calls;
    require(t.tickExactlyOnce() == BT::NodeStatus::FAILURE, "invalid input/status accepted");
    require(reqrep.calls == before, "invalid input/status sent a command");
    require(t.rootBlackboard()->get<std::string>("act_test_phase") == "failed",
            "ACT startup rejection left the phase at starting");
    require(t.rootBlackboard()->get<std::string>("act_test_result") == "fault" &&
            !t.rootBlackboard()->get<std::string>("act_test_error").empty(),
            "ACT startup rejection lost its fault reason");
  };
  for (const char* options : {"closed_mm=\"-1\"", "finish_opening_mm=\"6\"", "tolerance_mm=\"0\"",
       "timeout_ms=\"0\"", "feedback_timeout_ms=\"0\"", "robot_sel=\"0;1\"", "closed_mm=\"nan\""}) {
    publish();
    auto invalid = tree(options);
    rejected(invalid);
  }
  for (std::size_t size : {0U, 71U, 247U, 249U}) {
    payload_size = size;
    publish();
    auto invalid = tree();
    rejected(invalid);
  }
  payload_size = 248;
  reported_size = 256;
  publish();
  auto bad_layout = tree();
  rejected(bad_layout);
  reported_size = 248;
  ready = false;
  publish();
  auto unready = tree();
  rejected(unready);
  require(unready.rootBlackboard()->get<std::string>("act_test_error") == "ACT subsystem is not ready",
          "ACT-not-ready reason was replaced by a generic startup failure");
  ready = true;
  manager->set_conn_codeit(false);
  auto disconnected_start = tree();
  rejected(disconnected_start);
  require(disconnected_start.rootBlackboard()->get<std::string>("act_test_error") == "Codeit driver disconnected",
          "driver-disconnected startup lost its specific error");
  manager->set_conn_codeit(true);
  publish(50, true, false);
  auto disabled_start = tree();
  rejected(disabled_start);
  require(disabled_start.rootBlackboard()->get<std::string>("act_test_error").find(
              "RT gripper joint disabled or faulted") != std::string::npos,
          "disabled gripper startup lost the specific RT feedback error");
  publish();
  duplicate = true;
  publish();
  auto ambiguous = tree();
  rejected(ambiguous);
  auto named = tree("subsystem_name=\"ActSocketInference#2\"");
  start(named);
  finish();
  require(named.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "explicit instance rejected");
  duplicate = false;
  publish();
  auto wrong_instance = tree("subsystem_name=\"missing\"");
  rejected(wrong_instance);

  // Original failure and halt cleanup stay available, with the gateway rule unchanged.
  publish();
  auto rt_fault = tree();
  start(rt_fault);
  publish(50, true, false);
  require(rt_fault.tickExactlyOnce() == BT::NodeStatus::RUNNING, "RT fault did not stop first");
  stopFailure(rt_fault);

  // Preserve the first fault when the idle-only rule rejects cleanup. Do not
  // immediately repeat the same rejected stop or claim that ACT is confirmed off.
  publish();
  auto rejected_stop = tree();
  start(rejected_stop);
  const int before_rejection = reqrep.calls;
  publish(50, true, false);
  require(rejected_stop.tickExactlyOnce() == BT::NodeStatus::RUNNING, "rejected-stop fault missed");
  reqrep.last_request->complete(-1);
  require(rejected_stop.tickExactlyOnce() == BT::NodeStatus::FAILURE, "rejected stop falsely succeeded");
  const auto reason = rejected_stop.rootBlackboard()->get<std::string>("act_test_error");
  require(reason.find("RT gripper joint disabled or faulted") != std::string::npos &&
          reason.find("stop unconfirmed") != std::string::npos,
          "cleanup rejection replaced the first failure");
  require(rejected_stop.rootBlackboard()->get<std::string>("act_test_phase") == "stop_unconfirmed",
          "unconfirmed stop was not reported explicitly");
  rejected_stop.haltTree();
  require(reqrep.calls == before_rejection + 1, "rejected cleanup was resent immediately");
  finish();
  publish();
  auto replaced = tree();
  start(replaced);
  generation += 2;
  publish();
  require(replaced.tickExactlyOnce() == BT::NodeStatus::RUNNING, "replaced run did not stop first");
  stopFailure(replaced);
  publish();
  auto stale = tree("feedback_timeout_ms=\"30\"");
  require(stale.tickExactlyOnce() == BT::NodeStatus::RUNNING, "stale task failed to start");
  reqrep.last_request->complete(0);
  stale.tickExactlyOnce();
  arm();
  require(stale.tickExactlyOnce() == BT::NodeStatus::RUNNING, "stale task did not confirm");
  auto_nrt = false;
  std::this_thread::sleep_for(35ms);
  publish();
  require(stale.tickExactlyOnce() == BT::NodeStatus::RUNNING, "stale NRT did not stop first");
  stopFailure(stale);
  auto_nrt = true;

  // Re-receiving an identical RT or NRT header cannot refresh progress age.
  publish();
  auto frozen_rt = tree("feedback_timeout_ms=\"30\"");
  require(frozen_rt.tickExactlyOnce() == BT::NodeStatus::RUNNING, "frozen RT start failed");
  reqrep.last_request->complete(0);
  frozen_rt.tickExactlyOnce();
  arm();
  frozen_rt.tickExactlyOnce();
  for (int i = 0; i < 4; ++i) {
    std::this_thread::sleep_for(10ms);
    publish(50, false);
  }
  require(frozen_rt.tickExactlyOnce() == BT::NodeStatus::RUNNING, "frozen RT did not enter cleanup");
  require(frozen_rt.rootBlackboard()->get<std::string>("act_test_error").find("RT feedback stopped advancing") != std::string::npos,
          "duplicate RT frames refreshed progress time");
  stopFailure(frozen_rt);

  publish();
  auto frozen_nrt = tree("feedback_timeout_ms=\"30\"");
  require(frozen_nrt.tickExactlyOnce() == BT::NodeStatus::RUNNING, "frozen NRT start failed");
  reqrep.last_request->complete(0);
  frozen_nrt.tickExactlyOnce();
  arm();
  frozen_nrt.tickExactlyOnce();
  auto_nrt = false;
  for (int i = 0; i < 4; ++i) {
    std::this_thread::sleep_for(10ms);
    publish();
    publishAct(false);
  }
  require(frozen_nrt.tickExactlyOnce() == BT::NodeStatus::RUNNING, "frozen NRT did not enter cleanup");
  require(frozen_nrt.rootBlackboard()->get<std::string>("act_test_error").find("NRT status stopped advancing") != std::string::npos,
          "duplicate NRT frames refreshed progress time");
  stopFailure(frozen_nrt);
  auto_nrt = true;

  // A backwards frame followed by recovery between ticks must remain visible.
  publish();
  auto rewind = tree();
  start(rewind);
  frame -= 5;
  publish();
  frame += 5;
  publish();
  require(rewind.tickExactlyOnce() == BT::NodeStatus::RUNNING, "RT rewind did not enter cleanup");
  require(rewind.rootBlackboard()->get<std::string>("act_test_error").find("RT feedback counter moved backwards") != std::string::npos,
          "RT rewind between ticks was lost");
  stopFailure(rewind);

  publish();
  auto nrt_rewind = tree();
  start(nrt_rewind);
  nrt_frame -= 5;
  publishAct();
  nrt_frame += 5;
  publishAct();
  require(nrt_rewind.tickExactlyOnce() == BT::NodeStatus::RUNNING, "NRT rewind did not enter cleanup");
  require(nrt_rewind.rootBlackboard()->get<std::string>("act_test_error").find("NRT counter moved backwards") != std::string::npos,
          "NRT rewind between ticks was lost");
  stopFailure(nrt_rewind);
  publish();
  auto halted = tree();
  require(halted.tickExactlyOnce() == BT::NodeStatus::RUNNING, "halt start failed");
  const auto pending = reqrep.last_request;
  reqrep.complete_immediately = true;
  halted.haltTree();
  reqrep.complete_immediately = false;
  require(pending->canceled && reqrep.payload["sub_commands"][0]["send_enable"] == 0,
          "halt cleanup changed");

  // RunOnce keeps a completed tree from starting ACT again on subsequent ticks.
  finish();
  auto once = factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><RunOnce then_skip=\"false\">"
    "<RunActUntilGripperDone gripper_model=\"1\" closed_stable_ms=\"10\" stable_ms=\"15\"/>"
    "</RunOnce></BehaviorTree></root>");
  start(once);
  finish();
  require(once.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "RunOnce completion missed");
  const int once_calls = reqrep.calls;
  for (int i = 0; i < 5; ++i) {
    require(once.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "RunOnce lost completion");
    require(reqrep.calls == once_calls, "RunOnce re-enabled ACT");
  }
}

void runTests(FakeReqRep& reqrep) {
  auto* manager = G_BT_MANAGER();
  manager->set_model_size(2);
  manager->set_conn_codeit(true);
  BT::BehaviorTreeFactory factory;
  NodeRegistry::instance().registerAll(factory);

  const auto model_xml = manager->dump_nodes_xml();
  require(model_xml.find("ID=\"SetActSocketInference\"") != std::string::npos,
          "new node is missing from generated node models");

  auto defaults = makeTree(factory);
  require(defaults.tickExactlyOnce() == BT::NodeStatus::RUNNING, "default request must start");
  require(reqrep.payload == nlohmann::json{
      {"event_name", "SetActSocketInference"}, {"user_action", "Click"},
      {"sub_commands", {{{"selected", true}, {"update_enable", 1}, {"send_enable", 1}},
                        {{"selected", true}, {"update_enable", 1}, {"send_enable", 1}}}}},
      "default command does not match the driver protocol");
  const int pending_calls = reqrep.calls;
  require(defaults.tickExactlyOnce() == BT::NodeStatus::RUNNING, "pending request must wait");
  require(reqrep.calls == pending_calls, "pending tick resent the command");
  reqrep.last_request->complete(0);
  require(defaults.tickExactlyOnce() == BT::NodeStatus::SUCCESS, "success response was lost");

  for (int update : {0, 1}) {
    for (int send : {0, 1}) {
      auto tree = makeTree(factory, "robot_sel=\"1\" update_enable=\"" + std::to_string(update) +
                                   "\" send_enable=\"" + std::to_string(send) + "\"");
      require(tree.tickExactlyOnce() == BT::NodeStatus::RUNNING, "switch combination rejected");
      require(reqrep.payload["sub_commands"][0] == nlohmann::json{{"selected", false}},
              "command fields leaked to an unselected robot");
      require(reqrep.payload["sub_commands"][1] == nlohmann::json{
          {"selected", true}, {"update_enable", update}, {"send_enable", send}},
          "switch values were not forwarded");
      reqrep.last_request->complete(-1);
      require(tree.tickExactlyOnce() == BT::NodeStatus::FAILURE, "driver error was ignored");
    }
  }

  for (const char* attributes : {
           "update_enable=\"2\"", "send_enable=\"-1\"",
           "robot_sel=\"-1\"", "robot_sel=\"2\"",
           "robot_sel=\"0;2\"", "robot_sel=\"\""}) {
    const int calls = reqrep.calls;
    auto tree = makeTree(factory, attributes);
    require(tree.tickExactlyOnce() == BT::NodeStatus::FAILURE, "invalid input was accepted");
    require(reqrep.calls == calls, "invalid input sent a command");
  }

  // Missing blackboard values fail at tick time, while malformed literal
  // types are rejected by BehaviorTree.CPP when the tree is created.
  for (const char* port : {"update_enable", "send_enable"}) {
    const int calls = reqrep.calls;
    auto tree = makeTree(factory, std::string(port) + "=\"{missing_switch}\"");
    require(tree.tickExactlyOnce() == BT::NodeStatus::FAILURE,
            "missing blackboard input was accepted");
    require(reqrep.calls == calls, "missing blackboard input sent a command");
    bool rejected_type = false;
    try {
      auto invalid = makeTree(factory, std::string(port) + "=\"invalid\"");
    } catch (const BT::LogicError&) {
      rejected_type = true;
    }
    require(rejected_type, "malformed XML input was accepted");
    require(reqrep.calls == calls, "malformed XML input sent a command");
  }

  manager->set_model_size(0);
  auto no_models = makeTree(factory);
  const int before_no_models = reqrep.calls;
  require(no_models.tickExactlyOnce() == BT::NodeStatus::FAILURE, "missing model state accepted");
  require(reqrep.calls == before_no_models, "missing model state sent a command");
  manager->set_model_size(1);
  auto single_robot = makeTree(factory, "robot_sel=\"0\"");
  require(single_robot.tickExactlyOnce() == BT::NodeStatus::RUNNING, "single robot rejected");
  require(reqrep.payload["sub_commands"].size() == 1, "model count was cached");
  auto halted_request = reqrep.last_request;
  single_robot.haltTree();
  require(halted_request->canceled, "halt did not cancel the request");
  halted_request->complete(0);
  require(single_robot.tickExactlyOnce() == BT::NodeStatus::RUNNING,
          "late completion corrupted the next request");
  manager->set_conn_codeit(false);
  auto disconnected_request = reqrep.last_request;
  require(single_robot.tickExactlyOnce() == BT::NodeStatus::FAILURE,
          "disconnect did not fail the request");
  require(disconnected_request->canceled, "disconnect did not cancel the request");
  const int before_disconnect = reqrep.calls;
  auto disconnected = makeTree(factory, "robot_sel=\"0\"");
  require(disconnected.tickExactlyOnce() == BT::NodeStatus::FAILURE,
          "disconnected driver accepted a command");
  require(reqrep.calls == before_disconnect, "disconnected driver sent a command");

  manager->set_conn_codeit(true);
  reqrep.reject_creation = true;
  auto rejected = makeTree(factory, "robot_sel=\"0\"");
  require(rejected.tickExactlyOnce() == BT::NodeStatus::FAILURE, "request creation error ignored");
  reqrep.reject_creation = false;
  auto stopping = makeTree(factory, "robot_sel=\"0\"");
  require(stopping.tickExactlyOnce() == BT::NodeStatus::RUNNING, "shutdown request did not start");
  auto stopping_request = reqrep.last_request;
  manager->beginShutdown();
  require(stopping.tickExactlyOnce() == BT::NodeStatus::FAILURE, "shutdown did not fail request");
  require(stopping_request->canceled, "shutdown did not cancel request");
  const int before_shutdown = reqrep.calls;
  auto stopped = makeTree(factory, "robot_sel=\"0\"");
  require(stopped.tickExactlyOnce() == BT::NodeStatus::FAILURE, "shutdown accepted new command");
  require(reqrep.calls == before_shutdown, "shutdown sent a command");
}
}  // namespace

int main(int argc, char** argv) {
  FakeReqRep reqrep;
  // Keep tests isolated from live Groot2 servers and robot endpoints.
  BTManager::Construct(nullptr, &reqrep, false);
  try {
    const bool run_once = argc == 2 && std::string(argv[1]) == "--run-once";
    const bool scheduler = argc == 2 && std::string(argv[1]) == "--scheduler";
    const bool gripper = argc == 2 && std::string(argv[1]) == "--gripper";
    const bool lifecycle = argc == 2 && std::string(argv[1]) == "--lifecycle";
    const bool return_home = argc == 2 && std::string(argv[1]) == "--return-home";
    const bool grasp_retry = argc == 2 && std::string(argv[1]) == "--grasp-retry";
    if (argc == 3 && std::string(argv[1]) == "--pi05-tree") runPi05Tests(reqrep, argv[2]);
    else if (grasp_retry) runGraspRetryTests(reqrep);
    else if (return_home) runReturnHomeTests(reqrep);
    else if (lifecycle) runLifecycleTests(reqrep);
    else if (scheduler) runSchedulerTests(reqrep);
    else if (gripper) runGripperTests(reqrep);
    else if (run_once) runOnceTests(reqrep);
    else runTests(reqrep);
    BTManager::Destruct();
    std::cout << (grasp_retry ? "GraspRetry" : return_home ? "MoveAbsJJointTarget" : lifecycle ? "Lifecycle" : scheduler ? "Scheduler" : gripper ? "RunActUntilGripperDone" : run_once ? "RunActSocketInferenceOnce" : "SetActSocketInference")
              << " regression checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    BTManager::Destruct();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
