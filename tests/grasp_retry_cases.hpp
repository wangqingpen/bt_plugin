// Uses the fake command bus and HomeFeedback; never connects to robot hardware.
void runGraspRetryTests(FakeReqRep& reqrep) {
  // The shipped tree opens to 62.5 mm, at the centre of its 60..65 mm range.
  struct RetryFeedback : HomeFeedback {
    RetryFeedback() {
      gripper_position = gripper_actual_position = 0.0625;
      publish();
    }
  };
  auto* manager = G_BT_MANAGER();
  BT::BehaviorTreeFactory factory;
  NodeRegistry::instance().registerAll(factory);
  const auto directory = std::filesystem::path(__FILE__).parent_path() / "trees";
  std::ifstream file(directory / "test3-grasp-retry.xml");
  require(bool(file), "missing grasp retry example");
  const std::string original((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  auto replace = [](std::string& text, const std::string& from, const std::string& to) {
    std::size_t at = 0;
    while ((at = text.find(from, at)) != std::string::npos) {
      text.replace(at, from.size(), to);
      at += to.size();
    }
  };
  auto make = [&](int limit = -1) {
    auto xml = original;
    replace(xml, "idle_stable_ms=\"200\"", "idle_stable_ms=\"2\"");
    replace(xml, "stable_ms=\"200\"", "stable_ms=\"2\"");
    replace(xml, "max_attempts=\"-1\"", "max_attempts=\"" + std::to_string(limit) + "\"");
    return factory.createTreeFromText(xml);
  };
  auto until_request = [&](HomeFeedback& state, BT::Tree& tree, int previous) {
    for (int i = 0; i < 40 && reqrep.calls == previous; ++i)
      require(state.next(tree) == BT::NodeStatus::RUNNING, "retry tree terminated before expected request");
    require(reqrep.calls == previous + 1, "retry tree did not submit exactly one expected request");
    return reqrep.last_request;
  };
  auto arm = [&](HomeFeedback& state, BT::Tree& tree) {
    reqrep.last_request->complete(0);
    require(tree.tickExactlyOnce() == BT::NodeStatus::RUNNING, "ACT acknowledgement ended retry tree");
    state.generation += (state.generation & 1U) ? 2 : 1;
    state.act_running = true;
    state.act_state = 0x41430001;
    state.publish();
    require(tree.tickExactlyOnce() == BT::NodeStatus::RUNNING, "retry ACT active status rejected");
  };
  auto fail_grasp = [&](HomeFeedback& state, BT::Tree& tree) {
    state.act_running = false;
    ++state.generation;
    state.act_state = 0x41430007;
    state.gripper_position = 0.004;
    state.gripper_actual_position = 0.004;
    state.publish();
    return tree.tickExactlyOnce();
  };

  RetryFeedback state;
  state.position = state.target;
  state.publish();
  auto tree = make();
  int calls = reqrep.calls;
  require(tree.tickExactlyOnce() == BT::NodeStatus::RUNNING && reqrep.calls == calls,
          "initial home check did not wait without motion");
  until_request(state, tree, calls);
  require(reqrep.payload["event_name"] == "SetActSocketInference",
          "already home/open preparation sent unnecessary motion");
  arm(state, tree);
  calls = reqrep.calls;
  state.position = 0.1;
  state.gripper_position = 0; // Virtual close target, while the fingers hold an object.
  state.gripper_actual_position = 0.020;
  state.act_state = 0x41430002;
  for (int i = 0; i < 5; ++i)
    require(state.next(tree) == BT::NodeStatus::RUNNING && reqrep.calls == calls,
            "a successful hold prematurely stopped ACT or submitted a home move");
  state.act_state = 0x41430007;
  state.publish();
  require(tree.tickExactlyOnce() == BT::NodeStatus::RUNNING && reqrep.calls == calls,
          "terminal intent with active gates triggered recovery");
  state.position = 0.1;
  state.function = "JogAnyJ";
  manager->set_gateway_status(1);
  require(fail_grasp(state, tree) == BT::NodeStatus::RUNNING, "recoverable failure stopped the entire tree");
  state.next(tree);
  require(reqrep.calls == calls, "recovery ignored old ACT motion/busy gateway");
  state.function = "none";
  manager->set_gateway_status(0);
  until_request(state, tree, calls);
  require(reqrep.payload["event_name"] == "MoveAbsJJointTarget" &&
          reqrep.payload["sub_commands"][0]["jointtarget"] == "pick_home" &&
          reqrep.payload["sub_commands"][1] == nlohmann::json{{"selected", false}},
          "grasp failure did not return only the arm to pick_home");
  reqrep.last_request->complete(0);
  state.position = state.target;
  calls = reqrep.calls;
  until_request(state, tree, calls);
  require(reqrep.payload["event_name"] == "MoveAbsJVar" &&
          reqrep.payload["sub_commands"][0] == nlohmann::json{{"selected", false}},
          "retry did not open only the gripper after home arrival");
  auto value = reqrep.payload["sub_commands"][1]["jointtarget_value"].get<std::string>();
  auto values_json = value;
  values_json.front() = '[';
  values_json.back() = ']';
  const auto joint_values = nlohmann::json::parse(values_json).get<std::vector<double>>();
  require(joint_values.size() == 10 && std::abs(joint_values[0] - 0.0625) < 1e-12 &&
          std::all_of(joint_values.begin()+1, joint_values.end(), [](double v) { return v == 0; }),
          "gripper command must use metres and ten Codeit joint slots");
  reqrep.last_request->complete(0);
  calls = reqrep.calls;
  for (int i = 0; i < 4; ++i)
    require(state.next(tree) == BT::NodeStatus::RUNNING && reqrep.calls == calls,
            "ACT restarted before gripper opening was confirmed");
  state.gripper_position = 0.0625;
  for (int i = 0; i < 5; ++i)
    require(state.next(tree) == BT::NodeStatus::RUNNING && reqrep.calls == calls,
            "virtual open target authorized ACT before physical gripper opened");
  state.gripper_actual_position = 0.0625;
  until_request(state, tree, calls);
  require(reqrep.payload["event_name"] == "SetActSocketInference", "retry did not restart ACT");
  arm(state, tree);
  state.act_running = false;
  ++state.generation;
  state.act_state = 0x41430003;
  state.publish();
  calls = reqrep.calls;
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 30 && result == BT::NodeStatus::RUNNING; ++i) result = state.next(tree);
  require(result == BT::NodeStatus::SUCCESS, "successful retry did not finish the one-job tree");
  require(reqrep.calls == calls, "already-home success submitted extra motion");
  require(tree.rootBlackboard()->get<std::string>("act_result") == "completed",
          "successful retry retained an old grasp failure");

  RetryFeedback missing;
  missing.position = missing.target;
  missing.physical_gripper_present = false;
  missing.publish();
  auto no_physical = make();
  calls = reqrep.calls;
  result = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 30 && result == BT::NodeStatus::RUNNING; ++i) result = missing.next(no_physical);
  require(result == BT::NodeStatus::FAILURE && reqrep.calls == calls,
          "missing physical gripper feedback fell back to virtual feedback or started ACT");

  RetryFeedback initial_away;
  initial_away.gripper_position = 0.004;
  initial_away.publish();
  auto preparation = make();
  calls = reqrep.calls;
  until_request(initial_away, preparation, calls);
  require(reqrep.payload["event_name"] == "MoveAbsJJointTarget", "ACT started before initial pick_home check");
  preparation.haltTree();

  RetryFeedback disabled;
  disabled.enabled = false;
  disabled.publish();
  auto unsafe = make();
  unsafe.rootBlackboard()->set("act_result", std::string("grasp_failed"));
  calls = reqrep.calls;
  require(unsafe.tickExactlyOnce() == BT::NodeStatus::FAILURE && reqrep.calls == calls,
          "disabled arm or stale failure marker authorized recovery/motion");

  RetryFeedback fatal;
  fatal.position = fatal.target;
  fatal.publish();
  auto fatal_tree = make();
  calls = reqrep.calls;
  until_request(fatal, fatal_tree, calls);
  arm(fatal, fatal_tree);
  fatal.model_error = -501;
  fatal.publish();
  calls = reqrep.calls;
  require(fatal_tree.tickExactlyOnce() == BT::NodeStatus::RUNNING, "fatal ACT fault did not enter existing cleanup");
  require(reqrep.calls == calls + 1 && reqrep.payload["event_name"] == "SetActSocketInference" &&
          reqrep.payload["sub_commands"][0]["send_enable"] == 0, "fatal fault submitted recovery motion");
  reqrep.last_request->complete(-1);
  calls = reqrep.calls;
  require(fatal_tree.tickExactlyOnce() == BT::NodeStatus::FAILURE && reqrep.calls == calls,
          "fatal/stop-unconfirmed error was automatically retried");

  RetryFeedback exhausted;
  exhausted.position = exhausted.target;
  exhausted.publish();
  auto limited = make(1);
  calls = reqrep.calls;
  until_request(exhausted, limited, calls);
  arm(exhausted, limited);
  fail_grasp(exhausted, limited);
  calls = reqrep.calls;
  result = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 30 && result == BT::NodeStatus::RUNNING; ++i) result = exhausted.next(limited);
  require(result == BT::NodeStatus::FAILURE && reqrep.calls == calls &&
          limited.rootBlackboard()->get<std::string>("act_result") == "retry_exhausted",
          "attempt limit did not recover home and stop");

  RetryFeedback rejected;
  rejected.position = rejected.target;
  rejected.publish();
  auto recovery_rejected = make();
  calls = reqrep.calls;
  until_request(rejected, recovery_rejected, calls);
  arm(rejected, recovery_rejected);
  rejected.position = 0.1;
  fail_grasp(rejected, recovery_rejected);
  calls = reqrep.calls;
  until_request(rejected, recovery_rejected, calls);
  reqrep.last_request->complete(-1);
  calls = reqrep.calls;
  require(recovery_rejected.tickExactlyOnce() == BT::NodeStatus::FAILURE && reqrep.calls == calls,
          "recovery rejection restarted ACT");

  RetryFeedback halted;
  halted.position = halted.target;
  halted.publish();
  auto canceled = make();
  calls = reqrep.calls;
  until_request(halted, canceled, calls);
  arm(halted, canceled);
  halted.position = 0.1;
  fail_grasp(halted, canceled);
  calls = reqrep.calls;
  auto old = until_request(halted, canceled, calls);
  canceled.haltTree();
  require(old->canceled && canceled.rootBlackboard()->get<std::string>("act_result") == "canceled",
          "halt did not cancel recovery waiting and invalidate result");
  calls = reqrep.calls;
  old->complete(0);
  require(reqrep.calls == calls, "late recovery reply restarted ACT");

  auto loop = factory.createTreeFromFile((directory / "test3-grasp-retry-loop.xml").string());
  require(loop.rootNode()->registrationName() == "Repeat", "continuous retry example is not a loop");
}
