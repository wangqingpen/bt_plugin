// Included inside the command-test anonymous namespace to reuse its fake bus.
struct HomeFeedback {
  BTManager* manager{G_BT_MANAGER()};
  std::int64_t frame{1};
  double position{0}, velocity{0}, target{0.5};
  double gripper_position{0.065}, gripper_velocity{0};
  double gripper_actual_position{0.065};
  bool physical_gripper_present{true};
  bool gripper_enabled{true};
  std::string gripper_function{"none"};
  bool enabled{true}, matrix{false}, duplicate_target{false}, ready{true}, act_running{false};
  int model_error{0}, target_dof{6};
  std::uint64_t generation{2};
  std::int32_t act_state{0};
  std::string function{"none"}, target_name{"pick_home"};

  HomeFeedback() {
    manager->set_conn_codeit(true);
    manager->set_gateway_status(0);
    publish();
  }
  void publish(bool advance = true) {
    if (advance) ++frame;
    overall_system_rtstate::SystemRtState rt;
    rt.mutable_head()->set_frame_id(frame);
    rt.mutable_head()->set_timestamp(frame * 2);
    rt.set_system_running_state(true);
    auto* arm = rt.add_model();
    arm->set_model_name("arm");
    arm->set_current_func_name(function);
    arm->set_error_code(model_error);
    for (int i = 0; i < 6; ++i) {
      auto* joint = arm->add_joint();
      joint->set_position(position);
      joint->set_velocity(velocity);
      joint->set_is_enabled(enabled);
    }
    auto* gripper_model = rt.add_model();
    gripper_model->set_current_func_name(gripper_function);
    auto* gripper = gripper_model->add_joint();
    gripper->set_is_enabled(gripper_enabled);
    gripper->set_position(gripper_position);
    gripper->set_velocity(gripper_velocity);
    auto bytes = rt.SerializeAsString();
    require(manager->update_rt(bytes.data(), bytes.size()), "home RT update failed");

    overall_system_nrtstate::SystemNrtState nrt;
    nrt.mutable_head()->set_frame_id(frame);
    nrt.mutable_head()->set_timestamp(frame * 2);
    auto* model = nrt.add_model();
    model->set_model_name("arm");
    if (matrix) {
      auto* variable = model->add_matrix_variables();
      variable->set_matrixvar_name(target_name);
      for (int i = 0; i < target_dof; ++i) variable->add_data(target);
    } else {
      auto* point = model->add_teach_points();
      point->set_point_name(target_name);
      for (int i = 0; i < target_dof; ++i) point->add_jointtarget(target);
    }
    if (duplicate_target) *model->add_teach_points() = model->teach_points(0);
    nrt.add_model();
    auto* act = nrt.add_subsystem();
    act->set_subsystem_name("ActSocketInference#0#0");
    act->set_state(1);
    std::array<char, 248> data{};
    auto field = [&](std::size_t offset, const auto& value) {
      std::memcpy(data.data() + offset, &value, sizeof(value));
    };
    field(0, std::uint32_t(248));
    field(8, std::int32_t(act_running));
    field(12, std::int32_t(ready));
    field(16, std::int32_t(1));
    field(40, std::int32_t(act_running));
    field(44, act_state);
    field(56, generation);
    field(64, generation);
    act->set_data(data.data(), data.size());
    if (physical_gripper_present) {
      auto* physical = nrt.add_subsystem();
      physical->set_subsystem_name("TwoFingersGripperSerialYSEG2#0#1");
      physical->set_state(1);
      physical->set_data(reinterpret_cast<const char*>(&gripper_actual_position), sizeof(gripper_actual_position));
    }
    bytes = nrt.SerializeAsString();
    require(manager->update_nrt(bytes.data(), bytes.size()), "home NRT update failed");
  }
  BT::NodeStatus next(BT::Tree& tree, int delay_ms = 3) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    publish();
    return tree.tickExactlyOnce();
  }
};

void runReturnHomeTests(FakeReqRep& reqrep) {
  auto* manager = G_BT_MANAGER();
  BT::BehaviorTreeFactory factory;
  NodeRegistry::instance().registerAll(factory);
  require(manager->dump_nodes_xml().find("position_tolerance_rad") != std::string::npos,
          "return-home ports missing from generated node model");
  auto tree = [&](std::map<std::string, std::string> overrides = {}) {
    std::map<std::string, std::string> ports{
      {"robot_sel", "0"}, {"jointtarget", "pick_home"}, {"speed", "v50"},
      {"timeout_ms", "1000"}, {"feedback_timeout_ms", "1000"},
      {"idle_stable_ms", "2"}, {"stable_ms", "2"},
      {"phase", "{home_phase}"}, {"error", "{home_error}"}};
    for (auto& port : overrides) ports[port.first] = port.second;
    std::string xml = "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
                      "<RunOnce then_skip=\"false\"><MoveAbsJJointTarget ";
    for (const auto& port : ports) xml += port.first + "=\"" + port.second + "\" ";
    return factory.createTreeFromText(xml + "/></RunOnce></BehaviorTree></root>");
  };
  auto submit = [&](HomeFeedback& state, BT::Tree& t) {
    const int before = reqrep.calls;
    require(t.tickExactlyOnce() == BT::NodeStatus::RUNNING, "home did not wait for stable idle");
    require(reqrep.calls == before, "home submitted before idle hold");
    require(state.next(t) == BT::NodeStatus::RUNNING, "home submission failed");
    require(reqrep.calls == before + 1, "home must submit exactly one request");
  };

  HomeFeedback state;
  auto normal = tree();
  const int initial_calls = reqrep.calls;
  manager->set_gateway_status(1);
  require(normal.tickExactlyOnce() == BT::NodeStatus::RUNNING, "busy gateway was not awaited");
  manager->set_gateway_status(0);
  state.function = "JogAnyJ";
  require(state.next(normal) == BT::NodeStatus::RUNNING, "old ACT motion was not awaited");
  state.function = "none";
  state.velocity = 0.2;
  state.next(normal);
  require(reqrep.calls == initial_calls, "busy arm received a home command");
  state.velocity = 0;
  state.next(normal);
  state.next(normal);
  require(reqrep.calls == initial_calls + 1, "idle arm did not receive its home command");
  require(reqrep.payload == nlohmann::json{
    {"event_name", "MoveAbsJJointTarget"}, {"user_action", "Click"},
    {"sub_commands", {{{"selected", true}, {"jointtarget", "pick_home"}, {"speed", "v50"}},
                      {{"selected", false}}}}}, "home command mapping/model selection is incorrect");
  for (int i = 0; i < 3; ++i)
    require(normal.tickExactlyOnce() == BT::NodeStatus::RUNNING, "pending request did not wait");
  require(reqrep.calls == initial_calls + 1, "home command was resent while pending");
  reqrep.last_request->complete(0);
  require(state.next(normal) == BT::NodeStatus::RUNNING, "acknowledgement was treated as arrival");
  require(state.next(normal) == BT::NodeStatus::RUNNING, "wrong position was treated as arrival");
  state.position = state.target;
  state.velocity = 0.1;
  require(state.next(normal) == BT::NodeStatus::RUNNING, "moving arm was treated as arrived");
  state.velocity = 0;
  require(state.next(normal) == BT::NodeStatus::RUNNING, "arrival hold was skipped");
  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  state.publish(false);
  require(normal.tickExactlyOnce() == BT::NodeStatus::RUNNING, "duplicate frame authorized completion");
  require(state.next(normal) == BT::NodeStatus::SUCCESS, "stable target arrival was not accepted");
  for (int i = 0; i < 3; ++i) require(normal.tickExactlyOnce() == BT::NodeStatus::SUCCESS,
                                    "RunOnce lost home completion");
  require(reqrep.calls == initial_calls + 1, "completed tree repeated its home command");

  // An already-at-home move may finish between frames; no RUNNING edge is required.
  HomeFeedback custom;
  custom.target_name = "park_a";
  custom.matrix = true;
  custom.position = custom.target;
  custom.publish();
  auto configured = tree({{"jointtarget", "park_a"}, {"speed", "v20"}});
  reqrep.complete_immediately = true;
  submit(custom, configured);
  reqrep.complete_immediately = false;
  require(reqrep.payload["sub_commands"][0]["jointtarget"] == "park_a" &&
          reqrep.payload["sub_commands"][0]["speed"] == "v20", "custom target/speed were lost");
  require(configured.tickExactlyOnce() == BT::NodeStatus::RUNNING, "cached pre-ack RT authorized completion");
  require(custom.next(configured) == BT::NodeStatus::RUNNING, "post-ack hold was skipped");
  require(custom.next(configured) == BT::NodeStatus::SUCCESS, "zero-distance move did not complete");

  HomeFeedback padded;
  padded.target_dof = 10;
  padded.publish();
  auto padded_point = tree();
  submit(padded, padded_point);
  padded_point.haltTree();

  HomeFeedback no_request_state;
  auto no_request = tree();
  require(no_request.tickExactlyOnce() == BT::NodeStatus::RUNNING, "request-creation test did not wait");
  reqrep.reject_creation = true;
  require(no_request_state.next(no_request) == BT::NodeStatus::FAILURE,
          "request creation failure was ignored");
  reqrep.reject_creation = false;

  for (const auto& bad : std::vector<std::map<std::string, std::string>>{
      {{"robot_sel", "-1"}}, {{"robot_sel", "2"}}, {{"robot_sel", "0;0"}},
      {{"robot_sel", ""}}, {{"jointtarget", "missing"}}, {{"jointtarget", "x||Enable"}},
      {{"speed", "v50 --all"}}, {{"timeout_ms", "0"}}, {{"stable_ms", "0"}},
      {{"position_tolerance_rad", "nan"}}, {{"velocity_tolerance_rad_s", "-1"}}}) {
    HomeFeedback valid;
    const int calls = reqrep.calls;
    auto invalid = tree(bad);
    require(invalid.tickExactlyOnce() == BT::NodeStatus::FAILURE, "invalid home configuration accepted");
    require(reqrep.calls == calls, "invalid home configuration submitted motion");
  }
  for (int mode = 0; mode < 4; ++mode) {
    HomeFeedback invalid;
    if (mode == 0) invalid.target_dof = 5;
    if (mode == 1) invalid.target = std::numeric_limits<double>::quiet_NaN();
    if (mode == 2) invalid.duplicate_target = true;
    if (mode == 3) invalid.enabled = false;
    invalid.publish();
    auto t = tree();
    const int calls = reqrep.calls;
    require(t.tickExactlyOnce() == BT::NodeStatus::FAILURE, "bad target or disabled joint accepted");
    require(reqrep.calls == calls, "invalid feedback submitted motion");
  }

  HomeFeedback rejected_state;
  auto rejected = tree();
  submit(rejected_state, rejected);
  auto request = reqrep.last_request;
  const int rejected_calls = reqrep.calls;
  request->complete(-1);
  require(rejected.tickExactlyOnce() == BT::NodeStatus::FAILURE, "move rejection was ignored");
  require(reqrep.calls == rejected_calls, "rejected move was retried");
  require(rejected.rootBlackboard()->get<std::string>("home_error").find("rejected") != std::string::npos,
          "move rejection detail was lost");

  HomeFeedback stale_state;
  auto stale = tree({{"feedback_timeout_ms", "10"}});
  submit(stale_state, stale);
  request = reqrep.last_request;
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  stale_state.publish(false);
  require(stale.tickExactlyOnce() == BT::NodeStatus::FAILURE && request->canceled,
          "duplicate feedback bypassed stale detection");

  HomeFeedback rewound_state;
  auto rewound = tree();
  submit(rewound_state, rewound);
  rewound_state.frame -= 2;
  rewound_state.publish(false);
  rewound_state.frame += 4;
  rewound_state.publish();
  require(rewound.tickExactlyOnce() == BT::NodeStatus::FAILURE, "counter rewind between ticks was lost");

  HomeFeedback changed_state;
  auto changed = tree();
  submit(changed_state, changed);
  changed_state.target += 0.1;
  changed_state.publish();
  require(changed.tickExactlyOnce() == BT::NodeStatus::FAILURE, "changed target was silently accepted");

  HomeFeedback waiting_state;
  manager->set_gateway_status(1);
  auto waiting = tree({{"timeout_ms", "10"}});
  const int waiting_calls = reqrep.calls;
  waiting.tickExactlyOnce();
  require(waiting_state.next(waiting, 15) == BT::NodeStatus::FAILURE, "idle wait did not time out");
  require(reqrep.calls == waiting_calls, "idle timeout submitted a move");

  HomeFeedback timeout_state;
  auto timeout = tree({{"timeout_ms", "15"}});
  submit(timeout_state, timeout);
  request = reqrep.last_request;
  require(timeout_state.next(timeout, 20) == BT::NodeStatus::FAILURE && request->canceled,
          "pending motion timeout did not fail/cancel local waiting");

  HomeFeedback halted_state;
  auto halted = tree();
  submit(halted_state, halted);
  auto old_request = reqrep.last_request;
  halted.haltTree();
  require(old_request->canceled, "halt did not cancel home request waiting");
  submit(halted_state, halted);
  old_request->complete(0);
  require(halted.tickExactlyOnce() == BT::NodeStatus::RUNNING, "old reply completed a new home request");
  request = reqrep.last_request;
  manager->set_conn_codeit(false);
  require(halted.tickExactlyOnce() == BT::NodeStatus::FAILURE && request->canceled,
          "disconnect did not fail/cancel home request waiting");
  manager->set_conn_codeit(true);

  // Use the shipped tree to test the full ACT-success -> home -> end sequence.
  HomeFeedback integration;
  auto example_path = std::filesystem::path(__FILE__).parent_path() / "trees/test3-return-home.xml";
  auto sequence = factory.createTreeFromFile(example_path.string());
  const int before_act = reqrep.calls;
  require(sequence.tickExactlyOnce() == BT::NodeStatus::RUNNING && reqrep.calls == before_act + 1,
          "example did not start ACT once");
  reqrep.last_request->complete(0);
  sequence.tickExactlyOnce();
  integration.act_running = true;
  integration.generation = 3;
  integration.act_state = 0x41430001;
  integration.publish();
  require(sequence.tickExactlyOnce() == BT::NodeStatus::RUNNING, "example lost active ACT status");
  integration.act_running = false;
  integration.generation = 4;
  integration.act_state = 0x41430003;
  manager->set_gateway_status(1);
  integration.function = "JogAnyJ";
  integration.publish();
  require(sequence.tickExactlyOnce() == BT::NodeStatus::RUNNING && reqrep.calls == before_act + 1,
          "home was submitted before the last ACT motion ended");
  manager->set_gateway_status(0);
  integration.function = "none";
  integration.next(sequence);
  integration.next(sequence, 210);
  require(reqrep.calls == before_act + 2 && reqrep.payload["event_name"] == "MoveAbsJJointTarget",
          "example did not submit exactly one return-home command after ACT");
  reqrep.last_request->complete(0);
  integration.position = integration.target;
  require(integration.next(sequence) == BT::NodeStatus::RUNNING, "example ended on command acknowledgement");
  integration.next(sequence);
  require(integration.next(sequence, 210) == BT::NodeStatus::SUCCESS, "example did not finish after home arrival");
  sequence.tickExactlyOnce();
  require(reqrep.calls == before_act + 2, "completed example repeated ACT or home");

  HomeFeedback failed_act;
  failed_act.ready = false;
  failed_act.publish();
  auto failure_sequence = factory.createTreeFromFile(example_path.string());
  const int before_failure = reqrep.calls;
  require(failure_sequence.tickExactlyOnce() == BT::NodeStatus::FAILURE,
          "unready ACT was accepted by the example");
  require(reqrep.calls == before_failure, "ACT failure still submitted return-home motion");
}
