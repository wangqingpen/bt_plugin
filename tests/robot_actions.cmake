add_executable(robot_actions_test
  robot_actions_test.cpp
  ${PROJECT_SOURCE_DIR}/source/bt_manager.cpp
  ${PROJECT_SOURCE_DIR}/source/run_act_until_gripper_done.cpp
  ${PROJECT_SOURCE_DIR}/source/move_absj_joint_target.cpp
  ${PROJECT_SOURCE_DIR}/source/robot_actions.cpp)
target_compile_definitions(robot_actions_test PRIVATE PLUGIN_LOG_NAME=BtPlug)
target_link_libraries(robot_actions_test PRIVATE
  gateway_proto::gateway_proto fmt::fmt-header-only xplugin::sdk
  behaviortree_cpp tinyxml2::tinyxml2 Threads::Threads)
add_test(NAME robot_actions COMMAND robot_actions_test
  ${CMAKE_CURRENT_SOURCE_DIR}/trees/generic
  ${CMAKE_CURRENT_BINARY_DIR}/generic-frames.json)
set_tests_properties(robot_actions PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
