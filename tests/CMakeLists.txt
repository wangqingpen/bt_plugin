add_executable(set_act_socket_inference_test
  set_act_socket_inference_test.cpp
  ${PROJECT_SOURCE_DIR}/source/bt_manager.cpp
  ${PROJECT_SOURCE_DIR}/source/run_act_until_gripper_done.cpp
  ${PROJECT_SOURCE_DIR}/source/move_absj_joint_target.cpp
)
target_compile_definitions(set_act_socket_inference_test PRIVATE
  PLUGIN_LOG_NAME=BtPlug
)
target_link_libraries(set_act_socket_inference_test PRIVATE
  gateway_proto::gateway_proto fmt::fmt-header-only xplugin::sdk
  behaviortree_cpp tinyxml2::tinyxml2 Threads::Threads
)
add_test(NAME set_act_socket_inference COMMAND set_act_socket_inference_test)
add_test(NAME run_act_socket_inference_once COMMAND set_act_socket_inference_test --run-once)
add_test(NAME run_act_until_gripper_done COMMAND set_act_socket_inference_test --gripper)
add_test(NAME bt_scheduler_wakeup COMMAND set_act_socket_inference_test --scheduler)
add_test(NAME bt_explicit_start COMMAND set_act_socket_inference_test --lifecycle)
add_test(NAME move_absj_joint_target COMMAND set_act_socket_inference_test --return-home)
add_test(NAME act_grasp_retry COMMAND set_act_socket_inference_test --grasp-retry)
set_tests_properties(act_grasp_retry PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
set_tests_properties(bt_explicit_start PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
set_tests_properties(bt_scheduler_wakeup PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)

add_test(NAME pi05_grasp_tree COMMAND set_act_socket_inference_test --pi05-tree
  ${CMAKE_CURRENT_SOURCE_DIR}/trees/test-pi05-grasp-retry-loop.xml)

include(${CMAKE_CURRENT_SOURCE_DIR}/robot_actions.cmake)
