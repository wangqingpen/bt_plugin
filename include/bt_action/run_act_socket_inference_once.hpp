#pragma once

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <string>
#include <vector>

#include "bt_json_request.hpp"
#include "bt_manager.hpp"

class RunActSocketInferenceOnce final : public BT::StatefulActionNode {
public:
  RunActSocketInferenceOnce(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<int>>("robot_sel", "0", "Select exactly one command model"),
        BT::InputPort<int>("timeout_ms", 10000, "Driver one-shot timeout in milliseconds"),
        BT::InputPort<std::string>("subsystem_name", "", "ACT subsystem name; empty selects the only instance"),
    };
  }

  BT::NodeStatus onStart() override {
    auto* manager = G_BT_MANAGER();
    if (!manager || manager->is_stopping() || !manager->is_conn_codeit()) {
      return BT::NodeStatus::FAILURE;
    }

    int timeout_ms = 10000;
    std::string subsystem_name;
    if (!getInput("timeout_ms", timeout_ms) || timeout_ms <= 0 ||
        !getInput("subsystem_name", subsystem_name)) {
      LOG_ERROR("[RunActSocketInferenceOnce] timeout_ms must be positive and subsystem_name must be readable");
      return BT::NodeStatus::FAILURE;
    }

    // 一条 ACT 单步已同时控制机械臂和夹爪，只通过一个模型通道触发。
    std::vector<int> robot_sel;
    if (!getInput("robot_sel", robot_sel) || robot_sel.size() != 1) {
      LOG_ERROR("[RunActSocketInferenceOnce] robot_sel must select exactly one model");
      return BT::NodeStatus::FAILURE;
    }

    const int model_count = manager->get_model_size();
    if (model_count <= 0 || robot_sel.front() < 0 || robot_sel.front() >= model_count) {
      LOG_ERROR("[RunActSocketInferenceOnce] invalid model index: {} (model count: {})",
                robot_sel.front(), model_count);
      return BT::NodeStatus::FAILURE;
    }

    bt_manager_cmd::CmdDynBuilder builder;
    builder.init_models(static_cast<std::size_t>(model_count))
        .select_models(robot_sel)
        .set_event("RunActSocketInferenceOnce")
        .add_field({{"timeout_ms", timeout_ms}});
    if (!subsystem_name.empty()) builder.add_field({{"subsystem_name", subsystem_name}});
    const nlohmann::json command = builder.build();

    // 给驱动的单步等待时间留出响应传输余量。
    const auto request_timeout = std::chrono::milliseconds(timeout_ms) + std::chrono::seconds(2);
    if (!request_.start(manager->get_reqrep(), "main.handler.req.dyncmd", command, request_timeout,
                        manager->tickNotifier())) {
      LOG_ERROR("[RunActSocketInferenceOnce] request start failed: {}", request_.message());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    auto* manager = G_BT_MANAGER();
    if (!manager || manager->is_stopping() || !manager->is_conn_codeit()) {
      request_.cancel();
      return BT::NodeStatus::FAILURE;
    }
    return request_.tick("RunActSocketInferenceOnce");
  }

  void onHalted() override { request_.cancel(); }

private:
  BtJsonRequest request_;
};
