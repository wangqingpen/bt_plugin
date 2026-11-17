#pragma once

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <vector>

#include "bt_json_request.hpp"
#include "bt_manager.hpp"

class SetActSocketInference final : public BT::StatefulActionNode {
public:
  SetActSocketInference(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<int>>("robot_sel", "0;1", "Robot selection"),
        BT::InputPort<int>("update_enable", 1, "Update enable: 0 or 1"),
        BT::InputPort<int>("send_enable", 1, "Send enable: 0 or 1"),
    };
  }

  BT::NodeStatus onStart() override {
    using namespace std::chrono_literals;
    auto* manager = G_BT_MANAGER();
    if (!manager || manager->is_stopping() || !manager->is_conn_codeit()) {
      return BT::NodeStatus::FAILURE;
    }

    int update_enable = 1;
    int send_enable = 1;
    if (!getInput("update_enable", update_enable) ||
        (update_enable != 0 && update_enable != 1) ||
        !getInput("send_enable", send_enable) ||
        (send_enable != 0 && send_enable != 1)) {
      LOG_ERROR("[SetActSocketInference] update_enable and send_enable must be 0 or 1");
      return BT::NodeStatus::FAILURE;
    }

    std::vector<int> robot_sel;
    if (!getInput("robot_sel", robot_sel) || robot_sel.empty()) {
      LOG_ERROR("[SetActSocketInference] robot_sel must not be empty");
      return BT::NodeStatus::FAILURE;
    }

    const int model_count = manager->get_model_size();
    if (model_count <= 0) {
      LOG_ERROR("[SetActSocketInference] robot model state is not available");
      return BT::NodeStatus::FAILURE;
    }
    for (int index : robot_sel) {
      if (index < 0 || index >= model_count) {
        LOG_ERROR("[SetActSocketInference] invalid model index: {} (model count: {})",
                  index, model_count);
        return BT::NodeStatus::FAILURE;
      }
    }

    const nlohmann::json command = bt_manager_cmd::CmdDynBuilder{}
                                       .init_models(static_cast<std::size_t>(model_count))
                                       .select_models(robot_sel)
                                       .set_event("SetActSocketInference")
                                       .add_field({{"update_enable", update_enable},
                                                   {"send_enable", send_enable}})
                                       .build();
    if (!request_.start(manager->get_reqrep(), "main.handler.req.dyncmd", command, 10s,
                        manager->tickNotifier())) {
      LOG_ERROR("[SetActSocketInference] request start failed: {}", request_.message());
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
    return request_.tick("SetActSocketInference");
  }

  void onHalted() override { request_.cancel(); }

private:
  BtJsonRequest request_;
};
