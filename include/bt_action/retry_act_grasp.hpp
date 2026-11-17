#pragma once

#include <behaviortree_cpp/control_node.h>
#include "bt_manager.hpp"

// Child 0: prepare home/open gripper and run ACT. Child 1: return home after
// a confirmed, locally stopped GraspFailed. Other failures always escape.
class RetryActGrasp final : public BT::ControlNode {
public:
  RetryActGrasp(const std::string& name, const BT::NodeConfig& config)
      : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("max_attempts", -1, "Maximum ACT attempts; -1 retries grasp failures indefinitely"),
      BT::BidirectionalPort<std::string>("result", "Share with the ACT result output"),
      BT::OutputPort<std::string>("phase"),
    };
  }

  BT::NodeStatus tick() override {
    if (childrenCount() != 2) throw BT::LogicError("RetryActGrasp requires attempt and recovery children");
    if (status() == BT::NodeStatus::IDLE) {
      if (!getInput("max_attempts", max_attempts_) || max_attempts_ == 0 || max_attempts_ < -1)
        return failed("invalid_attempt_limit");
      attempts_ = 1;
      recovering_ = false;
      setOutput("result", std::string("running"));
      setOutput("phase", std::string("attempting"));
    }
    setStatus(BT::NodeStatus::RUNNING);
    auto* manager = G_BT_MANAGER();
    if (!manager) return failed("fault", "BT manager unavailable");
    if (manager->is_stopping()) return failed("fault", "BT manager stopping");
    if (!manager->is_conn_codeit()) return failed("fault", "Codeit disconnected");
    if (manager->get_gateway_status() == 2) return failed("fault", "gateway status=2");

    if (!recovering_) {
      const auto result = children_nodes_[0]->executeTick();
      if (result == BT::NodeStatus::RUNNING) return result;
      if (result == BT::NodeStatus::SUCCESS) {
        resetChildren();
        setOutput("phase", std::string("done"));
        return BT::NodeStatus::SUCCESS;
      }
      std::string reason;
      if (result != BT::NodeStatus::FAILURE || !getInput("result", reason) || reason != "grasp_failed")
        return failed("fault", "attempt ended without confirmed grasp_failed; result=" + reason);
      haltChild(0);
      recovering_ = true;
      LOG_WARN("[RetryActGrasp] attempt={} grasp_failed; returning to pick_home", attempts_);
      setOutput("phase", std::string("recovering"));
    }

    const auto recovered = children_nodes_[1]->executeTick();
    if (recovered == BT::NodeStatus::RUNNING) return recovered;
    if (recovered != BT::NodeStatus::SUCCESS) return failed("recovery_failed");
    haltChild(1);
    if (max_attempts_ != -1 && attempts_ >= max_attempts_) return failed("retry_exhausted");
    ++attempts_;
    LOG_INFO("[RetryActGrasp] home confirmed; preparing attempt={}", attempts_);
    recovering_ = false;
    // Clear the previous recoverable result before home/open prechecks. A fault
    // there must never inherit GraspFailed and authorize another recovery.
    setOutput("result", std::string("running"));
    setOutput("phase", std::string("attempting"));
    manager->tickNotifier()();
    return BT::NodeStatus::RUNNING;
  }

  void halt() override {
    BT::ControlNode::halt();
    recovering_ = false;
    attempts_ = 0;
    setOutput("result", std::string("canceled"));
    setOutput("phase", std::string("halted"));
  }

private:
  BT::NodeStatus failed(const std::string& reason, const std::string& detail = {}) {
    LOG_ERROR("[RetryActGrasp] stopped: reason={} attempt={} detail={}", reason, attempts_, detail);
    resetChildren();
    setOutput("result", reason);
    setOutput("phase", std::string("failed"));
    return BT::NodeStatus::FAILURE;
  }
  int max_attempts_{-1};
  std::uint64_t attempts_{0};
  bool recovering_{false};
};
