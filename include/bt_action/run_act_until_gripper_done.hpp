#pragma once

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "bt_manager.hpp"
#include "bt_json_request.hpp"

// 连续 ACT 的生命周期由本节点持有。结束依据是反馈中的本轮闭合→重新张开。
class RunActUntilGripperDone : public BT::StatefulActionNode {
public:
  RunActUntilGripperDone(const std::string& name, const BT::NodeConfig& config)
      : RunActUntilGripperDone(name, config, false) {}

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  RunActUntilGripperDone(const std::string& name, const BT::NodeConfig& config, bool pi05)
      : BT::StatefulActionNode(name, config), pi05_(pi05) {}

private:
  const bool pi05_;
  using Clock = std::chrono::steady_clock;
  enum class Phase { Starting, WaitingClose, WaitingOpen, Stopping };
  enum class HealthCheck { Waiting, Healthy, Completed, GraspFailed, Stopped, Failed };
  struct ActHealth {
    std::string name;
    std::int32_t enabled{}, ready{}, send_enabled{}, last_error{};
    std::int32_t run_state{};
    std::uint64_t generation{}, send_generation{};
  };
  nlohmann::json command(bool enable) const;
  bool readActHealth(const BTManager::NrtSnapshot& snapshot, ActHealth& health,
                     std::string& error) const;
  HealthCheck checkActHealth(std::string& error);
  bool validFeedback(const BTManager::RtSnapshot& snapshot, std::string* error = nullptr) const;
  std::string diagnosticContext() const;
  BT::NodeStatus failBeforeStart(const std::string& error);
  BT::NodeStatus stopUnconfirmed(const std::string& detail);
  BT::NodeStatus stopWithResult(bool success, const std::string& error);
  void stopBounded() noexcept;

  BtJsonRequest request_;
  Phase phase_{Phase::Starting};
  int command_model_{0}, model_count_{0}, gripper_model_{-1};
  int timeout_ms_{120000}, feedback_timeout_ms_{1000}, command_timeout_ms_{2000};
  int closed_stable_ms_{200}, stable_ms_{500};
  double closed_mm_{5.0}, finish_mm_{50.0}, tolerance_mm_{2.0};
  std::string subsystem_name_;
  std::string failure_reason_;
  Clock::time_point deadline_{}, health_deadline_{};
  std::uint64_t rt_regression_baseline_{}, nrt_regression_baseline_{};
  std::uint64_t baseline_generation_{0}, active_generation_{0}, active_send_generation_{0};
  bool owns_run_{false}, stop_success_{false};
  bool health_confirmed_{false};
};

class RunPi05UntilGripperDone final : public RunActUntilGripperDone {
public:
  RunPi05UntilGripperDone(const std::string& name, const BT::NodeConfig& config)
      : RunActUntilGripperDone(name, config, true) {}
};
