#pragma once

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "bt_manager.hpp"
#include "bt_json_request.hpp"

// Wait for idle, submit one named MoveAbsJ, then verify its acknowledged result
// against advancing RT feedback. This node never starts or stops ACT itself.
class MoveAbsJJointTarget final : public BT::StatefulActionNode {
public:
  MoveAbsJJointTarget(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}
  ~MoveAbsJJointTarget() override { request_.cancel(); }

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using Clock = std::chrono::steady_clock;
  enum class Phase { WaitingIdle, WaitingReply, Settling };
  struct Target {
    int model;
    std::vector<double> joints;
  };

  bool resolveTargets(const BTManager::RtSnapshot& rt, const BTManager::NrtSnapshot& nrt,
                      std::vector<Target>& targets, std::string& error) const;
  bool readFeedback(const BTManager::RtSnapshot& rt, const BTManager::NrtSnapshot& nrt,
                    std::string& error);
  bool stationary(const overall_system_rtstate::SystemRtState& rt, bool at_target) const;
  BT::NodeStatus fail(const std::string& error);

  BtJsonRequest request_;
  Phase phase_{Phase::WaitingIdle};
  std::vector<int> selected_;
  std::vector<Target> targets_;
  std::vector<double> jointtarget_values_;
  bool skip_if_at_target_{false};
  std::string jointtarget_, speed_;
  std::string feedback_subsystem_;
  double physical_position_{0};
  int model_count_{0}, timeout_ms_{60000}, feedback_timeout_ms_{1000};
  int idle_stable_ms_{200}, stable_ms_{200};
  double position_tolerance_{0.01}, velocity_tolerance_{0.01};
  std::uint64_t rt_regressions_{0}, nrt_regressions_{0};
  Clock::time_point deadline_{}, stable_since_{}, target_since_{}, last_frame_at_{}, reply_frame_at_{};
  Clock::time_point last_nrt_frame_at_{};
};
