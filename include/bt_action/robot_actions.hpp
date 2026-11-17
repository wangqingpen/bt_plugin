#pragma once
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <memory>
#include <string>
#include <vector>

namespace robot_bt {
struct Context;
class Runner;
enum class Kind { Joint, Line, Dout, Wait };
struct Command {
  Kind kind{Kind::Wait};
  int model{}, output{}, value{}, msec{}, timeout_ms{100000000};
  unsigned node_uid{};
  std::string node_name;
  std::string speed{"v5"};
  std::string target_name;
  std::vector<double> target_values;
  double ratio{4}, zone_position{}, zone_orientation{};
  bool optimal{};
};
struct Frame { std::string command; };
// Pure driver serialization. Topology and port values are supplied by the tree.
std::vector<Frame> compileBatch(const std::vector<Command>& commands, int model_count);
}

class RobotTask final : public BT::DecoratorNode {
public:
  using BT::DecoratorNode::DecoratorNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;
private:
  std::shared_ptr<robot_bt::Context> context_;
};

class RobotCommand : public BT::StatefulActionNode {
public:
  RobotCommand(const std::string&, const BT::NodeConfig&, robot_bt::Kind);
  ~RobotCommand() override;
  robot_bt::Command prepare();
  void joinBatch(const std::shared_ptr<robot_bt::Runner>&);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList commonPorts();
  static BT::PortsList motionPorts();
private:
  robot_bt::Kind kind_;
  std::shared_ptr<robot_bt::Runner> runner_;
  bool batched_{};
};

#define ROBOT_ACTION_CLASS(Type, KindValue) \
class Type final : public RobotCommand { \
public: \
  Type(const std::string& n,const BT::NodeConfig& c):RobotCommand(n,c,robot_bt::Kind::KindValue){} \
  static BT::PortsList providedPorts(); \
};
ROBOT_ACTION_CLASS(RobotMoveJ, Joint)
ROBOT_ACTION_CLASS(RobotMoveL, Line)
ROBOT_ACTION_CLASS(HrcsiSetDO, Dout)
ROBOT_ACTION_CLASS(RobotWait, Wait)
#undef ROBOT_ACTION_CLASS

class RobotSync final : public BT::ControlNode {
public:
  using BT::ControlNode::ControlNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;
private:
  std::shared_ptr<robot_bt::Runner> runner_;
};

class RobotAtTarget final : public BT::ConditionNode {
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
