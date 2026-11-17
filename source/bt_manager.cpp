

#include "bt_manager.hpp"

#include <fstream>

#include "auto_registration.hpp"
// 注册所有行为树节点
#include "bt_action/jog_joint.hpp"
#include "bt_action/jog_joint_to_angle.hpp"
#include "bt_action/move_absj.hpp"
#include "bt_action/move_absj_joint_target.hpp"
#include "bt_action/mobile_robot_modbus_tcp_jgbot.hpp"
#include "bt_action/run_act_socket_inference_once.hpp"
#include "bt_action/run_act_until_gripper_done.hpp"
#include "bt_action/retry_act_grasp.hpp"
#include "bt_action/set_act_socket_inference.hpp"
// 注册所有行为树条件
#include "bt_condition/is_enable.hpp"
#include "bt_condition/is_system_running.hpp"
#include "tinyxml2.h"
#include <behaviortree_cpp/xml_parsing.h>

constexpr auto CONFIG_PATH_BT_NODES = "../bt_trees/bt_nodes.xml";

bool BTManager::init() {
  stopping_.store(false, std::memory_order_release);

  blackboard_ = BT::Blackboard::create();
  // blackboard_->set<int>("gateway_status", 0);  // 默认空闲

  std::ofstream xmlFile(CONFIG_PATH_BT_NODES);
  if (!xmlFile) {
    LOG_ERROR("[{}] ❌ 无法打开配置文件进行保存: {}", TAG, CONFIG_PATH_BT_NODES);
    return false;
  }

  try {
    xmlFile << dump_nodes_xml();
  } catch (const std::exception& e) {
    LOG_ERROR("[{}] ❌ 写入行为树节点时发生异常: {}", TAG, e.what());
    return false;
  }

  LOG_INFO("[{}] ✅ 行为树节点已生成: {}", TAG, CONFIG_PATH_BT_NODES);
  return true;
}

void BTManager::setMonitorCallbacks(MonitorTreeResetCallback tree_reset,
                                    MonitorNodeStatusCallback node_status) {
  std::lock_guard<std::mutex> lock(monitor_callbacks_mtx_);
  monitor_tree_reset_callback_ = std::move(tree_reset);
  monitor_node_status_callback_ = std::move(node_status);
}

void BTManager::clearMonitorCallbacks() noexcept {
  std::lock_guard<std::mutex> lock(monitor_callbacks_mtx_);
  monitor_tree_reset_callback_ = {};
  monitor_node_status_callback_ = {};
}

BTManager::OperationResult BTManager::loadTree(const std::string& xml) {
  std::lock_guard lifecycle_lock(lifecycle_mtx_);
  if (stopping_.load(std::memory_order_acquire))
    return {false, "behavior tree manager is stopping"};
  if (xml.empty()) return {false, "no behavior tree file specified"};

  // Serialize load/start/stop. After this barrier no old tick can run, and
  // parsing the replacement tree never grants permission to execute it.
  stopWorker();
  tree_path_ = xml;
  const auto result = switchTree();
  if (result.success) LOG_INFO("[BT] loaded {}; awaiting explicit start", tree_path_);
  return result;
}

BTManager::OperationResult BTManager::start(int period) {
  std::lock_guard lifecycle_lock(lifecycle_mtx_);

  if (stopping_.load(std::memory_order_acquire)) {
    return {false, "behavior tree manager is stopping"};
  }

  // Repeated start during an active run is idempotent.
  if (is_running() && !paused_.load(std::memory_order_acquire)) {
    return {true, {}};
  }
  if (tree_path_.empty()) {
    return {false, "no behavior tree has been loaded"};
  }
  stopWorker();
  bool rebuild;
  {
    std::lock_guard tree_lock(tree_mtx_);
    rebuild = !tree_.rootNode() || completed_;
  }
  // An explicit new start resets RunOnce and other cached terminal states.
  // A manually paused, incomplete tree keeps its pending request when resumed.
  if (rebuild) {
    const auto result = switchTree();
    if (!result.success) return result;
  }

  tick_period_ = std::chrono::milliseconds(period > 0 ? period : 50);
  execution_armed_.store(true, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  paused_.store(false, std::memory_order_release);

  try {
    worker_ = std::thread([this] { BTManager::loop(); });
  } catch (const std::exception& e) {
    running_.store(false, std::memory_order_release);
    execution_armed_.store(false, std::memory_order_release);
    paused_.store(true, std::memory_order_release);

    // 线程创建失败时清理树和 Publisher；保留路径供下一次显式 start() 重建。
    {
      std::lock_guard<std::mutex> tree_lock(tree_mtx_);
      if (tree_.rootNode()) tree_.haltTree();
      reset();
    }
    return {false, std::string("failed to create behavior tree thread: ") + e.what()};
  } catch (...) {
    running_.store(false, std::memory_order_release);
    execution_armed_.store(false, std::memory_order_release);
    paused_.store(true, std::memory_order_release);
    {
      std::lock_guard<std::mutex> tree_lock(tree_mtx_);
      if (tree_.rootNode()) tree_.haltTree();
      reset();
    }
    return {false, "failed to create behavior tree thread: unknown error"};
  }

  LOG_INFO("[BT] explicit start accepted: file={} period_ms={}", tree_path_, tick_period_.count());
  return {true, {}};
}

std::function<void()> BTManager::tickNotifier() const {
  return [weak = std::weak_ptr<TickSignal>(tick_signal_)] {
    if (auto signal = weak.lock()) {
      {
        std::lock_guard<std::mutex> lock(signal->mutex);
        ++signal->generation;
      }
      signal->cv.notify_all();
    }
  };
}

void BTManager::pause() {
  paused_ = true;
  tickNotifier()();
}

void BTManager::resume() {
  if (!execution_armed_.load(std::memory_order_acquire)) {
    LOG_INFO("[BT] resume ignored; awaiting explicit start");
    return;
  }
  paused_ = false;
  tickNotifier()();
}

BT::NodeStatus BTManager::step() {
  pause();
  std::lock_guard<std::mutex> lk(tree_mtx_);
  if (!execution_armed_.load(std::memory_order_acquire) || !tree_.rootNode())
    return BT::NodeStatus::IDLE;
  return tickTree(true);
}

void BTManager::stopWorker() {
  execution_armed_.store(false, std::memory_order_release);
  paused_.store(true, std::memory_order_release);
  running_.store(false, std::memory_order_release);
  tickNotifier()();
  if (worker_.joinable()) worker_.join();
}

void BTManager::stop() {
  std::lock_guard lifecycle_lock(lifecycle_mtx_);
  stopWorker();

  std::lock_guard<std::mutex> lk(tree_mtx_);
  if (tree_.rootNode()) tree_.haltTree();

  reset();
}

void BTManager::finishRun(BT::NodeStatus status) {
  if (status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE) return;
  completed_ = true;
  execution_armed_.store(false, std::memory_order_release);
  paused_.store(true, std::memory_order_release);
  LOG_INFO("[BT] execution finished: status={}; awaiting explicit start",
           status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
}

BT::NodeStatus BTManager::tickTree(bool exactly_once) {
  try {
    const auto status = exactly_once ? tree_.tickExactlyOnce() : tree_.tickOnce();
    finishRun(status);
    return status;
  } catch (const std::exception& e) {
    LOG_ERROR("[BT] tick failed: {}", e.what());
  } catch (...) {
    LOG_ERROR("[BT] tick failed with unknown error");
  }

  // An exceptional node must also revoke execution permission. Otherwise the
  // worker terminates the host process, or a later step retries the broken run.
  finishRun(BT::NodeStatus::FAILURE);
  try {
    tree_.haltTree();
  } catch (const std::exception& e) {
    LOG_ERROR("[BT] halt after tick failure failed: {}", e.what());
  } catch (...) {
    LOG_ERROR("[BT] halt after tick failure failed with unknown error");
  }
  return BT::NodeStatus::FAILURE;
}

std::string BTManager::dump_nodes_xml() {
  using namespace tinyxml2;

  XMLDocument doc;

  // <root>
  auto* root = doc.NewElement("root");
  root->SetAttribute("BTCPP_format", "4");
  doc.InsertEndChild(root);

  auto* model = doc.NewElement("TreeNodesModel");
  root->InsertEndChild(model);

  for (const auto& node : NodeRegistry::instance().getAllMeta()) {
    // Action / Condition / Decorator / Control
    auto* node_elem = doc.NewElement(node.type.c_str());
    node_elem->SetAttribute("ID", node.name.c_str());

    for (const auto& p : node.ports) {
      const char* tag = nullptr;

      if (p.direction == "Input") {
        tag = "input_port";
      } else if (p.direction == "Output") {
        tag = "output_port";
      } else {
        tag = "inout_port";
      }

      auto* port = doc.NewElement(tag);
      port->SetAttribute("name", p.name.c_str());
      port->SetAttribute("type", p.type.c_str());

      if (!p.default_value.empty()) {
        port->SetAttribute("default", p.default_value.c_str());
      }

      if (!p.description.empty()) {
        port->SetAttribute("description", p.description.c_str());
      }

      node_elem->InsertEndChild(port);
    }

    model->InsertEndChild(node_elem);
  }

  XMLPrinter printer;
  doc.Print(&printer);

  return printer.CStr();
}

bool BTManager::is_running() const {
  return running_.load(std::memory_order_acquire) && execution_armed_.load(std::memory_order_acquire);
}

void BTManager::set_model_size(const int& sz) { model_size.store(sz, std::memory_order_relaxed); }

int BTManager::get_model_size() const { return model_size.load(std::memory_order_relaxed); }

void BTManager::set_gateway_status(const int& st) {
  gateway_status.store(st, std::memory_order_relaxed);
  if (st == 2 && is_running()) stop();
}

int BTManager::get_gateway_status() const { return gateway_status.load(std::memory_order_relaxed); }

void BTManager::set_idle_stable_count(int v) {
  idle_stable_count.store(v, std::memory_order_relaxed);
}

int BTManager::inc_idle_stable_count() {
  return idle_stable_count.fetch_add(1, std::memory_order_relaxed) + 1;
}

bool BTManager::update_rt(const void* data, size_t size) {
  overall_system_rtstate::SystemRtState updated;
  if (!updated.ParseFromArray(data, size)) return false;

  const int model_count = updated.model_size();
  {
    std::lock_guard state_lock(state_mtx_);
    const auto now = std::chrono::steady_clock::now();
    if (updated.has_head()) {
      if (rt_msg.has_head() && (updated.head().frame_id() < rt_msg.head().frame_id() ||
                               updated.head().timestamp() < rt_msg.head().timestamp()))
        ++rt_regression_count_;
      if (!rt_msg.has_head() || updated.head().frame_id() != rt_msg.head().frame_id() ||
          updated.head().timestamp() != rt_msg.head().timestamp()) rt_advanced_at_ = now;
    } else rt_advanced_at_ = {};
    rt_msg = std::move(updated);
    rt_received_at_ = now;
  }
  set_model_size(model_count);
  return true;
}

bool BTManager::update_nrt(const void* data, size_t size) {
  overall_system_nrtstate::SystemNrtState updated;
  if (!updated.ParseFromArray(data, size)) return false;

  std::lock_guard state_lock(state_mtx_);
  const auto now = std::chrono::steady_clock::now();
  if (updated.has_head()) {
    if (nrt_msg.has_head() && (updated.head().frame_id() < nrt_msg.head().frame_id() ||
                             updated.head().timestamp() < nrt_msg.head().timestamp()))
      ++nrt_regression_count_;
    if (!nrt_msg.has_head() || updated.head().frame_id() != nrt_msg.head().frame_id() ||
        updated.head().timestamp() != nrt_msg.head().timestamp()) nrt_advanced_at_ = now;
  } else nrt_advanced_at_ = {};
  nrt_msg = std::move(updated);
  nrt_received_at_ = now;
  return true;
}

overall_system_rtstate::SystemRtState BTManager::get_rt() const {
  std::lock_guard state_lock(state_mtx_);
  return rt_msg;
}

overall_system_nrtstate::SystemNrtState BTManager::get_nrt() const {
  std::lock_guard state_lock(state_mtx_);
  return nrt_msg;
}

BTManager::NrtSnapshot BTManager::get_nrt_snapshot() const {
  std::lock_guard state_lock(state_mtx_);
  return {nrt_msg, nrt_received_at_, nrt_advanced_at_, nrt_regression_count_};
}

BTManager::RtSnapshot BTManager::get_rt_snapshot() const {
  std::lock_guard state_lock(state_mtx_);
  return {rt_msg, rt_received_at_, rt_advanced_at_, rt_regression_count_};
}

void BTManager::loop() {
  const auto signal = tick_signal_;
  std::unique_lock<std::mutex> lk(signal->mutex);

  while (running_) {
    signal->cv.wait(lk, [this] { return !running_ || (execution_armed_ && !paused_); });
    if (!running_) break;

    // 在 tick 前记录通知序号，避免丢失 tick 期间（包括同步完成）的通知。
    const auto generation = signal->generation;
    lk.unlock();
    {
      std::lock_guard<std::mutex> tree_lock(tree_mtx_);
      if (running_ && execution_armed_ && !paused_ && tree_.rootNode()) {
        tickTree(false);
      }
    }
    lk.lock();

    signal->cv.wait_for(lk, tick_period_, [this, &signal, generation] {
      return paused_ || !execution_armed_ || !running_ || signal->generation != generation;
    });
  }
}

BTManager::OperationResult BTManager::switchTree() {
  std::lock_guard<std::mutex> lk(tree_mtx_);

  try {
    if (tree_.rootNode()) tree_.haltTree();

    observer_.reset();
    publisher_.reset();

    if (!factory_) {
      factory_ = std::make_unique<BT::BehaviorTreeFactory>();
      // registerNodes(*factory_);
      NodeRegistry::instance().registerAll(*factory_);
    }

    tree_ = factory_->createTreeFromFile(tree_path_, blackboard_);
    completed_ = false;

    // Isolated tests/embedders may use callbacks without opening a Groot socket.
    if (enable_groot_) publisher_ = std::make_unique<BT::Groot2Publisher>(tree_);

    observer_ = std::make_unique<StatusObserver>(
        tree_,
        [this](const auto& node, auto prev, auto curr) { onNodeStatusChanged(node, prev, curr); });

    onTreeReset();

    LOG_DEBUG("switchTree()");
    return {true, {}};

  } catch (const std::exception& e) {
    LOG_ERROR("[BT] load failed: {}", e.what());

    // createTreeFromFile() 可能在部分成员已创建后抛出异常。失败时统一清理，
    // 确保 start() 返回失败后管理器中不会留下半初始化的树或 Publisher。
    if (tree_.rootNode()) tree_.haltTree();
    reset();
    return {false, e.what()};
  } catch (...) {
    LOG_ERROR("[BT] load failed with unknown error");
    if (tree_.rootNode()) tree_.haltTree();
    reset();
    return {false, "unknown behavior tree loading error"};
  }
}

void BTManager::onNodeStatusChanged(const BT::TreeNode& node, BT::NodeStatus prev,
                                    BT::NodeStatus curr) {
  // StatusObserver 在 tick 线程内同步调用本函数。监控回调只将
  // UID、新状态和时间戳放入队列，不在此处发送 WebSocket。
  // 当节点回到 IDLE 时，使用 10 + prev 编码。这与
  // Groot2Publisher 原状态缓冲区的 11~14 编码保持一致，
  // 前端无需因从轮询切换为事件驱动而修改状态解析。
  std::uint8_t status_code = static_cast<std::uint8_t>(curr);
  if (curr == BT::NodeStatus::IDLE) {
    status_code = static_cast<std::uint8_t>(10 + static_cast<int>(prev));
  }

  std::lock_guard<std::mutex> lock(monitor_callbacks_mtx_);
  if (monitor_node_status_callback_) {
    try {
      monitor_node_status_callback_(node.UID(), status_code, ts_now_ms());
    } catch (const std::exception& e) {
      // 监控是附加功能；投递失败不能中断行为树 tick。
      LOG_ERROR("[BT] monitor status callback failed: {}", e.what());
    } catch (...) {
      LOG_ERROR("[BT] monitor status callback failed with unknown error");
    }
  }
}

void BTManager::onTreeReset() {
  LOG_INFO("[BT] tree reset");

  // WriteTreeToXML(..., true, true) 与 Groot2Publisher 使用相同的生成方式，
  // 会在 XML 中附加 _uid/_fullPath。前端正是通过这个 UID
  // 把后续节点状态事件匹配到正确的树节点。
  try {
    const std::string tree_xml = BT::WriteTreeToXML(tree_, true, true);

    std::lock_guard<std::mutex> lock(monitor_callbacks_mtx_);
    if (monitor_tree_reset_callback_) monitor_tree_reset_callback_(tree_xml);
  } catch (const std::exception& e) {
    // 树已经创建成功时，不能因为监控快照生成失败而让
    // switchTree() 判定整棵行为树加载失败。
    LOG_ERROR("[BT] monitor tree reset callback failed: {}", e.what());
  } catch (...) {
    LOG_ERROR("[BT] monitor tree reset callback failed with unknown error");
  }
}

void BTManager::registerNodes(BT::BehaviorTreeFactory& f) {
  f.registerSimpleAction("SayHello", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
}

void BTManager::reset() {
  observer_.reset();
  publisher_.reset();
  factory_.reset();
  tree_ = {};
  completed_ = false;
}

// bt action
REGISTER_NODE(JogJoint, "JogJoint");
REGISTER_NODE(JogJointToAngle, "JogJointToAngle");
REGISTER_NODE(MoveAbsJ, "MoveAbsJ");
REGISTER_NODE(MoveAbsJJointTarget, "MoveAbsJJointTarget");
REGISTER_NODE(MobileRobotModbusTCPJGBOT, "MobileRobotModbusTCPJGBOT");
REGISTER_NODE(SetActSocketInference, "SetActSocketInference");
REGISTER_NODE(RunActSocketInferenceOnce, "RunActSocketInferenceOnce");
REGISTER_NODE(RunActUntilGripperDone, "RunActUntilGripperDone");
REGISTER_NODE(RetryActGrasp, "RetryActGrasp");
REGISTER_NODE(RunPi05UntilGripperDone, "RunPi05UntilGripperDone");
using RetryPi05Grasp = RetryActGrasp;
REGISTER_NODE(RetryPi05Grasp, "RetryPi05Grasp");

// bt condition
REGISTER_NODE(IsSystemRunning, "IsSystemRunning");
REGISTER_NODE(IsEnable, "IsEnable");
