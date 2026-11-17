# 通用行为树节点：已实现并部署

当前入口为 **huashu-editable.xml**。流程由树中的连接、顺序和端口决定；改变流程不需要重编译插件，也不需要修改固定节拍表。

## 如何打开和编辑

1. 在现有行为树网页按 **Ctrl+F5** 刷新，重新读取节点库。
2. 在树文件列表选择 **huashu-editable.xml**。旧网页缓存可能仍显示旧树，需要切换文件。
3. 展开相应阶段，修改动作节点参数，增删节点或调整连接和顺序，保存后重新加载。
4. 确认现场条件后，由操作人员点击运行。部署过程只加载了树，没有启动机器人。

九个阶段已直接展开为普通 Sequence，内部有 351 个动作和 37 个同步组，总计 399 个运行节点。阶段不再是不可见的固定步骤调用。

编辑器的树结构拖放会把被拖动节点插到目标节点之后；只移动画布位置不会改变执行顺序。以保存后的树结构和连接为准。

## 节点的含义

| 节点 | 用途 | 常改参数 |
|---|---|---|
| RobotTask | 包住整个任务，读取设备配置，检查实时状态，管理故障停止 | profile、feedback_timeout_ms |
| RobotMoveJ | 单模型关节运动 | model、target、speed |
| RobotMoveL | 单模型直线运动，内部处理驱动的轨迹准备与启动协议 | model、target、speed、ratio、zone_position、zone_orientation、optimal |
| HrcsiSetDO | 华数控制器数字输出 | model、do_num、value |
| RobotWait | 指定模型等待，单位毫秒 | model、msec |
| RobotSync | 把直接子动作放入同一个多模型执行帧 | 拖入或移除子动作；每个子动作使用不同 model |
| RobotAtTarget | 检查是否已经到位，可作为 Fallback 或 IfThenElse 的条件 | model、kind、target、到位容差 |

Sequence、Fallback、IfThenElse、Repeat 等继续使用行为树原有控制节点。RobotTask 在任务根部放一个即可；它会向普通节点和参数化 SubTree 传递上下文，不用给每个动作接 session 端口。

例如，把某个 RobotMoveJ 的 target 从 JR10 改为另一个关节点、speed 从 v100 改为 v40，保存并重新加载即可。把 RobotWait 的 msec 改成 500，就会生成 500 ms 的等待指令。端口可写成 `{delay}`、`{target_name}` 等黑板绑定，每次动作重新进入时读取当时的值。

RobotSync 的直接子节点支持 RobotMoveJ、RobotMoveL、HrcsiSetDO、RobotWait，必须使用不同模型。不能把同一个模型的两个动作同时放进去；这种顺序动作应放在 Sequence 下。同步组会先验证全部子动作，再发送请求。关节与直线混合时，先准备直线轨迹，再在同一执行帧启动运动。

普通 Parallel 中的这些动作会共用请求仲裁，按可用时机逐个提交；需要驱动层同步时使用 RobotSync。

## 点位和设备配置

- `huashu-editable.xml`：只有树结构和节点参数，决定做什么、按什么顺序做。
- `huashu.robot.json`：模型自由度、工作坐标系和 IO 控制器路由，属于设备配置。
- `huashu.points.json`：纯点位库，没有步骤、跳转或节拍执行规则。

动作优先查找同一模型下唯一的实时示教点，找不到再查点位库。同名实时点的更新值可以直接使用，不再要求等于旧程序中的值。也可以设置 `target_values`，用逗号分隔数值；非空时优先于 target 名称。关节目标支持实际轴数或 10 个值，后端补足驱动的 10 槽格式；直线目标为原驱动使用的 7 个姿态数值，保留原数据顺序，不自行变换坐标。

通用节点没有固定三模型、309 节拍、66 点位、351 动作或源文件哈希要求。这些数量仅描述本次还原的初始工艺。当前设备配置仍按实际硬件设置：model 0 左臂、model 1 右臂、model 2 腰部；共享华数控制器的数字输出通过 model 0 路由。

RobotAtTarget 健康状态下未到位返回 FAILURE，允许 Fallback 执行补充动作；反馈过期、格式错误或设备故障会中止任务，不能被 Fallback 当成普通“未到位”继续发送运动。动作收到驱动成功回复后，还会等待更新的实时到位反馈；超时、失败和中断不会自动重发原动作。

## 已删除和保留的代码

已从 `/home/ai/codeit109/bt_plugin` 当前源码移除：

- `include/bt_action/huashu_step.hpp`
- `source/huashu_step.cpp`
- `tests/huashu_step_test.cpp`
- `tests/huashu_runtime.cmake`
- 旧固定树及其程序 JSON 测试夹具。

替换为 `include/bt_action/robot_actions.hpp`、`source/robot_actions.cpp`、`tests/robot_actions_test.cpp` 和 `tests/robot_actions.cmake`。删除固定执行器时一并移除了 next_beat、固定数量校验、源文件哈希锁定和隐藏步骤表编译逻辑。原有 ACT、PI0.5 等与本次问题无关的节点和已有修改保留。

`source/behavior_tree_plug.cpp` 的保存请求上限从 64 KiB 调整为 1 MiB。编辑器修复了 SubTree 引用 ID 丢失、超过十个同级节点的顺序错误、自定义控制节点类型识别，并允许显示和编辑实例名称。前端采用新的文件哈希更新入口，刷新后加载新代码。

## 验证结果和边界

- 9 项 CTest 全部通过，包含既有 ACT/PI0.5 回归和新增通用节点测试。
- 模拟总线验证了增删重排、端口和黑板更新、Repeat、参数化 SubTree、Parallel 请求仲裁、同步组整体校验、实时点位更新、新点位库、单模型运行、到位反馈、超时、故障和中断。
- 初始完整树产生 456 个多模型驱动帧，与原程序顺序及数值一致；原程序省略的速度在初始树中显式填写为驱动默认 v100。
- 编辑器实际代码函数完成全树解析—图结构—XML 往返；往返 XML 在 C++ 模拟执行中产生完全相同的指令帧。15 个同级节点重排和 SubTree 引用保留测试通过。
- Codeit 1.4.0 原生解析器接受全部 456 帧、535 条有效子指令。
- 部署后实测 134,753 字节请求保存成功，保存的树重新加载成功；7 种新节点注册，2 种旧节点消失；399 个节点全部 IDLE。

实体机器人尚未启动，未验证真实节拍、机构运动与现场抓取效果。Windows 界面自动化因工具无法确认浏览器地址而停止，因此没有宣称完成鼠标拖拽的界面验收；已完成编辑器代码函数、实际保存接口、重新加载和执行语义验证。

## 服务器位置和启动

源码：`/home/ai/codeit109/bt_plugin`

运行文件：`/home/test/Desktop/rpc_gateway/current_version/bt_trees/huashu-editable.xml` 及同目录的两个 JSON。

构建、交付和日志：`/home/ai/codeit109/generic-robot-runtime-20261114`

原版本备份：`/home/ai/codeit109/generic-robot-runtime-20261114/backup`。旧固定树及其程序 JSON 已移出当前运行树目录，保留在备份中。

重新构建与测试可执行：

```bash
cd /home/ai/codeit109/bt_plugin
cmake -S . -B build-generic -DBT_PLUGIN_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-generic -j3
ctest --test-dir build-generic --output-on-failure
```

交付包里的二进制适用于当前服务器的 Linux ARM64 环境；`deploy_runtime.py` 是本次版本迁移脚本，包含已核对的旧前端版本检查。它不是每次修改树后都需要执行的操作。

通过网页运行已加载的树，或由现场操作人员在服务器执行以下命令，启动一次任务：

```bash
python3 /home/ai/codeit109/generic-robot-runtime-20261114/start_robot_tree.py
```

运行其他已保存的通用节点树可加 `--file 文件名.xml`。此启动脚本会发送实际运行指令；本次部署没有执行它。
