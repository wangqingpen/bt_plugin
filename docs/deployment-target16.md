# 192.168.2.16 通用行为树部署记录

已在目标机本地重新编译并部署。网关版本 v1.3.8，前端 v2.3.17，Linux ARM64。9 项 CTest 全部通过，原有 13 种节点保留，新增 RobotTask、RobotMoveJ、RobotMoveL、HrcsiSetDO、RobotWait、RobotSync、RobotAtTarget。

打开该机器的行为树网页，按 Ctrl+F5 刷新，选择 **huashu-editable.xml**。该树共有 399 个运行节点，包括 351 个可编辑动作和 37 个同步组。流程顺序和参数由树决定；改树不需要重编译插件。

## 当前实机条件

网关已连接正在运行的 Codeit，5866、5870、19091 连接已确认。部署没有重启或替换 Codeit 进程。

只读预检发现两个问题：

1. 使用华数工艺配置时，报告 `profile active work object mismatch: 0`，即 model 0 的活动工作坐标与工艺配置不一致。
2. 使用仅检查本机实时状态的独立测试时，报告 `joint disabled, faulted or invalid`，即关节状态检查未通过。此错误可能来自未使能、关节故障或无效位置/速度，当前日志没有细分到具体轴。

因此尚未完成真实 Codeit 指令收发验证。原计划的 RobotWait(0 ms) 测试在设备检查阶段被拦截，没有下发 Sleep，没有发送运动或数字输出，没有自动使能或更改坐标系。

主树目前已加载且全部 IDLE。**整套华数工艺尚不具备直接启动条件**：需要现场核对工作坐标标定与点位参考系，并使关节状态满足运行要求。不能通过删除预检条件来消除这两个问题。

## 本机路径

| 内容 | 位置 |
|---|---|
| 当前运行 XML | `/home/test/Desktop/rpc_gateway/current_version/bt_trees/huashu-editable.xml` |
| 设备配置与点位库 | 同目录 `huashu.robot.json`、`huashu.points.json` |
| 当前插件 | `/home/test/Desktop/rpc_gateway/current_version/plugins/libBehaviorTreePlugin.so` |
| 完整源码 | `/home/test/Desktop/generic-robot-runtime-20261114/bt_plugin` |
| 构建目录 | `/home/test/Desktop/generic-robot-runtime-20261114/build` |
| 交付目录、日志与预检记录 | `/home/test/Desktop/generic-robot-runtime-20261114` |
| 替换前备份 | `/home/test/Desktop/generic-robot-runtime-20261114/backup/target16-20261114-140530` |
| 目前实际运行的 Codeit | `/home/test/Public/jww/run7/codeit-deploy` |

原有树文件和网关连接配置均保留。前端采用新文件哈希更新入口，刷新后加载修正过的编辑器。超过 64 KiB 的树保存及重新加载已实测通过。

## 重新构建

SDK 仅放在交付目录中，没有改写系统开发环境：

```bash
cd /home/test/Desktop/generic-robot-runtime-20261114
cmake -S bt_plugin -B build -DCMAKE_BUILD_TYPE=Release \
  -DBT_PLUGIN_BUILD_TESTS=ON \
  -DRPC_GATEWAY_PLUGIN_OUTPUT_DIR="$PWD/artifacts" \
  -Dxplugin-dev_DIR="$PWD/sdk/lib/cmake/xplugin-dev"
cmake --build build -j3
ctest --test-dir build --output-on-failure
```

`deployment16.json` 记录安装文件哈希、备份和进程；`tests16.log` 为本机测试结果；`preflight16.json`、`communication16.json` 记录两次只读预检及拦截原因。`deploy16.py` 是此次受版本检查保护的迁移脚本，不是日常运行树时需要执行的脚本。
