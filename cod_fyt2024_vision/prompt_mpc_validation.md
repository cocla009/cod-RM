# Armor Solver MPC 闭环验证与算法加固任务

## 1. 任务范围

当前代码基线为提交 `059ed30`。本任务专门验证和加固已经接入的云台 MPC，不重复 ALG-1、ALG-2，也不使用 `optim-agent` 做普通参数搜索。

本任务以上位机算法和上位机控制接口为设计主线。下位机当前实现不构成算法上限；如果 MPC 需要新的角度、角速度、角加速度或其他控制接口，应优先修改上位机消息和协议定义，并将下位机视为必须同步适配的执行端。不得为了迁就现有下位机接口而削弱 MPC 的状态模型、约束或控制目标。

必须先阅读：

- `rm_auto_aim/armor_solver/src/gimbal_mpc.cpp`
- `rm_auto_aim/armor_solver/include/armor_solver/gimbal_mpc.hpp`
- `rm_auto_aim/armor_solver/src/aim_reference.cpp`
- `rm_auto_aim/armor_solver/src/armor_solver.cpp`
- `rm_auto_aim/armor_solver/test/test_gimbal_mpc.cpp`
- `rm_auto_aim/armor_solver/test/test_aim_reference.cpp`
- `/home/cola009/cod/sp_vision_25/tasks/auto_aim/planner/planner.cpp`
- `/home/cola009/cod/sp_vision_25/tasks/auto_aim/planner/tinympc/`

同济代码只作为 MPC 结构、参考轨迹和约束设计的只读参考，不直接复制实现，不引入其第三方依赖。

当前已知缺口：`tools/autoresearch/validate_solver.py` 仍是独立的解析控制模拟器，没有真正调用 C++ MPC。因此本任务必须新增独立的 ROS 无关云台闭环仿真或等价纯 C++ 测试，不能仅凭现有离线分数判断 MPC 成败。

## 2. 目标

完成以下闭环算法工作：

1. 为云台建立 ROS 无关的确定性动态仿真器，至少支持角度、角速度、角加速度和 jerk 限制；
2. 将 `AimReferenceGenerator`、`GimbalMpc` 和仿真器串成可重复运行的测试；
3. 对比解析单点控制和 MPC 控制；
4. 覆盖静止目标、低速旋转、高速旋转、选板切换、短时遮挡、目标年龄、控制延迟、姿态噪声和 yaw 环绕；
5. 检查最大速度、最大加速度、最大 jerk、参考轨迹连续性和 MPC 求解状态；
6. MPC 无效、数值非有限、目标过期、参考轨迹不连续、选板跳变或跟踪丢失时必须安全降级，并强制 `fire_advice=false`；
7. 每次小修改后先运行对应纯算法测试，再进行下一步修改；
8. 最终完成完整构建、ASAN/UBSAN、所有直接 GTest、闭环仿真和固定离线验证，并提交代码。

## 3. 工作顺序

严格按以下顺序推进：

1. 只读扫描当前 MPC、参考轨迹、消息和测试；
2. 先增加最小仿真模型和测试，不修改 MPC；
3. 运行新增测试，记录基线行为；
4. 每次只修改一个明确的算法问题，例如离散模型、角度展开、约束投影、warm-start、失败降级或开火门控；
5. 每次修改后运行受影响的纯 C++ 测试和 `git diff --check`；
6. 算法稳定后运行固定离线验证器，不能修改验证器、场景、评分、随机种子或安全阈值；
7. 最后执行大测试并提交。

允许修改：

- `rm_auto_aim/armor_solver/include/`
- `rm_auto_aim/armor_solver/src/`
- `rm_auto_aim/armor_solver/test/`
- 必要的 `rm_auto_aim/armor_solver/CMakeLists.txt`
- 必要的 `rm_bringup/config/node_params/armor_solver_params.yaml`，但只能在算法实现稳定后记录经过验证的 MPC 参数；
- 必要的 `rm_interfaces/msg/GimbalCmd.msg` 及其纯接口适配，只在确实需要输出 MPC 速度/加速度时修改。
- 下位机协议和执行端适配可以随上位机接口同步修改，但不得反向限制上位机算法设计；任何新增接口都必须在 ROS 无关仿真和直接测试中先验证。

禁止：

- 使用 `optim-agent`；
- 安装 ROS、摄像头、串口、GPU 或新的第三方求解器作为前置条件；
- 修改 `tools/autoresearch/validate_solver.py` 的逻辑；
- 为了提高指标而放宽安全门槛或删除测试；
- 只做 YAML 参数搜索而不验证 MPC 闭环；
- 把 `build/`、`install/`、`log/`、`venv/` 或运行状态目录提交。

## 4. 安全与验收

固定离线验证命令：

```bash
cd /home/cola009/cod/cod_fyt2024_vision
timeout 120s venv/bin/python tools/autoresearch/validate_solver.py \
  --config rm_bringup/config/node_params/armor_solver_params.yaml \
  --frames 600 --dt 0.01
```

固定安全条件：

- `worst_false_fire_rate == 0`；
- 高速场景中心跟踪时不得开火；
- 静止目标召回率不得低于已有基线；
- MPC 约束违约必须为 0，或明确记录为失败并禁止开火；
- 所有输出必须有限，yaw 必须正确处理 `-pi/pi` 环绕；
- 求解器失败必须有有限降级路径，不能死循环。

最终大测试至少包括：

```bash
git diff --check
g++ -std=c++17 -Wall -Wextra -Werror ...
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined ...
bash --noprofile --norc -c 'source /opt/ros/humble/setup.sh && colcon build --packages-select rm_interfaces armor_solver --cmake-args -DBUILD_TESTING=ON'
ctest --test-dir build/armor_solver --output-on-failure
```

如果 ROS 的 `ctest` 包装器因环境缺少 `ament_cmake_test` 失败，必须同时运行并记录直接 GTest 二进制结果，不能把环境包装器失败误判为算法测试通过。

## 5. 持久化进度与异常退出

本任务使用独立状态文件，不能覆盖旧任务的 `experiments/phase_status.json` 和 `experiments/autoresearch_log.jsonl`：

- `experiments/mpc_status.json`
- `experiments/mpc_log.jsonl`

每个阶段记录：任务名、改动文件、测试命令、测试结果、固定离线结果、错误签名和提交哈希。不得记录未实际运行的指标。

错误规则：

- 每条命令外层超时为 120 秒；
- 同一归一化错误第 10 次出现时立即记录 `exit` 并停止；
- 环境或验证异常连续第 11 次出现时立即记录 `exit` 并停止；
- 数值发散、越界访问、非有限输出或安全回归立即停止；
- 普通测试未达预期但没有异常时，先修复或记录原因，不得无限重试。

## 6. 最终报告

最终报告必须包含：

- MPC 相对解析控制的误差、平滑性和约束对比；
- 每个场景的结果；
- 是否保持 `false_fire_rate=0`；
- 每个小修改对应的测试；
- 最终大测试命令和结果；
- 最终提交哈希；
- 未解决风险；
- 是否因异常阈值退出。

## 7. 启动命令

从仓库父目录启动新的 job，不要恢复旧的 `algorithm-v1`、`algorithm-v2` 或 `armor-solver-v2` 会话：

```bash
cd /home/cola009/cod

FROZEN_GOALS='- 完成 MPC 闭环仿真与算法加固；
  - 对比解析控制和 MPC 控制；
  - 验证约束、失败降级和开火安全；
  - 完成逐步测试、最终大测试并提交；
  - 异常达到阈值时记录 exit 并停止。'

timeout --signal=INT --kill-after=10s 120m \
  codex-autoresearch \
  --prompt-file ./cod_fyt2024_vision/prompt_mpc_validation.md \
  --frozen-goals-text "$FROZEN_GOALS" \
  --state-dir ./.codex-run-armor-solver-mpc-v1
```

不要把 `FROZEN_GOALS` 写入 Markdown；它只是在终端启动命令中使用的 shell 变量。
