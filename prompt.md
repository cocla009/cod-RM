# codex-autoresearch 任务契约：Armor Solver 轨迹预测与火控参数优化

## 0. 当前状态与启动规则

本任务只负责定义长期自动优化计划，不在确认前启动优化循环。

项目目录：

```text
/home/cola009/cod/cod_fyt2024_vision
```

所有 Python 命令必须先激活以下环境：

```text
/home/cola009/cod/cod_fyt2024_vision/venv
```

在开始任何循环前，必须由任务发起者确认本文件中的优化方向、验证命令和指标定义。未确认前只能进行只读扫描、基线复现和验证器完整性检查，不得自动修改参数或提交候选结果。

## 1. 扫描结论与唯一优化方向

本轮只优化一个方向：

> 基于 `sp_vision_25` 的“目标轨迹—射击时刻—云台跟随—开火窗口”思想，自动调优当前 `cod_fyt2024_vision` 中 Armor Solver 的未来预测、装甲板选板、中心跟踪切换和火控容差参数。

当前项目已经具备以下基础能力，不在本轮重复实现：

- 11 状态目标模型和球面观测 EKF；
- 弹丸飞行时间迭代和预测后最近装甲板选择；
- 低速最近板锁定；
- 高速 coming/leaving 非对称选板；
- 中心跟踪与中心跟踪禁火；
- yaw/pitch 机械偏置和高速 yaw 的 pitch 补偿；
- 装甲板几何、火控角度容差和目标丢失安全逻辑。

本轮只搜索这些已有机制的参数组合，优先评估在高速旋转、短时遮挡和目标距离变化时的稳定性。第一阶段不引入 TinyMPC、QP 求解器、新线程模型、新 ROS 消息或新的第三方依赖。

## 2. 主指标、方向和基线

### 2.1 主指标

主指标为离线四场景综合分数 `score`，越低越好：

```text
scenario_score = p95_error_deg
               + 2 * mean_error_deg
               + 20 * false_fire_rate
               + 5 * (1 - fire_recall)

score = mean(scenario_score over all scenarios)
```

其中：

- `p95_error_deg`：目标真实最佳装甲板方向与模拟云台方向的 95 分位角度误差，单位为度；
- `false_fire_rate`：模拟开火但真实角度不满足安全容差，或处于遮挡状态时的比例；
- `fire_recall`：非中心跟踪帧中满足真实角度容差的有效开火比例；
- 高速近距离中心跟踪被视为安全行为，不因禁火被扣分；
- `worst_p95_step_ms`：单个仿真帧评估耗时的 95 分位，仅作为实时性预筛选指标。

方向：最小化 `score`，同时必须满足所有硬门槛。

### 2.2 固定验证命令

基线和每次候选评估必须使用同一条主验证命令；不得通过修改验证器、场景、权重或输出格式来提高分数。

```bash
cd /home/cola009/cod/cod_fyt2024_vision
source /home/cola009/cod/cod_fyt2024_vision/venv/bin/activate
python tools/autoresearch/validate_solver.py \
  --config rm_bringup/config/node_params/armor_solver_params.yaml \
  --frames 600 \
  --dt 0.01
```

该命令只使用 Python 标准库，覆盖以下确定性场景：

1. `static_target`：静止目标；
2. `low_speed_spin`：低速旋转目标；
3. `high_speed_spin`：高速近距离旋转目标，验证中心跟踪禁火；
4. `occluded_fast_target`：高速旋转并带周期性短时遮挡和目标年龄延迟。

验证器固定文件：

```text
tools/autoresearch/validate_solver.py
```

当前基线验证器 SHA-256：

```text
b399b1710ca17bc7412f070ecd83e7fe3b4bcc50ec78740bb0fdc1c241468ca3
```

### 2.3 已运行的基线值

在当前参数文件上运行上述命令得到：

```json
{
  "score": 12.662609755338032,
  "worst_p95_error_deg": 10.408429088152628,
  "worst_false_fire_rate": 0.0,
  "worst_p95_step_ms": 0.006933
}
```

其中主指标 `score`、角度误差和开火指标是确定性的。宿主机微秒级计时会受系统调度影响，重复运行观测到的 `worst_p95_step_ms` 范围约为 `0.0066–0.0089 ms`；该字段只用于检查是否超过 50 ms 硬门槛，不用于比较微小性能差异。

场景级基线：

```text
static_target:
  score=0.02699284801323734
  p95_error_deg=0.008160300202327154
  fire_recall=0.9983333333333333

low_speed_spin:
  score=13.257720308458378
  p95_error_deg=5.7557543232659345
  fire_recall=0.43

high_speed_spin:
  score=23.446994567715258
  p95_error_deg=10.408429088152628
  center_tracking_ratio=1.0
  fire_recall=1.0

occluded_fast_target:
  score=13.918731297165252
  p95_error_deg=3.4264880309678487
  fire_recall=0.0
```

当前代码的 ROS 测试基线也必须保持：

```text
rm_utils：4/4 CTest 项通过
armor_solver：6/6 CTest 项通过
```

## 3. 目标值与停止条件

### 3.1 目标值

一个候选版本只有同时满足以下条件才允许接受：

- `score <= 9.5`，相对基线至少改善约 25%；
- `worst_p95_error_deg <= 10.408429088152628`，不得恶化最差角度误差；
- `worst_false_fire_rate <= 0.01`；
- `static_target.fire_recall >= 0.98`；
- `low_speed_spin.fire_recall >= 0.40`；
- `high_speed_spin.center_tracking_ratio >= 0.95`，中心跟踪期间不得开火；
- `worst_p95_step_ms <= 50.0`；
- 完整 ROS 构建成功，`rm_utils` 和 `armor_solver` 测试全部通过。

`worst_p95_step_ms` 是宿主机离线评估的预筛选门槛，不能替代嵌入式实测。最终部署前必须在目标嵌入式设备上对同一版本进行端到端时间戳测试，单帧从检测结果进入解算器到云台命令发布的 95 分位延迟不得超过 50 ms。

### 3.2 停止条件

满足下列任意条件时停止循环并保留当前最佳版本：

- 找到满足全部硬门槛且 `score <= 9.5` 的版本，并连续 20 次候选没有进一步改善；
- 连续 30 次候选未改善主指标；
- 累计 200 次有效候选；
- 达到 12 小时运行时长；
- 验证器、构建环境或 `optim-agent` 不可用；
- 出现任何无法解释的数值发散、非法开火或测试回归。

## 4. 参数提议协议

本轮涉及可调参数，因此参数提议必须使用 `optim-agent`，并且 backend 必须为 `"codex"`。`optim-agent` 只负责提出参数值，不负责判定实验是否成功；最终判定只能来自固定验证命令和 ROS 测试。

开始循环前必须确认：

```bash
cd /home/cola009/cod/cod_fyt2024_vision
source /home/cola009/cod/cod_fyt2024_vision/venv/bin/activate
optim-agent --help
```

具体子命令以已安装版本的 `--help` 为准，但每次调用都必须等价于以下请求，并在日志中记录完整请求：

```json
{
  "backend": "codex",
  "objective": "minimize score under all safety and latency gates",
  "parameters": {
    "prediction_delay": [0.0, 0.15],
    "controller_delay": [0.0, 0.10],
    "max_tracking_v_yaw": [3.0, 8.0],
    "center_tracking_distance": [0.8, 3.0],
    "min_switching_v_yaw": [0.3, 3.0],
    "coming_angle": [25.0, 80.0],
    "leaving_angle": [5.0, 45.0],
    "fire_margin": [0.3, 1.5],
    "min_fire_tolerance": [0.3, 2.0],
    "max_fire_tolerance": [2.0, 8.0]
  },
  "constraints": [
    "leaving_angle < coming_angle",
    "max_fire_tolerance >= min_fire_tolerance",
    "do not change bullet_speed, armor geometry, safety gates, or validator"
  ]
}
```

禁止手工绕过 `optim-agent` 直接修改上述参数进行候选搜索。若 `optim-agent` 不存在或无法使用 backend `codex`，立即停止，不得退化为随机搜索、网格搜索或模型自行猜参数。

## 5. 完整执行步骤

### 阶段 A：冻结和基线复现

1. 激活指定 venv；
2. 校验 `validate_solver.py` SHA-256；
3. 运行固定验证命令，确认主指标与本文件基线一致；
4. 使用 ROS 2 Humble 构建并运行完整测试；
5. 将基线写入实验日志；
6. 未完成以上步骤不得进入参数循环。

### 阶段 B：参数候选循环

每个 trial 严格执行以下顺序：

1. 调用 `optim-agent`，明确传入 `backend="codex"`；
2. 只将提议值写入临时参数副本，不直接污染基线配置；
3. 检查范围、单位和参数间约束；
4. 运行固定离线验证命令；
5. 若任一硬门槛失败，立即拒绝候选；
6. 若离线指标通过，再运行：

   ```bash
   cd /home/cola009/cod/cod_fyt2024_vision
   source /home/cola009/cod/cod_fyt2024_vision/venv/bin/activate
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-up-to armor_solver --cmake-args -DBUILD_TESTING=ON
   colcon test --packages-up-to armor_solver --event-handlers console_direct+ --return-code-on-test-failure
   ```

7. 只有指标改善、测试全通过且没有安全回归时，才接受候选；
8. 接受候选后创建一次中文、详细的 Git 提交；拒绝候选不得提交；
9. 将完整结果写入 JSONL 实验日志。

### 阶段 C：代码级增量优化（可选）

只有参数搜索达到停止条件、且仍未达到目标时，才允许进行代码优化。代码优化必须服务于同一轨迹预测和火控方向，例如：

- 减少重复的装甲板轨迹计算；
- 改善角度环绕和装甲板切换的数值连续性；
- 在不改变火控安全语义的情况下减少动态分配；
- 将未来轨迹和飞行时间迭代封装为可测试的纯算法函数。

每次代码改动仍必须经过固定验证器、完整构建、完整测试和代码 review。不得借代码重构机会改变评分函数、场景或安全门槛。

### 阶段 D：嵌入式实时性复核

最终候选必须在实际嵌入式设备上测量：

- 检测结果接收时间；
- Tracker/EKF 完成时间；
- Solver 完成时间；
- 云台命令发布时间；
- 单帧总延迟的平均值、95 分位和最大值。

95 分位总延迟超过 50 ms 的候选必须拒绝，即使离线 score 更低。

## 6. 工作边界

### 允许修改

- `cod_fyt2024_vision/rm_auto_aim/armor_solver/src/`
- `cod_fyt2024_vision/rm_auto_aim/armor_solver/include/`
- `cod_fyt2024_vision/rm_auto_aim/armor_solver/test/`
- `cod_fyt2024_vision/rm_auto_aim/armor_solver/README.md`
- `cod_fyt2024_vision/rm_bringup/config/node_params/armor_solver_params.yaml` 中本文件列出的 Solver 参数；
- `cod_fyt2024_vision/tools/autoresearch/` 中的验证辅助代码，但基线冻结后不得修改 `validate_solver.py`；
- `experiments/` 中的日志、临时参数副本和结果摘要；
- `prompt.md` 的勘误，但勘误必须单独记录，不能改变冻结指标。

### 禁止修改

- `rm_interfaces/msg/` 及任何 ROS 消息、话题和通信协议；
- `rm_auto_aim/armor_detector/`、相机驱动、串口驱动、rune 模块和机器人描述；
- `sp_vision_25/` 及参考源码；
- `venv/`、`build/`、`install/`、`log/` 和 `.git/`；
- 弹速、装甲板物理尺寸、目标状态定义、EKF 稳定性保护和中心跟踪禁火安全语义；
- 新增第三方依赖，尤其是未经确认的 MPC/QP 求解器；
- 通过删除、跳过或放宽测试来获得更高分数；
- 修改验证器场景、权重、阈值、随机种子或输出格式来制造指标改善。

## 7. 实验日志格式

日志文件固定为：

```text
experiments/autoresearch_log.jsonl
```

每个 trial 一行合法 JSON，不得使用无法机器解析的自由文本。格式如下：

```json
{
  "trial": 0,
  "timestamp": "2026-09-13T21:30:00+08:00",
  "commit": "当前提交哈希或 null",
  "proposal_engine": "optim-agent",
  "backend": "codex",
  "proposal": {
    "prediction_delay": 0.045,
    "controller_delay": 0.025
  },
  "validation_command": "完整命令字符串",
  "score": 12.662609755338032,
  "worst_p95_error_deg": 10.408429088152628,
  "worst_false_fire_rate": 0.0,
  "worst_p95_step_ms": 0.006933,
  "scenario_metrics": {},
  "ros_build": "pass|fail",
  "ros_tests": "10/10",
  "decision": "baseline|accepted|rejected|blocked",
  "reason": "接受或拒绝的具体原因"
}
```

若候选导致构建、测试、验证器或实时性门槛失败，也必须记录，不能静默丢弃。日志中不得记录未实际运行的指标。

## 8. 冻结目标 + 可调整执行步骤

### 冻结目标（不可调整）

1. 主指标定义、四个场景、帧数、时间步长、评分公式和基线验证器；
2. `score` 最小化方向；
3. 50 ms 嵌入式 95 分位延迟硬门槛；
4. false fire、中心跟踪禁火、静止目标稳定性和 ROS 全量测试门槛；
5. 工作边界和禁止修改文件；
6. 所有参数提议必须来自 `optim-agent`，backend 必须为 `codex`；
7. 发生验证器不一致、工具缺失或安全回归时停止，而不是自行放宽标准。

### 可调整执行步骤（可在不改变冻结目标的前提下调整）

1. 每轮调用 `optim-agent` 的候选数量和搜索顺序；
2. 参数候选的临时文件组织方式；
3. 在允许文件范围内对轨迹计算进行等价重构；
4. trial 的并行度，但不得让候选之间共享未记录状态；
5. 日志的附加诊断字段；
6. 达到停止条件前的运行时长和候选预算；
7. 在真实日志可用后增加回放场景，但必须保留原四个冻结场景和原始基线。

任何“可调整执行步骤”都不得改变冻结指标、验证命令、安全门槛或工作边界。

## 9. 当前确认点

本文件已经完成方向、验证器、实际基线和任务契约定义，但尚未启动 `optim-agent` 循环。请先确认本优化方向和 `prompt.md` 内容；确认后才允许执行阶段 A 之后的长期自动优化。
