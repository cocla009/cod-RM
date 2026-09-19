# 阶段 6：MPC 上位机参数优化任务契约

本阶段只在算法阶段 `ALG-1`、`ALG-2` 已验收且算法代码冻结后执行。当前验收状态见 `experiments/phase_status.json`。本阶段的目标是在固定离线验证器上寻找更低的 Solver 参数分数；不修改 C++、测试、协议、ROS 消息或验证器。

## 运行入口

执行器为仓库内的 `tools/autoresearch/run_mpc_parameter_optimization.py`，不调用 `optim-agent`，不依赖真实下位机。所有状态保存在仓库和父目录状态目录中，可中断后继续：

```bash
cd /home/cola009/cod
cd cod_fyt2024_vision
timeout --signal=INT --kill-after=10s 120m \
  venv/bin/python tools/autoresearch/run_mpc_parameter_optimization.py \
  --max-candidates 200
```

状态目录为 `../.codex-run-armor-solver-mpc-params-v1/`。恢复时直接再次执行同一命令；runner 从 `state.json`、`experiments/phase_status.json` 和 JSONL 最大 trial 继续，不重复已记录 proposal。基线 YAML 的 SHA-256、验证器 SHA-256 和最佳候选均持久化。

## 不可变验证契约

固定验证器为 `tools/autoresearch/validate_solver.py`，SHA-256 必须为 `b399b1710ca17bc7412f070ecd83e7fe3b4bcc50ec78740bb0fdc1c241468ca3`。每轮执行：

```bash
timeout 120s venv/bin/python tools/autoresearch/validate_solver.py \
  --config experiments/parameter_candidates/phase6-trial-N.yaml \
  --frames 600 --dt 0.01
```

固定场景、评分、随机条件和安全阈值不得改动。基线 YAML `rm_bringup/config/node_params/armor_solver_params.yaml` 只读；候选从基线副本生成，最佳候选只写 `experiments/best_armor_solver_params.yaml`，不得覆盖基线。

## 可搜索参数和约束

只允许搜索以下参数，单位保持现有 YAML 约定：`prediction_delay [0,0.15]` 秒、`controller_delay [0,0.10]` 秒、`max_tracking_v_yaw [3,8]` rad/s、`center_tracking_distance [0.8,3]` 米、`min_switching_v_yaw [0.3,3]` rad/s、`coming_angle [25,80]` 度、`leaving_angle [5,45]` 度、`fire_margin [0.3,1.5]`、`min_fire_tolerance [0.3,2]` 度、`max_fire_tolerance [2,8]` 度。必须满足 `leaving_angle < coming_angle` 和 `max_fire_tolerance >= min_fire_tolerance`。弹速、装甲板几何、MPC 结构、安全逻辑和验证器不可搜索。

## 每轮流程

1. 检查工具、基线哈希、验证器哈希和算法完成标记。
2. 生成唯一候选 YAML，并记录候选路径和完整 proposal。
3. 运行固定验证器，解析有限的 JSON 指标。
4. 先过安全门槛，再比较 score；`worst_false_fire_rate == 0`、高速场景 `center_tracking_ratio` 不低于基线、静止场景召回率不低于基线、`worst_p95_step_ms <= 20` ms 才能接受。
5. 接受严格降低 score 的候选，更新最佳候选；否则记录 `rejected`。每轮先写 JSONL，再更新 `phase_status.json` 和 `state.json`。

## 异常和停止

- 同一归一化错误第 10 次立即写入 `decision: exit` 并停止。
- 环境或验证异常连续第 11 次立即退出；单条命令超过 120 秒算异常。
- 非有限指标、验证器哈希变化、候选写入失败、ASAN/UBSAN 或安全回归立即停止，不放宽门槛重试。
- 达到 `score <= 9.5`、累计 200 个有效候选、连续 30 个候选无改进、候选池耗尽或外层 120 分钟超时即停止。
- 正常拒绝不增加异常计数。停止时保留最佳候选、JSONL、状态文件和退出原因；基线 YAML 不变。

阶段完成后必须报告有效候选数、最佳 score、最佳 YAML、验证器哈希、安全指标、是否达到目标、是否触发异常退出以及剩余风险。
