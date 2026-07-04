# agentic-et1-tracker Simulation Test Record

目标：记录 agentic-et1-tracker 仿真状态机/功能验证。每次测试一条，按时间追加，包含命令/场景、结果、证据路径和结论。

## 20260704

### Round 20260704-1

- 命令/场景：`manual_gate.py all --start-tracker --enable-loco-temp --require-loco --mujoco-land-settle --mujoco-land-release-check-s 0 --standby-soak-s 2 --standby-soak-interval-s 0.5 --artifacts-dir /tmp/agentic-et1-sim-records/round-20260704-1`
- 结果：FAIL。
- 原因：不是 tracker 合同失败；复用旧 MuJoCo 时机器人已进入 `bad_orientation/passive`，`root_z=-0.708`。
- 证据：`/tmp/agentic-et1-sim-records/round-20260704-1/manual_gate_report_20260704-140508_FAILED.json`
- 结论：仿真前置环境必须从干净姿态启动或 reset。

### Round 20260704-2

- 命令/场景：同 Round 20260704-1，但加 `--start-mujoco-cmd ...` 由 `manual_gate.py` 直接启动 MuJoCo。
- 结果：FAIL。
- 原因：MuJoCo 刚启动时 lowstate 未连接，`POST /fixstand` 返回 `ROBOT_DISCONNECTED`。
- 证据：`/tmp/agentic-et1-sim-records/round-20260704-2/manual_gate_report_20260704-140611_FAILED.json`
- 结论：本环境中 `manual_gate.py` 直接启动 MuJoCo 后需要预热等待。

### Round 20260704-3

- 命令/场景：先手动预启动 MuJoCo，确认 `sim-control status` 显示 `root_z=1.439`，等待 3s；再运行 `manual_gate.py all --start-tracker --enable-loco-temp --require-loco --mujoco-land-settle --mujoco-land-release-check-s 0 --standby-soak-s 2 --standby-soak-interval-s 0.5 --artifacts-dir /tmp/agentic-et1-sim-records/round-20260704-3`
- 结果：PASS。
- 覆盖：`startup_control_recovery`、standby vs urgent_stop/passive idle semantics、`idle_set_preempt_resume_clear`、`active_user_idle_enabled_standby_handoff`、`active_user_idle_enabled_urgent_stop_clears_idle`、`queue_interrupt_status_contract`、`held_interrupt_handoff`、`short_hold_settle_standby`、`transition_interrupt_and_standby_handoff`、loco_upper bounded blackbox、visual、standby_soak。
- 证据：报告 `/tmp/agentic-et1-sim-records/round-20260704-3/manual_gate_report_20260704-140724.json`；截图 `/tmp/agentic-et1-sim-records/round-20260704-3/mujoco_visual_gate_20260704-140715.png`
- 结论：主 e2e/visual/loco/soak 通过；截图人工检查机器人直立可见，无黑屏/错窗/倒地。

### Round 20260704-4

- 命令/场景：临时 direct HTTP 测试脚本，尝试从上一轮留下的 MuJoCo 继续启动 tracker。
- 结果：FAIL。
- 原因：启动即 `bad_orientation/passive`；`manual_gate.py` 停 tracker 后，MuJoCo 无控制继续运行导致姿态越界。
- 证据：`/tmp/agentic-et1-sim-records/round-20260704-4/direct_http_report.json`
- 结论：direct HTTP 测试也需要自带干净 MuJoCo 启动和 landing settle，不能复用无控制后的仿真状态。

### Round 20260704-5（white-box TDD 回归）

- 命令/场景：`ctest --test-dir deploy/robots/agentic-et1-tracker/build -R 'agentic_et1_tracker_(api|runtime|http|loco_upper)_tests' --output-on-failure`
- 结果：PASS，4/4 pass。
- 覆盖：`control command` vs `/execute` race、`idle clear` during `transition->idle`、held `GeneralTracker` interrupt to `loco_upper`。
- 结论：三项 findings 已通过 ctest 小范围 white-box TDD 回归验证。

## 已完成 TDD 回归 findings

- `control command` vs `/execute` race。
- `idle clear` during `transition->idle`。
- held `GeneralTracker` interrupt to `loco_upper`。

以上三项已通过 `ctest --test-dir deploy/robots/agentic-et1-tracker/build -R 'agentic_et1_tracker_(api|runtime|http|loco_upper)_tests' --output-on-failure` 验证，结果 4/4 pass。
