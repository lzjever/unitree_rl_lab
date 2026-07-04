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

### Round 20260704-6

- 命令/场景：干净预启动 MuJoCo，确认 `sim-control status` 正常；运行当前 HEAD 的 `manual_gate.py all --start-tracker --enable-loco-temp --require-loco --mujoco-land-settle --mujoco-land-release-check-s 0 --standby-soak-s 5 --standby-soak-interval-s 0.5 --artifacts-dir /tmp/agentic-et1-sim-records/round-20260704-6`
- 结果：PASS。
- 覆盖：landing settle、startup recovery、standby/urgent_stop/passive idle semantics、idle preempt/resume/clear、active user standby/urgent_stop handoff、queue/interrupt contract、held interrupt handoff、short hold settle、transition interrupt + standby handoff、loco_upper bounded blackbox、visual screenshot、5s standby soak。
- 证据：报告 `/tmp/agentic-et1-sim-records/round-20260704-6/manual_gate_report_20260704-150338.json`；截图 `/tmp/agentic-et1-sim-records/round-20260704-6/mujoco_visual_gate_20260704-150327.png`
- 结论：当前主 e2e/visual/loco/soak gate 通过；截图人工检查机器人直立可见，无黑屏/错窗/倒地。

### Round 20260704-7

- 命令/场景：直接 HTTP P1 诊断脚本，覆盖 pending-control 黑盒窗口和 `GeneralTracker hold -> loco_upper interrupt`；脚本自行启动 MuJoCo 和 tracker。
- 结果：FAIL。
- 原因：前置时序失败；tracker 刚启动时 lowstate 未连接，`POST /fixstand` 返回 `ROBOT_DISCONNECTED`。
- 证据：`/tmp/agentic-et1-sim-records/round-20260704-7/direct_p1_report.json`
- 结论：不是业务路径失败；直接诊断脚本需要等待 `/status.ready=true` 且 `block=null` 后再发控制命令。

### Round 20260704-8

- 命令/场景：修正前置等待后重跑直接 HTTP P1 诊断。
- 结果：FAIL。
- 原因：`loco_upper hold -> /standby` 的 handoff 窗口中，`/execute` 和 `/execute_loco_upper` 返回 409，但非空 `/idle` 被 200 接受并设置 `idle.enabled=true`。
- 证据：`/tmp/agentic-et1-sim-records/round-20260704-8/direct_p1_report.json`
- 结论：发现真实缺陷。根因是非空 idle 只按 `ctrl/pending_control` gate，漏掉 `ctrl=running + active.kind=transition + transition.target=standby` 的 standby handoff 状态。后续修正为 API/store 共享 handoff gate，空 idle clear 仍允许。

### Round 20260704-9

- 命令/场景：修复非空 idle 漏网后重跑直接 HTTP P1 诊断。
- 结果：FAIL。
- 原因：非空 `/idle` 已正确 409，但同一 standby handoff 窗口中 `/execute_loco_upper` 仍可能 200 queued；`/execute` 正确 409。
- 证据：`/tmp/agentic-et1-sim-records/round-20260704-9/direct_p1_report.json`
- 结论：发现同源缺陷。根因是 `/execute_loco_upper` 和 store `acceptQueued/acceptInterrupt` 仍只看 `ctrl/pending_control`，没有统一识别 standby handoff transition。后续修正为 `controlHandoffBlocksUserWork(snapshot)`，所有 user work submit 共享该判断。

### Round 20260704-10

- 命令/场景：引入共享 `controlHandoffBlocksUserWork(snapshot)` 后重跑直接 HTTP P1 诊断。
- 结果：PASS。
- 覆盖：`loco_upper hold -> /standby` handoff 窗口立即投递 `/execute`、`/execute_loco_upper`、非空 `/idle` 均 409；handoff 收敛到 standby 后 `/execute` 正常 200；`GeneralTracker hold -> /execute_loco_upper mode=interrupt` 中旧 run 为 `stopped/interrupt`，新 run 为 `executor=loco_upper`，queue 为空。
- 证据：`/tmp/agentic-et1-sim-records/round-20260704-10/direct_p1_report.json`
- 结论：P1 黑盒诊断通过；standby/control handoff 窗口不再漏入新 user work，且收敛后正常恢复投递能力。

### Round 20260704-11（white-box 全量回归）

- 命令/场景：`cmake --build deploy/robots/agentic-et1-tracker/build --target agentic-et1-tracker agentic_et1_tracker_api_tests agentic_et1_tracker_runtime_tests agentic_et1_tracker_http_tests agentic_et1_tracker_loco_upper_tests -j2 && ctest --test-dir deploy/robots/agentic-et1-tracker/build -R 'agentic_et1_tracker_' --output-on-failure`
- 结果：PASS，13/13 pass。
- 覆盖：tracker core、robot core、trk、loco_upper、api、runtime、policy step runner、policy、control、http、app、policy_onnx、unitree sdk robot tests。
- 结论：Round 8/9 修复后的 white-box 全量回归通过。

### Round 20260704-12

- 命令/场景：在 Round 8/9 修复后，干净预启动 MuJoCo，确认 `sim-control status` 正常；再次运行 `manual_gate.py all --start-tracker --enable-loco-temp --require-loco --mujoco-land-settle --mujoco-land-release-check-s 0 --standby-soak-s 5 --standby-soak-interval-s 0.5 --artifacts-dir /tmp/agentic-et1-sim-records/round-20260704-12`
- 结果：PASS。
- 覆盖：与 Round 6 相同的完整 e2e/visual/loco/standby soak 主 gate；验证最新 handoff 修复没有破坏主路径。
- 证据：报告 `/tmp/agentic-et1-sim-records/round-20260704-12/manual_gate_report_20260704-154412.json`；截图 `/tmp/agentic-et1-sim-records/round-20260704-12/mujoco_visual_gate_20260704-154400.png`
- 结论：最新工作树的完整仿真 gate 通过；截图人工检查机器人直立可见，无黑屏/错窗/倒地。

## 已关闭 findings

- `control command` vs `/execute` race：已通过 Round 5 TDD 和 Round 11 全量回归验证。
- `idle clear` during `transition->idle`：已通过 Round 5 TDD 和 Round 11 全量回归验证。
- held `GeneralTracker` interrupt to `loco_upper`：已通过 Round 5 TDD、Round 10 仿真黑盒和 Round 11 全量回归验证。
- standby/control handoff 窗口漏入非空 `/idle`：Round 8 复现，已通过共享 handoff gate 修复，Round 10 仿真黑盒通过。
- standby/control handoff 窗口漏入 `/execute_loco_upper`：Round 9 复现，已通过 `controlHandoffBlocksUserWork(snapshot)` 统一修复，Round 10 仿真黑盒通过。
- 主 e2e/visual/loco/standby soak gate：Round 12 在最新工作树重新验证通过。
