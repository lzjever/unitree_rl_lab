# agentic-et1-tracker adaptive motion handoff GA 收口计划

本文按当前 reviewer 结论收口 adaptive motion handoff。它不是从零实现方案，也不再把已经完成的 core adaptive transition 能力继续列为待实现 delta。当前阶段只做 GA 合同收口、测试补齐和 release 验收。

GA 策略保持 KISS/DRY/YAGNI：

- 只保留一种 bridge 产品语义：conservative same-contact bridge。
- 只保留一种 startup 优化：成功 A->B user bridge 后的 reduced warmup。
- 不新增 HTTP API、dashboard、status field、在线调参、动作图系统或 gait planner。
- RuntimeControlLoop 不直接拥有 Ruckig 细节；bridge 构造、gate、错误映射和 diagnostics 仍集中在 transition helper。
- `/standby_velocity`、`/stop`、`/passive`、controlled stop 和 safety sink 语义不变。

## Done

以下能力已作为当前 adaptive transition 基线存在，不再作为下一阶段开发项重新展开：

- Ruckig-backed adaptive transition bridge 已是内部 state-to-state bridge 路径；Ruckig feasible duration 是 bridge duration 的权威来源，再按 runtime dt 离散。
- policy runtime transition sample dt 使用 `DeployConfig.step_dt`，metadata fps 使用 `1 / step_dt`；非 policy playback 才使用 target trk fps。
- A->B completed user bridge 已采用 peek-before-commit 方向：bridge commit 前不 pop、consume、start 或改变 waiting 队首 B。
- internal bridge 不生成 user run id，不写 user history，不改变 FIFO 队列语义。
- controlled DoF 已收敛到 26 joints；root yaw Ruckig 不属于当前 GA 路径。
- startup hold 不做完整 skip；当前 GA 只允许成功 A->B bridge 后 reduced warmup。

这些项目后续只需要通过合同测试和 release selftest 固化，避免出现第二套实现或旧 fixed-horizon/legacy smoother 语义回流。

## Closed GA Contract / Release Acceptance

以下内容是当前 GA 已收口的合同和发布验收口径。仿真/真机 smoke 只作为 release acceptance，不再作为继续开发 blocker。

### 1. Conservative Bridge Contract

GA 只允许 same-contact bridge：

- source frame 与 aligned target frame0 的左右 contact metadata 必须完全相同。
- source/target 两脚 contact 值都必须是明确非 0；0 在 bridge gate 中视为 unknown/no-contact。
- mismatch、unknown/no-contact、single-support 切换、missing metadata 一律 reject bridge。
- 不进入额外 contact taxonomy allowlist；不要留下当前可配置但未验证的放行路径。
- 旧 midpoint contact 切换只能作为迁移/回归测试输入，不能作为 GA bridge 成功条件。

benign bridge gate reject 的 runtime 语义必须固定：

- A->B：B 留在 waiting 队首，后续按 FIFO normal start / full startup hold；不标记 B failed，不生成 internal run，不写 history。
- background->user：abort background transition/playback，然后用 full startup hold 启动 target user。
- return_to_idle / return_to_standby：沿用现有安全状态机，不伪造 Ruckig bridge。

### 2. Unsafe/Invalid 不得 Normal-Start 放行

必须把 benign bridge gate reject 与 unsafe/invalid transition failure 分开。以下情况不能被普通 normal start fallback 掩盖：

- endpoint velocity/acceleration/limit 明显越界。
- actual-source gap 超过 raw-start guard。
- Ruckig input invalid 或 trajectory generation failure 暴露非法端点。
- target validation 失败。
- robot unsafe、safety sink、passive 或 urgent stop 相关路径。
- root body 缺失、root yaw quaternion NaN/Inf、不可归一化或 yaw extraction 失败。

root yaw 当前只做 post-alignment residual gate，不做 Ruckig。valid quaternion 但 residual 超阈值时 reject bridge；若 target validation 与 raw-start guard 仍通过，按对应 context 的 benign reject 处理。invalid quaternion 采用保守语义：视为 unsafe/invalid transition failure，而不是 benign reject，也不能为了启动 target 走普通 normal start 放行。若 invalid 来自 target，应按 target validation/safety 失败处理；若 invalid 来自 actual/source，应按现有 robot safety 或 raw-start guard 失败处理。

### 3. A->B Reduced Warmup Boundary

当前 GA 不实现完整 policy handoff seed。reduced warmup 只允许在以下条件同时成立时使用：

- source A 是正常完成的 GeneralTracker user。
- waiting 队首 B 是可 bridge 的 GeneralTracker user。
- bridge 成功 commit，且 contact/root yaw/raw-start/Ruckig/target validation 全部通过。
- B 通过 bridge 末帧进入 startup，带 `arrived_via_user_bridge` 或等价内部 context。

其他路径都使用 full startup hold：cold start、direct start、standby->user、background->user、bridge benign reject fallback、安全路径和 invalid transition failure 后的恢复路径。

### 4. Release 和 Tests 验收

GA 合同以测试和 release selftest 关闭，而不是扩大功能：

- 单元测试覆盖 explicit same-contact allowed，以及 mismatch/unknown/no-contact/single-support switch/missing metadata rejected。
- Runtime 测试覆盖 A->B peek-before-commit、benign reject 后 B 仍为队首、background->user benign reject 使用 full startup hold。
- Runtime 测试覆盖 unsafe/invalid transition failure 不走 normal start fallback，至少包含 raw-start guard 大 gap、Ruckig invalid input、target validation failure、robot unsafe/safety sink、root yaw invalid quaternion。
- Startup 测试覆盖成功 A->B bridge 才 reduced warmup；cold/direct/standby/background/reject fallback 都 full startup hold。
- 本阶段已收口 release selftest/config fail-fast：required GA flat knobs、transition limits、contact guard、root yaw guard 缺失时由 release binary parse-only check fail fast，已安装 shared config 也由 parse-only check 覆盖；release template/help/scripts 不暴露 legacy smoother 开关。更广 runtime 合同和仿真验收仍按下方 checklist/验收项推进。
- 仿真/真机 smoke 只保留 GA 必需断言：无 passive/fall/safety sink/NaN/Inf/policy runner error，bridge dt/fps/frame count/duration 一致，controlled DoF v/a/j limits 内，queue/run-id/history 无异常。

## Deferred / Future

以下内容明确不进入当前 GA，不应在当前计划中包装成 blocker：

- 完整 policy handoff seed：anchor frame、`PolicyStepRunner` history、`last_action`、policy hidden/history state 的严格继承或重建。
- root yaw Ruckig：yaw sample/rebuild、roll/pitch 保持、非 root quat、`body_ang_vel` 策略及完整测试。
- contact taxonomy allowlist：稳定 contact taxonomy、配置 schema、仿真覆盖和 release policy。
- nested `transition.contexts` / `transition.limits` / `transition.low_state` 全量 schema：当前只要求 release 所需配置同源、fail fast，不做完整嵌套 schema 迁移。
- 全套仿真 telemetry 治理：跨场景指标治理、长期数据保留、在线分析流程、复杂 telemetry taxonomy。
- contact-aware gait planner、全轨迹优化、完整用户动作重规划、Ruckig tracking/intermediate waypoint、Pro-only 能力。

## Guardrails

- 不新增 HTTP API、endpoint、dashboard 或 status JSON field。
- 不改变 LocoUpper 或非 GeneralTracker 的队列/启动路径。
- 不改变 queue FIFO。
- 不用 root xy 驱动 duration；root xy 只允许诊断。
- 不把 `/stop` 做成平滑停止。
- 不暴露 GA/runtime legacy smoother 开关；回滚定义为部署上一 release artifact。
- 测试 profile 可保留 legacy smoother 对照，但不能成为 release/runtime 分支。

## Handoff Checklist

- [x] 文档、代码和测试不再把已完成 core adaptive transition 能力列为待实现。
- [x] GA bridge 只有 explicit same-contact allowed；mismatch/unknown/no-contact/single-support switch/missing metadata 全部 reject。
- [x] A->B benign reject 保持 B 为 waiting 队首，后续 FIFO normal start / full startup hold，不 fail request。
- [x] background->user benign reject abort background，并用 full startup hold 启动 target user。
- [x] unsafe/invalid transition failure 不 normal-start 放行；root yaw invalid quaternion 走保守失败语义。
- [x] 成功 A->B bridge 才 reduced warmup；其他 startup/reject/safety 路径 full startup hold。
- [x] Release selftest/config fail-fast 覆盖 required GA flat knobs/limits/contact/root yaw guard，release template/help/scripts 无 legacy smoother 开关。
- [x] Runtime/unit tests 继续覆盖上述 bridge/startup/safety 合同。
- [x] Deferred 列表保持延期，不新增 API、dashboard、status field 或第二套 bridge 做法。

## Release Acceptance Checklist

- [ ] 仿真 smoke：按 GA 必需断言验收，不作为继续开发 blocker。
- [ ] 真机 smoke：按 GA 必需断言验收，不作为继续开发 blocker。
