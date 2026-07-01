# agentic-et1-tracker 自适应动作衔接剩余 Delta 计划

本文是基于当前代码状态的剩余开发计划，不是从零实现方案。当前代码已经有 Ruckig-backed fixed-horizon synthetic transition、completed user A->B bridge、peek-before-commit、以及跳过 target startup hold 的片段。剩余工作是把这些片段收敛为可验证、语义一致、可发布的实现，保持 HTTP API、队列 FIFO 和安全状态机不扩大。

Ruckig 仍只用于内部 state-to-state transition bridge：每个受控 DoF 提供 current/target position、velocity、acceleration，并受 velocity、acceleration、jerk limits 约束。本文不依赖 Ruckig tracking、intermediate waypoint、云端或 Pro-only 能力。

参考：

- Ruckig Tutorial: https://docs.ruckig.com/tutorial.html
- Ruckig project overview: https://github.com/pantor/ruckig

## 目标

- 把已存在的 Ruckig fixed-horizon transition 改为：先由 Ruckig 求 feasible trajectory duration，再按 policy/playback dt 离散；超过 context cap 才 gate fail。
- 把已存在的 A->B completed user bridge 收敛为明确的事务语义：peek-before-commit、gate rejected 不等于 target request failed、失败时不丢队首 B。
- 修正 policy runtime 下 transition sample dt：优先使用 `DeployConfig.step_dt`，synthetic transition metadata fps 写 `1 / step_dt`；只有非 policy playback 才允许使用 target trk fps。
- 给 skip startup hold 加上 policy state handoff 语义；若暂不实现 runner history/last_action/anchor seed，则收口为 A->B 专用 reduced warmup，不允许直接完全 skip。
- 明确本轮 controlled DoF：仅 26 joints 进入 Ruckig；root yaw Ruckig defer，当前用 post-alignment root yaw residual gate 防止无约束大 yaw bridge。
- 迁移当前 midpoint contact 行为到显式 contact gate 和测试：same-contact allowed；double-support 仅 allowlist；mismatch/unknown/single-support/missing metadata reject bridge。
- 补齐最小配置 schema、测试模板和 release selftest，避免阈值散落在 helper、RuntimeControlLoop 或测试里。
- 保持 KISS/DRY/YAGNI：不新增端点、dashboard、动作图系统、在线调参界面或完整 gait planner。

## 非目标

- 不重写 RuntimeControlLoop 状态机。
- 不改变 active user running/preparing interrupt 的 controlled stop 语义。
- 不修改 `/standby_velocity`、`/stop`、`/passive` 的安全语义。
- 不生成 internal transition 的 run id，不把 internal transition 写入 history。
- 不新增 HTTP API；默认不新增 status JSON 字段。transition context 默认只写内部 diagnostics/logs。
- 不优化、重采样或重写完整用户 trk；只生成衔接 bridge。
- 不用 root xy 驱动 duration；root xy 只记录和诊断。
- 不把 `body_pos`、`ref_com`、`body_lin_vel` 等非 controlled 通道拿去套 joint limits 跑 Ruckig。
- 不暴露 legacy smoother release/runtime 开关；回滚定义为回到上一 release artifact。测试 profile 可以保留 legacy 对照。

## 当前代码基线

### 已存在

- `src/trk_synthetic_transition.cpp` 已有 synthetic transition 生成路径，并已出现 Ruckig-backed fixed-horizon transition 片段；当前剩余问题是 duration 语义仍偏 fixed/requested horizon，需要改成 Ruckig feasible duration 权威。
- 已有 completed user A->B bridge 片段，且已经包含 peek-before-commit 的方向；剩余工作是补齐 gate fail 后的 runtime 语义、诊断和测试。
- 已有 skip target startup hold 片段；剩余工作是证明 policy state handoff 完整，或收敛为 A->B 专用 reduced warmup。
- 当前 transition 已经混合使用 Ruckig 和历史行为：`body_pos_w`、`body_lin_vel_w`、`ref_com_rel_navi`、`ref_com_vel_navi` 已被送入 `sampleRuckig`，但这些非 controlled 通道仍混用默认 limits，单位和物理语义未收敛。剩余工作是把这些通道从 controlled Ruckig 中移除/迁移为同一 normalized sample time 下的派生、保持或诊断策略；contact midpoint 切换需要改为显式 gate。
- `RuntimeControlLoop` 已覆盖 idle->user、standby->user、user 完成后->idle/standby、idle->idle、transition 中被 user 抢断、hold->next/standby。
- `/standby_velocity` 是普通待机/停止动作语义；`/stop` 是 urgent software stop；`/passive` 是 passworded safety sink。本文不改变这些语义。

### 剩余产品问题

- fixed/requested horizon 仍会让小 gap 过拖、大 gap 过急，且容易和 policy dt 不对齐。
- bridge gate rejected 目前容易被误读为 target request failed；需要区分 benign bridge gate rejected 和 unsafe/invalid transition failure，前者优先保留用户显式 target，后者不能被普通 normal start 放行。
- 直接 skip startup hold 如果没有 policy runner state handoff，会造成首个 policy step 的 history/last_action/anchor 不清晰。
- contact midpoint 切换在 GA 语义上过宽，需要显式限制在 same-contact 或 allowlisted double-support。

## 产品原则

- **user work 优先**：用户显式提交、排队或打断的动作优先于 idle/standby/background。背景动作不能阻塞用户动作。
- **queue FIFO**：正常排队动作保持 FIFO。bridge 只能减少卡顿，不能跳队、丢 request 或改变队首。
- **benign bridge gate rejected 不是 request failed**：contact gate reject、post-alignment root yaw residual gate reject、无法生成平滑 bridge 但 target 本身可执行、duration context cap reject 且 target validation 与 raw-start guard 均通过时，不 fail 用户 target。A->B 保持 B 队首 normal start；background->user abort background 后走 normal startup hold。
- **unsafe/invalid transition failure 不能 normal start 放行**：端点速度越限、大 gap 超过安全 raw-start guard、standby raw-start root yaw residual 超阈值、Ruckig input invalid、target validation 失败、robot unsafe/safety sink 等，按现有错误/安全语义 fail target 或进入安全路径。该边界必须与 standby raw-start guard 一致。
- **安全边界不跨越**：`/stop`、`/passive`、controlled stop 不参与 smoother 回滚或 bridge 优化。
- **DRY**：duration/context、Ruckig 参数、contact gate、错误映射和 diagnostics 集中在 transition helper。
- **YAGNI**：不做 gait planner、全轨迹优化、复杂 contact schedule、用户可配置 jerk profile。

## 剩余 Delta 范围

### 1. Runtime 衔接语义

- 保留已存在的 A->B completed user bridge 接入点，但明确只适用于普通非 hold GeneralTracker user A 正常完成、waiting 队首 B 也是可 bridge 的 GeneralTracker user。
- A->B bridge 必须继续在 A 正常完成后、active A 边界状态还可用时构造；不能先进入 `GeneralTrackerIdle` 再从 history 恢复 A 末帧。
- waiting 队首 B 保持 peek-before-commit：bridge 成功 commit 前不得 pop、consume、标记 started 或改变队列位置。
- A->B benign bridge gate reject 时，B 必须留在 waiting 队首；runtime 继续走现有完成路径，后续 `startNext` 按 FIFO normal start B。记录 gate reject reason，但不把 B 标记为 failed。
- background->user benign bridge gate reject 时，abort background transition/playback，然后按 normal startup hold 启动 target user。该情况同样不等于 target request failed。
- unsafe/invalid transition failure 不走上述 normal start fallback：端点速度越限、大 gap 超过安全 raw-start guard、standby raw-start root yaw residual 超阈值、Ruckig input invalid、target validation 失败、robot unsafe/safety sink 等，按现有错误/安全语义 fail user request 或进入安全路径。
- internal bridge 不生成 run id，不写 history，不应用到 LocoUpper 或其他非 GeneralTracker 路径。
- cold start、direct start、standby->user、safety 相关路径保持保守 startup hold；A->B bridge 的优化不得外溢。

### 2. Startup Hold 与 Policy Handoff

- `arrived_via_user_bridge` 或等价 context 必须定义完整 handoff 语义：
  - B 的 anchor frame 如何由 bridge 末帧和 B 首帧建立。
  - `PolicyStepRunner` history 如何 seed。
  - `last_action` 如何继承、重置或由 bridge 末帧推导。
  - 首个 policy step 如何避免重复 0.5s startup hold，同时不丢 bridge 末帧 warm start。
- 如果上述 runner history/last_action/anchor seed 本轮不实现，则 skip startup hold 收口为 A->B 专用 reduced warmup gate：
  - reduced warmup 由配置控制。
  - 只作用于 `arrived_via_user_bridge`。
  - 不允许对 cold start、direct start、standby->user、background->user 直接完全 skip。

### 3. Duration、dt 与 Metadata

- policy runtime 下 transition sample dt 必须优先使用 `DeployConfig.step_dt`，不是 target trk fps。
- policy runtime 生成的 synthetic transition metadata fps 必须写 `1 / step_dt`，用于 playback/policy trace 和测试解释。
- 只有非 policy playback 才允许使用 target trk fps 作为 transition sample dt。
- 测试必须覆盖 target trk fps 与 `DeployConfig.step_dt` 不一致的场景，确认 frame count、metadata fps、actual duration 与 policy step 对齐。
- `estimateTransitionContext(...)` 只提供 context min/max cap、gap diagnostics、limits 和可选 minimum-duration hint；它不拥有最终 duration。
- Ruckig duration 语义改为：

```text
context = estimateTransitionContext(...)
ruckig_input = build_input_without_fixed_horizon_reverse_solve(context, source, target)
feasible_duration = ruckig_trajectory.duration
actual_frames = ceil_to_dt_and_min_frames(feasible_duration, sample_dt, min_transition_frames)
actual_duration = actual_frames * sample_dt
if actual_duration > context_max_duration_s + duration_dt_tolerance_s:
    gate_fail(duration_exceeds_context_cap)
```

- 不再要求 `trajectory.duration == requested_horizon`，也不再从 fixed horizon 反推 Ruckig。
- 若使用 Ruckig `minimum_duration`，它只能是下限 hint，不能把 requested horizon 当作必须等于的最终时长。
- `transition_duration_s` 只保留为 legacy 测试基线或上一 release artifact 的行为说明；正常 GA 路径不把它作为 small gap 下限。

### 4. Controlled DoF 与 Root Yaw Gate

- 本轮 GA controlled DoF 收敛为 26 joints。Ruckig limits 只覆盖这 26 个 joint DoF 的 velocity、acceleration、jerk、单位和顺序。
- root yaw Ruckig 不在本轮实现；当前要求是在 target alignment 之后检查 source root yaw 与 aligned target frame0 root yaw 的 shortest-angle residual，超过阈值时 reject bridge，避免无约束大 yaw bridge。
- root yaw residual gate 必须明确：
  - root body index 来源。
  - yaw 从 root quaternion 提取的坐标系和 wrap 策略。
  - threshold、NaN/非法 quaternion 的 unsafe/internal failure 语义。
  - A->B、background/idle->user、standby raw-start fallback 的 reject 语义。
- 若未来实现 root yaw Ruckig，需要单独 PR 明确 yaw sample/rebuild、roll/pitch、非 root quat 和 `body_ang_vel` 策略及测试。
- `body_pos`、`ref_com`、`body_lin_vel`、非 root quat、contact 以外的非 controlled TRK 通道不能使用 joint limits 跑 Ruckig。允许的最小策略是同一 normalized sample time 下保持、派生或诊断，并验证首尾一致、无 NaN、无明显不连续。
- 若未来要让非 controlled 通道进入 Ruckig，必须另行定义单位、limits 和物理语义；本计划不做。

### 5. Contact Gate

- 迁移当前 midpoint contact 行为：不再默认在 `t=0.5` 切换 contact 并视为 GA bridge 成功。
- contact gate 规则：
  - same-contact allowed。
  - double-support 仅在 `contact_allowlist` 明确列出时 allowed。
  - mismatch、unknown、single-support 切换、missing metadata 一律 reject bridge。
- contact gate reject 时按 runtime context 走保守路径：
  - A->B：B 留在 waiting 队首，后续 normal start。
  - background->user：abort background，target user normal startup hold。
  - return_to_standby/idle：沿用现有安全状态机，不伪造 Ruckig bridge。
- 上述 contact gate reject 属于 benign bridge gate reject；若同时发现 target validation、raw-start guard、Ruckig input 或 robot safety 问题，必须升级为 unsafe/invalid transition failure，不能用普通 normal start 掩盖。

### 6. 最小 Config Schema

必须新增或补齐集中配置，来源可以是 `DeployConfig`/app config/集中常量，但测试必须读取同一来源：

| 配置项 | 内容 |
| --- | --- |
| `transition.contexts.<context>.min_duration_s` | context 下限 |
| `transition.contexts.<context>.max_duration_s` | context cap |
| `transition.contexts.<context>.min_frames` | 最少 transition 帧数 |
| `transition.contexts.<context>.duration_dt_tolerance_s` | dt round-up 容差 |
| `transition.limits.controlled_dofs` | 26 joints 顺序；root yaw Ruckig defer |
| `transition.limits.velocity` | 每个 controlled DoF velocity limit |
| `transition.limits.acceleration` | 每个 controlled DoF acceleration limit |
| `transition.limits.jerk` | 每个 controlled DoF jerk limit |
| `transition.contact_allowlist` | allowlisted double-support/stable contact 状态 |
| `transition.low_state.max_age_s` | actual low_state freshness |
| `transition.low_state.max_gap` | actual-source q/dq gap gate |

同步要求：

- 更新 `app_config_tests`，覆盖缺字段、非法 min/max、DoF 数量不匹配、单位/顺序不明确。
- 更新 config templates/examples，避免 release 和 sim 使用不同隐式默认。
- 增加 release selftest：配置缺失或 limits 不完整时 fail fast；不能在 release profile 静默回到 legacy smoother。

### 7. 文件和模块建议

保持改动集中，优先收敛现有文件，不为了计划新增大系统：

- `include/agentic_et1_tracker/trk/synthetic_transition.hpp`
- `src/trk_synthetic_transition.cpp`
- `include/agentic_et1_tracker/trk/transition_smoothing.hpp`
- `src/transition_smoothing.cpp`
- `include/agentic_et1_tracker/runtime/runtime_control_loop.hpp`
- `src/runtime_control_loop.cpp`
- `include/agentic_et1_tracker/runtime/runtime_config.hpp`
- `include/agentic_et1_tracker/app/app_config.hpp`
- `src/app_config.cpp`
- `tests/trk_synthetic_transition_tests.cpp`
- `tests/runtime_control_loop_tests.cpp`
- `tests/app_config_tests.cpp`

实现约束：

- RuntimeControlLoop 不直接调用 Ruckig API；它只调用内部 transition helper。
- Ruckig 参数构造、duration/context 估算、contact gate、fallback/error code 映射集中在 helper 层。
- 保留现有 `makeSyntheticTransitionTrk` 兼容入口或 options overload；不要让 fixed-horizon 和 feasible-duration 两套 GA 逻辑长期并存。

## 禁止扩张项

- 禁止新增 HTTP API、endpoint、dashboard 或治理流程。
- 禁止改变 LocoUpper 或非 GeneralTracker 的队列/启动路径。
- 禁止改变 queue FIFO。
- 禁止用 root xy 驱动 duration。
- 禁止实现 contact-aware gait 或 footstep planner。
- 禁止把 `/stop` 做成平滑停止。
- 禁止依赖 Ruckig tracking/intermediate waypoint 或 Pro-only 能力。
- 禁止把 Ruckig 用于完整用户动作重规划。
- 禁止暴露 GA/runtime legacy smoother 开关。

## 测试与验收

### 单元测试

- duration/dt：
  - policy runtime 使用 `DeployConfig.step_dt` 采样，metadata fps 为 `1 / step_dt`。
  - target trk fps 与 policy `step_dt` 不一致时，policy dt 优先；非 policy playback 才使用 target trk fps。
  - Ruckig `trajectory.duration` 是权威 duration；不要求等于 requested horizon。
  - Ruckig feasible duration 超过 context cap + tolerance 时 gate fail。
  - dt round-up、min frames、actual duration、frame count 一致。
  - small gap 不被 `transition_duration_s` 拖成旧固定等待。
  - post-alignment root yaw residual 使用最小角差 gate；root xy 只记录诊断，不改变 duration。
- config/limits：
  - 最小 config schema 字段可加载。
  - context min/max/min_frames/tolerance 缺失或非法时报明确错误。
  - 26 joint limits 数量、单位、顺序不匹配时 release/selftest fail；root yaw residual gate threshold 与语义有独立测试。
  - low_state freshness/gap 从同一配置读取；测试不复制硬编码阈值。
  - `app_config_tests`、config templates、release selftest 覆盖同一 schema。
- contact gate：
  - same-contact allowed。
  - allowlisted double-support allowed。
  - mismatch、unknown、single-support、missing metadata reject bridge。
  - 当前 midpoint contact 行为有迁移测试，证明 GA 不再用中点切换绕过 contact gate。
- Ruckig bridge：
  - controlled DoF 仅为 26 joints；root yaw Ruckig defer。
  - 输出满足 configured v/a/j limits。
  - post-alignment root yaw residual gate 覆盖 root body index、yaw extract/wrap、非法 quaternion、A->B/background/idle/standby fallback 语义。
  - `body_pos_w`、`body_lin_vel_w`、`ref_com_rel_navi`、`ref_com_vel_navi` 等非 controlled 通道从 controlled Ruckig 中移除/迁移，使用同一 normalized sample time 派生/保持/诊断，不再混用默认 limits 独立 Ruckig。
  - 输出无 NaN/Inf；首尾 frame 精确匹配 source/target controlled positions。
  - Ruckig failure 不静默产出 legacy bridge。
- legacy profile：
  - 测试 profile 可保留 legacy smoother 对照。
  - release/runtime 不暴露 legacy smoother 开关；回滚以上一 release artifact 为准。

### Runtime 测试

- 现有路径不回退：idle->user、standby->user、user 完成后->idle/standby、idle->idle、transition 中被 user 抢断、hold->next/standby。
- A->B completed user bridge：
  - 只对 GeneralTracker waiting 生效。
  - bridge 在 A complete 且 A 边界仍可用时构造。
  - waiting 队首 B peek-before-commit；成功前不得 pop/consume/started。
  - internal bridge 不生成 run id、不写 history、不影响 LocoUpper/非 GeneralTracker。
  - B 不可 bridge、boundary incomplete、contact gate reject、无平滑 bridge 但 target validation 与 raw-start guard 均通过、duration context cap reject 且 raw-start guard 通过时，B 仍留在 waiting 队首，后续 FIFO normal start。
  - 端点速度越限、大 gap 超过安全 raw-start guard、Ruckig input invalid、target validation 失败、robot unsafe/safety sink 时，不 normal start 放行，按现有错误/安全语义 fail target 或进入安全路径。
- bridge gate rejected semantics：
  - A->B benign bridge gate reject 不 fail target request。
  - background->user benign bridge gate reject 会 abort background，并用 normal startup hold 启动 target user。
  - unsafe/invalid transition failure 覆盖 endpoint velocity limit、raw-start guard 大 gap、Ruckig input invalid、target validation、robot unsafe/safety sink，并按现有错误/安全语义处理。
- startup hold/handoff：
  - 若实现完整 handoff，测试覆盖 anchor/history/last_action/`PolicyStepRunner` seed。
  - 若采用 reduced warmup，测试确认只作用于 `arrived_via_user_bridge`。
  - cold start、direct start、standby->user、background->user、安全路径不完全 skip startup hold。
- 状态和 history：
  - transition context 不把 internal transition 当作 user run。
  - 默认不新增 status JSON 字段；若决定 additive status field，必须更新 JSON schema/golden/API 兼容测试。

### 仿真验收

仿真验收不能只依赖肉眼。每个通过用例必须有日志和断言：

- 无 passive、fall、safety sink、NaN/Inf、policy runner error。
- policy/write interval jitter 在可接受范围内，且 transition 前后无明显 tick drift。
- transition sample dt、metadata fps、frame count、actual duration 与 policy/playback dt 一致。
- Ruckig output 在 declared controlled DoF 的 v/a/j limits 内。
- upper-body q actual-ref max/RMS 在 transition 前、中、后均记录并满足阈值。
- motor command q/dq 无 spike 或 saturation。
- transition 前后 3-5 step lowcmd discontinuity 在阈值内。
- Ruckig feasible duration 和离散 actual duration 不超过 context cap + tolerance。
- contact gate 覆盖 same-contact allowed、allowlisted double-support allowed、mismatch/unknown/single-support/missing rejected。
- queue A->B 不出现 duplicate run id，不写 internal history；benign bridge gate reject 后 B 仍为队首并按 FIFO 启动。
- background->user benign bridge gate reject 后 target user 通过 normal startup hold 启动。
- unsafe/invalid transition failure 不被 normal start 掩盖，并与 standby raw-start guard 边界一致。
- `/standby_velocity`、`/stop`、`/passive` 语义保持不变。

仿真日志至少记录：

- transition context。
- sample dt 来源、metadata fps、frame count、actual duration。
- estimator diagnostics、Ruckig feasible duration、duration cap/gate result。
- Ruckig result/error。
- controlled DoF/limits source。
- velocity/acceleration/jerk max。
- post-alignment root yaw residual、root xy diagnostic gap。
- contact gate result。
- actual-source gap、low_state freshness。
- fallback/zero velocity reason。
- policy/write jitter、upper-body actual-ref max/RMS、motor q/dq spike/saturation、lowcmd discontinuity。

## 风险与回滚

- **风险：feasible duration 超过 cap 过多**
  若 target validation 与 raw-start guard 均通过，按 benign bridge gate reject 走对应保守路径；若 cap failure 暴露大 gap 已超过安全 raw-start guard 或端点状态无效，则按 unsafe/invalid transition failure 处理。不要把 cap 后 duration 强行写回 Ruckig，也不要为了通过而放宽到无界等待。

- **风险：policy handoff 不完整**
  不直接完全 skip startup hold。先收敛为 A->B 专用 reduced warmup，再用测试证明不会影响 cold/direct/standby/background paths。

- **风险：contact mismatch 被平滑掩盖**
  contact gate 不通过时拒绝 bridge。当前 midpoint contact 行为只作为迁移对象和测试输入，不作为 GA 通过条件。

- **风险：非 controlled 通道与 controlled DoF 不一致**
  当前 `body_pos_w`、`body_lin_vel_w`、`ref_com_rel_navi`、`ref_com_vel_navi` 已混入 `sampleRuckig` 并使用默认 limits。迁移目标是从 controlled Ruckig 中移除这些非 controlled 通道，改为同一 normalized sample time 派生/保持/诊断；无法证明一致时 gate fail，不扩大到全身 Ruckig。

回滚策略：

- GA/runtime 不暴露 legacy smoother 开关。
- 回滚定义为部署上一 release artifact。
- 测试 profile 可以保留 legacy smoother 对照，用于比较和回归定位。

## Handoff Checklist

- [ ] 文档保持为剩余 delta 计划，不再把已存在的 Ruckig fixed-horizon、A->B bridge、peek-before-commit、skip startup hold 写成未实现能力。
- [ ] policy runtime transition sample dt 优先使用 `DeployConfig.step_dt`，metadata fps 写 `1 / step_dt`；非 policy playback 才使用 target trk fps。
- [ ] 测试覆盖 target fps 与 policy `step_dt` 不一致。
- [ ] Ruckig duration 改为 feasible duration 权威，再按 dt 量化；超过 context cap + tolerance 才 gate fail。
- [ ] 移除 `trajectory.duration == requested_horizon` 这类测试/要求。
- [ ] A->B benign bridge gate reject 保持 B 队首，后续 FIFO normal start。
- [ ] background->user benign bridge gate reject abort background，并 normal startup hold 启动 target user。
- [ ] 区分 benign bridge gate rejected、unsafe/invalid transition failure 和 target request failed；endpoint velocity limit、raw-start guard 大 gap、Ruckig input invalid、target validation、robot unsafe/safety sink 不被 normal start 放行。
- [ ] skip startup hold 补齐 policy state handoff；否则改为 A->B 专用 reduced warmup。
- [ ] contact gate 迁移 midpoint 行为：same-contact allowed、allowlisted double-support allowed、mismatch/unknown/single-support/missing rejected。
- [ ] controlled DoF 固定为 26 joints；`body_pos_w`、`body_lin_vel_w`、`ref_com_rel_navi`、`ref_com_vel_navi` 等非 controlled 通道从 controlled Ruckig 中移除/迁移，不再混用默认 limits 跑 Ruckig。
- [ ] root yaw Ruckig defer；本轮明确 root body index、yaw extract/wrap、post-alignment residual gate 和各 runtime context reject 语义。若实现 root yaw Ruckig，单独 PR 处理 yaw rebuild、roll/pitch、非 root quat、`body_ang_vel` 策略。
- [ ] 补齐最小 config schema：contexts min/max/min_frames/tolerance、limits、contact_allowlist、low_state freshness/gap。
- [ ] 同步 `app_config_tests`、config templates、release selftest。
- [ ] release/runtime 不暴露 legacy smoother 开关；回滚到上一 release artifact。
- [ ] 仿真验收加入 policy/write interval jitter、upper-body q actual-ref max/RMS、motor command q/dq spike/saturation、transition 前后 3-5 step lowcmd discontinuity。
