# agentic-et1-tracker 自适应动作衔接开发计划

本文是开发计划，不代表功能已经实现。目标是先规划前两个阶段，聚焦 agentic-et1-tracker 当前动作衔接体验中最容易感知的卡顿、突变和等待问题，同时保持实现简单、可回退、可测试。

## 目标

- 让 idle/standby/background 到 user motion，以及普通 user motion 之间的衔接更自然，减少固定时长 transition 带来的“有时太短、有时太拖”的体感问题。
- 第一阶段必须解决 GeneralTracker 普通非 hold user A 正常结束且 B 已 waiting 时的连续动作缺口：从 A 末帧到 B 首帧生成 internal user-to-user bridge。
- 第一阶段用最小实现替代固定全局 transition duration：按 transition context 选择不同的 min/max cap 和 rate hints，再根据 source/target reference gap 估算 duration。
- 第一阶段只允许 yaw 和 upper-body/key joint gap 参与 duration 估算；root xy 只记录和诊断，不驱动 duration。
- 第一阶段同时把 transition 插值曲线从线性推进升级为更平滑的时间曲线，优先使用简单、确定、无额外依赖的 smoothstep/smootherstep。该曲线是体感优化启发式，不是速度、加速度、jerk 约束轨迹。
- 第二阶段仅作为实测仍需要时的后续阶段，再评估 Ruckig 或 jerk-limited smoother；它只用于内部 state-to-state transition bridge，不重写用户 trk。
- 保持 HTTP API 不变，避免把过渡策略暴露成用户必须理解的新概念。
- `transition_duration_s` 的语义收敛为兼容 fallback：估算失败或 adaptive smoothing 关闭时使用该值；估算成功时由 transition context 的 min/max 和 gap/rate hints 决定，避免旧配置继续拖慢小 gap。

## 非目标

- 不重写 RuntimeControlLoop 状态机。
- 不把 active user running/preparing interrupt 从 controlled stop 改成 direct current-frame transition。
- 不修改 `/standby_velocity`、`/stop`、`/passive` 的安全语义。
- 不生成 internal transition 的 run id，不把 internal transition 写入 history。
- 不新增对外状态端点；状态解释只在现有 status 承载范围内增加或利用 transition context，做不到则 defer。
- 不做 contact-aware gait 规划，不改 foot contact 语义为复杂步态优化。
- 不优化或重采样完整用户 trk，不改用户上传动作本体。
- 不把 Ruckig tracking 或 intermediate waypoint 能力作为第一阶段核心假设。
- 不引入新的 HTTP API、复杂策略 DSL、在线参数调优界面或产品级可视化面板。

## 当前行为审查

### 已确认实现事实

- 当前合成过渡在 `src/trk_synthetic_transition.cpp`，`makeSyntheticTransitionTrk` 使用固定 duration。
- 当前过渡对 `joint_pos`、`body_pos`、`ref_com` 做线性插值；quat 使用 nlerp；contact 在 `t=0.5` 切换；velocity 使用有限差分；`body_ang_vel` 全 0。
- 当前 `transition_duration_s` 配置值存在差异：`app_config` 为 0.30s，sim example 为 0.7s。
- `RuntimeControlLoop` 已覆盖 idle->user、standby->user、user 完成后->idle/standby、idle->idle、transition 中被 user 抢断、hold->next/standby。
- 普通非 hold user A 正常结束且 user B 已在 waiting queue 时，当前不是 A->B direct transition。实际流程是：A 结束；因为 waiting 非空，完成路径跳过 user->idle/standby transition；`finishActive` 进入 `GeneralTrackerIdle`；下一 tick `startNext` 启动 B；在 policy runtime 下 B 仍会经历固定 0.5s startup hold，并按 policy step 计数。这会造成用户动作之间的体感间隔。
- active user running/preparing interrupt 当前走 controlled stop。第一阶段不改变该语义。
- idle/standby/background transition/playback 是背景任务，应被 user queue/interrupt 平滑接管。
- `/standby_velocity` 是普通待机/停止动作语义，保留 idle config；`/stop` 是 urgent software stop，清 idle、abort，不承诺平滑；`/passive` 是 passworded safety sink，不自动恢复。

### 产品体验问题

- 固定 duration 无法同时适配小幅衔接和大幅衔接：小 gap 时显得拖，大 gap 时显得急。
- 线性 alpha 在起止点速度不连续，容易让用户感知到“突然开始”和“突然结束”。
- user-to-user queue 当前只避免了先回 idle/standby；它没有 A 末帧到 B 首帧的 bridge，且 B 的固定 startup hold 仍可能让连续动作看起来像中间停了一拍。
- transition 是背景内部行为，但状态查询如果只显示“在 transition”，用户可能难以理解当前是在接管、回待机，还是排队动作之间的桥接。
- 过渡期间被 user 抢断的行为已经存在，但需要继续保持“user work 优先”的直觉：用户主动动作不应被 idle/standby 背景动作拖住。

## 产品原则

- **user work 优先**：用户显式提交、排队或打断的动作优先于 idle/standby/background。背景动作只提供默认姿态和空闲体验，不能阻塞用户动作。
- **idle/standby 是背景**：idle/standby transition/playback 不生成用户可见 run id，不进入 history，不让用户误以为这是一个正式任务。
- **queue FIFO**：正常排队动作保持 FIFO。优化衔接时只减少桥接卡顿，不改变队列顺序。
- **interrupt 语义保守**：active user running/preparing interrupt 继续走 controlled stop。更激进的 current-frame direct transition 必须另起设计和安全评审。
- **安全边界不跨越**：`/stop` 仍是 urgent software stop，清 idle、abort，不承诺平滑；`/passive` 仍是 passworded safety sink，不自动恢复。
- **状态查询要解释 transition**：不新增端点、不生成 run id；只在现有 status 能承载的范围内增加或利用 transition context，例如 from background to user、user bridge、return to standby。若现有 status 不适合承载，先 defer，不为第一阶段扩大接口面。
- **KISS**：第一阶段只做 duration 估算和确定性平滑曲线，不引入重型优化器或复杂配置面。
- **DRY**：duration 估算、clamp、曲线函数应集中实现，RuntimeControlLoop 不应复制多套过渡策略。
- **YAGNI**：在没有实测证据前，不做完整 gait planner、全轨迹优化、复杂 contact schedule、用户可配置 jerk profile、动作图编辑器。

## 阶段 1 计划：GeneralTracker bridge + context-aware duration + 平滑曲线

阶段 1 的目标是用小改动修掉最明显的连续动作缺口，保持依赖、HTTP API 和队列语义基本不变。

### 1.1 Runtime 衔接点

- idle->user、standby->user、user 完成后->idle/standby、idle->idle、transition 中被 user 抢断、hold->next/standby 继续沿用现有状态机路径。
- 新增或泛化 internal user-to-user bridge：仅当普通非 hold GeneralTracker user A 正常结束、GeneralTracker waiting queue 非空、FIFO 队首 B 也是可 bridge 的 GeneralTracker user 时生效。bridge 的 source 是 A 末帧，target 是 B 首帧。
- user-to-user bridge 是内部过渡，不生成 run id，不写 history，不改变 queue FIFO，不跳过队首，不应用到 LocoUpper 或其他非 GeneralTracker 路径。
- B 经 internal user-to-user bridge 接入后，必须收缩或跳过 B 的二次 startup hold，避免 bridge 完成后再固定停 0.5s。建议用明确的 runtime context 标记，例如 `arrived_via_user_bridge`，只影响这个 target user。
- cold start、direct start、standby->user、background->user interrupt、safety 相关路径继续保守不动；standby->user 第一阶段明确不借此跳过 startup hold。
- 状态解释只在现有 status 可承载的范围内增加 transition context；如果现有 status 不适合，先 defer，不新增端点。

### 1.2 设计自适应 duration 估算

建议新增一个内部 helper，例如 `estimateSyntheticTransitionDuration(...)`。它接收 transition context、source/target reference frame，以及 runtime 调用侧可选传入的诊断/options 信号。不要把 `low_state` 硬塞进当前 `makeSyntheticTransitionTrk` 签名；需要实际状态时由 RuntimeControlLoop 调用侧先整理为 options 或诊断输入。

transition context 必须影响策略和 cap，不能所有 transition 共用一套上限：

- `background_to_user`：优先响应速度，使用较短 max cap；gap 很大时也只做必要桥接，避免用户动作启动被背景过渡拖住。
- `user_to_user`：兼顾连续性和响应，使用中等 max cap；重点避免 A/B 之间突跳和二次 startup hold 叠加。
- `return_to_standby_or_idle`：允许更慢、更保守的 max cap，因为它不阻塞用户显式动作。
- `background_to_background` 或 idle->idle：作为背景维护路径，保守即可，不为第一阶段引入复杂策略。

第一阶段估算信号限制为：

- `source/target joint_pos gap`：优先统计 upper-body 或参与 bridge 的关键 joints。可用 max abs delta 或 RMS delta，第一阶段建议 max abs + capped RMS，避免单点异常完全主导。
- `root yaw gap`：允许参与估算，但必须明确使用 root alignment 前的 yaw 或 runtime 诊断信号。root alignment 可能抹掉对齐后的 XY/yaw gap，不能在对齐后误以为 gap 已消失。
- `actual low_state upper-body q gap`：只作为保守拉长和诊断信号。当实际机器人状态可用、时间新鲜、维度可信时，比较 current q 与 target 首帧 q；不可用时跳过。它不修正 transition 起点，也不表示 actual q 会替换 reference source frame。
- `root xy gap`：第一阶段只记录和诊断，不驱动 duration，避免范围蔓延到位置/步态规划。
- `reference COM/body_pos gap`：可记录用于诊断；第一阶段不单独拉长 duration。

建议 duration 计算保持线性、可解释：

```text
raw_duration = context_min_duration_s
raw_duration = max(raw_duration, key_joint_gap / joint_rate_hint)
raw_duration = max(raw_duration, yaw_gap / yaw_rate_hint)
raw_duration = max(raw_duration, low_state_q_gap / conservative_q_rate_hint)  # if available
clamped_duration = clamp(raw_duration, context_min_duration_s, context_max_duration_s)
actual_duration = quantize_to_target_dt(clamped_duration, target_fps_or_policy_dt, min_transition_frames)
```

离散帧约束必须显式处理：

- transition 帧数按 target trk fps 或 policy dt 量化，不能只保留浮点 duration。
- 设置最少帧数，避免过短 transition 退化为首尾两帧甚至单帧跳变。
- 量化后的 `actual_duration` 写回 metadata 或等价诊断字段，便于测试和实机日志解释“请求 duration”和“实际 duration”的差异。

配置建议：

- 保留现有 `transition_duration_s`，语义固定为兼容 fallback：估算失败或 adaptive smoothing 关闭时使用该值；估算成功时不把它作为下限，否则旧 sim 配置的 0.7s 会继续拖慢 small gap。
- 新增配置保持最小：优先只加 context min/max 和少量 rate hints。若暂时不想扩配置，先用代码常量，并在测试中固定这些常量。
- 不引入 per-joint 策略 DSL，不让用户通过 HTTP API 调参。

异常处理：

- source/target 缺 frame、字段维度不匹配、duration 估算输入不可用时，回到 `transition_duration_s`。
- 估算结果必须按 context clamp 和量化，禁止生成接近 0 的 bridge，也禁止因为异常 gap 产生很长 transition。
- 如果 target 是空动作、非法动作或 validation 失败，不应由 duration 估算吞掉错误。

### 1.3 替换线性时间推进为平滑曲线

第一阶段保持插值对象不变，只替换时间参数：

- 原始归一化时间 `u = i / (N - 1)`。
- 用 `smootherstep(u) = u^3 * (u * (u * 6 - 15) + 10)` 或 `smoothstep(u) = u^2 * (3 - 2u)` 得到 `alpha`。
- `joint_pos`、`body_pos`、`ref_com` 使用 `alpha` 插值。
- quat 可继续 nlerp；如果已有可靠 quaternion 工具，再考虑局部替换为 slerp，但第一阶段不强求。
- velocity 继续由生成后的 position 有限差分得到，先保持实现简单；如测试显示边界速度仍突兀，再局部用曲线导数生成 reference velocity。
- `body_ang_vel` 继续保持当前策略，除非后续测试证明它是主要体感问题。

这只是启发式体感优化，不是满足速度、加速度、jerk limit 的轨迹生成器。smoothstep/smootherstep 的中段峰值速度高于平均速度；如果 rate hints 或 robot limit 参与计算，必须按曲线导数留 margin，并用测试检查 velocity、acceleration、jerk。

contact 处理建议保持保守：

- 第一阶段不做 contact-aware gait。
- 继续沿用现有中点切换，或只做最小清理以保证 duration 变化后仍确定。
- 不根据 contact 动态拉长/缩短 duration，避免第一阶段范围膨胀。

### 1.4 阶段 1 禁止扩张项

- 禁止引入 Ruckig 依赖。
- 禁止新增 HTTP API。
- 禁止改用户动作 trk 内容或批量重采样用户 trk。
- 禁止改 active user interrupt 的 controlled stop 语义。
- 禁止改变 LocoUpper 或非 GeneralTracker 的队列/启动路径。
- 禁止改变 queue FIFO。
- 禁止用 root xy 驱动 duration。
- 禁止实现 contact-aware gait 或 footstep planner。
- 禁止把 `/stop` 做成平滑停止。
- 禁止为了状态解释引入大型 UI 或 dashboard。

## 阶段 2 计划：实测仍需要时的 jerk-limited bridge

阶段 2 只有在阶段 1 的仿真和机器人实测仍显示明显 jerk、边界速度突变或关节冲击时再进入。它是内部 state-to-state transition bridge 的后续增强，不是当前必做项，也不是新的动作执行框架。

### 2.1 能力边界

- 可评估 Ruckig Community 版本适合的 state-to-state trajectory：输入 current/target position、velocity、acceleration，并受 max velocity、max acceleration、max jerk 约束。
- 不依赖 Pro-only tracking 或 intermediate waypoint 能力。
- 边界状态必须有明确来源：current/target position、velocity、acceleration 来自相邻帧有限差分和/或 runtime measured state。不能简单把 velocity/acceleration 全设 0；那可能制造新的拼接不连续。
- 只用于内部 synthetic transition bridge，不用于完整用户动作重规划，不改变用户 trk 本体。

### 2.2 集成策略

- 在一个 helper 内分派 smoother，例如 `simple|ruckig`；RuntimeControlLoop 只请求生成 transition trk，不感知 smoother 类型。
- 每个通道设置保守的 max velocity、max acceleration、max jerk，优先来自现有 policy/robot limit；没有明确来源时使用保守常量并标注需要实机校准。
- Ruckig 失败、输入维度不匹配、约束不可满足、计算超时或生成帧数异常时，自动回到阶段 1 simple smoother。
- 输出仍生成普通 synthetic transition trk，使后续 playback/policy 路径不需要知道 smoother 类型。
- 先做 offline generation，避免在线控制链路引入调度不确定性。除非有明确性能数据，否则不把 online Ruckig 放进实时路径。

### 2.3 阶段 2 禁止扩张项

- 禁止依赖 tracking/intermediate waypoint 作为必需能力。
- 禁止把 Ruckig 用于完整用户动作重规划。
- 禁止引入用户可上传 jerk profile 或复杂 per-joint 策略配置。
- 禁止为了 smoother 改变 `/stop`、`/passive`、controlled stop 的安全语义。
- 禁止在没有指标的情况下扩大到 contact/gait planner。

## 测试与验收

### 单元测试

- `trk_synthetic_transition_tests` 增加 duration 估算覆盖：
  - small gap 得到接近 context min 的 duration；只有估算失败或 adaptive smoothing 关闭时才回到 `transition_duration_s`。
  - large joint gap 得到更长 duration，并被对应 context max clamp。
  - 同一 gap 在 `background_to_user`、`user_to_user`、`return_to_standby_or_idle` 下使用不同 cap。
  - yaw wrap 使用最小角差。
  - root yaw 估算输入明确来自对齐前或诊断信号；对齐后的 root gap 不误用于判断。
  - root xy gap 只记录诊断，不改变 duration。
  - low_state 不可用时回退到 source/target gap；low_state 可用时只保守拉长，不改变 source frame。
  - 缺字段或维度异常时回退到 `transition_duration_s` 或触发现有 validation。
- 插值曲线测试：
  - alpha 起点为 0，终点为 1，单调递增。
  - smoothstep/smootherstep 中间值符合预期。
  - 更新现有锁定线性中间值、恒速 velocity 的测试期望；平滑曲线下中间值和 velocity 不再等同于线性插值/恒速。
  - 生成帧数随 adaptive duration 改变但首尾 frame 精确匹配 source/target。
  - 按 target fps 或 policy dt 量化帧数，满足最少帧数，并把实际 duration 写入 metadata 或等价诊断字段。
  - 对生成轨迹检查 velocity、acceleration、jerk 指标，至少能发现明显超过预期的情况。
- 配置测试：
  - 旧配置仍可加载。
  - `transition_duration_s` 作为兼容 fallback 的行为稳定。
  - context min/max 和 rate hints 未配置时使用代码常量或配置层回填值。
  - min > max 等非法配置有明确处理。

### Runtime 测试

- 覆盖现有 RuntimeControlLoop 路径不回退：
  - idle->user。
  - standby->user。
  - user 完成后->idle/standby。
  - idle->idle。
  - transition 中被 user 抢断。
  - hold->next/standby。
- 普通非 hold user A 正常结束且 B 已 waiting 的 current behavior regression：
  - 先锁定当前事实：A 结束后因 waiting 非空跳过 user->idle/standby transition，`finishActive` 进入 `GeneralTrackerIdle`，下一 tick `startNext` 启动 B，policy runtime 下 B 仍有固定 0.5s startup hold。
  - 实现 Phase 1 后，将该测试更新为新期望：A 末帧到 B 首帧生成 internal user-to-user bridge，且 B 不再经历重复的 0.5s startup hold。
- A->B bridge 测试：
  - 只对 GeneralTracker waiting 生效。
  - 不改变 FIFO，不跳过队首。
  - LocoUpper 或非 GeneralTracker target 不走该 bridge。
  - internal bridge 不生成 run id、不写 history。
- target user no duplicate startup hold 测试：
  - 仅 `arrived_via_user_bridge` 或等价 context 的 B 收缩/跳过 startup hold。
  - cold start、direct start、standby->user、background->user interrupt、safety 路径保持现有 startup hold/保守行为。
- 状态查询测试应能解释 transition context，但不把 internal transition 当作 user run。
  - 如果 Phase 1 defer 状态解释，则测试明确记录 defer，不新增状态端点。

### 产品验收

- 小动作衔接不会明显拖慢。
- 大动作衔接不会出现比当前更明显的突跳。
- 连续 GeneralTracker user queue 动作之间不应出现明显回 idle/standby 或固定停顿的体感。
- user interrupt 背景动作时，接管路径稳定且状态可解释。
- `/standby_velocity`、`/stop`、`/passive` 语义保持不变。
- internal transition 不出现在用户历史记录里。

### 实机/仿真观测指标

- transition 实际 duration 分布。
- 最大 joint delta、最大估算速度、最大估算加速度。
- root xy gap 诊断分布，但不作为 Phase 1 duration 决策依据。
- transition 起止 3-5 帧的 position/velocity 连续性。
- user-to-user queue 的动作间空档时长，包括 startup hold。
- user interrupt background 的响应时间。
- fallback 次数和原因。

## 风险与回滚

- **风险：duration 估算过短**  
  可能导致大 gap bridge 仍突兀。缓解方式是提高对应 context min 或降低 rate hints；回滚到固定 `transition_duration_s`。

- **风险：duration 估算过长**  
  可能让用户感觉动作启动慢。缓解方式是降低 max 或提高 rate hints；对 small gap 加更强 clamp。

- **风险：low_state 噪声影响估算**  
  只在 low_state 时间新鲜且维度可信时使用；否则跳过。必要时对 gap 做 cap，不对 low_state 做复杂滤波。

- **风险：平滑曲线改变中段速度**  
  smootherstep 起止更柔，但中段速度更高。需要检查 max velocity/acceleration 指标。若不满足，回退 smoothstep 或固定 duration。

- **风险：queue 间 startup hold 仍造成体感停顿**  
  Phase 1 必须处理 internal user-to-user bridge 后 target user 的二次 startup hold。若实测仍停顿，优先检查 bridge 调度、policy dt 量化和 no duplicate startup hold 测试，而不是扩大到 cold start 或 standby->user。

- **风险：Ruckig 集成复杂度超过收益**  
  阶段 2 必须 feature flag 可关闭，并保留 simple smoother fallback。Ruckig 失败不影响动作执行主路径。

回滚策略：

- 保留旧固定 duration 路径或通过配置关闭 adaptive duration。
- 保留 simple linear/smoothstep fallback。
- Ruckig/jerk-limited smoother 必须可关闭，关闭后回到阶段 1 行为。
- 安全路径 `/stop`、`/passive`、controlled stop 不参与回滚开关，因为计划不修改它们。

## Handoff Checklist

- [ ] 确认目标文件仍是开发计划，不写成已实现说明。
- [ ] 实现前再次阅读 `src/trk_synthetic_transition.cpp` 和 `include/agentic_et1_tracker/trk/synthetic_transition.hpp`，确认当前接口边界。
- [ ] 实现前检查 `RuntimeControlLoop` 中 transition 创建点，避免复制 duration 估算逻辑。
- [ ] 明确 `transition_duration_s` 在阶段 1 只作为兼容 fallback。
- [ ] 为 adaptive duration 定义 context min/max 和 rate hints，或先固定为代码常量并说明选择依据。
- [ ] 加单元测试覆盖 small gap、large gap、yaw wrap、root xy 只诊断、low_state missing、clamp、兼容 fallback 路径。
- [ ] 加 Runtime regression 锁定当前普通非 hold A+B 行为，再在 Phase 1 实现后更新为 user-to-user bridge 期望。
- [ ] 加 A->B bridge 只对 GeneralTracker waiting 生效、不改 FIFO、不影响 LocoUpper/非 GeneralTracker 的测试。
- [ ] 加 target user no duplicate startup hold 测试，并确认 cold start、direct start、standby->user 保守不动。
- [ ] 保证 HTTP API 不变。
- [ ] 保证 internal transition 不生成 run id、不进 history。
- [ ] 若状态解释无法在现有 status 内承载，明确 defer，不新增端点。
- [ ] 保证 `/standby_velocity`、`/stop`、`/passive` 语义不变。
- [ ] 阶段 2 只有在实测仍需要时才引入；引入前确认版本、许可证、构建方式和 Community 能力边界。
- [ ] 阶段 2 必须可关闭，失败自动回退 simple smoother。
- [ ] 实机前在仿真记录 duration、velocity、acceleration、fallback reason。
- [ ] 实机灰度时保留快速回滚配置。
