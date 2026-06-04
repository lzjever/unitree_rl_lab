# Idle Transition Smoothing Handoff Plan

本文档只规划 `agentic-et1-tracker` 后续开发，不修改运行代码。范围固定为：

1. synthetic transition 太快，增加全局受控配置 `transition_duration_s`。
2. 补齐 `idle -> idle` 插值。
3. 补齐 `idle -> user` reference-level 插值。
4. 保持 `/stop` 和 passworded `/passive` 立即中断，不走插值。

原则：KISS/DRY/YAGNI。只复用现有 synthetic transition 与状态机能力，不扩大 API 或动作格式。

## 背景

当前 runtime 已有内部 synthetic reference transition，能在 user 完成后过渡到 standby/idle，也能从 holding user 过渡到下一个 user。但 transition 时长硬编码为 `0.30s`，缺少全局配置入口；同时 idle 相关路径仍存在两个跳变：

- `idle -> user`：用户 `/execute` 抢占 active idle 时，当前逻辑先停 idle，再直接进入 user 准备/播放，缺少从 idle 当前 reference 到 user 首帧的 reference-level 插值。该项是 GA 必修。
- `idle -> idle`：idle 动作播完后回到 idle 状态，再直接启动下一个 idle 动作，缺少内部平滑。这是内部平滑修复，不引入 idle 调度、权重、优先级或 transition profile。

目标是把这些跳变统一收敛到现有内部 transition 机制，同时保持控制/安全命令的立即性。

## 当前代码证据（文件/行号）

- Synthetic transition 时长硬编码：
  `unitree_rl_lab/deploy/robots/agentic-et1-tracker/src/runtime_control_loop.cpp:20`
  定义 `kSyntheticTransitionDurationS = 0.30`。
- 两处实际使用该硬编码：
  `src/runtime_control_loop.cpp:1316-1320` 和 `src/runtime_control_loop.cpp:1508-1512`
  调用 `makeSyntheticTransitionTrk(..., kSyntheticTransitionDurationS)`。
- Synthetic transition 本身已经是内存 `.trk`，接口接受 `duration_s`：
  `include/agentic_et1_tracker/trk/synthetic_transition.hpp:9-13`。
  实现按 `target_fps * duration_s` 算帧数，并对 joint/body/ref 做插值：
  `src/trk_synthetic_transition.cpp:261-298`。
- `RuntimeConfig` 目前没有 transition duration：
  `include/agentic_et1_tracker/runtime/runtime_config.hpp:7-12`。
  `AppConfig` 目前有 `stop_hold_s`、`idle_mode`、`passive_password` 等全局项：
  `include/agentic_et1_tracker/app/app_config.hpp:36-55`。
- 配置解析已有正数/非负 double helper 和顶层字段解析模式：
  `src/app_config.cpp:121-165`、`src/app_config.cpp:372-390`。
- idle 直接启动 user/idle 播放，没有 transition：
  `src/runtime_control_loop.cpp:708-734` 的 `startIdle()` 直接 `MotionRequest -> MotionRequest event`；
  `src/runtime_control_loop.cpp:677-687` 的 `startNext()` 直接把 waiting user 设为 active user。
- active idle 被 user queue 抢占时当前是立即停 idle：
  `src/runtime_control_loop.cpp:253-257`；
  interrupt 路径同样先 `stopIdleActive()` 后入 waiting：
  `src/runtime_control_loop.cpp:447-457`。
- user 完成后已有 `user -> idle` 和 `user -> standby` transition：
  `src/runtime_control_loop.cpp:1073-1077`、`src/runtime_control_loop.cpp:1357-1440`、
  `src/runtime_control_loop.cpp:1443-1492`。
- transition 是内部状态，不是用户 run：
  `src/runtime_control_loop.cpp:1603-1621` 创建 transition active；
  `src/runtime_control_loop.cpp:2406-2458` 将 `active.kind` 暴露为 `transition` 且 `id` 为空；
  `src/json_codec.cpp:82-87` 只有 `ActiveKind::User` 才暴露 id。
- reference sink 已支持 user 和 transition 发布：
  `src/runtime_control_loop.cpp:2363-2378` 发布 user reference；
  `src/runtime_control_loop.cpp:2380-2394` 发布 transition reference。
- `/stop` 和 `/passive` 已是立即控制入口：
  HTTP route 在 `src/http_server.cpp:102-105`；
  API `/stop` 只接受空 body 并直接提交 stop：`src/api_service.cpp:448-460`；
  passworded `/passive` 校验后直接提交 passive：`src/api_service.cpp:463-485`。
- runtime stop/passive 对 transition 是 abort/clear，不等待插值：
  `/stop` 遇到 `ActiveKind::Transition` 调 `abortTransition()` 并进入 stopping：
  `src/runtime_control_loop.cpp:315-320`；
  passive 控制清空 idle、abort transition、clear reference 并进 Passive：
  `src/runtime_control_loop.cpp:350-372`。
- RuntimeBridge 是运行时命令优先级和 stop watermark 的权威层：
  `src/runtime_bridge.cpp:144` 的 `RuntimeBridge::consumeNextCommand()` 选择最高优先级命令，并在 Stop 被消费时取消/清理 stop sequence 之前的 queued motion；
  `src/runtime_bridge.cpp:169` 的 `RuntimeBridge::priority()` 定义 Stop 最高，Passive/FixStand/StandbyVelocity 次之，高于 Interrupt/Queue/IdleConfig。
  `CommandMailbox` 只能视为 core legacy/secondary boundary，不作为本计划的 stop watermark 权威引用。

## 产品/API 决策

- 新增唯一全局 runtime/config 配置：顶层 YAML `transition_duration_s`，传入 `RuntimeConfig::transition_duration_s`。
- `transition_duration_s` 不是 HTTP/API 合同，不出现在 `/execute`、`/idle`、`/stop`、`/passive` request/response schema 中。
- GA 默认值必须保持现有行为：`0.30` 秒。不要把默认值改成 `0.60`。
- `0.60` 秒只能作为 MuJoCo/operator 调参候选值，通过配置显式覆盖；它不是 GA 默认。
- 配置必须做 bounded positive 校验，或在 runtime 使用前做局部校验。建议 `0 < transition_duration_s <= 5.0`，非法值 fail-fast。
- 不增加 `/execute`、`/idle` 或任何 per-request HTTP 参数。
- 不改变 `/stop` 空 body 合同。
- 不改变 passworded `/passive` body 合同。
- 不改 `.trk` 格式，不写临时 transition 文件。
- Synthetic transition 继续是内部 active：不生成用户 run id，不进 queue，不消耗 `queue.limit`，不可 `GET /status?id=...` 查询。
- `transition.target` 继续只使用现有 `"user" | "idle" | "standby"`，不新增公开 target 类型。

## 实现计划

1. 配置最小改造
   - 在 `RuntimeConfig` 增加 `double transition_duration_s{0.30};`。
   - 在 `AppConfig` 增加同名顶层字段，并在 `loadAppConfig()` 解析后赋给 `config.runtime.transition_duration_s`。
   - 增加 bounded positive 校验。优先在 `loadAppConfig()` fail-fast；若部分测试或工具直接构造 `RuntimeConfig` 绕过 app config，则在 runtime 使用点增加局部 validation guard。非法值必须 reject/fail-fast，或让本次 transition 安全失败并保持状态一致；不得静默改写或截断配置值。
   - config example/template 可在实现 PR 中按需补一行注释或显式值；不要引入嵌套 transition 配置对象。

2. 替换硬编码
   - 删除或停止使用 `kSyntheticTransitionDurationS`。
   - 在所有 `makeSyntheticTransitionTrk()` 调用处使用已校验的 `config_.transition_duration_s`。
   - 不改 `makeSyntheticTransitionTrk()` API，避免扩散。

3. DRY 出统一 transition 启动路径
   - 保留 `makeSyntheticTransitionTrk()` 作为唯一插值生成器。
   - 尽量复用 `PendingTransition`、`startInternalTransition()`、`publishReferenceTransition()`。
   - 如果需要新增 helper，只做一层薄封装，例如：
     `startSyntheticTransitionFromFrame(source_frame, target, target_frame, target_fps)`。
   - 避免复制 `startTransitionFromCompletedUserToIdle()` 的大段 load/target 逻辑。

4. 补齐 GA 必修 `idle -> user`
   - 当 active idle 收到用户 queue/interrupt 时，不直接 `stopIdleActive()` 后 `startNext()`。
   - 若当前 idle active 有 `active_track_` 和当前 frame，则加载 user target track，构建 `TransitionTargetKind::User`，从 idle 当前 frame 插值到 user 首帧。
   - target load 失败或 target 首帧缺失时，必须发布该 user run `Failed`，不得静默丢失，也不得回退成 idle 继续播放。
   - 只有缺 source frame 时，才允许回退到现有直接启动 user 路径。
   - transition target user 的 `target_id`、`target_state=queued` 继续按现有 public contract 暴露。
   - queue 与 interrupt 对 active idle 都视为用户抢占，目标 user 不排在 idle 后面。

5. 补齐内部 `idle -> idle`
   - 在 active idle 到达最后一帧时，如果 idle pool 仍非空、没有 waiting user，则选择下一个 idle motion。
   - 以当前 idle active frame 作为 source，以下一个 idle track 第 0 帧作为 target，启动内部 synthetic transition。
   - transition 完成后进入目标 idle active，仍不生成 run id、不写 user history。
   - 不引入 idle 调度、权重、优先级、transition profile、per-idle 参数。
   - transition 构建失败时必须满足三点：
     不卡在 `ActiveKind::Transition`；不写 user history；不 tight-loop 重试同一个失败 transition。
   - 简单 fallback 建议：停止当前 idle，回到 `GeneralTrackerIdle`，跳过本 tick 的 idle auto-start；下一 tick 再按现有 idle pool 逻辑尝试。必要时推进 `idle_next_index_`，避免同一坏目标连续 tight-loop。

6. 保持即时中断硬约束
   - `CommandKind::Stop` 和 `CommandKind::Passive` 不进入任何 smoothing helper。
   - `/stop` 和 passworded `/passive` 在 active user、active idle、active transition 中都必须立即 abort/clear，不等待 `transition_duration_s`。
   - 不改变 RuntimeBridge stop watermark 语义；`RuntimeBridge::consumeNextCommand()` 中 Stop 消费时对 stop sequence 之前 pending/queued motion 的取消清理必须不变。
   - 不改变 RuntimeBridge priority 语义；Stop 必须继续最高优先级，Passive/FixStand/StandbyVelocity 必须继续高于 Interrupt/Queue/IdleConfig。
   - `/fixstand`、`/standby_velocity` 可继续按当前控制命令语义 abort transition；本计划不把它们改成平滑过渡。

## TDD 测试计划

- `tests/app_config_tests.cpp`
  - 默认配置包含 `transition_duration_s == 0.30`，且 runtime 同步。
  - 显式 YAML `transition_duration_s: 1.0` 可解析并同步到 runtime。
  - `0`、负数、NaN/非数值、超过上限值被拒绝。

- `tests/runtime_control_loop_tests.cpp`
  - 配置 `transition_duration_s=1.0`、target fps 50 时，transition frames 应按 `ceil(duration * fps) + 1` 变长。
  - active idle 中途收到 user queue，进入 `transition.target:"user"`，`target_id` 为用户 run id，reference sink 第 0 帧等于被抢占 idle 当前 reference。
  - active idle 中途收到 user interrupt，同样从当前 idle frame 起 transition，并且不留下 idle run history。
  - idle->user target load 失败或首帧缺失时，对应 user run 发布 `Failed`。
  - idle->user 缺 source frame 时允许回退现有直接启动路径，并有测试锁住该 fallback。
  - active idle 播放完成后进入 `active.kind:"transition"`、`transition.target:"idle"`，完成后进入下一个 idle active。
  - `idle -> idle` 不产生 run history，不改变 user queue。
  - `idle -> idle` transition 构建失败后不处于 Transition、不写 user history、不在同一 tick tight-loop 重试。
  - transition 期间 `/stop` 立即 abort，`transition.active=false`，reference cleared，idle config 按现有 stop 语义清空，并保留 stop watermark 行为。
  - transition 期间 passworded `/passive` 立即 abort/clear，进入 Passive，idle config 清空。

- `tests/api_tests.cpp`
  - `/execute` body 里出现 `transition_duration_s` 仍返回 `REQUEST_INVALID`。
  - `/idle` body 里出现 `transition_duration_s` 仍返回 `REQUEST_INVALID`。
  - `/stop` 非空 body 仍 rejected；passworded `/passive` 合同不变。
  - API schema 负例保留在 API 层即可，不要求 `http_server_tests.cpp` 重复覆盖。

- `tests/trk_synthetic_transition_tests.cpp`
  - 保持现有插值策略测试；不新增 easing/adaptive/contact 策略测试。

## 验收计划

- 构建并跑 tracker 单测：
  `cmake --build build-agentic-et1-tracker-robot --target agentic_et1_tracker_core_tests agentic_et1_tracker_runtime_tests agentic_et1_tracker_api_tests agentic_et1_tracker_app_tests`
- 跑相关 ctest：
  `ctest --test-dir build-agentic-et1-tracker-robot --output-on-failure -R "agentic_et1_tracker_(core|runtime|api|app).*tests"`
- 手工/仿真验收只看行为：
  - 默认配置下 synthetic transition 仍约 `0.30s`。
  - 显式配置 `transition_duration_s: 0.60` 可作为 MuJoCo/operator 调参候选值观察更平滑效果。
  - `GET /status` during smoothing 显示 `active.kind:"transition"`、`id:null`。
  - idle 播放中发 `/execute`，先看到 `transition.target:"user"`，随后 user run 从 frame 0 开始。
  - idle pool 两个动作连续播放时，中间先出现 internal transition。
  - `/stop` 和 passworded `/passive` 在 transition 任意帧都立即清空 reference，不等待 `transition_duration_s`。

## 风险与非目标

风险：

- transition duration 被 operator 调长会让用户动作启动更晚；GA 默认必须保持 `0.30`。
- idle pool 中短动作连续播放时，transition 占比可能偏高；先接受全局值，不做 adaptive。
- loader 失败路径要避免卡在 `transition`、丢失 user run 状态或 tight-loop。
- reference sink 异常必须继续被吞掉，不能改变运动状态。
- 直接构造 `RuntimeConfig` 的测试或工具可能绕过 app config 校验；实现时需要 bounded positive 的局部 validation guard。非法值必须 reject/fail-fast，或让本次 transition 安全失败，不能静默改写或截断配置值。

明确非目标：

- 不增加 per-request HTTP 参数。
- 不把 `transition_duration_s` 暴露为 HTTP/API 合同。
- 不做 adaptive duration。
- 不做 easing 曲线。
- 不做 contact-aware 高级策略。
- 不做 idle 调度、权重、优先级或 transition profile。
- 不改 `.trk` 格式。
- 不把 synthetic transition 暴露成用户 run。
- 不改 ET1 app。
- 不改控制策略、policy model、deploy config 语义。
- 不改变 RuntimeBridge stop watermark 或 RuntimeBridge priority 语义。`CommandMailbox` 如仍存在，只能作为 secondary/core legacy boundary，不作为行为权威。
- 不引入新的公开状态机大改或新错误码，除非测试证明无法表达。

## Handoff Checklist

- [ ] 在实现前确认只改 `agentic-et1-tracker`，不改 ET1 app。
- [ ] 先写/更新配置和 runtime 行为测试，确认失败。
- [ ] 加 `RuntimeConfig::transition_duration_s{0.30}` 与 `AppConfig` 解析。
- [ ] 加 bounded positive 校验；runtime 局部 validation guard 必须 reject/fail-fast 或安全失败 transition，不能静默改写或截断配置值。
- [ ] 替换 `kSyntheticTransitionDurationS` 硬编码。
- [ ] DRY 出从任意当前 reference frame 启动 internal transition 的最小 helper。
- [ ] 实现 GA 必修 `idle -> user` reference-level synthetic transition。
- [ ] 实现内部 `idle -> idle` synthetic transition。
- [ ] 锁定 fallback：idle->user target 失败发布 user failed；idle->idle 失败不 Transition、不 history、不 tight-loop。
- [ ] 保持 `/stop`、passworded `/passive` 走立即 abort/clear 路径。
- [ ] 确认 RuntimeBridge stop watermark 与 RuntimeBridge priority 语义未改变。
- [ ] 确认 `/execute`、`/idle` 不接受 transition 参数。
- [ ] 跑单测和 ctest 验收命令。
- [ ] 手工检查 `/status`：transition 无用户 id、无 queue 占用、无 run history。
