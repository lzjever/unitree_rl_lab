# Hold Interrupt Handoff Fix Plan

本文档是 `agentic-et1-tracker` 的开发 handoff 计划，用于修复
`hold:true` 用户 `.trk` 在 `holding` 后收到新 user run 时的交接缺陷。

范围只限：

- `deploy/robots/agentic-et1-tracker` runtime 内部 `Holding -> user` handoff。
- `packaging/skills/et1-action` 输出合同的最小修正。
- 对应 unit tests、skill tests、manual/sim e2e gate。

本文件只是计划文档，不代表代码已经修复。

## 1. 当前问题

用户手测发现：

1. 执行 `hold=true` 的 user `.trk`。
2. 旧 run 到达 `progress=1.0`、`state=holding`。
3. 再通过 `et1-action run-trk` 提交新动作。skill 默认使用 tracker
   `mode:"interrupt"`，并返回 `ok:true` 和新 run id。
4. tracker 没有切换到新动作：
   - 旧 hold run 仍然 active 且 `state=holding`。
   - 新 run 变成 `failed`，错误为 `INTERNAL_ERROR`。
   - queue 为空。
   - `transition.target=user` 没有启动。

已确认正常的相邻路径：

- active running user interrupt 正常。
- active transition early interrupt 正常。
- queued user 被 interrupt 取消正常。
- holding -> standby 正常。

因此问题不是 skill 没有提交 interrupt，也不是 HTTP/API mode 选择错误；问题在
runtime 对 `MotionState::Holding` 的 user handoff 路径。

## 2. 根因

当前 `RuntimeControlLoop::handleInterrupt()` 对 `Holding` 没有走 running
interrupt 的 robust handoff：

1. `handleInterrupt()` 先 `cancelWaiting(StopReason::Interrupt)`。
2. 对 holding user，它把新 request 放入 `waiting_`，然后保持旧 active
   holding 并返回。
3. 下一 tick，`advanceHolding()` 或 `advanceHoldingWithPolicy()` 看到
   `waiting_.front()`，调用 `startTransitionFromHoldingToNextUser()`。
4. `startTransitionFromHoldingToNextUser()` 先 `pop_front()` 新 request，再用
   held final frame 直接走 `startSyntheticTransitionFromActiveFrame()`。
5. `startSyntheticTransitionFromFrame()` 在 target align、yaw residual、contact、
   velocity、acceleration、jerk 等约束不满足时，会把 target request 发布为
   `Failed/InternalError`。
6. 旧 held source 只有在 internal transition 成功启动后才会 terminal；因此
   transition 未启动时，新 run failed，旧 run 继续占 foreground holding。

running interrupt 正常，是因为 `tryStartRunningUserInterruptHandoff()` 已经有
更稳健的路径：

- 先加载 target。
- 先试 `makeUserTransitionTracks()`。
- 再试 `makeUserTransitionTracksFromControllableSource()`。
- 成功后才 `startInternalTransition()`。
- benign transition reject 不会提前把合法 target 标成 failed。
- safety/fatal terminal 有明确处理。

holding->queue 也有同类缺陷，因为它同样经过
`advanceHolding()` -> `startTransitionFromHoldingToNextUser()`。holding->standby
不是同类问题：它有专用 controllable standby source。holding->idle 没有直接边。
holding->passive/fixstand/urgent_stop 走控制/停止路径，不依赖 user synthetic
transition。transition->holding 不是直接边。

## 3. 产品心智

`Holding` 是 active-like foreground user intent，不是 idle，也不是 done 后的
空闲状态。

| 用户意图 | API / skill 行为 | 产品语义 |
| --- | --- | --- |
| 新动作、改做另一个动作 | `run-text` / `run-trk` 默认 `mode:"interrupt"` | 替换当前 foreground user intent。 |
| 明确“排在后面/做完再...” | `--mode queue` | 只有用户明确 append 才排队。 |
| 普通停下、回待命 | `standby` / `POST /standby` | 普通停下，保留 idle 配置，不是急停。 |
| 紧急停止、abort、kill | `urgent-stop --urgent` / `POST /urgent_stop` | 急停，清 idle，不承诺 smoothing。 |
| 显式 passive | passworded `POST /passive` | safety sink，不是普通停止。 |
| fixstand | `POST /fixstand` | recovery/preparation pose，不是普通停止。 |

最小 terminal 选择：

- holding + interrupt：旧 held run 标为 `Stopped`，`stop_reason=Interrupt`。
- holding + queue：旧 held run 已自然到达末帧；transition 成功启动时保持现有
  queue 语义，标为 `Done`，`stop_reason=None`。
- holding + invalid target：target 标 `Failed`；旧 held source 保持 holding，不
  释放 foreground。
- holding + valid target 但 direct + controllable-source handoff 都无法构造：
  必须进入确定 fallback，不能每 tick 重试卡住。
  - interrupt：旧 source 离开 foreground 时 `Stopped/Interrupt`，controlled
    stop/standby 后启动 replacement target。
  - queue：旧 source 视为已完成 `Done/None`，target 继续通过 no-active /
    standby->user 路径启动。
- holding + standby：保持现有语义，走 dedicated standby handoff。

不要把 `Holding` 当成 idle/background。interrupt 必须替换当前 foreground
intent；queue 只能来自用户明确 append。

## 4. KISS / DRY / YAGNI Guardrails

| 原则 | 本计划约束 |
| --- | --- |
| KISS | 服务端优先修复 `Holding -> user` handoff；公开 API 保持 `queue/interrupt` 两种 user run mode。 |
| DRY | 抽出 running interrupt 已有的“从当前 reference 到新 user”的 robust handoff，供 running、holding queue、holding interrupt 复用。 |
| YAGNI | 不新增 `smooth_interrupt`、priority queue、playlist、delete/reorder、per-request transition profile、dashboard 或新调度 daemon。 |
| 安全优先 | readiness、bad orientation、fault、model/write error 仍高于 handoff 平滑性。 |
| Skill 边界 | skill 不做“检测 holding 后 standby/retry”的 workaround；skill 只提交用户意图并正确表达 accepted/confirmed。 |

明确不做：

- 不改 `.trk` 格式。
- 不新增 HTTP API。
- 不扩大到 ET1 app、loco upper executor 或策略训练。
- 不把 queue-ahead/sequence 下沉成 tracker playlist。
- 不让 agent 选择内部 handoff profile。

## 5. 状态转换矩阵

| Source | Event | 期望行为 | Source terminal | Target 结果 |
| --- | --- | --- | --- | --- |
| running GeneralTracker user | user interrupt | 走 robust current-reference handoff；成功进入 `transition.target=user`；benign reject fallback controlled stop/restart。 | `Stopped/Interrupt` | transition 后 running；不得因 benign reject 提前 failed。 |
| holding GeneralTracker user | user interrupt | 取消 waiting；从 held reference 用同一 robust helper 到 replacement user。 | 成功启动 transition 后 `Stopped/Interrupt` | 新 id 不 failed；queue empty；`transition.target=user` 或新 active user。 |
| holding GeneralTracker user | user queue | 保留 FIFO append 语义；从 held reference 用同一 robust helper 到 next user。 | 成功启动 transition 后 `Done/None` | 新 id 不 failed；transition 后 running。 |
| holding A + queued B | interrupt invalid C | interrupt 先取消 B；C load/validation failed。 | A 保持 `Holding` | B `Canceled/Interrupt`；C `Failed`；queue empty。 |
| holding A | queue invalid B + queued valid C | B load/validation failed and consumed；C 留在 queue 等下一轮。 | A 在 C transition 成功前保持 `Holding` | B `Failed`；C 不丢失、不被 canceled。 |
| holding GeneralTracker user | valid target but direct + controllable-source both cannot build, non-safety | 不把合法 target 标 `INTERNAL_ERROR`；走确定 fallback，不允许每 tick 重试卡住。 | interrupt: 离开 foreground 时 `Stopped/Interrupt`；queue: `Done/None` | interrupt target 经 controlled stop/standby 后启动；queue target 经 no-active/standby->user 启动。 |
| holding GeneralTracker user | target start/readiness/safety/fatal failure | safety/fault 优先；不得继续 holding 假装可 handoff。 | 按 safety path | target failed；active 进入 passive/fault 或现有 safety terminal。 |
| holding GeneralTracker user | standby | 继续使用 holding->standby dedicated controllable source。 | 现有语义 | final standby；不走 user helper。 |
| holding GeneralTracker user | urgent_stop | 走控制/停止路径。 | 现有 urgent stop 语义 | 不 smoothing，清 idle。 |
| holding GeneralTracker user | passive/fixstand | 走 safety/recovery control path。 | 现有 control 语义 | 不依赖 user synthetic transition。 |
| transition target user | user interrupt | 保持现有 current transition frame -> replacement user。 | 旧 target canceled/stopped | 回归锁定，不受本修复破坏。 |
| transition target idle/standby or standby playback | user queue / interrupt | 背景 transition/playback 仍从 current reference 抢占到 user；interrupt 取消本地 waiting。 | 背景 target 不写 user history | 回归锁定，不受 holding 修复破坏。 |

## 6. 实现计划

### 6.1 Runtime helper

主改动文件：

- `src/runtime_control_loop.cpp`
- `include/agentic_et1_tracker/runtime/runtime_control_loop.hpp`
- `tests/runtime_control_loop_tests.cpp`

建议把 `tryStartRunningUserInterruptHandoff()` 中通用的 target handoff 逻辑抽成
一个私有 helper，例如：

```cpp
enum class UserHandoffResult {
  Started,
  NoTransition,
  TargetFailed,
  SafetyTerminal,
};
```

helper 输入建议包括：

- source `TrkFrameView`。
- target `MotionRequest`。
- source completion state/reason/error。
- optional entry low state。
- 是否允许 publish target failure。
- caller kind：running interrupt、holding interrupt、holding queue。该值只影响
  failure mapping 和 fallback，不改变公开 API。

helper 负责：

1. 规范化 target request 为 `Queued`、`frame=0`、`err=Ok`、
   `stop_reason=None`。
2. load target `.trk`，填充 `frames/fps/duration_s`。
3. 对 target track 做 align / transition 构造：
   - 先试 `makeUserTransitionTracks()`。
   - 再试 `makeUserTransitionTracksFromControllableSource()`。
4. 成功后构造 `PendingTransition`：
   - `target_kind=User`
   - `target_id=request.id`
   - `target_state=Queued`
   - `target_request=target_request`
   - `target_track=aligned target`
   - `source_completion_state/reason/error` 由 caller 传入
5. 调用 `startInternalTransition()` 成功后，才发布 source terminal status。
6. 对 safety/fatal start failure 保持现有 fault/passive 处理。

helper 返回合同必须保护 running interrupt 现有行为：

- running wrapper 调用 shared helper 时禁用 target failure publication。
- running 下 target load/validation failure、direct-build benign failure、
  controllable-source benign failure 都映射为 `NoTransition` / `Fallback`，交给
  running 现有 controlled stop/restart 或旧 failure path 处理。
- running 下只有 readiness failure、fatal transition start failure、已经进入
  safety/fault 的情况返回 `SafetyTerminal`。
- holding caller 可以 publish target failure，但只限 invalid target 或
  safety/fatal terminal；合法 target 的 benign handoff reject 不得发布
  `InternalError`。

helper 不应该：

- 用 `startSyntheticTransitionFromActiveFrame()` 当 probe，因为它会在 benign
  build reject 时发布 target `Failed/InternalError`。
- 在 transition 成功前 `pop` 后丢失 target。
- 在 transition 成功前 terminal source holding。
- 在 benign direct synthetic build reject 时 fail 合法 target。
- 修改 `RuntimeBridge`、HTTP schema 或 queue mode。

### 6.2 Holding interrupt path

推荐最小改法：

1. 在 `handleInterrupt()` 中，保留 `cancelWaiting(StopReason::Interrupt)`。
2. 对 active holding GeneralTracker + GeneralTracker target，直接调用
   holding handoff helper，source completion 使用：
   - `MotionState::Stopped`
   - `StopReason::Interrupt`
   - `ErrorCode::Ok`
3. helper `Started` 或 `SafetyTerminal` 时直接 return。
4. target invalid 时 fail target，旧 holding source 保持 active holding，直接
   return。
5. safety/readiness/fatal failure 时，replacement target 必须发布 `Failed`，error
   使用实际 readiness/fatal error；active/source 进入 passive/fault 或现有
   safety terminal；不能继续 holding，也不能把 target 留在 queue 里重复处理。
6. 如果 helper 返回 benign `NoTransition`，不得把 target 标 failed，也不得把
   target 无限排在 holding 后面。必须走确定 fallback：
   - 旧 source 进入 controlled stop/standby 路径，并在离开 foreground 时发布
     `Stopped/Interrupt`。
   - replacement target 保留为 post-stop work，controlled stop/standby 完成后按
     no-active/standby->user 路径启动。
   - 公开 `queue.ids` 不应表现为“replacement target 排在 holding 后面”；如果内部
     复用 waiting 存储 post-stop work，也必须先让 source 离开 foreground。
7. 不允许每 tick 重试同一个 impossible handoff；一次 direct +
   controllable-source 均失败后必须进入 fallback 或 safety terminal。

注意：`MotionRequest` 当前没有保留原始 `MotionMode`。如果仍然只把 interrupt
request 放进 `waiting_`，`startTransitionFromHoldingToNextUser()` 后续无法可靠
区分 source terminal 应该是 `Stopped/Interrupt` 还是 queue 的 `Done/None`。因此
holding interrupt 最好在 `handleInterrupt()` 内直接启动，或在内部 pending 结构中
显式保存 source completion 语义。不要为此扩展公开 API。

### 6.3 Holding queue path

`startTransitionFromHoldingToNextUser()` 也改为使用同一个 helper：

1. 只在 helper 成功、target invalid、safety terminal，或决定进入确定
   no-active/standby fallback 后，从 `waiting_` 消费 target。
2. 对 queue source completion 传：
   - `MotionState::Done`
   - `StopReason::None`
   - `ErrorCode::Ok`
3. target invalid 时发布 target failed，并消费该 invalid target；source 继续
   holding；后续 valid queued target 保留并在下一轮尝试。
4. benign `NoTransition` 时不得保留 target 让它每 tick 重试卡住，也不得发布
   target `InternalError`。必须把旧 source 视为已完成 `Done/None`，释放
   foreground，并让 target 作为 post-holding work 继续通过
   no-active/standby->user 路径启动。
5. safety/readiness/fatal failure 时必须定义并测试：
   - target 发布 `Failed`，error 使用实际 readiness/fatal error。
   - active/source 按现有 safety/fault path 进入 passive/fault 或相应 terminal。
   - 不能继续 holding，也不能把 failed target 留在 queue 里重复处理。

这会修复同类的 holding->queue target `INTERNAL_ERROR` 丢失问题，同时保持 queue
是用户明确 append 的产品合同。

### 6.4 Running interrupt regression

`tryStartRunningUserInterruptHandoff()` 可保留为薄 wrapper，调用新 helper并继续
提供 running 专属 fallback：

- `Started`：return。
- `SafetyTerminal`：return。
- `TargetFailed`：running wrapper 正常不应由 shared helper 直接发布；invalid
  target 必须保持 running 现有 failure/fallback path。
- `NoTransition`：走现有 controlled stop/restart fallback。

不要让 running interrupt 因抽 helper 失去这些现有能力：

- benign reject fallback。
- no duplicate waiting copy。
- source `Stopped/Interrupt`。
- target 不因 benign reject failed。
- safety/readiness/fault 优先。
- invalid target 的旧行为不漂移：新增回归测试
  `running interrupt invalid target keeps old failure path`。

## 7. Skill 最小合同修正

主改动文件：

- `packaging/skills/et1-action/scripts/et1-action`
- `packaging/skills/et1-action/references/output-contract.md`
- `packaging/skills/et1-action/tests/test_et1_action.py`

skill 当前提交 interrupt 是正确的，不要加 holding 检测、standby、retry 或
fallback workflow。

需要修正的是输出合同：

- direct `run-trk` / `run-text` 非 `--wait` 成功提交后，`ok:true` 只代表
  tracker accepted / submitted，不代表动作已经完成，也不保证已经 running。
- 非 `--wait` 成功输出建议改为：
  - `accepted:true`
  - `confirmed:false`
  - `state` 使用 tracker 返回的 state，通常是 `queued`
  - 保留 `active.run_id`
  - 可增加或保留 `tracker_state` 作为明确信息
- 不再硬编码 `state:"running"`。
- `--wait` 到 `state:"done"` 或 `state:"holding"` 时，输出
  `accepted:true`、`confirmed:true`，`state` 使用最终 tracker state。
- `--wait` 模式如果 run 后续变成 `failed/canceled/stopped` 且不是成功终态，应
  返回 `ok:false`，`error.code:"TRACKER_RUN_FAILED"`。
- `--wait --hold` 到 `state:"holding"` 应视为 wait 成功，因为 hold 的成功状态
  就是持续 holding。

这只是 CLI/output contract 修正，不改变 tracker API，不改变默认
`mode:"interrupt"`。

## 8. TDD 测试计划

核心功能开发必须先写失败测试，再实现。

### 8.1 Runtime unit tests

推荐位置：`tests/runtime_control_loop_tests.cpp`。

| Test | 关键断言 |
| --- | --- |
| holding GeneralTracker interrupt starts user transition | 旧 run 先到 `holding`；interrupt 新 run 后进入 `active.kind=transition`、`transition.target=user`、`target_id=new id`；queue empty；旧 run `Stopped/Interrupt`。 |
| holding interrupt uses controllable-source fallback | 构造 held final frame 与 target 首帧 direct synthetic 不可行但 controllable source 可行；新 run 不 failed；transition started。 |
| holding queue uses same fallback | queue target 使用同一 fallback；旧 hold 在 transition started 后 `Done/None`；target 不 failed。 |
| holding interrupt cancels queued ids | holding A + queued B + interrupt C；B `Canceled/Interrupt`；C 成为 target；queue empty。 |
| holding interrupt invalid target preserves held source and fails target | holding A + queued B + interrupt invalid C；B `Canceled/Interrupt`；C `Failed`；A 仍 `Holding`；queue empty。 |
| holding queue invalid consumes only invalid target | holding A + queue invalid B + queued valid C；B `Failed` and consumed；C 保留；A 在 C transition 成功前保持 `Holding`。 |
| holding interrupt benign no-transition uses deterministic fallback | direct + controllable-source 都失败但非 safety；A 离开 foreground 时 `Stopped/Interrupt`；C post-stop 启动；不 failed；不每 tick 重试。 |
| holding queue benign no-transition releases source | direct + controllable-source 都失败但非 safety；A `Done/None`；B 通过 no-active/standby->user 启动；不每 tick 重试。 |
| holding interrupt safety failure | replacement target `Failed` with readiness/fatal error；active/source 进入 safety/passive/fault；queue empty。 |
| holding queue safety failure | target `Failed` with readiness/fatal error；source 按 safety/fault path；failed target 不留 queue。 |
| running interrupt regressions | smooth success、benign reject fallback、safety terminal、no duplicate waiting、source `Stopped/Interrupt` 全部保持。 |
| running interrupt invalid target keeps old failure path | shared helper 不直接 publish target failure；invalid target 仍走 running 既有 failure/fallback 行为。 |
| standby/passive/fixstand regressions | holding->standby 仍走 standby source；passive/fixstand/urgent_stop 不走 user helper、不 smoothing user target。 |
| transition/background transition regressions | `transition.target=user/idle/standby`、standby playback 下 queue/interrupt 既有抢占或等待行为不变。 |

必要时增加 test-only fixtures 或 knobs 来分别制造：

- direct synthetic reject。
- controllable-source fallback success。
- target load/validation failure。
- transition start fatal。

### 8.2 Skill tests

推荐位置：`packaging/skills/et1-action/tests/test_et1_action.py`。

| Test | 关键断言 |
| --- | --- |
| fake tracker returned queued, CLI not hardcode running | fake `/execute` 返回 `state:"queued"`；`run-trk` / `run-text` 输出 `state:"queued"`、`accepted:true`、`confirmed:false`。 |
| direct output accepted only | 文档和 JSON 字段说明 `ok:true` 是 accepted/submitted，不代表 complete。 |
| run-trk --wait failed run returns failure | fake run status 后续 `failed`；CLI 返回 `ok:false`、`TRACKER_RUN_FAILED`。 |
| run-text --wait failed run returns failure | 同上。 |
| --wait done success | fake run status 到 `done`；CLI 返回 `ok:true`、`accepted:true`、`confirmed:true`、`state:"done"`。 |
| --hold --wait holding success | fake run status 到 `holding`；CLI 返回 `ok:true`、`accepted:true`、`confirmed:true`、`state:"holding"`，不 timeout。 |
| no standby/retry workaround | holding 状态下 direct run 仍只提交 interrupt；不自动调用 `/standby` 再 retry。 |

### 8.3 Manual / sim e2e gate

推荐在 `tools/manual_gate.py` 增加 `held_interrupt_handoff`，并在
`tools/test_manual_gate.py` 补假 HTTP 流测试。

manual gate 步骤：

1. recover to standby。
2. 执行 A：`execute(..., hold=True)`。
3. 显式等待 A `state=="holding"`，不要只等 running。
4. 提交 C：`mode="interrupt"`。
5. 轮询 C：
   - C 不得 `failed`。
   - status 出现 `transition.target=="user"` 且 `target_id==C`，或 C 已成为 active
     user。
   - `queue.ids == []`。
6. 轮询 A terminal：
   - `state=="stopped"`。
   - `stop_reason=="interrupt"`。
7. 调 `/standby` 收尾。
8. 断言最终：
   - ctrl 是 standby / standby_velocity。
   - active none。
   - queue empty。
   - 没有进入 passive。

可选增加 skill-driven smoke：

1. 用 `et1-action run-trk --hold --wait` 把 A 推到 `holding`，确认 skill 输出
   `confirmed:true`、`state:"holding"`。
2. 再用 `et1-action run-trk` 提交 C，确认非 wait 输出
   `accepted:true`、`confirmed:false`，并用 tracker `/status?id=<C>` 验证 C
   没有 failed。
3. skill smoke 只验证 CLI 合同和真实 tracker 组合，不替代上面的 runtime gate。

该 gate 是修复验收项，不需要进入默认发布 gate；但在合并前应由开发者或 operator
在 sim/manual 环境跑过。

## 9. 验收 Gates

最小合并条件：

- C++ unit tests 覆盖 holding interrupt、holding queue、invalid target、
  running regression、control regression。
- skill tests 覆盖 output contract 和 `--wait --hold`。
- `git diff --check` 通过。
- manual/sim `held_interrupt_handoff` gate 通过，或在 PR 中明确记录未跑原因。

行为验收：

- holding interrupt 后，新 run id 不再因为 benign transition build reject 变成
  `Failed/InternalError`。
- 旧 held run 不再继续无限占 foreground；不允许每 tick 重试卡住。
- queue interrupt 仍取消旧 waiting ids。
- holding queue 与 holding interrupt 使用同一 robust handoff 实现。
- legal target 无法 direct/controllable-source handoff 时，interrupt 和 queue 都有
  确定 fallback，并最终启动 target 或进入 safety terminal。
- standby、passive、fixstand、urgent_stop 语义不漂移。

## 10. 风险与回滚

| 风险 | 影响 | 缓解 |
| --- | --- | --- |
| 抽 helper 时改变 running interrupt 行为 | 破坏已正常路径 | 先写 running regression tests；running wrapper 保留 fallback。 |
| source terminal 语义混淆 | holding interrupt 被标 done 或 queue 被标 stopped | interrupt 直接传 `Stopped/Interrupt`；queue 传 `Done/None`；不要依赖 `MotionRequest` 推断 mode。 |
| target 在 benign reject 时丢失 | 新 run accepted 后 queue empty 且 failed | helper 必须 no-commit build；成功后才 consume/terminal。 |
| invalid target 保留旧 source 但 queue 处理不清 | 可能重复失败或卡住 | invalid target 明确 publish failed 并从 pending 中移除；其它合法 queue 不丢。 |
| skill 输出兼容性 | 旧调用方以为 `state:"running"` 才成功 | 用 `accepted:true`/`confirmed:false` 明确合同；`--wait` 提供 confirmed 路径。 |
| safety/fault 被平滑逻辑绕过 | 机器人安全风险 | readiness/fault/model/write failure 仍按现有 terminal path；测试覆盖。 |

回滚策略：

- runtime helper 与 skill 输出修正应分 commit 或至少可独立 revert。
- 如果 runtime 修复出现异常，回滚 holding helper 接入点，保留 tests 作为 failing
  regression 证据。
- 不需要 API migration，因为本计划不新增公开 API。

## 11. 开发顺序

建议按以下顺序交付：

1. 写 runtime failing tests：先复现 held interrupt C failed、A 仍 holding。
2. 抽 user handoff helper，不改变行为。
3. 接入 running wrapper，确认 running regressions 仍过。
4. 接入 holding interrupt，确认 old source `Stopped/Interrupt`。
5. 接入 holding queue，确认 old source `Done/None`。
6. 修 skill output contract 和 tests。
7. 增加 manual gate。
8. 跑 unit tests、skill tests、manual/sim gate，并更新相关行为矩阵文档中已过时的
   holding 条目。

完成后，`MOTION_TRANSITION_BEHAVIOR_MATRIX.md` 中关于 active user holding
interrupt/queue 的描述应同步为新合同：`Holding -> user` 与 running interrupt
共用 robust handoff，benign direct synthetic reject 不再把合法 target 标
`INTERNAL_ERROR`。
