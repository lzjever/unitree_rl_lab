# User Interrupt Smooth Handoff

本文档记录 `agentic-et1-tracker` 已实现的用户动作 interrupt 平滑交接行为，以及该实现的产品/工程边界。该行为用于解决连续用户动作/插话场景下的产品语义和执行稳定性缺口。

范围只限：

- `deploy/robots/agentic-et1-tracker` runtime 对 `mode:"interrupt"` 的内部执行语义。
- `packaging/skills/et1-action` 对长序列、普通停止、急停、passive/recovery 的 agent 使用指引和最小测试。

不修改 ET1 app，不新增 `.trk` 格式，不新增 tracker playlist，不新增 HTTP sequence API。

## 1. Background

最近一次 botified 测试中，机器人不是“用户动作正常完成后自动进入 passive”，而是在连续用户动作被多次 interrupt 的过程中触发了 `ROBOT_BAD_ORIENTATION -> passive`。

已确认的事实：

- `ROBOT_BAD_ORIENTATION` 来自当前机器人 low state 姿态安全检查，不是 `.trk` 文件首帧校验。
- `et1-action run-text` 默认提交 tracker `mode:"interrupt"`，命令返回成功只表示“动作已生成并提交”，不表示机器人已经执行完成。
- 当前 runtime 对 active running GeneralTracker user 的 interrupt 会先尝试 current-frame synthetic handoff；成功时进入 `transition.target:"user"`，失败的 benign reject 才 fallback 到 controlled stop/restart。
- safety/readiness/fault 仍是最高优先级。bad orientation、fault、model/write error 不为了 interrupt 平滑而 fallback 或绕过安全路径。

产品层面的合同是：用户和 LLM agent 认为“现在改做新动作”就是前景意图接管。tracker 对这个合同只暴露 `mode:"interrupt"`，内部优先做平滑交接，不能安全交接时再回到 controlled stop/restart。

## 2. Product Contract

### 2.1 One Intent, One Way

| 用户意图 | Skill / API 行为 | 说明 |
| --- | --- | --- |
| 新动作、改做另一个动作 | `run-text` / `run-trk`，默认 tracker `mode:"interrupt"` | 替换当前 foreground intent。 |
| 明确“排在后面/做完再...” | `--mode queue` 或 sequence append | 只有明确追加时才排队。 |
| 多段长动作 | `sequence-start` | skill 本地 sequence 是唯一编排层；tracker 只执行普通 user run/queue/transition。 |
| 普通“停下/暂停/放松/回到待命” | `standby` / `POST /standby` | 取消 foreground user work，回普通 standby，保留 idle 配置。 |
| “不要动/站着别动/no idle” | `idle-clear` then `standby` | 清 idle 后进入纯 standby；不使用 urgent stop。 |
| “紧急停止/abort/kill/赶快停止” | `urgent-stop --urgent` / `POST /urgent_stop` | 立即急停，清 idle，不承诺 smoothing。 |
| 显式 passive | `passive --password ...` / `POST /passive` | passworded safety sink；不自动恢复。 |
| 进入站立构型 | `fixstand` / `POST /fixstand` | recovery/preparation pose，不是普通“站着别动”。 |

### 2.2 Updated Runtime Contract

`mode:"interrupt"` 仍然只有一个公开语义：新用户动作接管 foreground user intent。

内部执行规则升级为：

1. 如果当前 active 是 running GeneralTracker user，且目标也是 GeneralTracker user，runtime 先尝试从当前 active reference frame 构造 synthetic transition 到新 user 首帧。
2. 如果 transition 成功：
   - 旧 user run 终结为 `stopped`，`stop_reason:"interrupt"`。
   - 当前状态进入 internal `transition`，`transition.target:"user"`，`transition.target_id` 为新 run id。
   - 新 user run 在 transition 完成后进入 running。
   - 不进入 `Stopping`。
3. 如果 transition 不能安全构造：
   - 保持当前 controlled stop/restart fallback。
   - 旧 user 仍按 interrupt stopped。
   - 新 user 保持可执行请求，等 stop 后按现有启动路径运行。
   - benign transition reject 不得把新 user 提前标记为 `failed`。
4. safety/readiness/fault 优先级不变。bad orientation、fault、model/write error 仍按现有 safety path 处理，不能为了平滑而绕过。

这不是新增 `smooth_interrupt` 模式，也不新增参数。对外仍然只有 `mode:"interrupt"`。

### 2.3 Sequence Contract

`sequence-start` 属于 `et1-action` skill 本地编排，不下沉到 tracker。

当前 skill 已实现 bounded queue-ahead：

- 默认 `ET1_ACTION_QUEUE_AHEAD=3`。
- 未来段按 serial 顺序生成，并最多向 tracker 提前提交 3 个 unfinished tracker runs。
- queue-ahead 降低段间空隙，但意味着 `sequence-replace-tail` 只能替换尚未 submitted 的本地尾部。

本计划不把 queue-ahead 改成 tracker playlist，也不新增 run 删除/重排接口。需要做的是把文档、测试、agent 指引完全统一到这一条规则，避免旧文档残留“只提交当前段”的矛盾描述。

queue-ahead 的用户失败语义必须明确：

- `sequence-replace-tail` 只替换未 submitted 的本地段。
- 如果用户要求替换的尾部已经 submitted 到 tracker，skill 不能沉默成功。
- 默认返回 compact JSON，包含 `ok:false`、明确错误码，例如 `TAIL_ALREADY_SUBMITTED`、`replaceable_count`、`submitted_count`、单一 `next:"sequence-interrupt"`。
- 如果用户真正想改变已经 submitted 或正在执行的动作，应使用 `sequence-interrupt` 或新的 `run-text/run-trk` foreground intent。
- 不为了支持 replace submitted tail 而新增 tracker 删除、重排或 playlist API。

## 3. KISS / DRY / YAGNI Guardrails

| 原则 | 本计划约束 |
| --- | --- |
| KISS | 只改变 active running GeneralTracker user interrupt 的内部优先路径；公开 API 不变。 |
| DRY | 复用现有 `RuntimeControlLoop` synthetic transition helper、alignment、readiness 和 run status 机制。 |
| YAGNI | 不做 tracker playlist、priority queue、per-request transition profile、复杂调度 daemon、dashboard、数据库或新 status schema。 |
| 低心智负担 | 用户只需要理解普通动作、追加、standby、urgent_stop、passive/fixstand；不用选择“平滑模式”。 |
| 一个功能一种做法 | 普通停止只用 `standby`；急停只用 `urgent_stop`；新动作接管只用 `interrupt`。 |

## 4. Implementation Plan

### 4.1 Runtime: Smooth Active User Interrupt

主改动点：

- `src/runtime_control_loop.cpp`
- `include/agentic_et1_tracker/runtime/runtime_control_loop.hpp`
- `tests/runtime_control_loop_tests.cpp`

推荐实现：

1. 在 `RuntimeControlLoop::handleInterrupt(MotionRequest request)` 中，在 `waiting_.push_back(std::move(request))` 和 active user running 走 `markActiveStopping()` 之前，增加窄分支：
   - `active_kind_ == ActiveKind::User`
   - `active_ != nullptr`
   - `active_->state == MotionState::Running`
   - `active_->executor == MotionExecutor::GeneralTracker`
   - `isGeneralTrackerRequest(request)`
   - `active_track_` 存在并能取当前 `active_->frame`
2. 新增私有 helper，例如：
   - `RunningInterruptHandoffResult tryStartRunningUserInterruptHandoff(const MotionRequest& request);`
   - 返回语义至少区分 `Started`、`Fallback`、`SafetyTerminal`，不能只用裸 `bool` 混淆 benign reject 和 safety terminal。
3. helper 只负责：
   - 加载/对齐 target track。
   - 复用现有 current-reference / controllable-source synthetic transition path。
   - 成功后把 source user 终结为 `Stopped + Interrupt + Ok`。
   - 设置 `PendingTransition.target_kind = User`、`target_request = request`、`target_track = aligned target`。
   - 设置 `source_completion_state = Stopped`、`source_completion_reason = Interrupt`、`source_completion_error = Ok`。
   - 设置 `target_id = request.id`、`target_state = Queued`，并通过 `transition.target_id` 暴露目标 user run。
4. helper 不应该：
   - 调用 `markActiveStopping()` 后再尝试 transition，因为那会清理 active track/reference。
   - 在 transition 成功前把 request push 到 `waiting_`；否则 target 会同时存在于 transition 和 waiting queue。
   - 在 benign transition reject 时 publish target failed；benign reject 不得提前失败新 run。
   - 新建第二套 transition builder。
   - 修改 `RuntimeBridge::submitInterrupt()` 或 stop watermark。
   - 处理 LocoUpper 或 Preparing；这些保持现有 fallback。

fallback 行为：

- transition build/yaw/contact/raw-start 等 benign gate reject 时，回到当前 stop/restart 路径；旧 user `stopped + interrupt`，新 user 进入 waiting 并后续启动。
- target `.trk` 本身 invalid、target validation failure、model/runtime fatal error、readiness/safety failure 不是 benign reject；按现有 failed/safety/fault 语义处理。
- helper 不能直接调用会在 align/yaw/build 失败时 publish target failed 的现有 `startSyntheticTransitionFromActiveFrame()` 作为“探测函数”。应使用 `makeUserTransitionTracks*()` 或等价 no-commit path，再 commit 到 internal transition。
- helper 失败必须无副作用，或显式返回原始 request 给 caller；caller 必须还能按现有 stop/restart fallback 把 request 放入 waiting。
- safety terminal 仍立即走 safety path，不做 fallback start。
- internal transition 不作为独立 user run 暴露；target user 在过渡期间通过 `transition.target_id` 表示，并从 `queue.ids` 中排除。

### 4.2 Skill: Sequence And Intent Documentation Sync

主改动点：

- `packaging/skills/et1-action/SKILL.md`
- `packaging/skills/et1-action/references/intent-mapping.md`
- `packaging/skills/et1-action/references/sequence-workflow.md`
- `packaging/skills/et1-action/tests/test_et1_action.py`

需要同步：

1. 明确 `sequence-start` 默认 queue-ahead 是性能优化，不是 tracker playlist。
2. 明确 `sequence-replace-tail` 只能替换未 submitted local tail。
3. 明确直接 `run-text` / `run-trk` 会取消本地 active sequence，防止旧 worker 继续生成或提交 stale motions。
4. 明确新动作插话：
   - ready `.trk`：直接 `mode=interrupt`。
   - 文本新动作：使用 `sequence-interrupt --text` 或新的 `run-text`；旧本地 sequence 必须取消。
5. 保持普通 stop 三分法：
   - `standby` 保留 idle。
   - `idle-clear` 清 idle 后 standby。
   - `urgent-stop --urgent` 只用于明确紧急指令。
6. passive/fixstand 仍为显式 operator/recovery，不自动 follow `next`。
7. 明确 `run-text` / `run-trk` accepted 只代表生成并提交成功，不代表机器人动作执行完成；agent 需要用 `sequence-status`、`/status?id=<run_id>` 或 full `/status` 查询真实执行进度。

### 4.3 Documentation Reconciliation

需要更新或标注的文档：

- `MOTION_TRANSITION_BEHAVIOR_MATRIX.md`
  - 当前文档把 active user running interrupt 的 controlled stop 作为 P0 回归锁定。实现完成后必须改为新的合同：优先 smooth handoff，失败 fallback controlled stop。
- `SKILL_SEQUENCE_WORKFLOW_PLAN.md`
  - 清理“只提交当前段”与当前 queue-ahead 实现冲突的旧表述，或明确为历史设计。
- `README.md` / `CONTROL_STATE_MACHINE_REDESIGN_PLAN.md`
  - 只在必要处补一句：`mode:"interrupt"` 内部 may smooth handoff；agent 不需要也不能选择 stop profile。

文档目标是消除冲突，不新增治理文档层。

## 5. TDD Plan

### 5.1 C++ Unit Tests

先写失败测试，再实现。

必测：

| Test | 断言 |
| --- | --- |
| running GeneralTracker user interrupt starts transition | 不进入 `Stopping`；`active.kind=="transition"`；`transition.target=="user"`；`transition.target_id` 是新 run；旧 run `stopped + interrupt`。 |
| transition completes into new user | transition 完成后新 run running/done；新 run progress 从 user track 开始；transition 不进 queue/history。 |
| fallback when transition cannot build | benign reject 回到现有 controlled stop；新 run 等 stop 后启动或保持 queued；不误标 done/failed；无 duplicate waiting copy。 |
| preparing user interrupt unchanged | preparing 继续 controlled stop，不尝试 current-frame transition。 |
| LocoUpper interrupt unchanged | LocoUpper 仍走 loco stopping，不走 GeneralTracker transition。 |
| transition target user interrupted unchanged | 已在 transition 中时仍从 current transition frame 重建到 urgent user。 |
| safety/readiness failure wins | bad orientation/fault/model/write error 仍进入 passive/fault，不为了 interrupt 平滑绕过。 |

现有“active user running interrupt controlled stop”测试不能继续作为主合同原样保留。它应改成 fallback/preparing/LocoUpper 场景，或者拆成 smooth success 与 fallback 两类断言。

推荐测试位置：

- `tests/runtime_control_loop_tests.cpp`

### 5.2 Skill Tests

必测：

| Test | 断言 |
| --- | --- |
| sequence queue-ahead default documented behavior | 默认 queue-ahead 为 3，submitted segment 不可被 `sequence-replace-tail` 改写。 |
| replace submitted tail returns actionable error | 已 submitted tail 返回 `TAIL_ALREADY_SUBMITTED`、`replaceable_count`、`submitted_count`、单一 `next:"sequence-interrupt"`。 |
| direct run-text cancels local sequence | 旧 worker 不再继续提交 stale motions。 |
| ordinary stop maps standby | 不调用 urgent_stop，保留 idle。 |
| no-idle intent maps idle-clear + standby | 清 idle 后 standby。 |
| urgent stop requires guard | 缺 `--urgent` 不发 HTTP urgent_stop。 |
| passive requires password | 缺 password 不发 HTTP passive。 |

推荐测试位置：

- `packaging/skills/et1-action/tests/test_et1_action.py`

### 5.3 Manual E2E / Visual Gate

不进入默认发布 gate，手动需要时运行。

最小仿真场景：

1. 启动 MuJoCo 和 tracker。
2. `fixstand -> standby`。
3. 执行一个 4-6 秒 walking/turning `.trk`。
4. 在 active running 中途投递另一个 ready `.trk`，`mode:"interrupt"`。
5. 观察：
   - `/status` 进入 `transition.target:"user"`，不是 `stopping`。
   - 旧 run 查询为 `stopped + interrupt`。
   - 新 run transition 后进入 running/done。
   - 在选定安全 fixture 下不应新增 passive/fault/fall。
6. 对比 fallback 场景：用可控 yaw/root/contact gate reject 的 target，确认系统 controlled stop/restart，不崩溃、不绕过 safety。
7. 如果真实触发 safety，验收标准是正确进入 passive/fault/manual，并保留可诊断状态；不能为了满足视觉平滑而绕过 safety。

视觉 gate 只做定性验证：动作切换是否显著少跳变，机器人是否无明显跌倒/姿态越界。

## 6. Acceptance Criteria

### Runtime

- active running GeneralTracker user 收到 GeneralTracker interrupt 时，优先尝试 current-frame synthetic transition。
- transition 成功时不进入 `Stopping`，status 暴露为 internal transition，target id 指向新 user run。
- source user 终结状态清楚：`stopped` + `stop_reason:"interrupt"`。
- transition 失败时保持现有 controlled stop fallback。
- Preparing、LocoUpper、urgent_stop、passive、fixstand、standby、安全/fault 路径不被改变。

### Product / Skill

- LLM agent 不需要新 API 或新参数。
- direct `run-text` / `run-trk` 默认仍是用户新意图接管。
- accepted/submitted 响应不等于动作执行完成；agent 不应口头宣称完成，除非查询到 run done/holding 或用户只要求提交。
- 长序列仍使用 `sequence-start`，queue-ahead 语义明确且测试锁定。
- submitted sequence tail 不可替换时返回明确 compact error 和单一 next action，不沉默成功。
- 普通 stop、no-idle standby、urgent_stop、passive/fixstand 的使用边界在 skill 中保持一致。

### Documentation

- 没有文档继续把 active running user interrupt 的 controlled stop 写成唯一/必须的 P0 行为。
- 没有文档要求 tracker 实现 sequence playlist、run 删除、queue 重排。
- 没有文档要求 agent 根据 `next` 自动执行 passive recovery chain。

## 7. Risks And Mitigations

| Risk | Mitigation |
| --- | --- |
| Reference frame 平滑不等于真实机器人状态平滑 | 复用现有 controllable-source / readiness / raw-start / contact/root-yaw gate；失败 fallback controlled stop。smooth handoff 只降低跳变风险，不承诺避免所有 passive/fault。 |
| helper 构建失败却提前把 target run 标 failed | 使用 no-commit probe 或窄 helper，保证 benign reject 可 fallback；只有 invalid target/safety/fatal error 才 failed/safety。 |
| accidentally clears active reference before transition | 新分支必须在 `markActiveStopping()` 前执行；测试锁定 status 不进入 stopping。 |
| target duplicated in transition and waiting | 新分支必须放在 `waiting_.push_back()` 前；成功后 target 只能通过 `transition.target_id` 暴露，不留 waiting 副本。 |
| scope creep into sequence/server playlist | 明确 sequence stays in skill；tracker 只管 run/queue/transition。 |
| agent 继续用多个 direct run-text 组织长动作 | skill docs 强化：多段 prompt 用 `sequence-start`；直接 run-text 是替换当前 foreground intent。 |
| safety 被平滑目标稀释 | bad orientation/fault/readiness/model/write error 仍最高优先；不要降低阈值或自动恢复 passive。 |

## 8. Non-goals

- 不做新的 HTTP route、request 参数或 stop profile。
- 不做 tracker-side sequence/playlist。
- 不做 run id 删除、重排、优先级队列。
- 不做 dashboard、database、telemetry governance。
- 不改变 ET1 app。
- 不改变 `.trk` 格式。
- 不把 LocoUpper 纳入本次 smooth interrupt。
- 不降低 orientation safety threshold。
- 不自动从 passive 恢复。

## 9. Implementation Checklist

- [x] 写 C++ tests，覆盖 running user interrupt smooth handoff、benign fallback、invalid target、safety terminal 和 transition-start fatal。
- [x] 实现窄路径，在 `waiting_.push_back()` 前尝试 current-frame transition，成功后只通过 `transition.target_id` 暴露目标 run。
- [x] 跑 `runtime_control_loop_tests` 相关测试。
- [x] 更新 skill docs/tests，锁定 sequence queue-ahead 和 intent mapping。
- [x] 更新冲突文档，确保 active running user interrupt 不再被描述成必须 controlled stop/restart。
- [x] 运行 tracker 单测最小集。
- [x] 手动仿真 e2e/visual gate，确认 interrupt 切换状态和肉眼效果。
- [x] review 确认未新增 API、未影响 urgent_stop/passive/fixstand/standby。

## 10. Team Review Summary

产品 review 结论：

- 用户心智需要保持三件事简单稳定：新动作接管、普通 standby、明确 urgent/passive。
- sequence 是 skill 编排，不应变成 tracker playlist。
- queue-ahead 可以保留为性能优化，但必须清楚说明 submitted segment 不可替换。
- 不要自动 follow `next` 做 passive recovery。

研发 review 结论：

- 主缺口在 `RuntimeControlLoop::handleInterrupt()` 的 running user 分支。
- 正确改法是新增窄 helper，在 `markActiveStopping()` 前尝试 current-frame transition。
- 失败 fallback 保持现有 stop/restart，Preparing/LocoUpper/transition-in-progress 行为不变。
- 不建议扩展 HTTP API 或 status schema；现有 `active/transition/exec/queue` 字段足够验收。
