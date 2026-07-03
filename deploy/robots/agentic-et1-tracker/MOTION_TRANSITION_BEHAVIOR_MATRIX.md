# Motion Transition Behavior Matrix

本文档是 `agentic-et1-tracker` 动作/控制转换行为的 handoff 基线。它描述：

- 当前已实现合同。
- 已收敛的 P1 行为合同。
- 已实现回归锁定项。
- P2 backlog，明确不进入当前已收敛主路径。

范围只限 `deploy/robots/agentic-et1-tracker`。不扩大 HTTP API，不修改 ET1 app，不修改 `.trk` 格式。

主要代码参考：

- Runtime 状态机：[include/agentic_et1_tracker/runtime/runtime_control_loop.hpp](include/agentic_et1_tracker/runtime/runtime_control_loop.hpp)、[src/runtime_control_loop.cpp](src/runtime_control_loop.cpp)
- Command priority / stop watermark：[src/runtime_bridge.cpp](src/runtime_bridge.cpp)
- HTTP API gate：[src/api_service.cpp](src/api_service.cpp)
- 状态 JSON：[src/json_codec.cpp](src/json_codec.cpp)
- 相关测试：[tests/runtime_control_loop_tests.cpp](tests/runtime_control_loop_tests.cpp)、[tests/api_tests.cpp](tests/api_tests.cpp)

## Reading contract / Delivery boundary

本文档交付定位是当前行为基线 + 已收敛 P1 合同，不是全量长期 roadmap。

| 标记 | 含义 | Handoff 用法 |
| --- | --- | --- |
| Current | 当前代码已经实现或 HTTP/API 已经暴露的合同；文档必须与代码一致。 | 必须保持，作为回归锁定。 |
| Current/P1 | 已实现的 P1 行为；当前行为和推荐行为已经收敛。 | 必须保持，作为 P1 回归锁定；不新增 API、不改 ET1、不扩大 `.trk` 格式。 |
| Backlog/P2 | 有价值但不进入当前已收敛主路径的改进或补测。 | 不阻塞 Current/P1，不应驱动范围蔓延。 |

优先级含义：

| Priority | 含义 |
| --- | --- |
| P0 | 当前安全/API 合同或已实现行为回归锁定；不能被 P1 或 P2 改动破坏。 |
| P1 | 已完成并需要回归锁定的主路径：background transition/playback 下 user `.trk` queue 抢占背景，target-specific interrupt/status/API 行为保持一致。 |
| P2 | Backlog；可测试性、边界一致性或 operator 体验改进，不进入当前主路径。 |

## 1. Product Invariants / Non-goals

### Current: 已实现必须保持

| 不变量 | 结论 |
| --- | --- |
| HTTP `/execute` gate 顺序 | 先 request shape，再 readiness/manual gate，再 controller gate；blocked controller 在 readiness OK 时才返回 conflict；拒绝发生在 validator、command sink、run id generator 之前。 |
| active idle 是背景 | user queue / interrupt 已可抢占 active idle，并从 idle current reference 平滑到 user 首帧。 |
| idle 是 background config | 用户动作 queue/interrupt 只打断当前 idle 播放，不清 idle config；失败也保留配置并可在回到 standby 链路后恢复。只有 `/urgent_stop`、passworded `/passive`、`/idle {"paths":[]}` 清 idle config。 |
| transition 过程中收到新的 user interrupt | 从当前 transition reference frame 重新构造到 urgent user 首帧的 synthetic transition。 |
| transition 目标是 user 时收到新的 user queue | 保留旧 target user，新 user 排队；不抢占已有前景 user target。 |
| active user running 是前景动作 | GeneralTracker user -> GeneralTracker user interrupt 优先尝试 current-frame synthetic handoff；成功时进入 internal `transition.target:"user"`，失败时 fallback controlled stop/settle 后再启动 urgent。Preparing、LocoUpper 和 safety/fault 路径不改变。 |
| active user holding 是前景动作 | `hold:true` 到末帧后的 `holding` 仍是 foreground user intent。holding interrupt 取消 waiting，并直接尝试 held-frame -> replacement user handoff；成功时旧 source `Stopped/Interrupt` 且 queue empty。holding queue 复用同一 handoff；成功时旧 source `Done/None`。 |
| passive/fixstand/fault/urgent_stopping 是安全、恢复或姿态控制边界 | HTTP `/execute` 在 readiness OK 时拒绝 user `.trk`；readiness/manual error 优先于 controller conflict。runtime 低层也不得在这些状态启动 waiting user。 |
| urgent_stop/passive/fixstand/standby 是 control/safety 命令 | 不生成 user run、不进入 queue。urgent_stop/passive/fixstand 不走 smoothing；`/standby` 一般不走 smoothing，唯一例外是静态 user holding 可从 held reference 平滑到 standby reference。 |
| synthetic transition 是内部对象 | 无 run id，不占 `queue.limit`，不进入 `queue.ids`，不进 user run history，不可通过 `GET /status?id=...` 查询。 |

### Current/P1: 已实现并需要锁定

| 目标 | 结论 |
| --- | --- |
| background transition/playback 下 user queue 抢占背景 | `transition.target=idle`、`transition.target=standby`、standby playback 收到 queue user `.trk` 时，从 current reference 平滑切到 user，不等背景完成。 |
| `transition.target=user` 下 queue 继续等待 | 当前 transition 已归属前景 user target；新 user queue 不抢占。 |
| active user running interrupt 优先 smooth handoff | active running GeneralTracker user 收到 GeneralTracker interrupt 时，从当前 reference frame 构造到新 user 首帧的 synthetic transition；benign reject fallback controlled stop，不提前失败新 user。 |
| active user preparing interrupt 继续 controlled stop | Preparing 还没有稳定 current-frame source，继续 controlled stop/restart。 |
| interrupt 对 background transition/playback 直接抢占 | `transition.target=idle`、`transition.target=standby`、standby playback 收到 interrupt user `.trk` 时，从 current reference 平滑切到 urgent user，并取消本地 waiting。 |
| holding user queue/interrupt 复用 robust handoff | holding + interrupt 从 held reference 到 replacement user，旧 source `Stopped/Interrupt`，成功后 queue empty；holding + queue 从 held reference 到 next user，旧 source `Done/None`。benign no-transition reject 走确定 fallback，不把合法 target 标成 `INTERNAL_ERROR`。 |
| background queue 抢占失败语义最小化 | target user load/align/transition build 失败时 publish target `Failed`、`stop_reason=None`；old idle/standby target 不写 user history，不恢复旧 background transition/playback；停止当前 background/idle active 播放但保留 idle config，以便回到可播放 idle 的 standby 链路后恢复。 |

### KISS / DRY / YAGNI

| 原则 | 本文推荐 |
| --- | --- |
| KISS | 继续使用现有 `/execute {path, mode, hold}`、`/idle`、`/urgent_stop`、`/passive`、`/fixstand`、`/standby`。 |
| DRY | 复用现有 current reference / synthetic transition helper，不复制 idle/user/standby 三套插值逻辑。 |
| YAGNI | 不新增 queue priority、per-request transition profile、外部 transition asset、复杂 blending、contact-aware 策略、远程 motion payload。 |

## 2. 状态和事件词汇表

### Runtime / Active / Transition Target

| 名称 | 当前集合 | 说明 |
| --- | --- | --- |
| `RuntimeInternalState` | `Passive`, `FixStand`, `Velocity`, `GeneralTrackerIdle`, `GeneralTrackerActive`, `GeneralTrackerTransition`, `Stopping`, `Fault` | runtime 内部状态集合。 |
| `ActiveKind` | `None`, `User`, `Idle`, `Transition` | 对外 `active.kind` 的权威来源。只有 `User` 有用户 run id。 |
| `TransitionTargetKind` | `User`, `Idle`, `Standby` | 内部 synthetic transition 的目标。 |
| Public controller mapping | `Velocity` 和 control runtime 下 `GeneralTrackerIdle` 对外是 `standby`；`GeneralTrackerActive` / `GeneralTrackerTransition` 对外是 `running`，preparing 阶段临时对外是 `preparing` | `ctrl` 只是 controller 大类，不足以判断是否在执行用户动作。 |
| Legacy/internal `ControllerState::Idle` | 不作为正常对外待命合同 | 当前 `/execute` 也会被 controller gate 拒绝；它不是 user `.trk` 的 no-active accepting state，no-active user path 应走 public `standby`。 |

### Agent status contract

LLM agent 判断用户动作时不能只看 `ctrl:"running"`。应优先读取：

- `active.kind`：区分 `user`、`idle`、`transition`、`none`。
- `transition.target` / `transition.target_id`：区分 transition 是去 user 还是去 background；`target_id` 才是目标 user run id。
- `exec.id`：只有 active user run 才代表正在执行的用户动作。
- `queue.ids`：表示等待中的 user work，不表示当前正在执行。

### 事件

| 事件 | 入口 | 说明 |
| --- | --- | --- |
| user queue | `POST /execute {"mode":"queue"}` 或省略 mode | 普通用户 `.trk`。 |
| user interrupt | `POST /execute {"mode":"interrupt"}` | 紧急用户 `.trk`：取消本地 waiting；对 background transition/playback 从 current reference 抢占到 urgent user；对 user-owned transition 取消旧 target 后从 current reference 到 urgent user；active running GeneralTracker user 优先 smooth handoff，benign reject fallback controlled stop；active holding GeneralTracker user 直接尝试 held-frame -> replacement user handoff，成功后 queue empty；active preparing 和 LocoUpper 保持 controlled stop。 |
| idle config | `POST /idle {"paths":[...]}` | 配置 background idle pool；不是 run 提交。 |
| idle clear | `POST /idle {"paths":[]}` | 清空 idle pool；当前 API 允许任意状态。 |
| urgent_stop | `POST /urgent_stop` 空 body | 最高优先级紧急控制命令；不 smoothing。 |
| passive safety command | passworded `POST /passive` | HTTP 入口必须有正确 password；lowcmd/manual gate 仍拒绝；readiness OK 时可进入 runtime sink；readiness 非 OK 时仅 `ROBOT_BAD_ORIENTATION` 且 `block=="bad_orientation"` 作为 safety exception 可进入 runtime sink。其他 readiness/fault error 拒绝，不进 sink。命令进入 runtime 后是 safety sink，不 smoothing。 |
| fixstand | `POST /fixstand` 空 body | HTTP 入口仅在 readiness OK 或 bad_orientation recovery 场景进入 sink；其他 Fault/readiness error 先返回错误。runtime 命令效果是姿态恢复/控制，不 smoothing。 |
| standby | `POST /standby` 空 body | 回 Velocity0/standby 控制；不生成 user run、不进 queue；一般不 smoothing，静态 user holding 例外；保留 idle config 时，回到可播放 idle 的 standby 链路后可由 background idle manager 按现有规则自动播放。 |
| completion | 当前 user/idle/transition 播放结束 | 触发 user->idle、user->standby、idle->idle、transition target start。 |
| readiness failure | low state / orientation / lowcmd occupancy / model failure | 安全优先，不能被 user `.trk` 绕过。 |

## 3. Current Behavior + Recommendation Matrix

本节的 Delivery class 以开头 Reading contract 为准。Current/P1 表示已经实现并需要保持的主路径；当前行为和推荐行为已经收敛。已实现的 `/standby`、`/fixstand`、idle config 行为作为 Current/P0 或 P2 regression lock 保留，不扩大为新的主实现范围。

### User `.trk` 进入路径

`POST /execute` 当前 gate 顺序：先完成 body/path/mode/hold request shape 解析，再读取 status 并执行 readiness gate；readiness 非 OK 时返回 readiness error，`lowcmd_occupied` 返回 manual next action。只有 readiness OK 且 public ctrl 属于 blocked set 时，才返回 `CONTROL_STATE_CONFLICT`。这些拒绝都发生在 validator、command sink、run id generator 之前。

| Source state | Event | Current behavior | Reasonable? | Recommended behavior | Delivery class | Test gap |
| --- | --- | --- | --- | --- | --- | --- |
| `Passive` | user queue / interrupt | `/execute` 先过 readiness gate；若 readiness OK，controller gate 返回 `CONTROL_STATE_CONFLICT`，建议 `/fixstand then /standby`；runtime 低层即使存在 waiting，也不会在 Passive 中启动 user motion。 | 是 | 保持条件式拒绝。user `.trk` 不能打断 passive；readiness/manual error 必须优先于 controller conflict。 | P0 | API 已覆盖；保留 gate ordering 和 no validator/sink/id 断言。 |
| `FixStand` | user queue / interrupt | `/execute` 先过 readiness gate；若 readiness OK，controller gate 返回 `CONTROL_STATE_CONFLICT`，建议 `/standby`；runtime 低层不在 FixStand 中启动 waiting user。 | 是 | 保持条件式拒绝。FixStand 是恢复/姿态控制，不接 user motion；readiness/manual error 必须优先。 | P0 | API 已覆盖；保留 gate ordering 和 no validator/sink/id 断言。 |
| `Fault` | user queue / interrupt | `/execute` 先过 readiness gate；若 readiness OK，controller gate 返回 `CONTROL_STATE_CONFLICT`，建议 `/fixstand`；runtime 低层不在 Fault 中启动 waiting user。 | 是 | 保持条件式拒绝。fault 只能走 recovery/safety path；readiness/manual error 必须优先。 | P0 | API 已覆盖；保留 gate ordering 和 no validator/sink/id 断言。 |
| `Stopping` | user queue / interrupt | `/execute` 先过 readiness gate；若 readiness OK，controller gate 返回 `CONTROL_STATE_CONFLICT`；runtime 低层若已有 stop watermark 后的 queue/interrupt，会保留为 post-stop work，不生成 transition；`stopHoldTicks()==0`，stop settle 当前为 0 tick。 | 部分合理 | 外部 API 继续条件式拒绝新的 user `.trk`。低层已经接受的 post-stop work 可保留，但只能在退出 `Stopping` 后按 standby/no-active 规则启动。 | P0 | 低层 stop watermark 已有；需保留 API gate ordering 回归。 |
| legacy/internal Idle | user queue / interrupt | `/execute` 先过 readiness gate；若 readiness OK，controller gate 返回 `CONTROL_STATE_CONFLICT` / `wrong ctrl; check /status`；不调用 validator/sink/id generator。 | 是 | 保持。Idle 不是 user motion accepting state；用户应等 status 回到可接受的 `standby`/running path。 | P0 | 补/保留 legacy/internal Idle execute gate 测试。 |
| no active + `Velocity` / public `standby` | user queue | command 入 waiting；tick 中 `startNext()`；control runtime 且有 `standby_track_` 时，从 `standby_track_` 最后一帧 synthetic transition 到 user 首帧。 | 是 | 保持。standby 是背景，user 可立即接管；source 优先使用当前/最近 standby reference，当前实现使用 standby asset last frame。 | Current | 已有 standby no-active user gate 测试；queue/interrupt 对称断言是 regression lock，不是 P1 主目标。 |
| no active + `Velocity` / public `standby` | user interrupt | 与 queue 基本等价：无前景可中断，进入 waiting 后按 standby->user gate 启动。 | 是 | 保持。无 active 时 interrupt 不需要额外语义。 | Current | 回归锁定。 |
| `GeneralTrackerIdle` no active | user queue / interrupt | 如果 idle pool 有候选，tick 可自动 start idle；若 user 已 waiting，则 user 优先 `startNext()`。 | 是 | 保持 user 优先。idle 只能在无 user active/queue/pending 时启动。 | Current | 保留 idle 不阻塞 user 测试。 |
| active idle | user queue | 当前直接尝试 `idle current frame -> user first frame` synthetic transition；成功后 `active.kind:"transition"`、`transition.target:"user"`、`target_id` 为 user run id；失败时发布 user failed，停止当前 idle 播放但保留 idle config。 | 是 | 保持。queue 对 active idle 已经是已实现抢占；用户动作打断 idle 不清配置，失败也保留并可在回到 standby 链路后恢复。 | Current | 已有 active idle preempt 和 failure 测试。 |
| active idle | user interrupt | 取消 waiting；同 queue，从 idle 当前帧 transition 到 urgent user。 | 是 | 保持。interrupt 不应比 queue 更慢。 | Current | 已有 active idle interrupt 测试。 |
| active user preparing | user queue | 新 user 进入 waiting；当前 preparing user 不被打断。 | 是 | 保持。preparing 属于前景 user ownership。 | Current | preparing 专门测试可进 P2。 |
| active user preparing | user interrupt | 取消 waiting；urgent 入 waiting；当前 active 被标记 `Stopping`，进入 `Stopping`，之后 urgent 从 standby/no-active 规则启动。不是 current-frame transition。 | 是，保守 | 保持 controlled stop。Preparing 不走 current-frame user handoff。 | P0 | 已覆盖 `[runtime-p1]` preparing interrupt controlled-stop 回归。 |
| active user running | user queue | 新 user 进入 waiting；等待当前 user 完成/holding/urgent_stop 后启动。 | 是 | 保持 FIFO queue。 | Current | 已有基础 queue 行为；保留。 |
| active user running | user interrupt | GeneralTracker user -> GeneralTracker user 时先尝试 current-frame synthetic handoff；成功后旧 user `stopped` + `stop_reason:"interrupt"`，active 进入 `transition.target:"user"` 且 `transition.target_id` 为新 run；benign reject fallback controlled stop/settle 后 urgent 启动。 | 是 | 新主合同。不得把 benign transition reject 误标新 user failed；不得绕过 readiness/safety/fault。 | P0 | 需要 smooth success、fallback、safety 优先测试。 |
| active user holding | user queue | waiting 不为空时，复用 robust handoff 从 held/current user frame 到下一 user；source user 在 transition 成功启动时转 `done`、`stop_reason=None`。benign no-transition reject 走确定 fallback，不把合法 target 标成 `INTERNAL_ERROR`。 | 是 | 保持。holding 是 foreground user intent，也是静态/可控 reference；queue 表示用户明确 append。 | Current/P1 | 已有 holding->user 测试；保留 holding queue handoff/fallback 回归。 |
| active user holding | user interrupt | 取消 waiting，并直接尝试从 held/current frame handoff 到 replacement user；成功后旧 source `stopped` + `stop_reason:"interrupt"`，queue empty，进入 `transition.target:"user"` 或新 user active。benign no-transition reject 走确定 fallback，不把合法 target 标成 `INTERNAL_ERROR`。 | 是 | 保持。interrupt 替换当前 foreground user intent，不把 holding 当 idle/background。 | Current/P1 | 已有 holding interrupt/transition 测试；保留 held_interrupt_handoff 回归。 |
| `transition -> user` | user queue | 新 user 只进入 waiting；不中断当前 transition，旧 target user 仍优先。 | 是 | 保持。当前 transition 已经归属某个前景 user target，新 queue 不应抢占它。 | Current/P1 | 已覆盖 `[runtime-p1]` user-owned transition waits。 |
| `transition -> user` | user interrupt | 取消 waiting；取消旧 target user；从当前 transition frame 重新构建到 urgent user 的 transition。 | 是 | 保持。interrupt 应覆盖旧 user target。 | Current/P1 | 作为 target-specific interrupt 回归保留。 |
| `transition -> idle` | user queue | 抢占背景 idle target：取消 idle target，从当前 transition frame transition 到 user；不等背景 transition 完成。 | 是 | 保持。idle target 是 background-owned，queue user `.trk` 应立即接管。 | Current/P1 | 已覆盖 `[runtime-p1]` `transition.target=idle` queue 抢占测试。 |
| `transition -> idle` | user interrupt | 抢占背景 idle target；取消 waiting；从当前 transition frame 重新构建到 urgent user。 | 是 | 保持。interrupt 与 queue 一样从 current reference 接管，但会取消本地 waiting。 | Current/P1 | 已覆盖 `[runtime-p1]` target=idle interrupt current-frame 测试。 |
| `transition -> standby` synthetic | user queue | 抢占背景 standby target：从当前 transition frame transition 到 user；旧 standby target 被丢弃，不继续 standby playback。 | 是 | 保持。standby target 是 background-owned，queue user `.trk` 应立即接管。 | Current/P1 | 已覆盖 `[runtime-p1]` `transition.target=standby` queue 抢占测试。 |
| `transition -> standby` synthetic | user interrupt | 抢占背景 standby target；取消 waiting；从当前 transition frame transition 到 urgent user；旧 standby target 被丢弃。 | 是 | 保持。 | Current/P1 | 已覆盖 `[runtime-p1]` target=standby interrupt current-frame 测试。 |
| standby playback as `ActiveKind::Transition` | user queue | 抢占背景 standby playback：从当前 playback frame transition 到 user；不等 playback 完成。 | 是 | 保持。standby playback 是 background-owned reference。 | Current/P1 | 已覆盖 `[runtime-p1]` standby playback queue 抢占测试。 |
| standby playback as `ActiveKind::Transition` | user interrupt | 抢占背景 standby playback；取消 waiting；从当前 playback frame transition 到 urgent user。 | 是 | 保持。 | Current/P1 | 已覆盖 `[runtime-p1]` standby playback interrupt current-frame 测试。 |

### Control / Safety 命令路径

| Source state | Event | Current behavior | Reasonable? | Recommended behavior | Delivery class | Test gap |
| --- | --- | --- | --- | --- | --- | --- |
| any active user | `/urgent_stop` | cancels waiting through urgent stop sequence；active user 标记 `Stopping`/`urgent_stopping`；清 reference；进入 urgent stop path；`stopHoldTicks()==0`。 | 是 | 保持立即 urgent_stop，不 smoothing。 | P0 | 已有 stop watermark/active urgent_stop 测试。 |
| active idle | `/urgent_stop` | 清 idle config；`stopIdleActive()`；不写 idle/user history；回 `GeneralTrackerIdle` / public standby。 | 是 | 保持。urgent_stop 是显式紧急控制，应清掉背景 idle。 | P0 | 已有 urgent_stop clears idle 测试。 |
| active transition | `/urgent_stop` | `abortTransition()`，target user 如有则 canceled；进入 `urgent_stopping`；不播放 standby reference。 | 是 | 保持。urgent_stop 不等待 transition 完成。 | P0 | 已有 urgent_stop aborts active transition 测试。 |
| `Passive` | `/urgent_stop` | 基本 no-op/safety sink；保持 Passive。 | 是 | 保持幂等。 | P0 | 保留。 |
| `Fault` | `/urgent_stop` | 不恢复 fault；清理按 bridge/status 语义执行。 | 是 | 保持。fault recovery 必须走 FixStand/operator。 | P0 | API fault stop 已有部分覆盖。 |
| HTTP admitted state / runtime any state | passworded `/passive` | HTTP 入口必须有正确 password；lowcmd/manual gate 仍拒绝；readiness OK 时可进入 runtime sink；readiness 非 OK 时仅 `ROBOT_BAD_ORIENTATION` 且 `block=="bad_orientation"` 可作为 safety exception 进入 runtime sink。其他 readiness/fault error 拒绝，不进 sink。进入 runtime 后 cancel waiting、清 idle config、active idle stop、transition abort、active user stopped、clear reference、进入 Passive。 | 是 | 区分 HTTP 可达性和 runtime 命令效果。保持 admitted command 的立即 safety sink；不 smoothing；不要把 `/passive` 写成无条件 any-state 后门，也不要写成所有 readiness 非 OK 都拒绝。 | P0 | 保留 password、manual gate、readiness OK admitted、bad_orientation exception admitted、其他 readiness/fault error rejected、runtime transition/passive 覆盖。 |
| `Passive` / `Fault` recovery | `/fixstand` | HTTP 入口仅在 readiness OK 或 bad_orientation recovery 场景进入 sink；其他 Fault/readiness error 先返回错误，不进入 runtime sink。进入 runtime 后 reset active/transition，进入 FixStand。 | 是 | 保持受 gate 保护的恢复入口。不要把 `/fixstand` 描述成所有 Fault/readiness error 都可直接进入 sink。 | P0 | 保留/补 readiness OK、bad_orientation recovery、其他 Fault/readiness error 不进 sink 的 API 回归。 |
| active user running/preparing | `/fixstand` | cancel waiting；active user `Stopping`；进入 `Stopping`，post-stop control 为 FixStand。 | 是 | 保持，不 smoothing。 | P0 | 建议补 active user -> fixstand post-stop test。 |
| active idle | `/fixstand` | stop idle；进入 FixStand；idle config 保留但不作为 FixStand 自动播放许可。 | 部分合理 | 停止背景 idle；保留 idle config；FixStand 期间绝不自动播放 idle，之后必须显式 `/standby` 才可回到可播放 idle 的 standby 链路。 | P0 | 补 active idle + fixstand 保留 config 但不自动 idle 测试。 |
| active transition | `/fixstand` | abort transition；后续进入 FixStand 或 Stopping path。 | 是 | 保持，不 smoothing。 | P0 | 补 target=user/idle/standby 三类 transition abort 测试。 |
| active user holding | `/standby` | 尝试 held frame -> standby first frame synthetic transition；再播放 standby asset；失败则 user done 后回 GeneralTrackerIdle。 | 是 | 保持。这是 `/standby` 不走 smoothing 的唯一例外：仅静态 user holding 可从 held reference 平滑到 standby reference；readiness 不允许时必须走 safety/passive/fault。 | Current | 已有 holding->standby/standby playback 测试；作为 regression lock。 |
| active user running/preparing | `/standby` | active user controlled stop；post-stop control standby。 | 是 | 保持，不 smoothing。 | P0 | 建议补 running/preparing -> standby 测试。 |
| active idle | `/standby` | `stopIdleActive()`；idle config 保留，回到可播放 idle 的 standby 链路后，若 idle config 非空且无 user work、ready/safe，idle 可按现有自动播放规则重新启动。 | 是 | 保持。`/standby` 不清 idle config、不生成 user run、不进 queue；后续 idle 启动是 background idle manager 的正常行为，不是 `/standby` 队列化。若 caller 需要纯 `standby` 且不播放 idle，应先 `/idle {"paths":[]}` 再 `/standby`；不要把 urgent `/urgent_stop` 当普通待命入口。 | P2 | 保留/补测试锁定现有行为：active idle 停止、idle config 保留、满足自动播放条件时可重新 idle。 |
| active transition | `/standby` | abort 当前 transition；进入 standby control path。 | 是 | 保持，不 smoothing；若 target 是 background standby，本命令与目标一致也不需要继续平滑。 | P0 | 补 target=user/idle/standby 分类测试。 |
| any non-fault/non-passive safe state | nonempty `/idle` | API 允许在 `standby`、`preparing`、`running` 等状态配置；runtime 替换 idle config；若 active idle 则 stop 当前 idle。 | 是 | 保持配置和播放解耦；idle config 不生成 run。 | P2 | 已有 API/runtime idle config 测试。 |
| passive/fixstand/urgent_stopping/fault | nonempty `/idle` | API gate 拒绝；避免未来自动播放跨过安全链路。 | 是 | 保持。 | P0 | 已有 API 测试。 |
| any state | `/idle {"paths":[]}` | API 允许清空；runtime 清 idle config，不校验 path。 | 是 | 保持安全清空能力。 | Current | 已有 API 测试。 |

### Completion / Readiness 路径

| Source state | Event | Current behavior | Reasonable? | Recommended behavior | Delivery class | Test gap |
| --- | --- | --- | --- | --- | --- | --- |
| active user, `hold:true` | user reaches last frame | user 进入 `MotionState::Holding`，持续发布 last frame。 | 是 | 保持。holding 不是 done 后的隐藏 active；`GET /status?id=...` 应显示 holding。 | Current | 已有 holding 测试。 |
| active user, `hold:false`, idle configured, no waiting | completion | 优先 user current/last frame -> idle first frame synthetic transition；source user 在 transition 开始时 done。 | 是 | 保持。idle 是自然背景回落。 | Current | 已有 user->idle 测试。 |
| active user, `hold:false`, no idle, standby track available, no waiting | completion | user current/last frame -> standby first frame synthetic transition；随后 standby playback；最后回 public standby。 | 是 | 保持。standby 是自然背景回落。 | Current | user completion->standby 端到端是 regression lock。 |
| active user, waiting not empty | completion | 不走 idle/standby；finish user done；回 accepting state 后 start next user。 | 是 | 保持 user work 优先于背景。 | Current | user completion + waiting skips idle/standby 是 regression lock。 |
| active idle, idle pool still configured, no waiting | completion | idle current/last frame -> next idle first frame synthetic transition；不生成 history。 | 是 | 保持。idle->idle 是内部背景平滑。 | Current regression / P2 coverage | 已有 idle->idle 测试；保留；后续只需按 P2 覆盖分类补缺口。 |
| active idle, waiting not empty | completion | 不进入 idle->idle；后续 user 优先。 | 是 | 保持。 | Current | idle completion + waiting user 是 regression lock。 |
| transition target user | transition completion | target user 进入 `Running`，`exec.frame` 从 0 开始；transition 不计入 user progress。 | 是 | 保持。 | Current | 已有多处 transition target user 测试。 |
| transition target idle | transition completion | 没有 user queue/interrupt 抢占时，target idle 进入 active idle；无 user id/history。若 user queue/interrupt 在 background transition 期间到达，会在 completion 前从 current reference 抢占到 user。 | 是 | 保持。背景 target 不写 history；user work 优先通过抢占路径处理，不等 idle 播放完成。 | Current/P1 | 已覆盖 background queue/interrupt 抢占 target idle；保留 user->idle/idle->idle 回归。 |
| transition target standby | transition completion | 没有 user queue/interrupt 抢占时，target standby 进入 standby playback，仍用 `ActiveKind::Transition`；playback 完成后回 `GeneralTrackerIdle` / public standby。若 user queue/interrupt 到达，会从 current reference 抢占到 user。 | 是 | 保持。背景 target/playback 不写 history；user work 优先通过抢占路径处理。 | Current/P1 | 已覆盖 background queue/interrupt 抢占 target standby 和 standby playback。 |
| waiting user start gate | readiness failure before start | start readiness 非 OK 时不 pop waiting；queue 保留；非 fault 进入 Passive，fault 进入 Fault。 | 是 | 保持安全优先；ready 恢复后仍需 operator 从 Passive/Fault 路径恢复，不自动跨安全边界启动 queued user。 | P0 | 已有 start gate keeps queued work；保留。 |
| active user preparing/start | non-fault readiness failure | active request reset 为 `Queued`、frame 归 0、放回 waiting front 并 publish queued；进入 Passive。fault readiness 则 fail active 并进入 Fault。 | 是 | 保持。preparing/start 还未真正运行 user policy，非 fault readiness failure 不应写 failed history。 | P0 | 已有 preparing gate keeps queued work；补 fault 分支。 |
| active user running/holding | readiness/write/model failure | 按现有 helper publish failed/stopped；非 fault readiness 进入 Passive，fault/model failure 进入 Fault；readiness failure 的 target/user `stop_reason` 保持 `None`。 | 是 | 保持安全优先；不要把 running/holding 回放到 waiting。 | P0 | 已有部分测试；按错误类型补齐。 |
| active idle readiness failure | readiness failure | idle stop，无 user history；进入 Passive/Fault。 | 是 | 保持。idle 不能掩盖 safety。 | P0 | 已有 bad orientation idle 测试。 |
| transition target user | target start/readiness/model failure | target user publish `Failed`，transition 自身无 history；readiness 非 OK 进入 Passive/Fault，model failure 进入 Fault；failed target `stop_reason` 为 `None`。 | 是 | 保持。target user failure 必须可通过 user run id 查询。 | P0 | 补 target user readiness/model failure 断言。 |
| transition target idle/standby | target start/readiness/model failure | target idle/standby 不写 user history；按现有 helper abort background target，进入 Passive/Fault 或 safe idle/standby fallback。 | 是 | 保持不写 idle/standby history；失败不能伪造成 user run。 | P0 | 补 target=idle/standby readiness/model failure。 |
| lowcmd occupied/manual | execute/readiness/control | API 将 lowcmd occupied 导向 manual，不自动抢回 LowCmd。 | 是 | 保持。user `.trk` 不得绕过 manual/operator 边界。 | P0 | 保留 API/readiness 测试。 |

## 4. 回归锁定和剩余缺口（按交付边界）

| Class | Item | Why it matters | Required action |
| --- | --- | --- | --- |
| P0 regression lock | 不得让 user `.trk` 进入 starting/public idle/passive/fixstand/fault/urgent_stopping 的外部 API 合同必须持续锁住 | 安全/恢复/非 accepting 状态不能被动作请求打断；但 readiness/manual error 必须先于 controller conflict。 | 保留 `/execute` readiness-before-controller gate order；回归测试覆盖 queue/interrupt、readiness 非 OK、readiness OK conflict，以及 validator/sink/id generator 未调用。 |
| Current/P1 regression lock | `transition.target=idle` 中 user queue 抢占背景目标 | idle 是背景；queued user 不应等 target idle 启动或播放完成。 | 保留 `[runtime-p1]` queue preempts background transition 测试，断言从 current transition frame 到 user。 |
| Current/P1 regression lock | `transition.target=standby` synthetic 中 user queue 抢占背景目标 | standby 是背景；queued user 不应等 standby playback。 | 保留 `[runtime-p1]` target standby queue 抢占测试，断言旧 standby target 被丢弃且不继续 playback。 |
| Current/P1 regression lock | standby playback 中 user queue 抢占背景播放 | standby playback 仍是 background reference。 | 保留 `[runtime-p1]` standby playback queue 抢占测试，断言从 current playback frame 到 user。 |
| Current/P1 regression lock | `transition.target=user` 中 user queue 等待 | 当前 transition 已归属前景 user target，queue 不应抢占。 | 保留 `[runtime-p1]` user-owned transition waits 测试。 |
| Current/P1 regression lock | queue 抢占 background transition/playback 的失败语义 | target user load/align/transition build 失败时不能恢复旧背景，也不能写 idle/standby history；用户动作打断 idle 不应清 idle config。 | target user publish `Failed`、`stop_reason=None`；停止当前 background/idle active 播放但保留 idle config；old background target 不写 user history；不恢复旧 background transition/playback。 |
| Current/P1 regression lock | user/background/playback target-specific interrupt 行为 | interrupt 对 user-owned transition 取消旧 target 后 current-frame 到 urgent user；对 background transition/playback current-frame 抢占，并取消本地 waiting。 | 保留 `[runtime-p1]` target-specific interrupt tests，断言 source frame 是被打断 transition/playback 当前帧。 |
| P0 regression lock | active user running interrupt smooth success/fallback | running GeneralTracker user interrupt 优先 current-frame transition；benign reject 回到 controlled stop；safety/readiness/fault 仍最高优先。 | 新增 running smooth handoff、fallback、no duplicate waiting、safety wins 测试；旧 running controlled-stop 测试只保留为 fallback 场景。 |
| Current/P1 regression lock | active user holding queue/interrupt handoff | holding interrupt 从 held frame 到 replacement user，旧 source `Stopped/Interrupt` 且 queue empty；holding queue 复用 handoff，旧 source `Done/None`；合法 target 的 benign no-transition 不得 `INTERNAL_ERROR` 丢失。 | 保留 held_interrupt_handoff、holding queue handoff、benign fallback 回归。 |
| P0 regression lock | active user preparing interrupt remains controlled stop | Preparing 继续 controlled stop/restart，不尝试 current-frame transition。 | 保留 `[runtime-p1]` preparing interrupt controlled stop 测试。 |
| P0 regression lock | `/passive`、`/fixstand` HTTP gate 不能被 runtime sink 语义掩盖 | `/passive` 必须有正确 password；lowcmd/manual gate 仍拒绝；readiness OK 可进 sink；readiness 非 OK 仅 `ROBOT_BAD_ORIENTATION` 且 `block=="bad_orientation"` 可作为 safety exception 进 sink；其他 readiness/fault error 拒绝。`/fixstand` 只有 readiness OK 或 bad_orientation recovery 才进入 sink。 | 回归测试分别覆盖 admitted command 与 gate rejected command；rejected path 不进 sink。 |
| P2 backlog | active idle 收到 `/standby` 后因 idle config 保留而可能自动重新 idle 的合同需要锁定 | `/standby` 是 control command，不是 user run；保留 idle config 后，回到可播放 idle 的 standby 链路即可继续由 background idle manager 管理。 | 锁定产品合同：`/standby` 停止当前 active idle、保留 idle config；若无 user work 且 ready/safe，idle 可按现有自动播放规则重新启动。需要纯 `standby` 时，caller 先 `/idle {"paths":[]}` 再 `/standby`。 |
| P2 backlog | preparing 状态 queue 行为可补专门测试 | preparing 与 running 同属 user-owned foreground；interrupt controlled stop 已覆盖。 | 如需更细覆盖，可补 active user preparing + queue 测试；不影响 Current/P1 合同。 |
| P2 backlog | control command abort transition 缺 target 分类覆盖 | urgent_stop/passive/fixstand 以及 `/standby` abort active transition 时都不应继续 smoothing。 | 对 target=user/idle/standby 分别测试 abort、target user status、reference clear。 |

## 5. Current 状态机规则

### 5.1 User work 优先级规则

| 当前 owner | queue | interrupt |
| --- | --- | --- |
| none / standby | 立即按 standby->user gate 启动；source 使用 standby reference。 | 同 queue。 |
| idle active | 抢占 idle；从 idle current frame 到 user first frame。 | 取消 waiting；从 idle current frame 到 urgent user first frame。 |
| user preparing | FIFO 等当前 user。 | 取消 waiting；当前 user controlled stop/settle；urgent post-stop 启动。 |
| user running | FIFO 等当前 user。 | GeneralTracker->GeneralTracker 优先 smooth handoff；benign reject fallback controlled stop/settle；LocoUpper/safety paths unchanged。 |
| user holding | 从 held frame 到 next user；旧 source `Done/None`。 | 取消 waiting；从 held frame 到 replacement user；旧 source `Stopped/Interrupt`，queue empty。 |
| transition target user | 保留当前 target user，新 user 排队。 | 取消旧 target；从 current transition frame 到 urgent user。 |
| transition target idle | 抢占背景 idle target；从 current transition frame 到 user。 | 同 queue，并取消 waiting。 |
| transition target standby | 抢占背景 standby target；从 current transition frame 到 user。 | 同 queue，并取消 waiting。 |
| standby playback | 抢占背景 playback；从 current playback frame 到 user。 | 同 queue，并取消 waiting。 |
| starting/public idle/passive/fixstand/fault/urgent_stopping | readiness OK 时 API controller gate 拒绝；readiness 非 OK 时 readiness/manual error 优先。 | 同 queue。 |

### 5.2 Control 命令规则

| 命令 | 推荐规则 |
| --- | --- |
| `/urgent_stop` | 最高优先级；取消 stop watermark 之前的 queued/pending user；清 idle config；active user 进入 urgent_stopping；idle/transition 立即 abort；不 smoothing；不播放 standby reference；idle FixStand 只清理本地状态并留在 FixStand。 |
| `/passive` | HTTP 入口必须有正确 password；lowcmd/manual gate 仍拒绝；readiness OK 时可进入 runtime sink；readiness 非 OK 时仅 `ROBOT_BAD_ORIENTATION` 且 `block=="bad_orientation"` 作为 safety exception 可进入 runtime sink；其他 readiness/fault error 拒绝，不进 sink。命令进入 runtime 后立即 safety sink，取消 waiting，清 idle config，active user stopped，idle/transition abort，clear reference，不 smoothing。 |
| `/fixstand` | HTTP 入口只在 readiness OK 或 bad_orientation recovery 场景进入 sink；其他 Fault/readiness error 先返回错误。runtime 命令效果是 recovery/control path；不清 idle config；FixStand 期间绝不自动播放 idle，之后必须显式 `/standby` 才可回到可播放 idle 的 standby 链路；active user controlled stop 后进入 FixStand；idle/transition abort；不 smoothing。 |
| `/standby` | ordinary standby/control path；不清 idle config；不生成 user run、不进 queue；active user running/preparing controlled stop；静态 user holding 可走 held reference -> standby reference gated transition；active idle/transition abort。回到可播放 idle 的 standby 链路后，若 idle config 非空且无 user work、ready/safe，idle 可按现有 background idle manager 自动播放规则重新启动。需要纯 `standby` 时，caller 先 `/idle {"paths":[]}` 再 `/standby`；`/urgent_stop` 保留为 urgent/immediate 停止。 |
| `/idle` nonempty | 只配置背景池；不生成 run；不跨 passive/fixstand/fault/urgent_stopping 自动播放。 |
| `/idle` clear | 任意状态安全清空。 |

### 5.3 Synthetic transition 归属规则

| 目标 | run id | queue.limit | history | status |
| --- | --- | --- | --- | --- |
| target user | user run id 只属于 target user；transition 自身无 id | 不占 | transition 不进 history；target user 正常 publish queued/running/failed；failed target `stop_reason=None` | `active.kind:"transition"`，`transition.target:"user"`，`transition.target_id:<run id>` |
| target idle | 无 | 不占 | 不写 user history；失败也不写 idle history | `active.kind:"transition"`，`transition.target:"idle"` |
| target standby | 无 | 不占 | 不写 user history；失败也不写 standby history | `active.kind:"transition"`，`transition.target:"standby"` |

## 6. 安全边界

| Boundary | 必须行为 |
| --- | --- |
| passive | user `.trk` 拒绝。HTTP `/passive` 不是无条件 any-state 后门：必须有正确 password；lowcmd/manual gate 仍拒绝；readiness OK 时可进入 runtime sink；readiness 非 OK 时仅 `ROBOT_BAD_ORIENTATION` 且 `block=="bad_orientation"` 可作为 safety exception 进入 sink；其他 readiness/fault error 拒绝，不进 sink。runtime admitted command 是 safety sink。 |
| fixstand | user `.trk` 拒绝；idle config 可保留但不得自动播放；先 `/standby` 后才能回到可播放 idle 的 standby 链路。HTTP `/fixstand` 仅在 readiness OK 或 bad_orientation recovery 时进入 sink；其他 Fault/readiness error 先返回错误。 |
| fault | user `.trk` 拒绝；恢复入口是受 gate 保护的 `/fixstand` 或 operator/manual path，不是任意 Fault 都直接进 FixStand sink。 |
| urgent_stopping | 外部 user `.trk` 拒绝；已被 low-level bridge 接收的 post-stop work 只能在 urgent_stopping 结束后启动。 |
| legacy/internal Idle | `/execute` 也被 controller gate 拒绝；它不是 user `.trk` accepting state。 |
| lowcmd occupied/manual | 不自动抢回 LowCmd；API `next:"manual"`。 |
| bad orientation | 不得因 user `.trk` 绕过；`/passive` 仅在正确 password 且 `ROBOT_BAD_ORIENTATION` + `block=="bad_orientation"` 时可作为 safety exception 进 runtime sink；`/fixstand` recovery 例外按现有 readiness gate。 |
| urgent_stop/passive/fixstand/standby | safety/control 命令不生成 user run、不进入 queue。urgent_stop/passive/fixstand 不走 smoothing；`/standby` 仅允许静态 user holding -> standby 例外。 |

## 7. 验收测试矩阵

| Area | Test case | Expected |
| --- | --- | --- |
| idle preempt | active idle + queue user | `active.kind:"transition"`，`transition.target:"user"`，source frame 等于 idle current reference。 |
| idle preempt | active idle + interrupt user | 同上，并取消旧 waiting。 |
| idle failure | idle->user target load/validation fail | target user `Failed`；当前 idle 播放停止但 idle config 保留；无 idle history。 |
| standby no-active | no active standby + queue user | 从 standby reference 到 user first frame transition。 |
| standby no-active | no active standby + interrupt user | 与 queue 等价；无 active 可中断。 |
| foreground queue | active user running + queue | 新 user 在 `queue.ids`；当前 user 不被打断。 |
| foreground interrupt | active user running + GeneralTracker interrupt | 成功时 `active.kind:"transition"`、`transition.target:"user"`、`transition.target_id` 为新 run，旧 user `stopped` + `stop_reason:"interrupt"`；fallback 时 controlled stop/restart。 |
| holding queue | active user holding + queue | held frame -> next user transition；held user `Done/None`；benign no-transition fallback 不 `INTERNAL_ERROR` 丢 target。 |
| holding interrupt | active user holding + interrupt | waiting canceled；held frame -> replacement user transition；old source `Stopped/Interrupt`；queue empty；benign no-transition fallback 不 `INTERNAL_ERROR` 丢 target。 |
| transition user queue | `transition.target=user` + queue | 旧 target user 保持；新 user 排队。 |
| transition user interrupt | `transition.target=user` + interrupt | 旧 target canceled；current transition frame -> urgent user。 |
| transition idle queue | `transition.target=idle` + queue | 背景 idle target canceled；current transition frame -> user。 |
| transition idle queue failure | `transition.target=idle` + queue user load/align/build fail | target user `Failed` with `stop_reason=None`；old idle target 无 user history；停止当前 background/idle active 播放但保留 idle config；不恢复旧 background transition。 |
| transition idle interrupt | `transition.target=idle` + interrupt | current transition frame -> urgent user。 |
| transition standby queue | `transition.target=standby` + queue | 背景 standby target canceled；current transition frame -> user；不继续 standby playback。 |
| transition standby queue failure | `transition.target=standby` + queue user load/align/build fail | target user `Failed` with `stop_reason=None`；old standby target 无 user history；停止当前 background playback；idle config 如已配置则保留；不继续 standby playback。 |
| transition standby interrupt | `transition.target=standby` + interrupt | current transition frame -> urgent user。 |
| standby playback queue | standby playback active + queue | current playback frame -> user；不等 playback 完成。 |
| standby playback queue failure | standby playback active + queue user load/align/build fail | target user `Failed` with `stop_reason=None`；old standby playback 无 user history；停止当前 background playback；idle config 如已配置则保留；不恢复 standby playback。 |
| standby playback interrupt | standby playback active + interrupt | current playback frame -> urgent user。 |
| user completion | user done + idle configured + no waiting | user->idle transition；user done；idle no run id/history。 |
| user completion | user done + no idle + standby asset | user->standby transition then standby playback；no user history for transition/playback。 |
| idle completion | idle done + idle pool + no waiting | idle->idle transition；no user history。 |
| readiness start | queued/preparing user + non-fault readiness failure | queued user remains or is put back in `queue.ids`; enter Passive; no failed history。 |
| readiness target user | transition target user + readiness/model failure | target user `Failed` with `stop_reason=None`; transition no history; enter Passive/Fault as mapped。 |
| readiness background target | transition target idle/standby + readiness/model failure | no user history for idle/standby; enter Passive/Fault or helper fallback。 |
| urgent_stop | active transition + `/urgent_stop` | transition abort；target user canceled if any；reference clear；no standby playback。 |
| passive runtime | admitted `/passive` + active user/idle/transition | immediate passive; no smoothing; waiting and idle cleared。 |
| passive HTTP gate | `/passive` with correct password + readiness OK | admitted into runtime sink。 |
| passive HTTP gate | `/passive` with correct password + `ROBOT_BAD_ORIENTATION` + `block=="bad_orientation"` | admitted into runtime sink as safety exception。 |
| passive HTTP gate | `/passive` without correct password or with lowcmd/manual gate failure | rejected before runtime sink; no command side effect。 |
| passive HTTP gate | `/passive` with other readiness/fault error | rejected before runtime sink; no command side effect。 |
| fixstand runtime | admitted `/fixstand` + active user/idle/transition | user controlled stop or background abort; no smoothing；idle config 保留但 FixStand 期间不自动 idle。 |
| fixstand HTTP gate | `/fixstand` with readiness OK or bad_orientation recovery | admitted into runtime sink。 |
| fixstand HTTP gate | `/fixstand` with other Fault/readiness error | returns error before runtime sink。 |
| standby explicit | active user holding + `/standby` | held reference -> standby reference transition；不生成 user run、不进入 queue。 |
| standby explicit | active user running/preparing + `/standby` | controlled stop；post-stop standby；不做 current-frame smoothing。 |
| standby explicit | active idle + `/standby` | idle stops；idle config 保留；回到可播放 idle 的 standby 链路后，若无 user work 且 ready/safe，可按现有 background idle manager 规则自动重新 idle；不生成 user run、不进 queue。 |
| safety | active idle/user/transition + readiness failure | enter Passive/Fault as mapped; user failed/stopped where applicable; idle/transition no history。 |
| API gate | starting/public idle/passive/fixstand/urgent_stopping/fault + `/execute` queue/interrupt, readiness OK | 409 `CONTROL_STATE_CONFLICT`，不调用 validator/sink/id generator。 |
| API gate order | blocked ctrl + `/execute`, readiness non-OK | readiness/manual error wins over `CONTROL_STATE_CONFLICT`，不调用 validator/sink/id generator。 |
| API shape | `/execute` with transition params or unknown fields | 400 `REQUEST_INVALID`; API 不扩大。 |

### Current/P1 回归测试集

已收敛并应保留：

- `transition.target=idle + queue`：背景 idle target canceled；从 current transition frame 到 user；`active.kind:"transition"`、`transition.target:"user"`、`transition.target_id` 为 queued user id。
- `transition.target=user + queue`：旧 target user 保持；新 user 只进入 waiting/queue；不得抢占当前 user transition。
- `transition.target=standby + queue`：背景 standby target canceled；从 current transition frame 到 user；不继续 standby playback。
- `standby playback active + queue`：从 current playback frame 到 user；不等 playback 完成。
- background queue 抢占失败：target user `Failed` with `stop_reason=None`；old idle/standby/standby playback 不写 user history；不恢复旧 background transition/playback；停止当前 background/idle active 播放但保留 idle config。
- holding queue/interrupt handoff：holding 是 foreground user intent；interrupt 从 held frame 到 replacement user，旧 source `Stopped/Interrupt` 且 queue empty；queue 复用 held-frame handoff，旧 source `Done/None`；合法 target 的 benign no-transition fallback 不产生 `INTERNAL_ERROR`。
- target-specific interrupt 回归：running user interrupt 成功时 current-frame 到 urgent user；fallback 时 controlled stop；`transition.target=user` interrupt 取消旧 user target；`transition.target=idle`、`transition.target=standby`、standby playback interrupt 都从 current reference 到 urgent user。
- status/API 回归：agent 不能只用 `ctrl:"running"` 判断 user motion；断言 `active.kind`、`transition.target/target_id`、`exec.id`、`queue.ids` 一致；blocked `/execute` 保持 readiness/manual error before controller conflict，且不调用 validator/sink/id generator。

其他回归锁定，不扩大为新主路径：

- `/standby`：holding -> standby 平滑例外、running/preparing controlled stop、active idle 保留 idle config 且可能按现有 background idle manager 重新 idle。
- `/fixstand`：admitted command 不 smoothing；active idle 停止且保留 idle config；FixStand 期间不自动 idle；HTTP 只在 readiness OK 或 bad_orientation recovery 进入 sink。
- `/passive`：正确 password、lowcmd/manual gate、readiness OK admitted、bad_orientation exception admitted、其他 readiness/fault error rejected 与 runtime safety sink 分开锁定。

## 8. Maintenance Guidance

| Topic | Guidance |
| --- | --- |
| API | 不新增 HTTP endpoint，不新增 `/execute` 参数，不暴露 transition profile。继续使用现有 `/execute {path, mode, hold}`；保持 readiness-before-controller gate order。 |
| Current frame source | 复用 current reference helper / `TrkFrameView` 路径；不要从 status JSON 反解析 reference。 |
| Synthetic transition | 继续用 internal in-memory `TrkTrack`；不落盘；不生成 run id；不进 queue/history。 |
| Background target detection | 在 runtime 内部区分 `transition.target=user` 与 `transition.target=idle/standby`，并把 standby playback 也视为 background owner。只有 background target/playback 可被 queue 抢占。 |
| Queue handling during transition | `CommandKind::Queue` 遇到 active transition/playback 时，如果 owner 是 background，应走 current-frame-to-user helper；如果 owner 是 user target，保持 waiting。 |
| Background preempt failure | queue 抢占 background transition/playback 时，若 target user load/align/transition build 失败，publish target `Failed`、`stop_reason=None`；停止当前 background/idle active 播放但保留 idle config；不要恢复旧 background transition/playback；old idle/standby target 无 user history。 |
| Interrupt handling during transition | 对 background transition/playback 保持 current-frame-to-urgent helper；对 user-owned transition 取消旧 target user 后重建到 urgent user。 |
| Active user interrupt | running GeneralTracker user interrupt 复用 current-reference synthetic transition helper；holding GeneralTracker user interrupt 复用同一 robust handoff 从 held frame 到 replacement user，成功后旧 source `Stopped/Interrupt` 且 queue empty；benign reject fallback 不把合法 target 标成 `INTERNAL_ERROR`。Preparing、LocoUpper、urgent_stop/passive/fixstand/standby 不使用该 user handoff。 |
| Urgent/passive/control | urgent_stop/passive 清 idle config；fixstand/standby 保留 idle config。urgent_stop/passive/fixstand 不调用 smoothing helper；`/standby` 仅允许静态 user holding -> standby 例外。 |
| Standby playback | 视为 background transition owner；queue/interrupt 都可抢占。 |
| Standby active idle | 保留 idle config；不新增 idle 保留但禁播的 runtime 状态。`/standby` 回到可播放 idle 的 standby 链路后，若 idle config 非空且无 user work、ready/safe，idle 可按现有 background idle manager 规则自动重新启动；这不是 `/standby` 队列化或生成 user run。需要纯 `standby` 时，caller 先 `/idle {"paths":[]}` 再 `/standby`。 |
| Tests | 保留 target-specific `[runtime-p1]` tests；API gate tests 确保 starting/public idle/passive/fixstand/urgent_stopping/fault 在 readiness OK 时拒绝 user `.trk`，且 `/execute` readiness 非 OK 时返回 readiness/manual error。另锁定 `/passive`：正确 password + readiness OK admitted、正确 password + bad_orientation exception admitted、无正确 password/lowcmd/manual/其他 readiness/fault error rejected，rejected path 不进 runtime sink。 |
