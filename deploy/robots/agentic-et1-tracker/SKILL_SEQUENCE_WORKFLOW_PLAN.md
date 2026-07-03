# ET1 Skill Sequence Workflow 后续开发计划

> **Historical/Obsolete API names (2026-07):** This file is a historical
> planning handoff and may still mention old public route names. The GA/current
> API is `POST /standby` for ordinary standby and `POST /urgent_stop` for urgent
> stop. Legacy `POST /standby_velocity` and `POST /stop` are not successful
> aliases; if present they reject with `CONTROL_ROUTE_RENAMED`. Use `README.md`
> and `CONTROL_STATE_MACHINE_REDESIGN_PLAN.md` as the current contract.

更新日期：2026-06-05

本文档只规划 ET1 动作生成与执行 skill/workflow 层的最小改进。目标是让
agent 面对长动作序列、多段生成、异步执行和用户中途插话时更可靠、更低延迟。

## 1. 目标与非目标

目标：

- 在 skill/workflow wrapper 层补一个薄调度入口，统一组合 preset、nl2trk
  和 trk2motion。
- 长序列默认异步执行，尽快返回可轮询的 sequence id，避免 agent 被
  `--wait` 长时间阻塞。
- 支持多段动作的生成/执行流水线：第一段生成完成后立即执行，后续段在执行中继续准备。
- 明确用户插话语义：追加、替换尾部、打断、取消到 standby、紧急停止各自有稳定映射。
- 保持输出短、确定、机器可读：一行 compact JSON，错误有明确 `code` 和 `next`。
- 保持现有低层能力复用，降低 skill 文档负担，减少 agent 临场拼命令的机会。
- wrapper 维护本地 sequence state；P0 只向 tracker 提交当前段，下一段可以生成、stage、ready，但必须留在 wrapper 本地 state，等当前段结束后再提交。

非目标：

- 不改 `agentic-et1-tracker` 服务端 HTTP 接口。
- 不做 botified fast direct path。
- 不把 tracker 扩展成 playlist、任务数据库、云端调度器或动作 catalog 服务。
- 不改变 tracker 的职责：tracker 仍只执行本地绝对路径 `.trk`，并提供已有控制/status 接口。
- 不假设 tracker 支持按 run id 删除、替换或重排未来队列；公开接口没有这个能力。
- 不在本计划中重写 Kimodo/motion2trk 服务、preset 匹配算法或 `.trk` 格式。
- 不新增复杂优先级、权重、图形界面、远程 URL、上传、非 `.trk` payload。

约束：

- KISS：workflow CLI 只做 agent 已经需要的编排，不提前设计大系统。
- DRY：路径 staging、preset 查询、nl2trk 调用、trk2motion 调用、输出 envelope
  只封装一处，不在多个 SKILL.md 里复制流程。
- YAGNI：P0 只交付能解决阻塞等待、长序列和插话的最小闭环。

## 2. 当前问题

当前 skill 单动作路径基本可用：

- `et1-nl2trk` 能从自然语言生成单个 `.trk`。
- `et1-nl2trk-preset` 能优先复用常见 preset。
- `et1-trk2motion` 能执行 `.trk`、设置 idle、进入 standby、做显式 urgent stop。

主要短板出现在 agent 编排层：

- 长动作序列需要 agent 手动拆段、生成、排队、等待和续接，容易产生重复逻辑。
- `run --wait` 或 `wait` 会阻塞当前 agent 回合；动作执行期间用户插话时，agent
  可能无法及时响应。
- 多段生成没有统一流水线：如果所有段都先生成再执行，首动作延迟高；如果边生成边执行，
  当前没有统一状态和错误契约。
- 如果 wrapper 一次性把所有段提交进 tracker queue，后续 `replace_tail` 无法可靠删除已经提交的未来 run。
- `serial_id` 连续生成的规则需要 agent 记忆，长序列中容易漏传或错误并发。
- 普通“停下”和紧急停止的区别已经在 trk2motion skill 中定义，但长序列层没有统一取消语义。
- preset idle loading 目前散落在文档描述中，agent 仍可能手工查找、复制和设置 idle。

## 3. 推荐最小架构

新增一个薄包装 workflow CLI，建议命名为 `et1-action`。它不是 tracker 服务端的一部分，
也不改变 tracker HTTP schema。它只在 skill/workflow 层组合已有工具：

```text
user intent
  -> et1-action
       -> preset lookup/staging
       -> nl2trk generation when preset misses
       -> trk2motion run/status/standby/stop/idle
       -> local sequence state file
  -> tracker executes .trk only
```

职责边界：

- `et1-action` 负责意图到 workflow 命令、长序列状态、异步 job、插话修改队列。
- `et1-nl2trk-preset` 负责常见动作复用和 preset staging。
- `et1-nl2trk` 负责生成单段 `.trk`，包括连续动作所需 `serial_id`。
- `et1-trk2motion` 负责把单个 `.trk` 交给 tracker、查询状态、standby、urgent stop。
- tracker 只接收 `.trk` 执行请求和已有控制请求，不知道“长序列”这个概念。
- wrapper 是 sequence queue 的权威；P0 tracker queue 只用于当前段执行，不作为可编辑长序列存储或未来段缓冲。

本地状态建议：

- 使用普通本地 JSON 状态文件即可，例如 `$ET1_ACTION_STATE_DIR`，默认落在技能数据目录。
- 每个 sequence 一个 `seq_id`，记录 segments、当前运行 run id、已提交到 tracker 的 segment、生成状态、取消标志、错误、更新时间和 worker heartbeat。
- P0 不需要 daemon。异步可先用后台子进程和状态文件实现；status/cancel 等命令只读写状态并调用 trk2motion。
- 如果后续发现进程生命周期不可靠，再评估轻量 worker；不要在 P0 引入常驻服务。
- `sequence-status` 必须能识别后台 worker stale：如果 `state` 是 `starting|running|waiting_for_generation|canceling` 且 heartbeat 超过阈值未更新，输出 `state:"failed"`/`error.code:"WORKER_DIED"` 或 `state:"stale"`/`error.code:"WORKER_STALE"`，并给出 `next:"sequence-status"`。说明文字或 debug/verbose 可以提示用户/agent 也可以选择 `sequence-cancel`，但默认 JSON 不能输出组合 next。
- wrapper 不自动 follow tracker 或 trk2motion 返回的 `next`。特别是不能自动执行 `passive -> fixstand -> standby` 恢复链；这些只在 operator 或用户显式控制时发生。

## 4. 核心命令设计

所有命令默认输出一行 compact JSON。命令名保持动词清晰，不暴露内部实现细节。

P0 一等命令只包括：`run-text`、`run-trk`、`sequence-start`、`sequence-status`、
`sequence-append`、`sequence-replace-tail`、`sequence-cancel`、`sequence-interrupt`、
`standby`、`urgent-stop`、`idle-load`。

### 单次动作

`run-text`：

- 输入自然语言动作。
- 先查 preset；命中则 stage preset；未命中则调用 nl2trk。
- 默认提交给 trk2motion 执行。
- 短动作可选 `--wait`，但默认不 wait。
- skill 产品默认用 tracker `mode:"interrupt"` 提交新用户意图；如果用户明确要求“排在当前动作后面”，才显式 `--mode queue`。raw HTTP `/execute` / `/execute_loco_upper` 省略 `mode` 时仍默认 `queue`，这是服务端合同，不因 skill 默认而改变。

`run-trk`：

- 输入本地绝对 `.trk`。
- 直接调用 trk2motion `run`。
- 用于兼容现有工作流和调试。
- 默认同样使用 tracker `mode:"interrupt"`；需要保留当前 tracker work 时显式 `--mode queue`。

### 序列控制

`sequence-start`：

- 显式启动长序列。
- 返回 `seq_id`，不等待整段完成。
- 支持 `--serial-id` 或自动生成连续序列 id。

`sequence-status`：

- 查询 sequence 摘要、当前 active run、已完成/生成中/排队/取消段、最后错误。
- 可选 `--watch` 仅用于人类调试；skill 默认不要用长时间 watch。

`sequence-cancel`：

- 取消 sequence 的未开始段。
- 对正在执行的当前段，默认调用 `standby`，让机器人回到普通待命。

`sequence-append`：

- 在尾部追加新段。
- 如果 sequence 仍在运行，新段加入后续生成/执行流水线。

`sequence-replace-tail`：

- 只替换 wrapper 本地尚未提交给 tracker 的尾部段。
- 当前正在执行的段默认不中断。
- 已提交到 tracker 的 run 不能假设可删除；如果必须改变已经提交或正在执行的动作，必须走 `sequence-interrupt` 或 `standby`。
- 如果用户明确要求“现在改成/别做后面的/后面换成”，映射到这个命令。

`sequence-interrupt`：

- 用户明确要求“立刻/马上/打断当前动作并改做...”时的稳定入口。
- 取消 wrapper 本地未提交段。
- 如果目标 `.trk` 已 ready/staged，或 preset hit 后已经完成 staging，立即调用
  trk2motion `run --mode interrupt`。
- 如果目标动作还需要 nl2trk 生成，P0 最小语义是先执行 `standby`，让当前 sequence
  cancel_to_standby；然后后台生成，生成完成后普通执行，不承诺无缝立即切。

### 控制与 idle

`idle-load`：

- 封装 idle preset 查找、staging 和 trk2motion `idle set`。
- 用户说“加载 idle 动作/放松点/别直挺挺站着”时走这个命令。
- 只配置 idle pool，不提交用户 run，不调用 urgent stop。

`standby`：

- 进入 StandbyVelocity/普通站立待命。
- 普通“停下/停止/不要动/站着别动”在长序列层映射为 `cancel_to_standby`/`standby`。
- 普通停止必须保留 idle 配置，不调用 urgent stop，不清 idle pool。

`urgent-stop`：

- 只在用户明确表达紧急停止、abort、kill、紧急停止、赶快停止时使用。
- 调用 trk2motion `stop --urgent`。
- 不把普通取消、状态查询、用户犹豫或“等一下”映射到 urgent stop。

`fixstand` 和 `passive`：

- 不作为 `et1-action` P0 一等命令，不进入普通 hot path。
- 仅作为 progressive disclosure/operator 显式控制能力保留在底层 trk2motion skill 中。
- wrapper 不因为 `ready` 的 `next` 字段自动调用 `fixstand`、`passive` 或 `standby`。

## 5. 长序列调度

默认策略：

- 长序列默认异步，立即返回 `seq_id`。
- 第一段生成完成后立即执行，降低首动作延迟。
- 后续段在前一段执行期间生成并 stage 到 wrapper 本地 ready 状态，形成“当前段执行 -> 下一段生成/stage/ready -> 当前段结束后再提交”的流水线。
- 默认不并发生成，避免 Kimodo `serial_id` 连续性、资源占用和错误恢复复杂化。
- P0 不一次性提交整套长序列到 tracker queue，也不提交任何未来段；wrapper 只提交当前段。
- 下一段可以是 `generating|ready`，但必须留在 wrapper 本地 state；当前段结束后，worker 才能提交下一段。
- 已提交到 tracker 的段视为当前段且不可编辑。wrapper 可以取消本地未提交段，但不能承诺删除、替换或重排 tracker 内部 run。

连续动作：

- 如果动作段需要自然连续，使用同一个 `serial_id` 顺序调用 nl2trk。
- 同一个 `serial_id` 的段必须按顺序生成，不得并发。
- `sequence-start` 未指定 `serial_id` 时，workflow 可生成短稳定 id，并在状态中记录。
- preset 段参与 sequence 时不更新 Kimodo continuity；其前后是否继续使用同一
  `serial_id` 由 workflow 状态显式记录，避免隐式猜测。

独立动作：

- 独立段理论上可并发生成，但 P0 默认不并发。
- P1 可增加 `--parallel-independent`，仅对明确标记为独立且不共享 `serial_id` 的段启用。
- 并发失败不能影响已执行段的状态记录；错误必须落到 segment 级别并给出 `next`。

执行队列：

- workflow 只把当前要执行的 ready `.trk` 交给 trk2motion；本地 ready 的下一段不能提前提交。
- 不要求 tracker 支持 `paths` 或 playlist。
- 不要求 tracker 支持按 run id 删除/替换未来队列；这是 P0 设计的硬边界。
- `replace_tail`、`append`、`cancel` 的主要操作对象是 wrapper 本地 state 中尚未提交的 segments。
- `sequence-cancel`/`standby` 只需停止当前段并清理本地后续；因为 P0 没有 tracker 未来段提交，不会遗留 tracker 已提交的未来 run。
- 如果下一段未及时生成完成，sequence 状态显示 `waiting_for_generation`，不要忙等刷屏。
- 如果 tracker 返回控制状态冲突，workflow 可以保留底层 `error.code` 和
  `error.message`，并补充 sequence 的 `seq_id`、`segment_id`。默认 `next` 必须归一化为单个 P0 命令；底层 raw `next` 只能进入 `--debug`/`--verbose`。

## 6. 用户插话语义

用户插话必须固定映射为五类 workflow 意图，避免继续扩张：`append`、
`replace_tail`、`interrupt`、`cancel_to_standby`、`urgent_stop`。

`append`：

- 用户说“后面再加一个.../接着再.../做完这个再...”。
- 不影响当前段和已提交段，只在 wrapper 本地尾部追加。

`replace_tail`：

- 用户说“后面的不要了，换成.../之后改成.../别做后面那些了”。
- 保留当前正在执行段，只替换 wrapper 本地尚未提交给 tracker 的尾部。
- 如果目标尾部已经提交给 tracker，P0 不能承诺删除；应返回当前可替换数量，并把默认 `next` 指向 `sequence-interrupt`。说明文字或 debug/verbose 可以提示也可选择 `standby`。

`interrupt`：

- 用户说“立刻改做.../马上换成.../打断当前动作”。
- 使用 P0 一等命令 `sequence-interrupt`；取消 wrapper 本地未提交段。
- 只有新目标动作已经 ready，才可以立即使用 trk2motion `run --mode interrupt`。ready
  包括现成/staged `.trk`，或 preset hit 后已经完成 staging。
- 如果新目标动作还需要 nl2trk 生成，P0 最小语义是先执行 `standby`，让当前 sequence
  cancel_to_standby；然后后台生成，生成完成后再普通执行，不承诺无缝立即切换。

`cancel_to_standby`：

- 用户普通说“停下/停止/不要动/站着别动/先别做了”。
- 取消当前 sequence，清掉 workflow 未开始段，调用 trk2motion `standby`。
- 保留 idle 配置，不调用 `/stop`，不清 idle pool，不自动进入 fixstand/passive。

`urgent-stop`：

- 用户明确说“紧急停止/赶快停止/abort/kill/emergency stop”。
- 调用 trk2motion `stop --urgent`。
- 这是安全急停语义，会沿用底层 `/stop` 的强语义；不要用于普通停下。

插话输出应包含：

- `intent`：`append|replace_tail|interrupt|cancel_to_standby|urgent_stop`。
- `seq_id`：受影响的 sequence。
- `state`：修改后的 sequence 状态。
- `next`：下一步建议，默认只能是单个 P0 命令，例如 `sequence-status`、
  `sequence-interrupt`、`standby`、`run-text`。如果有多个可选动作，只在说明文字或
  `--debug`/`--verbose` 中描述，不放进默认 JSON。

## 7. 状态与输出契约

统一输出原则：

- stdout 永远一行 compact JSON。
- 默认字段只包含 `ok`、`cmd`、`intent`、`seq_id`、`state`、`active`、
  `segments`、`next`、`error`。不相关字段可以省略，但不要新增默认字段。
- 成功输出 `ok:true`；失败输出 `ok:false`，必须有 `error.code`、`error.message`、`next`。
- 默认不输出 `.trk` 路径、prompt、duration、完整 tracker status、完整 pose、长数组、
  大段日志或多行说明；这些只放在 `--debug`/`--verbose`。
- 子命令调用失败时，默认可以保留底层 `error.code` 和 `error.message`；workflow 只补充上下文，不重命名错误。默认 `next` 必须归一化为单个 P0 命令，底层 raw `next` 只能进入 `--debug`/`--verbose`。
- 输出的 `next` 只是给 agent/user 的建议，不触发 wrapper 自动执行后续控制命令。
- 默认 `next` 只能是单个 P0 一等命令：`run-text`、`run-trk`、`sequence-start`、
  `sequence-status`、`sequence-append`、`sequence-replace-tail`、`sequence-cancel`、
  `sequence-interrupt`、`standby`、`urgent-stop`、`idle-load`。

建议成功示例：

```json
{"ok":true,"cmd":"sequence-start","seq_id":"seq_20260605_001","state":"running","active":{"segment_id":"s1","run_id":"run_abc"},"segments":{"done":0,"running":1,"ready":0,"generating":1,"pending":2,"submitted":1,"failed":0},"next":"sequence-status"}
```

建议错误示例：

```json
{"ok":false,"cmd":"sequence-append","seq_id":"seq_20260605_001","error":{"code":"SEQUENCE_NOT_FOUND","message":"sequence does not exist"},"next":"sequence-status"}
```

最小状态文件字段：

- `seq_id`
- `state`：`starting|running|waiting_for_generation|canceling|canceled|completed|failed|stale`
- `active.segment_id`
- `active.run_id`
- `segments.done/running/ready/generating/pending/submitted/failed/canceled`
- `last_error`
- `updated_at`
- `heartbeat_at`
- `next`

`heartbeat_at` 只需要写入状态文件；默认 stdout 不必输出，除非 `state:"stale"` 或
`--debug`。stale 判定建议：

- worker 每次状态迁移、生成开始/结束、提交 tracker、轮询 tracker 后更新 heartbeat。
- `sequence-status` 看到 active state 且 heartbeat 超过阈值未更新时，不盲目认为 sequence
  仍在正常运行。
- 如果 worker 进程已确认不存在，输出 `state:"failed"`、`error.code:"WORKER_DIED"`。
- 如果无法确认进程是否存在但 heartbeat 超时，输出 `state:"stale"`、
  `error.code:"WORKER_STALE"`、`next:"sequence-status"`。说明文字或 debug/verbose
  可以提示也可选择 `sequence-cancel`。

错误 code 建议：

- `REQUEST_INVALID`
- `SEQUENCE_NOT_FOUND`
- `SEQUENCE_NOT_RUNNING`
- `SEGMENT_GENERATION_FAILED`
- `SEGMENT_STAGE_FAILED`
- `TRACKER_RUN_FAILED`
- `TRACKER_STATE_CONFLICT`
- `WORKER_STALE`
- `WORKER_DIED`
- `CANCELED`
- 以及透传的 trk2motion/nl2trk 既有错误。

## 8. Skill 文档改造

目标是让 SKILL.md 短而稳定，把细节放到 references，减少 agent 读长文档的延迟。

建议结构：

- `et1-action/SKILL.md`：只写何时使用、常用命令、插话映射和安全边界。
- `references/sequence-workflow.md`：长序列状态机、调度、serial_id 规则。
- `references/intent-mapping.md`：中文/英文用户说法到 append、replace_tail、
  interrupt、cancel_to_standby、urgent_stop 的映射。
- `references/output-contract.md`：一行 JSON、错误 code、`next` 字段。
- `references/compatibility.md`：如何兼容旧 `et1-trk2motion`、`et1-nl2trk`、
  `et1-nl2trk-preset` CLI。

已有 skills 调整：

- `et1-trk2motion` 继续聚焦单 `.trk` 执行、状态、standby、urgent stop、idle set/clear。
- `et1-nl2trk` 继续聚焦单段文本到 `.trk`，保留 `serial_id` 说明。
- `et1-nl2trk-preset` 继续聚焦 preset 查询/staging；需要同步安装到 `~/.agents`
  和 `~/.codex`，避免 agent 与 Codex skill 版本不一致。
- idle preset loading 从 trk2motion SKILL.md 的长段流程收敛到 `et1-action idle-load`，
  trk2motion 只保留底层 `idle set/clear` 合同。

DRY 要求：

- preset staging 目录选择只在 workflow CLI 中实现一次。
- compact JSON envelope 只在 workflow CLI 中实现一次。
- 用户插话映射只维护一份 references，不在多个 SKILL.md 重写。

## 9. 测试计划

采用小范围 TDD。先给 workflow CLI 增加 mock 测试，再实现最少逻辑。

P0 测试重点：

- 意图映射：普通“停下”映射 `cancel_to_standby`；“紧急停止/赶快停止”映射
  `urgent_stop`；“后面加...”映射 `append`；“后面的换成...”映射 `replace_tail`。
- 单动作兼容：`run-text` preset 命中时不调用 nl2trk；preset miss 时调用 nl2trk；
  最终调用 trk2motion run。
- 序列状态机：`sequence-start` 创建 `seq_id`；第一段 ready 后立即 run；后续段进入
  generating/pending/ready；P0 只提交当前段，下一段 ready 后仍留在 wrapper 本地 state；
  当前段结束后才提交下一段；完成后 `completed`。
- 取消/插话：`sequence-cancel` 清未开始段并调用 standby；`replace-tail` 不影响当前段；
  `replace-tail` 不能修改已 submitted 当前段；ready 目标的 `sequence-interrupt` 可用
  `mode=interrupt`；未 ready 目标的 `sequence-interrupt` 先走 `standby`，后台生成后普通执行；
  `sequence-interrupt` 不误用 urgent stop。
- worker 状态：heartbeat 正常更新；worker 死亡输出 `WORKER_DIED`；heartbeat 超时且进程未知输出
  `WORKER_STALE` 和 `next:"sequence-status"`；可选的 `sequence-cancel` 只能出现在说明文字或
  debug/verbose 中。
- 输出契约：所有 stdout 一行 JSON；默认字段只含 `ok/cmd/intent/seq_id/state/active/segments/next/error`；
  错误有 `error.code` 和 `next`；路径、prompt、duration、完整 status 只在 debug/verbose 输出。
- `next` 契约：默认 `next` 只能是单个 P0 命令，例如 `sequence-status`、
  `sequence-interrupt`、`standby`、`run-text`；不输出裸 intent，不输出组合 next，
  不泄漏底层非 P0 raw next。
- 兼容旧 CLI：现有 `et1-trk2motion`、`et1-nl2trk`、`find_preset.py` 调用参数不被破坏。

测试方法：

- mock `et1-trk2motion`、`et1-nl2trk`、`find_preset.py` 为本地假 CLI，断言调用顺序和参数。
- 用临时目录模拟 sequence state。
- 用失败注入覆盖生成失败、tracker run 失败、状态文件不存在、取消中追加、已 submitted
  当前段替换、未 ready interrupt、worker stale、裸 intent `next`、组合 next、底层非 P0
  raw next 泄漏等边界。
- 不需要真机、不需要真实 Kimodo、不需要真实 tracker 服务即可覆盖 P0。

P1/P2 可补：

- `serial_id` 顺序生成测试。
- 独立段并发开关测试。
- 后台进程恢复/孤儿状态清理的更完整测试。
- 与安装版 `~/.agents`、`~/.codex` skill 同步的 packaging 检查。

## 10. 分阶段交付

### P0：最小可落地闭环

目标：解决阻塞等待、长序列首段延迟和普通插话。

交付项：

- 新增 `et1-action` workflow CLI 的最小子命令：
  `run-text`、`run-trk`、`sequence-start`、`sequence-status`、
  `sequence-cancel`、`sequence-append`、`sequence-replace-tail`、`sequence-interrupt`、
  `standby`、`urgent-stop`、`idle-load`。
- `sequence-start` 默认异步返回 `seq_id`。
- 第一段生成完成立即执行，后续段串行生成。
- wrapper 本地维护 sequence state；P0 只提交当前段，不向 tracker 提交任何未来段。
- 下一段可以生成/stage/ready，但必须留在 wrapper 本地 state，等当前段结束后再提交。
- `replace-tail` 只替换本地未 submitted 段，不承诺删除 tracker queue 中的 run。
- `sequence-interrupt` 是明确打断入口：ready/staged 目标可立即 `run --mode interrupt`；
  未 ready 目标先 `standby`，后台生成完成后再普通执行。
- sequence state 有 heartbeat/stale/failed 定义。
- 普通“停下”文档和 wrapper 映射为 `cancel_to_standby`/`standby`，保留 idle。
- urgent stop 必须显式命令或明确紧急意图。
- 一行 compact JSON 输出合同和错误 `next`；默认不输出路径、prompt、duration、完整 status；
  `next` 只能是单个 P0 命令。多个可选动作只进说明文字或 debug/verbose。
- mock CLI 单元测试覆盖 P0 流程。
- SKILL.md 缩短，详细规则移入 references。

P0 暂不做：

- 并发生成。
- 常驻 daemon。
- tracker 服务端接口变更。
- botified fast direct path。
- 复杂优先级和动作数据库。
- 自动 follow `ready.next`。
- 自动 `passive -> fixstand -> standby` 恢复。
- 把 `fixstand`、`passive` 放进普通 hot path。

### P1：连续性和恢复增强

目标：让长序列更稳，但仍保持 wrapper 级别。

交付项：

- `serial_id` 连续段严格顺序生成和状态记录。
- 后台 worker 异常退出后的状态恢复/标记失败。
- 更完整的 references：sequence workflow、intent mapping、output contract。
- `et1-nl2trk-preset` 安装/同步到 `~/.agents` 的 packaging 检查。

### P2：可选性能优化

目标：只在 P0/P1 使用证明需要时再做。

交付项：

- 独立段可选并发生成，默认仍关闭。
- 更好的 sequence 清理策略，例如过期状态文件清理。
- 人类调试用 `sequence-status --watch`。
- 更完整的 metrics/debug 输出，但默认 stdout 仍保持一行 compact JSON。

P2 仍不做：

- tracker playlist API。
- 上传/远程 URL。
- 云端任务系统。
- UI。

## 11. 验收标准

P0 完成时应能证明：

- agent 发起三段自然语言动作时，首段生成完成后能立即执行，并返回 `seq_id`。
- sequence 执行中用户普通说“停下”，workflow 执行 cancel + standby，不调用 urgent stop。
- sequence 执行中用户说“后面加一个挥手”，workflow append 到尾部。
- sequence 执行中用户说“后面的换成鞠躬”，workflow replace_tail 只替换未 submitted 尾部，不影响当前段。
- sequence 不会一次性把所有段提交进 tracker queue，也不会向 tracker 提交任何未来段；tracker 中只有当前段。
- sequence 执行中用户明确说“立刻改做...”，`sequence-interrupt` 对 ready/staged 目标使用
  `run --mode interrupt`，对未 ready 目标先 standby、后台生成、生成后普通执行。
- worker 死亡或 heartbeat 超时时，`sequence-status` 能输出 failed/stale 和明确错误 code。
- 所有默认 `next` 都是单个 P0 命令；不输出裸 intent、不输出组合 next、不泄漏底层非 P0 raw next。
- 所有命令 stdout 都是一行 compact JSON。
- mock 测试覆盖成功、失败、取消和插话主路径。
- 未修改 tracker 服务端 HTTP 接口。
