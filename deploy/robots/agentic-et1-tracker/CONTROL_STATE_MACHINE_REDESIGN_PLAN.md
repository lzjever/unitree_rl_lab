# agentic-et1-tracker 控制状态机/API 收口计划

更新日期：2026-07-03

## 1. 背景与目标

`agentic-et1-tracker` 尚未 GA。近期仿真失败已经证明：在 active user motion 且 idle enabled 时，使用当前公开普通停止 `/standby_velocity` 或紧急停止 `/stop` 中断，会进入 generic stopping/hold 路径并摔倒；无 idle 时 `/standby_velocity` 稳定。这说明当前 API 命名和内部状态语义把“普通 standby handoff”和“generic stop bookkeeping”混在一起，调用方无法稳定表达产品意图，状态也可能在物理交接尚未完成前提前清理。

本计划的目标是在一轮开发内收紧控制状态机和公开 API：

- 普通停止/待命公开主接口改为 `POST /standby`。
- 紧急停止公开主接口改为 `POST /urgent_stop`。
- 普通 `standby` 保留 idle config；`urgent_stop` 清 idle config。
- 普通 `standby` 不得把 generic `stopping` 或 `exec.state:"stopping"` 暴露为成功路径。
- 状态清理必须跟随明确的物理交接完成条件；不能把“内部变量已清空”当成“机器人已经交接到 standby”。

## 2. 设计原则

KISS：

- 一个产品意图只对应一个公开接口：普通待命是 `/standby`，紧急停止是 `/urgent_stop`。
- 不在普通 stop、standby、idle、urgent_stop 之间引入可选模式矩阵。
- 状态字段只暴露 agent 决策必要信息：当前 active owner、用户执行状态、idle 状态、transition 状态、控制边界、错误和下一步。

DRY：

- 一个物理交接只保留一种实现：user/idle/loco_upper 到 standby 都走同一套 standby handoff/transition 语义，不能复制出 `/standby_velocity`、`/stop`、hold fallback 三套相似但不一致的路径。
- API route、skill CLI、README、manual gate、测试中的命名必须共享同一合同，不能让 skill 继续包装旧 HTTP 名称。
- 状态序列化只从同一份 control state snapshot 生成，避免 `ctrl`、`active`、`exec` 各自讲不同故事。

YAGNI：

- 不引入新调度系统，不做 playlist、数据库、UI、云服务、远程 URL、上传或非 `.trk` 格式。
- 不把本次收口扩展成长期多阶段迁移；一轮内完成 API、状态机、测试、文档和 skill 更新。
- 不把 `passive`、`fixstand` 放进普通 hot path，也不做自动 `passive -> fixstand -> standby` 恢复链。

## 3. 产品语义边界

### standby

`standby` 是普通“停下/待命”的产品入口。它是可恢复、非紧急、保留 idle config 的控制意图。明确“不要动/站着别动/completely still/no idle”属于清 idle 后 standby，skill 层使用 `idle-clear`，底层顺序是 `POST /idle {"paths":[]}` then `POST /standby`。

- 接口：`POST /standby`，空 body。
- 可中断 active user、active idle、active transition 和 queued user work。
- 保留 idle pool/config；回到可播放 idle 的 standby 链路后，idle manager 可以按既有规则重新启动 idle。
- 不创建用户 run id，不进入 user queue/history。
- 成功标准是完成 standby handoff/transition，或已经确认处于 ready standby；不能仅因为 active user 已被标记 canceled 就返回成功。
- active user + idle enabled -> standby 必须走 standby handoff/transition，或进入明确 fail-safe；不得先 generic hold/stopping 再假装已经 standby。

### urgent_stop

`urgent_stop` 是紧急/立即停止入口，只用于用户明确表达 emergency stop、abort、kill、紧急停止、赶快停止等意图。

- 接口：`POST /urgent_stop`，空 body。
- 立即 abort active user、idle、holding、transition 和 queued work。
- 清 idle pool/config。
- 不播放 standby reference，不承诺平滑交接。
- 可以暴露 `ctrl:"urgent_stopping"` 或等价紧急停止控制边界；不得伪装成普通 standby 成功。
- 结束落点必须明确：回到 safe standby、进入 passive/fault，或返回需人工处理的 `manual`/`fault` 错误。

### passive

`passive` 是 safety sink，不是普通停止。

- password-gated。
- 停止 active work，清 user queue，清 idle pool/config。
- 不自动恢复 LowCmd，不自动进入 fixstand 或 standby。
- 只能由 operator/显式授权调用；agent 不得因为 status `next` 字段自动执行 passive。

### fixstand

`fixstand` 是姿态恢复/固定构型入口，不是普通静止站立。

- 不清 idle config，但 fixstand 期间不得自动播放 idle。
- user `.trk` 不应从 fixstand 直接启动；需要显式 `/standby` 回到可执行/可 idle 的 standby 链路。
- `bad_orientation` 等恢复场景可以显式使用 fixstand，但不能成为普通 standby fallback。

### idle

`idle` 是 background config，不是用户 run。

- `POST /idle {"paths":[...]}` 原子替换 idle pool；`{"paths":[]}` 清空。
- idle 没有 run id，不进入 `exec`、`queue` 或 user history。
- idle playback 可以被 user `/execute` 或 `/execute_loco_upper` 抢占。
- `/standby` 保留 idle config；`/urgent_stop` 和 `/passive` 清 idle config。

### execute queue / interrupt

`/execute` 和 `/execute_loco_upper` 只表达用户动作。

- `mode:"queue"` 追加用户 run。
- `mode:"interrupt"` 表示新用户意图抢占当前用户/idle/background owner。
- `mode:"interrupt"` 不新增 smooth/stop profile 参数。active running GeneralTracker user 收到 GeneralTracker interrupt 时，runtime 内部优先尝试 current-frame synthetic handoff 到新 user；benign reject fallback controlled stop/restart。Preparing、LocoUpper、safety/fault 路径保持原合同。
- `/execute` accepted/submitted 只表示 run 已被接受/提交，不表示动作完成；完成、holding、fault/passive 等进度必须通过 `/status?id=<run_id>` 或 full `/status` 确认。
- queue/interrupt 不能作为普通停止或紧急停止的隐式替代。
- `exec` 和 `queue` 只描述用户 run；idle 和 transition 必须分别在 `idle`、`transition` 中表达。

### loco_upper

`loco_upper` 是 `/execute_loco_upper` 的 executor 变体，不是独立产品状态机。

- active owner、queue、standby、urgent_stop、passive、fixstand 的公开语义与普通 `/execute` 一致。
- loco_upper runtime 可以有内部 `entry/motion/holding/exit` phase，但外部状态仍通过 `active.kind`、`exec`、`transition` 和 `ctrl` 表达。
- `/standby` 中断 loco_upper 时必须使用同一 standby handoff/transition 合同，必要时走 loco_upper 专用的 bounded handoff 实现，但不能暴露另一套产品状态。
- `/urgent_stop` 中断 loco_upper 时清理 executor state 和 idle config；如果物理停止失败，状态必须进入 fault/passive/manual，而不是显示普通 standby 成功。

## 4. API 变更建议

### 公开接口

本轮完成后，公开 HTTP 合同以以下命名为准：

| 意图 | 新接口 | idle config | 是否用户 run | 说明 |
| --- | --- | --- | --- | --- |
| 普通待命 | `POST /standby` | 保留 | 否 | 普通“停下/待命”。 |
| 完全静止/no idle | `POST /idle {"paths":[]}` then `POST /standby` | 清空 | 否 | “不要动/站着别动/completely still/no idle”；不使用 urgent stop。 |
| 紧急停止 | `POST /urgent_stop` | 清空 | 否 | emergency/abort/kill 语义。 |
| 安全 sink | `POST /passive` | 清空 | 否 | password-gated，不自动恢复。 |
| 姿态恢复 | `POST /fixstand` | 保留但禁播 | 否 | 显式恢复入口，不是普通 standby。 |
| idle 配置 | `POST /idle` | 设置/清空 | 否 | 配置 background idle pool。 |
| 用户动作 | `POST /execute` | 不改变 | 是 | GeneralTracker 用户动作。 |
| loco_upper 用户动作 | `POST /execute_loco_upper` | 不改变 | 是 | loco-upper executor 用户动作。 |

### 旧 alias 处理

明确建议：由于项目未 GA，不在最终 release/packaged skill/README 中保留 `/standby_velocity` 或 `/stop` 作为成功 alias。

- `/standby_velocity`：从公开文档和 skill 中移除。若实现层需要短期兼容内部脚本，本轮结束前也必须改完这些脚本；最终 API 不保留成功 alias。
- `/stop`：从公开文档和 skill 中移除。不要把它保留为 `/urgent_stop` 的成功 alias，因为名称过泛，容易再次被普通“停下”误用。
- 如果开发期间临时留下旧 route，只允许返回结构化错误，例如 `CONTROL_ROUTE_RENAMED`，`next:"standby"` 或 `next:"urgent_stop"`；不得执行控制动作，不得进入 release artifact 验收。
- `next` token 同步改为 `standby`、`urgent_stop`、`fixstand`、`passive`、`manual`、`status`、`retry`、`wait_robot`、`fix` 等新命名；不再返回 `standby_velocity` 或 `stop`。

### 文档和 skill

- README、ACCEPTANCE、manual gate 文档、skill references、skill output-contract、intent-mapping 统一改成 `/standby` 和 `/urgent_stop`。
- high-level skill 命令可以继续叫 `standby` 和 `urgent-stop --urgent`，但底层 HTTP 必须分别映射到 `/standby` 和 `/urgent_stop`。
- skill 必须明确：普通“停止/待命”走 `standby`；明确“不要动/站着别动/completely still/no idle”走 `idle-clear`；只有 emergency/abort/kill/紧急措辞走 `urgent-stop --urgent`。
- 搜索并消除 release 路径中的旧 token：`/standby_velocity`、`standby_velocity`、`/stop`、`next:"stop"`。测试 fixture 若需要旧路由，只能用于验证“旧路由被拒绝/提示 renamed”。

## 5. 内部状态机模型

### 外部可见状态

建议保留并收紧现有状态分层：

- `active.kind:"none"`：没有 user/idle/transition owner。
- `active.kind:"user"`：唯一 waitable active，`active.id` 为用户 run id。
- `active.kind:"idle"`：background idle active，`id:null`。
- `active.kind:"transition"`：内部物理交接 active，`id:null`。

`exec` 只描述用户 run，允许状态建议为：

- `queued`
- `preparing`
- `running`
- `holding`
- `done`
- `canceled`
- `failed`

普通 `/standby` 成功路径禁止使用 `exec.state:"stopping"`。如果正在执行物理交接，应通过 `active.kind:"transition"` 和 `transition.target:"standby"` 表达；如果交接失败，应返回 `failed/canceled` 和 top-level `err`，而不是把 stopping 当成成功中间态。

`ctrl` 建议收口为产品边界：

- `starting`
- `standby`
- `running`
- `transition`
- `urgent_stopping`
- `passive`
- `fixstand`
- `fault`
- `manual`

其中 `standby_velocity` 只可作为内部实现名或 debug detail，不作为公开 `ctrl` 合同。

### 允许转换

| From | Intent | To | 约束 |
| --- | --- | --- | --- |
| `starting` | runtime ready | `standby` | 完成 LowCmd/MotionSwitcher/asset gate 后才 ready。 |
| `standby` | `/execute` queue/interrupt | `user` 或 `transition -> user` | 可从 standby reference handoff 到用户首帧。 |
| `standby` | idle auto-play | `idle` 或 `transition -> idle` | 仅 idle config 非空、无 user work、ready/safe。 |
| `user` | user queue | `user` 后续 FIFO | 当前 user 完成/holding/canceled 后启动下一 run。 |
| `user` | user interrupt | `transition -> user` 或 fallback stop/restart | running GeneralTracker user 优先 smooth handoff；Preparing/LocoUpper 或 benign transition reject 保持 controlled stop/restart。 |
| `user` | `/standby` | `transition -> standby` | 必须走 standby handoff；失败走 fail-safe。 |
| `idle` | `/execute` | `transition -> user` | 停止当前 idle playback，保留 idle config。 |
| `idle` | `/standby` | `transition -> standby` 或 `standby` | 停止当前 idle playback，保留 idle config。 |
| `transition` | `/standby` | `transition -> standby` | abort 旧 target，重建或复用 standby handoff。 |
| `any safe active` | `/urgent_stop` | `urgent_stopping -> standby/passive/fault/manual` | 清 idle config，不承诺平滑。 |
| `standby` | `/fixstand` | `fixstand` | 显式姿态恢复；idle config 保留但禁播。 |
| `fixstand` | `/standby` | `transition -> standby` 或 `standby` | 显式回到可执行/可 idle standby。 |
| `passive` | `/fixstand` | `fixstand` | 仅显式授权/ready 条件满足；不自动恢复。 |
| `fault/manual` | operator recovery | `fixstand` 或 `standby` | 取决于错误类型和 readiness gate。 |

### 禁止转换

- `user/idle/transition -> generic stopping -> standby` 作为普通 `/standby` 成功路径。
- `user/idle/transition -> clear active -> standby`，但物理 LowCmd/reference 尚未交接完成。
- `passive/fault/manual -> /execute` 直接启动用户动作。
- `fixstand -> idle auto-play`，除非先显式 `/standby`。
- `standby -> passive -> fixstand -> standby` 自动恢复链。
- `/standby` 清 idle config。
- `/urgent_stop` 保留 idle config。
- `loco_upper` 建立独立公开状态机或绕过统一 standby/urgent_stop 语义。

### standby handoff 成功条件

`/standby` 返回成功应满足以下之一：

- 已经处于 `ctrl:"standby"`，`active.kind:"none"`，且 ready/safe。
- 已成功启动 `transition.target:"standby"`，响应明确表示 `accepted:true`、`confirmed:false`，由后续 status 确认最终 `standby`。
- 同步实现选择等待时，必须等到 transition 完成并确认 `ctrl:"standby"` 后才返回 confirmed success。

不得在 active user 被标记 canceled、exec 被清空、reference 被置空、或进入 generic hold 后立即声明 `standby` 成功。

### 失败 fallback

active user + idle enabled -> `/standby` 的失败必须显式且安全：

- standby handoff 构建失败：不返回 standby success；保留或标记取消 user 的规则必须与物理控制实际一致；top-level `err` 应说明 `STANDBY_HANDOFF_FAILED` 或等价错误，`next` 为 `urgent_stop`、`manual` 或 `status`。
- standby reference/asset 缺失：API readiness gate 失败，不启动 generic hold fallback。
- LowCmd/write/readiness 失败：进入 `passive`、`fault` 或 `manual`，并带明确错误；不得显示 `ctrl:"standby"`。
- 物理交接超时：进入 fail-safe 路径，优先保护机器人；如果升级为 urgent stop，必须清 idle config 并报告 `stop_reason:"urgent_stop"` 或等价字段。
- idle restart 只能发生在最终 `standby` ready/safe 之后；不能在 handoff 中途抢回 active owner。

## 6. TDD 测试清单

先补失败测试，再改实现。最少测试覆盖如下：

### API route 和合同

- `POST /standby` 空 body accepted；非空 body rejected。
- `POST /urgent_stop` 空 body accepted；非空 body rejected。
- `/standby` 保留 idle config；`/urgent_stop` 清 idle config。
- `/standby_velocity` 和 `/stop` 在 release 合同中不成功执行；若 route 存在，只返回 renamed 错误和新 `next` token。
- error envelope 和 `next` token 不再出现 `standby_velocity` 或 `stop`。

### 状态序列化

- 普通 standby 过程中不出现 `exec.state:"stopping"` 作为成功路径。
- 物理交接中使用 `active.kind:"transition"`、`transition.target:"standby"`。
- 只有用户 run 出现在 `exec`/`queue`；idle 和 standby transition 不写 user history。
- active cleanup 不早于 transition completion；测试需断言中途 status 仍显示 transition 或明确未 confirmed。

### standby 主路径

- active user + idle enabled -> `/standby`：进入 standby handoff/transition，最终 `ctrl:"standby"` 或 idle 按规则重启；不摔、不 generic hold。
- active user + idle disabled -> `/standby`：稳定进入 standby。
- active idle -> `/standby`：停止当前 idle playback，保留 idle config，最终回 standby 后可按规则重新 idle。
- active transition target=user/idle/standby -> `/standby`：abort 旧 target，进入统一 standby handoff；不继续旧 smoothing。
- user holding -> `/standby`：允许 held reference -> standby reference 的明确 handoff，但仍不得暴露 generic stopping 成功。

### urgent_stop 主路径

- active user/idle/holding/transition -> `/urgent_stop`：立即 abort，清 queue 和 idle config，不播放 standby reference。
- idle FixStand + idle config -> `/urgent_stop`：清 idle/queue/active，runtime 消费后保持 FixStand；public `urgent_stopping` 只允许作为短暂 latch。
- loco_upper active/holding/transition -> `/urgent_stop`：清 executor state，清 idle config；失败进入 fault/passive/manual。
- repeated `/urgent_stop` 幂等，不能恢复 idle 或启动 user work。

### 控制边界

- passive/fault/manual 下 `/execute` 拒绝，readiness/manual error 优先于 controller conflict。
- fixstand 下不自动 idle；显式 `/standby` 后才允许 idle auto-play。
- `/passive` 清 idle config；`/fixstand` 保留 idle config 但禁播。

### fail-safe

- standby asset 缺失或 handoff builder 失败时，`/standby` 不返回 success，不暴露 `ctrl:"standby"`。
- standby handoff 超时进入明确 fail-safe；如果升级为 urgent stop，则 idle config 被清空。
- LowCmd/write failure 不被吞掉；status 进入 passive/fault/manual 并给出可操作 `next`。

### docs/skill 测试

- release README、skill references、intent mapping、output-contract 中不再出现旧 public token。
- skill `standby` 调用 `/standby`。
- skill `urgent-stop --urgent` 调用 `/urgent_stop`。
- 普通中文“停下/待命”映射 standby；明确“不要动/站着别动/no idle”映射 `idle-clear`；“紧急停止/abort/kill”才映射 urgent-stop。

## 7. manual / e2e / visual gate

### 仿真复现链路

本轮必须把已知失败做成可重复验收链路：

1. 启动 MuJoCo + tracker sim，使用 app-owned config 和 release assets。
2. 配置 idle pool：`POST /idle {"paths":[...idle.trk...]}`。
3. 启动一个 active user motion：`POST /execute` 或 `POST /execute_loco_upper`。
4. 在 user motion 稳定运行且 idle enabled 时调用新 `POST /standby`。
5. 验证 status 经过 `transition.target:"standby"` 或明确 accepted/confirmed 模型，最终到 `ctrl:"standby"` 或按规则 idle active；视觉上不摔。
6. 单独重复 active user + idle enabled -> `POST /urgent_stop`，验证清 idle config，并确认状态落点不是普通 standby 假成功。
7. 对照 active user + idle disabled -> `/standby`，确认保持稳定。

### e2e gate

建议更新并运行：

```sh
tools/manual_gate.py e2e --url http://127.0.0.1:8083 \
  --motion-dir /absolute/dir/listed/in/motion_dirs
```

新增 e2e 断言：

- `/standby` 和 `/urgent_stop` 新路由可用。
- idle enabled + active user -> `/standby` 不出现 old generic stopping success。
- `/urgent_stop` 清 idle config。
- status/error/next 不输出旧 token。

### visual gate

建议更新并运行：

```sh
tools/manual_gate.py visual --url http://127.0.0.1:8083 \
  --motion-dir /absolute/dir/listed/in/motion_dirs
```

visual checklist 至少记录：

- active user + idle enabled -> `/standby` 全过程机器人保持站立。
- `/standby` handoff 期间没有突然 reference 清空或姿态塌陷。
- idle restart 只发生在 standby ready/safe 之后。
- `/urgent_stop` 的视觉表现与普通 standby 区分清楚；它是紧急停止，不是平滑待命。

### full local simulation acceptance

使用现有 full local sim 入口时，保留临时 config 降低 `hz` 的做法，使 transition/urgent_stopping HTTP 窗口可观察：

```sh
tools/manual_gate.py e2e --url http://127.0.0.1:8083 \
  --motion-dir /absolute/dir/listed/in/motion_dirs \
  --start-tracker --enable-loco-temp --require-loco
```

这条链路必须覆盖 `/execute_loco_upper` active 时的 `/standby` 和 `/urgent_stop`，避免只验证 GeneralTracker。

## 8. 一轮完成实施步骤

本工作不拆成长期多阶段；同一轮 PR/变更包完成以下事项：

1. 添加 TDD 失败测试：新 route、旧 route renamed/rejected、standby handoff 状态、idle preserve/clear、active user + idle enabled 复现、loco_upper 中断、docs/skill token 扫描。
2. 收口 route 和 error token：新增 `/standby`、`/urgent_stop`，移除 release 成功 alias，统一 `next` token。
3. 收口状态模型：公开 `ctrl` 使用 `standby`、`urgent_stopping` 等产品边界；普通 standby 使用 `transition.target:"standby"`，不暴露 `exec.state:"stopping"` 成功路径。
4. 实现统一 standby handoff：active user、active idle、active transition、loco_upper 都进入同一产品语义；物理交接完成前不提前清 active/confirmed。
5. 实现 urgent_stop 语义：立即 abort，清 queue 和 idle config，loco_upper executor 同步清理；失败落入 fault/passive/manual。
6. 更新 skill 和文档：README、ACCEPTANCE、manual gate、packaging skills、output contract、intent mapping 全部使用新命名。
7. 运行测试：unit/runtime/http/skill tests、release selftest、manual e2e、visual gate、full local sim loco_upper gate。
8. 记录验收结果：把失败复现、修复后 status 截图/JSON、visual gate artifact 路径和剩余风险写入 acceptance 记录。

## 9. 交付判定

完成条件：

- 公开 API 只有 `/standby` 表达普通待命，只有 `/urgent_stop` 表达紧急停止。
- 普通 standby 保留 idle config，urgent_stop 清 idle config。
- active user + idle enabled -> standby 不再通过 generic stopping/hold 假成功。
- 状态清理与物理交接完成条件绑定；中途状态可观察、可解释。
- skill 和文档不再引导用户或 agent 调用 `/standby_velocity` 或 `/stop`。
- MuJoCo e2e/visual gate 覆盖已知失败链路并通过。

非目标：

- 不新增调度系统。
- 不新增 motion playlist 或多格式输入。
- 不做自动 passive/fixstand 恢复链。
- 不改变 `.trk` allowlist、安全资产归属、loco_upper bounded execution 的核心产品范围。
