# agentic-et1-tracker 后续开发计划

更新日期：2026-06-03

## 1. 目标与范围快照

`agentic-et1-tracker` 的产品定位是只给 LLM agent 调用的本地 HTTP/CLI 服务。核心职责只保留三类：

- 执行本地绝对路径 `.trk`。
- 提供基础机体控制：`/passive`、`/fixstand`、`/standby_velocity`、`/stop`。
- 暴露紧凑、可轮询、适合 agent 判断下一步的状态查询。

必须同时支持 MuJoCo 模拟器和 ET1 真机。运行时资产必须使用本 app 下的 `config` release assets，不改 ET1 app，不 fallback 到 `deploy/robots/et1` app tree。

当前不能声明 GA 完成。以下 gate 仍未完成：

- broader MuJoCo/operator acceptance。targeted standby_ref simulator asset
  accepted 已记录，但完整 MuJoCo 操作矩阵仍 pending。
- ET1 real robot acceptance。

已完成的软件证据：`standby_ref.trk` asset 与 manifest 已记录，targeted
simulator asset accepted，standby_ref runtime gate 已接入，并有
unit/runtime/release selftest 覆盖。不要据此宣称整体 GA。

## 2. KISS / DRY / YAGNI

后续开发必须把 KISS、DRY、YAGNI 当作交付约束，而不是口号：

- KISS：HTTP schema 保持小而稳定；状态字段只保留 agent 决策必要信息；错误只给一个明确 `next`。
- DRY：`.trk` path 校验、状态序列化、skill CLI 输出合同、release selftest 不得各自复制一套不一致规则。
- YAGNI：不做当前 agent 执行闭环不需要的能力。

明确禁止范围蔓延：

- 不设计上传、远程 URL、非 `.trk` 多格式、内嵌 motion payload。
- 不设计 playlist、权重调度、motion catalog、UI、数据库或云服务。
- 不把 `standby_ref.trk` 放进用户 motion allowlist、idle pool 或 `/execute` 用户路径。
- 不引入 ET1 app tree fallback；缺 asset 就显式 gate/pending/fail。

## 3. 当前已实现 / 当前 public API 合同

### HTTP/CLI surface

- `GET /health`：服务健康。
- `GET /status`：完整但紧凑的 runtime state。
- `GET /status?id=<id>`：只查询用户 run id。
- `POST /execute`：只接受 `{"path":"/absolute/file.trk","mode":"queue|interrupt","hold":true|false}`；`mode` 可省略，默认 `queue`；`hold` 可省略，等价 false；`hold` 必须是 boolean；拒绝 `paths` 和任何额外字段。
- `POST /idle`：`{"paths":["/abs/a.trk"]}` 原子覆盖 idle pool；`{"paths":[]}` 清空；这是配置接口，不产生用户 run id。
- `POST /stop`：空 body；abort user / idle / holding / transition；清 idle config；不播放 `standby_ref.trk`。
- `POST /passive`：空 body；进入 Passive safety sink；停止 active work，清 user queue，清 idle pool/config；不播放 `standby_ref.trk`；不得自动恢复 `lowcmd_occupied`。
- `POST /fixstand`：空 body；进入 FixStand，是 LowCmd 未被占用时的姿态恢复入口；不播放 `standby_ref.trk`。
- `POST /standby_velocity`：空 body；进入 StandbyVelocity/Velocity0，是正常可执行 `.trk` 和 idle auto-play 的待命状态；不播放 `standby_ref.trk`。

skill CLI 必须保持 one-line compact JSON；`holding` 对 `run --hold --wait` 和 `wait` 是成功状态。

### 状态边界

`active.kind` 是外部状态权威，只能是：

- `none`：无 user/idle/transition active。
- `user`：唯一 waitable active，`active.id` 为用户 run id。
- `idle`：内部 idle active，`id:null`，不进入用户 queue/history。
- `transition`：内部 synthetic transition，`id:null`，不进入用户 queue/history，不消耗 queue limit。

`exec` 和 `queue` 只描述用户 run。idle 进度只在 `idle` 对象中。transition 进度只在 `transition` 对象中。

已实现合同必须稳定：

- `/execute` optional boolean `hold`。
- `MotionState::Holding`，JSON 字符串 `holding`。
- `ActiveKind::Transition`，JSON 字符串 `transition`。
- transition status：`active/target/target_id/target_state/frame/frames/progress`。
- hold-last：`hold:true` 的用户 run 到最后一帧后继续发布末帧 reference，并保持原 run id queryable。
- user-to-user synthetic transition。
- user-to-idle synthetic transition。
- `/idle` 配置接口。
- `/stop` 可 abort user、idle、holding、transition，并且不播放 `standby_ref.trk`。
- `/passive` 可 abort user、idle、holding、transition，清 user queue 和 idle pool/config，并停在 Passive safety sink；与 `/stop` 的区别是落点固定为 passive safety sink，不保留恢复到 standby 的语义；与 `/fixstand` 的区别是 passive 是 safety sink，fixstand 是姿态恢复；与 `/standby_velocity` 的区别是 passive 不进入可执行/idle 待命状态。

`standby_ref` asset 已记录且 targeted simulator asset accepted；runtime gate 已接入，
并由 unit/runtime/release selftest 覆盖。broader MuJoCo/operator 与 real
robot GA gates 仍 pending；只有这些外部 gate 完成后，才能声明整体 GA。

## 4. 后续 gate / 最少工作包

### WP1 合同稳定化

目标：冻结 agent 可依赖的 HTTP/CLI/status 合同，避免后续小改破坏 agent。

交付项：

- 保持 `/execute` schema 严格：只允许 `path/mode/hold`。
- 保持 `/idle` 仅配置，不提交 run。
- 保持 `active.kind`、`exec`、`queue`、`idle`、`transition` 边界。
- 保持 `/stop` abort holding/transition 且不触发 standby_ref。
- 保持 `/passive` 清 active/queue/idle 并进入 Passive safety sink，且不触发 standby_ref 或 reclaim LowCmd。
- 同步 README、skill references、skill CLI tests，只记录合同，不扩展产品能力。

### WP2 MuJoCo visual acceptance

目标：用当前 HEAD 的真实 runtime 行为做模拟器验收，不复用旧语义证据。

验收操作顺序修正：如果 `/status` 已经是 `ready:true`
`ctrl:"standby_velocity"`，smoke/普通验收不要先发 `/passive`。这只是验收
操作顺序修正，不是代码语义变更，也不新增 API。`/passive` 只在专门验证
Passive safety sink 时发送，且需提前准备 MuJoCo reset/upright/operator
支撑；`bad_orientation` 恢复应先 `/fixstand`，等待 `ready:true`
`ctrl:"fixstand"`、`block:null`、`err:null` 后再 `/standby_velocity`。

最少场景：

- sim config 使用 app-owned assets，`mode_machine:0`，不调用 MotionSwitcher。
- 启动到可控状态，验证 `/status.pose`、`ctrl`、`ready/block/err`。
- `/execute` 普通完成、`hold:true` 末帧 holding、user-to-user transition、user-to-idle transition。
- `/idle` set/clear、idle active 被用户 `/execute` 抢占。
- `/stop` 在 user/idle/holding/transition 中立即 abort，不播放 standby_ref。
- 专门 passive safety-sink 场景：`/passive` 在 user/idle/holding/transition 中立即进入 Passive safety sink，清 user queue 和 idle pool/config，不自动恢复 `lowcmd_occupied`。
- `bad_orientation -> passive`，恢复路径为 `/fixstand` 到 `ready:true ctrl:"fixstand" block:null err:null` 后再 `/standby_velocity`。
- `lowcmd_occupied -> manual`，不自动恢复。

证据必须写入 `ACCEPTANCE.md` 或等价验收记录，包含日期、commit、config、命令摘要、关键 `/status` 样例和观察结论。

### WP3 real robot acceptance

目标：在 ET1 真机/operator 窗口验证与 MuJoCo 一致的安全和合同边界。

最少场景：

- real config 使用 app-owned assets，`mode_machine:1` 下 startup MotionSwitcher release 行为符合预期。
- LowCmd owner preflight 能阻止外部占用下的 writing runtime。
- `/fixstand`、`/standby_velocity`、普通 `.trk`、`hold:true`、`/stop` 人工可控。
- holding/transition 中 operator 可随时停止。
- `bad_orientation` 和 `lowcmd_occupied` 不被 `/stop`、idle 或 transition 绕过。

真机验收必须由 operator 明确记录通过/失败和风险备注。失败项不得用 MuJoCo 证据替代。

### WP4 standby_ref runtime playback gate

状态：standby_ref runtime gate 已接入，并由 unit/runtime/release selftest 覆盖。
targeted standby_ref simulator asset accepted 已记录。broader MuJoCo/operator 与
real robot GA gates 仍 pending；不要声明整体 GA。

交付条件：

- [x] 保持已固化的 `config/reference/standby/v0/standby_ref.trk` 和
  `config/reference/standby/v0/ASSET_MANIFEST.yaml`，记录 source、sha256、
  frames、fps、duration、simulator acceptance 结论。
- [x] runtime 接入 standby_ref playback，并有 unit/runtime 覆盖。
- [ ] broader MuJoCo/operator 审核通过。
- [ ] real robot/operator 审核通过。
- [x] candidate 工具输出的 `CANDIDATE_MANIFEST.json` 只记录候选生成证据，不是 release manifest，不进入 runtime release 约定。
- [x] release package selftest 检查 `standby_ref.trk` 和 `ASSET_MANIFEST.yaml`。
- [x] runtime 只读取 app-local release asset；缺失或损坏不得 fallback 到 ET1 app tree。

仍需保持：

- `/stop` 不播放 standby_ref。
- `/passive`、`/fixstand` 不播放 standby_ref。
- `passive/fault/lowcmd_occupied` 不自动启动 standby_ref。
- standby_ref 播放完成后仍进入 `standby_velocity`，不替代 Velocity0 policy。

### WP5 release / skill 验证

目标：保证 agent 使用方式和发布包一致。

交付项：

- [x] packaged skill tests 通过。
- [x] 安装到本地 agent/codex skill 位置后的 diff 或版本证据通过。
- CLI `run --hold --wait`、`status`、`raw` 输出仍为 compact one-line JSON。
- aarch64 release package 从当前 `HEAD` 克隆构建，包含 app-owned assets、skill、scripts、selftest。
- 明确记录：未提交 tracked changes、untracked files、本地 config edits 不会进入 release workspace。

## 5. TDD 要求

每个代码改动必须先有针对性小测试，再改实现。测试应靠近风险点：

- API schema 改动：优先 `api_tests` / `http_server_tests`，断言 request shape、错误码、无副作用。
- runtime 状态改动：优先 `runtime_control_loop_tests` / `runtime_bridge_tests`，断言 active.kind、exec、queue、idle、transition。
- path/asset 改动：优先 loader/validator/config 小测试，避免把真实文件系统规则复制到 API 测试。
- skill 改动：优先 skill CLI 单元测试，断言 one-line JSON 和 exit code。
- release 改动：优先脚本 selftest 或最小 shell-level 验证。

避免大而泛的测试。不要为了一个小状态字段重跑/重写整套端到端矩阵；端到端只用于 gate acceptance。

## 6. 验收 checklist

API schema：

- [x] `/execute` 只接受 `path/mode/hold`；`hold` 非 boolean、`paths`、额外字段均 400 `REQUEST_INVALID`。
- [x] `/idle` set/clear 原子；失败不污染旧配置。
- [x] 空 body 控制接口拒绝非空 body。

Runtime state：

- [x] `active.kind` 仅为 `none/user/idle/transition`。
- [x] `exec/queue` 只描述用户 run。
- [x] `holding` 保持原 run id，`progress:1`，持续发布末帧。
- [x] transition 不进入 queue/history，不产生 run id。
- [x] `/stop` abort user/idle/holding/transition，不播放 standby_ref。
- [x] `/passive` abort active work，清 user queue 和 idle pool/config，进入 Passive safety sink，且不自动恢复 `lowcmd_occupied`。

Skill CLI：

- [x] `run --hold --wait` 在 `holding` 返回 `ok:true`。
- [x] `status` 默认 compact，不输出大 pose。
- [x] raw HTTP fallback 覆盖 `/execute hold`、`/idle`、`/stop`。

MuJoCo：

- [ ] sim config 不调用 MotionSwitcher。
- [ ] smoke/普通验收不从 ready `standby_velocity` 先发 `/passive`；`/passive` 只在专门 safety-sink 场景且准备 reset/upright/operator 支撑后执行。
- [ ] 普通 run、hold-last、user-to-user transition、user-to-idle transition 可视通过。
- [ ] `/stop`、bad_orientation、lowcmd_occupied 行为符合安全合同。

Real robot：

- [ ] startup release/preflight 行为通过。
- [ ] operator 验证 FixStand、StandbyVelocity、run、hold、stop。
- [ ] 安全异常不被自动恢复逻辑绕过。

Release package：

- [x] app-owned assets 完整，无 ET1 app fallback。
- [x] release notes 明确 `standby_ref.trk` asset 已模拟器接受、runtime gate
  已接入、broader MuJoCo/operator 与真机 gate pending，且不声明整体 GA。
- [x] standby release asset selftest 检查
  `config/reference/standby/v0/standby_ref.trk` 和
  `config/reference/standby/v0/ASSET_MANIFEST.yaml`；不把
  `CANDIDATE_MANIFEST.json` 当 release manifest。
- [ ] aarch64 package 从 `HEAD` 构建；所有 release 必要变更已 commit。
- [x] package selftest 和 skill tests 通过。

## 7. 安全约束

- `lowcmd_occupied` 是 manual/operator 状态。任何 `/stop`、`/idle`、用户 `/execute`、readiness 重算或 transition 完成逻辑都不得自动 reclaim LowCmd。
- `bad_orientation` 默认进入 passive/safety path；`/passive` 和 `/fixstand` 是 LowCmd 未被占用时的软件恢复例外。
- `/stop` 是 active work 的最高优先级命令，但不是 safety bypass；它不得播放 `standby_ref.trk`，也不得绕过 bad orientation 或 lowcmd occupied。
- policy inference failure、safety limit、disconnect 必须继续走现有 safety/fault/manual 路径。

## 8. 工作树 / 发布注意

aarch64 release path 会从当前 git commit `HEAD` 准备干净 clone。未提交 tracked changes、untracked files、本地 config edits 都不会进入 release workspace。

发布前必须：

- `git status` 确认所有 release 需要的代码、文档、asset、manifest、skill 变更已 commit。
- 不依赖开发机上的 build-local candidate、临时 `.trk`、本地绝对路径或 ET1 app tree。
- 在 release notes 中明确三个 GA gate 状态：`standby_ref.trk` release asset、MuJoCo visual acceptance、real robot acceptance。
