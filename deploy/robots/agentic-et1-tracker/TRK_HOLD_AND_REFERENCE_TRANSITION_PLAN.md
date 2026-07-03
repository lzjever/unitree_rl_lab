# TRK Hold 与 Reference Transition 后续开发计划

> **Historical/Obsolete API names (2026-07):** This file is a historical
> planning handoff and may still mention old public route names. The GA/current
> API is `POST /standby` for ordinary standby and `POST /urgent_stop` for urgent
> stop. Legacy `POST /standby_velocity` and `POST /stop` are not successful
> aliases; if present they reject with `CONTROL_ROUTE_RENAMED`. Use `README.md`
> and `CONTROL_STATE_MACHINE_REDESIGN_PLAN.md` as the current contract.

本文档面向 `agentic-et1-tracker` 后续开发 handoff。目标是把用户 `.trk`
动作结束后的末帧保持、`.trk` reference 到 `.trk` reference 的平滑内部过渡、
以及内置 standby 过渡 reference asset 收敛到一个小而明确的实现范围。

本计划只描述后续开发，不要求本次修改代码、ET1 app 或
`config.sim.yaml.example`。

## 0. 当前状态同步

- 已实现并进入当前合同：`POST /execute` 的 `hold` 参数、
  `MotionState::Holding`、`ActiveKind::Transition`、`transition` status、
  hold-last、user synthetic transition 和 idle synthetic transition。
- `standby_ref.trk` 已在 2026-06-03 固化为 app-local release asset：
  `config/reference/standby/v0/standby_ref.trk` 与 manifest 已提交，模拟器视觉
  验收已记录；真机/operator gate 仍 pending。
- standby_ref runtime gate 已接入：runtime 会加载 app-local
  `standby_ref.trk`，内部 playback/abort 行为已有 unit/runtime/release
  selftest 覆盖。targeted standby_ref simulator asset accepted 已记录；broader
  MuJoCo/operator 场景和 real robot GA gates 仍 pending。
- direct `/standby` 与 Standby/Velocity0 policy 路径仍然可用；
  standby reference 不替代 Velocity0，也不能作为整体 GA 能力宣称。

## 1. 目标范围

- 扩展 `POST /execute`，新增可选 `hold` 参数。
- 支持用户 `.trk` 动作结束后保持 last frame，供下一段 `.trk` 或后续 gated
  standby 过渡使用。
- 支持从一个 `.trk` reference 到另一个 `.trk` reference 的内部 synthetic
  transition。
- 新增 app-local 内置 `standby_ref.trk` release asset，作为离开 held
  reference 并回到 `standby_velocity`/`Velocity0` 前的过渡 reference。
- 补齐状态、API、测试、验收和发布资产影响说明。

## 2. 设计约束

### KISS

- `/execute` 只新增一个布尔参数：`hold`。
- `hold` 省略时保持现有行为，默认 `false`，避免破坏已有客户端。
- `hold` 是 per-run metadata，不是 `mode`，不得复活或混用现有
  `stop_hold_s`。
- 不提供 transition duration、blend curve、目标姿态、资产选择等 API 选项。
- synthetic transition 使用固定内部策略和固定内部参数，先满足稳定、安全、
  可测试，不做可配置工作流。
- 实现拆成两个收敛切片：
  - A. hold-last 最小闭环：API 透传、run holding、status/wait 语义、控制命令
    退出。
  - B. reference transition + `standby_ref.trk` asset：user-to-user synthetic
    transition、user-to-idle synthetic transition、自然回 standby 的短
    reference、asset packaging。
- 这是开发拆分，不是产品多阶段功能膨胀；A/B 都保持同一个最终 API 面。

### DRY

- 路径校验复用现有 `/execute` validator：绝对路径、`.trk`、存在性、
  `motion_dirs` allowlist、parse 和 schema 校验不重复实现。
- `.trk` 读取复用 `TrkLoader`、`TrkTrack`、`TrkFrameView` 和现有
  GeneralTracker policy runner。
- 状态和错误 envelope 复用现有 `ok/error/next` 合同。
- reference frame 发布复用现有 `ReferenceFrameSnapshot` 链路，只补充必要的
  kind/transition 元数据。

### YAGNI

- 不设计上传、远程 URL、内嵌 payload、多格式输入或批量 `/execute paths`。
- 不设计用户可选 transition asset、队列优先级、复杂调度、外部业务逻辑或 UI。
- 不扩大 `/execute` gate 和 control conflict 的 public controller state
  surface。
- 不在运行时调用 `nl2trk`、Kimodo、motion2trk 或任何在线生成服务。
- `standby_ref.trk` 只通过离线生成、人工/仿真筛选、manifest 固化进入发布包。

## 3. API 合同

### `/execute`

目标 request：

```http
POST /execute
{"path":"/absolute/file.trk","mode":"queue|interrupt","hold":true}
```

规则：

- 允许字段收敛为 `path`、可选 `mode`、可选 `hold`。
- `hold` 必须是 boolean；其他类型返回 400 `REQUEST_INVALID`。
- `hold` 省略时按 `false` 处理。
- 额外字段、`paths`、URL、上传、多格式输入继续拒绝。
- request schema 失败时不得调用 validator、command sink 或 id generator。
- `hold` 是用户 run 的属性，不是 queue 模式，不影响 `mode:"queue"` 和
  `mode:"interrupt"` 的现有语义。
- `hold` 不读取、不覆盖、不复活 `stop_hold_s`；`stop_hold_s` 仍按现有停止
  保持用途处理，不参与 run 结束后的 reference holding。

建议 response 回显：

```json
{"ok":true,"id":"run-id","state":"queued","q":1,"hold":true}
```

### 行为语义

- `hold:false`：用户 `.trk` 播放完成后按现有语义结束用户 run。之后保持现有
  idle 语义：如果 idle pool 已配置，且 runtime ready/safe、无 user active、
  无 user queue/interrupt/pending，则从用户最后一帧到选中 idle `.trk` 第一帧
  插入 internal synthetic transition，然后进入 idle；否则按既有路径回到
  `standby_velocity`。standby reference transition 已受 app-local validated
  standby_ref runtime gate 保护；使用范围仍限于自然回 standby 或显式
  `/standby_velocity`，不表示 broader MuJoCo/operator 或 real robot GA 完成。
- `hold:true`：用户 `.trk` 到达末帧后，run 进入 `holding`，runtime 持续发布
  和执行该 `.trk` 的最后一帧 reference。
- holding 期间 `exec.state:"holding"`、`progress:1`，用户 run id 仍是原
  `/execute` 生成的 id，并且仍可通过 `GET /status?id=<id>` 查询；holding
  不是新 run，也不是到末帧立刻 `done` 后继续 hold。
- holding 期间接受下一次用户 `/execute`：
  - `queue`：新 run 正常进入用户 queue；runtime 从 held last frame 到新 run
    第一帧插入内部 synthetic transition 后启动新 run。
  - `interrupt`：停止当前 holding run，取消符合现有 interrupt 规则的等待项，
    从 held last frame 到 interrupt 目标第一帧插入内部 synthetic transition。
- `/stop`、`/passive`、`/fixstand`、`/standby_velocity` 可以结束 holding。
  `/stop` 是立即终止语义，不播放 `standby_ref.trk`；自然完成后无新 user
  work 或显式 `/standby_velocity` 回 standby 才允许使用已 runtime-gated 的
  standby_ref。
- 当前一个 holding run 因新用户 run 开始 transition 而让路时，前一个 run
  才转为 `done`。
- `/standby_velocity` 从 holding 明确回 standby 时，held run 最终状态为
  `done`：动作已经自然到达末帧，用户只是离开该姿态，不是中止动作。
- `/stop`、`/passive`、`/fixstand` 从 holding 退出时，held run 最终状态为
  `stopped`。
- 现有 wait 客户端如果只把 `done/stopped/failed/canceled` 视为终态，会在
  `holding` 持续等待；skill/CLI 必须新增 hold-aware 等待策略，避免把正常
  holding 误判为 timeout。

### Holding/Transition 控制命令表

| 输入 | holding 中行为 | transition 中行为 |
| --- | --- | --- |
| `/execute queue` | 新用户 run 进入用户 queue；held run 在 user-to-user transition 开始时转 `done`；transition 不占 queue limit | 如果当前 transition 目标是 user，新 run 排在目标 user 后；如果当前 transition 目标是 reserved/gated standby，可停止该 standby transition 并重建到新 user 的 transition；均不生成 transition id |
| `/execute interrupt` | 按现有 interrupt 规则取消/改写用户等待队列；held run 在新 user transition 开始时转 `done` | 按现有 interrupt 规则处理用户 work；中断当前 internal transition，并从当前 reference frame 重建到 interrupt 目标的 transition；不生成 transition id |
| `/stop` | 立即 abort held user；held run 转 `stopped`；保留 stop watermark；清理 active/queue/idle 按现有 stop 合同；不播放 `standby_ref.trk`；之后由现有逻辑进入 `standby_velocity` 或 safety/passive/fault | 立即 abort 当前 internal transition 或 standby_ref；保留 stop watermark；清理 active/queue/idle 按现有 stop 合同；不播放 `standby_ref.trk`；之后由现有逻辑进入 `standby_velocity` 或 safety/passive/fault |
| `/passive` | held run 转 `stopped`；立即进入安全 passive 路径；不播放 `standby_ref.trk` | 立即停止 internal transition 并进入 passive；不播放 `standby_ref.trk` |
| `/fixstand` | held run 转 `stopped`；进入 FixStand；不播放 `standby_ref.trk` | 停止 internal transition 并进入 FixStand；不播放 `standby_ref.trk` |
| `/standby_velocity` | held run 转 `done`；明确回 standby。runtime-gated standby_ref 可走 held frame -> standby_ref -> `standby_velocity`；若 safety/readiness 不允许则 abort 到 safety/passive/fault，不强行播放 | 停止或接管当前 internal transition。runtime-gated standby_ref 可走当前 frame -> standby_ref -> `standby_velocity`；若 safety/readiness 不允许则 abort 到 safety/passive/fault |

## 4. Internal Synthetic Transition

synthetic transition 是 runtime 内部对象，不是用户提交的 motion：

- 不进入用户 queue。
- 不生成用户 run id。
- 不占 `queue.limit`。
- 不出现在 `queue.ids`。
- 不可通过 `GET /status?id=...` 查询。
- 不写入用户 run history。

最小实现建议：

- 新增内部状态，例如 `GeneralTrackerSyntheticTransition`。
- 新增小型 app-local helper 负责生成 in-memory `TrkTrack`；不要改
  `ObservationBuilder`，也不要直接 include ET1 app 中的
  `deploy/include/LinearInterpolator.h`。可以参考其插值思路，但实现必须留在
  `agentic-et1-tracker` 本地边界内。
- 输入为 `source_frame` 和 `target_frame`：
  - `source_frame` 来自 held last frame、当前 user/idle reference frame；standby
    reference 当前帧只在 standby_ref runtime gate 允许的回 standby 路径中参与。
  - `target_frame` 来自下一段用户 `.trk` 第一帧、选中 idle `.trk` 第一帧；
    `standby_ref.trk` 第一帧只在 standby_ref runtime gate 允许的回 standby
    路径中参与。
- 在内存中构造 synthetic `TrkTrack`，不落盘，不写入用户 motion 目录。
- fps 使用 target `.trk` fps；内部 duration 固定为一个小常量，先不暴露配置。
- 输出 `TrkTrack` metadata：
  - synthetic transition 使用 endpoint-inclusive span，与现有 TRK 约定一致。
  - `intervals = ceil(duration_s * fps)`，至少 1 个 interval。
  - `frames = intervals + 1`，至少 2 帧。
  - `fps = target.metadata.fps`，如果目标 fps 无效则拒绝构造。
  - metadata `duration_s = (frames - 1) / fps`。
- 输出 required arrays 必须覆盖现有 schema 的 10 类字段，shape 与用户 `.trk`
  完全一致：
  - `joint_pos`: `[frames, 26]`
  - `joint_vel`: `[frames, 26]`
  - `body_pos_w`: `[frames, 27, 3]`
  - `body_quat_w`: `[frames, 27, 4]`
  - `body_lin_vel_w`: `[frames, 27, 3]`
  - `body_ang_vel_w`: `[frames, 27, 3]`
  - `left_foot_contact_state`: `[frames]`，frame view 为 scalar
  - `right_foot_contact_state`: `[frames]`，frame view 为 scalar
  - `ref_com_rel_navi`: `[frames, 3]`
  - `ref_com_vel_navi`: `[frames, 3]`
- frame 插值策略固定为 MVP：
  - position、joint、COM 线性插值。
  - quaternion 使用 shortest-path nlerp 后 normalize；点积为负时先翻转目标
    quaternion，避免走长弧。
  - contact state 使用 nearest/step：前半段保持 source，后半段切到 target。
  - `joint_vel` 使用 `joint_pos` finite difference；首末帧可用邻帧差分或置零，
    但必须 finite。
  - `body_lin_vel_w` 使用 `body_pos_w` finite difference。
  - `body_ang_vel_w` MVP 置零；后续只有在有明确测试收益时再改为 quaternion
    finite difference。
  - `ref_com_vel_navi` 使用 `ref_com_rel_navi` finite difference。
  - 所有输出 float 必须 finite；NaN/Inf 直接构造失败。
- synthetic transition 完成后立即启动目标用户 `.trk`、进入目标 idle `.trk`；
  回 standby 路径可在 runtime gate 允许时继续播放 `standby_ref.trk`。

对用户 run 的归属：

- 过渡到下一条用户 `.trk` 时，synthetic transition 依附于目标 run 的准备阶段，
  但不计入目标 run 的 `frame/frames/progress`。
- 过渡到 idle `.trk` 时，synthetic transition 使用 `transition.target:"idle"`，
  `target_id:null` 或内部 idle marker；不生成 user id、不进用户 queue、不写用户
  history。transition 完成后 idle 仍用独立 `active.kind:"idle"` 表达。
- transition 期间 active kind 应明确为 `transition`，不得用
  `active.kind:"user"` 搭配 `exec.state:"queued"` 表示内部过渡。
- 目标 run 的 `.trk` 第一帧开始执行时，`exec.frame` 从 0 正常推进。
- 上一条 holding run 在新用户 transition 开始时完成为 `done`；显式
  `/standby_velocity` 离开 held 姿态时也完成为 `done`；`/stop`、`/passive`、
  `/fixstand` 退出时完成为 `stopped`，不再继续占用 active user id。

## 5. Built-in Standby Reference Asset

新增 app-owned release asset：

```text
deploy/robots/agentic-et1-tracker/config/reference/standby/v0/standby_ref.trk
deploy/robots/agentic-et1-tracker/config/reference/standby/v0/ASSET_MANIFEST.yaml
deploy/robots/agentic-et1-tracker/config/reference/standby/v0/README.md
```

当前 release 中该 asset 已固化并记录 targeted simulator asset acceptance。
runtime gate 已接入并由 unit/runtime/release selftest 覆盖；broader
MuJoCo/operator 场景和 real robot gate 仍 pending，`transition.target:"standby"`
仍不能作为整体 GA 能力宣称。

用途边界必须写死：

- `standby_ref.trk` 只用于到 `standby_velocity` 前的短过渡 reference。
- `standby_ref.trk` 不能替代 `/standby_velocity`。
- `standby_ref.trk` 不能替代 Velocity0 policy。
- `standby_ref.trk` 不能循环播放。
- `standby_ref.trk` 不是 idle pool，不接受用户配置，不进入 `motion_dirs`
  allowlist，不作为 `/execute` 用户 path。
- `standby_ref.trk` 播放完成后，runtime 必须进入 `standby_velocity`，后续仍由
  Velocity0 维持站立。
- `standby_ref.trk` 不能从 `passive`、`fault` 或 `lowcmd_occupied` 自动启动；
  这些状态仍按既有 safety/manual gate 处理。

加载路径和 allowlist：

- 新增内部 app-owned asset loader，默认使用 app-local release-resolved path：
  `deploy/robots/agentic-et1-tracker/config/reference/standby/v0/standby_ref.trk`。
- 该 loader 可复用 `TrkLoader` 和 validator 机制，但必须使用独立 internal
  allowlist：仅允许上述 release-resolved standby reference 目录。
- internal allowlist 不等同用户 `motion_dirs`，也不能写回用户
  `motion_dirs`。
- 用户不得通过 `/execute` 引用 `standby_ref.trk`；这不能破坏现有用户 motion
  allowlist。

生成和固化流程：

- 可离线使用 `nl2trk` 或其他生成方式产出候选 reference。
- repo-local pre-release candidate 可以用
  `tools/derive_standby_ref_candidate.py` 从 source `.trk` exact-slice tail
  window 生成；默认 tail window 为 25 frames，输出必须使用
  `standby_ref.candidate.trk` 和 `CANDIDATE_MANIFEST.json`，不得直接输出
  release `standby_ref.trk`。
- candidate manifest 必须保持
  `status:"candidate_pending_mujoco_acceptance"`，记录 source/candidate
  sha256、source frame window、frames/fps/duration、size、tool version 和静态
  metrics。candidate 可放在 build-local evidence 目录，例如
  `build/standby_ref_candidate/`，不得写入用户 `motion_dirs`，不得写入
  `config/reference/standby/v0/`。
- 候选必须离线筛选：schema 校验、MuJoCo 视觉检查、低风险姿态检查、起止帧与
  Velocity0 兼容性检查。
- 通过筛选后将单个 `standby_ref.trk` 固化为 app-local release asset，并记录
  sha256、size、fps、frames、duration、来源说明和验收日期。
- 运行时绝不调用 `nl2trk`，只读取已固化的 `standby_ref.trk`。

发布影响：

- release package 需要包含 `config/reference/standby/v0`。
- asset manifest 是发布审计材料，不是运行时下载器、注册表或 fallback。
- 不要求修改 `config.sim.yaml.example`。新 release asset 由 packaging 包含；
  本计划不做 YAML 配置迁移。

## 6. 状态与 Status API

建议新增可见语义：

- `MotionState::Holding`，JSON 字符串为 `holding`。
- `ActiveKind::Transition`，JSON 字符串为 `transition`。

不新增 public `ControllerState::Holding` 或
`ControllerState::Transitioning`。对外 `ctrl` 保持现有枚举表，不扩大
`/execute` gate 和 control conflict 的 public controller state surface。

`ctrl` 语义：

- holding 是 GeneralTracker user active 的一种 run state；对外 `ctrl` 可保持
  现有 `running`。
- transition 是内部 GeneralTracker 过渡；对外用 `active.kind:"transition"` 和
  独立 `transition` 字段表达，不新增 public `ctrl`。

`GET /status` 保持现有字段，并新增最小 transition/hold 信息：

```json
{
  "ctrl": "running",
  "active": {"kind":"user","id":"run-a"},
  "exec": {
    "id":"run-a",
    "state":"holding",
    "frame":119,
    "frames":120,
    "time_s":2.38,
    "duration_s":2.4,
    "progress":1,
    "hold":true,
    "stop_reason":null,
    "err":null
  },
  "transition": {
    "active": false,
    "target": null,
    "target_id": null,
    "target_state": null,
    "frame": 0,
    "frames": 0,
    "progress": 0
  }
}
```

transition 中：

```json
{
  "ctrl": "running",
  "active": {"kind":"transition","id":null},
  "exec": null,
  "transition": {
    "active": true,
    "target": "user",
    "target_id": "run-b",
    "target_state": "queued",
    "frame": 8,
    "frames": 25,
    "progress": 0.32
  },
  "queue": {"n":0,"limit":8,"ids":[]}
}
```

转入 idle 的 transition 中：

```json
{
  "ctrl": "running",
  "active": {"kind":"transition","id":null},
  "exec": null,
  "transition": {
    "active": true,
    "target": "idle",
    "target_id": null,
    "target_state": "running",
    "frame": 8,
    "frames": 25,
    "progress": 0.32
  },
  "queue": {"n":0,"limit":8,"ids":[]},
  "idle": {"enabled":true,"n":2,"active":false,"current":null}
}
```

reserved/gated standby transition 中（当前缺少 validated app-local
`standby_ref.trk` asset 与 manifest 时不得 emitted）：

```json
{
  "ctrl": "running",
  "active": {"kind":"transition","id":null},
  "exec": null,
  "transition": {
    "active": true,
    "target": "standby",
    "target_id": null,
    "target_state": null,
    "frame": 8,
    "frames": 25,
    "progress": 0.32
  }
}
```

约束：

- `exec` 和 `queue` 仍只描述用户 run。
- transition 期间 `active.kind` 使用 `transition`；`transition.target` 和
  `transition.target_id` 指向目标。
- 为避免 LLM agent 混淆，transition 期间 top-level `exec` 建议为 `null`；
  目标用户 run 只通过 `transition.target_id` 和可选 `target_state` 暴露。
  `GET /status?id=<target_id>` 可以返回该用户 run 的 `queued` 状态，
  但 synthetic transition 的 `frame/progress` 不得计入该 run 的
  `exec.progress`。
- `transition.target:"idle"` 表示 user-to-idle，或 future gated standby-to-idle
  的内部过渡；`target_id` 为 `null`，不生成用户 id。transition 完成后才切到
  `active.kind:"idle"`，idle 仍不进入用户 queue/history。
- `transition.target:"standby"` 是 reserved/gated target；当前 validated
  app-local `standby_ref.trk` asset 与 manifest 不存在时不应 emitted，也不应
  在 skill/docs 中声明 available。当前已可用的 app-local transition target 是
  `user` 和 `idle`。
- synthetic transition 不应让 `queue.n` 增加。
- `GET /status?id=<id>` 只返回用户 run；transition 没有 id，也不能查询。
- `/_sim/reference_frame` 在 reference enabled 时应能显示 holding frame 和
  synthetic transition frame；standby_ref frame 已受 runtime gate 覆盖，broader
  MuJoCo/operator 验收仍 pending。
- status 不需要暴露绝对 path；保持当前隐私和 token 紧凑策略。

## 7. Runtime 实现步骤

1. 扩展数据模型
   - `include/agentic_et1_tracker/core/types.hpp`：
     在 `MotionRequest` 增加 `hold`；增加 `MotionState::Holding`、
     `ActiveKind::Transition`。不新增 public `ControllerState`。
   - `include/agentic_et1_tracker/api/service.hpp`：
     在 `ExecuteCommand` 透传 `hold`。
   - 增加内部 transition 状态和 `TransitionStatus`。
   - `ExecutionCommandSink`、`CommandMailbox`、`RuntimeBridge` 透传 `hold`。

2. 扩展 `/execute` schema
   - `src/api_service.cpp` 的 `AgentApiService::execute` 允许 `hold` boolean。
   - 保持其他额外字段拒绝。
   - response 回显 `hold`。
   - schema 失败路径继续保证 validator/sink/id generator 调用次数为 0。
   - `src/runtime_bridge.cpp` 的 `motionRequest` 将 `ExecuteCommand::hold`
     写入 `MotionRequest::hold`。

3. 实现 last frame hold
   - 用户 `.trk` 到达最后一帧时，如果 `hold:true`，不立即
     `finishActive(Done)`。
   - 将状态切到 `holding`，保持 `active.kind:"user"` 和原 run id。
   - 每个 tick 继续写最后一帧对应的 GeneralTracker command，并发布最后一帧
     reference。
   - `/stop`、`/passive`、`/fixstand`、`/standby_velocity` 和 interrupt 能退出
     holding，且不破坏现有 stop watermark。
   - 主要落点在 `src/runtime_control_loop.cpp` 的 `advanceActiveWithPolicy`；
     纯测试/无 policy 路径的 `advanceActive` 也要保持同等语义。

4. 实现 synthetic transition builder
   - 从现有 `TrkTrack` frame view 读取 source/target。
   - 用 app-local helper 构造 in-memory transition track。
   - 不修改 `ObservationBuilder`。
   - 不直接 include `deploy/include/LinearInterpolator.h`，只参考其插值思路。
   - 复用 policy runner 执行，不走用户 queue，不生成 id。
   - 增加单元测试覆盖插值维度、frame count、边界帧、quaternion normalization。

5. 实现 user-to-user transition
   - 从 holding 到下一条用户 run 时插入 synthetic transition，主要落点在
     `src/runtime_control_loop.cpp` 的 `startNext`。
   - transition 完成后目标 run 从 frame 0 开始正常执行。
   - 队列容量和 `queue.ids` 只反映用户等待队列。
   - `src/runtime_control_loop.cpp` 的 `startIdle` 仍保持现有 idle 优先级：用户
     work 为空且 ready/safe 时才可启动 idle，不因 hold/transition 扩大触发条件。
   - 当 `hold:false` 用户 run 自然完成且 idle pool 已配置、ready/safe、无 user
     work 时，从用户最后一帧到选中 idle `.trk` 第一帧插入 synthetic transition，
     status 使用 `transition.target:"idle"`；完成后进入 `active.kind:"idle"`。

6. 实现 gated standby transition
   - 当前 release 已固化 app-local `standby_ref.trk` 与 manifest，runtime gate
     已接入，并由 unit/runtime/release selftest 覆盖；当前 release 仍不应声明
     broader MuJoCo/operator 或 real robot GA。
   - 加载固化 `standby_ref.trk`。
   - 新增内部 app-owned asset loader，使用 app-local release-resolved default
     path 和独立 internal allowlist。
   - 在 runtime gate 允许的回 standby 路径中，从当前 held frame 到
     `standby_ref.trk` 第一帧插入 synthetic transition。
   - 在 runtime gate 允许的回 standby 路径中播放 `standby_ref.trk`。
   - 完成后进入 `standby_velocity`，由 Velocity0 维持站立。
   - standby_ref 只用于自然完成且无新 user work，或显式 `/standby_velocity`
     回 standby；不得用于 `/stop`。
   - `src/runtime_control_loop.cpp` 的 `handleStop` 必须立即 abort 当前
     user/transition/standby_ref，清理 active/queue/idle 按现有 stop 合同，不播放
     standby_ref，并继续保留 stop watermark。
   - `handleControl` 必须能打断 holding/transition，并继续遵守 safety gate；
     `/passive` 和 `/fixstand` 不播放 standby_ref，不得从
     `passive/fault/lowcmd_occupied` 自动启动 standby ref。

7. 扩展 status/reference 输出
   - `src/json_codec.cpp` 的 `statusSnapshotJson` 增加 `transition`。
   - `motionStatusJson` 支持 `holding`。
   - reference sink 发布 hold、synthetic transition；standby_ref 当前帧只在
     runtime gate 允许的回 standby 路径中发布。

8. 发布资产落地
   - 离线生成候选 `standby_ref.candidate.trk`。
   - 离线校验并筛选一个版本。
   - 添加 asset、manifest、README。
   - 更新 packaging/selftest 中的资产存在性和 sha256 检查。

9. skill/CLI/docs 更新
   - `packaging/skills/et1-trk2motion/scripts/et1-trk2motion` 在现有子命令风格下
     支持 `et1-trk2motion run PATH --hold`，只把 `hold:true` 传给 `/execute`，
     不增加新 workflow。
   - 更新 `packaging/skills/et1-trk2motion/references/raw-http.md`、
     `state-machine.md`、`output-contract.md` 和 `SKILL.md`。
   - 更新 `deploy/robots/agentic-et1-tracker/README.md`，其中 `active.kind` 合同
     必须从 `none|user|idle` 扩展为 `none|user|idle|transition`。
   - 当请求 `hold:true` 且 `--wait` 时，默认 wait 在 `holding` 返回成功 held
     状态。后续如果需要“继续等到终态”，再另加显式选项；本计划只要求
     hold-aware default。
   - transition 期间等待逻辑读取 `transition.active` 和 `target_id`，不得把内部
     transition 当成用户 run 终态或 timeout。

## 8. 测试计划

API tests：

- `/execute` 接受 `hold:true`、`hold:false` 和省略 `hold`。
- `/execute` 拒绝 `hold` 非 boolean。
- `/execute` 继续拒绝 `paths` 和所有额外字段。
- schema 失败不调用 validator、sink、id generator。
- response 回显 `hold`。
- `hold` 作为 per-run metadata 透传，不改变 `mode`，不读取或修改
  `stop_hold_s`。

Core/status tests：

- `MotionState::Holding` 序列化为 `holding`。
- `ActiveKind::Transition` 序列化为 `transition`。
- 不新增 public `ControllerState::Holding` 或 `ControllerState::Transitioning`；
  holding/transition 的 top-level `ctrl` 保持现有 `running` 或既有安全状态。
- `GET /status` 包含 `transition` 默认 inactive 对象。
- holding 中 `active.kind:"user"`、`exec.state:"holding"`、`progress:1`。
- transition 中 `active.kind:"transition"`，`transition.target/target_id` 指向
  目标，top-level `exec:null`，`queue.limit` 不变、`queue.ids` 不包含
  transition。
- user-to-idle transition 显示 `transition.target:"idle"`、`target_id:null`，
  transition 完成后 `active.kind:"idle"`，且用户 queue/history 不变。
- `GET /status?id=<transition>` 不存在，因为 transition 没有 id。
- transition 期间目标用户 run 只通过 `transition.target_id` 和可选
  `target_state` 暴露；`GET /status?id=<target_id>` 可返回 `queued`，
  但 synthetic transition 的 frame/progress 不计入该 run 的 progress。

Runtime tests：

- `hold:false` 保持原有用户 run 完成行为；完成后如果 idle pool 已配置且
  ready/safe/no user queue/interrupt/pending，则进入 idle，否则按既有路径回
  `standby_velocity`。standby transition/ref 由 runtime gate 覆盖，broader
  MuJoCo/operator 和 real robot 验收仍 pending。
- `hold:true` 到最后一帧后进入 holding，不立即清 active user。
- holding run 到末帧后 `exec.state:"holding"`、`progress:1`，原 id 仍可通过
  `GET /status?id=<id>` 查询。
- holding run 在新用户 run 开始 transition 时才转 `done`。
- `/standby_velocity` 从 holding 回 standby 时 held run 转 `done`。
- `/stop`、`/passive`、`/fixstand` 从 holding 退出时 held run 转 `stopped`。
- holding 每个 tick 重复发布最后一帧 reference。
- holding 后提交下一条 `queue` run，会插入 synthetic transition，transition
  不增加用户 queue limit 用量。
- holding 后提交 `interrupt` run，会按现有 interrupt 规则处理用户队列，并插入
  synthetic transition。
- `hold:false` 用户 run 自然完成且 idle pool 可启动时，插入 user last frame ->
  idle first frame 的 synthetic transition，`transition.target:"idle"`，不生成
  用户 id、不进 queue/history。
- synthetic transition 完成后，目标 user run 从 frame 0 开始。
- `/stop` 在 holding、transition 或 standby_ref 中立即 abort，保留现有 stop
  watermark 语义，清理 active/queue/idle 按现有 stop 合同，不播放
  `standby_ref.trk`。
- `/passive` 立即进入 passive 且不播放 `standby_ref.trk`；`/fixstand` 进入
  FixStand 且不播放 `standby_ref.trk`。
- `/standby_velocity` 是明确回 standby；runtime-gated standby_ref 可走
  standby_ref；safety/readiness 不允许时必须 abort 到既有
  safety/passive/fault 路径。
- 自然完成后无新 user work、或显式 `/standby_velocity` 回 standby 时，允许执行
  current frame -> synthetic transition -> `standby_ref.trk` ->
  `standby_velocity`。
- `standby_ref.trk` 不能循环播放，不能从 `passive/fault/lowcmd_occupied`
  自动启动。
- bad orientation、lowcmd occupied、policy inference failure 仍走现有 safety/fault
  路径；不得因为 transition 绕过 safety gate。

TRK/asset tests：

- synthetic builder 输出 10 个 required arrays，shape 与 schema 一致，metadata
  `frames/fps/duration_s` 按 endpoint-inclusive span 一致，所有 float finite。
- synthetic builder 覆盖 shortest-path nlerp normalize、finite difference
  velocity、contact nearest/step 的边界测试。
- standby_ref asset 已记录且 targeted simulator accepted；runtime gate 已接入，
  `standby_ref.trk` 必须通过 `TrkLoader` 和 validator。broader
  MuJoCo/operator 与 real robot GA gate 仍 pending。
- standby asset loader 使用独立 internal allowlist，只允许
  app-local release-resolved default path；不读取、不扩展用户 `motion_dirs`。
- 用户 `/execute` 引用 `standby_ref.trk` 必须继续被用户 motion allowlist 拒绝。
- asset manifest sha256、size、duration、fps 与文件一致。
- 缺失或损坏 `standby_ref.trk` 时，service 不应静默 fallback 到 ET1 app tree；
  应报告明确 not-ready 或 internal asset error。

Skill/CLI/docs tests：

- `et1-trk2motion run PATH --hold` 发送 `{"hold":true}`，不改变 `mode`，不增加
  其他 workflow。
- README、raw HTTP、state-machine、output-contract 和 skill 文档包含
  `holding`、`active.kind:"transition"`、`transition.target:"idle"` 和
  `transition` 字段说明。
- README 的 `active.kind` 合同更新为 `none|user|idle|transition`。
- 请求 `hold:true` 且 `--wait` 时，默认 wait 在 `holding` 返回成功 held 状态；
  transition 不被误判为 timeout。

MuJoCo acceptance：

- 单条 `hold:true` 动作结束后机器人保持末帧，无明显跳变。
- held last frame 到下一条 `.trk` 第一帧 transition 连续，足端和躯干无明显突变。
- user run 自然完成后进入 idle 时，用户最后一帧到 idle 第一帧 transition
  连续，随后 `/status.active.kind` 切到 `idle`。
- runtime-gated standby_ref 的 held last frame 回 standby 路径连续，并最终看到
  `ctrl:"standby_velocity"`；仍需 broader MuJoCo/operator 验收记录。
- `/stop` 在 holding/transition/standby_ref 中立即终止，不播放 standby_ref。
- `standby_ref.trk` 不替代 Velocity0：过渡完成后 Velocity0 仍在运行。
- transition 期间 `/status` 显示 `active.kind:"transition"` 和
  `transition.active:true`，用户 queue 不增加。
- `/_sim/reference_frame` 可观察到 hold/user/idle transition reference frame；
  standby_ref reference frame 的 broader MuJoCo/operator 验收仍 pending。

Real robot acceptance：

- 仅在 MuJoCo 验收通过后进行。
- 操作员确认 hold、user-to-user transition 均可人工停止；standby transition
  仍需 real robot/operator 验收。
- 验证 `/stop`、`/passive`、`/fixstand` 在 holding/transition 中优先级正确。
- 验证低电平占用和 bad orientation 不被 hold/transition 自动恢复逻辑绕过。

## 9. 风险与缓解

- 末帧长期保持可能让策略处于训练分布边缘：先限制为显式 `hold:true`，默认不启用。
- synthetic transition 插值可能产生不自然脚接触或速度：先用短时、保守插值，并以
  MuJoCo 视觉和安全验收作为发布 gate。
- `standby_ref.trk` 与 Velocity0 姿态不兼容会导致过渡末端跳变：runtime
  playback gate 已 unit-covered，但 broader MuJoCo/operator 和 real robot gate
  仍需验证 Velocity0 动态兼容性。
- 状态扩展可能影响 LLM agent 解析：保持 `exec/queue` 用户语义不变，并将
  transition 放到独立字段。
- 内部 transition 如果误入 queue，会破坏 queue limit 和 run history：测试必须
  明确断言不进 queue、不生成 id、不占 limit。
- asset 缺失风险：发布包 selftest 必须检查 `standby_ref.trk` 和 manifest。

## 10. 非目标

- 不实现上传接口。
- 不支持远程 URL、base64、JSON motion payload、多格式输入或目录扫描。
- 不支持用户指定 transition 曲线、duration、standby asset、blend profile。
- 不把 synthetic transition 暴露为可提交任务。
- 不让 synthetic transition 拥有用户 run id。
- 不把 `standby_ref.trk` 放入 idle pool 或用户 motion allowlist。
- 不运行时调用 `nl2trk` 或任何外部生成服务。
- 不改 ET1 app。
- 不改 `config.sim.yaml.example`。
- 不引入外部业务状态机、复杂工作流或 UI。

## 11. 验收清单

- `/execute {"path":"...","hold":true}` 可接受，并在最后一帧进入 `holding`。
- `/execute` 省略 `hold` 行为兼容当前版本。
- holding 中 `/status` 清楚显示 user run 正在 hold last frame。
- holding/transition 不新增 public controller state；对外 `ctrl` 保持现有表。
- user-to-user synthetic transition 不进用户 queue、不生成用户 run id、不占
  queue limit。
- standby reference runtime gate 已接入；只有 broader MuJoCo/operator 和 real
  robot/operator 验收完成后才能宣称整体 GA。缺失安全资产时不得播放 standby_ref。
- 运行时不调用 `nl2trk`；`standby_ref.trk` 必须离线生成、筛选、固化并记录
  manifest 后才能作为 release asset 使用。
- `et1-trk2motion run PATH --hold` 和 wait 逻辑能正确处理 `holding` 与
  transition。
- 所有新增状态/API 有单元测试、runtime 测试和 MuJoCo 验收记录。
