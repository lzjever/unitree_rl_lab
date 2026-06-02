# Agentic ET1 Tracker GA 状态机与 Idle 动作池开发计划

本文档是当前 GA idle/status 实现计划和验收参考，不是临时草稿。

## 0. 收敛澄清（2026-06-02）

- `lowcmd_occupied` 是 block-aware manual 状态：即使 `err.code` 暂为
  `ROBOT_NOT_READY`，API 也必须由 `block:"lowcmd_occupied"` 导出
  `next:"manual"`；`/stop`、`/idle`、readiness 重算或用户 `/execute`
  都不能自动恢复或抢回 LowCmd。
- `active.kind` 是外部状态权威。`active.kind:"idle"` 表示 idle active，
  不是用户 run；`exec`、`queue`、`GET /status?id=` 只表示用户动作，不能
  从 `ctrl:"running"` 推断有用户 run id。
- runtime/bridge 直连测试可保留内部 queue-in-FixStand/stop-watermark 需要
  的低层能力；GA 对外门禁保持 API-only：`AgentApiService` 在
  `passive/fixstand/stopping/fault` 对 `/execute` 先返回冲突，不进入
  validator、sink 或 id generator。

## 1. 目标和非目标

目标：

- 面向 GA 和 LLM agent 使用，保持 HTTP 合同短、确定、可轮询、可恢复。
- 保持普通 `/execute` 现有合同不变：`POST /execute {"path":"/absolute/file.trk","mode":"queue|interrupt"}`，只接受本地绝对 `.trk`，不上传，不支持 `paths`。
- 新增/改造 `/idle` 为独立 idle 动作池配置接口：`POST /idle {"paths":["/abs/a.trk","/abs/b.trk"]}` 覆盖动作池，`POST /idle {"paths":[]}` 清空动作池。
- idle 与用户 motion queue 完全隔离：不占 `queue.limit`，不进入 `queue.ids`，不生成用户 run id，不可被 `GET /status?id=` 查询。
- 建立清晰状态链路：`passive -> fixstand -> standby -> execute 或 idle`，其中 standby 对应内部 `standby_velocity`/`Velocity0`。
- 修正 safety 阻断项：orientation outside safe limits 在执行/standby 路径进入 `passive`，同时保留 FixStand 软件恢复例外；`lowcmd_occupied` 保持 fault/manual，不自动恢复。

非目标：

- 不设计上传、远程 URL、非 `.trk`、内嵌 motion payload、批量 `/execute paths`。
- 不设计复杂调度、权重、优先级配置、OTA、UI。
- 不把 idle 当成用户 run；不提供 idle 历史 run 查询。

## 2. 当前基线和必须保持的行为

当前代码入口需要沿用并最小改动：

- HTTP 路由在 `deploy/robots/agentic-et1-tracker/src/http_server.cpp` 的 `AgentHttpServer::installHandler`。
- `/execute` gate 在 `src/api_service.cpp` 的 `AgentApiService::execute`、`executeBlockedByController`、`controlStateConflictInfo`。
- 控制状态枚举在 `include/agentic_et1_tracker/core/types.hpp` 的 `ControllerState`。
- runtime 内部状态在 `include/agentic_et1_tracker/runtime/runtime_control_loop.hpp` 的 `RuntimeInternalState`。
- runtime 状态映射和启动/停止链路在 `src/runtime_control_loop.cpp` 的 `controllerStateForInternal`、`isMotionAcceptingState`、`startNext`、`advanceActiveWithPolicy`、`writeFixStand`、`writeStandbyVelocity`、`handleStop`、`handleControl`、`enterStopping`、`readinessRequiresFault`。
- safety readiness 在 `src/robot_io.cpp` 的 `mapRobotReadiness`、`hasSafeBodyOrientation`。
- status/run 存储在 `src/runtime_status_store.cpp` 的 `RuntimeStatusStore`。
- JSON schema 在 `src/json_codec.cpp` 的 `statusSnapshotJson`。
- 当前 HTTP contract 记录在 `README.md`。
- ET1 参考链路在 `deploy/robots/et1/config/config.yaml`：`Passive`、`FixStand`、`Velocity`、`Track` 主路径；`deploy/robots/et1/src/State_Velocity.cpp` 已有 `bad_orientation -> Passive`；`deploy/robots/et1/src/State_Track.cpp` 的 active track `bad_orientation` 当前注释掉。

必须保持：

- `/execute` request shape、现有错误 envelope、`.trk` 和 `motion_dirs` allowlist 校验行为不退化。
- 用户 queue FIFO、`queue.limit`、`queue.ids`、`GET /status?id=<id>` 只代表用户提交的动作。
- `/fixstand`、`/standby_velocity`、`/stop` 空 body 合同保持。
- 已有 run 状态、stop reason、queue cancel/interrupt 语义保持，除非测试明确围绕新状态链路反转。

## 3. 目标状态机和转换表

目标状态链路：

```text
passive -> fixstand -> standby_velocity -> execute
                                  \-> idle
```

转换表：

| 当前状态 | 允许输入 | 目标/行为 | 禁止或冲突 |
| --- | --- | --- | --- |
| `passive` | `/fixstand`, `/stop`, `/idle` | `/fixstand` 是软件恢复入口；`/stop` 保持/回到安全 sink；`/idle` 只配置或清空，不播放 | `/standby_velocity`、`/execute` 返回 `CONTROL_STATE_CONFLICT` |
| `fixstand` | `/standby_velocity`, `/fixstand`, `/stop`, `/idle` | `/standby_velocity` 进入 standby；`/stop` 停止后按 safety 落点；`/idle` 只配置或清空，不播放 | `/execute` 返回冲突 |
| `standby_velocity` | `/execute`, `/idle`, `/fixstand`, `/stop` | 用户动作优先；无用户 active/queue 且 idle 配置可用时自动播放 idle | 无 |
| `idle active` | `/execute`, `/stop`, `/fixstand`, `/standby_velocity`, `/idle` | `/execute` 抢占 idle；`/stop` 停 idle 并清空 idle 配置；`/idle` 只更新配置，不进入用户 queue | idle 期间不得阻塞用户 queue/interrupt |
| `preparing/running` | `/execute`, `/stop`, 控制命令, `/idle` | `queue` 排队；`interrupt` 抢断用户 active；`/stop` 最高优先级；`/idle` 可配置但不立即播放 | 无 |
| `stopping` | `/stop`, `/status`, `/idle` | 幂等停止；停止完成后按 safety 回 `standby_velocity` 或 `passive/fault`；`/idle {"paths":[]}` 至少可清空 | `/execute` 返回冲突或保持现有策略，不在 stopping 新开 idle |
| `fault/manual` | `/fixstand`, `/stop`, `/status`, `/idle` | `lowcmd_occupied` 等人工占用需 operator 释放；`/idle` 只配置或清空，不播放；不得自动恢复 | `/execute` 不播放 |

说明：

- 对外 `ctrl` 可以继续使用已有 `standby_velocity`、`preparing`、`running`、`stopping`，但必须通过 top-level `active:{"kind":"none|user|idle","id":...}` 区分 idle 与用户动作。
- idle 播放使用 GeneralTracker/Track 能力，但它是 runtime 内部 idle active，不是用户 `exec`。

## 4. HTTP 合同

`/execute`：

```http
POST /execute
{"path":"/absolute/file.trk","mode":"queue|interrupt"}
```

- `mode` 保持现有默认值和语义。
- GA 必须收紧 request schema：只允许 `path` 和可选 `mode`；任何额外字段都返回 400 `REQUEST_INVALID`。
- 明确拒绝 `paths`：`{"paths":[...]}` 和 `{"path":"...","paths":[...]}` 都必须 400，且不得调用 validator、command sink 或 id generator。
- 不接受上传、URL、非 `.trk`。
- 如果 idle 正在播放，`mode:"queue"` 和 `mode:"interrupt"` 都应让 idle 让路；`interrupt` 应尽快进入停止 idle 并启动用户动作。

`/idle`：

```http
POST /idle
{"paths":["/abs/a.trk","/abs/b.trk"]}
```

成功响应建议：

```json
{"ok":true,"idle":{"enabled":true,"n":2,"active":false}}
```

清空：

```http
POST /idle
{"paths":[]}
```

响应：

```json
{"ok":true,"idle":{"enabled":false,"n":0,"active":false}}
```

校验规则：

- `paths` 必须存在且为数组。
- 每个 path 必须复用 `/execute` 的绝对路径、`.trk`、存在性、`motion_dirs` allowlist 校验。
- 任一路径失败则整个请求失败，旧 idle 配置不变。
- `/idle` 是配置接口，不是 run 提交接口：配置/清空与播放解耦。
- 设置配置可在 `passive`、`fixstand`、`standby_velocity`、`preparing/running`、`stopping`、`fault/manual` 接受；实现若需要更保守，至少必须允许 `paths:[]` 在 `passive/fixstand/stopping/fault` 清空。
- `/idle` 配置不应把 standby 或 robot ready 当作接收前置条件；只有自动播放受 readiness/safety gate 限制。
- 自动播放只在 `standby_velocity` 且 `ready=true`、orientation safe、无用户 active、无用户 queue/interrupt/pending 时发生。

错误响应必须复用现有 envelope，不新增 `err/msg` schema：

```json
{"ok":false,"error":{"code":"TRK_PATH_NOT_ALLOWED","message":"track path is not allowed","retryable":false},"next":"fix"}
```

错误码复用现有枚举；除非作为单独兼容性变更列出，不修改现有 HTTP code 映射。特别是当前 `TRK_FILE_NOT_FOUND` 是 400，不在本计划中硬改为 404。

| 场景 | HTTP | `error.code` | `next` | 说明 |
| --- | --- | --- | --- | --- |
| body 非 JSON | 400 | `REQUEST_INVALID` | `fix` | 修正 JSON/body 后重试 |
| `/stop`、`/fixstand`、`/standby_velocity` body 非空 | 400 | `REQUEST_INVALID` | `fix` | 空 body required |
| `/execute` 缺少 `path`、`path` 非字符串、`mode` 非法、出现 `paths` 或其他额外字段 | 400 | `REQUEST_INVALID` | `fix` | schema 失败时不触发 validator/sink/id generator |
| `/idle` 缺少 `paths`、非数组、元素非字符串 | 400 | `REQUEST_INVALID` | `fix` | schema 失败时旧 idle 配置不变 |
| path 为空、URL、非绝对路径或非 `.trk` | 400 | `REQUEST_INVALID` | `fix` | 复用现有 validator 映射；真实规则由 trk validator tests 覆盖 |
| 文件不存在 | 400 | `TRK_FILE_NOT_FOUND` | `fix` | 保持现有 HTTP code |
| 不在 allowlist | 403 | `TRK_PATH_NOT_ALLOWED` | `fix` | 复用 `/execute` validator |
| parse/内容校验失败 | 400 | `TRK_PARSE_FAILED`/`TRK_VALIDATION_FAILED` | `fix` | 旧 idle 配置不变 |
| 状态不允许执行用户动作 | 409 | `CONTROL_STATE_CONFLICT` | `fixstand`/`standby_velocity`/`status` | 仅限制执行，不限制 idle 配置 |
| robot/service/model 未 ready | 503 或现有映射 | 复用现有 readiness error | 复用现有 `next` | 不新增错误码 |

`/stop`：

- 必须空 body。
- 最高优先级：停止当前用户 active；取消本次 stop watermark 之前已接受的用户 queued/pending；保留 stop 之后新接受的用户 queue/interrupt。
- 可无条件停止 idle active 并清空 idle config；idle 没有 run id，不参与 stop watermark。
- safety 允许时回 `standby_velocity`；bad orientation、lowcmd occupied 等 safety override 不可被绕过。

## 5. `/status` 目标 schema

原则：

- 在现有 `GET /status` 输出上追加字段，保留 `stop_reason`、`hz`、`low_ms`、`pose`、`block`、`err` 等现有字段。
- `exec` 和 `queue` 只表示用户动作；新增独立 `idle` 字段，避免 LLM agent 把 idle 当成用户 run。
- 统一判别字段为 top-level `active:{"kind":"none|user|idle","id":...}`。`none` 和 `idle` 的 `id` 为 `null`；只有用户动作填用户 run id。
- idle status 默认不暴露绝对 path，避免 token、隐私和紧凑性问题；用 `current` 或 `index` 表示当前 idle 条目。

目标 top-level 示例：

```json
{
  "ok": true,
  "ready": true,
  "mode": "sim",
  "robot": "idle",
  "ctrl": "standby_velocity",
  "stop_reason": null,
  "hz": 1000,
  "active": {"kind": "none", "id": null},
  "exec": null,
  "queue": {"n": 0, "limit": 8, "ids": []},
  "idle": {
    "enabled": true,
    "n": 2,
    "active": false,
    "current": null,
    "frame": 0,
    "frames": 0,
    "time_s": 0,
    "duration_s": 0,
    "progress": 0
  },
  "low_ms": 0,
  "block": null,
  "err": null,
  "pose": {"q": [1, 0, 0, 0], "g": [0, 0, 0], "p": null, "v": null}
}
```

idle 播放中：

```json
{
  "active": {"kind": "idle", "id": null},
  "exec": null,
  "queue": {"n": 0, "limit": 8, "ids": []},
  "idle": {
    "enabled": true,
    "n": 2,
    "active": true,
    "current": 0,
    "frame": 12,
    "frames": 120,
    "time_s": 0.24,
    "duration_s": 2.4,
    "progress": 0.1
  }
}
```

用户动作中：

```json
{
  "active": {"kind": "user", "id": "a7K3p9Qx"},
  "exec": {"id": "a7K3p9Qx", "state": "running"},
  "idle": {"enabled": true, "n": 2, "active": false, "current": null}
}
```

`GET /status?id=<id>`：

- 只查询用户 run id。
- idle 不生成 id，不在 `RuntimeStatusStore::findRun` 出现。
- 对 idle path 或空 id 返回现有 `RUN_NOT_FOUND`/bad request 风格，不新增 idle run 查询。

## 6. Runtime 架构方案

新增独立 active 模型和 idle 配置，名称可等价调整：

```text
ActiveKind
- None
- User
- Idle

IdleConfig
- paths: vector<filesystem::path>
- enabled(): !paths.empty()

IdleRuntime
- active: bool
- path: optional<path>
- current/index: optional<size_t>
- frame/time/progress
- stop_requested: bool
```

架构要求：

- 不把 idle 放入 `MotionQueue`、`waiting_` 或 `RuntimeStatusStore` 的 run map。
- `/idle` 的 API 层先完成全部 path 校验，再原子覆盖 idle 配置；失败时旧配置不变。
- 抽出通用内部 motion runner，复用 GeneralTracker/Track stepping、准备、停止和 safety policy。
- 只有 `ActiveKind::User` 发布 `MotionStatus`、`exec` 和 run map；`ActiveKind::Idle` 只发布 `StatusSnapshot.idle` 和 top-level `active.kind`。
- 建议把现有 `advanceActiveWithPolicy()` 拆为“通用 stepping + 按 kind 发布状态”，避免复制一套 policy runner。
- idle 随机/轮询策略保持简单；可选避免连续重复，不把具体随机实现细节作为 GA 合同。
- runtime tick 顺序保持用户优先：
  1. 处理 `/stop`、control command、`/execute interrupt`。
  2. 如果用户 active 或用户 queue 非空，不启动 idle。
  3. 如果 idle active 且用户 `/execute` 到来，先停止 idle，再让用户动作进入现有 queue/interrupt 流程。
  4. 只有 `standby_velocity`/内部 Velocity0 且 `ready=true`、无用户 active、无用户 queue、orientation safe 时随机选择 idle path 播放。
- idle 播放完成后回 standby，再按同样条件随机选择下一条；不要忙循环，至少等待一个 tick 并重新读取 readiness。
- `/stop` 调用 `handleStop` 时必须同时：
  - 停止用户 active。
  - 取消本次 stop watermark 之前已接受的用户 waiting/queue。
  - 保留 stop 之后新接受的用户 queue/interrupt。
  - 停止 idle active。
  - 清空 idle config。
  - 发布 status，使 `idle.enabled=false`、`idle.active=false`，且 `queue.ids` 只包含 watermark 之后仍应保留的用户 id。
- `handleControl(FixStand/StandbyVelocity)` 应停止 idle active；是否清空 idle 配置只由 `/stop` 和 `/idle {"paths":[]}` 负责，除非实现中为了安全选择更严格策略并补充测试。
- idle 抢断不应污染用户 run 的 `stop_reason`，因为 idle 没有 run id。

## 7. 安全策略

orientation：

- `hasSafeBodyOrientation` outside safe limits 必须在 standby、idle active、user active、preparing/running/stopping 强制进入 `passive`。
- `FixStand` 是显式软件恢复例外：只要 lowstate fresh、mode machine ok、lowcmd 未被外部占用，允许写 FixStand；否则会失去软件恢复入口。
- 当前 active tracking 代码在 `src/runtime_control_loop.cpp` 使用 `OrientationSafety::Skip`，计划要求反转：active tracking 不得跳过 orientation safety；相关测试同步反转。
- 不要求修改 ET1 app；本计划只要求 agentic runtime 层实现上述 orientation policy。ET1 参考中 `State_Velocity.cpp` 已有 `bad_orientation -> Passive`，可作为行为参照。

lowcmd occupied：

- `lowcmd_occupied` 表示底层控制权被占用，应保持 fault/manual/operator 状态；这是 GA 合同迁移项。
- 不应因为 `/stop`、`/idle`、用户 `/execute` 或 readiness 重算而自动恢复。
- 推荐保持 `block:"lowcmd_occupied"` 可见；即使暂时保留现有 `ROBOT_NOT_READY` 作为 `err.code`，API 响应也必须由 `block` 驱动 `next:"manual"`，不得继续给 LLM agent `wait_robot` 的自动恢复暗示。
- 同步更新 API next action/error table、README、raw HTTP 参考和测试；LLM agent 看到该状态时只能 `status`/`manual`，不能绕过。

`/stop` safety override：

- `/stop` 是 HTTP 控制优先级最高的命令，但不是 safety bypass。
- `/stop` 可请求停止用户 active，取消 stop watermark 之前的用户 queued/pending，保留 stop 之后新接受的用户 queue/interrupt。
- `/stop` 可无条件停止 idle active 并清空 idle 配置。
- bad orientation 必须落 `passive`；lowcmd occupied 必须保持 fault/manual，直到 operator 释放。

## 8. 精确 TDD 计划

只跑相关测试，避免无关格式化和大范围测试。

API tests：`deploy/robots/agentic-et1-tracker/tests/api_tests.cpp`

- 新增 `/idle` request shape：缺 `paths`、非数组、包含非字符串、空数组成功清空。
- 新增 idle path 校验复用 `/execute`：API tests 使用 fake validator，验证逐项调用、失败映射、旧配置原子性，不承担真实 path validator 全部规则。
- 真实规则如非绝对、非 `.trk`、不存在、不在 `motion_dirs`、parse/内容校验，放在 trk validator tests 覆盖。
- 新增原子性：`paths` 中一个失败时返回错误，旧 idle 配置保持不变。
- 保持 `/execute` 不接受 `paths`，普通单 `path` 合同不变；`{"paths":[...]}` 和 `{"path":"...","paths":[...]}` 均 400 `REQUEST_INVALID`，且 validator/sink/id generator 调用次数为 0。
- `/stop` 非空 body 返回 `REQUEST_INVALID`，空 body 成功并清 idle；用户 queue 断言必须遵守 stop watermark，不要求清掉 stop 后新接受的 id。

HTTP tests：`tests/http_server_tests.cpp`

- `AgentHttpServer::installHandler` 注册 `/idle`。
- `POST /idle` 成功/失败 JSON 和 HTTP code 与本计划错误表一致。
- `/stop` 空 body 约束通过 HTTP 层验证。
- `/status` 输出在现有字段上新增 `active.kind` 和 `idle` 字段；`stop_reason/hz/low_ms/pose` 保留；`exec/queue` 不因 idle active 改变。

Runtime/store tests：`tests/runtime_bridge_tests.cpp`、`tests/runtime_control_loop_tests.cpp`

- idle 配置不进入 `RuntimeStatusStore` run map；`findRun` 查不到 idle。
- standby 且 `ready=true`、无用户 active/queue、orientation safe 时启动 idle。
- 用户 queue 到来时 idle 让路；用户 interrupt 到来时尽快抢断 idle。
- idle active 时 `queue.ids` 仍为空，`queue.limit` 不变。
- 用户 active/queue 存在时不启动 idle；用户动作完成后回 standby 再按条件启动 idle。
- `/stop` 停用户 active，取消 stop watermark 之前的用户 queued/pending，保留 stop 后新接受的用户 queue/interrupt，同时停 idle、清 idle 配置。
- `/idle` 配置/清空与播放解耦；至少 `paths:[]` 在 `passive/fixstand/stopping/fault` 可清空，自动播放只在 standby+ready+safe+无用户工作时发生。
- HTTP/API 层：`passive` 正常只接受 `/fixstand` 执行恢复；`fixstand` 正常只接受 `/standby_velocity` 进入 standby；`/execute` 在 `fixstand` 返回冲突，standby 后才接受 `/execute`/可播放 idle。
- RuntimeBridge 直连 queue-in-FixStand 仅作为内部/API-only gate 的遗留语义暂保留，用于低层 queue/stop-watermark 覆盖；不扩大到 HTTP/API `/execute` 合同。
- 反转 active tracking orientation skip 相关测试：standby/idle/user active/preparing/running/stopping 中 bad orientation 进入 `passive`，而不是继续 track 或自动 fault。
- 增加 FixStand 恢复例外测试：lowstate fresh、mode ok、lowcmd 未占用时 bad orientation 仍允许写 FixStand；lowcmd 占用时不允许。

Robot IO tests：`tests/robot_io_tests.cpp`

- `mapRobotReadiness` 在 bad orientation 时给出阻断，并由 runtime 映射为 `passive`。
- `lowcmd_occupied` 保持 `block:"lowcmd_occupied"` 可见；如保留 `ROBOT_NOT_READY`，API next 必须转为 `manual`，不随 stop/idle 自动 ready。
- 覆盖 standby、idle、active 三类调用点，避免只测 Velocity。

Skill/CLI tests：

- 必须更新 `packaging/skills/et1-trk2motion/SKILL.md` 和 `packaging/skills/et1-trk2motion/references/raw-http.md` 的 status/error 合同说明。
- `packaging/skills/et1-trk2motion/scripts/et1-trk2motion` 保持 `/execute` 单 path 调用，不改成 `paths`。
- 脚本 `short_status` 至少输出 `active.kind`、`idle.enabled`、`idle.active`。
- `packaging/skills/et1-trk2motion/tests/test_et1_trk2motion.py` 更新 short status 兼容 `active.kind`/`idle` 字段，确保 idle active 不返回用户 id、不进入 wait-id，LLM agent 仍只以用户 run id 轮询。
- 如新增 idle CLI 子命令，应只做配置/清空，不返回 run id，不执行 wait-id 流程。

建议局部测试命令：

```bash
ctest --test-dir build -R 'agentic_et1_tracker_(api|http|runtime|robot_core)_tests' --output-on-failure
ctest --test-dir build -R 'agentic_et1_tracker_trk_tests' --output-on-failure
python3 deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion/tests/test_et1_trk2motion.py
```

## 9. 仿真和真机验证计划

MuJoCo：

- MuJoCo 路径：`/home/galbot/works/et1/unitree_mujoco`。
- 测试 `.trk` 路径：`/home/galbot/works/et1/generated/`。
- 使用至少两条低幅度、短时长 `.trk` 配置 idle pool。
- 验证 standby 后 idle 自动随机播放，`/status` 中 `idle.active=true`、`exec=null`、`queue.ids=[]`。
- idle 播放中提交 `/execute mode=queue`：idle 停止，用户 run 获得 id 并执行；`GET /status?id=<id>` 正常。
- idle 播放中提交 `/execute mode=interrupt`：尽快抢断 idle，用户动作优先。
- 用户动作排队/运行期间 idle 不启动。
- `/stop` 空 body：用户 active 停止，stop watermark 之前的用户 queued/pending 取消，stop 之后新接受的用户 queue/interrupt 保留；idle active 和 idle config 全部清空；安全允许时回 `standby_velocity`。
- 模拟 bad orientation：standby/idle/user active/preparing/running/stopping 均进入 `passive`，不能继续 idle 或 user track；FixStand 在 lowstate fresh、mode ok、lowcmd 未占用时仍可作为恢复入口。

真机：

- 先使用低风险、短动作、小幅度 `.trk`，每次只配置 1-2 条 idle。
- 验证链路顺序：`/fixstand` -> `/standby_velocity` -> `/idle` -> `/execute` -> `/stop`。
- 现场 operator 准备低层急停/接管；出现 `lowcmd_occupied` 时确认 tracker 保持 manual/fault，不自动抢回。
- 首轮只验证 idle standby 播放、用户抢占、`/stop` 清空、bad orientation passive，不做长时间随机循环。

## 10. 验收标准和开发顺序

验收标准：

- `/execute` 合同完全保持单 path；传 `paths` 失败。
- `/idle` 能覆盖/清空 idle pool；任一路径校验失败时旧配置不变。
- idle 不进入用户 queue/run/status-id 体系。
- `standby_velocity` 且 ready、安全、无用户工作时才播放 idle。
- 用户 `/execute` 总是优先；`interrupt` 能尽快抢断 idle。
- `/stop` 空 body、最高优先级，按 stop watermark 取消用户 queued/pending，保留 stop 后新接受的用户 queue/interrupt，清 idle config，且不绕过 safety。
- orientation outside safe limits 在 standby/idle/user active/preparing/running/stopping 进入 `passive`；FixStand 恢复例外保留。
- `lowcmd_occupied` 保持 fault/manual/operator 语义，`block` 可见且 `next:"manual"`，不自动恢复。
- 精确测试覆盖 API、HTTP、runtime、robot_io、skill/CLI schema，相关反转测试已更新；`packaging/skills/et1-trk2motion/SKILL.md`、`references/raw-http.md` 和脚本 short status 已同步。

开发顺序：

1. 先补 API/HTTP/status schema 测试，锁定 `/execute` 不变和 `/idle` 原子校验。
2. 增加 `StatusSnapshot`/JSON 的 `active.kind`、`idle` 字段，但不改变用户 `exec/queue`。
3. 增加 `IdleConfig`/`IdleRuntime` 独立结构和 `/idle` 配置通道。
4. 在 runtime tick 中接入 idle 调度、用户抢占、`/stop` watermark 和 idle 清空。
5. 修正 orientation safety，移除 active tracking 的 `OrientationSafety::Skip` 语义并反转测试，同时保留 FixStand 恢复例外。
6. 固化 `lowcmd_occupied` manual/fault 行为，并同步 API next action/error table。
7. 更新 README、`packaging/skills/et1-trk2motion/SKILL.md`、`references/raw-http.md` 和 skill/CLI 测试兼容 status schema。
8. 跑精确测试，再做 MuJoCo 验证，最后按低风险真机清单验收。
