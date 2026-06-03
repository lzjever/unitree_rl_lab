# agentic-et1-tracker PRD

GA 范围，KISS 收口版。

## 1. 结论

`agentic-et1-tracker` 是给 LLM agent 调用的独立 HTTP tracker app。它只做一件事：接收本机 `.trk` 路径，按队列或抢断语义执行 ET1 轨迹，并提供短、稳定、可高频轮询的状态接口。

它不是 `unitree_rl_lab/deploy/robots/et1` 的 wrapper，也不修改、链接或包装现有 ET1 app。ET1 app 只作为行为和底层边界参考。

这个要求合理，原因是：

- 不影响已有 ET1 app 的真机能力和人工操作流。
- 让 agentic tracker 有自己的产品语义、状态机和接口合同。
- 避免把现有 request 文件、FSM、profile 切换等历史机制暴露给 LLM agent。
- 真机安全上更容易审查，因为所有 agent 入口都在新 app 内闭环。

代价是会重新实现一小部分 ET1 相关控制逻辑，后续需要维护与模型/轨迹格式的一致性。这个代价可以接受，因为本服务的范围很小。

## 2. GA 范围

必须支持：

- 只执行 `.trk`。
- 只接受 `.trk` 路径。
- `.trk` 必须是 `agentic-et1-tracker` GeneralTracker GA schema 下的 ET1TRK1 v1 runtime cache 内容，不是 NPZ 或通用轨迹格式。
- 异步返回短 `id`。
- 动作顺序排队执行。
- 立即接受抢断请求，取消等待队列，并触发受控 stop-to-idle；新动作只能在 stop-to-idle 后开始。
- `/stop` 取消控制线程消费 stop 命令时已经存在的 active，以及 sequence 不大于该 stop command 且仍 queued/pending 的动作；健康可控路径回到 `ctrl:"idle"`。
- 查询当前动作、队列、机器人和控制器状态。
- 真机和仿真使用同一套 HTTP 接口。

明确不支持：

- 文件上传。
- 除 `.trk` 以外的任何动作输入。
- `.et1trk` 作为 HTTP 输入。
- 写 `debug/general_tracker_request.txt`。
- 调用现有 ET1 app 的 request 文件机制。
- 链接现有 ET1 app 的 CMake target 或内部类。
- 动作库、动作注册、动作搜索。
- pause/resume/seek。
- 多机器人。
- 在线 retarget。
- 复杂业务状态判断。
- 默认返回完整诊断大包。

KISS 的关键不是删掉队列和抢断，而是把它们限制成最小确定语义：一个当前动作、一个有界 FIFO 队列、一个抢断模式、一个 stop-to-idle；不增加上传、资产管理、profile 切换或通用 policy 平台。

## 3. 产品边界

### 3.1 用户

唯一目标用户是 LLM agent。

接口设计优先级：

1. 少接口。
2. 短字段。
3. 状态稳定。
4. 可高频轮询。
5. 错误能指导下一步动作。

不为人类操作台、网页 UI、长期资产管理或调度平台设计额外功能。

### 3.2 核心使用流

核心接口是 4 个 route path：`/health`、`/status`、`/execute`、`/stop`。其中 `/status?id=ID` 是 `/status` 的查询形态，不是额外 route。agent 高频核心动作是 `/status`、`/status?id=ID`、`/execute`、`/stop`；`/health` 仅用于 readiness 探活。

```text
GET  /health
GET  /status
POST /execute       # queue 或 interrupt
GET  /status?id=ID  # 查询指定动作
POST /stop          # 接受停止命令
```

顺序执行：

```bash
curl -s -X POST http://127.0.0.1:8080/execute \
  -H 'Content-Type: application/json' \
  -d '{"path":"/home/galbot/motions/a.trk","mode":"queue"}'
```

抢断执行：

```bash
curl -s -X POST http://127.0.0.1:8080/execute \
  -H 'Content-Type: application/json' \
  -d '{"path":"/home/galbot/motions/b.trk","mode":"interrupt"}'
```

终止并回 idle：

```bash
curl -s -X POST http://127.0.0.1:8080/stop
```

## 4. 与 ET1 App 的关系

### 4.1 只参考，不依赖

允许参考：

- Unitree DDS 连接方式。
- `LowCmd`、`LowState`、`HighState` 语义。
- `mode_machine` 真机/仿真判断。
- ET1TRK1 runtime cache 格式。
- GeneralTracker policy 所需观测字段。
- ONNX policy 推理和 joint PD 输出方式。
- 50 Hz tracker 执行节奏。

禁止：

- 修改 `unitree_rl_lab/deploy/robots/et1` 的现有 app。
- 在新 app 中 include ET1 app 的 `State_Track`、`State_Velocity`、`CtrlFSM` 等内部头文件。
- 链接 `et1_controller_lib` 或其他 ET1 app 内部 target。
- 把 HTTP 请求写入 ET1 app 的 request 文件。
- 运行 ET1 app 后再在外面包装控制。
- 运行时读取 `unitree_rl_lab/deploy/robots/et1/config/policy/...`。
- 把 ET1 app 的 policy/deploy 目录作为新 app 默认运行资产。

新 app 必须有自己的入口、配置、状态机、队列、HTTP server、轨迹 loader、状态快照和控制循环。

### 4.2 可共享的底层依赖

允许使用底层第三方或平台依赖：

- 新 app 本目录内的头文件和源文件。
- Unitree SDK2。
- ONNX Runtime。
- yaml-cpp。
- Eigen。
- 一个轻量 HTTP/JSON 实现，必须采用新 app 目录内私有 vendored/header-only 或 vendored source 方式。
- Catch2 v3，仅用于测试 target。
- 标准 C++ 库。

这些是平台依赖，不是 ET1 app 依赖。

允许复制思想，不允许复制依赖关系：

- 可以重新编写 DDS wrapper，但必须放在新 app 自己的目录。
- 可以重新实现 ET1TRK1 parser，但不能 include ET1 app 的 `State_Track.h`。
- 可以参考 policy observation 组织方式，但新 app 必须拥有自己的 observation/action 代码。
- 可以参考 `mode_machine==0` 的仿真判断，但新 app 必须自己实现检查。

### 4.3 `.trk` 与现有 `.et1trk`

ET1 参考实现中，非 `.npz` 文件会直接按 ET1TRK1 cache 读取；历史资产多用 `.et1trk` 后缀。

本服务的 API 合同仍然只接受 `.trk`：

- `.trk` 文件内容必须是 `agentic-et1-tracker` GeneralTracker GA schema 下的 ET1TRK1 v1 runtime cache。
- `.et1trk` 不作为 HTTP API 输入。
- 如果上游仍产出 `.et1trk`，应在服务外离线改名或复制为 `.trk`。
- 服务不做 `.npz -> .trk` 转换。
- 服务不做 `.et1trk -> .trk` 转换。
- 服务不承诺兼容所有历史 ET1TRK1 文件；ET1TRK1 magic 只是容器前提，数组合同以本 PRD 的 app-local `TrkSchema` 为准。

这样可以把 agent 侧输入收敛到一个短而稳定的概念，同时不误解底层格式边界。

### 4.4 GA profile 绑定

GA 只绑定一份 GeneralTrackerCLN frozen profile。ET1 `config/config.yaml` 里的 tracker 配置只是参考来源和兼容目标，不是运行时依赖。

冻结 profile 的最小合同：

- `policy_dir: config/policy/general_tracker_cln`
- 默认 `policy_file: multi_policy_v17c2_70k.onnx`
- `deploy_file: deploy.yaml`，即 `config/policy/general_tracker_cln/params/deploy.yaml`
- `fps: 50`
- `no_global_mode: true`
- `use_motion_root_command: true`
- `use_motion_velocity_command: true`

运行时资产必须由新 app 自己目录或外部部署资产目录提供，例如新 app 的 `config/policy/general_tracker_cln`。允许从 ET1 app 参考配置拷贝或导出一份 frozen profile，但交付物归 `agentic-et1-tracker` 管理，不从 `deploy/robots/et1` 运行时读取。GA 默认配置必须指向真实存在的 agentic-owned asset；如果部署包未携带或未挂载该 frozen profile，服务只能进入 not-ready/error，不能回退读取 ET1 app 的 policy/deploy tree。

`GeneralTrackerCJM` 虽然在当前 ET1 配置中复用了 `config/policy/general_tracker` 和旧 onnx/deploy 文件，但它是另一个 profile，不是本服务的 GA 绑定对象。`GeneralTrackerCLN` 使用 `config/policy/general_tracker_cln/params/deploy.yaml`，包含 `command_yaw`、`future_commands` 等观测合同。GA 不承诺兼容 CJM、legacy `GeneralTracker`、legacy `track`、dance 或其他历史 profile。

因此，HTTP 接收的 `.trk` 必须匹配本 app-local GeneralTracker GA schema；`ET1TRK1` 容器合法不等于可执行。

## 5. 架构

### 5.1 进程模型

新 app 是独立二进制：

```text
agentic-et1-tracker
  ├── AppRunner
  ├── AgentHttpServer
  ├── RuntimeBridge
  ├── MotionQueue
  ├── CommandMailbox
  ├── RuntimeControlLoop
  ├── TrackerController       # core/test façade only
  ├── TrkSchema
  ├── TrkValidator
  ├── TrkLoader
  ├── PolicyRuntime
  ├── RobotIO
  └── StatusSnapshot
```

新 app 目录：

```text
unitree_rl_lab/deploy/robots/agentic-et1-tracker/
```

这个目录拥有自己的 `main.cpp`、`CMakeLists.txt`、`config.yaml`、`include/` 和 `src/`。不得要求 ET1 app 为它改代码。

### 5.2 模块职责

`AgentHttpServer`：

- 处理 HTTP 请求。
- 解析 JSON。
- 返回短 JSON。
- 只通过 `RuntimeBridge` 提交命令或读取快照，不直接写机器人命令。

`AppRunner`：

- 负责启动顺序、配置加载、依赖 wiring 和生命周期管理。
- 生产运行时必须把 HTTP、`RuntimeBridge`、`CommandMailbox`、`MotionQueue`、`RuntimeControlLoop`、`PolicyRuntime` 和 `RobotIO` 接成同一套 app。
- 不承载产品语义，不增加 route。

`RuntimeBridge`：

- 是生产 HTTP adapter 与控制运行时之间的唯一边界。
- 为 accepted 的 `/execute` 和 `/stop` 分配单调递增 command sequence。
- 把 queue/interrupt/stop 写入 `MotionQueue` 或 `CommandMailbox`，并读取 `StatusSnapshot`。
- 不推进 reference，不运行 ONNX，不发布 `LowCmd`。

`MotionQueue`：

- 有界 FIFO。
- 保存 `.trk` 路径、短 `id`、状态和时间戳。
- 保存 request sequence，用于 stop watermark 判定。
- 不落盘。
- 只保存请求元数据，不缓存轨迹 payload。

`CommandMailbox`：

- 保存高优先级 stop/interrupt 命令。
- 保存 stop command sequence/watermark。
- 控制线程消费。

`RuntimeControlLoop`：

- 新 app 自己的控制状态机。
- 单活动执行。
- 在生产路径中负责 queue、interrupt、stop-to-idle。
- 生产路径中唯一能推进 reference、运行 policy、写 `LowCmd` 的模块。
- 消费 `CommandMailbox` 和 `MotionQueue`，并更新 `StatusSnapshot`。

`TrackerController`：

- 只作为 core/test façade 或状态机辅助层存在。
- 可在单线程 core 单测中同步覆盖状态机和队列行为。
- 不属于生产 HTTP adapter。
- 生产路径不得用同步 `TrackerController` façade 直接改运行时状态、推进 reference、运行 policy 或写 `LowCmd`。

`TrkValidator`：

- 入队前轻量校验 `.trk`。
- 使用 app-local `TrkSchema`。
- 只做 metadata/schema/bounds 校验，不读取 contact payload。
- 不构造完整轨迹对象。
- 不分配大 payload。

`TrkSchema`：

- 定义 ET1TRK1 header、dtype、数组名、shape 和上限。
- 是 `TrkValidator` 与 `TrkLoader` 的唯一 schema 来源。
- 不依赖 ET1 app 内部头文件或常量。

`TrkLoader`：

- 控制线程中加载已校验 `.trk`。
- 只支持 app-local ET1TRK1 v1 GeneralTracker GA schema runtime cache。
- 使用 app-local `TrkSchema`。
- 加载 payload 时校验 contact 值域；值域错误使当前动作 `failed`。

`PolicyRuntime`：

- 加载 ONNX 模型。
- 维护 policy history。
- 生成 action。
- 仅支持固定 GeneralTrackerCLN frozen profile 和 `config/policy/general_tracker_cln/params/deploy.yaml` 的 GA 最小合同，不做通用 policy 平台。

`RobotIO`：

- 管理 Unitree DDS。
- 读取 LowState/HighState。
- 发布 LowCmd。
- 判断 sim/real。
- 对 lowcmd 通道占用做 best-effort 检测。
- 检查 `mode_machine`。

`StatusSnapshot`：

- 控制线程更新。
- HTTP 线程只读。

### 5.3 线程原则

HTTP 服务可以并发处理请求，但 HTTP 线程不能直接控制机器人。

HTTP 线程只做：

- 解析 JSON。
- 校验 `.trk` 路径。
- 生成短 `id`。
- 通过 `RuntimeBridge` 写入 `MotionQueue` 或 `CommandMailbox`。
- 读取 `StatusSnapshot`。
- 返回响应。

控制线程只做：

- 消费 stop/interrupt 命令。
- 消费队列中的下一个 `.trk`。
- 加载 reference。
- 执行 ONNX policy。
- 写 LowCmd。
- 更新状态快照。

允许为了 core 单测提供一个纯 core façade，在单线程测试中同步调用控制器方法以覆盖状态机和队列行为。该 façade 不属于生产 HTTP adapter，不得发布 LowCmd，也不得让 TDD 锁死生产并发模型。生产 HTTP adapter 必须经过 `RuntimeBridge`、`CommandMailbox`、`MotionQueue` 和 `RuntimeControlLoop`；只有控制循环 tick 可以改变控制状态、推进 reference、运行 policy 和写 `LowCmd`。

请求被 accepted 后、控制 tick 消费前，`/status?id=ID` 必须已经能查询到该动作的 `queued` 元数据。已 accepted 但尚未被控制 tick 消费的动作若被 `/stop` 或 `interrupt` 清空，`/status?id=ID` 必须能查询到 `canceled` 和对应动作级 `stop_reason`。

### 5.4 命令优先级

控制线程按固定优先级消费命令：

1. `stop`
2. `interrupt`
3. `queue`

规则：

- `/stop` 是幂等的；无 active 时也返回成功。
- `RuntimeBridge` 为每个 accepted command 分配单调递增 sequence；stop 命令携带自己的 stop sequence，作为本次 stop watermark。
- `/stop` 只取消控制线程消费该 stop 命令时已经存在的 active，以及 sequence `<= stop_sequence` 且仍 queued/pending 的动作。
- sequence `> stop_sequence` 的新 `queue` 或 `interrupt` 是 stop 之后 accepted 的新工作，不属于本次 stop 的取消集合，也不应被本次 stop 再取消。
- `/stop` 处理中收到新的 `queue` 请求时，新请求可以入队，但不能在 stop-to-idle 完成前启动。
- `/stop` 处理中收到新的 `interrupt` 请求时，新动作成为 stop-to-idle 后的待执行动作；当前 stopping 继续完成。
- `stop` 触发 `ctrl:"stopping"` 时，顶层 `stop_reason:"stop"`。
- `interrupt` 触发新的 `ctrl:"stopping"` 时，顶层 `stop_reason:"interrupt"`。
- 如果已经处于 `ctrl:"stopping"`，后续 `interrupt` 不改写当前 stopping 的顶层 `stop_reason`；它只替换等待队列队首并清空旧等待。
- 被 `/stop`、`interrupt` 或替换清空的已接受 queued 动作进入 `canceled`，保留在 recent ring buffer 中供 `/status?id` 查询。
- `interrupt` 不取消已经完成的 recent 记录。
- 控制线程消费不是严格到达顺序，而是全局优先级 `stop > interrupt > queue`；同一优先级内再按进入 `CommandMailbox`/`MotionQueue` 的 sequence 排序。
- 不需要复杂事务系统；单进程内的 command sequence/watermark 足够定义取消集合。

### 5.5 控制状态机

新 app 必须实现自己的最小状态机：

| 状态 | 含义 | 允许进入条件 | 退出条件 |
| --- | --- | --- | --- |
| `starting` | 初始化 DDS、policy、配置 | 进程启动 | 初始化成功或失败 |
| `idle` | 可执行 tracker idle | ready 且无 active | 有 queued/interrupt |
| `preparing` | 加载 `.trk`、reset policy history | 取到下一个动作 | 加载成功或失败 |
| `running` | 正在执行 active | `preparing` 成功 | done/stop/interrupt/fault |
| `stopping` | 受控停止并回 idle | stop 或 interrupt | idle 或 fault |
| `fault` | 安全阻塞 | safety/model/robot 错误 | 人工或重启恢复 |

HTTP 状态里的 `ctrl` 只能来自这个状态机，不暴露 ET1 参考 app 的 FSM 名称。

`stopping` 同时覆盖用户 stop 和 interrupt 两种停止来源。HTTP 不暴露单独的抢断状态；来源只通过 `stop_reason:"stop"|"interrupt"` 表示。`ctrl` 离开 `stopping` 后，顶层 `stop_reason` 恢复为 `null`，recent 动作可以保留自己的停止或取消原因。若 stopping 期间接受新的 interrupt 作为待执行动作，当前顶层 `stop_reason` 保持进入 stopping 时的原因，不被后来的待执行请求改写。

### 5.6 构建隔离

新 app 的 CMake 必须比现有 ET1 app 更硬隔离：

- 禁止 blanket include `${PROJECT_SOURCE_DIR}/../../include`。
- 禁止 include `deploy/include/FSM/*` 或任何 ET1 app 内部目录。
- 禁止链接 `et1_controller_lib` 或任何 ET1 app 内部 target。
- 禁止使用全局 `include_directories()` 和全局 `link_libraries()`。
- 所有 include/link 必须通过 `target_include_directories()`、`target_link_libraries()` 绑定到新 app 自己的 target。
- 允许的 include/link 来源仅限本目录头、Unitree SDK2、ONNX Runtime、yaml-cpp、Eigen、Catch2 v3、私有 vendored HTTP/JSON 和标准 C++ 库。

如果需要 FSM、observation 或 action 的能力，必须在新 app 本目录内实现最小等价代码，不能通过 include 上层 `deploy/include` 复用。

### 5.7 测试构建

core TDD 使用 Catch2 v3 + CTest。

CMake 选项：

- `AGENTIC_ET1_BUILD_TESTS=ON`，默认打开。
- `AGENTIC_ET1_BUILD_ROBOT=OFF`，默认关闭。
- `AGENTIC_ET1_BUILD_ONNX=OFF`，默认关闭。
- `AGENTIC_ET1_BUILD_PERF_SMOKE=OFF`，默认关闭。

默认 core tests 不依赖 MuJoCo、Unitree SDK2 或 ONNX Runtime。queue、状态机、gate、status/progress、`TrkSchema`、`TrkValidator` 这类核心逻辑必须能在默认选项下用 fake/stub 跑通。ONNX tests 随 `AGENTIC_ET1_BUILD_ONNX=ON` 进入构建；Unitree SDK robot tests 随 `AGENTIC_ET1_BUILD_ROBOT=ON` 进入构建；real/non-stub runtime factory 只有 `AGENTIC_ET1_BUILD_ONNX=ON` 且 `AGENTIC_ET1_BUILD_ROBOT=ON` 时启用；perf smoke 单独由 `AGENTIC_ET1_BUILD_PERF_SMOKE=ON` opt-in。

手动/集成测试可使用当前 worktree 中的 Unitree MuJoCo 仿真环境：MuJoCo simulator installation under `/home/galbot/works/et1`，sample/generated `.trk` under `/home/galbot/works/et1/generated/`。这只是仿真验收环境信息，不改变默认 core tests 不依赖 MuJoCo/Unitree SDK2/ONNX Runtime 的要求；不得为此复制或移动 `.trk` 或资产。

## 6. 执行模型

### 6.1 最小队列模型

GA 使用有界内存 FIFO 队列：

- 一个 `active`。
- 一个 `queue`。
- 一个 recent ring buffer，用于短时间内按 `id` 查询完成结果。
- 队列不落盘。
- 服务重启后队列丢失。
- 队列只保存 `.trk` 路径和执行状态，不保存动作文件内容。

默认队列长度为 `8`。队列满时返回 `QUEUE_FULL`。

`MotionRequest` 最小字段：

```json
{
  "id": "a7K3p9Qx",
  "path": "/home/galbot/motions/a.trk",
  "state": "queued",
  "frame": 0,
  "frames": 300,
  "err": null
}
```

实现中还应记录 `enqueued_at`、`started_at`、`ended_at`，但默认 API 不返回这些时间戳，避免状态过长。

### 6.2 执行模式

`POST /execute` 支持两个模式：

- `queue`：加入 FIFO 队列。若当前空闲，则很快成为 active。
- `interrupt`：清空等待队列，请求当前 active 受控停止，并把新动作放到队首。

抢断语义必须简单确定：

1. 请求被立即接受；等待队列中已接受但未执行的旧动作进入 `canceled`。
2. 如果当前 active 正在 `running`，控制器进入 `ctrl:"stopping"`，顶层 `stop_reason:"interrupt"`。
3. 新动作进入 `queued`，排在队首。
4. 控制器完成受控 stop-to-idle。
5. stop-to-idle 完成且 start gate 满足后，新动作才可开始 `running`。

如果请求发生在 `ctrl:"stopping"` 期间，当前 stopping 原因保持不变；新 interrupt 只替换 stop-to-idle 后的待执行队首，并把被替换或清空的 queued 动作标记为 `canceled`。`interrupt` 不能在真机中直接硬切 reference。

`/execute` 分成 accept gate 和 start gate，避免队列接收与真正启动混在一起。

accept gate 在 HTTP 线程中执行，只决定请求能否进入执行系统。必须全部满足：

- service 已完成初始化。
- Unitree lowstate 已连接且新鲜。
- `mode_machine` 检查通过。
- policy model 和 deploy 配置已加载。
- 当前不处于 `fault`。
- `path` 是 absolute local path，canonical 后通过 allowlist，存在、可读、扩展名为 `.trk`。
- `.trk` 通过 `TrkValidator` 的 app-local ET1TRK1 v1 GeneralTracker GA schema 校验。
- `TrkValidator`/HTTP accept gate 只做 metadata、schema 和 bounds 轻量校验，不读取 contact payload；metadata 合法但 contact 值域错误的 `.trk` 可被 accepted。
- 对 `mode:"queue"`，队列未满。

`ctrl:"running"` 和 `ctrl:"stopping"` 本身不是 accept gate 的拒绝理由。`stopping` 期间允许接受合法 `queue` 或 `interrupt` 请求；它们只能排队或更新待执行头部，不能直接启动。

accept gate 不满足时返回 `SERVICE_NOT_READY`、`ROBOT_DISCONNECTED`、`ROBOT_NOT_READY`、`MODEL_NOT_READY`、`TRK_*` 或 `QUEUE_FULL`，且不入队。`fault` 或 `ready=false` 的 not-ready 场景不入队，agent 应先轮询 `/status` 并处理 `block`。

start gate 在控制线程中执行，只决定队列头动作何时真正开始。必须满足：

- stop-to-idle 已完成。
- `ctrl:"idle"`。
- robot、`mode_machine`、policy 仍 ready。
- 当前不处于 `fault`。

start gate 不满足时不得推进 reference、不得加载新 `.trk`、不得发布新动作的 LowCmd。已接受请求保持 `queued`，直到 start gate 满足，或被后续 `/stop`、`interrupt`、`fault` 语义处理。

### 6.3 Stop-to-idle

`POST /stop` 是基础能力，语义固定：

1. HTTP adapter 通过 `RuntimeBridge` 写入带 sequence 的 stop 命令并立即返回 accepted，不等待控制线程消费 ack。
2. stop 命令由控制线程消费时，以 stop command sequence 作为 watermark 建立本次 stop 的取消集合。
3. 停止控制线程消费 stop 时已经存在的 active。
4. 清空 sequence `<= stop_sequence` 且仍 queued/pending 的动作，并标记为 `canceled`。
5. 进入 `ctrl:"stopping"`，`stop_reason:"stop"`。
6. 健康可控路径下，控制器回到 tracker idle 控制态。
7. agent 通过后续 `/status` 和 `/status?id=ID` 查询停止、取消和失败结果。

`/stop` 是软件控制停止，不等价于硬件急停。

`/stop` 与 `/execute` 不同：即使 `ready=false`、机器人断连或已经处于 `fault`，也必须尽可能返回成功的可理解状态。无法发布 LowCmd 时，不做硬件承诺；`/status` 必须返回明确 `block`/`err`，不承诺回到 `ctrl:"idle"`。

`POST /stop` 响应不返回精确取消数量，也不承诺控制线程已经消费 stop。取消集合、active 最终 `stopped/failed`、queued 最终 `canceled` 等结果只通过后续 `/status` 和 `/status?id=ID` 体现。stop 后 accepted 的新 `queue/interrupt` sequence 大于该 stop watermark，不属于本次取消集合；若调用方在 stop 后继续提交新工作，`queue.n > 0` 不表示本次 stop 未收敛。

这里的 `idle` 是新 app 自己实现的 tracker idle 控制态，不承诺物理已静止或执行质量。GA 采用保守实现：

- 停止推进 reference。
- 若 `RobotIO` 仍可控，控制循环在每个 tick 发布当前安全关节目标的 hold `LowCmd`，持续时间由 `stop_hold_s` 配置。
- 之后进入配置的 tracker idle。
- 不进入硬件断力。
- 不承诺从 `Passive` 自动站起。
- 若机器人断连、DDS 不可写或进入 fault，`stop_hold_s` 不构成物理保持承诺；状态必须暴露明确 `block`/`err`。

### 6.4 Policy/Observation 合同

GA 只支持固定 GeneralTrackerCLN frozen profile 和 `config/policy/general_tracker_cln/params/deploy.yaml` 的最小运行合同，不扩展成通用 policy 平台。

固定绑定：

- profile：GeneralTrackerCLN frozen profile
- policy 目录：新 app 自有 `unitree_rl_lab/deploy/robots/agentic-et1-tracker/config/policy/general_tracker_cln` 或外部部署资产目录
- 默认模型：`multi_policy_v17c2_70k.onnx`
- deploy 配置：`config/policy/general_tracker_cln/params/deploy.yaml`
- motion：`.trk` 内容必须匹配本 app-local GeneralTracker GA `TrkSchema`

禁止把 `unitree_rl_lab/deploy/robots/et1/config/policy/...` 作为运行时 `policy_dir` 或默认 deploy/model 来源。允许拷贝或导出一份 frozen profile 到新 app 的 `config/policy/general_tracker_cln` 或部署资产目录；manifest/hash 只用于 release/package 检查和部署审计，不是 runtime fallback、远程下载或 ET1 路径解析机制。GA 交付时默认 `policy_dir`、ONNX 和 deploy YAML 必须解析到 agentic-owned 真实文件。

必须支持并校验：

- `observations.obs_current`。
- `observations.obs_history`。
- command/reference obs：`command_root_ori_b`、`command_xy_yaw_vel`、`command_jnt_pos`、`command_foot_support_state`、`ref_com_rel_navi`、`ref_com_vel_navi`。
- live robot obs：`projected_gravity`、`base_ang_vel`、`joint_pos_rel`、`joint_vel_rel`、`last_action`。
- 每个 obs 的 `history_length`，尤其 `obs_history` 的历史长度。
- `actions.JointPositionAction.scale` 和 `actions.JointPositionAction.offset`。
- `default_joint_pos` 长度 26、finite；`joint_pos_rel` 必须按 policy joint order 计算为 `live_joint_pos - default_joint_pos`；不得用零向量、`JointPositionAction.offset` 或 SDK order joint position 代替。
- `joint_ids_map` 和 `sdk_joint_ids_map`。
- `policy_kp` 和 `policy_kd`。

启动时必须检查 deploy 配置、ONNX 输入输出维度、action 维度、joint map、default_joint_pos、kp/kd 长度一致。不一致进入 not-ready，不接受 `/execute`。GA 不支持运行时切换任意 policy 类型、自动推断未知 observation、profile registry 或多 policy 编排。

Reference/Observation 派生最小公式合同：

- `.trk` reference 只使用 body index `0` 作为 root；quat order 为 `wxyz`，实现必须 normalize，非法或零范数在控制线程加载或执行时使动作 `failed`。
- 控制 tick 使用 `frame = clamp(round(elapsed_s * fps), 0, F - 1)`；GA 不做 seek/pause/resume/loop。
- action start 时固定两项 yaw bias：首帧 reference root yaw、进入动作时 live robot root yaw；`no_global_mode=true` 下 reference 和 live robot root 都按各自初始 yaw 去全局化。
- `command_root_ori_b = R(q_robot.conjugate() * q_ref)` 的前两列，flatten 顺序必须是 `[R00,R01,R10,R11,R20,R21]`。
- `command_xy_yaw_vel`：reference root linear/angular velocity 先做 no-global yaw align，再转到 current reference yaw navigation frame，输出 `[vx, vy, wz]`。
- `command_jnt_pos = joint_pos[frame]`，policy order 26 维。
- foot contact `0..2` 转 6 维 one-hot，左脚 `[0..2]`，右脚 `[3..5]`；不得 clamp 非法值。
- `ref_com_rel_navi`、`ref_com_vel_navi` 从 `.trk` 当前帧原样复制。
- live obs：`projected_gravity = q_robot.conjugate() * (0,0,-1)`，`base_ang_vel` 来自 LowState IMU gyro，关节 q/dq 用 `sdk_joint_ids_map` 转 policy order；`joint_pos_rel = q - default_joint_pos`，`joint_vel_rel = dq`，`last_action` 是上一 tick raw policy action。
- history 按 deploy `obs_history` 合同 oldest -> newest flatten；`obs_current` 按 deploy `obs_current` 顺序拼接。

明确不支持：

- `GeneralTrackerCLN` 的 `command_yaw`、`future_commands` schema。
- `GeneralTrackerCJM` profile 语义。
- legacy `track` deploy。
- dance/nohead 等其他 tracker profile。

## 7. API

所有响应都是 JSON。除 `/health` readiness 探活外，成功处理的请求包含 `ok:true`，失败响应包含 `ok:false`。`/health` 在 not-ready 时也可以返回 HTTP 200 和 `ok:false`，表示进程可达但尚未 ready。

字段名尽量短，降低 agent 高频读取成本。

### 7.1 `GET /health`

用途：readiness 探活。

响应：

```json
{
  "ok": true,
  "state": "ready",
  "mode": "sim"
}
```

字段：

- `state`: `starting|ready|error`
- `mode`: `sim|real|unknown`

ready 响应必须是 `ok:true,state:"ready"`。starting 或 error/not-ready 响应可以是 `ok:false,state:"starting"` 或 `ok:false,state:"error"`。`/health` 只做 readiness 探活和粗状态，不承载详细原因；`state:"ready"` 与 `/status.ready=true` 对齐，详细阻塞原因只从 `/status` 获取。

### 7.2 `GET /status`

用途：agent 高频轮询的主要状态接口。

响应：

```json
{
  "ok": true,
  "mode": "sim",
  "ready": true,
  "robot": "running",
  "ctrl": "running",
  "stop_reason": null,
  "hz": 50,
  "exec": {
    "id": "a7K3p9Qx",
    "state": "running",
    "frame": 128,
    "frames": 300,
    "time_s": 2.56,
    "duration_s": 6.0,
    "progress": 0.43
  },
  "queue": {
    "n": 2,
    "limit": 8,
    "ids": ["b8Lm2sV1", "c2Qr9pAa"]
  },
  "low_ms": 4,
  "block": null,
  "err": null
}
```

空闲响应：

```json
{
  "ok": true,
  "mode": "real",
  "ready": true,
  "robot": "idle",
  "ctrl": "idle",
  "stop_reason": null,
  "hz": 50,
  "exec": null,
  "queue": {"n": 0, "limit": 8, "ids": []},
  "low_ms": 4,
  "block": null,
  "err": null
}
```

`robot`：

- `disconnected`
- `not_ready`
- `idle`
- `running`
- `holding`
- `fault`

`robot` 表示连接、健康和本 app 的控制输出意图，不表示硬件确认状态、物理已静止或动作执行质量。`robot:"idle"` 只表示 app 当前意图是 idle 输出；agent 判断 `/stop` 收敛不应依赖它。`robot:"holding"` 只表示 app 正在 stop hold 阶段输出 hold 控制意图，不表示硬件确认进入保持状态。

`ctrl`：

- `starting`
- `idle`
- `preparing`
- `running`
- `stopping`
- `fault`

`stop_reason`：

- `null`
- `stop`
- `interrupt`

`stop_reason` 只在顶层 `ctrl:"stopping"` 时表示本次停止来源；`ctrl` 不暴露单独的抢断状态。queued 动作被 `/stop`、`interrupt` 或替换取消时，动作状态为 `canceled`，动作自己的 `stop_reason` 可以是 `stop` 或 `interrupt`，并可通过 `/status?id=ID` 查询；顶层 `stop_reason` 在 `ctrl` 非 `stopping` 时必须为 `null`。

`exec.state`：

- `queued`
- `running`
- `stopping`
- `done`
- `stopped`
- `canceled`
- `failed`

默认状态不返回完整关节数组。高频状态要保持短。

默认 `/status` 永不返回 `exec.path`，只返回 `time_s`、`duration_s` 这类短字段。`path` 只在 `/status?id=ID` 返回；安全部署可以关闭 `/status?id=ID` 的 path 暴露，但不能让 `/status` 响应形态随配置变化。

`err` 的形状固定为 `null` 或短 `ErrorObject`，只包含 `code/message/retryable`：

```json
{"code":"MODEL_INFERENCE_FAILED","message":"short reason","retryable":false}
```

`block` 是当前阻塞状态的短字符串，`err` 是最近一次明确错误；二者都不承载长诊断。

### 7.3 `GET /status?id=ID`

用途：按动作 `id` 查询单个动作状态。

响应：

```json
{
  "ok": true,
  "id": "a7K3p9Qx",
  "state": "running",
  "path": "/home/galbot/motions/a.trk",
  "frame": 128,
  "frames": 300,
  "time_s": 2.56,
  "duration_s": 6.0,
  "progress": 0.43,
  "stop_reason": null,
  "robot": "running",
  "err": null
}
```

说明：

- active 动作返回实时进度。
- queued 动作返回 `state:"queued"`，`frame` 可为 `0`。
- 完成动作从 recent ring buffer 返回。
- stopped 或 canceled 动作可返回动作自己的 `stop_reason:"stop"|"interrupt"`；其他状态返回 `null`。
- 找不到 `id` 返回 `404 RUN_NOT_FOUND`。

### 7.4 `POST /execute`

用途：提交一个 `.trk` 路径异步执行。

请求：

```json
{
  "path": "/home/galbot/motions/a.trk",
  "mode": "queue"
}
```

字段：

- `path`：必填，本机 `.trk` 绝对路径。
- `mode`：可选，`queue|interrupt`，默认 `queue`。

响应：

```json
{
  "ok": true,
  "id": "a7K3p9Qx",
  "state": "queued",
  "q": 1
}
```

要求：

- 必须异步返回。
- `id` 使用 8 到 10 位 base62 字符。
- `id` 只需在当前服务进程内无冲突。
- 不接收文件上传。
- 不接收非路径输入。
- 不接收 URL、相对路径或空路径。
- 不接收非 `.trk` 文件。
- `.trk` 必须能被 app-local ET1TRK1 v1 loader 直接读取，并满足 GeneralTracker GA `TrkSchema`。
- 接受请求只代表已进入执行系统，不代表已经开始运动。
- 响应中的 `q` 是本请求 accepted 后等待队列里的动作数量，包含本请求；不包含 active 和 recent。
- accepted 记录保存 canonical path，不保存用户原始 path 作为执行依据。

`mode:"queue"`：

- 若 controller 空闲且 start gate 满足，动作进入队列头，很快开始执行。
- 若已有 active 或 `ctrl:"stopping"`，动作排在队尾。
- 若队列满，返回 `QUEUE_FULL`。
- 若 accept gate 不满足，返回错误，不入队。

`mode:"interrupt"`：

- 立即接受抢断请求，取消旧等待，并触发受控 stop-to-idle。
- 如果当前 active 正在 `running`，状态表现为 `ctrl:"stopping"`、`stop_reason:"interrupt"`。
- 清空等待队列。
- 新动作排到队首。
- 若请求发生在 `ctrl:"stopping"` 期间，新动作替换等待中的旧队首，清空其他等待动作，当前顶层 `stop_reason` 不变，且仍必须等待 stop-to-idle 完成。
- 返回新动作 `id`。

### 7.5 `POST /stop`

用途：请求停止本次已有工作；响应只表示 stop 命令已 accepted。

请求体可省略。

响应：

```json
{
  "ok": true,
  "state": "accepted"
}
```

语义：

- `/stop` 响应不等待控制线程消费 stop，也不返回精确取消数量。
- stop 命令带有 accepted 时分配的 command sequence；控制线程消费该 stop 时，用该 sequence 作为本次 stop watermark。
- stop 命令被控制线程消费时，停止当时存在的 active。
- 清空 sequence `<= stop_sequence` 且仍 queued/pending 的 queue 项。
- 被清空的 queued/pending 动作标记为 `canceled`，动作自己的 `stop_reason:"stop"`。
- sequence `> stop_sequence` 的新 `queue/interrupt` 不属于该 stop；它们只能在 stop-to-idle 完成后按自身语义继续。

```text
stop accepted: sequence=10
本次 stop 只取消 sequence <= 10 且仍 queued/pending 的动作
stop 期间新 queue accepted: sequence=11，不属于本次 stop 取消集合
```

- 被停止的 active 动作最终变为 `stopped`；若停止期间发生 fault，则变为 `failed` 并返回明确 `err`。
- 健康可控路径下控制器目标状态为 `ctrl:"idle"`。
- 如果没有 active，HTTP 也返回成功；控制线程消费 stop 时仍按 watermark 清空旧 queued/pending。
- 取消和停止结果通过后续 `/status` 和 `/status?id=ID` 查询体现。

agent 调用 `/stop` 后应轮询 `/status`，直到：

- 健康可控路径：`ctrl:"idle"`，且本次 stop 涉及的 active 不再是 `running/stopping`。
- 若调用方 stop 后没有提交新工作，通常还应看到 `exec:null`、`queue.n == 0`。
- 若 stop 后又有新 `queue/interrupt` 被接受，`queue.n > 0` 或后续新 `exec` 不表示本次 stop 失败。

如果机器人已断连或 `fault` 无法恢复，`/status` 必须返回明确的 `robot`/`block`/`err`，而不是让 agent 一直等待 `idle`。

## 8. 错误模型

错误响应统一格式：

```json
{
  "ok": false,
  "error": {
    "code": "QUEUE_FULL",
    "message": "motion queue is full",
    "retryable": true
  },
  "next": "status"
}
```

`error` 与状态里的 `err` 使用同一个短 `ErrorObject` 形状：`code/message/retryable`。`message` 只放短原因，不放长诊断、堆栈或多行上下文。

`next`：

- `status`：查询状态再决定。
- `retry`：可重试同一请求。
- `wait_robot`：等待机器人 ready。
- `fix`：修正请求或文件。
- `stop`：先停止当前动作。
- `manual`：需要人工处理。

GA 错误码：

- `REQUEST_INVALID`
- `SERVICE_NOT_READY`
- `ROBOT_DISCONNECTED`
- `ROBOT_NOT_READY`
- `ROBOT_BAD_ORIENTATION`
- `MODEL_NOT_READY`
- `MODEL_INFERENCE_FAILED`
- `TRK_PATH_NOT_ALLOWED`
- `TRK_FILE_NOT_FOUND`
- `TRK_PARSE_FAILED`
- `TRK_VALIDATION_FAILED`
- `QUEUE_FULL`
- `RUN_NOT_FOUND`
- `SAFETY_LIMIT_TRIGGERED`
- `INTERNAL_ERROR`

错误码必须稳定。agent 逻辑只依赖 `code` 和 `next`。

`QUEUE_FULL` 固定返回 `next:"status"`。`retryable` 可以为 `true`，但 agent 下一步应先查询 `/status` 判断 active、queue 和 stop 状态，再决定是否重试。

### 8.1 事件 x 状态/错误矩阵

最小可测映射：

| 事件 | 条件 | HTTP 结果 | 状态变化 | 错误/阻塞 |
| --- | --- | --- | --- | --- |
| `queue` | accept gate 通过、队列未满 | `ok:true,state:"queued"` | 新动作 queued | `err:null` |
| `queue` | 队列满 | `ok:false` | 不入队 | `QUEUE_FULL,next:"status"` |
| `interrupt` | accept gate 通过 | `ok:true,state:"queued"` | 旧 queued `canceled`；running 则 `ctrl:"stopping"` | 顶层 `stop_reason:"interrupt"`，已在 stopping 则保持原原因 |
| `stop` | 任意非崩溃状态 | `ok:true,state:"accepted"` | 消费时已有 active/queued 停止或取消；健康路径进入/保持 `stopping` 后回 `idle` | 无法控制时给 `block/err`，不承诺 `idle` |
| loader fail | 控制线程加载失败 | 请求已 accepted | 当前动作 `failed`，进入 `idle` 或 `fault` | `TRK_PARSE_FAILED` 或 `TRK_VALIDATION_FAILED` |
| invalid contact payload | metadata/schema/bounds 合法，contact payload 值域非法 | 请求可 accepted | `TrkLoader` 加载失败，当前动作 `failed` | `TRK_VALIDATION_FAILED` 或 loader validation failure |
| inference fail | ONNX 推理失败 | 请求已 accepted | 当前动作 `failed`，`ctrl:"fault"` | `MODEL_INFERENCE_FAILED,next:"manual"` |
| robot disconnect | lowstate 超时/断连 | `/execute` 拒绝 | `ready:false`；active failed 或 stopping blocked | `ROBOT_DISCONNECTED,block:"lowstate_timeout"` |
| bad orientation | 姿态 gate 触发 | `/execute` 拒绝或运行中失败 | `ctrl:"fault"` | `ROBOT_BAD_ORIENTATION,block:"bad_orientation"` |
| fault | safety/model/robot fatal | `/execute` 拒绝 | `ctrl:"fault"` | 对应 `err.code`，`next:"manual"` |

## 9. `.trk` 校验

入队前必须做轻量校验，避免 metadata、schema 或 bounds 无效的文件进入控制路径：

- `path` 是 absolute local path；URL、相对路径、空路径直接拒绝。
- `path` 存在。
- canonical path 在允许目录内。
- symlink canonical 后逃逸 allowlist 时拒绝。
- 扩展名为 `.trk`。
- 文件可读。
- magic 为 `ET1TRK1`。
- cache version 必须等于当前 app-local schema version：ET1TRK1 v1。
- 总帧数大于 0。
- 总时长在配置上限内。

不做其他格式兼容。不是 `.trk` 就直接拒绝；ET1TRK1 高版本或非 v1 文件直接拒绝，避免把未知 schema 当作可执行轨迹。

### 9.1 App-local `TrkSchema`

新 app 必须定义自己的 `TrkSchema`，作为 validator 和 loader 的唯一 schema 来源。禁止 validator 与 loader 各自复制一份 shape/dtype 常量。

`TrkSchema` 固化 `agentic-et1-tracker` GeneralTracker GA schema 的 ET1TRK1 v1 最小合同：

- 所有整数和浮点 payload 均为 little-endian；GA 不做跨 endian 自动转换。
- header：`magic[8]`、`version:uint32`、`array_count:uint32`，其中 `magic == {'E','T','1','T','R','K','1','\0'}`，`version == 1`。
- dtype enum：`Float32=1`、`Float64=2`、`Bool=3`、`Int32=4`、`Int64=5`、`UInt8=6`、`Int8=7`。
- `joint_dim=26`。
- `body_count=27`。
- `fps` 来自配置，默认 `50`。

wire format 必须对齐现有 ET1TRK1 v1、`scripts/et1/convert_track_npz.py` 和 ET1 `State_Track` runtime cache：不更换 magic/version，不引入新 converter。这里只做格式对齐；新 app 不 include/link ET1 app，也不在运行时读取 ET1 app 资产。

每个 array 连续编码为：

```text
name_len:uint32
name:name_len bytes      # 非空，不含 NUL，不要求 NUL 结尾
dtype:uint32
ndim:uint32
shape:uint32[ndim]
byte_count:uint64
payload:byte_count bytes
```

数组之间无 padding、无 alignment、无额外目录表。payload 按 C-order flat layout 存储，最后一维连续；`byte_count` 必须等于 `prod(shape) * dtype_size`。shape 的 wire field 是 `uint32[ndim]`；validator/loader 内部做 dims 乘积、`element_count * item_size` 和 offset 计算时必须提升到 `uint64` 或更宽整数，防止溢出。

必须数组和 shape，`F` 为 `frame_count`：

| array | dtype | shape |
| --- | --- | --- |
| `joint_pos` | `Float32` 或 `Float64` | `[F, 26]` |
| `joint_vel` | `Float32` 或 `Float64` | `[F, 26]` |
| `body_pos_w` | `Float32` 或 `Float64` | `[F, 27, 3]` |
| `body_quat_w` | `Float32` 或 `Float64` | `[F, 27, 4]` |
| `body_lin_vel_w` | `Float32` 或 `Float64` | `[F, 27, 3]` |
| `body_ang_vel_w` | `Float32` 或 `Float64` | `[F, 27, 3]` |
| `left_foot_contact_state` | `Int64`、`Int32`、`UInt8` 或 `Int8` | `[F]` |
| `right_foot_contact_state` | `Int64`、`Int32`、`UInt8` 或 `Int8` | `[F]` |
| `ref_com_rel_navi` | `Float32` 或 `Float64` | `[F, 3]` |
| `ref_com_vel_navi` | `Float32` 或 `Float64` | `[F, 3]` |

`F` 由 `joint_pos.shape[0]` 确定。所有必须数组的第一维必须等于同一个 `F`，且 `F > 0`。`duration_s = (F - 1) / fps`，不得超过 `max_track_duration_s`。

### 9.2 Parser 上限与 payload 规则

validator 和 loader 都必须检查以下上限：

- `array_count <= 64`。
- `name_len <= 128`。
- `ndim <= 4`。
- 单数组 `byte_count <= 256 MiB`。
- 总 payload `byte_count` 累计不得超过配置上限，默认 `512 MiB`。
- dims 乘积、`element_count * item_size`、文件 offset 计算不得整数溢出。
- `byte_count` 必须等于 `element_count * dtype_size`。
- payload 区间必须落在文件实际大小内。

payload 处理规则：

- `TrkValidator` 只扫描 header、数组 metadata 和 payload offset，不读取 payload 到内存。
- HTTP accept gate 不读取 contact payload；contact 值域错误不作为 HTTP 拒绝条件。
- `TrkValidator` 对未知数组只要 metadata 合法，就按 `byte_count` seek 跳过。
- `TrkLoader` 只读取必须数组的 payload。
- `TrkLoader` 对未知数组必须按 `byte_count` seek 跳过，不能分配 unknown payload。
- 未知 dtype 直接拒绝，即使该数组不是必须数组。
- 必须数组 dtype 不在允许集合内时直接拒绝。
- 必须数组缺失、重复、shape 不匹配或 frame_count 不一致时直接拒绝。
- 未知数组允许重复；必须数组不允许重复。
- contact payload 加载时每个值必须是 `0`、`1` 或 `2`，否则 `TrkLoader` 加载失败并把当前动作标记为 `failed`，错误为 `TRK_VALIDATION_FAILED` 或 loader validation failure。每只脚有 3 类 contact/support 状态；控制侧按固定 `GeneralTracker` 观测合同把左右脚状态生成 6 维 `command_foot_support_state`。值域外数据不能被 clamp 或映射为默认值。

校验实现必须使用轻量 header parser，不应在 HTTP 线程构造完整 loader。原因：

- 完整 loader 会读取 payload 并分配数组内存。
- `.npz` 不能进入在线执行路径。
- 轨迹异常不能导致控制进程崩溃。

控制线程仍必须捕获 loader 和 ONNX 推理异常，并把当前动作标记为 `failed`。

### 9.3 路径安全与 TOCTOU

路径合同：

- `POST /execute` 只接受 absolute local path。
- URL、相对路径、空路径、非 `.trk` 后缀直接拒绝。
- canonical 后必须位于配置 allowlist 目录内。
- symlink 指向 allowlist 外时返回 `TRK_PATH_NOT_ALLOWED` 或等价 symlink escape 错误。
- accepted 记录保存 canonical path；后续执行不使用用户原始 path。

TOCTOU 处理：

- HTTP accept gate 可以做一次轻量 scan，但只代表当时可接受。
- 控制线程加载前必须对保存的 canonical path 重新做 allowlist、存在、可读、schema 校验。
- scanner/loader 必须基于同一个已打开文件句柄完成 scan 和 payload read；不能 scan 一个 path 后关闭，再重新打开同名 path 读 payload。
- 如果文件在两次校验之间被替换，控制线程以重新打开并已扫描的那个文件内容为准。
- 如果控制线程重校验失败，动作进入 `failed`，返回 `TRK_PARSE_FAILED`、`TRK_VALIDATION_FAILED` 或 `TRK_PATH_NOT_ALLOWED`。
- 不引入文件上传、文件复制或服务端缓存 payload。

## 10. 状态语义

### 10.1 `ready`

`ready=true` 表示：

- Unitree lowstate 已连接。
- lowstate 新鲜度在阈值内。
- `mode_machine` 检查通过。
- controller 初始化完成。
- policy 已加载。
- 不处于 fault。

`ready=false` 时，`GET /status` 和 `GET /health` 仍可用，但 `/execute` 不入队。agent 应先等待或处理 `block`。

`ready=true` 不表示 `ctrl` 一定是 `idle`。例如 stop-to-idle 期间，robot/model 仍健康时可以保持 `ready=true`、`ctrl:"stopping"`；此时 `/execute` 可通过 accept gate，但 start gate 必须等待 `ctrl:"idle"`。`ctrl:"idle"` 表示 app 停止推进 reference 并回到 tracker idle 控制态；不承诺物理机器人已经静止。

### 10.2 `block`

常见值：

- `lowstate_timeout`
- `mode_machine_mismatch`
- `policy_not_loaded`
- `bad_orientation`
- `safety_limit`
- `controller_not_ready`

为空表示没有已知阻塞。

### 10.3 `progress`

`frame` 是 zero-based 当前参考帧，`frames` 是总帧数。

运行中：

```text
progress = min(1.0, (frame + 1) / frames)
```

动作 `done` 时必须强制 `progress = 1.0`。范围为 `0.0` 到 `1.0`。

它只表示参考轨迹进度，不表示物理执行质量。

### 10.4 `id`

动作 `id` 是运行期 ID：

- 短。
- 当前进程内唯一。
- 服务重启后不保证可查询。
- 完成、停止、取消、失败后只在 recent ring buffer 中保留一段时间。
- recent 默认保留最近 `recent_limit` 条，先进先出淘汰；淘汰后 `/status?id=ID` 返回 `RUN_NOT_FOUND`。

## 11. 性能要求

目标是轻量、低延迟、可高频轮询。

可测条件：

- 在本机 loopback `127.0.0.1` 测量。
- 进程启动后 warmup 100 次请求，不计入统计。
- 每项至少 1000 个样本。
- 并发度：`/health`、`/status` 用 4 个客户端并发；`/execute` 用 1 个提交线程，避免把队列满测成延迟。
- `/status` 使用短默认响应，不打开完整诊断。
- `/execute` 使用 allowlist 内合法小 `.trk`。
- 分层验收：`AGENTIC_ET1_BUILD_ROBOT=OFF` 的 core HTTP/fake controller 必须满足；`AGENTIC_ET1_BUILD_ROBOT=ON` 的集成环境记录指标，控制循环不得被 HTTP 拉低。

目标：

- `/health` p95 < 2 ms。
- `/status` p95 < 5 ms。
- `/execute` accept p95 < 20 ms。
- HTTP 并发请求不会阻塞控制循环。
- 状态读取使用快照，避免锁住控制线程。
- 控制循环目标 `50 Hz`；稳定运行时平均频率在 `49.5-50.5 Hz`，单 tick jitter p95 < 5 ms。robot build on 时如硬件/仿真环境导致偏差，必须记录 hz/jitter，不得由 HTTP 请求造成可重复降频。

实现要求：

- HTTP 线程池 2 到 4 个线程。
- `StatusSnapshot` 使用短锁或原子替换。
- `MotionQueue` 使用有界内存队列。
- `CommandMailbox` 承载 stop/interrupt 这类高优先级命令。
- HTTP 线程不做 ONNX 推理、DDS 发布或长时间解析。

## 12. 配置

最小配置：

```yaml
agentic_et1_tracker:
  bind: "127.0.0.1"
  port: 8080
  network: "lo"
  domain_id: 0
  mode_machine: 1
  motion_dirs:
    - "/home/galbot/motions"
  queue_limit: 8
  recent_limit: 32
  max_track_duration_s: 120
  stop_hold_s: 0.5
  idle_mode: "hold_current"
  policy:
    profile: "GeneralTrackerCLN"
    policy_dir: "config/policy/general_tracker_cln"
    policy_file: "multi_policy_v17c2_70k.onnx"
    deploy: "config/policy/general_tracker_cln/params/deploy.yaml"
    fps: 50
```

`domain_id` 为 DDS domain，范围 `0..232`，默认 `0`。MuJoCo 仿真验收时 tracker `domain_id` 必须与 MuJoCo `domain_id` 一致。`policy_dir` 和 `deploy` 的相对路径按 agentic-et1-tracker 配置文件所在目录解析；外部部署资产必须使用绝对路径，或使用相对配置文件目录的路径。

默认只监听 `127.0.0.1`。如需远程调用，应由部署环境显式开放并处理网络安全。

## 13. 安全边界

本服务不是硬件急停系统。

必须遵守：

- 默认本地监听。
- Unitree DDS 本身不提供强互斥；进程启动时做 LowCmd 通道占用 best-effort 检测，配合本 app 进程锁和部署层单控制进程约束，降低双写风险。
- best-effort 检测发现疑似已有控制进程占用时应拒绝启动或进入 `error`，但 PRD 不把它描述成硬互斥保证。
- `.trk` 路径必须受 allowlist 限制。
- 执行中允许 queue 和 interrupt，但语义必须确定。
- `/stop` 必须取消 stop 被控制线程消费时已有的 active/queued；健康可控路径回到 `ctrl:"idle"`，fault/disconnected 只承诺明确 `block/err`。
- `interrupt` 必须先进入受控 stop-to-idle 过渡，再加载新 `.trk`，不能在真机中直接硬切 reference。
- tracker 执行态必须由新 app 自己实现 bad-orientation safety fallback；检测到坏姿态时应进入 `fault` 或拒绝继续执行。
- 出现 safety/fault 时拒绝新执行。
- HTTP 不直接写 `LowCmd`。

## 14. 验收标准

### 14.1 隔离性

- 构建新 app 不修改 `unitree_rl_lab/deploy/robots/et1`。
- 新 app 不 include ET1 app 内部头文件。
- 新 app 不 blanket include `${PROJECT_SOURCE_DIR}/../../include`。
- 新 app 不 include `deploy/include/FSM/*`。
- 新 app 不链接 ET1 app 内部 library/target。
- 新 app 运行时不读取 `unitree_rl_lab/deploy/robots/et1/config/policy/...`。
- 新 app 不把 ET1 app policy/deploy 目录作为默认运行资产；frozen profile 必须位于新 app 目录或外部部署资产目录。
- 默认配置指向的 `policy_dir`、ONNX 和 deploy YAML 必须是 agentic-owned 真实文件，不允许用 ET1 app policy/deploy tree 兜底。
- 新 app CMake 不使用全局 `include_directories()` 或 `link_libraries()`。
- 新 app include/link 全部是 target-level，且只来自允许依赖集合。
- 新 app 不写 `debug/general_tracker_request.txt`。
- ET1 app 原有编译、运行和 request 文件机制不受影响。
- ET1 app 与新 app 不应同时控制 LowCmd；启动 best-effort 占用检查、本 app 进程锁和部署层单控制进程约束必须生效，但 best-effort 不视为硬互斥。

### 14.2 仿真

- 手动/集成仿真验收可使用当前 worktree 中的 Unitree MuJoCo 仿真环境：MuJoCo simulator installation under `/home/galbot/works/et1`，sample/generated `.trk` under `/home/galbot/works/et1/generated/`；该信息不表示已完成仿真测试。
- MuJoCo sim config 的 `domain_id` 必须与 tracker config 的 `domain_id` 一致。
- MuJoCo 验收前必须满足同一 DDS `network` + `domain_id` 下单 LowCmd owner：不存在旧 `et1_ctrl`、`unitree_mujoco`、`agentic-et1-tracker` 或其他同域 LowCmd owner。不同 DDS domain 的既有控制进程必须记录为隔离条件，不能误判为同一 owner；真机 GA 仍需单控制进程约束和真实硬件验收，不能由仿真隔离条件代替。
- 启动仿真且 readiness 通过后，`GET /health` 返回 `ok:true,state:"ready"`。
- `GET /status` 返回 `mode:"sim"`。
- `POST /execute mode=queue` 一个合法 `.trk`，立即返回 `id`。
- 轮询 `/status?id=ID` 能看到 `frame` 增长。
- 连续提交多个 queue 动作，按 FIFO 顺序执行。
- 执行中提交 `mode=interrupt`，请求立即接受，当前动作受控 stop-to-idle，等待队列取消，新动作在 stop-to-idle 后执行。
- 由 interrupt 触发的 stopping 期间，`/status` 只出现 `ctrl:"stopping"` 和 `stop_reason:"interrupt"`，不出现单独的抢断 ctrl 状态。
- `ctrl:"stopping"` 且 robot/model ready 时，新的合法 `queue` 或 `interrupt` 可以被接受，但必须等 stop-to-idle 完成后才开始。
- stop 已在进行时提交 `mode=interrupt`，当前顶层 `stop_reason` 仍为 `stop`，新动作成为 stop-to-idle 后的待执行队首，旧等待动作可通过 `/status?id` 查到 `state:"canceled"`。
- 执行中 `POST /stop` 后，HTTP 响应只表示 accepted；本次 stop 被控制线程消费时已有 active 最终 `stopped` 或明确 `failed`，已有 queued 变为 `canceled`。
- 健康可控且 stop 后无新请求时，控制器回到 `ctrl:"idle"`，`exec:null`，`queue.n==0`。
- stop 后接受新 queue/interrupt 时，新工作不属于本次 stop 的取消集合；`queue.n>0` 或新 `exec` 不使本次 stop 验收失败。
- robot disconnect 或 fault 期间 `/stop` 不承诺 `ctrl:"idle"`，但 `/status` 必须返回明确 `block` 或 `err`。
- 提交 metadata/schema 损坏或缺少关键数组的 `.trk`，请求被拒绝，控制进程不崩溃；payload 语义错误（如 contact 值域非法）若 metadata 合法可被 accepted，但控制线程加载失败并将动作标记为 `failed`。
- oversized metadata、frame_count 不一致、非法 dtype 或畸形 byte_count 的 `.trk` 被拒绝。
- 包含未知但合法数组的 `.trk` 可被 validator/loader 跳过未知 payload，不分配 unknown payload。
- tracker 中触发 bad orientation 时，`/status` 能看到 `robot:"fault"` 或 `block:"bad_orientation"`，并拒绝新执行。

### 14.3 真机

- 真机 lowstate 连接后，`GET /status` 返回 `mode:"real"`。
- `mode_machine` 不匹配时 `ready=false`，并给出 `block`。
- 合法 `.trk` 可执行。
- queue、interrupt、stop-to-idle 的语义与仿真一致。
- LowCmd 占用检测按 best-effort 验证；部署层仍需单控制进程约束。

### 14.4 Core Tests

- 默认 `AGENTIC_ET1_BUILD_TESTS=ON`、`AGENTIC_ET1_BUILD_ROBOT=OFF`。
- core tests 使用 Catch2 v3 + CTest。
- 默认 core tests 不依赖 Unitree SDK2 或 ONNX Runtime。
- 默认不打开 `AGENTIC_ET1_BUILD_ONNX`、`AGENTIC_ET1_BUILD_ROBOT`、`AGENTIC_ET1_BUILD_PERF_SMOKE` 时，core tests 不依赖 MuJoCo、Unitree SDK2 或 ONNX Runtime。
- queue、interrupt/stop 状态机、accept gate/start gate、progress、`TrkSchema`、`TrkValidator` 都有核心单测。
- `/stop` 单测覆盖响应只表示 accepted、stopping 期间新 queue/interrupt 不属于本次 stop 取消集合，且不要求无条件 `queue.n==0`。
- 路径安全单测覆盖 URL/相对路径拒绝、canonical allowlist、symlink escape、控制线程重校验失败。
- loader 单测覆盖同一打开文件句柄 scan+payload read 的 TOCTOU 约束。

### 14.5 性能

- 按第 11 节条件输出 `/health`、`/status`、`/execute` p95。
- core/fake controller 层达到目标延迟。
- robot build on 时记录控制循环 hz 和 jitter；HTTP 并发轮询不造成可重复降频。

### 14.6 Agent 体验

agent 高频使用的是 4 个核心动作，另有 `GET /health` 用于 readiness 探活；route path 仍只有 4 个：`/health`、`/status`、`/execute`、`/stop`。

- `GET /status`
- `GET /status?id=ID`
- `POST /execute`
- `POST /stop`

`GET /health` 仅用于探活。

## 15. 后续但非 GA

以下能力可以以后再讨论，不进入本次 GA：

- 调试接口返回完整关节和 IMU 原始数据。
- 持久化执行历史。
- 多机器人。
- 远程鉴权。
- 动作预览。
- 轨迹质量评分。
- 自动恢复策略。
- 在线 retarget。

## 16. Review 结论

产品侧结论：

- 独立 app 是合理边界。
- 队列和抢断是基础能力，但必须做成单一确定语义。
- `interrupt` 默认清空等待队列，避免 agent 处理旧队列残留。
- 默认状态必须短，否则高频轮询会浪费上下文。
- 动作库、上传、pause/resume、复杂诊断会导致范围蔓延，应删除。

机器人/实现侧结论：

- 现有 ET1 app 只能参考，不能作为运行时依赖。
- `.trk` 是 API 合同；底层真实内容必须是 app-local ET1TRK1 v1 GeneralTracker GA schema runtime cache。
- 新 app 必须自己实现 queue、interrupt、stop-to-idle。
- 生产 HTTP 必须通过 `RuntimeBridge`、`CommandMailbox`、`MotionQueue` 和 `RuntimeControlLoop`；`TrackerController` 只能作为 core/test façade，不能作为生产同步控制入口。
- 新 app 必须自己实现 app-local `TrkSchema`，并让 validator/loader 共用它。
- 新 app 必须自己实现轻量 `.trk` 预校验、payload skip 和异常隔离。
- 新 app 只支持固定 GeneralTrackerCLN frozen profile 和 `config/policy/general_tracker_cln/params/deploy.yaml` 的最小 Policy/Observation 合同。
- frozen profile 必须复制或导出到新 app 目录或外部部署资产目录，默认配置必须指向 agentic-owned 真实资产。
- 真机/仿真能力应继承 Unitree SDK 和 `mode_machine` 语义，而不是继承 ET1 app 代码。
- HTTP 线程只提交命令，不能直接发布 LowCmd。

最终结论：

本 PRD 以“独立 app、只支持 `.trk` 路径、短接口、有界队列、确定抢断、stop-to-idle、真机/仿真一致、零影响现有 ET1 app”为 GA 范围。任何会引入资产管理、复杂调度、业务判断、多格式兼容或依赖现有 ET1 app 内部实现的能力都不进入当前版本。

## 17. 开发交付定义

本节只评价 PRD/产品范围成熟度，不宣称当前代码实现已经达到 GA runtime。PRD/产品范围已闭合；当前代码已有主要 runtime/integration slices、AppRunner/runtime wiring、HTTP via `RuntimeBridge`、ONNX/RobotIO opt-in、Reference/Observation、LowCmd、stop_hold、asset isolation 和 focused tests 的实现证据，但仍不是可直接真机/仿真 GA runtime。GA 取决于 app-owned frozen profile 资产证据、`ONNX+ROBOT` build、MuJoCo/真机验收和真实 integration perf 记录。

### 17.1 产品/文档闭合

- 产品范围闭合：只做 `.trk` 执行、queue、interrupt、stop、status。
- 接口闭合：只需要 `/health`、`/status`、`/execute`、`/stop`。
- 工程边界闭合：独立 app，不改、不包、不链 ET1 app，CMake 只用 target-level include/link。
- 状态机闭合：`starting/idle/preparing/running/stopping/fault`。
- 停止语义闭合：不暴露单独抢断状态，统一用 `ctrl:"stopping"` 和 `stop_reason`；stop 取消集合由 command sequence/watermark 定义。
- Policy/Observation 边界闭合：只支持固定 GeneralTrackerCLN frozen profile 和 `config/policy/general_tracker_cln/params/deploy.yaml` 的最小合同。
- 安全边界闭合：LowCmd best-effort 占用检测、本 app 进程锁、部署层单控制进程约束、allowlist、bad orientation、fault 拒绝执行。
- 输入边界闭合：只接受 `.trk` 路径，内容必须满足 app-local ET1TRK1 `TrkSchema`。
- 性能边界闭合：HTTP 线程不推理、不发布 LowCmd、不读大 payload。
- 测试边界闭合：core TDD 使用 Catch2 v3 + CTest，默认不依赖 Unitree/ONNX。
- 验收边界闭合：隔离性、仿真、真机、agent 体验都有明确验收项。

### 17.2 GA 验收/证据清单

GA 代码交付必须保留已有 runtime/integration 实现，并满足以下验收和证据：

- 默认 app-owned frozen profile 交付时必须存在于新 app 目录或外部部署资产目录，默认配置指向真实存在的 agentic-owned ONNX/deploy 资产；当前资产证据应包括 manifest、hash 校验和 non-symlink 检查记录。
- `AGENTIC_ET1_BUILD_ONNX=ON` 且 `AGENTIC_ET1_BUILD_ROBOT=ON` 的 non-stub runtime factory 可构建、链接并运行真实 ONNX Runtime 与 Unitree SDK2 RobotIO。
- MuJoCo 仿真验收按第 14 节完成，覆盖 generated `.trk`、HTTP `/health`、`/status`、`/execute`、frame progress、`/stop`、queue/interrupt/stop-to-idle 和 fault/disconnect 基本路径。
- 真机验收按第 14 节完成，覆盖 LowCmd 通道占用 best-effort、`mode_machine`、readiness/safety gates、stop_hold、fault fallback 和部署层单控制进程约束。真机验收为 external pending，需要 ET1 硬件和操作者窗口；这不阻止 MuJoCo、`ONNX+ROBOT` build、fake tests 和证据记录继续推进。
- 真实 integration perf 记录按第 11 节和第 14 节完成，至少覆盖 50 Hz 控制循环、HTTP p95、queue/interrupt/stop-to-idle、fault/disconnect 和资产隔离。

### 17.3 成熟度结论

文档稳定成熟，无产品开放问题；当前实现已有主要 runtime/integration slices、focused tests 和 app-owned assets 的 manifest/hash/non-symlink 证据，但仍需按 GA 验收/证据清单完成 `ONNX+ROBOT` build、MuJoCo/真机验收和真实 perf 记录。真机验收仍是 external pending，不得因此阻塞 MuJoCo、build、fake tests 和非真机证据推进；也不能把未完成验收的状态标记为 GA runtime。
