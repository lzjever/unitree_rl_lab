# LOCO Upper Bounded Execution Plan

状态：handoff-ready plan
日期：2026-06-17
Scope：simulation-first，仅要求本机 MuJoCo 验收，不包含真机 GA

## 1. 背景

`agentic-et1-tracker` 的 `/execute_loco_upper` 是新的 executor：下半身由 locomotion lower policy 控制，上半身直接跟随 TRK upper joints。产品方向已经确认改为 bounded execution：

- 合法 TRK 应被 `/execute_loco_upper` 接受并进入执行。
- 下半身 root motion 投影到 locomotion 能力包络和 `max_radius_m`。
- 上半身 joint target 做 position clamp、velocity/acceleration rate-limit 和 smoothing。
- 物理能力类超限不再硬拒绝，只在 status 中用 compact flags 报告。

本计划替代旧 PRD 中“upper 超限拒绝”的语义。旧语义下，upper position/velocity/acceleration 超过机器人物理能力会导致请求被拒绝；新语义下，这些情况属于可边界化执行能力问题，必须进入 bounded execution。

## 2. 当前问题和实证

当前实现仍把 upper 物理能力超限当成 validation failure：

- `src/api_service.cpp`：`/execute_loco_upper` 调用 precheck，失败不入队。
- `src/loco_upper_precheck.cpp`：调用 `extractAndValidateUpperJointTargets`。
- `src/loco_upper_validator.cpp`：position/velocity/acceleration 超限返回 `TRK_VALIDATION_FAILED`。
- `src/runtime_control_loop.cpp`：`prepareLocoUpperTrack` 再次重复硬校验。
- `src/loco_upper_lowcmd_composer.cpp`：写 `LowCmd` 前仍硬检查 upper target 在 limits 内。

实证问题：

- 生成的 walk+wave TRK 被拒绝，诊断结果：`frame 134, joint 17, upper joint_pos is above limit`。
- `/home/galbot/works/et1/generated/walk5_fwd.trk` staging 后被拒绝，诊断结果：`frame 148, joint 17, upper joint velocity exceeds limit`。

结论：主要拒绝原因不是下身/root path，而是 upper validator 对物理能力边界过硬。

## 3. 新产品合同

`/execute_loco_upper` 对所有“合法 TRK”接受执行。

仍拒绝的输入或运行前条件：

- 路径不允许。
- 文件损坏或无法解析。
- schema/shape/fps/frame 非法。
- 必需数组缺失。
- 任意必要数值出现 `NaN` 或 `Inf`。
- 模型/config/mapping 不满足。
- robot readiness 不满足；这是 API/runtime 调用 compiler 前后的 gate，不属于 compiled plan 或 compiler result。

不再拒绝的物理能力类问题：

- root path 超 `max_radius_m`。
- root velocity/yaw/accel 或 locomotion command 超 locomotion envelope。
- upper position 超限。
- upper velocity 超限。
- upper acceleration 超限。
- 动作太快但可通过 rate-limit/smoothing 边界化。

这些问题必须在 compile/prepare 阶段被投影、clamp、rate-limit 或 smooth，并在状态中报告。

`max_radius_m` 是每次 `/execute_loco_upper` 请求发起的 bounded execution
radius。客户端未提供时使用服务端 finite default；服务端 configured max 只表示
部署能力上限，并作为请求/default radius 的 effective cap。raw TRK root path
超过 effective radius 时必须 accepted，并把 root trajectory
projection/clamp 到限制内执行；不能因为动作本身超过半径而 rejected、failed 或
进入 passive。

时间对齐硬约束：

- bounded execution 不改变 `fps`、`frame_count` 或 raw TRK `duration_s`；当前 `duration_s` 口径为 `(frame_count - 1) / fps`。
- root 和 upper 必须保持原 TRK frame index 对齐；第 `i` 帧 root command 对应第 `i` 帧 upper target。
- rate-limit/smoothing 只能修改每帧目标值，不能 time-warp、重采样、插帧、丢帧或改变 raw TRK `duration_s`。
- 若未来需要时间重排，必须另开产品决策，不纳入本计划。

执行中的安全故障保持现有路径：

- 显式授权的 `/passive`。
- bad orientation。
- 摔倒风险或姿态越界。
- `LowCmd` 写失败。
- policy 输出非有限值。
- 其他已有 safety/fault/passive 条件。

radius limit 本身不属于 safety/fault/passive 条件。运行时到达 effective radius
边界时，应 suppress outward radial velocity，记录 `radius_limit_reached`，并保持受控执行、受控完成、hold、bounded exit 或回 standby。

## 4. 目标和非目标

目标：

- `/execute_loco_upper` 接受合法 TRK，不因 upper/root 物理能力超限拒绝。
- API precheck/probe 和 runtime prepare 使用同一套 planner/compiler 逻辑。
- runtime 每帧使用 compiled upper plan，不再直接从 raw TRK 取 upper target。
- composer 只保留最后安全防线，正常路径传入值应已完成 clamp/rate-limit。
- status 用 minimal flags 暴露 bounded execution 是否介入，并贯通 queued/active/recent 状态链路。
- 不影响 `/execute` GeneralTracker、`/standby_velocity`、`/stop`、`/passive` 等现有功能。

非目标：

- 不引入 fast direct path。
- 不引入上传流程。
- 不设计复杂 profile 系统。
- 不暴露 per-joint 大量 telemetry。
- 不做真机 GA。
- 不重写 locomotion policy、robot readiness 或已有 safety state machine。

## 5. KISS/DRY/YAGNI 原则

- KISS：先实现一个清晰的 `LocoUpperPlanCompiler`，只覆盖当前 `/execute_loco_upper` 所需的 bounded execution。
- DRY：API precheck/probe 和 runtime prepare 复用同一 compiler，不维护两套校验和修正逻辑。
- YAGNI：只新增必要 status flags；不增加复杂配置层、profile、细粒度 telemetry 或外部接口扩展。
- Fail early 在 compiler 内只用于 TRK 合法性、config、mapping 和 limits 可解释性；robot readiness 是 API/runtime gate；能力边界用 compile flags 报告，不用于拒绝。
- Safety stays local：composer 和 runtime safety 仍作为防线，但不承担业务层 plan 修正。

## 6. API 和状态合同

请求字段：

- 保留 `/execute_loco_upper` 原请求字段。
- 不新增必填字段。
- 不改变已有路径、TRK、执行模式相关字段语义。
- `max_radius_m` 是客户端请求级行动范围约束；缺省时用服务端 default，effective radius 由请求/default 值再按服务端 max capability cap 得到。
- 有限正数请求值高于服务端 max capability 时，不因请求本身失败；effective radius 使用服务端 max。

响应和 status：

- 合法 TRK 即使触发 bounded execution，也返回 accepted/queued。
- 对外 status 使用 compact flags：
  - `radius_clamped`：root path 被 `max_radius_m` 投影。
  - `radius_limit_reached`：runtime 到达 effective radius 边界并 suppress outward radial velocity；这不是 failure/passive 标志。
  - `envelope_clamped`：root velocity/yaw/accel 或 locomotion command 被能力包络限制。
  - `upper_clamped`：至少一个 upper joint position 被 clamp。
  - `upper_rate_limited`：至少一个 upper joint velocity 或 acceleration 被 rate-limit/smoothing 影响。
- `radius_limit_reached` 是正常 bounded execution 状态，不应配套
  `loco.reason:"radius_limit"` 或其他 radius-specific reason。
- 内部可区分 `upper_accel_limited`，用于测试、日志或 compiler 诊断；外部 status 保持紧凑，不新增独立公开 flag，除非后续产品明确需要。
- root bounded 状态必须复用现有公开字段 `radius_clamped` 和 `envelope_clamped`；不要新增 `root_projected` 或其他 root projection 公开字段。
- status flags 不能只存在于 compiler 内部，必须写入 `LocoRunStatus`，经 `RuntimeBridge`/`RuntimeStatusStore` 保存，通过 JSON codec 输出，并由 API/status tests 覆盖。
- queued、active、recent status 都必须能携带 `upper_clamped` 和 `upper_rate_limited`；无 bounded execution 时显式为 `false` 或保持既有 false 默认值。

拒绝响应：

- 只用于 TRK 合法性、路径、安全输入、配置、mapping、limits 可解释性，以及 API/runtime readiness gate 等不可执行条件。
- 错误信息必须说明拒绝来自 legality/config/mapping/readiness gate，而不是 upper/root 物理能力边界。

示例外部 status 语义：

```json
{
  "radius_clamped": true,
  "radius_limit_reached": false,
  "envelope_clamped": false,
  "upper_clamped": true,
  "upper_rate_limited": true
}
```

`strict_pose` 语义：

- `strict_pose` 不再用于 API precheck/probe 拒绝 root path 超 `max_radius_m`。
- 无论 `strict_pose` 是否开启，root path 超 radius 都应 accepted，并在 compiler 中投影，设置 `radius_clamped`。
- `strict_pose` 只约束 runtime highstate freshness/jump 是否足以声明物理 radius containment；pose source 不可信时可按 readiness/pose gate 处理，但不能把 raw root 超 radius 或运行时到达半径边界解释为 passive/fault。

## 7. 模块设计和边界

### LocoUpperPlanCompiler

新增或重构为单一入口：

- 输入 raw TRK 和 `LocoUpperCompileOptions`。
- 输出 `CompiledLocoUpperPlan` 和 `LocoUpperCompileFlags`。
- 执行 legality validation、root projection、upper clamp、rate-limit、smoothing。
- 对物理能力问题不返回 validation failure。
- 保持 raw TRK 的 `fps`、`frame_count`、`duration_s` 和 root/upper frame index 对齐；当前 `duration_s` 口径为 `(frame_count - 1) / fps`。
- 不检查 robot readiness，不产生 `RobotNotReady` 诊断；readiness 由 API/runtime gate 处理。
- 输出的每帧 joint target 必须是 full 26 logical joints（`kPolicyJointCount`），与当前 runtime/composer 输入形状一致；不是 14 维 upper-only 数组。

### Compiler options/factory

新增 `LocoUpperCompileOptions` 或等价 factory 注入：

- API precheck/probe 和 runtime prepare 必须从同一 provider/factory 获取 robot/model mapping、joint limits、lower deploy locomotion envelope、`max_radius_m` 和 runtime config。
- 不允许 API 使用轻量 limits，而 runtime 使用另一套 envelope 重新 compile。
- API precheck 可以只调用同一 compiler 的 probe 模式，但 probe 与 runtime compile 的 options 必须一致。
- 若 runtime prepare 因状态原因需要重新 compile，必须使用同一 options provider，并对同一 TRK 得到同一 bounded flags 和同一帧数/形状。

### Loco upper validator

这里的 Validator 只指 `src/loco_upper_validator.cpp` 相关的 loco-upper validator，职责收窄为可解释性校验：

- 检查 schema/shape/fps/frame/array 存在性。
- 检查 finite values。
- 检查 mapping 是否可解释。
- 不因为 position/velocity/acceleration 超物理能力返回 `TRK_VALIDATION_FAILED`。

`loco_upper_validator` 可以产生 diagnostics，但能力类 diagnostics 应转为 compile flags。不要修改 `/execute` 使用的全局 TRK validator、路径检查或 schema validation；GeneralTracker `/execute` 合同不变。

### API precheck/probe

`src/api_service.cpp` 和 `src/loco_upper_precheck.cpp` 应改为调用 compiler：

- compiler 成功：请求 accepted，status 初始化携带 compile flags。
- compiler 失败：仅限 legality/config/mapping/limits 可解释性类错误。
- robot readiness 在调用 compiler 前后按现有 API/runtime 路径 gate；readiness rejected 不写入 compiled plan，也不作为 compiler diagnostic/result。
- API precheck/probe 必须通过 shared factory 构造 compiler/options，配置与 runtime prepare 一致。
- 不在 API 层重复 upper limit 判断。

### Runtime prepare

`src/runtime_control_loop.cpp::prepareLocoUpperTrack` 应使用同一 compiler：

- 避免与 API precheck 语义漂移。
- 可复用 API 阶段缓存的 compiled plan；若进程内状态不适合缓存，则在 runtime prepare 重新 compile，但必须调用同一实现。
- runtime readiness 在 prepare 前后按现有路径 gate；readiness failure 不伪装成 compiler failure。
- runtime prepare 使用 shared factory 的同一 mapping、limits、lower deploy envelope 和 runtime config。
- prepare 后 runtime 保存 compiled plan，执行 loop 每帧读取 compiled targets。

### Runtime execution

执行 loop：

- lower body 使用 bounded root locomotion plan。
- upper body 使用 compiled plan 中的 full 26 logical joint frame。
- 不再从 raw TRK 每帧取 upper target。
- entry、exit、hold、current fallback 都不能回读 raw TRK；只能从 compiled plan 或 bounded standby target 读取。
- status 中持续携带 `radius_clamped`、`radius_limit_reached`、`envelope_clamped`、`upper_clamped`、`upper_rate_limited`。
- runtime 到达 effective radius 边界时，必须 suppress outward radial velocity，设置 sticky `radius_limit_reached`，继续受控执行或受控完成/回 standby；不得因为 radius limit 本身进入 passive/fault。
- 执行期间不得改变 raw TRK `fps`、`frame_count`、`duration_s` 或 root/upper frame index 对齐。

### 时间对齐研发合同

- `CompiledLocoUpperPlan.fps` 必须等于 raw TRK `fps`。
- `CompiledLocoUpperPlan.frame_count` 必须等于 raw TRK `frame_count`。
- bounded execution 必须保持 raw TRK `duration_s` 不变；当前 `duration_s` 口径为 `(frame_count - 1) / fps`。
- `root_frames.size()` 和 `joint_pos_frames.size()` 必须等于 `frame_count`。
- root 和 upper 的第 `i` 帧必须来自 raw TRK 的同一第 `i` 帧。
- clamp、projection、rate-limit 和 smoothing pass 只能改每帧目标值，不能 time-warp、resample 或改变 raw TRK `duration_s`。

### Status propagation

- `LocoRunStatus` 增加或复用 `radius_clamped`、`radius_limit_reached`、`envelope_clamped`、`upper_clamped`、`upper_rate_limited` 字段。
- `RuntimeBridge` 和 `RuntimeStatusStore` 必须保存 compile flags，并在 queued、active、recent 三类 run status 中保留这些字段。
- JSON codec 必须序列化这些字段，API status tests 必须覆盖 queued/active/recent 三种状态。
- 任何状态层丢失 `upper_clamped` 或 `upper_rate_limited` 都视为未完成。

### LowCmd Composer

`src/loco_upper_lowcmd_composer.cpp` 保持最后安全防线：

- 正常路径要求传入 upper target 已经满足 limits。
- 若仍收到非有限值或越界值，进入现有 fault/safety 路径或返回明确 guard failure。
- 不在 composer 里偷偷 clamp/rate-limit 业务目标，避免隐藏 planner/compiler bug。

## 8. 数据结构草案

### Compile options

```cpp
struct LocoUpperCompileOptions {
  const RobotModelMapping* mapping = nullptr;
  const JointLimits* joint_limits = nullptr;
  const LowerDeployEnvelope* lower_envelope = nullptr;
  double max_radius_m = 0.0;
  RuntimeLocoUpperConfig runtime_config;

  // true means API probe/precheck may skip caching, but must use the same
  // compiler logic and options as runtime prepare.
  bool probe_only = false;
};

LocoUpperCompileOptions makeLocoUpperCompileOptions(
    const RuntimeConfig& runtime_config,
    const RobotModel& robot_model,
    const LowerDeployConfig& lower_deploy_config);
```

语义：

- API precheck/probe 和 runtime prepare 都通过同一 factory/provider 构造 options。
- options 中的 mapping、limits、lower envelope、`max_radius_m` 和 runtime config 不允许分叉。

### Compile flags

```cpp
struct LocoUpperCompileFlags {
  bool radius_clamped = false;
  bool envelope_clamped = false;
  bool upper_clamped = false;
  bool upper_rate_limited = false;

  // Internal only. Can be used by unit tests/logging, but should not expand
  // public status unless product asks for it.
  bool upper_accel_limited = false;
};
```

### Compile diagnostics

```cpp
struct LocoUpperCompileDiagnostic {
  enum class Kind {
    TrkSchemaInvalid,
    TrkShapeInvalid,
    TrkFpsInvalid,
    TrkFrameInvalid,
    MissingRequiredArray,
    NonFiniteValue,
    MappingInvalid,
    ConfigInvalid,
    RootRadiusClamped,
    RootEnvelopeClamped,
    UpperPositionClamped,
    UpperVelocityLimited,
    UpperAccelerationLimited,
  };

  Kind kind;
  int frame = -1;
  int joint = -1;
  std::string message;
};
```

### Compiled plan

```cpp
struct CompiledLocoUpperPlan {
  // Preserved from the raw TRK; bounded execution must not resample or stretch.
  double fps = 0.0;
  int frame_count = 0;

  // Root command after projection to max_radius_m and locomotion envelope.
  std::vector<RootCommandFrame> root_frames;

  // Full logical joint positions after upper clamp, rate-limit, and smoothing.
  // Each frame has exactly kPolicyJointCount == 26 values, matching runtime and
  // composer expectations. This is not a 14-d upper-only array.
  // Size must stay aligned with root_frames and frame_count.
  std::vector<std::array<double, kPolicyJointCount>> joint_pos_frames;

  LocoUpperCompileFlags flags;
  std::vector<LocoUpperCompileDiagnostic> diagnostics;
};
```

runtime 语义：

- frame `i` 的 `joint_pos_frames[i]` 是 composer 的唯一正常输入来源。
- entry/exit/hold/current fallback 不得从 raw TRK 读值；只能使用 `joint_pos_frames` 或 bounded standby target。
- 若缺少 compiled plan，runtime 应拒绝或进入既有 safety 路径，不允许退回 raw TRK 执行。

### Compile result

```cpp
struct LocoUpperCompileResult {
  bool ok = false;
  CompiledLocoUpperPlan plan;
  std::string reject_code;
  std::string reject_message;
};
```

语义：

- `ok == false` 只表示 legality/config/mapping/limits 可解释性问题。
- robot readiness 不属于 compiler result；由 API/runtime gate 使用现有拒绝语义处理。
- bounded execution 介入时 `ok == true`，并设置 `flags`。

## 9. 实施步骤：TDD 小闭环

1. 写 failing tests：合法 TRK 但 upper position 超限时，compiler 成功并设置 `upper_clamped`。
2. 写 failing tests：合法 TRK 但 upper velocity 超限时，compiler 成功并设置 `upper_rate_limited`。
3. 写 failing tests：合法 TRK 但 upper acceleration 超限时，compiler 成功，内部 `upper_accel_limited == true`，外部 status 仍只体现 `upper_rate_limited`。
4. 写 failing tests：root path 超 `max_radius_m` 时，compiler 成功并设置 `radius_clamped`。
5. 写 failing tests：root velocity/yaw/accel 或 locomotion command 超 envelope 时，compiler 成功并设置 `envelope_clamped`。
6. 写 failing tests：bounded execution 后 `fps`、`frame_count`、raw TRK `duration_s`、root/upper frame index 对齐不变。
7. 写 failing tests：compiled joint frame 是 full 26 logical joints（`kPolicyJointCount`），不是 14 维 upper-only。
8. 写 failing tests：queued/active/recent status 都能输出 `radius_clamped`、`radius_limit_reached`、`envelope_clamped`、`upper_clamped` 和 `upper_rate_limited`。
9. 写 failing tests：`strict_pose` 不因 root path 超 radius 触发 API precheck 拒绝。
10. 实现 `LocoUpperCompileOptions`/factory，API precheck/probe 和 runtime prepare 共用 mapping、limits、lower deploy envelope、`max_radius_m`、runtime config。
11. 实现 `LocoUpperPlanCompiler` 的 legality validation skeleton，先复用现有 parser/mapping/limit helper。
12. 将 upper position hard validation 改为 clamp pass。
13. 将 upper velocity hard validation 改为 rate-limit pass。
14. 将 upper acceleration hard validation 改为 smoothing/rate-limit pass。
15. 将 root radius/envelope hard validation 改为 projection pass。
16. 改 API precheck/probe 调用同一 compiler/factory，删除或绕开能力类拒绝分支。
17. 改 runtime prepare 调用同一 compiler/factory，并保存 compiled plan。
18. 改 runtime execution 从 compiled plan 的 full 26 维 frame 读取 targets；entry/exit/hold/current fallback 只读 compiled plan 或 bounded standby target。
19. 调整 composer guard：只检测非有限值和异常越界，正常路径不做业务修正。
20. 扩展 `LocoRunStatus`、`RuntimeBridge`/`RuntimeStatusStore`、JSON codec 和 API/status tests，贯通 `radius_clamped`、`radius_limit_reached`、`envelope_clamped`、`upper_clamped`、`upper_rate_limited`。
21. 跑单元测试和 API-level tests。
22. 本机 MuJoCo 仿真执行 walk+wave 和 `walk5_fwd.trk`，确认不因 upper 能力边界拒绝。

每一步都应保持可回滚、可测试；不要在一个改动中同时重写 API、runtime、composer 和 status。

## 10. 测试矩阵

### API

- 合法 TRK，无能力超限：accepted，flags 全 false。
- 合法 TRK，upper position 超限：accepted，`upper_clamped == true`。
- 合法 TRK，upper velocity 超限：accepted，`upper_rate_limited == true`。
- 合法 TRK，upper acceleration 超限：accepted，`upper_rate_limited == true`。
- 合法 TRK，root 超 `max_radius_m`：accepted，root plan 被投影，`radius_clamped == true`。
- 合法 TRK，root velocity/yaw/accel 或 locomotion command 超 envelope：accepted，root plan 被限制，`envelope_clamped == true`。
- `strict_pose == true` 且 root path 超 radius：accepted，`radius_clamped == true`，不在 precheck 拒绝。
- `request.max_radius_m` 为有限正数但高于服务端 max capability：accepted，effective radius 使用服务端 max。
- 非法路径：rejected。
- 文件损坏：rejected。
- schema/shape/fps/frame 非法：rejected。
- 必需数组缺失：rejected。
- `NaN`/`Inf`：rejected。
- mapping/config/limits 不可解释：由 compiler 拒绝。
- robot readiness 不满足：由 API/runtime readiness gate 拒绝，不属于 compiler result。
- queued、active、recent status JSON 都包含 `upper_clamped` 和 `upper_rate_limited`。
- API/status tests 覆盖 status flags 从 accepted/queued 到 active/recent 的传播。

### Compiler/Planner

- position clamp 后所有 upper joint 在 limit 内。
- velocity rate-limit 后相邻帧 delta 满足 limit。
- acceleration smoothing 后二阶变化满足 limit 或 documented bound。
- clamp 和 rate-limit 同时触发时 flags 都正确。
- root projection 后半径不超过 `max_radius_m`，并设置 `radius_clamped`。
- root projection 后 `vx`/`vy`/yaw/accel 不超过 locomotion envelope，并设置 `envelope_clamped`。
- 输出 `fps`、`frame_count`、`duration_s` 与 raw TRK 一致。
- `root_frames` 和 `joint_pos_frames` frame index 一一对齐，无 time-warp、重采样、插帧、丢帧或改变 raw TRK `duration_s`。
- `joint_pos_frames[i].size() == kPolicyJointCount == 26`。
- API probe 和 runtime prepare 对同一 TRK、同一 options 输出一致 flags、帧数和 target 形状。
- diagnostics 能定位 frame/joint，但公开 status 不暴露 per-joint telemetry。
- compiler diagnostics/result 不包含 `RobotNotReady`。
- `loco_upper_validator` 的能力类 diagnostics 转为 compile flags；全局 TRK validator 行为不变。

### Runtime

- `prepareLocoUpperTrack` 不再重复调用旧 hard validator。
- runtime 保存并使用 compiled plan。
- 执行 loop 不从 raw TRK 直接读取 upper target。
- entry、exit、hold、current fallback 不从 raw TRK 读值。
- composer 收到 full 26 logical joint frame。
- status 在执行中保持 compile flags。
- API compile 和 runtime compile 对同一 TRK 结果一致。
- rate-limit/smoothing 只改变每帧目标值，不改变执行时钟、帧数或 raw TRK `duration_s`。
- `strict_pose` highstate freshness/jump pose-source gate 仍按现有 runtime 路径生效，但 radius boundary 本身不触发 passive/fault。

### Composer guard

- compiled target 正常通过 composer。
- 非有限 upper target 被 guard 拦截。
- 明显越界 upper target 被 guard 拦截并暴露为内部 bug/safety condition。
- composer 不执行静默 clamp/rate-limit。

### Regression

- `/execute` GeneralTracker 不受影响。
- `/standby_velocity` 不受影响。
- `/stop` 不受影响。
- `/passive` 不受影响。
- bad orientation、policy 非有限输出、`LowCmd` 写失败仍走现有 safety/fault/passive 路径。
- `/execute` 使用的全局 TRK validator、路径检查、schema validation 合同不变。

## 11. 仿真验收计划

验收范围：

- 仅要求本机 MuJoCo。
- 不要求真机。
- 不要求真机 GA。

候选 TRK：

- 已知 walk+wave TRK，之前在 `frame 134, joint 17` 因 position above limit 被拒绝。
- `/home/galbot/works/et1/generated/walk5_fwd.trk`，之前在 `frame 148, joint 17` 因 velocity exceeds limit 被拒绝。
- 一条无 upper/root 能力超限的 baseline TRK。

验收步骤：

1. 启动本机 MuJoCo simulation。
2. 调用 `/execute_loco_upper` 执行 baseline TRK，确认 accepted 且 flags false。
3. 调用 `/execute_loco_upper` 执行 walk+wave TRK，确认 accepted，`upper_clamped == true` 或 `upper_rate_limited == true`。
4. 调用 `/execute_loco_upper` 执行 `walk5_fwd.trk`，确认 accepted，`upper_rate_limited == true`。
5. 观察 robot root motion 未超过 bounded path，upper motion 连续、无明显瞬时跳变。
6. 截图或读图可作为定性门禁：确认机器人处于可见、非倒地、动作连续状态。
7. 检查日志无旧 hard validation rejection。
8. 检查 safety/fault/passive 未被异常触发；若触发，错误应来自运行时安全条件，而不是 precheck upper 能力拒绝。

通过标准：

- 三条候选 TRK 均可在 simulation 中进入执行。
- 两条历史拒绝用例不再因 upper position/velocity/acceleration 物理能力超限被拒绝。
- status flags 能说明 bounded execution 是否介入。
- queued、active、recent status 都能看到 `upper_clamped`/`upper_rate_limited`。
- 无新增对其他 executor/control endpoint 的回归。

## 12. 风险和反模式

风险：

- API precheck 和 runtime prepare 分叉，导致 staging 接受但 runtime 拒绝。
- 只关掉 precheck，导致 composer 或 runtime 中后段继续硬拒绝。
- composer 静默修正 target，隐藏 compiler 缺陷。
- rate-limit/smoothing 若改变动作时长、frame count 或 frame alignment，属于违反时间对齐硬约束，必须阻断合入。
- flags 暴露过细，形成难以维护的外部 telemetry contract。
- root projection 与 upper timing 不一致，导致视觉动作和位移语义偏离过大。
- flags 只停留在 compiler 内部，未贯通 `LocoRunStatus`、status store 或 JSON codec。
- API precheck/probe 和 runtime prepare 使用不同 limits、mapping 或 lower envelope，导致 accepted 后 runtime 重新 compile 出不同计划。
- compiled target 输出 14 维 upper-only，和当前 runtime/composer 的 full 26 logical joints 形状不匹配。
- entry/exit/hold/current fallback 退回 raw TRK，绕过 bounded execution。
- 误改 `/execute` 全局 validator，造成 GeneralTracker 合同回归。

反模式：

- 不要只删除 validator 判断。
- 不要把 bounded execution 逻辑散落到 API、runtime、composer 三处。
- 不要让 status flags 只存在于 compiler flags 或日志。
- 不要在 API 层另建轻量 limits/envelope。
- 不要把 compiled upper frame 做成 14 维 upper-only。
- 不要在任何 fallback 路径读取 raw TRK target。
- 不要用 `strict_pose` 在 precheck 拒绝 root radius 超限。
- 不要修改 `/execute` 的全局 TRK validator、路径检查或 schema validation。
- 不要新增 fast direct path。
- 不要新增上传能力。
- 不要新增复杂 profile。
- 不要公开 per-joint 大量 telemetry。
- 不要把真机 GA 放入本阶段交付。
- 不要影响 `/execute` GeneralTracker、`/standby_velocity`、`/stop`、`/passive` 的现有行为。

## 13. 交付 checklist

- [ ] `LocoUpperPlanCompiler` 单一入口落地。
- [ ] `LocoUpperCompileOptions`/factory 落地，API precheck/probe 和 runtime prepare 共用 mapping、limits、lower envelope、`max_radius_m`、runtime config。
- [ ] API precheck/probe 使用 compiler。
- [ ] Runtime prepare 使用 compiler。
- [ ] Runtime execution 使用 compiled upper plan。
- [ ] Compiled joint frame 是 full 26 logical joints（`kPolicyJointCount`），不是 14 维 upper-only。
- [ ] entry/exit/hold/current fallback 只读 compiled plan 或 bounded standby target，不读 raw TRK。
- [ ] `loco_upper_validator` 不再因 upper/root 物理能力边界拒绝。
- [ ] `/execute` 全局 TRK validator、路径检查、schema validation 不变。
- [ ] Composer 保持 guard，不做业务 clamp/rate-limit。
- [ ] `LocoRunStatus` 可携带 bounded flags。
- [ ] `RuntimeBridge`/`RuntimeStatusStore` 保存并传播 bounded flags。
- [ ] JSON codec 输出 bounded flags。
- [ ] queued/active/recent status tests 覆盖 `upper_clamped` 和 `upper_rate_limited`。
- [ ] `radius_clamped` status flag 接入。
- [ ] `radius_limit_reached` status flag 接入；到达边界时 suppress outward radial velocity，但不 passive/fault。
- [ ] `envelope_clamped` status flag 接入。
- [ ] `upper_clamped` status flag 接入。
- [ ] `upper_rate_limited` status flag 接入。
- [ ] 内部 `upper_accel_limited` 可测试但不扩张公开 API。
- [ ] bounded execution 不改变 `fps`、`frame_count`、raw TRK `duration_s` 或 root/upper frame index 对齐。
- [ ] `strict_pose` 不再让 API precheck/probe 拒绝 root radius 超限；runtime highstate freshness/jump pose-source gate 保持现有语义，radius boundary 本身不触发 passive/fault。
- [ ] 历史 position rejection 用例 accepted。
- [ ] 历史 velocity rejection 用例 accepted。
- [ ] 非法 TRK 仍由 compiler rejected；readiness 仍由 API/runtime gate rejected，且不出现在 compiler result/diagnostics。
- [ ] GeneralTracker、standby、stop、passive regression tests 通过。
- [ ] 本机 MuJoCo 仿真验收通过。
- [ ] README/PR notes 明确：本计划替代旧 PRD 的 “upper 超限拒绝” 语义。
