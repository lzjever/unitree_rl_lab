# 状态/接口契约一致性修正计划

> 本文件是后续开发计划，用于 handoff 给产品/研发/测试团队；本次不要求实现。计划目标是收敛已有 review findings，修正文档契约不一致，并安排一个已确认的中等风险代码修复。

## 背景

agentic-et1-tracker 的 HTTP API、`et1-action` CLI/skill 输出、README 状态矩阵和验收文档已经形成了可用的状态契约，但 review 发现部分描述与 runtime 真实行为、skill 心智模型或验收断言不完全一致。这类问题不会直接改变机器人行为，却会让 agent、operator 和后续开发者对 `/idle`、`/fixstand`、`/passive`、`standby` 等边界产生不同理解。

当前原则是先收敛契约：能用文档澄清的地方修文档，不为了文档一致性改 runtime。已经确认会导致 CLI 成功误判的 `standby`/`idle-clear` 瞬时状态问题，作为独立最小代码修复交付，不与纯文档 cleanup 绑定。

## 目标

- 统一 README、matrix、ACCEPTANCE、skill references 与 runtime 真实 gate 的状态/接口契约表述。
- 保持 HTTP 与 CLI 分层清晰：HTTP 字段、CLI argv、skill `next` token 不混用。
- 明确 `/idle` 是配置 endpoint，不是 run endpoint；非空 `/idle` 用 allow/deny 状态集描述，空 `/idle {"paths":[]}` 任意状态可清空。
- 明确 `/fixstand`、`/passive`、`/standby`、`/urgent_stop` 的业务边界，避免 agent 自动恢复或错误升级到 safety sink。
- 后续最小修复 `et1-action` 的 standby 确认逻辑，避免 `ctrl=fixstand` 且 `active.kind=none` 的瞬时状态被误判为 standby 成功。
- 增加精准验收点覆盖 disputed 状态，不扩大默认发布 gate。

## 非目标

- 不新增 endpoint。
- 不做 runtime 大改。
- 不把 `/idle` 改成 run 或用户动作提交语义。
- 不做密码/权限系统重构。
- 不做自动 passive 恢复，也不让 agent 自动 follow `passive -> fixstand -> standby`。
- 不引入远程 payload、`.et1trk` 新格式、queue priority、transition profile。
- 不把 manual gate、E2E、soak 纳入默认发布 gate；这些只作为 opt-in 开发/调试/发布仪表。
- 不为测试工具本身增加二级验证；新增测试只验证真实契约和风险点。

## 产品契约原则

1. 一个功能一种做法：普通停止只走 `standby`，急停只走 `urgent-stop`/`POST /urgent_stop`，完全静止/不播 idle 只走 `idle-clear`，显式固定站姿才走 `fixstand`。
2. HTTP 与 CLI 分层：HTTP response 的 `next:"urgent_stop"` 是接口建议字段，CLI argv 是 `urgent-stop`；文档必须明确 HTTP `next` 不是 CLI argv。
3. `/idle` 只管理 idle pool：非空 `/idle` 是配置 idle motions，空 `/idle {"paths":[]}` 是清空 idle config；二者不代表用户动作开始、完成或正在播放。
4. 文档优先贴近 runtime 真实行为：已存在的 runtime gate 不因 README wording 而重写；如需改变 runtime，必须另起产品/安全 review。
5. 安全边界优先：`passive` 是 passworded safety sink，`fixstand` 是显式 recovery/control prep，不是普通“站着别动”。
6. 治理只做仪表：manual gate、E2E、soak、release report 用于观察和发布证据，不成为普通开发主线或默认阻塞项。
7. KISS/DRY/YAGNI：修正文档和一个确认函数即可解决的，不拆多套规则，不扩展协议，不为未来可能性预留复杂机制。

## 已确认问题（Confirmed Findings）

| ID | 范围 | 已确认问题 | 修正方向 | 类型 |
| --- | --- | --- | --- | --- |
| F1 | README `/idle` 摘要与状态表 | non-empty `/idle` 摘要写成只接受 standby 或 user preparing/running，但表格又写 active idle、holding、transition 接受。真实 gate 阻塞 Starting/Internal Idle/Passive/FixStand/Stopping/UrgentStopping/Fault，接受 public standby/preparing/running；holding、active idle、transition 因 public `ctrl:"running"` 而接受。 | 只修文档，不改 runtime。README 摘要和表格统一为明确 allow/deny，并解释 active idle/holding/transition 被接受的原因是 public `ctrl:"running"`。 | 文档 |
| F2 | Matrix `/idle` 规则 | non-empty `/idle` 拒绝集不完整，遗漏 starting/internal idle/stopping 等。 | Matrix 改为显式 allow/deny：allow public standby/preparing/running；deny starting/internal idle/passive/fixstand/stopping/urgent_stopping/fault。 | 文档 |
| F3 | README fault 行 | fault 行把 `/fixstand` 放在 accepts，容易被理解为任意 fault 都可恢复。 | 限定为 readiness OK 或 `block:"bad_orientation"` recovery 且 LowCmd/free 条件满足；其他 fault/manual/lowcmd_occupied 仍需 operator/manual。 | 文档 |
| F4 | passive password 示例 | README 暴露 agent-facing passive password 示例，与 skill 的“operator 必须显式提供密码、agent 不查默认密码”冲突。 | 修 README 表述为 `{"password":"<operator-provided-password>"}`，说明 password 由 operator 显式提供，agents/skills must not assume a default password；不做密码系统重构。 | 文档 |
| F5 | output-contract 成功字段 | 默认 success fields 漏 `matched`；同时容易被误读为完整 schema。`standby`/`idle-load` 状态语义不够清楚。 | `output-contract.md` 将 `matched` 加入 common success fields，并说明 fields 是 common/not exhaustive。补充：`standby` 可能返回 `STANDBY_NOT_CONFIRMED`；`idle-load` 成功表示 idle pool 配置成功，不代表 idle 正在播放。 | 文档 |
| F6 | HTTP next 与 CLI next 命名 | HTTP `next` 与 CLI 命令命名不同，如 `urgent_stop` vs `urgent-stop`。 | 保留分层，README 与 `output-contract.md` 都要明确 HTTP `next` 不是 CLI argv；CLI/skill 层负责映射。 | 文档 |
| F7 | `et1-action` standby 误判 | `standby`/`idle-clear` 可能把 `ctrl:"fixstand"` 且无 active 的瞬时状态误判为 standby 成功。 | 独立最小代码修复：`standby` 使用收紧后的 `standby_confirmed()`；`idle-clear` 使用同一拒绝原则，但成功必须确认 ordinary standby 且 idle config 已清空，不能接受 active idle 分支。 | 代码 |
| F8 | ACCEPTANCE disputed 状态 | 验收缺少 explicit disputed points：non-empty `/idle` accepted/rejected 状态和 gate 顺序、HTTP `/idle {"paths":[]}` 任意状态清空、CLI `idle-clear` clear-then-standby 的可能失败语义、passive 缺少 `--password` 时不执行、旧路由 `/stop`/`/standby_velocity` 仍只返回 renamed error。 | 在 ACCEPTANCE 增加精准验收点；保持为单元/契约级断言，不扩大默认 gate。 | 文档/测试 |

## 实施计划

### 1. 文档契约收敛

- 更新 README `/idle` 段落：
  - 明确 `/idle {"paths":[]}` clears idle pool and is accepted in any controller state。
  - 明确 non-empty `/idle` allow：public `ctrl:"standby"`、user `preparing`、user `running`，以及 public `ctrl:"running"` 下的 active idle、holding、transition。
  - 明确 non-empty `/idle` deny：starting、internal idle、passive、fixstand、stopping、urgent_stopping、fault。
  - 明确 non-empty `/idle` gate 顺序按实现：blocked controller 先返回 controller conflict；只有 controller 允许后才检查 readiness/manual error。
  - 区分 HTTP `/idle {"paths":[]}` 任意状态清空与 CLI `idle-clear` 的 clear-then-standby 行为；CLI 成功必须确认 ordinary standby 且 idle config 已清空，active idle 不能作为成功。
  - 避免把 `/idle` 描述成 run submission 或动作完成信号。
- 更新 README 状态矩阵：
  - 表格和摘要使用同一套 allow/deny wording。
  - fault 行把 `/fixstand` 限定为 readiness OK 或 `bad_orientation` recovery 条件，不写成任意 fault 可接受。
  - passive 示例改为 operator-provided password，不出现默认密码字面量；agents/skills must not assume a default password。
  - 明确 HTTP response 的 `next` 是接口建议字段，不是 CLI argv；例如 HTTP `next:"urgent_stop"` 需要由 CLI/skill 层映射为 `urgent-stop`。
- 更新 skill `references/output-contract.md`：
  - `matched` 加入 common success fields。
  - 写明 success fields 是常见字段，不是 exhaustive schema；命令可以追加字段。
  - 补充 `standby` 可能因未确认返回 `STANDBY_NOT_CONFIRMED`。
  - 补充 `idle-load` 只代表配置成功，不代表 idle 正在播放。
  - 补充 `idle-clear` 先发送 HTTP clear，再请求 ordinary `/standby`；成功必须确认 ordinary standby 且 idle config 已清空，active idle 不能作为成功；不能 standby 的状态可能让 CLI 失败，但 clear request 已发出。
  - 明确 HTTP response 的 `next` 是接口建议字段，不是 CLI argv；CLI/skill `next` token 使用 CLI 命名。
- 更新 ACCEPTANCE：
  - 增加 disputed 状态显式验收点，见“验收标准”。
  - 不把 manual gate、E2E、soak 设为默认发布 gate。

### 2. 独立最小代码修复

该交付可独立于纯文档 cleanup 合入。只修改 `et1-action` 中一个 standby 确认函数 `standby_confirmed()`；`standby` 使用该判断，`idle-clear` 使用同一拒绝原则但成功分支更窄：必须确认 ordinary standby 且 idle config 已清空，不能接受 active idle 分支。

建议确认规则：

```text
standby_confirmed(status) rejects first:
- false if ctrl in {"passive", "fault", "stopping", "urgent_stopping", "fixstand"};
- false if active.kind in {"user", "transition"};
- false if ctrl, active.kind, or required idle fields are missing, unknown, or ambiguous.

standby_confirmed(status) returns true only when:
- ctrl == "running" AND active.kind == "idle" AND idle.active == true; or
- ctrl in {"standby", "standby_velocity"} AND active.kind == "none".

idle_clear_confirmed(status) returns true only when:
- ctrl in {"standby", "standby_velocity"} AND active.kind == "none"; and
- idle config is proven cleared, for example idle.enabled == false and idle.n == 0.
```

说明：

- `ctrl:"running"` 且 `active.kind:"idle"`/`idle.active:true` 可确认为普通 standby 命令后 idle background 接管，符合当前产品语义；缺少 public `ctrl:"running"` 时不能仅凭 active idle 成功。
- `ctrl:"standby"` 或 `ctrl:"standby_velocity"` 且 `active.kind:"none"`，可确认纯普通 standby。
- `ctrl:"fixstand"` 不是 standby；即使无 active，也必须继续等待后续 standby 状态或返回未确认。
- `idle-clear` 必须证明 ordinary standby 且 idle config 已清空；不能只靠 active idle 确认，也不能返回 fixstand 瞬态作为成功。

### 3. 保持业务边界

- 不调整 HTTP runtime gate。
- 不改变 `/standby`、`/urgent_stop`、`/passive`、`/fixstand` 的实际优先级。
- 不让 agent 根据 `next` 自动做 passive recovery chain。
- 不把旧路由 `/stop`、`/standby_velocity` 恢复为成功路径；它们继续只返回 renamed/error 指引。

## 精准测试计划

### 单元/契约测试

新增重点只放在两个 skill 测试，覆盖本轮已确认的 CLI 成功误判风险。

- 在 `packaging/skills/et1-action/tests/test_et1_action.py` 增加 standby 序列测试：
  - fake tracker 状态序列：`fixstand(active none) -> standby(active none)`。
  - 执行 `et1-action standby`。
  - 断言不会在 `fixstand(active none)` 提前成功。
  - 断言最终输出 `state:"standby"` 或现有 standby success shape，且 `ctrl` 为 `standby`/`standby_velocity`。
- 在同一测试文件增加 `idle-clear` 复用序列测试：
  - fake tracker 状态序列同上。
  - 执行 `et1-action idle-clear`。
  - 断言最终 `state:"standby"`，`idle.enabled:false` 且 `idle.n:0`，不返回 `state:"fixstand"` 或 `ctrl:"fixstand"` 作为成功。
  - 断言请求顺序仍是 `POST /idle {"paths":[]}` then `POST /standby`。
- 增加/调整 output contract 文档测试：
  - success fields 文档包含 `matched`。
  - 文档明确 common/not exhaustive。
  - 文档包含 `STANDBY_NOT_CONFIRMED` 和 `idle-load` 配置语义。
  - 文档包含 HTTP `next` 不是 CLI argv 的说明。
- 增加/调整 README/ACCEPTANCE 文档断言：
  - non-empty `/idle` allow/deny 状态集完整。
  - passive 示例不包含默认密码字面量。
  - fault `/fixstand` recovery 条件被限定。

### API 行为回归

- 现有 API tests 保持覆盖 HTTP `/idle {"paths":[]}` 任意 controller 状态清空、non-empty `/idle` allow/deny、旧路由 renamed error 等契约；必要时只补文档断言或一条轻量 contract check，不新增 active idle/holding/transition/旧路由的重复 case。
- non-empty `/idle` accepted/rejected 状态以文档契约为准：
  - accepted：public standby、user preparing/running、active idle、holding、transition，因为 public `ctrl:"running"`。
  - rejected：starting/internal idle/passive/fixstand/stopping/urgent_stopping/fault。
  - gate order：blocked controller 先于 readiness/manual error 返回 controller conflict；controller 允许后才检查 readiness。
- passive 缺少 `--password` 不执行：
  - CLI `passive` 缺少 `--password` 时在 HTTP 前失败。
  - tracker records 不出现 `POST /passive`。
- 旧路由保持 renamed error：
  - 现有覆盖应继续证明 `POST /stop` 和 `POST /standby_velocity` 只返回 renamed/error 指引，不作为成功路径；不为它们新增多组重复 case。

## 验收标准

- README 中 `/idle` 摘要、状态矩阵和示例对 non-empty `/idle` 的 accepted/rejected 状态描述一致。
- Matrix 明确 non-empty `/idle` 的 allow/deny，不再靠“运行中”等模糊描述推断。
- README fault 行不会表达“任意 fault 可 `/fixstand` 恢复”；只允许 readiness OK 或 `bad_orientation` recovery 条件。
- README 不暴露默认 passive password 示例；skill 心智模型保持 operator 显式提供密码。
- `output-contract.md`：
  - common success fields 包含 `matched`。
  - 明确 fields common/not exhaustive。
  - 明确 `standby` 可能返回 `STANDBY_NOT_CONFIRMED`。
  - 明确 `idle-load` 成功不是正在播放。
  - 明确 HTTP `next` token 不是 CLI argv。
- README 和 `output-contract.md` 双处都明确 HTTP response `next` 不是 CLI argv，CLI/skill 层负责命名映射。
- `et1-action standby` 不会把 `ctrl:"fixstand"` 且 `active.kind:"none"` 当作 standby 成功。
- `et1-action idle-clear` 使用同一拒绝原则，并且必须证明 ordinary standby 且 idle config 已清空；不返回 fixstand 成功，也不把 active idle 当作清空成功。
- ACCEPTANCE 显式覆盖：
  - non-empty `/idle` accepted/rejected 状态。
  - non-empty `/idle` blocked controller 先于 readiness/manual error。
  - HTTP `/idle {"paths":[]}` 任意状态清空。
  - CLI `idle-clear` 先 clear 再 ordinary standby；成功必须确认 ordinary standby 且 idle config 已清空，active idle 不能作为成功；不能 standby 的状态可失败，不能写成任意状态成功。
  - passive 缺少 `--password` 时不执行。
  - 旧路由 `/stop`、`/standby_velocity` 只返回 renamed error。
- 默认发布 gate 不新增 manual gate、E2E 或 soak；这些仍是 opt-in 仪表。

## 风险与回滚

| 风险 | 影响 | 缓解 | 回滚 |
| --- | --- | --- | --- |
| 文档 wording 过度承诺 runtime 行为 | agent/operator 误判状态 | 使用 allow/deny 精确列表；避免“总是”“任意”等词 | 回滚对应文档段落，不影响 runtime |
| `standby_confirmed()` 收紧后 standby 更容易返回未确认 | CLI 可能在瞬态状态下多等或返回 `STANDBY_NOT_CONFIRMED` | 拒绝 unsafe/terminal、user/transition、字段不足状态；只接受 public `ctrl:"running"` 下的 active idle 或纯 standby；idle-clear 另证 ordinary standby + idle config cleared | 回滚单函数改动 |
| HTTP/CLI `next` 分层说明不足 | agent 可能把 `urgent_stop` 当 CLI argv | 在 README/skill output contract 双处说明映射边界 | 文档补丁即可 |
| 验收项膨胀为默认 gate | 发布变慢、主线噪声增加 | ACCEPTANCE 写明 disputed contract tests 是精准断言，manual/e2e/soak opt-in | 移除 gate 配置，不移除测试用例 |

## 不做事项

- 不新增 endpoint。
- 不改 `/idle` 为 run。
- 不重构 runtime 状态机。
- 不重构 password/auth/permission 系统。
- 不引入自动 passive 恢复。
- 不新增远程 payload、`.et1trk`、queue priority、transition profile。
- 不把 governance/reporting 做成主线功能。
- 不把 manual gate、E2E、soak 纳入默认发布 gate。
- 不为 `standby` 和 `idle-clear` 复制两套不一致的拒绝逻辑；`idle-clear` 只在成功分支额外收紧为 ordinary standby + idle config cleared。
- 不为测试工具本身增加二级验证；只补契约风险点的精准测试。

## 建议交付顺序

1. 先提交 README、output-contract、ACCEPTANCE 文档修正，确保团队先对契约达成一致。
2. 单独提交 `standby_confirmed()` 最小代码修复和两个 skill 精准测试；该提交可独立评审、独立回滚。
3. 最后跑 skill 单元测试与 API contract tests，人工 review 文档 wording 是否仍有“任意 fault”“默认密码”“HTTP next 等于 CLI argv”等误导。
