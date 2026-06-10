---
name: et1-trk2motion
description: "Execute and control ET1 .trk robot motions through the local agentic-et1-tracker HTTP service. Use when Codex needs to run a .trk file on the ET1 simulator or robot, enter FixStand, enter StandbyVelocity, perform an explicit emergency stop, poll status, wait for a run, repeat a .trk, or make compact low-latency tracker calls for an LLM agent."
---

# ET1 TRK -> Motion

Use the bundled CLI by default. Do not write `curl` in the hot path.

Default tracker: `http://127.0.0.1:8083`; override with `ET1_TRACKER_URL`.

```bash
<skill-dir>/scripts/et1-trk2motion state
<skill-dir>/scripts/et1-trk2motion ready
<skill-dir>/scripts/et1-trk2motion run /abs/file.trk --wait
<skill-dir>/scripts/et1-trk2motion run /abs/file.trk --hold --wait
<skill-dir>/scripts/et1-trk2motion repeat /abs/file.trk -n 3
<skill-dir>/scripts/et1-trk2motion idle set /abs/idle-a.trk /abs/idle-b.trk
<skill-dir>/scripts/et1-trk2motion idle clear
<skill-dir>/scripts/et1-trk2motion passive --password "$ET1_PASSIVE_PASSWORD"
<skill-dir>/scripts/et1-trk2motion fixstand  # explicit stand configuration/control prep only
<skill-dir>/scripts/et1-trk2motion standby
<skill-dir>/scripts/et1-trk2motion stop --urgent  # direct urgent stop only
```

Never infer `stop` or `/stop` from status, impatience, or generic stop
language. Use `stop --urgent` only when the user directly and urgently asks for
stop/abort/kill/emergency stop/紧急停止/赶快停止. Normal requests to stand
still, stop moving, recover, relax, or stand quietly use `standby`
(`/standby_velocity`) and preserve idle config; do not clear idle unless the
user explicitly asks. For Chinese requests, general
"停下/停止/恢复/放松/静止站立/不要动/站着别动" also means `standby` with idle preserved
unless the user clearly asks for an urgent stop or explicit idle clear. The CLI
refuses `stop` without `--urgent` before HTTP.

Do not call `fixstand` for those ordinary intents. `fixstand` is a
fixed-configuration control step, not normal autonomous stand-still/velocity
control; the robot can lose autonomous standing/velocity control after it.
Only use it when the user explicitly asks to enter the stand
configuration/进入站立构型, or for an explicitly named control-state
recovery/preparation step.

`passive` is an explicit user command that stops active work and clears queued
work plus the idle pool. It posts `{"password":"..."}` to `/passive` and
requires `--password VALUE` or `ET1_PASSIVE_PASSWORD`; the CLI refuses before
HTTP when neither is provided. Never save or echo the password in user-visible
output.
`ready` is read-only. It polls status and returns `ok:true` only when the
tracker is already `standby_velocity` and ready; it never posts `/fixstand` or
`/standby_velocity`. After `passive`, never auto-restore: only run `fixstand`
when the user explicitly asks for that fixed-configuration step, then run
`standby` only after a separate explicit user request. If status is
`fixstand`, the user must explicitly call `standby`.
`run`, `exec`, and `repeat` post directly to `/execute` by default and do not call
`ready` or recover control state. If `/execute` rejects the current
`passive`/`fixstand` state, return the compact server error and `next` as-is.
Their default mode is `queue`; only use `--mode interrupt` when the user
explicitly asks to interrupt the current user action.
`run PATH --hold` sends `hold:true` to `/execute`; omitted `--hold` sends no
`hold` field.
`idle set` configures the idle pool only; it returns no run id. `idle clear`
posts `{"paths":[]}`. Idle never appears in user `exec/queue` or `status?id=`.
For "加载idle动作", "放松点", "别直挺挺站着", or similar requests that mean loading
idle behavior, find all `.trk` under the current working directory's
`preset-trk/idle/`, copy them into a tracker-readable motion directory (for
example `/home/galbot/works/agent-test/generated/`; prefer the
project/environment configured `generated`/`motion_dirs` location when
available), then run
`scripts/et1-trk2motion idle set /path/to/idle-a.trk /path/to/idle-b.trk` with
the copied absolute paths. This only sets the idle pool: it is not `/stop`, not
`fixstand`, and not `idle clear`. If the same wording means stop the current
action instead of loading idle, use `standby` and preserve existing idle
config. After success, briefly tell the user idle is set.

Useful options:
`run PATH --mode interrupt|queue --hold --wait --timeout S --poll S`
`exec PATH --mode interrupt|queue --wait`
`repeat PATH -n N --mode interrupt|queue --verbose`

All CLI output is one compact JSON line. Prefer short output; do not paste full
pose/status arrays unless asked. Short `state/status` includes compact
`active.kind/id`, `transition.active/target/target_id/frame/frames/progress`,
`idle.enabled/n/active/current/frame/frames/progress`,
`exec.id/state/frame/frames/progress`, and `queue.ids/n/limit`.
`active.kind` is `none|user|idle|transition`. Treat `active.kind=="user"` plus
`id` as the only waitable active run; `idle` and `transition` have `id:null`.
`run --hold --wait` and `wait ID` return `ok:true` when the run reaches
`state:"holding"`. Internal transitions do not enter user `queue`, consume
`queue.limit`, create a run id, or write user history.
Use `active.kind` plus `transition.target` to interpret `ctrl:"running"`:
idle/background work can be preempted by user `run`, while a transition whose
target is `user` owns that foreground target and queued user work waits.

`/stop` immediately aborts user, idle, holding, and transition work, clears the
idle pool, and does not play `standby_ref.trk`; this is why agents must reserve
it for direct urgent stop requests and must go through `stop --urgent`. Direct
`standby`/StandbyVelocity remains the normal way to leave motion and stand
still while preserving idle config.

Final user reply after live control: action taken, run id if any, terminal state
if waited, and current `ctrl/ready/err`.

If high-level commands are not enough, read:
`references/output-contract.md`, `references/state-machine.md`, then
`references/raw-http.md`. Use `raw METHOD PATH [JSON]` instead of `curl`.
