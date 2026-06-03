---
name: et1-trk2motion
description: "Execute and control ET1 .trk robot motions through the local agentic-et1-tracker HTTP service. Use when Codex needs to run a .trk file on the ET1 simulator or robot, enter FixStand, enter StandbyVelocity, stop motion, poll status, wait for a run, repeat a .trk, or make compact low-latency tracker calls for an LLM agent."
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
<skill-dir>/scripts/et1-trk2motion stop
<skill-dir>/scripts/et1-trk2motion passive
<skill-dir>/scripts/et1-trk2motion fixstand
<skill-dir>/scripts/et1-trk2motion standby
```

`passive` stops active work and clears queued work plus the idle pool.
`ready` handles `passive -> fixstand -> standby` and `fixstand -> standby`,
polling until convergence within `--timeout`.
`run` defaults to `ready` first, then execute. Use `--recover off` only when
the caller intentionally manages control state. `run PATH --hold` sends
`hold:true` to `/execute`; omitted `--hold` sends no `hold` field.
`idle set` configures the idle pool only; it returns no run id. `idle clear`
posts `{"paths":[]}`. Idle never appears in user `exec/queue` or `status?id=`.

Useful options:
`run PATH --mode interrupt|queue --hold --wait --timeout S --poll S`
`repeat PATH -n N --verbose`

All CLI output is one compact JSON line. Prefer short output; do not paste full
pose/status arrays unless asked. Short `state/status` includes compact
`active.kind/id`, `transition.active/target/target_id/frame/frames/progress`,
and `idle.enabled/n/active/current/frame/frames/progress`. `active.kind` is
`none|user|idle|transition`. Treat `active.kind=="user"` plus `id` as the only
waitable active run; `idle` and `transition` have `id:null`. `run --hold --wait`
and `wait ID` return `ok:true` when the run reaches `state:"holding"`.
Internal transitions do not enter user `queue`, consume `queue.limit`, create a
run id, or write user history.

`/stop` immediately aborts user, idle, holding, and transition work. The
`standby_ref.trk` asset is recorded, simulator accepted, and runtime-gated
internally; real-robot GA gates remain pending. Direct
`standby`/StandbyVelocity remains available.

Final user reply after live control: action taken, run id if any, terminal state
if waited, and current `ctrl/ready/err`.

If high-level commands are not enough, read:
`references/output-contract.md`, `references/state-machine.md`, then
`references/raw-http.md`. Use `raw METHOD PATH [JSON]` instead of `curl`.
