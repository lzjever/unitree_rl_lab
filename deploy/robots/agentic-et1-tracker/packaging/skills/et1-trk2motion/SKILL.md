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
<skill-dir>/scripts/et1-trk2motion repeat /abs/file.trk -n 3
<skill-dir>/scripts/et1-trk2motion stop
<skill-dir>/scripts/et1-trk2motion fixstand
<skill-dir>/scripts/et1-trk2motion standby
```

`ready` handles `passive -> fixstand -> standby` and `fixstand -> standby`.
`run` defaults to `ready` first, then execute. Use `--recover off` only when
the caller intentionally manages control state.

Useful options:
`run PATH --mode interrupt|queue --wait --timeout S --poll S`
`repeat PATH -n N --verbose`

All CLI output is one compact JSON line. Prefer short output; do not paste full
pose/status arrays unless asked.

Final user reply after live control: action taken, run id if any, terminal state
if waited, and current `ctrl/ready/err`.

If high-level commands are not enough, read:
`references/output-contract.md`, `references/state-machine.md`, then
`references/raw-http.md`. Use `raw METHOD PATH [JSON]` instead of `curl`.
