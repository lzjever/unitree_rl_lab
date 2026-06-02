# Agentic ET1 Tracker Acceptance Evidence

Date: 2026-05-29
Environment: local workspace `/home/galbot/works/et1`

## Build/test evidence

- Default configure/build is the real integration path:
  `AGENTIC_ET1_BUILD_ONNX=ON` and `AGENTIC_ET1_BUILD_ROBOT=ON`.
- Hermetic stub/test configure/build must be requested explicitly:
  `-DAGENTIC_ET1_BUILD_ONNX=OFF` and `-DAGENTIC_ET1_BUILD_ROBOT=OFF`.
- Single-option integration builds remain valid for narrow integration checks,
  but they are not the GA default server build.
- `/tmp/agentic-et1-tracker-verify-robot`: 12/12 passed with the real
  integration build.
- `/tmp/agentic-et1-tracker-verify-perf`: perf smoke target compiled
  successfully.
- `git diff --check`: passed.

## Docs/skill evidence

2026-06-02 docs/skill CLI checks only:

- `python3 deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion/tests/test_et1_trk2motion.py`:
  16/16 passed.
- `python3 /home/galbot/.agents/skills/et1-trk2motion/tests/test_et1_trk2motion.py`:
  16/16 passed.
- `python3 /home/galbot/.codex/skills/et1-trk2motion/tests/test_et1_trk2motion.py`:
  16/16 passed.
- `diff -rq deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion /home/galbot/.agents/skills/et1-trk2motion`:
  no differences.
- `diff -rq deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion /home/galbot/.codex/skills/et1-trk2motion`:
  no differences.

This evidence does not claim MuJoCo or real-robot acceptance.

## Asset evidence

App-local release assets live under
`deploy/robots/agentic-et1-tracker/config` and must be used at runtime without
fallback to the ET1 app tree:

- GeneralTracker: `config/policy/general_tracker`
- StandbyVelocity: `config/policy/velocity/v0`
- FixStand posture: `config/posture/fixstand/v0/fixstand.yaml`
- Passive posture: `config/posture/passive/v0/passive.yaml`

## GA idle/status/control acceptance items

Contract-level GA evidence must cover:

- `/execute` accepts only `path` and optional `mode`; extra fields and `paths`
  return 400 `REQUEST_INVALID` before validator, sink, or id allocation.
- `/idle {"paths":[...]}` atomically configures the idle pool; `{"paths":[]}`
  clears it; idle never creates a user run id.
- `/status.active.kind` is authoritative. `exec` and `queue` describe user runs
  only; idle progress lives under `idle` and is not queryable by
  `GET /status?id=...`.
- `/stop` stops idle, clears idle config, and preserves the user stop watermark
  so work accepted after the stop is not canceled by the older stop.
- `block:"lowcmd_occupied"` maps to manual/operator action and must not imply
  automatic wait/retry recovery.
- `bad_orientation` enters the safety path; `/fixstand` is only the software
  recovery exception when LowCmd is free.

## Manual MuJoCo acceptance

No fresh MuJoCo visual acceptance has been recorded for the current revision.
Earlier MuJoCo evidence predated the current FixStand/StandbyVelocity/FSM
semantics and is historical only. No idle/status/control MuJoCo acceptance has
been recorded in this docs/skill pass.

Use `config.sim.yaml.example` for local MuJoCo acceptance. It sets
`mode_machine: 0`, `network: "lo"`, app-owned policy/control assets, and
`motion_dirs: ["/home/galbot/works/et1/generated"]`. It keeps the startup
LowCmd owner preflight at the default `200` ms.

Pending MuJoCo evidence must record:

- Startup LowCmd owner preflight scoped to the same DDS `network` and
  `domain_id`; `lowcmd_occupied` must prevent any writing runtime from
  starting.
- Startup `/status` with `ctrl:"fixstand"` when using default config.
- `/status.pose` with compact `q/g/p/v` fields during idle and running states.
- Manual `/fixstand` and `/standby_velocity` requests accepted with empty body
  only when `/status` shows a ready runtime that can consume them.
- Static not-ready startup/model-load failure snapshots reject `/fixstand` and
  `/standby_velocity` with compact readiness errors and no queued control
  command.
- Operator-controlled MuJoCo rope/keyboard timing for track scenarios.
- `/execute` with a local allowed `.trk` path only; no uploads or other formats.
- `/execute` rejects `paths` and extra fields with 400 `REQUEST_INVALID`.
- `/idle` set/clear with at least two low-risk idle tracks.
- Idle auto-play shows `active.kind:"idle"`, `active.id:null`, `exec:null`,
  and unchanged user `queue`.
- `GET /status?id=<id>` works only for user run ids; idle is not queryable.
- Queue FIFO, interrupt, and stop/cancel behavior.
- `/stop` clears idle config while preserving stop-watermark behavior for user
  work accepted after the stop.
- After `.trk` done or `/stop`, top-level `ctrl:"standby_velocity"`.
- `lowcmd_occupied -> manual` and `bad_orientation -> passive` with FixStand
  recovery exception.
- Fault/disconnect handling and latency/performance evidence.

## GA gates

| gate | status | required evidence |
| --- | --- | --- |
| Default ROBOT/ONNX build | recorded above | Keep default configure/build on the real integration path. |
| Hermetic stub tests | recorded above | Keep explicit stub/test configure and tests passing. |
| Docs/skill CLI contract | recorded above | Keep packaged and installed skill tests/diffs passing. |
| MuJoCo visual acceptance | pending | Record scenarios listed above with `config.sim.yaml.example`. |
| Real robot acceptance | pending | ET1 hardware/operator validation. |

Do not mark GA until the two pending external gates are complete.

## Remaining external pending

True real-robot validation is still pending and requires ET1 hardware and an
operator window.
