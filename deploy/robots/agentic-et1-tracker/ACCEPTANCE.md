# Agentic ET1 Tracker Acceptance Evidence

Date: 2026-05-29
Environment: local workspace `/home/galbot/works/et1`

## Build/test evidence

- `/tmp/agentic-et1-tracker-verify`: 10/10 passed,
  default `ONNX=OFF ROBOT=OFF`.
- `/tmp/agentic-et1-tracker-verify-onnx`: 11/11 passed,
  `ONNX=ON ROBOT=OFF`.
- `/tmp/agentic-et1-tracker-verify-robot`: 12/12 passed,
  `ONNX=ON ROBOT=ON`.
- `/tmp/agentic-et1-tracker-verify-perf`: perf smoke target compiled
  successfully.
- `git diff --check`: passed.

## Asset evidence

App-local release assets live under
`deploy/robots/agentic-et1-tracker/config` and must be used at runtime without
fallback to the ET1 app tree:

- GeneralTracker: `config/policy/general_tracker`
- StandbyVelocity: `config/policy/velocity/v0`
- FixStand posture: `config/posture/fixstand/v0/fixstand.yaml`
- Passive posture: `config/posture/passive/v0/passive.yaml`

## Manual MuJoCo acceptance

No fresh MuJoCo visual acceptance has been recorded for the current revision.
Earlier MuJoCo evidence predated the current FixStand/StandbyVelocity/FSM
semantics and is historical only.

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
- Queue FIFO, interrupt, and stop/cancel behavior.
- After `.trk` done or `/stop`, top-level `ctrl:"standby_velocity"`.
- Fault/disconnect handling and latency/performance evidence.

## GA gates

| gate | status | required evidence |
| --- | --- | --- |
| Build and hermetic tests | recorded above | Keep all default tests passing. |
| ROBOT/ONNX integration build | recorded above | Keep app and robot integration tests passing. |
| MuJoCo visual acceptance | pending | Record scenarios listed above with `config.sim.yaml.example`. |
| Real robot acceptance | pending | ET1 hardware/operator validation. |

Do not mark GA until the two pending external gates are complete.

## Remaining external pending

True real-robot validation is still pending and requires ET1 hardware and an
operator window.

## Worktree note

`source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` has an
unrelated pre-existing modification.
