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

No fresh MuJoCo visual acceptance was run for this document update. Earlier
MuJoCo evidence in this file predated the FixStand/StandbyVelocity behavior and
must be treated as historical, not acceptance of the current control-mode
transitions.

Pending manual acceptance should record:

- Preflight LowCmd ownership scoped to the same DDS `network` and `domain_id`.
- Startup `/status` with `ctrl:"fixstand"` when using default config.
- Manual `/fixstand` and `/standby_velocity` requests accepted with empty body.
- Operator-controlled MuJoCo rope/keyboard timing for track scenarios.
- `/execute` with a local allowed `.trk` path only; no uploads or other formats.
- Queue FIFO, interrupt, and stop/cancel behavior.
- After `.trk` done or `/stop`, top-level `ctrl:"standby_velocity"`.
- Fault/disconnect handling and latency/performance evidence.

## Remaining external pending

True real-robot validation is still pending and requires ET1 hardware and an operator window. Do not mark GA runtime solely from this document.

## Worktree note

`source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` has an
unrelated pre-existing modification.
