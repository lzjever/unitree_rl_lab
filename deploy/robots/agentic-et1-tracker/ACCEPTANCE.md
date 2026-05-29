# Agentic ET1 Tracker Acceptance Evidence

Date: 2026-05-29
Environment: local workspace `/home/galbot/works/et1`

- Unitree MuJoCo simulator: installed under `/home/galbot/works/et1`.
- Test `.trk` files: under `/home/galbot/works/et1/generated/`.

## Build/test evidence

- Default build/test at `/tmp/agentic-et1-final-default`: 9/9 passed.
- ONNX+ROBOT build/test at `/tmp/agentic-et1-final-onnx-robot`: 11/11 passed.
- Perf smoke at `/tmp/agentic-et1-final-perf`: 1/1 passed, 7.10s.

## Asset evidence

App-local ONNX and deploy files exist, are non-symlinks, and have manifest/hash evidence.

- ONNX: sha256 `33ba3c2aedbfa296a4ac5c1b851eda4d34a21a8026e2f5bbcef2ef688188ccaa`, size `3831179`.
- `deploy.yaml`: sha256 `af5caf7f18a98beacdc8ed9f74105ba85a25a96870066f7f235d6d8c984cfb7a`, size `4403`.

## MuJoCo acceptance evidence

Preflight LowCmd ownership is scoped to the same DDS `network` and `domain_id`.
This run used `network=lo` and `domain_id=1`. The existing `et1_ctrl -n lo`
process was recorded as an isolation condition in domain 0, not as a same-domain
LowCmd owner for this MuJoCo acceptance run.

- `/health`: ready.
- `/status`: `mode:"sim", ready:true, ctrl:"idle"` before actions.
- FIFO: IDs `00000000`, `00000001`; early id1 running, id2 queued; final both done; top idle.
- Interrupt: id `00000002` interrupted by `00000003`; old state `stopping` with `stop_reason:"interrupt"`; after hold old `stopped`, new `running`.
- Stop-to-idle/cancel queued: active `00000003`, queued `00000004`; stop accepted; after hold active `stopped`, queued `canceled`, top idle.
- Post-stop new queue: stop active `00000000`, then new queue `00000001`; after hold old `stopped`, new `running`, then `done`, top idle.
- Disconnect: after stopping MuJoCo, `/status` had `block:"lowstate_timeout"`, `ready:false`; `/execute` rejected with `ROBOT_NOT_READY`; `/stop` accepted without claiming readiness.
- Real integration latency while running: `/health` avg `0.119ms`, p95 `0.153ms`; `/status` avg `0.158ms`, p95 `0.195ms`.

## Remaining external pending

True real-robot validation is still pending and requires ET1 hardware and an operator window. Do not mark GA runtime solely from this document.

## Worktree note

`source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` has an unrelated pre-existing modification. The new app and PRD are untracked in the current worktree.
