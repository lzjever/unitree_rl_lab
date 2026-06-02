# agentic-et1-tracker

## Build Matrix

The real integration server binary requires both integration options
(`BUILD_ONNX=ON` / `BUILD_ROBOT=ON` in shorthand):

```sh
cmake -S unitree_rl_lab/deploy/robots/agentic-et1-tracker \
  -B build-agentic-et1-tracker-onnx-robot \
  -DAGENTIC_ET1_BUILD_ONNX=ON \
  -DAGENTIC_ET1_BUILD_ROBOT=ON
```

Only this `ONNX=ON` and `ROBOT=ON` combination links the real runtime factory,
ONNX policy runtime, and Unitree SDK robot I/O into `agentic-et1-tracker`.

`AGENTIC_ET1_BUILD_ONNX=ON` by itself builds ONNX policy integration targets.
`AGENTIC_ET1_BUILD_ROBOT=ON` by itself builds robot I/O integration targets.
Either single-option build is useful for integration testing, but it is not a
ready real integration server binary.

Opt-in performance smoke tests are excluded from default builds:

```sh
cmake -S unitree_rl_lab/deploy/robots/agentic-et1-tracker \
  -B build-agentic-et1-tracker-perf-smoke \
  -DAGENTIC_ET1_BUILD_PERF_SMOKE=ON
cmake --build build-agentic-et1-tracker-perf-smoke \
  --target agentic_et1_tracker_perf_smoke_tests
ctest --test-dir build-agentic-et1-tracker-perf-smoke -L perf --output-on-failure
```

## Testing Notes

Default core tests must remain hermetic: they use fake/stub dependencies and do
not require MuJoCo, Unitree SDK2, or ONNX Runtime.

## HTTP Contract

`agentic-et1-tracker` is a local HTTP API for LLM agents. Keep calls short.

Commands:

- `GET /health`: `{"ok":bool,"state":"starting|ready|error","mode":"sim|real|unknown"}`
- `GET /status`: full runtime state.
- `GET /status?id=<id>`: one run plus compact runtime context.
- `POST /execute`: `{"path":"/absolute/file.trk","mode":"queue|interrupt"}`.
  `mode` is optional and defaults to `queue`.
- `POST /stop`: empty body; stops active work and cancels queued work.
- `POST /fixstand`: empty body; enter FixStand.
- `POST /standby_velocity`: empty body; enter StandbyVelocity.

`/execute` accepts local `.trk` paths allowed by `motion_dirs` only. It does not
accept uploads, non-`.trk` files, or embedded motion payloads.
Control-changing routes return the current `/status.err` readiness error before
claiming success when the runtime is unavailable or not ready. `/fixstand`
remains available from `passive` and `fault` recovery states; `/standby_velocity`
returns `CONTROL_STATE_CONFLICT` from those states.

Startup safety:

- Real Unitree SDK startup subscribes to `rt/lowcmd` on the configured
  `network/domain_id`, waits `lowcmd_startup_preflight_ms` (default `200`), and
  refuses to start the writing runtime if a fresh external LowCmd owner exists.
- Startup owner conflict status is compact: `ready:false`,
  `block:"lowcmd_occupied"`, `err.code:"ROBOT_NOT_READY"`.

Error envelope:

```json
{"ok":false,"error":{"code":"CONTROL_STATE_CONFLICT","message":"ctrl=fixstand; /standby_velocity","retryable":false},"next":"standby_velocity"}
```

`next` is one token-level action: `status`, `retry`, `wait_robot`, `fix`,
`fixstand`, `standby_velocity`, `stop`, or `manual`.

Common error handling:

| code | next | agent action |
| --- | --- | --- |
| `REQUEST_INVALID` | `fix` | Fix JSON/body/path/mode and retry. |
| `SERVICE_NOT_READY` | `status` | Poll `/status`. |
| `ROBOT_DISCONNECTED` | `wait_robot` | Wait; poll `/status`. |
| `ROBOT_NOT_READY` | `wait_robot` | Wait or operator action; poll `/status`. |
| `ROBOT_BAD_ORIENTATION` | `manual` | Operator fixes pose, then `/fixstand`. |
| `MODEL_NOT_READY` | `status` | Poll `/health` or `/status`. |
| `TRK_*` | `fix` | Use an allowed existing `.trk`. |
| `QUEUE_FULL` | `status` | Poll `/status`; retry after queue drains. |
| `RUN_NOT_FOUND` | `status` | Use a valid id or full `/status`. |
| `CONTROL_STATE_CONFLICT` | varies | Follow `error.message`; it names the current `ctrl` and the next route. |
| `SAFETY_LIMIT_TRIGGERED` | `manual` | Operator resolves; then `/fixstand`. |
| `INTERNAL_ERROR` | `manual` | Inspect service logs. |

Status schemas:

```json
// GET /status
{"ok":true,"ready":true,"mode":"sim","robot":"idle","ctrl":"standby_velocity","stop_reason":null,"hz":1000,"exec":null,"queue":{"n":0,"limit":8,"ids":[]},"low_ms":0,"block":null,"err":null,"pose":{"q":[1,0,0,0],"g":[0,0,0],"p":null,"v":null}}

// GET /status?id=<id>
{"ok":true,"id":"a7K3p9Qx","state":"queued","frame":0,"frames":120,"time_s":0,"duration_s":2.4,"progress":0,"stop_reason":null,"err":null,"path":"/absolute/file.trk","robot":"holding","ctrl":"fixstand","ready":false,"block":"operator_wait","queue_pos":1,"top_err":"ROBOT_NOT_READY"}
```

`queue_pos` is 1-based and `null` when the run is not queued. `top_err` is the
full `/status.err` code or `null`.
`pose` is intentionally small for frequent polling: `q` is lowstate quaternion
`[w,x,y,z]`, `g` is lowstate gyro `[x,y,z]`, `p` is highstate position or
`null`, and `v` is highstate linear velocity or `null`.

Controller states:

| ctrl | robot behavior | accepts | rejects/notes |
| --- | --- | --- | --- |
| `starting` | Runtime is initializing. | `/status`, `/health` | Control routes return readiness errors such as `SERVICE_NOT_READY` or `MODEL_NOT_READY`; wait for `ready:true`. |
| `passive` | Safety sink; publishes passive damping command. | `/fixstand`, `/stop` | `/execute` and `/standby_velocity` return `CONTROL_STATE_CONFLICT`. |
| `fixstand` | Holds configured stand posture. | `/standby_velocity`, `/stop`, `/fixstand` | `/execute` returns conflict; call `/standby_velocity` first. |
| `standby_velocity` | Velocity policy with zero command; robot stands idle. | `/execute`, `/fixstand`, `/stop` | Normal state for starting `.trk`. |
| `idle` | Tracker idle without full velocity runtime, mainly tests/stubs. | `/execute`, `/fixstand`, `/stop` | Real GA runtime normally uses `standby_velocity`. |
| `preparing` | Runtime is preparing a run. | `/stop`, `/execute` queue/interrupt | Poll `/status?id=<id>`. |
| `running` | Executing a `.trk` through GeneralTracker. | `/stop`, `/execute` queue/interrupt | `queue` waits; `interrupt` preempts current run. |
| `stopping` | Stop/interrupt transition to StandbyVelocity. | `/status`, `/stop` | `/execute` returns conflict; wait for `ctrl:"standby_velocity"`. |
| `fault` | Hard safety fault; no normal track execution. | `/fixstand`, `/stop` | `/execute` and `/standby_velocity` return conflict until resolved. |

`/execute` checks request shape, then readiness, then controller state. If
`starting` is not ready it returns the readiness error, not
`CONTROL_STATE_CONFLICT`. `/execute` does not enqueue in `passive`, `fixstand`,
`stopping`, or `fault`; those ready controller states return
`CONTROL_STATE_CONFLICT`. In `running` and `preparing`, queue and interrupt
requests are accepted because the runtime can process them without manual
control-state steps.

Startup defaults to FixStand. After a `.trk` run finishes or `/stop` completes,
the real runtime returns to StandbyVelocity. The app does not automatically
drive MuJoCo rope timing, keyboard controls, or other simulator-side actions;
those remain manual operator actions during MuJoCo acceptance.

## App-Owned Release Assets

Release policy/control assets are owned by `agentic-et1-tracker` and live under
`deploy/robots/agentic-et1-tracker/config`:

- GeneralTracker policy: `config/policy/general_tracker`
- StandbyVelocity policy: `config/policy/velocity/v0`
- FixStand posture: `config/posture/fixstand/v0/fixstand.yaml`
- Passive posture: `config/posture/passive/v0/passive.yaml`

Runtime configuration must point at these app-local release assets. Runtime
does not fall back to the ET1 app tree under `deploy/robots/et1`.

For manual or integration simulation testing in this workspace, the installed
Unitree MuJoCo simulator under `/home/galbot/works/et1` can be used. Test
`.trk` files are available under `/home/galbot/works/et1/generated/`.

These paths are environment notes for simulation acceptance only. They do not
change the HTTP input contract: `agentic-et1-tracker` accepts local `.trk` paths
only, not uploads or other motion formats.

Simulation configs may enable the hidden debug endpoint
`GET /_sim/reference_frame` with `reference.enabled: true` and
`mode_machine: 0`. It returns the latest raw `.trk` reference frame held in
memory for MuJoCo/debug consumers, or `{"ok":true,"active":false}` when no
track is active. This endpoint is sim-only, default-off, and intentionally not
part of the agent-facing command contract.

## Manual MuJoCo Acceptance

This is a manual integration acceptance skeleton, not a complete GA simulation
evidence script and not evidence that acceptance has already been completed.
Complete GA simulation evidence still needs recorded queue FIFO, interrupt,
stop-to-standby_velocity, `/fixstand`, `/standby_velocity`, fault/disconnect,
and performance results.

Prerequisites:

- Preflight: before acceptance, confirm there is no old `et1_ctrl`,
  `unitree_mujoco`, `agentic-et1-tracker`, or other LowCmd owner on the same
  DDS `network` and `domain_id`. Existing control processes on a different DDS
  domain must be recorded as isolation conditions, not treated as same-domain
  owners.
- Use the app-owned release assets under
  `deploy/robots/agentic-et1-tracker/config`. Do not point the new app at the
  ET1 app policy tree.
- `policy.deploy` must live under `policy.policy_dir/params`, and `lock_path`
  must be absolute when explicitly configured.
- Use `config.sim.yaml.example` for local MuJoCo acceptance. It sets
  `mode_machine: 0`, `network: "lo"`,
  `motion_dirs: ["/home/galbot/works/et1/generated"]`, and app-owned assets.
  It also keeps `lowcmd_startup_preflight_ms: 200` and enables the hidden
  sim-only reference endpoint. Adjust only `domain_id` or `port` when the local
  simulator requires it.
- Start the Unitree MuJoCo simulator only after the Preflight is clear. The
  operator controls MuJoCo rope timing and keyboard actions manually.

Command skeleton:

```sh
# terminal 1: start MuJoCo from the local simulator install
/home/galbot/works/et1/unitree_mujoco/simulate/build/unitree_mujoco

# terminal 2: start the new app with the simulation config
agentic-et1-tracker \
  --config deploy/robots/agentic-et1-tracker/config.sim.yaml.example

# terminal 3: exercise the HTTP contract
TRK=$(find /home/galbot/works/et1/generated -maxdepth 1 -name '*.trk' | head -n 1)
curl http://127.0.0.1:8083/health
curl http://127.0.0.1:8083/status
curl -X POST http://127.0.0.1:8083/fixstand
curl -X POST http://127.0.0.1:8083/standby_velocity
curl -X POST http://127.0.0.1:8083/execute \
  -H 'Content-Type: application/json' \
  -d "{\"path\":\"$TRK\"}"

# manually control MuJoCo rope/keyboard timing as needed for the scenario

# poll status until frame progress is visible, then stop
curl http://127.0.0.1:8083/status
curl -X POST http://127.0.0.1:8083/stop

# after done or stop, the top-level controller should be standby_velocity
curl http://127.0.0.1:8083/status
```
