# agentic-et1-tracker

## Build Matrix

The default build is the real integration server binary. It links the ONNX
policy runtime and Unitree SDK robot I/O unless explicitly disabled:

```sh
cmake -S unitree_rl_lab/deploy/robots/agentic-et1-tracker \
  -B build-agentic-et1-tracker
```

Use the stub/test runtime only when requested explicitly:

```sh
cmake -S unitree_rl_lab/deploy/robots/agentic-et1-tracker \
  -B build-agentic-et1-tracker-stub \
  -DAGENTIC_ET1_BUILD_ONNX=OFF \
  -DAGENTIC_ET1_BUILD_ROBOT=OFF
```

Single-option builds remain available for narrow integration work:
`AGENTIC_ET1_BUILD_ONNX=OFF` uses the policy stub, and
`AGENTIC_ET1_BUILD_ROBOT=OFF` uses the robot I/O stub.

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

Hermetic core tests must use the explicit stub/test runtime flags shown above;
that build does not require MuJoCo, Unitree SDK2, or ONNX Runtime.

Manual product gates are opt-in development/acceptance checks and are not part
of default `ctest`, release selftest, or packaging gates:

```sh
tools/manual_gate.py e2e --url http://127.0.0.1:8083 \
  --motion-dir /absolute/dir/listed/in/motion_dirs
tools/manual_gate.py visual --url http://127.0.0.1:8083 \
  --motion-dir /absolute/dir/listed/in/motion_dirs
```

By default the script only connects to an already running tracker and MuJoCo
session. Use `--start-tracker` or `--start-mujoco-cmd` only for explicit manual
sessions; the script stops only processes it started. `visual` writes a MuJoCo
screenshot and a JSON checklist report under `/tmp/agentic-et1-manual-gate` by
default.

For full local simulation acceptance, use an explicit temporary tracker config
with `--start-tracker --enable-loco-temp --require-loco`. The generated config
keeps release defaults untouched and lowers only the temporary runtime `hz` to
make short transition/stopping HTTP windows observable.

## HTTP Contract

`agentic-et1-tracker` is a local HTTP API for LLM agents. Keep calls short.

Commands:

- `GET /health`: `{"ok":bool,"state":"starting|ready|error","mode":"sim|real|unknown","cap":{"loco_upper":{"enabled":bool,"ready":bool,"default_radius_m":number,"max_radius_m":number,"strict_pose":bool}}}`
- `GET /status`: full runtime state.
- `GET /status?id=<id>`: one run plus compact runtime context.
- `POST /execute`: `{"path":"/absolute/file.trk","mode":"queue|interrupt","hold":true}`.
  Only `path`, optional `mode`, and optional boolean `hold` are allowed;
  `mode` defaults to `queue`, and omitted `hold` behaves as `false`.
  The packaged `et1-action` skill intentionally defaults `run-text` and
  `run-trk` to `mode:"interrupt"` for new user intent; that is a skill
  product decision, not the raw HTTP default.
- `POST /execute_loco_upper`:
  `{"path":"/absolute/file.trk","mode":"queue|interrupt","hold":true,"max_radius_m":0.8}`.
  Same local `.trk` allowlist contract as `/execute`, but routes the motion
  through the loco-upper executor when `loco_upper.enabled: true` and runtime
  `cap.loco_upper.ready: true`. `max_radius_m` is optional and is the
  per-request bounded execution radius. If omitted, the service uses
  `cap.loco_upper.default_radius_m`; if a finite positive request exceeds
  `cap.loco_upper.max_radius_m`, the effective radius is capped to that
  deployment maximum. Raw TRK root paths beyond the effective radius are
  projected/clamped and accepted. The loco radius guard applies while the
  loco-upper executor itself is active in `entry`, `motion`, `holding`, and
  bounded `exit`; at the boundary it suppresses outward radial velocity and
  reports `radius_limit_reached` without entering passive/fault. `/stop` still
  cancels that executor immediately. The sim example keeps this disabled by
  default; turn it on manually in `config.sim.yaml.example` only when testing
  `/execute_loco_upper`.
  The service config key remains `default_max_radius_m`; status/health expose
  the same client-facing default as `cap.loco_upper.default_radius_m`.
- `POST /idle`: `{"paths":["/absolute/idle-a.trk","/absolute/idle-b.trk"]}`.
- `POST /idle`: `{"paths":[]}` clears the idle pool.
- `POST /stop`: empty body; urgent/immediate software stop. It stops active
  work, cancels queued work, and clears idle config. Use `/standby_velocity`
  for ordinary stop/standby when idle config should be preserved.
- `POST /passive`: `{"password":"galaxy"}` by default; enter Passive safety
  sink when LowCmd is free; stops active work, clears queued work, and clears
  the idle pool. Configure the password with top-level `passive_password`.
- `POST /fixstand`: empty body; enter fixed stand configuration/recovery. This
  is not ordinary quiet standing.
- `POST /standby_velocity`: empty body; enter ordinary StandbyVelocity/Velocity0
  standby while preserving idle config.

`/execute` accepts local `.trk` paths allowed by `motion_dirs` only. It rejects
uploads, non-`.trk` files, embedded motion payloads, `paths`, and any JSON
field other than `path`, `mode`, or `hold` with 400 `REQUEST_INVALID` before
path validation, command sinks, or run id allocation. `hold` must be boolean.
Idle playback yields to user `/execute`; the accepted user run still receives
the only waitable run id. With `hold:true`, a user run that reaches its last
frame enters `state:"holding"` and keeps the same run id queryable through
`GET /status?id=<id>` until another user run or a control command releases it.

`/idle` is a config endpoint, not a run submission endpoint. It atomically
replaces the idle pool after validating every path with the same local `.trk`
rules as `/execute`; any failed path leaves the old idle config unchanged.
`{"paths":[]}` clears the config without path validation and is accepted in any
controller state. Non-empty `/idle` is accepted only after the control chain has
reached StandbyVelocity, or while a user motion is preparing/running. Idle does
not use `queue.limit`, `queue.ids`, `exec`, or `GET /status?id=...`.

`/standby_velocity` is the ordinary stop/standby command. It does not create a
user run and does not clear idle config; with idle motions loaded, background
idle may restart after the runtime returns to the standby chain.

`/stop` is highest priority for active work and is reserved for urgent/immediate
software stop semantics. It stops idle playback, immediately aborts holding runs
and internal transitions, cancels queued work, and clears the idle config. It
does not play `standby_ref.trk`. It preserves the stop watermark: user
queue/interrupt requests accepted after a pending stop are not canceled by that
older stop. For loco-upper this is an immediate cancellation path: runtime
clears the active loco executor state and returns through the existing
stopping/standby bookkeeping without further loco radius-guard ticks.

Control-changing routes return the current `/status.err` readiness error before
claiming success when the runtime is unavailable or not ready. Passworded
`/passive` and `/fixstand` are the software exceptions for
`block:"bad_orientation"` when LowCmd is not externally occupied; neither
bypasses `lowcmd_occupied`.
`/standby_velocity` returns `CONTROL_STATE_CONFLICT` from `passive` and `fault`.

Startup safety:

- Real Unitree SDK startup calls Unitree MotionSwitcher release first when
  `mode_machine: 1` and `release_motion_mode_on_startup: true` (default for
  robot config). It is not called for `mode_machine: 0` sim config.
- After that handoff, startup subscribes to `rt/lowcmd` on the configured
  `network/domain_id`, waits `lowcmd_startup_preflight_ms` (default `200`), and
  refuses to start the writing runtime if a fresh external LowCmd owner exists.
- Startup owner conflict status is compact: `ready:false`,
  `block:"lowcmd_occupied"`, `err.code:"ROBOT_NOT_READY"`.
- MotionSwitcher release failure status is compact: `ready:false`,
  `block:"motion_mode_release_failed"`, `err.code:"ROBOT_NOT_READY"`.
- Any API response caused by `block:"lowcmd_occupied"` uses `next:"manual"`;
  agents must not wait/retry to auto-reclaim LowCmd.

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
| `ROBOT_NOT_READY` | `wait_robot` or `manual` | If `block:"lowcmd_occupied"`, operator/manual only; otherwise wait and poll `/status`. |
| `ROBOT_BAD_ORIENTATION` | `manual` | Passworded `/passive` and `/fixstand` are software recovery routes only when `block:"bad_orientation"` and LowCmd is free; otherwise operator action. |
| `MODEL_NOT_READY` | `status` | Poll `/health` or `/status`. |
| `TRK_*` | `fix` | Use an allowed existing `.trk`. |
| `QUEUE_FULL` | `status` | Poll `/status`; retry after queue drains. |
| `RUN_NOT_FOUND` | `status` | Use a valid id or full `/status`. |
| `CONTROL_STATE_CONFLICT` | varies | Follow `error.message`; it names the current `ctrl` and the next route. |
| `SAFETY_LIMIT_TRIGGERED` | `manual` | True low-level safety fault or fall/orientation risk only; radius clamp/limit is not this error. |
| `INTERNAL_ERROR` | `manual` | Inspect service logs. |

Status schemas:

```json
// GET /status
{"ok":true,"ready":true,"mode":"sim","robot":"idle","ctrl":"standby_velocity","stop_reason":null,"hz":1000,"active":{"kind":"none","id":null},"exec":null,"queue":{"n":0,"limit":8,"ids":[]},"transition":{"active":false,"target":null,"target_id":null,"target_state":null,"frame":0,"frames":0,"progress":0},"idle":{"enabled":true,"n":2,"active":false,"current":null,"frame":0,"frames":0,"time_s":0,"duration_s":0,"progress":0},"low_ms":0,"block":null,"err":null,"pose":{"q":[1,0,0,0],"g":[0,0,0],"p":null,"v":null},"cap":{"loco_upper":{"enabled":true,"ready":true,"default_radius_m":0.8,"max_radius_m":2.0,"strict_pose":false}}}

// GET /status while idle is playing
{"ok":true,"ready":true,"mode":"sim","robot":"running","ctrl":"running","stop_reason":null,"hz":1000,"active":{"kind":"idle","id":null},"exec":null,"queue":{"n":0,"limit":8,"ids":[]},"transition":{"active":false,"target":null,"target_id":null,"target_state":null,"frame":0,"frames":0,"progress":0},"idle":{"enabled":true,"n":2,"active":true,"current":0,"frame":12,"frames":120,"time_s":0.24,"duration_s":2.4,"progress":0.1},"low_ms":0,"block":null,"err":null,"pose":{"q":[1,0,0,0],"g":[0,0,0],"p":null,"v":null},"cap":{"loco_upper":{"enabled":true,"ready":true,"default_radius_m":0.8,"max_radius_m":2.0,"strict_pose":false}}}

// GET /status while a held user run owns the reference
{"ok":true,"ready":true,"mode":"sim","robot":"running","ctrl":"running","stop_reason":null,"hz":1000,"active":{"kind":"user","id":"a7K3p9Qx"},"exec":{"id":"a7K3p9Qx","state":"holding","frame":119,"frames":120,"time_s":2.38,"duration_s":2.4,"progress":1,"hold":true,"stop_reason":null,"err":null},"queue":{"n":0,"limit":8,"ids":[]},"transition":{"active":false,"target":null,"target_id":null,"target_state":null,"frame":0,"frames":0,"progress":0},"idle":{"enabled":true,"n":2,"active":false,"current":null,"frame":0,"frames":0,"time_s":0,"duration_s":0,"progress":0},"low_ms":0,"block":null,"err":null,"pose":{"q":[1,0,0,0],"g":[0,0,0],"p":null,"v":null},"cap":{"loco_upper":{"enabled":true,"ready":true,"default_radius_m":0.8,"max_radius_m":2.0,"strict_pose":false}}}

// GET /status during an internal transition to a user run
{"ok":true,"ready":true,"mode":"sim","robot":"running","ctrl":"running","stop_reason":null,"hz":1000,"active":{"kind":"transition","id":null},"exec":null,"queue":{"n":0,"limit":8,"ids":[]},"transition":{"active":true,"target":"user","target_id":"b8L4s0Ry","target_state":"queued","frame":8,"frames":25,"progress":0.32},"idle":{"enabled":true,"n":2,"active":false,"current":null,"frame":0,"frames":0,"time_s":0,"duration_s":0,"progress":0},"low_ms":0,"block":null,"err":null,"pose":{"q":[1,0,0,0],"g":[0,0,0],"p":null,"v":null},"cap":{"loco_upper":{"enabled":true,"ready":true,"default_radius_m":0.8,"max_radius_m":2.0,"strict_pose":false}}}

// GET /status?id=<id>
{"ok":true,"id":"a7K3p9Qx","state":"holding","frame":119,"frames":120,"time_s":2.38,"duration_s":2.4,"progress":1,"hold":true,"stop_reason":null,"err":null,"path":"/absolute/file.trk","robot":"running","ctrl":"running","ready":true,"block":null,"queue_pos":null,"top_err":null}
```

`active.kind` is authoritative and is one of `none`, `user`, `idle`, or
`transition`. `user` with a non-null `id` is the only waitable active run;
`state:"holding"` is a user run state and should be treated as a successful
held state by hold-aware clients. `idle` and `transition` both have `id:null`;
`exec` and `queue` only describe user runs. Idle progress lives under `idle`
and is not queryable with `GET /status?id=...`. Internal synthetic transitions
live under `transition`, do not enter `queue`, do not consume `queue.limit`, do
not create a run id, and are not written to user run history.
`exec.hold` is present in run status and mirrors the accepted `/execute` hold
flag. `transition.target_state` is always present: `null` when no target state
applies, `queued` for a user transition target, or `running` for an idle
transition target.
`queue_pos` is 1-based and `null` when the user run is not queued. `top_err` is
the full `/status.err` code or `null`.
`pose` is intentionally small for frequent polling: `q` is lowstate quaternion
`[w,x,y,z]`, `g` is lowstate gyro `[x,y,z]`, `p` is highstate position or
`null`, and `v` is highstate linear velocity or `null`.

Short loco-upper handoff entry:

```yaml
agentic_et1_tracker:
  loco_upper:
    enabled: true
    policy_dir: "config/policy/loco_lower/et1_low"
    policy_file: "policy.onnx"
    deploy: "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"
    limits: "config/limits/et1_upper_body/v0/limits.yaml"
    joint_map: "config/limits/et1_upper_body/v0/joint_map.yaml"
```

```sh
curl -sS http://127.0.0.1:8083/health
curl -sS -X POST http://127.0.0.1:8083/execute_loco_upper \
  -H 'Content-Type: application/json' \
  -d '{"path":"/absolute/walk-wave.trk","mode":"queue","max_radius_m":0.8}'
curl -sS 'http://127.0.0.1:8083/status?id=<run_id>'
```

- Readiness/status fields:
  `cap.loco_upper.enabled`, `cap.loco_upper.ready`,
  `cap.loco_upper.default_radius_m`, `cap.loco_upper.max_radius_m`,
  `cap.loco_upper.strict_pose`.
- `GET /status?id=<run_id>` for a loco-upper run also returns
  `executor:"loco_upper"` and
  `loco.{max_radius_m,distance_m,radius_source,phase,radius_clamped,radius_limit_reached,envelope_clamped,upper_clamped,upper_rate_limited,raw_action_clamped,lower_q_limited,lower_action_clamped,reason}`.
- For loco-upper, `loco.max_radius_m` is the effective request/default radius
  after service max capping. `radius_clamped` means the raw TRK root path was
  projected into that radius. `radius_limit_reached` means runtime suppressed
  outward radial velocity at the boundary; it is not a failed/passive state and
  does not imply `loco.reason:"radius_limit"`. `upper_clamped` and
  `upper_rate_limited` report bounded upper-body intervention.
- Difference from old `/execute`: `/execute` stays on the existing
  GeneralTracker path and does not accept `max_radius_m` or return the loco
  run payload.

Controller states:

| ctrl | robot behavior | accepts | rejects/notes |
| --- | --- | --- | --- |
| `starting` | Runtime is initializing. | `/status`, `/health` | Control routes return readiness errors such as `SERVICE_NOT_READY` or `MODEL_NOT_READY`; wait for `ready:true`. |
| `passive` | Safety sink; publishes passive damping command; idle pool is cleared. | `/fixstand`, passworded `/passive`, `/stop`, `/idle {"paths":[]}` | `/execute`, `/standby_velocity`, and non-empty `/idle` return `CONTROL_STATE_CONFLICT`. If `block:"lowcmd_occupied"`, next action is `manual`. |
| `fixstand` | Holds configured stand posture. | `/standby_velocity`, passworded `/passive`, `/stop`, `/fixstand`, `/idle {"paths":[]}` | `/execute` and non-empty `/idle` return conflict; call `/standby_velocity` first. Passworded `/passive` and `/fixstand` are the `bad_orientation` software recovery exceptions. |
| `standby_velocity` | Velocity policy with zero command; robot stands idle. | `/execute`, `/idle`, passworded `/passive`, `/fixstand`, `/stop` | Normal state for starting user `.trk`; idle auto-play may start only when ready/safe and no user active/queue exists. |
| idle active (`active.kind:"idle"`) | GeneralTracker plays an idle pool motion without a user id. | `/execute`, `/stop`, `/idle`, passworded `/passive`, `/fixstand`, `/standby_velocity` | `exec:null`, user `queue` unchanged. User `/execute` preempts idle playback but keeps idle config. |
| `preparing`/`running` with `active.kind:"user"` | Preparing or executing a user `.trk`. | `/stop`, passworded `/passive`, `/execute` queue/interrupt, `/idle` config/clear | `queue` waits; `interrupt` preempts current user run. Poll `/status?id=<id>`. |
| holding (`active.kind:"user"`, `exec.state:"holding"`) | Holds the last reference frame of a user `.trk` submitted with `hold:true`. | `/execute` queue/interrupt, `/stop`, passworded `/passive`, `/fixstand`, `/standby_velocity`, `/idle` config/clear | Same user id remains queryable. `/stop`, passworded `/passive`, and `/fixstand` end it immediately; no `standby_ref.trk` playback. |
| transition active (`active.kind:"transition"`) | Internal synthetic reference transition toward `transition.target`. | `/execute` queue/interrupt, `/stop`, passworded `/passive`, `/fixstand`, `/standby_velocity`, `/idle` config/clear | Not a user run; no id, queue entry, queue limit use, or user history entry. `/stop` aborts immediately. |
| `stopping` | Stop/interrupt transition to StandbyVelocity or a safety state. | `/status`, passworded `/passive`, `/stop`, `/idle {"paths":[]}` | `/execute` and non-empty `/idle` return conflict at the HTTP API; wait for a stable state. |
| `fault` | Safety/manual state; no normal track execution. | passworded `/passive` only for `bad_orientation`, `/fixstand`, `/stop`, `/idle {"paths":[]}` | `/execute`, `/standby_velocity`, and non-empty `/idle` return conflict until resolved. `lowcmd_occupied` remains manual/operator. |

`/execute` checks request shape, then readiness, then controller state. If
`starting` is not ready it returns the readiness error, not
`CONTROL_STATE_CONFLICT`. `/execute` does not enqueue in `idle`, `passive`,
`fixstand`, `stopping`, or `fault`; those ready controller states return
`CONTROL_STATE_CONFLICT`. In `running` and `preparing`, queue and interrupt
requests are accepted because the runtime can process them without manual
control-state steps.

Startup defaults to FixStand. After a `.trk` run finishes or `/stop` completes
from active motion, the real runtime returns to StandbyVelocity. `/stop` in
Passive remains Passive, and `/stop` in idle FixStand remains FixStand. The app
does not automatically drive MuJoCo rope timing, keyboard controls, or other
simulator-side actions; those remain manual operator actions during MuJoCo
acceptance.

## App-Owned Release Assets

Release policy/control assets are owned by `agentic-et1-tracker` and live under
`deploy/robots/agentic-et1-tracker/config`:

- Default GeneralTrackerDR3 policy:
  `config/policy/general_tracker_dr3`
- StandbyVelocity policy: `config/policy/velocity/v0`
- FixStand posture: `config/posture/fixstand/v0/fixstand.yaml`
- Passive posture: `config/posture/passive/v0/passive.yaml`
- Standby reference asset: `config/reference/standby/v0/standby_ref.trk`
- Loco-upper lower locomotion policy: `config/policy/loco_lower/et1_low`
- Loco-upper upper-body limits/joint map: `config/limits/et1_upper_body/v0`

Runtime configuration must point at these app-local release assets. Runtime
does not fall back to the ET1 app tree under `deploy/robots/et1`.
Release packages carry the `general_tracker_dr3` policy directory with the
default `DR3-all.onnx` / `deploy_fut_obs.yaml` assets. They also carry the
`general_tracker_cln` directory with the previous `multi_policy_footstate3.onnx` /
`deploy_fut_multi_footstate.yaml` assets and the old
`multi_policy_v17c2_70k.onnx` / `deploy.yaml` CLN compatibility assets. They
omit the unused legacy `config/policy/general_tracker` tracker policy
directory. The public motion input contract remains the existing `.trk` format;
DR3 and CLN/footstate do not add `.et1trk` as a service input.

`standby_ref.trk` is now an app-owned release asset with simulator visual
acceptance recorded in its manifest. Runtime playback is gated internally by
unit-covered transitions from user reference to standby reference and then back
to StandbyVelocity/Velocity0. The real-robot/operator gate remains pending, and
this does not claim overall GA. Direct `/standby_velocity` and the
StandbyVelocity/Velocity0 policy path remain available.

For manual or integration simulation testing in this workspace, the installed
Unitree MuJoCo simulator under `/home/galbot/works/et1` can be used. Test
`.trk` files are available under `/home/galbot/works/agent-test/generated/`.

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
evidence script. The standby reference asset has targeted simulator visual
acceptance, but complete GA simulation evidence still needs recorded queue
FIFO, interrupt, stop-to-standby_velocity, `/fixstand`, `/standby_velocity`,
fault/disconnect, and performance results.

This is an acceptance operation-order note only, not a code semantics change or
new API. If `/status` already reports `ready:true` and
`ctrl:"standby_velocity"`, smoke and normal acceptance should not first send
passworded `/passive`. Use passworded `/passive` only for a dedicated passive
safety-sink scenario after MuJoCo reset/upright state or operator support is
prepared. To recover from `bad_orientation`, send `/fixstand`, wait for
`ready:true`, `ctrl:"fixstand"`, `block:null`, and `err:null`, then send
`/standby_velocity`.

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
- Use `config.sim.yaml.example` as the starting point for local MuJoCo
  acceptance. Keep sim-safe settings such as `mode_machine: 0`, `network: "lo"`,
  app-owned assets, `lowcmd_startup_preflight_ms: 200`,
  `release_motion_mode_on_startup: false`, and the hidden sim-only reference
  endpoint. Set `motion_dirs` to the test `.trk` directory for the run; this
  example uses `/home/galbot/works/agent-test/generated`, matching
  `config.sim.yaml.example`. Adjust `domain_id` or `port` when the local
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
TRK=$(find /home/galbot/works/agent-test/generated -maxdepth 1 -name '*.trk' | head -n 1)
curl http://127.0.0.1:8083/health
curl http://127.0.0.1:8083/status
# if already ready with ctrl:"standby_velocity", do not send passworded /passive first
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
