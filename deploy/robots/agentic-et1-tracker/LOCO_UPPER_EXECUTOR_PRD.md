# Loco Upper Executor PRD

> **Historical/Obsolete API names (2026-07):** This file is a historical
> planning handoff and may still mention old public route names. The GA/current
> API is `POST /standby` for ordinary standby and `POST /urgent_stop` for urgent
> stop. Legacy `POST /standby_velocity` and `POST /stop` are not successful
> aliases; if present they reject with `CONTROL_ROUTE_RENAMED`. Use `README.md`
> and `CONTROL_STATE_MACHINE_REDESIGN_PLAN.md` as the current contract.

Status: simulation-handoff-ready
Date: 2026-06-16
Scope: `agentic-et1-tracker`
Acceptance target: simulation-only P0. This PRD does not approve true-robot
deployment. True-robot GA requires a separate PRD and safety release gate.

## Summary

Add one new TRK execution path to `agentic-et1-tracker`:

> Lower body uses an ET1 locomotion policy, while upper body directly follows
> upper-body joint targets from the submitted `.trk`.

This is an additive feature. Existing `/execute`, `/idle`, `/stop`,
`/passive`, `/fixstand`, `/standby_velocity`, queue semantics, and status
semantics must keep their current behavior.

The new executor is not a full-body TRK tracker. It is a bounded locomotion
executor:

- The upper body follows the TRK joint reference.
- The lower body ignores TRK leg joints and is controlled by locomotion.
- The TRK root motion is converted into safe velocity commands.
- The locomotion command is projected into the locomotion model capability
  envelope.
- A per-request bounded execution radius constrains the planned and commanded
  locomotion area; raw TRK root motion outside that radius is projected/clamped
  and still accepted.

## Product Goal

LLM agents already generate and submit `.trk` motions. Some motions contain
useful upper-body gestures plus walking/turning root motion, but the full-body
GeneralTracker path can be fragile when the lower-body TRK gait is hard to
execute.

This feature gives agents a safer alternative for "move around while doing the
upper-body action":

- preserve the familiar TRK input workflow;
- make the lower-body behavior locomotion-policy-driven;
- keep upper-body gesture quality high;
- expose enough compact status for frequent agent polling;
- avoid any breaking change to the current tracker contract.

## Product Principles

- KISS: one new executor endpoint, one shared physical queue, no executor
  framework or dynamic policy marketplace.
- DRY: reuse existing path allowlist, IDs, queue, interrupt, stop, readiness,
  status, and safety machinery where semantics match.
- YAGNI: do not build uploads, remote asset management, planner profiles,
  trajectory previews, or multi-executor concurrency for P0.
- Compatibility first: the feature ships default-off and must not change
  existing behavior unless explicitly enabled and called through the new
  endpoint.
- Safety over fidelity: lower-body motion should approximate TRK displacement
  only inside the verified locomotion envelope.
- Explicit over magic: agents choose this executor explicitly. The service does
  not auto-detect when a TRK should use locomotion.

## Existing Function Guardrails

The new executor must not affect current users of `agentic-et1-tracker`.

- Default release config keeps `loco_upper.enabled:false`.
- Existing endpoint schemas, errors, existing status fields, queue behavior, model
  loading, StandbyVelocity, idle, stop/passive/fixstand behavior, and
  GeneralTracker execution must remain unchanged.
- The only allowed status/health shape addition is additive top-level
  `cap.loco_upper`. Existing fields keep their names, values, and semantics;
  GeneralTracker run status must not grow `loco`.
- `/execute` must not auto-route into loco-upper.
- Shared code is allowed only at stable boundaries: path allowlist, ID
  allocation, queue/recent-history storage, readiness gates, compact error
  helpers, and status serialization.
- Do not rewrite or generalize the existing controller/FSM only to support this
  feature. Add the smallest explicit loco-upper path that preserves old tests.

## User Mental Model

The agent has two ways to run a `.trk`:

| Need | Endpoint | Meaning |
| --- | --- | --- |
| Full-body sim2real tracker | `POST /execute` | Current GeneralTracker behavior. |
| Locomotion lower body + TRK upper body | `POST /execute_loco_upper` | New additive executor. |

The `mode` field continues to mean scheduling only: `queue` or `interrupt`.
It must not mean "which executor".

## API Contract

### New Endpoint

```http
POST /execute_loco_upper
{
  "path": "/absolute/file.trk",
  "mode": "queue",
  "hold": false,
  "max_radius_m": 0.8
}
```

Allowed keys:

- `path` required string; same local `.trk` allowlist rules as `/execute`.
- `mode` optional string, `queue` or `interrupt`; default `queue`.
- `hold` optional boolean; default `false`.
- `max_radius_m` optional positive number. It is the client-requested bounded
  execution radius for this action. If omitted, use the finite service config
  default. Omitted must not mean unlimited. If the request exceeds the service
  configured maximum capability, the effective radius is capped to that maximum
  and the response/status report the effective value. This is a command/planner
  radius unless fresh pose is available and configured as required.

Rejected body examples:

- extra keys;
- non-`.trk` files;
- upload payloads;
- non-finite or non-positive `max_radius_m`;
- `mode` values other than `queue` or `interrupt`.

Finite positive `max_radius_m` values above the configured service maximum are
not a validation failure; they are capped to the service maximum.

Response:

```json
{
  "ok": true,
  "id": "a7K3p9Qx",
  "state": "queued",
  "q": 1,
  "hold": false,
  "executor": "loco_upper",
  "max_radius_m": 0.8
}
```

### Existing Endpoint Compatibility

`POST /execute` remains unchanged:

```json
{"path":"/absolute/file.trk","mode":"queue","hold":false}
```

It must continue to reject extra fields. In particular, it must reject
`executor`, `kind`, `profile`, `max_radius_m`, and any locomotion-specific
field.

## Status Contract

The new executor must be visible in compact status.

Top-level status/health must expose capability discovery so an agent can avoid
guessing whether the endpoint is usable:

```json
{
  "cap": {
    "loco_upper": {
      "enabled": true,
      "ready": true,
      "default_radius_m": 0.8,
      "max_radius_m": 2.0,
      "strict_pose": false
    }
  }
}
```

If `enabled:true` but assets/config/model are invalid, `ready:false` and the
normal compact error should be `MODEL_NOT_READY`.

When `loco_upper.enabled:false`, `POST /execute_loco_upper` must return a
compact failure instead of `404` or schema errors:

```json
{
  "ok": false,
  "error": {
    "code": "MODEL_NOT_READY",
    "message": "loco_upper disabled",
    "retryable": false
  },
  "next": "status"
}
```

For active or queried user runs:

```json
{
  "id": "a7K3p9Qx",
  "state": "running",
  "executor": "loco_upper",
  "frame": 42,
  "frames": 180,
  "progress": 0.233,
  "hold": false,
  "loco": {
    "max_radius_m": 0.8,
    "distance_m": 0.31,
    "radius_source": "highstate",
    "phase": "motion",
    "radius_clamped": true,
    "radius_limit_reached": false,
    "envelope_clamped": false,
    "upper_clamped": false,
    "upper_rate_limited": false,
    "raw_action_clamped": false,
    "lower_q_limited": false,
    "lower_action_clamped": false,
    "reason": null
  }
}
```

Fields:

- `executor`: `general_tracker` or `loco_upper`.
- `loco.max_radius_m`: effective radius for this run.
- `loco.distance_m`: best available current displacement estimate from run
  start.
- `loco.radius_source`: `highstate`, `integrated`, or `null`; `null` means the
  run has not started or no displacement estimate is currently trusted. Do not
  report `planned` as a radius source.
- `loco.phase`: `queued`, `entry`, `motion`, `holding`, `exit`, `stopping`,
  `done`, `stopped`, `failed`, or `canceled`.
- `loco.radius_clamped`: sticky per-run flag; true once the planned lower-body
  path has been projected into the radius.
- `loco.radius_limit_reached`: sticky per-run flag; true once runtime radius
  limiting has actively suppressed outward locomotion.
- `loco.envelope_clamped`: sticky per-run flag; true once velocity or
  acceleration commands have been limited by the locomotion envelope.
- `loco.upper_clamped`: sticky per-run flag; true once any upper joint target
  has been clamped to configured upper limits.
- `loco.upper_rate_limited`: sticky per-run flag; true once any upper joint
  velocity or acceleration has been rate-limited or smoothed.
- `loco.raw_action_clamped`: sticky per-run flag; true once lower policy raw
  output has been clipped to the lower policy action range.
- `loco.lower_q_limited`: sticky per-run flag; true once composed lower joint
  targets have been limited to configured lower joint bounds.
- `loco.lower_action_clamped`: sticky per-run flag; true once lower policy
  output has been clamped to lower joint limits.
- `loco.reason`: compact machine-readable reason for failed/stopped/degraded
  states, or `null`.

P0 allowed `loco.reason` values:

```text
root_invalid upper_limit upper_dynamic pose_missing pose_jump
policy_nan policy_infer lower_limit mapping_invalid hold_timeout path_error deadline_miss
```

`radius_limit_reached` is a normal bounded execution status flag, not a
`loco.reason`; do not publish `loco.reason:"radius_limit"` or any other
radius-specific reason for radius limiting alone.

Queued runs should return `loco.phase:"queued"`, `distance_m:0`,
`radius_source:null`, and all sticky flags false. Terminal runs should keep the
final `loco` snapshot while the run remains in recent history, so
`GET /status?id=...` can explain how a loco-upper run ended.

For `executor:"general_tracker"`, the `loco` field must be absent or `null`.
GeneralTracker status must not grow synthetic loco fields.

`active.kind` remains unchanged. A running loco-upper action is still
`active.kind:"user"` with the normal run id. Do not add a public controller
state for this executor.

## Queue And Control Semantics

The new executor uses the existing physical action queue.

| Scenario | Required behavior |
| --- | --- |
| `queue` while idle/standby | Enqueue after older user runs. |
| `queue` while GeneralTracker user active | Same queue as `/execute`; no parallel execution. |
| `queue` while loco-upper active | Same FIFO queue. |
| `interrupt` | Preempt active user/idle/transition using existing interrupt semantics; pending user runs are cleared according to existing rules. |
| Active idle | User run preempts idle. Idle config is not a user queue entry. |
| `/standby_velocity` | Ordinary stop/cancel-to-standby. Lower velocity command goes to zero; upper body exits through a bounded interpolation. Idle config is preserved. |
| `/stop` | Urgent stop semantics stay unchanged: highest priority, clear idle, abort active/internal transition, no smoothing promise. For loco-upper this immediately cancels executor runtime, clears active loco state, and hands off to existing stopping/standby bookkeeping. |
| `/passive` | Passworded safety sink; clear active/queue/idle; no auto recovery. |
| `/fixstand` | Fixed stand configuration; not ordinary stop. Active loco-upper work terminates as `stopped` and does not wait for loco-upper exit smoothing. |
| `passive`/`fault` ctrl | Reject `/execute_loco_upper` like `/execute`. |
| `fixstand` ctrl | Reject and direct agent to `/standby_velocity`. |
| `stopping` ctrl | Match the current `/execute` accept gate exactly. In the current implementation this rejects non-empty execution requests and tells the agent to poll status. If `/execute` changes later, `/execute_loco_upper` must change with it. |

During `/standby_velocity` cancellation, a loco-upper run may stay visible as
the active user run with `loco.phase:"stopping"` until its upper-body exit
finishes. This is intentionally different from GeneralTracker's full-body
reference transition internals, because loco-upper does not use the
GeneralTracker transition path. Agents should read the run id and `loco.phase`,
not infer behavior from GeneralTracker-only transition fields.
`/stop` is intentionally different: it does not continue loco-upper ticking
after cancellation, so it does not keep running the loco radius guard once the
active runtime has been cleared.

## Hold Semantics

P0 should support `hold:true` for parity with `/execute`.

When a loco-upper run reaches its last TRK frame with `hold:true`:

- run state becomes `holding`;
- lower locomotion command is zero;
- upper body holds the final TRK upper-body joint target;
- the same run id remains queryable;
- `/standby_velocity`, `/fixstand`, `/stop`, `/passive`, a new user run, or
  configured `max_hold_s` can release it using existing control semantics.

`hold:true` is a P0 contract for this endpoint. Invalid or truly dangerous TRK
inputs must be rejected before execution, but upper/root physical capability
overruns that can be bounded must be clamped or rate-limited instead of making
hold a partial feature.

P0 must use a finite configurable `max_hold_s` for loco-upper. If the timeout
fires, the run exits through the same bounded upper-body exit path and reports
terminal `state:"done"`, `loco.phase:"done"`, and
`loco.reason:"hold_timeout"` without a top-level error. Agent wait logic should
treat this as successful completion of a bounded hold, not as failure. This
keeps simulation behavior bounded while preserving explicit user release
controls.

## Completion Semantics

When `hold:false` and the TRK frames finish:

1. lower locomotion command becomes zero;
2. the active user run enters `loco.phase:"exit"` while keeping the same id,
   `frame=frames-1`, and `progress=1.0`;
3. upper body transitions back to the configured standby upper posture over a
   short bounded interpolation;
4. after the exit interpolation completes, the user run becomes `done`;
5. normal idle/standby behavior resumes.

The exit interpolation is part of the same user run lifecycle, not a
GeneralTracker synthetic transition. It is interruptible, must not create a
second user run id, and must not consume queue capacity. During entry
interpolation the run is active with `loco.phase:"entry"` and frame `0`.
During `/standby_velocity` cancellation, the run uses `loco.phase:"stopping"`
until the bounded upper-body exit finishes, then terminates as `stopped`.
`/stop` and `/passive` still abort immediately without waiting for this exit.

P0 does not need to transition directly from loco-upper into an idle TRK
reference. Returning through standby first is simpler and safer. Existing idle
auto-play may start afterward if configured and the robot is in a valid
standby/running state.

## Execution Model

### Data Model Boundary

The implementation must thread executor type and loco-upper options through
the existing runtime data model instead of inferring behavior from endpoint
names after parsing.

Required internal data additions:

- `MotionExecutor`: `general_tracker` or `loco_upper`.
- `LocoRunOptions`: effective `max_radius_m` and request `hold`.
- `LocoRunStatus`: `phase`, radius fields, sticky clamp flags, and compact
  `reason`.
- `LocoUpperCapability`: `enabled`, `ready`, `default_radius_m`,
  `max_radius_m`, and `strict_pose`.
- `ExecuteCommand`, `MotionRequest`, `MotionStatus`, runtime queue entries,
  recent history, and `RuntimeBridge::motionRequest()` must preserve executor
  and loco payloads.
- `RuntimeStatusStore::queuedStatus()` must report queued loco-upper runs as
  loco-upper runs, not flatten them into ordinary GeneralTracker TRK status.
- `StatusSnapshot` and `HealthSnapshot` must include `LocoUpperCapability` even
  when loco-upper assets failed to load, so `/status` and `/health` can explain
  `MODEL_NOT_READY` before any run exists.

Dispatch-time validation is required. A run that was valid when queued may
become unsafe by the time it starts because robot readiness, orientation, pose
freshness, or controller state changed. Before a queued loco-upper run becomes
active, revalidate readiness and entry transition feasibility. Do not turn raw
root radius overflow into a dispatch failure; the same bounded compiler must
project it into the effective radius.

### Assets

Runtime must not depend on `/home/galbot/works/et1/.reference/...`.

The implementation must copy only required lower-locomotion assets into
`agentic-et1-tracker` owned release assets, then reference the copied paths from
config.

Reference source:

```text
/home/galbot/works/et1/.reference/deploy_loco_lower/deploy/robots/et1/config/policy/velocity/et1_low
```

Expected app-owned target shape:

```text
deploy/robots/agentic-et1-tracker/config/policy/loco_lower/et1_low/
  exported/policy.onnx
  params/deploy_lowobs10k.yaml
  ASSET_MANIFEST.yaml
  README.md
```

Do not copy reference build outputs, third-party binaries, FSM source trees,
Mimic/BFM/getup policies, `.npz`, `.csv`, or `.r1trk` assets unless a later
PRD explicitly expands scope.

P0 must treat the copied `et1_low` model/deploy as a first-class app-owned
policy contract:

- add a `LocoLowerDeployConfig` loader or extend the velocity loader only if it
  can support the copied deploy without changing StandbyVelocity semantics;
- validate the copied deploy schema, observation dimensions, action dimensions,
  command ranges, joint maps, stiffness, damping, and ONNX IO metadata in
  tests;
- fail startup as `MODEL_NOT_READY` when the configured loco-upper model/deploy
  contract does not match;
- do not hand-edit the reference asset into an undocumented shape without an
  app-owned README and manifest explaining the normalized contract.

### Joint Ownership

P0 fixed split:

- lower body: logical joints `0..11`, controlled only by lower locomotion;
- upper body: logical joints `12..25`, directly commanded from TRK `joint_pos`;
- unowned/unused motor slots, including any SDK indices outside the configured
  logical split, must receive explicit configured safe hold/passive commands
  every tick and must never inherit stale command memory;
- TRK lower-body joint targets and foot contacts are ignored for command
  output.

The `12..25` choice matches the reference locomotion/mimic implementation.
It includes waist joints. This improves "upper body follows TRK" fidelity but
can affect balance; simulation acceptance must validate it before the feature
is considered P0-ready.

Upper-body direct control is only allowed after bounded compilation:

- raw absolute joint targets outside configured upper-body joint limits are
  clamped into the configured limits and reported with `upper_clamped:true`;
- raw frame-to-frame joint velocity or acceleration beyond configured dynamics
  is rate-limited/smoothed and reported with `upper_rate_limited:true`;
- waist joints must use conservative configured limits; when waist or
  upper-body motion approaches loco-compatible dynamics, lower velocity should
  scale toward zero before any stability margin is consumed;
- entry, hold, motion, and exit targets must all use the same bounded targets;
- basic dangerous-pose/self-collision rejects must exist for known unsafe
  waist/arm/head combinations, even if implemented as conservative rule checks
  in P0;
- the LowCmd composer must never write an upper-body `q` outside those limits.

The logical-to-SDK motor mapping must be app-owned and tested. Do not assume the
reference locomotion repository's identity ordering is the same as the current
agentic tracker runtime.

### LowCmd Composition

Each control tick:

1. read lowstate/highstate and run existing freshness, mode-machine, and
   orientation safety checks;
2. compute current locomotion command `[vx, vy, yaw_rate]`;
3. run lower locomotion policy with that command in its observation at the
   configured policy decimation rate;
4. hold the last lower action between policy inference ticks;
5. clamp or fail lower-body motor commands against configured lower joint
   position/velocity/torque limits;
6. write lower-body motor commands from the locomotion policy;
7. overlay upper-body motor commands from the compiled bounded upper joint
   targets using configured upper-body gains;
8. explicitly populate all unowned motor slots with safe configured commands;
9. write the composed LowCmd.

The current `VelocityStepRunner` zero-command behavior must be generalized so
standby can still pass `[0,0,0]`, while loco-upper can pass non-zero
locomotion commands. Existing StandbyVelocity behavior must remain zero-command.

Implementation boundary:

- introduce an explicit `VelocityCommand`/`LocoVelocityCommand` value passed to
  the lower policy input builder;
- StandbyVelocity always passes zero command;
- loco-upper passes the planner command;
- reset policy history on executor state changes so standby zero-command
  history and loco-upper command history do not silently contaminate each other;
- create a dedicated `LocoUpperLowCmdComposer` or equivalent isolated composer
  so GeneralTracker, StandbyVelocity, and loco-upper writes cannot share stale
  motor command state;
- add tests proving GeneralTracker execution never uses the loco command path.

Policy timing must come from deploy/config. P0 must define:

- LowCmd write period;
- lower policy inference period and `policy_decimation`;
- command update period;
- deadline miss accounting.

If the copied lower policy deploy has `policy_decimation: 10`, the runtime must
not run inference every LowCmd tick unless a new validated deploy explicitly
requires that behavior.

### Runtime State Boundary

Loco-upper needs its own internal active/exit path. It must not reuse
GeneralTracker synthetic transitions for entry, completion, or
`/standby_velocity` cancellation, because those transitions are full-body
reference-policy paths.

Required internal states:

- `LocoUpperEntry`: upper body interpolates into the first TRK upper target;
  lower command remains zero or a bounded first planner command.
- `LocoUpperMotion`: TRK upper frames and lower velocity planner are active.
- `LocoUpperHolding`: lower command zero, final upper target held.
- `LocoUpperExit`: natural completion returns upper body to standby posture.
- `LocoUpperStopping`: `/standby_velocity` cancellation returns upper body to
  standby posture.
- `LocoUpperFaulted`: runtime fault/safety failure has terminated the run and
  yielded to the existing safety path.

Public state remains simple:

- while playing or holding, `active.kind:"user"`;
- `ctrl` maps to existing `preparing`, `running`, `stopping`, or
  `standby_velocity`;
- `loco.phase` is the compact sub-state for this executor;
- no new public `ControllerState` and no new public `ActiveKind`.

## Trajectory-To-Velocity Planner

P0 should use a deterministic local planner rather than a complex global
optimizer.

Inputs:

- TRK root position: `body_pos_w[root_body_index]`, default root body index `0`;
- TRK root orientation: `body_quat_w[root_body_index]`;
- optional cross-check fields: `body_lin_vel_w`, `body_ang_vel_w`;
- runtime pose: highstate position/yaw when fresh, otherwise command-integrated
  estimate.

Runtime pose must define frame and freshness. P0 uses the local odometry frame
reported by ET1 highstate when available. A highstate pose older than
`pose_fresh_timeout_ms`, a non-finite pose, or a discontinuous pose jump must
not be used to claim physical radius containment. With strict pose enabled,
that is a pose-source/readiness problem, not evidence that the requested action
itself breached the radius.

Timing rule: TRK frame/progress advances from monotonic simulation/control time
and TRK fps. It must not be tied to lower policy decimation. The lower policy
may hold last action between inference ticks, and the planner command may update
at a configured command period, but neither may change TRK playback speed.

Steps:

1. Validate all required TRK values are finite.
2. Extract planar root path `(x, y, yaw)` and unwrap yaw.
3. Rigidly align the TRK first frame to action start:

   ```text
   p_ref_aligned[i] =
     p_robot_start.xy + R_yaw_align * (p_ref_raw[i].xy - p_ref_raw[0].xy)
   yaw_ref_aligned[i] =
     yaw_robot_start + wrap(yaw_ref_raw[i] - yaw_ref_raw[0])
   ```

   This removes arbitrary offline/mocap world-frame offsets before feedback or
   radius projection.
4. Radius-project the planned path into the effective `max_radius_m` circle
   around the action start.
5. Generate velocity by finite differences over the projected path.
6. Apply light smoothing and acceleration/yaw-acceleration limits.
7. Convert world-frame path command into robot-body command:

   ```text
   v_cmd_body = R_robot_yaw^-1 * (v_ref_world + Kp * (p_ref - p_robot))
   yaw_cmd = yaw_rate_ref + Kyaw * wrap(yaw_ref - yaw_robot)
   ```

8. Project the command into configured locomotion limits.
9. Near the command radius boundary, remove outward radial velocity. If the
   projected plan or runtime estimate reaches the boundary, set
   `radius_limit_reached:true`, suppress further outward radial command, and
   keep the run under control. The run should complete the bounded plan, enter
   bounded exit, or return to standby through ordinary control semantics. Radius
   limiting by itself must not terminate as `failed`, must not report
   `SAFETY_LIMIT_TRIGGERED`, and must not enter passive.

The first implementation should not add splines, MPC, obstacle avoidance, or
multi-profile planners. Those belong in P1+ only after P0 simulator evidence.

Planner invariance test required: the same relative TRK trajectory with any
constant global XY offset must generate the same locomotion commands after
alignment.

## Locomotion Envelope

Velocity limits must come from the selected locomotion deploy/config, not from
hard-coded code constants.

The current app-owned velocity config exposes:

```yaml
commands:
  base_velocity:
    ranges:
      lin_vel_x: [-0.5, 1.0]
      lin_vel_y: [-0.3, 0.3]
      ang_vel_z: [-0.2, 0.2]
```

P0 must also enforce config-level acceleration limits. These are required
service config values, not hidden constants and not per-request API fields.
When loco-upper is enabled, missing or invalid limits make the loco-upper model
not ready.

The loader must parse and validate command ranges, acceleration limits,
`policy_decimation`, observation/action dimensions, joint maps, gains, and lower
joint safety limits for the copied lower policy. The current StandbyVelocity
loader may be extended only if this does not change zero-command standby
semantics; otherwise add a dedicated loco lower loader.

If commands are clamped, status must expose sticky
`loco.envelope_clamped:true`. If lower policy output is clamped, status must
expose sticky `loco.lower_action_clamped:true`. Clamping is allowed because this
executor only promises safe approximate displacement, not exact full-body/root
replay.

Upper-body dynamics also affect lower-body stability. P0 should implement the
simplest conservative rule:

- define loco-compatible limits for upper joint velocity, acceleration, and
  waist magnitude;
- scale lower `vx/vy/yaw_rate` toward zero when upper motion approaches those
  limits;
- clamp or rate-limit raw upper motion before execution when it exceeds
  configured physical limits; only genuinely dangerous poses or runtime safety
  faults should reject or fail.

## Command Radius Safety

`max_radius_m` is the client-requested bounded execution radius around the
action start. It limits the area the executor may plan and command for this
specific action. It is a planner/command radius by default. It is a physical
radius only when a fresh trusted position source is available and the service is
configured to require it.

For P0 simulation, call this a command radius, not a geofence. The acceptance
gate may compare simulated pose against the radius, but the API must not claim
physical containment unless strict pose is enabled and fresh pose is available.

Rules:

- effective radius is the finite client `request.max_radius_m`, capped by the
  service configured `loco_upper.max_radius_m`; if the client omits the field,
  use the finite service default;
- the service configured default is only a default for omitted requests;
- the service configured maximum is a deployment capability cap, not a safety
  fault threshold for raw TRK content;
- raw TRK path exceeding the radius is projected into the radius instead of
  blindly followed, rejected, or treated as a passive/fault condition;
- status exposes `radius_clamped:true` when projection occurs;
- runtime must suppress outward radial velocity if measured or estimated
  displacement reaches the effective radius boundary while the loco-upper
  executor is still active;
- status exposes sticky `radius_limit_reached:true` when runtime boundary
  limiting actively suppresses outward radial command;
- after the boundary is reached, the run remains controlled: it may continue
  non-outward components, complete the bounded plan, enter bounded exit, hold,
  or return to standby through ordinary control semantics;
- reaching the radius boundary or clamping a raw path is not a reason to enter
  passive, publish `SAFETY_LIMIT_TRIGGERED`, or fail with a radius reason;
- actual radius enforcement should use fresh highstate position when available;
- if only command integration is available, status must report
  `radius_source:"integrated"` and docs must state this is not a physical
  world-geofence guarantee;
- the runtime radius guard covers active loco-upper executor phases `entry`,
  `motion`, `holding`, and bounded `exit`; `/stop` is a higher-priority cancel
  path and is not required to keep running loco controller ticks after
  cancellation.

If product later requires strict physical radius guarantees, a reliable
localization source must become a hard dependency. That future strictness is
about pose-source validity, not about treating raw TRK radius overflow as a
safety fault.

## Validation And Rejection

Existing `TrackValidatorPort` remains the general TRK validator. Loco-upper
needs an additional loco-specific compiler/precheck that runs after standard
TRK validation and before id allocation. It validates root data, config,
mapping, numeric legality, dangerous-pose rules, and radius option shape.
Physical capability boundaries are handled by bounded compilation.
Dispatch-time revalidation runs again immediately before the queued item becomes
active.

Reject before queue/id allocation when:

- request JSON is invalid;
- path is not an allowed local `.trk`;
- TRK fails existing validation;
- upper-body joint values are non-finite;
- raw upper-body motion violates a true dangerous-pose/self-collision rule that
  cannot be made safe by clamp/rate-limit;
- `request.max_radius_m` is non-finite or non-positive;
- locomotion assets/model/deploy are not ready;
- controller/readiness gates reject as they would for `/execute`.

Finite positive request radius above the service maximum is accepted with the
effective radius capped to the service maximum. Raw root path outside the
effective radius is accepted and projected.

Reject or fail at runtime only for true runtime safety/readiness failures:

- lowstate becomes stale;
- highstate pose is required but stale/missing/jumping;
- robot orientation exits safe limits;
- policy inference fails;
- policy output is non-finite;
- LowCmd write fails;
- path tracking error exceeds configured safety threshold.

Orientation safety must continue to route to the existing passive/fault safety
path. Explicit authorized `/passive`, bad orientation/fall risk, and genuine
low-level safety faults are the conditions that may enter passive/fault. Radius
clamping or boundary limiting is not one of those conditions.

## Configuration

Suggested minimal config shape:

```yaml
agentic_et1_tracker:
  loco_upper:
    enabled: false  # release default; simulation acceptance config sets true
    policy_dir: "config/policy/loco_lower/et1_low"
    policy_file: "policy.onnx"
    deploy: "config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"
    default_max_radius_m: 0.8
    max_radius_m: 2.0
    radius_tolerance_m: 0.05
    strict_radius_requires_pose: false
    upper_body_start_joint: 12
    upper_body_limits: "config/limits/et1_upper_body/v0/limits.yaml"
    joint_map: "config/limits/et1_upper_body/v0/joint_map.yaml"
    entry_transition_s: 0.5
    exit_transition_s: 0.5
    max_hold_s: 10.0
    kp_xy: 0.4
    kp_yaw: 0.6
    max_lin_accel_mps2: 0.4
    max_yaw_accel_radps2: 0.5
    smoothing_window_frames: 5
    max_path_error_m: 0.35
    pose_fresh_timeout_ms: 100
    pose_jump_reject_m: 0.25
    upper_loco_scale_start: 0.7
    upper_loco_reject: 1.0
    sim_acceptance:
      deadline_miss_ratio_max: 0.005
      deadline_miss_consecutive_max: 2
      policy_inference_p99_period_fraction_max: 0.5
      status_poll_miss_ratio_delta_max: 0.001
```

Only request `max_radius_m` is per-request. Other tuning is service config.
`default_max_radius_m` supplies the action radius when the client omits
`max_radius_m`. The config key keeps its historical name; public capability
status exposes the same default as `default_radius_m` because clients see only
the default request radius. Config `max_radius_m` is the deployment capability
cap applied to the requested/default radius; it must not be interpreted as a
fault threshold for raw TRK root motion.

When `loco_upper.enabled:true`, the config loader must validate every
loco-upper field and asset path. A missing policy, deploy, upper-body limits
file, command range, gain, or planner limit must make the runtime health
`MODEL_NOT_READY`; it must not silently disable parts of the executor.

Implementation must add an explicit `LocoUpperConfig` to app config and runtime
factory construction. `loco_upper.enabled:false` means the endpoint is disabled
and capability discovery reports `enabled:false, ready:false`; all existing
endpoints must behave as before.

`strict_radius_requires_pose` is P0 behavior. When true, missing/stale/jumping
runtime pose rejects or aborts loco-upper runs before claiming physical radius
containment. It does not make raw TRK root radius overflow a rejection or
safety fault. When false, command-integrated radius is allowed but must be
reported as `radius_source:"integrated"`.

The upper-body limits file should provide one entry for each controlled logical
joint `12..25`:

```yaml
joint_min: [...]
joint_max: [...]
max_vel_radps: [...]
max_accel_radps2: [...]
```

P0 may use conservative app-owned ET1 limits. It must not infer unlimited joint
ranges from the TRK.

## Testing Strategy

Use TDD. Keep tests small and precise.

### API Tests

- `/execute` still rejects extra fields.
- `/execute_loco_upper` accepts only `path/mode/hold/max_radius_m`.
- capability discovery reports `cap.loco_upper.enabled/ready/default_radius_m/
  max_radius_m/strict_pose`.
- disabled loco-upper reports compact failure without changing existing
  endpoints.
- readiness and controller conflict behavior matches `/execute`.
- returned id is waitable through `GET /status?id=...`.
- invalid radius values return compact `REQUEST_INVALID`.

### Config And Factory Tests

- `loco_upper.enabled:false` is the default and keeps existing app
  config/factory behavior.
- `loco_upper.enabled:true` resolves app-owned policy, deploy, limits, and
  manifests without referencing `.reference`.
- bad loco-upper policy path, deploy path, limits path, command limits, or ONNX
  IO contract reports `MODEL_NOT_READY`.
- copied `et1_low` deploy/model contract loads in the app runtime under tests.

### Existing Behavior Regression Tests

- Run the current `/execute`, idle, `/stop`, `/passive`, `/fixstand`, and
  `/standby_velocity` unit/runtime tests with `loco_upper.enabled:false`.
- Run the same regression subset with `loco_upper.enabled:true` but without
  calling `/execute_loco_upper`; observable behavior must match baseline.
- GeneralTracker model/deploy selection, transition behavior, hold semantics,
  and existing status fields must remain unchanged. Additive top-level
  `cap.loco_upper` is allowed and should not make regression tests compare
  exact JSON object shape.

### Queue/Status Tests

- loco-upper queue and GeneralTracker queue are shared FIFO.
- queued loco-upper runs expose `executor:"loco_upper"` and
  `loco.phase:"queued"`.
- `interrupt` clears pending user work regardless of executor.
- `/stop`, `/passive`, `/fixstand`, `/standby_velocity` affect loco-upper active
  work through existing priority semantics.
- status exposes `executor:"loco_upper"` and compact `loco` fields.
- terminal status retains the final `loco` snapshot while the run remains in
  recent history.

### Planner Tests

- empty, one-frame, and too-short TRKs are rejected with compact validation
  errors.
- TRK fps/dt/frame count conversion preserves progress and duration.
- static TRK produces zero velocity command.
- straight forward, lateral, and yaw-only TRKs produce expected command signs.
- non-zero starting yaw, yaw wrap, and near-180-degree yaw changes produce
  continuous commands with correct signs.
- adding any constant global XY offset to the same relative TRK trajectory does
  not change generated commands after start alignment.
- commands are finite.
- velocity and acceleration limits are always respected.
- raw path outside radius is projected; `radius_clamped` is true.
- runtime radius guard runs every loco tick while the loco-upper executor is
  active across entry, motion, holding, and bounded exit; it suppresses outward
  radial velocity at the effective radius boundary, sets
  `radius_limit_reached`, and keeps the run controlled without passive/fault or
  radius-specific failure.
- `/stop` immediately cancels the loco-upper executor, clears active runtime
  state, and returns through stopping/standby without additional loco
  radius-guard ticks.
- invalid root data is rejected.
- missing pose with `strict_radius_requires_pose:true` rejects or aborts the
  run before claiming physical radius containment.
- stale, jumping, or NaN highstate pose is handled according to strict-pose
  config and never silently reports physical radius containment.

### LowCmd Composition Tests

- lower joints `0..11` come only from lower locomotion policy output.
- upper joints `12..25` come from TRK after entry interpolation.
- all other motor slots receive explicit configured safe commands every tick.
- raw upper joint targets outside limits, above max velocity, or above max
  acceleration are clamped/rate-limited in the compiled plan before LowCmd
  write.
- entry, hold, motion, and exit upper targets all use the same bounded targets.
- lower policy output outside configured lower limits is clamped or fails
  according to configured hard limits and sets `lower_action_clamped`.
- wrong SDK/logical joint mapping is caught by tests.
- StandbyVelocity still sends zero velocity command.
- GeneralTracker `/execute` still calls the existing full-body policy path.

### Runtime Tests

- loco-upper run starts from StandbyVelocity and reaches `done`.
- `hold:true` reaches `holding`, lower command zero, upper target held.
- `max_hold_s` timeout exits through bounded exit and reports
  terminal `state:"done"`, `loco.phase:"done"`, and
  `loco.reason:"hold_timeout"`.
- during entry interpolation status reports `loco.phase:"entry"` with frame `0`.
- during natural exit interpolation status reports `loco.phase:"exit"` with the
  same user id until terminal `done`.
- `/standby_velocity` during loco-upper reports `loco.phase:"stopping"` and
  terminates the same user id as `stopped`.
- reaching the effective radius boundary suppresses outward radial velocity,
  sets `radius_limit_reached`, and remains a controlled execution/completion or
  standby path rather than `failed`, `SAFETY_LIMIT_TRIGGERED`, passive, or a
  radius-specific reason.
- bad orientation enters existing safety path.
- interrupt from loco-upper to GeneralTracker and GeneralTracker to loco-upper
  preserves run history and queue semantics.
- injected policy inference failure, ONNX NaN, stale lowstate/highstate, and
  LowCmd write failure all produce terminal status and compact errors.

### Simulation P0 Release Gates

P0 acceptance is simulation-only. Do not use this PRD as approval to deploy the
feature on a true robot.

Required order:

1. pure unit tests and fake policy/runtime tests pass;
2. MuJoCo runs with short radius and slow commands pass;
3. MuJoCo runs with mixed queue/interrupt/hold/standby pass;
4. screenshot review and status/log review pass.

Fixed simulation profile:

- scene: `unitree_mujoco/unitree_robots/et1_v2/scene.xml`;
- initial state sequence: service ready, `/fixstand`, scripted foot contact
  setup when available, then `/standby_velocity`; if manual setup is used, the
  artifact must record the exact operator step and timing;
- seed: `20260616` for generated fixtures and any randomized idle/queue order;
- default scenario radius: `max_radius_m:0.5` unless the scenario overrides it;
- radius tolerance: `radius_tolerance_m:0.05`;
- command envelope: use the selected lower-locomotion deploy ranges, with
  acceleration limits from service config;
- screenshot resolution: at least 1280x720;
- cameras: one front/three-quarter camera, one side camera, and one top or
  overhead camera when checking radius.

Acceptance TRK fixtures must be checked in under an app-owned test fixture
directory, for example
`deploy/robots/agentic-et1-tracker/tests/fixtures/loco_upper/`, with one
metadata file per scenario recording prompt, seed, frame count, fps, expected
rough displacement/yaw, and requested `max_radius_m`.

Required MuJoCo scenarios:

| Scenario | Fixture/profile | Pass criteria |
| --- | --- | --- |
| static/root-still upper motion | `static_upper.trk`, 3-5 s, `max_radius_m:0.5` | no base jump, lower command stays near zero, upper follows TRK qualitatively |
| forward 0.4 m with small arms | `forward_small_upper.trk`, 3-6 s, `max_radius_m:0.5` | robot remains upright, moves in expected direction, no fall or spin |
| yaw turn plus short walk | `yaw_walk.trk`, 3-6 s, `max_radius_m:0.5` | yaw direction matches planned command, no wrap flip |
| large upper/waist motion | `large_upper_bounded.trk`, 2-5 s, `max_radius_m:0.5` | upper motion is clamped/rate-limited, lower velocity scales down, and the run rejects/fails only for true dangerous-pose or runtime safety conditions |
| radius clamp | `radius_clamp.trk`, raw path beyond radius, `max_radius_m:0.35` | raw path is accepted and projected; MuJoCo ground-truth displacement stays within the effective radius tolerance, and `radius_clamped` or `radius_limit_reached` is visible without passive/fault |
| queue then interrupt | two short fixtures, interrupt after 1 s | old active/queued work stops, new run starts smoothly, terminal statuses are retained |
| hold then standby | `hold_upper.trk`, `hold:true`, standby after 2 s | upper holds final target, lower zeroes, standby cancellation exits smoothly |

Simulation pass/fail gates:

- no `bad_orientation`, passive/fault, or fall unless that scenario explicitly
  injects a fault;
- body remains visually upright and feet remain plausibly grounded in captured
  screenshots;
- no visible teleport, 180-degree root flip, or sudden uncommanded full-body
  snap in screenshots;
- status JSON matches the expected run lifecycle and terminal state;
- command radius is judged from MuJoCo ground-truth base pose logs, independent
  of tracker `loco.distance_m`; logs must include `max_radius_m`,
  `measured_max_distance_m`, and `radius_tolerance_m`;
- all acceptance artifacts store command, config hash, model manifest, status
  log, and screenshot paths.

Screenshot/reading gate:

- capture a fixed 5 Hz screenshot sequence plus start, mid-run, end/terminal,
  side-view, and top/overhead radius view for each required MuJoCo scenario;
- use image review to qualitatively confirm upright posture, expected travel
  direction, upper-body gesture follow-through, no collapse, and no obvious
  discontinuity;
- a reviewer may fail the scenario on visual discontinuity even if unit tests
  pass.

Performance gates during simulation:

- LowCmd control loop deadline misses are measured and reported;
- deadline miss ratio must be `<= 0.005` and max consecutive misses must be
  `<= 2`, unless the simulation acceptance config explicitly sets stricter
  thresholds;
- lower policy inference p99 must be `<= 0.5 * policy_period`;
- `GET /status` polling at agent load, for example 20 concurrent clients at
  10 Hz, must not increase deadline miss ratio by more than `0.001` or corrupt
  status;
- HTTP errors remain compact and actionable.

## P0 Scope

- New endpoint `POST /execute_loco_upper`.
- Release default `loco_upper.enabled:false`; simulation acceptance explicitly
  enables it.
- Shared queue/id/status/recent history with existing user runs.
- Executor type/options/status payload threaded through runtime data model.
- Capability discovery in compact status/health.
- App-owned copy of selected lower locomotion assets.
- `LocoUpperConfig` parsing, path guards, app runtime factory loading, and
  `MODEL_NOT_READY` behavior for invalid loco-upper assets.
- Loco lower deploy/model IO contract support for the copied `et1_low` policy.
- Velocity command injection into lower locomotion policy without changing
  zero-command StandbyVelocity.
- TRK upper-body direct target overlay for joints `12..25`.
- Upper-body joint clamp/rate-limit and dangerous-pose validation.
- Root-to-velocity planner with yaw alignment, radius projection, smoothing,
  velocity limits, and acceleration limits.
- Per-request `max_radius_m`.
- Compact status for executor and bounded locomotion state.
- Stop/passive/fixstand/standby behavior unchanged.
- TDD coverage for API, planner, LowCmd composition, and runtime priority.
- MuJoCo-only acceptance with status logs and screenshot review.
- Full existing-behavior regression with loco-upper disabled and enabled-but
  unused.

Recommended implementation order inside P0:

1. config/factory, asset contract, and capability discovery tests;
2. API/data plumbing, queue/recent-history preservation, and status fields;
3. planner pure functions, radius projection, and pose-source tests;
4. lower policy command injection with StandbyVelocity regression tests;
5. LowCmd upper overlay, unowned motor safety, and lower/upper limit tests;
6. runtime active/hold/exit/stopping behavior and fault injection;
7. MuJoCo acceptance with screenshot review.

## P1+ Scope

- Better trajectory fitting or lookahead planner.
- More detailed locomotion telemetry.
- Executability score or preview endpoint.
- Dynamic upper-body joint split, if waist control proves unsafe.
- Strict physical radius guarantees backed by a validated localization source
  beyond MuJoCo ground truth.
- Loco-upper idle motions.
- Obstacle avoidance or map-aware navigation.
- Multi-profile locomotion policy selection.

## Non-Goals

- No change to `/execute` schema or behavior.
- No change to any existing endpoint behavior, existing status field semantics,
  queue priority, selected tracker policy, standby/idle semantics, or safety
  transition. Additive top-level `cap.loco_upper` is the only status/health
  contract addition.
- No uploads or remote motion downloads.
- No automatic executor selection from TRK content.
- No broad executor framework or rewrite of existing GeneralTracker/FSM
  internals for P0.
- No user-provided raw velocity commands.
- No concurrent execution of GeneralTracker and loco-upper runs.
- No `.r1trk` support in this feature.
- No passive auto-recovery.
- No true-robot GA or deployment approval in this PRD.
- No promise that arbitrary TRK is executable.
- No promise of exact global-position tracking without reliable localization.
- No promise that clipped lower-body motion preserves original TRK timing or
  spatial fidelity.

## Key Risks

- The lower locomotion policy may have been trained assuming upper body stays
  near fixstand; large upper-body TRK motions can still destabilize the robot.
- Radius without reliable pose is a commanded/integrated bound, not a physical
  geofence.
- Velocity clipping changes TRK displacement and can desynchronize upper-body
  gesture and lower-body travel.
- Including waist joints `12..13` in "upper body" improves fidelity but may
  reduce balance margin.
- Reference deploy/config shape may differ from current app-owned velocity
  parser and must be validated under tests before asset switch.
- Simulation-only acceptance can miss true-robot issues; real deployment must
  get a separate safety review and release gate.

## Handoff Checklist

- [ ] Implement via TDD; failing tests first.
- [ ] Keep `loco_upper.enabled:false` as the default release config.
- [ ] Do not alter existing endpoint behavior or existing status field
      semantics; only additive top-level `cap.loco_upper` is allowed.
- [ ] Do not add a second physical queue.
- [ ] Copy only required locomotion assets into app-owned config.
- [ ] Add `LocoUpperConfig` and factory tests for copied `et1_low` assets.
- [ ] Add capability discovery for `loco_upper`.
- [ ] Preserve executor/loco payloads through queue, active status, and recent
      history.
- [ ] Extend velocity policy input to accept explicit command while preserving
      zero-command StandbyVelocity.
- [ ] Add required planner and upper-body limit config; fail closed if missing.
- [ ] Validate lower/upper/unowned LowCmd isolation with unit tests.
- [ ] Validate lower joint limits, upper dynamics scaling/rejects, and policy
      decimation behavior.
- [ ] Validate entry/exit/stopping status timing with runtime tests.
- [ ] Verify stop/passive/fixstand/standby priority regression tests.
- [ ] Run existing behavior regression with loco-upper disabled and
      enabled-but-unused.
- [ ] Run MuJoCo acceptance with status logs and screenshot review.
