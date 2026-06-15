# TRK Termination To Standby Product Logic

This document is the product/technical handoff for ordinary user requests like
"stop", "pause", or "terminate the current TRK" while `agentic-et1-tracker` is
executing a user motion.

Scope is limited to `deploy/robots/agentic-et1-tracker` and the `et1-action`
skill semantics. It does not propose a new public API, a new TRK format, or any
change to the ET1 reference app.

## Summary

Ordinary user cancellation should mean:

> Cancel the foreground user TRK, converge to `standby_velocity`, and preserve
> the safety semantics of `/stop`, `/passive`, and `/fixstand`.

The product/API contract must not promise "smooth stop" as an external feature
or expose it as a separate LLM-facing choice. Internally, the runtime should
prefer a bounded, interruptible synthetic transition to the app-owned standby
reference when the current state makes that safe and available.

## Current Behavior

Current code only partly satisfies the desired user experience.

| Path | Current behavior | Smooth to standby? |
| --- | --- | --- |
| User TRK naturally completes, no idle configured | `startTransitionFromCompletedUserToStandby()` builds a synthetic transition, then standby playback. | Yes |
| User TRK naturally completes, idle configured | User transitions to next idle TRK. | Yes, to idle |
| User TRK is holding final frame, `/standby_velocity` | `startTransitionFromHoldingToStandby()` gates through standby reference. | Yes |
| User TRK running, `/standby_velocity` | `handleControl()` marks active as stopping and enters `Stopping`. | No |
| User TRK preparing without a loaded source frame, `/standby_velocity` | `handleControl()` falls back to stopping/cancel behavior. | No source frame; fallback is acceptable |
| Active transition, `/standby_velocity` | Current transition is aborted. | No |
| Active idle, `/standby_velocity` | Active idle is stopped; idle config is preserved and may restart later. | No |
| `/stop` | Clears idle, stops/aborts active work, no standby playback. | No, by design |
| `/passive` | Immediate safety sink; clears idle and active work. | No, by design |

Key code references:

- `RuntimeControlLoop::handleControl(ControlMode::StandbyVelocity)`
- `RuntimeControlLoop::startTransitionFromCompletedUserToStandby()`
- `RuntimeControlLoop::startTransitionFromHoldingToStandby()`
- `RuntimeControlLoop::handleStop()`
- `RuntimeBridge::stop()`
- `RuntimeBridge::standbyVelocity()`

## Product Semantics

### Public Intent Mapping

| Human intent | Skill/API mapping | Product meaning |
| --- | --- | --- |
| "stop", "pause", "terminate current action", "stand by" | `standby` / `/standby_velocity` | Ordinary cancellation; return to normal standby behavior. |
| "emergency stop", "abort", "kill", explicit urgent stop | `urgent-stop --urgent` / `/stop` | Immediate abort; do not smooth; clear idle. |
| "passive", "power/control off", explicit authorized passive | `/passive` with password | Safety sink; no automatic recovery. |
| "enter stand configuration" | `/fixstand` | Fixed preparation/recovery posture, not ordinary stop. |
| "do not idle / completely still" | clear idle, then standby | Pure standby without background idle. |

The LLM-facing contract should stay simple:

- Ordinary stop uses standby.
- Urgent stop uses `/stop`.
- Passive is explicit and passworded.
- Fixstand is explicit recovery/preparation.

Do not add `smooth_stop`, `stop_to_standby`, per-request transition duration,
blend/easing/contact-profile options, or agent-selected stop profiles.

### Standby And Idle

`standby_velocity` preserves idle configuration. If idle motions are loaded,
the robot may later enter background idle motion. That is correct for "stop the
current TRK" or "return to standby". If the user wants no idle motion, the caller
must clear idle first.

This distinction must remain clear in the skill:

- "stop current action" => standby, preserve idle.
- "stop and do not move / no idle" => idle clear, then standby.
- urgent wording => `/stop`.

## Recommended Product Logic

### P0: Safety Invariants

These must not change.

| Invariant | Required behavior |
| --- | --- |
| `/stop` is urgent | Immediate abort/stop semantics, clear idle, no standby transition. |
| `/passive` is a safety sink | Passworded explicit command or automatic safety path only; no smoothing or auto recovery. |
| `/fixstand` is recovery/preparation | Do not treat it as ordinary stop or a safety sink; no smoothing requirement. |
| Readiness or orientation failure | Safety state wins; do not continue a transition to satisfy smoothness. |
| User interrupt to a new TRK | Must not wait for ordinary standby convergence if a higher-priority user action is already known. |
| Transition remains interruptible | `/stop` and `/passive` must abort it immediately. |

### P1: Ordinary Standby Cancellation

For ordinary `/standby_velocity`, the preferred behavior should be:

| Source state | Recommended behavior |
| --- | --- |
| active user running with `active.kind=="user"`, `active_track_`, current source frame, and `standby_track_` | Best-effort synthetic transition from current user reference frame to standby reference; mark source user as stopped/canceled; finish in `standby_velocity`. |
| active user preparing without `active_track_` or a current source frame | Fall back to the existing stopping/cancel path; smooth standby transition is not promised without a source frame. |
| active user holding | Keep current holding-to-standby behavior. |
| active transition to idle/standby | If the user asked ordinary standby, continuing or rebuilding toward standby is acceptable; do not jump unless transition construction is unavailable. |
| active transition to user | Treat as foreground user cancellation: cancel target user and transition to standby if possible. |
| active idle | Product choice should remain conservative: standby stops current idle but preserves idle config. Smooth idle-to-standby is optional P2 because idle is background, not foreground user TRK. |

This should be an implementation improvement behind the existing empty-body
`/standby_velocity` endpoint, not a new API or request parameter.

### P2: Optional Refinements

| Item | Reason to defer |
| --- | --- |
| Smooth active idle to standby | Nice visual improvement, but not core "terminate user TRK". |
| Separate pure-standby command | Existing idle clear + standby is enough. |
| Per-request transition speed | Adds agent decision burden and API surface. Keep global `transition_duration_s`. |
| Direct active user interrupt via current-frame transition | Higher safety risk; existing controlled stop path is conservative. |
| Standby asset selection or upload | The app-owned standby reference is already the boundary; do not make it user motion or idle input. |

## Implementation Boundary

The smallest implementation should stay inside runtime control logic.

Recommended code boundary:

1. Extend `RuntimeControlLoop::handleControl(ControlMode::StandbyVelocity)` for
   foreground user cancellation.
2. Reuse `startSyntheticTransitionFromActiveFrame()` and standby reference
   alignment.
3. Preserve existing fallback to stopping when transition cannot be built.
4. Keep `/stop`, `/passive`, command priority, and `RuntimeBridge` stop
   watermark behavior unchanged.
5. Add a way to complete the source user as canceled/stopped instead of natural
   `Done` when the transition was caused by ordinary standby cancellation.
6. Do not allow standby reference playback from `Passive`, `Fault`,
   `lowcmd_occupied`, bad readiness, or bad orientation states except through
   the already-defined recovery/safety gates.

The current helper path hard-codes source user completion as `Done` when a
transition starts. That is correct for natural completion, but ordinary user
cancellation should publish a terminal state that communicates cancellation,
preferably `Stopped` with `StopReason::Stop`, not natural completion.

## Acceptance Criteria

### API Contract

- `/standby_velocity` remains an empty-body endpoint.
- No smooth/transition parameter is accepted by `/standby_velocity`, `/execute`,
  `/idle`, or `/stop`.
- `/stop` remains empty-body urgent stop and keeps its stop watermark semantics.
- `standby_ref.trk` remains internal app-owned reference data, not user motion,
  an idle pool item, or a fallback downloader target.

### Runtime Behavior

- Running user TRK + `/standby_velocity` enters `active.kind="transition"` with
  `transition.target="standby"` when a valid current frame and standby reference
  are available.
- Preparing user TRK without an active track/current source frame falls back to
  existing `Stopping`/terminal stop behavior.
- The transition has no run id, does not occupy queue limit, and is not returned
  as `exec`.
- During transition, status reports `active.kind="transition"`,
  `transition.target="standby"`, empty/null active id, and no user `exec`.
- The source user run is terminal and queryable as `stopped`/canceled, not
  natural done.
- Transition completion ends in public `ctrl:"standby_velocity"`.
- If idle config exists, it is preserved by standby semantics.
- If transition build fails, runtime falls back to the current safe stopping
  behavior.

### Safety Regression

- `/stop` still clears idle and does not play standby reference.
- `/stop` stays highest priority and preserves stop watermark behavior.
- `/passive` still aborts/clears immediately and does not auto recover.
- `/fixstand` remains explicit recovery/preparation, not ordinary stop.
- `/stop`, `/passive`, or `/fixstand` during the standby transition aborts or
  preempts immediately; they do not wait for transition completion.
- Readiness, bad orientation, lowcmd, model, or write failures still route to the
  existing safety/fault paths.
- `lowcmd_occupied` remains a manual block; standby reference does not auto-play
  through it.

### Interrupt Regression

- A new user TRK interrupt during standby transition can preempt that transition
  from the current reference frame.
- A queued user TRK during background standby/idle transition still follows the
  existing background-preempt rules.
- Foreground user interrupt policy remains conservative unless separately
  changed: do not introduce direct current-frame active-user interrupt as part
  of this work.

### Skill/Agent Contract

- `et1-action` continues mapping ordinary "stop/pause/do not continue" to
  `standby`.
- `et1-action` continues requiring urgent wording for `urgent-stop --urgent`.
- The skill docs should say standby may use an internal bounded transition, but
  agents must not choose between stop profiles or rely on a visual smoothness
  guarantee.
- The skill docs should keep the idle distinction explicit: standby preserves
  idle; clear idle first when the user asks for no background motion.

## Recommended Final Product Decision

Adopt the following product contract:

> Ordinary termination of a user TRK is `standby_velocity`. The tracker should
> use a bounded, interruptible internal transition toward the app-owned standby
> reference when safe and available. This is an implementation quality target,
> not a new external stop mode. `/stop` and `/passive` remain immediate
> safety/control boundaries and never wait for smoothing.

This is the best balance for the current product:

- It matches normal user expectations: "stop" means do not keep executing the
  current TRK.
- It keeps the LLM interface short and predictable.
- It avoids adding a new API or agent decision branch.
- It preserves emergency and passive safety semantics.
- It aligns manual cancellation with the already-smoothed natural completion
  path.
