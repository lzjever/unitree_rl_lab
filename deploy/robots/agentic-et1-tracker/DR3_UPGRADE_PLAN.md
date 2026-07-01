# DR3 Upgrade Plan

## Goal

Upgrade `deploy/robots/agentic-et1-tracker` to run the `DR3-all.onnx` GeneralTracker policy from the `chemyhao/unitree_ET1_lab` `mjlab_hyx` reference tree, while aligning the tracker runtime with the current C++ deploy timing semantics:

- policy/control period is `deploy_config.step_dt` (`0.02s`, 50 Hz);
- reference `fps` is only a sampling rate for `frame = round(playback_time * fps)`;
- `playback_time` advances by `env->step_dt` after each policy step;
- the control loop keeps an independent absolute phase with `sleep_until`, without wall-clock catch-up.

This is a targeted DR3 profile addition. Do not refactor unrelated tracker, LocoUpper, or packaging behavior while doing it.

## Confirmed References

- Reference repository: `/home/galbot/works/et1/.reference/unitree_ET1_lab_mjlab_hyx`
- Reference HEAD: `e3c50ad6075ea8f146b3f1fca89bd44b272868a5`
- DR3 model:
  - Path: `robots/et1/config/policy/mimic/general_tracking/exported/DR3-all.onnx`
  - Size: `3590360` bytes
  - SHA256: `fb48e575d81951e6cd23e65cde12258bd5feccfb8dfc3c9bce549d6c55269e12`
- DR3 deploy config:
  - Path: `robots/et1/config/policy/mimic/general_tracking/params/deploy_fut_obs.yaml`
  - `step_dt: 0.02`
  - Commented ONNX contract: `obs_current [1, 118]`, `obs_history [1, 5, 32]`, `actions [1, 26]`
  - Required switches: `use_motion_root_command=true`, `use_motion_velocity_command=false`, `use_future_motion_velocity_command=false`, `use_future_foot_support_command=false`
- Current agentic tracker root: `/home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker`

## Frequency Facts

Treat current `mjlab_hyx` HEAD as the final timing source. Commit `89bd671` (`update footstate4相关以及修改频率`) lives on `origin/footstate`; it is not an ancestor of HEAD `e3c50ad...`, so its decimation design is useful historical context only, not the target implementation.

The target behavior is the HEAD C++ deploy behavior:

- `State_Mimic::run_dt()` returns `0.02`.
- `State_Mimic::run()` samples the reference at `current_playback_time`, runs observation/policy/action once, then updates `playback_time_ = min(playback_time_ + env->step_dt, end_time)`.
- `ReferenceLoader::frame_index_from_time()` computes `round(time_s * fps)` and clamps to the track range.
- `CtrlFSM::run_loop_()` runs states in its own thread and uses `sleep_until(next_loop_start)` with absolute phase, resetting phase only on state change or large overrun.

Do not copy or recreate the unmerged `mimic_control_decimation` approach from `89bd671`.

## Timing Vocabulary / Timing Boundaries

Keep these timing domains separate. The DR3 change should make the boundaries more explicit, not merge them:

- Runtime tick: one `RuntimeControlLoop::tick()` execution, driven by `config.runtime.hz` from the app runner. It is the scheduler heartbeat, not the policy period.
- LowCmd publication: writes a `LowCmdFrame` to robot IO. GeneralTracker publishes only when its policy step is due; LocoUpper active phases keep their existing every-runtime-tick command write semantics.
- GeneralTracker policy step: one observation build, one ONNX inference, one action scaling, one LowCmd write. Its period must come from `deploy_config.step_dt`.
- `deploy_config.step_dt`: the policy/control period for GeneralTracker policy stepping. For DR3 this is `0.02s`; Velocity already follows this model via `velocity_deploy_config.step_dt`.
- Reference `fps`: the source motion sampling rate used only for `frame = round(playback_time * fps)`. It must not set policy cadence.
- `config.policy.fps` and TRK loader `fps`: metadata/fallback inputs used to populate `active.fps` and interpret reference frames. They are still reference sampling metadata, not a control loop rate.
- `.trk` `frame_count` and `duration_s`: metadata describing the source track. Do not redefine them as policy step count or policy duration.

Current agentic bug to fix:

- GeneralTracker `activePolicyIntervalTicks()` is bound to `active_->fps`, so a 30 fps or 60 fps track changes policy cadence.
- GeneralTracker frame advancement is due-step `++frame`, so it assumes one policy step equals one source frame.
- Correct behavior is `deploy_config.step_dt` driving policy steps and `frame = round(playback_time * fps)` sampling the source reference. Velocity already uses `step_dt`; GeneralTracker should use the same cadence vocabulary.

## DR3 Policy Contract

Add a new explicit profile/contract named `GeneralTrackerDR3`.

Expected tensors:

- input `obs_current`: float32 shape `[1,118]`
- input `obs_history`: float32 shape `[1,5,32]`
- output `actions`: float32 shape `[1,26]`

`obs_current` order and widths:

| Term | Width |
| --- | ---: |
| `command_yaw` | 2 |
| `command_root_ori_b` | 6 |
| `command_jnt_pos` | 26 |
| `projected_gravity` | 3 |
| `base_ang_vel` | 3 |
| `joint_pos_rel` | 26 |
| `joint_vel_rel` | 26 |
| `last_action` | 26 |

`obs_history` is non-temporal future command data:

- term name is singular: `future_command`
- horizon is `5`
- row width is `32`
- each row is `root_ori_b(6) + joint_pos(26)`
- it does not include `xy_yaw_vel`
- it does not include foot support state

The action map, SDK joint map, gains, action scale/offset, and `step_dt` should be imported from `deploy_fut_obs.yaml` without semantic changes.

## Why Agentic Cannot Run DR3 Today

The current agentic policy stack supports these deploy contracts only:

- `GeneralTracker`: `obs_current [1,131]`, temporal `obs_history [1,25,105]`
- `GeneralTrackerCLN`: `obs_current [1,121]`, future `obs_history [1,25,35]`, term `future_commands`
- `GeneralTrackerCLNFootstate`: `obs_current [1,127]`, future `obs_history [1,5,41]`, term `future_command_with_foot_support_state`

DR3 does not fit any of them:

- `deploy_config` detects CLN via plural `future_commands`; DR3 uses singular `future_command`.
- CLN current observations require `command_xy_yaw_vel`; DR3 omits it.
- Footstate CLN adds `command_foot_support_state` and 41-wide future rows; DR3 omits footstate.
- `policy_io_contract` has no `[118] / [5,32]` validation path.
- `observation_builder` always builds CLN future rows as root orientation + xy/yaw velocity + joints, optionally footstate; it cannot produce root orientation + joints only.
- `app_config` and `runtime_factory` only map known profile names to existing `ObservationContract` values.

## Implementation Scope

Add only the minimum new surface needed for DR3.

### 1. Assets and Config

- Add `config/policy/general_tracker_dr3/exported/DR3-all.onnx`.
- Add `config/policy/general_tracker_dr3/params/deploy_fut_obs.yaml`.
- Add `config/policy/general_tracker_dr3/README.md` and `ASSET_MANIFEST.yaml`.
- Record the source reference HEAD and model SHA256 in the manifest/readme.
- Add DR3 as a selectable `GeneralTrackerDR3` profile first and keep the current default profile unchanged.
- Switch `config.yaml` and `packaging/config.robot.yaml.template` defaults only in a separate follow-up after tests and sim acceptance pass, unless the development task explicitly requires switching defaults in the same change.

### 2. Deploy Config

- Add `ObservationContract::GeneralTrackerDR3`.
- Add frozen DR3 observation specs:
  - current terms listed above, total `118`;
  - history term `future_command`, width `32`, horizon `5`.
- Parse `deploy_fut_obs.yaml` by detecting `obs_history.future_command`.
- Keep `step_dt` required and validated as `0.02` for this frozen profile.
- Reject accidental `future_commands`, `command_xy_yaw_vel`, and footstate terms in DR3 configs.

### 3. Policy IO Contract

- Add constants for DR3 dimensions: current `118`, history length `5`, width `32`, action `26`.
- Extend deploy config validation to check DR3 exact terms, offsets, maps, and dimensions.
- Extend ONNX metadata validation to accept `obs_current`, `obs_history`, and `actions` exactly as DR3 requires.
- Keep existing GA/CLN/footstate contracts unchanged.

### 4. Observation Builder and Policy Math

- Extend `PolicyObservationParts` or the term lookup so `future_command` is a first-class term, not an alias that hides plural CLN behavior.
- For DR3 current observations:
  - keep `command_yaw`;
  - compute `command_root_ori_b` from motion root orientation;
  - do not compute or append `command_xy_yaw_vel`;
  - do not compute or append `command_foot_support_state`.
- For DR3 future history:
  - build 5 clamped future rows;
  - each row appends future `root_ori_b(6)` and future `joint_pos(26)`;
  - do not append velocity or footstate;
  - preserve `use_motion_root_command=true` semantics from the reference config.
- Keep temporal history behavior only for legacy `GeneralTracker`; DR3 uses direct future command history like CLN.

### 5. App Config and Runtime Factory

- Add profile string `GeneralTrackerDR3`.
- Map `GeneralTrackerDR3` to `ObservationContract::GeneralTrackerDR3` in `policyProfileMatchesDeployContract`.
- Set suggested paths:
  - `policy_dir: config/policy/general_tracker_dr3`
  - `policy_file: DR3-all.onnx`
  - `deploy: config/policy/general_tracker_dr3/params/deploy_fut_obs.yaml`
- Preserve path guards that prevent depending on the ET1 app deploy tree at runtime.

### 6. Runtime Scheduling

Change GeneralTracker policy cadence to use the deploy config, not the track FPS:

- Do not implement GeneralTracker cadence as a fixed integer tick interval from `ceil(step_dt * config_.hz)`. That is wrong for non-integer ratios; for example `runtime_hz=60` and `step_dt=0.02` gives `ceil(1.2)=2`, which would run at 30 Hz.
- Use one direct absolute/fractional phase accumulator for GeneralTracker policy due checks:
  - keep a policy phase in policy-time units, initialized when the active playback starts;
  - each runtime tick advances the scheduler phase by `1.0 / config_.hz`;
  - a policy step is due when the phase reaches the next `deploy_config.step_dt` boundary;
  - after one due step, advance the next boundary by exactly `deploy_config.step_dt`.
- Track `playback_time` per active GeneralTracker playback.
- On each policy step:
  - compute `frame = referenceFrameIndex(playback_time, active.fps, active.frames)`;
  - publish status/reference for that sampled frame;
  - run policy once;
  - advance `playback_time += deploy_config.step_dt`, clamped to duration/end.
- Keep the existing rule that each runtime tick executes at most one GeneralTracker policy step. If the runtime tick is late enough to cross multiple policy boundaries, run one step and leave the rest skipped; do not loop to catch up.
- `policyStartupHoldPolicySteps()` must compute startup hold count from `deploy_config.step_dt`, not from reference FPS or `active_->fps`. The same startup hold seconds must produce the same policy-step count for 30, 50, and 60 fps tracks.
- Do not change `.trk` `fps`, `frame_count`, or `duration_s` semantics.
- Do not resample, interpolate, insert, or drop reference frames. Sampling is by rounded source-frame lookup only.
- Do not add wall-clock catch-up loops. If the runtime tick is late, execute at most one policy step for that tick.
- Keep app-level runtime thread absolute-phase scheduling (`wait_until`/`sleep_until` style). Do not make policy timing depend on elapsed wall time since `started_at`.
- Keep LocoUpper semantics untouched: it currently writes commands every runtime tick for its active phases and has its own lower-policy decimation. Do not route LocoUpper through the DR3 policy period change.

## Packaging and Self-Test Checklist

- Include DR3 ONNX and deploy YAML in release workspace preparation.
- Include DR3 `ASSET_MANIFEST.yaml` in package verification.
- Ensure `selftest.sh` checks that the configured profile's ONNX and deploy YAML exist and that the ONNX SHA256 matches the manifest when DR3 is selected.
- Add a dry model-load selftest for `GeneralTrackerDR3` in sim/offline mode.
- Keep default config and robot template on the previous profile for the first DR3 landing unless a task explicitly says to switch them. If a later default-switch change is made, switch both together to avoid a split-brain package.
- Do not package files directly from `/home/galbot/works/et1/.reference/...`; copy assets into the agentic config tree.

## TDD Test Plan

Implement tests before or alongside each change:

- `deploy_config_tests.cpp`
  - loads DR3 `deploy_fut_obs.yaml`;
  - detects `GeneralTrackerDR3`;
  - verifies dimensions, term order, offsets, horizon `5`, width `32`, `step_dt=0.02`;
  - rejects plural `future_commands`, velocity-in-future, and footstate variants for DR3.
- `policy_io_contract_tests.cpp`
  - accepts DR3 metadata `[1,118]`, `[1,5,32]`, `[1,26]`;
  - rejects CLN/footstate shapes under `GeneralTrackerDR3`;
  - rejects DR3 shapes under old profiles.
- `observation_builder_tests.cpp`
  - verifies DR3 current obs size and term contents;
  - verifies future rows are `root_ori_b + joint_pos` only;
  - verifies horizon clamping at end of track.
- `policy_step_runner_tests.cpp`
  - verifies DR3 bypasses temporal `HistoryBuffer` and passes future command history directly;
  - verifies last action updates after inference.
- `onnx_policy_runtime_tests.cpp`
  - loads `DR3-all.onnx`;
  - validates names, dtypes, shapes, and one deterministic zero/synthetic inference call.
- `runtime_control_loop_tests.cpp`
  - verifies GeneralTracker policy steps are scheduled by `deploy_config.step_dt`, not `track.fps`;
  - verifies `frame = round(playback_time * fps)` for non-50 fps tracks;
  - verifies a non-50 fps track keeps policy cadence from `step_dt` while frame sampling follows source `fps`;
  - verifies non-integer `runtime_hz / (1 / step_dt)` ratios use fractional/absolute phase and do not create systematic slow playback;
  - verifies `runtime_hz=60` with `step_dt=0.02` does not degrade to a fixed 2-tick / 30 Hz cadence;
  - verifies no multi-step wall-clock catch-up after a delayed tick;
  - verifies `started_at` wall-clock elapsed time is not used to catch up policy/frame progression;
  - verifies startup hold duration is identical in seconds for 30, 50, and 60 fps tracks and is driven by `deploy_config.step_dt`;
  - verifies LocoUpper still writes every runtime tick.
- `app_config_tests.cpp` and `app_runtime_factory_tests.cpp`
  - accept `GeneralTrackerDR3`;
  - reject profile/deploy contract mismatches;
  - resolve DR3 package paths without reference-tree dependency.
- Package selftest
  - validates asset presence, manifest SHA256, and sim model-load path.
- Sim acceptance
  - run at least one `.trk` through sim/offline runtime with DR3;
  - confirm motion progresses at 50 Hz policy cadence and sampled frames match the reference FPS;
  - confirm no regression in existing CLN/footstate smoke tests.

## Implementation Order

1. Add DR3 asset directory, manifest, and deploy YAML.
2. Add `GeneralTrackerDR3` deploy config parsing and contract tests.
3. Add policy IO metadata validation for DR3.
4. Add observation builder and policy math support for singular `future_command`.
5. Add app config/profile/runtime factory support.
6. Change GeneralTracker policy scheduling to `deploy_config.step_dt` with explicit `playback_time`.
7. Update packaging and selftest.
8. Run unit tests, ONNX load test, package selftest, then sim acceptance.

## Acceptance Criteria

- `GeneralTrackerDR3` loads `DR3-all.onnx` and `deploy_fut_obs.yaml` from the agentic package tree.
- ONNX metadata validation passes only for `obs_current [1,118]`, `obs_history [1,5,32]`, `actions [1,26]`.
- DR3 observations contain no `command_xy_yaw_vel` and no footstate in current or future tensors.
- GeneralTracker policy cadence is 50 Hz from `step_dt=0.02`; reference FPS only affects frame sampling.
- GeneralTracker policy due checks use absolute/fractional phase, not fixed `ceil(step_dt * runtime_hz)` tick intervals.
- Runtime executes at most one policy step per runtime tick and does not catch up based on wall-clock elapsed time.
- Startup hold policy-step count is derived from `deploy_config.step_dt`, not reference FPS.
- `.trk` `fps`, `frame_count`, and `duration_s` keep their existing meaning; no resampling, interpolation, frame insertion, or frame dropping is introduced.
- LocoUpper command publication and lower-policy decimation behavior are unchanged.
- Hard fail-fast: `GeneralTrackerDR3` mixed with legacy, CLN, or Footstate deploy YAML must fail startup.
- Hard fail-fast: `GeneralTrackerDR3` mixed with legacy, CLN, or Footstate ONNX tensor contracts must fail startup.
- Existing GeneralTracker, CLN, CLNFootstate, Velocity, FixStand, and LocoUpper tests continue to pass.
- Release package selftest verifies DR3 assets and can load the model in sim/offline mode.

## Scope Guards

- KISS: add one new contract/profile; do not generalize an arbitrary policy schema system unless tests prove it is necessary.
- DRY: share existing vector/map/gain parsing and action scaling; only fork the observation contract specifics.
- YAGNI: do not import `89bd671` decimation code, new controller modes, new motion formats, or runtime dependencies from the reference app.
- Do not modify LocoUpper timing, standby/idle state-machine behavior, HTTP APIs, or `.trk` schema except where tests show DR3 playback requires a narrowly scoped status field for `playback_time`.
