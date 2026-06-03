# Agentic ET1 Tracker Acceptance Evidence

Date: 2026-05-29
Latest targeted update: 2026-06-04
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

2026-06-04 CLN model/release package verification for this CLN release
packaging change set:

- App-owned CLN model was byte-identical to the updated ET1 source asset
  `deploy/robots/et1/config/policy/general_tracker_cln/exported/multi_policy_v17c2_70k.onnx`;
  sha256 `d4f37c972eb5e98e37a1d425302a70729343009a1564974921965c5faea0d911`.
- `agentic_et1_tracker_policy_onnx_tests`:
  `OnnxPolicyRuntime constructs and runs the app-owned GeneralTrackerCLN policy`
  passed.
- `agentic_et1_tracker_app_tests "*AppConfig*" "~*simulation example*"`:
  193 assertions / 21 test cases passed; the local sim example test was
  excluded because the local working tree has an operator-specific
  `motion_dirs` edit.
- Temporary x86_64 release package built with
  `-DAGENTIC_ET1_BUILD_ROBOT=OFF`; packaged `scripts/selftest.sh` passed,
  verified the CLN model hash, and verified the unused legacy
  `config/policy/general_tracker` tracker policy directory is absent while
  `general_tracker_cln` and `velocity/v0` remain packaged.
- `git diff --check`, `bash -n packaging/build_release.sh`, and
  `bash -n packaging/scripts/selftest.sh`: passed.

This evidence is limited to model selection, config tests, and release package
selftest. It does not claim broader MuJoCo/operator acceptance or real-robot GA
completion.

2026-06-04 package/skill verification for commit `eb5be7602d796bcdba72e1f660333761864f9ab0`:

- Commit: `eb5be7602d796bcdba72e1f660333761864f9ab0`.
- Temporary x86_64 release package built from a clean detached HEAD worktree
  with `-DAGENTIC_ET1_BUILD_ROBOT=OFF` because local `unitree_sdk2` was not
  available for the default ROBOT release configure.
- `/tmp/agentic-et1-tracker-head-selftest-unpack/agentic-et1-tracker-head-eb5be76-selftest-x86_64/scripts/selftest.sh`:
  passed; verified packaged default CLN assets, `standby_ref.trk`, manifest
  files, and bundled binary/CLI help.
- `python3 deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion/tests/test_et1_trk2motion.py`:
  26/26 passed.

This evidence is limited to package selftest and skill tests. It does not
claim broader MuJoCo/operator acceptance or real-robot GA completion.

2026-06-03 targeted docs/skill/release verification:

- `cmake --build /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/build --target agentic_et1_tracker_core_tests agentic_et1_tracker_trk_tests agentic_et1_tracker_api_tests agentic_et1_tracker_runtime_tests`:
  passed.
- `ctest --test-dir /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/build -R 'agentic_et1_tracker_(core|trk|api|runtime)_tests' --output-on-failure`:
  4/4 passed.
- `python3 /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion/tests/test_et1_trk2motion.py`:
  21/21 passed.
- `python3 -m py_compile /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion/scripts/et1-trk2motion /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion/tests/test_et1_trk2motion.py`:
  passed.
- `git diff --check -- deploy/robots/agentic-et1-tracker/ACCEPTANCE.md`:
  passed.

This targeted verification did not execute MuJoCo visual acceptance or real
robot acceptance. At that point there was not yet a recorded app-local
`standby_ref.trk` release asset; see the later same-day standby reference asset
evidence below for the simulator-accepted app-local asset promotion.

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

- Default CLN policy: `GeneralTrackerCLN` at
  `config/policy/general_tracker_cln`
- Release packages carry this CLN tracker policy and omit the unused legacy
  `config/policy/general_tracker` tracker policy directory.
- StandbyVelocity: `config/policy/velocity/v0`
- FixStand posture: `config/posture/fixstand/v0/fixstand.yaml`
- Passive posture: `config/posture/passive/v0/passive.yaml`
- Standby reference: `config/reference/standby/v0/standby_ref.trk`
  with simulator acceptance recorded and real-robot validation pending.

## Offline standby_ref candidate evidence

2026-06-03 local offline candidate generation for later MuJoCo acceptance:

- Command:
  `python3 deploy/robots/agentic-et1-tracker/tools/derive_standby_ref_candidate.py --source /home/galbot/works/et1/generated/squat_stand.trk --out-dir deploy/robots/agentic-et1-tracker/build/standby_ref_candidate --tail-frames 25 --fps 50`
- Output:
  `deploy/robots/agentic-et1-tracker/build/standby_ref_candidate/standby_ref.candidate.trk`
  and `CANDIDATE_MANIFEST.json`.
- Manifest status: `candidate_pending_mujoco_acceptance`; this candidate is
  build-local evidence only and is not a release asset.
- Source: `/home/galbot/works/et1/generated/squat_stand.trk`,
  sha256 `e748af2d54ac6a08bfcf8e242d2f9c75e28a387af978716213b3e114349ee5f8`,
  frames `249`, exact tail window `224..248`.
- Candidate: sha256
  `6ca49404e1ee1008f6226a2f7c00e990f0447ae6c826657246b7a29fbb525741`,
  size `41400` bytes, frames `25`, fps `50.0`, duration `0.48` s.
- Static metrics: `joint_max_drift=0.0181495845`,
  `root_xyz_drift=0.00115438467`, `root_tilt_max_deg=3.02591769`,
  `root_lin_vel_max=0.0127365116`, left/right contact constant with values
  `[0]`.
- `python3 /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/tools/test_derive_standby_ref_candidate.py`:
  11/11 passed.
- `python3 -m py_compile /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/tools/derive_standby_ref_candidate.py /home/galbot/works/et1/unitree_rl_lab/deploy/robots/agentic-et1-tracker/tools/test_derive_standby_ref_candidate.py`:
  passed.

This candidate generation step alone did not record a `standby_ref.trk` release
asset. The later release asset promotion below records the accepted app-local
asset. Runtime gate coverage is now recorded below; broader MuJoCo/operator and
real-robot GA gates remain pending.

## Standby_ref release asset evidence

2026-06-03 targeted standby reference asset promotion after simulator review:

- User acceptance input: the candidate was reviewed in simulator and accepted
  for use as `standby_ref`.
- The build-local candidate was copied byte-for-byte to
  `deploy/robots/agentic-et1-tracker/config/reference/standby/v0/standby_ref.trk`.
- Release manifest and local README were added under
  `config/reference/standby/v0/`.
- Release asset sha256:
  `6ca49404e1ee1008f6226a2f7c00e990f0447ae6c826657246b7a29fbb525741`,
  size `41400` bytes, frames `25`, fps `50.0`, duration `0.48` s.
- Runtime gate status: app-local `standby_ref.trk` load/playback is wired into
  runtime and covered by unit/runtime/release tests.
- Status: release asset recorded and simulator visual acceptance recorded for
  this asset; broader MuJoCo/operator validation and real-robot/operator
  validation remain pending. This does not claim overall GA.

2026-06-03 targeted standby_ref runtime gate evidence:

- `agentic_et1_tracker_api_tests`: active transition `/status` JSON contract,
  including `active.kind:"transition"`, `exec:null`, transition progress, and
  user-only queue ids.
- `agentic_et1_tracker_app_tests`: user `/execute` of app-local
  `config/reference/standby/v0/standby_ref.trk` remains rejected by the user
  motion allowlist, with no user queue/history entry; missing/damaged internal
  standby asset reports not-ready without ET1 app tree fallback.
- `agentic_et1_tracker_runtime_tests`: runtime standby_ref playback/abort gate
  coverage for natural return to standby and control interruption.
- Release selftest covers packaged `standby_ref.trk` and manifest presence.

## GA idle/status/control acceptance items

Contract-level GA evidence must cover:

- `/execute` accepts only `path`, optional `mode`, and optional boolean `hold`;
  extra fields and `paths` return 400 `REQUEST_INVALID` before validator, sink,
  or id allocation.
- `/idle {"paths":[...]}` atomically configures the idle pool; `{"paths":[]}`
  clears it; idle never creates a user run id.
- `/status.active.kind` is authoritative. `exec` and `queue` describe user runs
  only; idle progress lives under `idle` and is not queryable by
  `GET /status?id=...`.
- `/stop` stops idle, clears idle config, and preserves the user stop watermark
  so work accepted after the stop is not canceled by the older stop.
- `block:"lowcmd_occupied"` maps to manual/operator action and must not imply
  automatic wait/retry recovery.
- `bad_orientation` enters the safety path; `/passive` and `/fixstand` are the
  software recovery exceptions when LowCmd is free.
- `/passive` clears active work, user queue, pending idle config, and idle
  status so later FixStand -> StandbyVelocity cannot resume old idle playback.
- Real `mode_machine: 1` startup may release Unitree MotionSwitcher default
  mode before LowCmd preflight. Sim `mode_machine: 0` startup must not call
  MotionSwitcher.

## Manual MuJoCo acceptance

No broad fresh MuJoCo visual acceptance has been recorded for the current
revision beyond the targeted `standby_ref.trk` simulator review above. Earlier
MuJoCo evidence predated the current FixStand/StandbyVelocity/FSM semantics and
is historical only. No idle/status/control MuJoCo acceptance has been recorded
in this docs/skill pass.

This section records an acceptance operation-order correction only. It does
not change code semantics, add an API, or claim that MuJoCo visual acceptance
has completed.

Use `config.sim.yaml.example` as the starting point for local MuJoCo
acceptance. The acceptance config should keep sim-safe settings such as
`mode_machine: 0`, `network: "lo"`, app-owned policy/control assets, the default
`200` ms LowCmd owner preflight, and disabled MotionSwitcher release. Set
`motion_dirs` to the test `.trk` directory for the run; the recommended local
test directory remains `/home/galbot/works/et1/generated`, but the current local
config may need adjustment to match the chosen test directory.

Smoke and normal acceptance runs must not begin by sending `/passive` when
`/status` already reports `ready:true` and `ctrl:"standby_velocity"`. Passive is
the safety sink/damping state; in MuJoCo without rope or operator support it can
let the robot fall and trigger `ROBOT_BAD_ORIENTATION`. Send `/passive` only in
a dedicated passive safety-sink scenario, and prepare MuJoCo reset/upright state
or operator support before doing so.

When recovering from `bad_orientation`, call `/fixstand`, wait until `/status`
shows `ready:true`, `ctrl:"fixstand"`, `block:null`, and `err:null`, then call
`/standby_velocity`.

Pending MuJoCo evidence must record:

- Startup LowCmd owner preflight scoped to the same DDS `network` and
  `domain_id`; `lowcmd_occupied` must prevent any writing runtime from
  starting.
- Sim config must not call MotionSwitcher. Real config should release Unitree
  default motion mode before LowCmd preflight and report
  `motion_mode_release_failed` if release fails.
- Startup `/status` with `ctrl:"fixstand"` when using default config.
- `/status.pose` with compact `q/g/p/v` fields during idle and running states.
- Manual `/fixstand` and `/standby_velocity` requests accepted with empty body
  only when `/status` shows a runtime that can consume them.
- Dedicated `/passive` safety-sink scenario only after MuJoCo reset/upright or
  operator support is prepared; do not use `/passive` as a smoke/acceptance
  prologue from an already ready `standby_velocity` state.
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
- `lowcmd_occupied -> manual` and `bad_orientation -> passive` with recovery
  through `/fixstand`, confirmed `ready:true ctrl:"fixstand" block:null
  err:null`, then `/standby_velocity`.
- Fault/disconnect handling and latency/performance evidence.

## GA gates

| gate | status | required evidence |
| --- | --- | --- |
| Default ROBOT/ONNX build | recorded above | Keep default configure/build on the real integration path. |
| Hermetic stub tests | recorded above | Keep explicit stub/test configure and tests passing. |
| Docs/skill CLI contract | recorded above | Keep packaged and installed skill tests/diffs passing. |
| `standby_ref.trk` release asset and runtime gate | unit/runtime/release covered; real robot pending | App-local asset, manifest, internal runtime playback, abort behavior, user allowlist rejection, and internal asset failure behavior are recorded; hardware/operator validation remains pending before GA. |
| MuJoCo visual acceptance | partial; pending | Targeted standby_ref simulator review and automated runtime gate coverage are recorded; broader scenarios listed above still need `config.sim.yaml.example` evidence. |
| Real robot acceptance | pending | ET1 hardware/operator validation. |

Do not mark GA until the pending broader MuJoCo/control and real-robot gates
are complete.

## Remaining external pending

True end-to-end MuJoCo control acceptance and real-robot validation are still
pending. The real-robot gate requires ET1 hardware and an operator window. The
standby reference asset is app-owned, simulator accepted, and runtime-gated by
unit tests, but hardware validation remains pending before GA.
