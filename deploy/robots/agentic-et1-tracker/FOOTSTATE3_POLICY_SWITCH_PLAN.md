# agentic-et1-tracker Footstate3 Policy Switch Plan

## Status

Handoff-ready plan. This document covers the work required to switch
`agentic-et1-tracker` to ET1's newer `multi_policy_footstate3.onnx` policy and
`deploy_fut_multi_footstate.yaml` deploy config.

Team review conclusion: this is not a file-name-only asset swap. The new policy
uses a different frozen observation contract, so the implementation must update
the parser, observation builder, policy math, IO contract checks, tests,
defaults, and release packaging together.

## Principles

- KISS: add the smallest explicit contract needed for footstate3; do not add a
  policy registry, downloader, dynamic observation DSL, or generic model
  framework.
- DRY: reuse the existing CLN future-command construction path where the row
  layout is identical, and add only the 6-dim foot-support suffix where needed.
- YAGNI: do not change HTTP APIs, queue behavior, idle/standby/passive semantics,
  release install flow, or robot control state machine for this task.
- Fail fast: mismatched profile, deploy YAML, and ONNX metadata must fail during
  startup/runtime initialization, not during robot motion.
- Asset boundary: repo defaults and release templates must use tracker-owned
  assets under `deploy/robots/agentic-et1-tracker/config`; runtime may still use
  already-supported external absolute `policy_dir` values such as installed
  release asset directories. It must never point directly under
  `deploy/robots/et1/config`, and `deploy` must stay inside the selected
  `policy_dir/params` tree.

## Source Facts

ET1 now points its `GeneralTrackerCLN` app state at:

| Asset | Source path | sha256 | Size |
|---|---|---:|---:|
| Policy | `deploy/robots/et1/config/policy/general_tracker_cln/exported/multi_policy_footstate3.onnx` | `3afdd52f115dc01b042cd3f1be40c90a2affcae5987e5cb5e52442a2115b37d7` | 3,650,408 |
| Deploy YAML | `deploy/robots/et1/config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml` | `89734594308d6e036d348f80e6fa2fa7e224d1e9a4da4ac2b53bb68de91095fc` | 3,638 |

Current `agentic-et1-tracker` default still uses:

- `config/policy/general_tracker_cln/exported/multi_policy_v17c2_70k.onnx`
- `config/policy/general_tracker_cln/params/deploy.yaml`
- old CLN shape: `obs_current=[1,121]`, `obs_history=[1,25,35]`,
  `actions=[1,26]`

New footstate3 shape:

- `obs_current=[1,127]`
- `obs_history=[1,5,41]`
- `actions=[1,26]`

New observation layout:

- `obs_current`: old CLN 121 dims plus `command_foot_support_state(6)` at
  offset 121.
- `obs_history`: one term named `future_command_with_foot_support_state`.
- Each future row is
  `command_root_ori_b(6) + command_xy_yaw_vel(3) + command_jnt_pos(26) + foot_support_onehot(6)`.
- Foot support one-hot comes from `.trk` frame fields:
  `left_foot_contact_state` and `right_foot_contact_state`, each allowed
  values `0..2`; layout is left 3 dims followed by right 3 dims.

## Scope

### Goals

1. Make footstate3 the default policy for simulator and release templates.
2. Keep old CLN policy support available as an explicit compatibility path for
   existing configs and rollback.
3. Preserve current HTTP service behavior and robot state machine semantics.
4. Add tests that lock the exact deploy contract and ONNX IO contract.
5. Package the new assets as tracker-owned release assets.

### Non-Goals

1. Do not change `.trk` public input format.
2. Do not accept `.et1trk` as a new public input.
3. Do not consume `torque_limit` in P0. The current tracker command path sends
   PD targets with `tau=0`; adding runtime torque limiting is a separate safety
   design if robot validation shows it is required.
4. Do not directly wrap or link ET1's app implementation.
5. Do not remove old CLN assets in the first transition release.

## Design Decision

Add a new explicit observation contract:

```cpp
enum class ObservationContract {
  GeneralTracker,
  GeneralTrackerCLN,
  GeneralTrackerCLNFootstate,
};
```

Use a matching explicit policy profile string:

- `GeneralTracker`: existing 131 / 25x105 legacy tracker contract.
- `GeneralTrackerCLN`: existing 121 / 25x35 CLN contract.
- `GeneralTrackerCLNFootstate`: new 127 / 5x41 footstate3 contract.

Default configs should switch to `GeneralTrackerCLNFootstate`. The old
`GeneralTrackerCLN` profile stays supported so existing installed configs do not
silently break when a release package is upgraded.

This is more explicit than reusing `GeneralTrackerCLN` for two incompatible
shapes and keeps startup validation simple.

## Implementation Plan

### 1. Copy App-Owned Assets

Copy the ET1 footstate3 files into the tracker-owned policy directory:

- Add
  `deploy/robots/agentic-et1-tracker/config/policy/general_tracker_cln/exported/multi_policy_footstate3.onnx`
- Add
  `deploy/robots/agentic-et1-tracker/config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml`

Keep the old `multi_policy_v17c2_70k.onnx` and `params/deploy.yaml` in the
package for one transition release. They are not defaults, but existing
installed robot configs may still point at them.

Update:

- `config/policy/general_tracker_cln/ASSET_MANIFEST.yaml`
- `config/policy/general_tracker_cln/README.md`

The manifest should mark footstate3 as the default required contract and may
list old CLN files as compatibility assets.

### 2. Update Config Defaults

Change the default policy profile and files in:

- `include/agentic_et1_tracker/app/app_config.hpp`
- `config.yaml`
- `config.sim.yaml.example`
- `packaging/config.robot.yaml.template`

Target values:

```yaml
policy:
  profile: "GeneralTrackerCLNFootstate"
  policy_dir: "config/policy/general_tracker_cln"
  policy_file: "multi_policy_footstate3.onnx"
  deploy: "config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml"
  fps: 50
```

For release template, keep the same paths under
`@PREFIX@/current/share/agentic-et1-tracker/...`.

### 3. Extend Deploy Config Parsing

Update:

- `include/agentic_et1_tracker/policy/deploy_config.hpp`
- `src/deploy_config.cpp`

Required behavior:

- Detect old CLN by `obs_history.future_commands`.
- Detect footstate3 by `obs_history.future_command_with_foot_support_state`.
- Set `ObservationContract::GeneralTrackerCLNFootstate` for footstate3.
- Validate `obs_current.use_gym_history == true`.
- Validate footstate3 `obs_history.use_gym_history == false`.
- Validate footstate3 history term `params.horizon == 5`.
- Validate term order exactly and reject drift.
- For every footstate3 observation term, keep the existing frozen metadata
  checks: `clip` and `scale` must be null or absent, and `history_length` must
  match the required value for that group.
- Validate term widths:
  - `obs_current_dim == 127`
  - `obs_history_width == 41`
  - `obs_history_length == 5`

Do not loosen validation to accept arbitrary observation names.

### 4. Extend Observation Builder

Update:

- `src/observation_builder.cpp`
- `include/agentic_et1_tracker/policy/policy_math.hpp` only if a new field is
  clearer than reusing the existing future buffer.

Required behavior:

- Fill `command_foot_support_state` for `GeneralTrackerCLNFootstate`.
- Generate footstate3 future rows using the existing CLN future command logic
  plus 6 dims of foot-support one-hot.
- Clamp future frame index to the final track frame, matching current CLN
  future behavior.
- Continue rejecting invalid contact values outside `0..2`.
- Keep `no_global_mode` yaw/root/velocity handling identical to old CLN.

Recommended minimal internal representation:

- Keep using `PolicyObservationParts.future_commands` as the buffer for
  non-temporal CLN-style future payloads.
- Let `policy_math` map both `future_commands` and
  `future_command_with_foot_support_state` to that same buffer.

This avoids adding duplicate storage names that do not change runtime behavior.

### 5. Update Policy Math and IO Contract

Update:

- `src/policy_math.cpp`
- `include/agentic_et1_tracker/policy/policy_io_contract.hpp`
- `src/policy_io_contract.cpp`
- `src/policy_step_runner.cpp`

Required behavior:

- Treat both CLN contracts as non-temporal history contracts.
- Validate footstate3 ONNX inputs:
  - input count: 2
  - `input[0]`: `obs_current`, float32, `[1,127]`
  - `input[1]`: `obs_history`, float32, `[1,5,41]`
  - output: `actions`, float32, `[1,26]`
- Validate footstate3 deploy terms exactly.
- Keep old `GeneralTrackerCLN` validation for compatibility.

### 6. Update Runtime Factory Contract Matching

Update:

- `src/app_config.cpp`
- `src/app_runtime_factory_real.cpp`
- related app config tests

Required behavior:

- Accept `policy.profile` values:
  - `GeneralTracker`
  - `GeneralTrackerCLN`
  - `GeneralTrackerCLNFootstate`
- Require profile and deploy observation contract to match exactly.
- Keep rejecting config paths that point into `deploy/robots/et1/config`.
- Keep rejecting `deploy` paths that escape the selected `policy_dir/params`
  tree.
- On mismatch, fail before robot control starts and report `MODEL_NOT_READY` or
  compact config error as current app conventions require.

### 7. Update Packaging

Update:

- `packaging/scripts/selftest.sh`
- `packaging/build_release.sh` only if it currently removes needed policy dirs
- `packaging/config.robot.yaml.template`

Required behavior:

- Release package includes footstate3 ONNX/YAML.
- Release package keeps old CLN assets for transition compatibility.
- Selftest checks footstate3 sha256.
- Selftest confirms the template default points at `multi_policy_footstate3.onnx`
  and `deploy_fut_multi_footstate.yaml`.
- If a lightweight offline policy metadata probe is available, selftest should
  run it; otherwise the C++ ONNX runtime test is the source of truth.

## File-Level Checklist

| File | Planned change |
|---|---|
| `config/policy/general_tracker_cln/exported/` | Add `multi_policy_footstate3.onnx`; keep old CLN ONNX for compatibility |
| `config/policy/general_tracker_cln/params/` | Add `deploy_fut_multi_footstate.yaml`; keep old `deploy.yaml` |
| `config/policy/general_tracker_cln/ASSET_MANIFEST.yaml` | Update default contract shape/hash/size |
| `config/policy/general_tracker_cln/README.md` | Document footstate3 as default and old CLN as compatibility |
| `include/agentic_et1_tracker/app/app_config.hpp` | Default profile/file/deploy to footstate3 |
| `config.yaml` | Default profile/file/deploy to footstate3 |
| `config.sim.yaml.example` | Default profile/file/deploy to footstate3 |
| `packaging/config.robot.yaml.template` | Default profile/file/deploy to footstate3 |
| `include/agentic_et1_tracker/policy/deploy_config.hpp` | Add `GeneralTrackerCLNFootstate` enum value |
| `src/deploy_config.cpp` | Parse and validate footstate3 deploy YAML |
| `src/observation_builder.cpp` | Build current and future foot-support observations |
| `src/policy_math.cpp` | Map new future term and non-temporal history behavior |
| `src/policy_step_runner.cpp` | Treat footstate3 like CLN for future-history mode |
| `include/agentic_et1_tracker/policy/policy_io_contract.hpp` | Add footstate3 dimensions |
| `src/policy_io_contract.cpp` | Validate footstate3 deploy and model metadata |
| `src/app_config.cpp` | Accept and validate footstate3 profile |
| `src/app_runtime_factory_real.cpp` | Match profile to deploy contract exactly |
| `tests/*` | Add/adjust TDD coverage listed below |
| `README.md` / `ACCEPTANCE.md` | Update default policy docs and validation status |

## TDD Plan

Write or update tests before production changes where practical.

| Test area | Required assertions |
|---|---|
| `tests/deploy_config_tests.cpp` | Real `deploy_fut_multi_footstate.yaml` loads as `GeneralTrackerCLNFootstate`; dims are 127, 5, 41 |
| `tests/deploy_config_tests.cpp` | Reject missing `command_foot_support_state`, missing `future_command_with_foot_support_state`, wrong order, wrong horizon, and wrong `use_gym_history` |
| `tests/observation_builder_tests.cpp` | Current foot support one-hot is left 3 dims + right 3 dims at current frame |
| `tests/observation_builder_tests.cpp` | Future rows append foot support at offset 35 and clamp beyond final frame |
| `tests/policy_math_tests.cpp` | `obs_current` foot support starts at offset 121 |
| `tests/policy_math_tests.cpp` | `obs_history` for footstate3 is direct 5x41 future payload, not temporal flattening |
| `tests/policy_io_contract_tests.cpp` | Accept `[1,127]`, `[1,5,41]`, `[1,26]` and reject old CLN shapes for footstate profile |
| `tests/policy_step_runner_tests.cpp` | Policy receives correct footstate3 input sizes and row ordering from a synthetic `.trk` |
| `tests/onnx_policy_runtime_tests.cpp` | App-owned `multi_policy_footstate3.onnx` + new YAML constructs runtime and returns 26 finite actions |
| `tests/app_config_tests.cpp` | Defaults, sim example, and release template point to app-owned footstate3 assets |
| `tests/app_runtime_factory_tests.cpp` | Profile/deploy mismatches fail early; ET1 source-tree paths remain rejected |
| Packaging selftest | New sha256 checks pass and release template uses footstate3 defaults |

Suggested focused verification command after implementation:

```bash
cmake --build deploy/robots/agentic-et1-tracker/build --parallel
ctest --test-dir deploy/robots/agentic-et1-tracker/build \
  -R 'agentic_et1_tracker_(policy_tests|policy_step_runner_tests|policy_onnx_tests|app_tests)' \
  --output-on-failure
ctest --test-dir deploy/robots/agentic-et1-tracker/build --output-on-failure
```

## Simulation and Robot Validation

### MuJoCo Gate

1. Start the simulator and tracker with `config.sim.yaml.example`.
2. Confirm `/health` and `/status` show model ready.
3. Run a known short `.trk` from `/home/galbot/works/et1/generated/`.
4. Verify no startup `MODEL_NOT_READY`, no NaN action, no shape mismatch.
5. Visually verify motion is stable through:
   - `fixstand`
   - `standby_velocity`
   - one user `.trk`
   - idle/standby fallback
   - interrupt with another `.trk`

### Real Robot Gate

Do not mark real robot validation complete until an operator explicitly confirms:

1. Service starts on robot and reaches ready state.
2. `fixstand -> standby_velocity` behaves as before.
3. A short low-risk `.trk` runs without orientation safety fault.
4. `/standby_velocity`, `/stop` for explicit urgent stop, and `/passive` password
   gate still behave as documented.
5. Status endpoint remains responsive under frequent polling.

## Risks

| Risk | Mitigation |
|---|---|
| Accidentally accepting old CLN deploy with new model | Explicit `GeneralTrackerCLNFootstate` profile and ONNX shape validation |
| Foot support one-hot order mismatch | Golden tests for left/right contact states and future-row offset 35 |
| Runtime config points into ET1 app assets | Keep existing path guard tests and add footstate defaults coverage |
| Existing robot installed config still points old CLN | Keep old CLN files and contract support during transition release |
| `torque_limit` in YAML is ignored | Document as non-goal; open separate safety task only if validation shows need |
| Scope creep into state machine or HTTP API | This plan explicitly excludes those changes |

## Rollback

Preferred rollback:

- Deploy the previous release package that contains old
  `multi_policy_v17c2_70k.onnx`, old `params/deploy.yaml`, and old 121 / 25x35
  CLN default.

Compatibility rollback inside a transition release:

- Use `policy.profile: "GeneralTrackerCLN"`.
- Use `policy_file: "multi_policy_v17c2_70k.onnx"`.
- Use `deploy: ".../params/deploy.yaml"`.

Do not rollback by pointing runtime config directly at
`deploy/robots/et1/config/policy/...`.

## Handoff Acceptance Criteria

- Footstate3 is the default in repo config, sim config, and release template.
- Old CLN remains available as an explicit compatibility profile.
- All focused policy/app tests pass.
- Full `ctest` passes.
- Release selftest validates footstate3 assets.
- MuJoCo validation is recorded.
- Real robot validation status is recorded honestly; if not run, docs say not run.
