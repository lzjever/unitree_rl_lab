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

For manual or integration simulation testing in this workspace, the installed
Unitree MuJoCo simulator under `/home/galbot/works/et1` can be used. Test
`.trk` files are available under `/home/galbot/works/et1/generated/`.

These paths are environment notes for simulation acceptance only. They do not
change the HTTP input contract: `agentic-et1-tracker` accepts local `.trk` paths
only, not uploads or other motion formats.

## Manual MuJoCo Acceptance

This is a manual integration acceptance skeleton, not a complete GA simulation
evidence script and not evidence that acceptance has already been completed.
Complete GA simulation evidence still needs recorded queue FIFO, interrupt,
stop-to-idle, fault/disconnect, and performance results.

Prerequisites:

- Preflight: before acceptance, confirm there is no old `et1_ctrl`,
  `unitree_mujoco`, `agentic-et1-tracker`, or other LowCmd owner on the same
  DDS `network` and `domain_id`. Existing control processes on a different DDS
  domain must be recorded as isolation conditions, not treated as same-domain
  owners.
- Install app-owned frozen assets under this app directory or an external asset
  directory. Do not point the new app at the ET1 app policy tree.
- `policy.deploy` must live under `policy.policy_dir/params`, and `lock_path`
  must be absolute when explicitly configured.
- Use a simulation config with `mode_machine: 0`, `network: "lo"`, tracker
  `domain_id` matching the MuJoCo `domain_id`, `motion_dirs` including
  `/home/galbot/works/et1/generated`, and policy paths pointing to real
  app-owned ONNX/deploy assets.
- Start the Unitree MuJoCo simulator only after the Preflight is clear.

Command skeleton:

```sh
# terminal 1: start MuJoCo from the local simulator install
/home/galbot/works/et1/unitree_mujoco/simulate/build/unitree_mujoco

# terminal 2: start the new app with the simulation config
agentic-et1-tracker --config /path/to/agentic-et1-tracker-sim.yaml

# terminal 3: exercise the HTTP contract
TRK=$(find /home/galbot/works/et1/generated -maxdepth 1 -name '*.trk' | head -n 1)
curl http://127.0.0.1:8080/health
curl http://127.0.0.1:8080/status
curl -X POST http://127.0.0.1:8080/execute \
  -H 'Content-Type: application/json' \
  -d "{\"path\":\"$TRK\"}"

# poll status until frame progress is visible, then stop
curl http://127.0.0.1:8080/status
curl -X POST http://127.0.0.1:8080/stop
```
