# GeneralTrackerDR3

App-owned DR3 GeneralTracker assets copied from the reference tree.

- Source repository: `/home/galbot/works/et1/.reference/unitree_ET1_lab_mjlab_hyx`
- Source HEAD: `e3c50ad6075ea8f146b3f1fca89bd44b272868a5`
- Model source: `robots/et1/config/policy/mimic/general_tracking/exported/DR3-all.onnx`
- Deploy source: `robots/et1/config/policy/mimic/general_tracking/params/deploy_fut_obs.yaml`

## Contract

- `obs_current`: float32 `[1, 118]`
- `obs_history`: float32 `[1, 5, 32]`
- `actions`: float32 `[1, 26]`
- `step_dt`: `0.02`

`obs_current` terms are fixed in this order:

1. `command_yaw` (2)
2. `command_root_ori_b` (6)
3. `command_jnt_pos` (26)
4. `projected_gravity` (3)
5. `base_ang_vel` (3)
6. `joint_pos_rel` (26)
7. `joint_vel_rel` (26)
8. `last_action` (26)

`obs_history` is the singular future term `future_command`, with five rows of
`root_ori_b(6) + joint_pos(26)`. It does not include xy/yaw velocity or foot
support state.

## Asset Hashes

- `exported/DR3-all.onnx`: size `3590360`, sha256 `fb48e575d81951e6cd23e65cde12258bd5feccfb8dfc3c9bce549d6c55269e12`
- `params/deploy_fut_obs.yaml`: size `3559`, sha256 `a92cd204343d513d0c74f14acfa2fcfbf38040f31c34baf293f3e5d465021bb2`
