ET1 general tracking policies.

`GeneralTrackerCLN` is the primary GeneralTracker profile. Unprefixed tracker
requests and the Velocity joystick shortcut route to `GeneralTrackerCLN`.
`GeneralTrackerCJM` remains available as an explicit profile with the `cjm`
request prefix or keyboard state `3`.

The CJM deploy observation layout targets `Mjlab-TrackMulti-SelfCollision-Hist-ET1-8GPU`:

- `obs_current`: 131 dims
- `obs_history`: 25 x 105 dims
- root orientation observation uses `command_root_ori_b`, matching mjlab
  `motion_root_ori_b` (`inv(robot_root_quat) * ref_root_quat`).
- `ref_com_rel_navi` and `ref_com_vel_navi` are required.

Replace `exported/model_19999.onnx` with an ONNX exported from that task, and use
a motion cache containing foot support and reference COM arrays.

Available CJM strategies:

- `0603-self_collision_ofwr_30k.onnx` + `deploy.yaml`
- `self_collision_a3_style_reward_v3_30k.onnx` +
  `deploy_self_collision_a3_style_reward_v3_30k.yaml`
  - source task: `MjTM-H-ET1-SelfCollision-A3-StyleRewardV3-LegacyPd-8GPU`
  - source checkpoint:
    `/home/galbot/WorkSpace/mjlab/logs/rsl_rl/et1_tracking/2026-06-07_17-07-29_et1_track_self_collision_a3_style_reward_v3_30k/model_29999.onnx`
  - deployed as:
    `config/policy/general_tracker_cjm/exported/self_collision_a3_style_reward_v3_30k.onnx`
  - observation layout: `obs_current = actor + actor_z_style + actor_style_phase`
    (`179` dims), `obs_history = actor_history` (`25 x 105` dims)
  - motion cache: convert source NPZ with
    `scripts/et1/convert_style_track_npz.py` so `z_style_50` is preserved.
    Missing or all-zero style latent data is treated as an error.
- `self_collision_a3_style_reward_v3_sdkpd_220ankle_30k.onnx` +
  `deploy_self_collision_a3_style_reward_v3_sdkpd_220ankle_30k.yaml`
  - source task: `MjTM-H-ET1-SelfCollision-A3-StyleRewardV3-8GPU`
  - source checkpoint:
    `/home/galbot/WorkSpace/mjlab/logs/rsl_rl/et1_tracking/2026-06-14_11-41-58_et1_track_self_collision_a3_style_reward_v3_sdkpd_220ankle_30k/model_29999.onnx`
  - deployed as:
    `config/policy/general_tracker_cjm/exported/self_collision_a3_style_reward_v3_sdkpd_220ankle_30k.onnx`
  - observation layout: `obs_current = actor + actor_z_style + actor_style_phase`
    (`179` dims), `obs_history = actor_history` (`25 x 105` dims)
  - PD/action layout: source `sdkpd_220ankle` settings, including 220 ankle
    stiffness.
  - motion cache: convert source NPZ with
    `scripts/et1/convert_style_track_npz.py` so `z_style_50` is preserved.
    Missing or all-zero style latent data is treated as an error.

Runtime trigger:

```bash
mkdir -p debug
printf '%s\n' 'config/policy/general_tracker_cln/params/your_motion.et1trk' > debug/general_tracker_request.txt
```

The same request file can route to other Track profiles by prefixing the first
line:

```bash
printf '%s\n' 'cjm config/policy/general_tracker_cjm/params/walk-2_et1_kpts.et1trk' > debug/general_tracker_request.txt
printf '%s\n' 'cln config/policy/general_tracker_cln/params/walk-cln-1_et1.et1trk' > debug/general_tracker_request.txt
```

No prefix routes to `GeneralTrackerCLN`. Use the `cjm` prefix to route to
`GeneralTrackerCJM`.

When a request file appears, Velocity routes to the selected profile and the
tracker loads the requested motion.
