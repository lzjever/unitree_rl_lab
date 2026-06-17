# ET1 Upper Body Limits

This directory contains app-owned upper-body joint limits and the frozen
logical-to-SDK joint map for `agentic-et1-tracker` loco-upper composition.

Required files:
- `limits.yaml`
- `joint_map.yaml`

`limits.yaml` must define 14-entry upper-joint vectors for:
- `upper_min_q`
- `upper_max_q`
- `max_vel_radps`
- `max_accel_radps2`

These files are release assets owned by `agentic-et1-tracker`. Runtime config
must read this app-local asset set and must not fall back to ET1 runtime assets.
