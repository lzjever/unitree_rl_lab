# Loco Lower ET1 Low Policy

This directory contains the app-owned lower locomotion policy assets for
`agentic-et1-tracker`.

Runtime contract:
- Input: `obs`, dimension 45
- Output: `actions`, dimension 12
- `policy_decimation`: 10
- `step_dt`: 0.02

Required files:
- `exported/policy.onnx`
- `params/deploy_lowobs10k.yaml`

These files are copied into the tracker app tree for release ownership and
auditability. They are the app-owned lower locomotion runtime assets used by
`/execute_loco_upper` when `loco_upper.enabled: true`.

Scope:
- wired into the lower locomotion runtime behind `loco_upper`
- simulation-only handoff target today
- not a true-robot GA claim or broader safety envelope
