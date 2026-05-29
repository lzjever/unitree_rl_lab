# StandbyVelocity Policy

This directory contains the app-owned StandbyVelocity release policy for
`agentic-et1-tracker`.

Runtime contract:
- Input: `obs`, dimension 225
- Output: `actions`, dimension 12
- Control rate: 50 Hz

Required files:
- `exported/policy.onnx`
- `params/deploy.yaml`

These files are release assets owned by `agentic-et1-tracker`. Runtime
configuration must point at this app-local policy and must not fall back to or
read the ET1 app policy tree under `deploy/robots/et1/config/policy/...`.
