# Passive Posture

This directory contains the app-owned Passive damping snapshot for
`agentic-et1-tracker`.

`passive.yaml` copies the numeric `mode` and `kd` values from the ET1
`FSM.Passive` configuration at release time. Runtime configuration must read
this local file and must not fall back to `deploy/robots/et1/config/config.yaml`.
