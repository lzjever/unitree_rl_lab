# FixStand Posture

This directory contains the app-owned FixStand posture snapshot for
`agentic-et1-tracker`.

`fixstand.yaml` copies the numeric `kp`, `kd`, `ts`, and `qs` values from the
ET1 `FSM.FixStand` configuration at release time. Runtime configuration must
read this local file and must not fall back to
`deploy/robots/et1/config/config.yaml`.
