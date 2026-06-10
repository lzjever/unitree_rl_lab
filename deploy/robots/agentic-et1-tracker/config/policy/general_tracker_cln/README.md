# GeneralTrackerCLN Footstate Profile

This directory is the default app-local policy profile path used by
`agentic-et1-tracker/config.yaml`.

It contains the app-owned GeneralTrackerCLNFootstate release assets:

- `exported/multi_policy_footstate3.onnx`
- `params/deploy_fut_multi_footstate.yaml`

The fixed policy contract is:

- `obs_current`: `[1, 127]`
- `obs_history`: `[1, 5, 41]`
- `actions`: `[1, 26]`

The old GeneralTrackerCLN compatibility assets remain packaged for explicit
rollback configs:

- `exported/multi_policy_v17c2_70k.onnx`
- `params/deploy.yaml`

Runtime configuration must point at this app-local profile and must not point to
or read the ET1 app policy tree under `deploy/robots/et1/config/policy/...`.
The public motion input contract remains the existing `.trk` path; the service
does not accept `.et1trk` as a public input format.

`ASSET_MANIFEST.yaml` records the accepted app-local release assets for
release/package audit. It is not a runtime fallback, downloader config, or
registry.
