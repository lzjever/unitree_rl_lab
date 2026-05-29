# GeneralTracker Frozen Profile

This directory is the default app-local policy profile path used by
`agentic-et1-tracker/config.yaml`.

It contains the app-owned frozen GeneralTracker profile required by
`agentic-et1-tracker` release packages:

- `exported/self_collision_footmesh_15k.onnx`
- `params/deploy.yaml`

These files are release assets owned by `agentic-et1-tracker`. Runtime
configuration must point at this app-local profile and must not point to or read
the ET1 app policy tree under `deploy/robots/et1/config/policy/...`.

Source checkouts that do not include these required files are not ready for
policy execution. In that state the HTTP service may still start, but `/status`
must report `ready:false` with `block:"policy_not_loaded"`, and `/execute` must
reject work.

`ASSET_MANIFEST.yaml` records the accepted app-local release assets for
release/package audit. `ASSET_MANIFEST.example.yaml` is only a template. Neither
manifest is a runtime fallback, downloader config, registry, or permission to
read assets from the ET1 app policy tree.
