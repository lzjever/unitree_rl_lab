# Raw HTTP Fallback

Prefer the CLI escape hatch so the agent still gets compact one-line JSON:

```bash
scripts/et1-trk2motion raw GET /status
scripts/et1-trk2motion raw POST /execute '{"path":"/abs/file.trk","mode":"interrupt"}'
scripts/et1-trk2motion raw POST /idle '{"paths":["/abs/idle.trk"]}'
scripts/et1-trk2motion raw POST /idle '{"paths":[]}'
scripts/et1-trk2motion raw POST /passive
scripts/et1-trk2motion raw POST /stop
```

Tracker endpoints used by the CLI:

- `GET /health`
- `GET /status`
- `GET /status?id=RUN_ID`
- `POST /passive`
- `POST /fixstand`
- `POST /standby_velocity`
- `POST /stop`
- `POST /execute` with only `{"path":"/abs/file.trk","mode":"interrupt|queue"}`
- `POST /idle` with only `{"paths":["/abs/a.trk"]}` or `{"paths":[]}`

`/execute` rejects extra fields and any `paths` field with `REQUEST_INVALID`.
`/idle` configures/clears the idle pool; it does not create a user run id.
In `/status`, `active.kind` is authoritative. `exec` and `queue` are user runs
only; idle progress lives under `idle`.

Only use direct HTTP knowledge when high-level commands cannot express the
operation. Do not emit `curl` unless a human explicitly asks for it.
