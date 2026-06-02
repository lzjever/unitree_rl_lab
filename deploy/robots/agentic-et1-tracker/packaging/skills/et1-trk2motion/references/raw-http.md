# Raw HTTP Fallback

Prefer the CLI escape hatch so the agent still gets compact one-line JSON:

```bash
scripts/et1-trk2motion raw GET /status
scripts/et1-trk2motion raw POST /execute '{"path":"/abs/file.trk","mode":"interrupt"}'
scripts/et1-trk2motion raw POST /stop
```

Tracker endpoints used by the CLI:

- `GET /health`
- `GET /status`
- `GET /status?id=RUN_ID`
- `POST /fixstand`
- `POST /standby_velocity`
- `POST /stop`
- `POST /execute` with `{"path":"/abs/file.trk","mode":"interrupt|queue"}`

Only use direct HTTP knowledge when high-level commands cannot express the
operation. Do not emit `curl` unless a human explicitly asks for it.
