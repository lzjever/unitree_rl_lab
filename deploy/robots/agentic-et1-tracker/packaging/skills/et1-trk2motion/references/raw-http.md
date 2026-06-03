# Raw HTTP Fallback

Prefer the CLI escape hatch so the agent still gets compact one-line JSON:

```bash
scripts/et1-trk2motion raw GET /status
scripts/et1-trk2motion raw POST /execute '{"path":"/abs/file.trk","mode":"interrupt"}'
scripts/et1-trk2motion raw POST /execute '{"path":"/abs/file.trk","mode":"interrupt","hold":true}'
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
- `POST /execute` with `path`, optional `mode:"interrupt|queue"`, and optional
  `hold:true`; omit `hold` instead of sending `hold:false`.
- `POST /idle` with only `{"paths":["/abs/a.trk"]}` or `{"paths":[]}`

`/execute` allows `path`, optional `mode`, and optional boolean `hold` only.
Omit `hold` for normal completion; send `hold:true` to keep the user run in
`state:"holding"` at the last reference frame. It rejects extra fields,
non-boolean `hold`, and any `paths` field with `REQUEST_INVALID`.
`/idle` configures/clears the idle pool; it does not create a user run id.
In `/status`, `active.kind` is authoritative and is
`none|user|idle|transition`. `exec` and `queue` are user runs only; idle
progress lives under `idle`. Internal synthetic transitions use
`active.kind:"transition"` plus the compact `transition` object
(`active`, `target`, `target_id`, `target_state`, `frame`, `frames`,
`progress`); `target_state` is `null` or a motion state string. They do not
enter queue/history and have no run id. `/stop` immediately aborts active user,
idle, holding, or transition work and does not play `standby_ref.trk`.

`standby_ref.trk` is gated until an app-local validated asset plus manifest is
present. Direct `/standby_velocity` remains valid even when that asset is
absent.

Only use direct HTTP knowledge when high-level commands cannot express the
operation. Do not emit `curl` unless a human explicitly asks for it.
