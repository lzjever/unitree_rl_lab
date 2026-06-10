# Raw HTTP Fallback

Prefer the CLI escape hatch so the agent still gets compact one-line JSON:

```bash
scripts/et1-trk2motion raw GET /status
scripts/et1-trk2motion raw POST /execute '{"path":"/abs/file.trk","mode":"queue"}'
scripts/et1-trk2motion raw POST /execute '{"path":"/abs/file.trk","mode":"queue","hold":true}'
scripts/et1-trk2motion raw POST /execute '{"path":"/abs/file.trk","mode":"interrupt"}'
scripts/et1-trk2motion raw POST /idle '{"paths":["/abs/idle.trk"]}'
scripts/et1-trk2motion raw POST /idle '{"paths":[]}'
scripts/et1-trk2motion raw POST /passive '{"password":"<user-provided-password>"}'
```

Tracker endpoints used by the CLI:

- `GET /health`
- `GET /status`
- `GET /status?id=RUN_ID`
- `POST /passive` with `{"password":"..."}`
- `POST /fixstand` for explicit fixed-configuration control steps only
- `POST /standby_velocity` for normal standby; preserves idle config
- `POST /execute` with `path`, optional `mode:"interrupt|queue"`, and optional
  `hold:true`; omit `hold` instead of sending `hold:false`.
- `POST /idle` with only `{"paths":["/abs/a.trk"]}` or `{"paths":[]}`
- Emergency only: `POST /stop`

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

The high-level `passive` command sends the passive password body for you and
requires `ET1_PASSIVE_PASSWORD` or `passive --password VALUE`; do not print or
save the password. `ready` is read-only and never posts `/fixstand` or
`/standby_velocity`. `run`, `exec`, and `repeat` default to `mode:"queue"` and
post directly to `/execute`; if the server returns a control-state error and
`next`, return that compact error without automatic recovery.

Do not use `/stop` as a normal "stand still" command. Use high-level
`standby`/`/standby_velocity` for normal do-not-move, stop/relax/recover,
静止站立, 不要动, or 站着别动 requests; `/standby_velocity` preserves idle
config. For pure standby with no later idle, the user must explicitly ask for
`idle clear` then `standby`, or an explicit clear-idle flow; do not infer idle
clearing. `/fixstand` is not normal stand-still control; use raw
`POST /fixstand` only when the user explicitly asks to enter the stand
configuration/进入站立构型, or for an explicit control-state
recovery/preparation step. Only call high-level `stop --urgent`, or raw
`POST /stop` as a last resort, when the user directly and urgently says
stop/abort/kill/紧急停止/赶快停止.

`standby_ref.trk` is recorded, simulator accepted, and runtime-gated
internally; real-robot GA gates remain pending. Direct `/standby_velocity`
remains valid.

Only use direct HTTP knowledge when high-level commands cannot express the
operation. Do not emit `curl` unless a human explicitly asks for it.
