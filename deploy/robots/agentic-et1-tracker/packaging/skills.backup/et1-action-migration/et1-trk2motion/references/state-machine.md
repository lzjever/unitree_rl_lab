# State Machine

Hot path:

```bash
scripts/et1-trk2motion ready
scripts/et1-trk2motion run /abs/file.trk --wait
scripts/et1-trk2motion run /abs/file.trk --hold --wait
scripts/et1-trk2motion standby
```

Control states:

- `passive`: only leave this state when the user explicitly calls `fixstand`,
  then explicitly calls `standby`.
- `fixstand`: fixed configuration / preparation state, not normal stand-still
  or velocity control. Only enter StandbyVelocity when the user explicitly
  calls `standby`.
- `standby_velocity`: ready for `/execute` and idle auto-play.
- `running` with `active.kind=="idle"`: idle is active; user `run` preempts it.
- `running` with `active.kind=="user"`: wait by user id.
- `running` with `active.kind=="user"` and `state=="holding"`: the user run
  reached its last frame with `hold:true`; `wait` returns success and the id
  remains queryable until another user run or a control command releases it.
- `running` with `active.kind=="transition"`: internal synthetic transition to
  `transition.target`. Validated current local targets are `user`, `idle`, and
  the internal `standby_ref.trk` gate. Standby has no run id, does not enter
  queue/history, and does not consume `queue.limit`; real-robot GA gates remain
  pending.
- `stopping`: poll `state`; `stop` remains idempotent.
- `fault` or `block:"lowcmd_occupied"`: manual/operator state.

Status interpretation:

- Do not decide from `ctrl:"running"` alone. Read `active.kind`, compact
  `exec`, compact `queue`, and `transition.target/target_id`.
- `active.kind=="user"` plus `active.id` is the active waitable user run.
- `active.kind=="idle"` and `transition.target` of `idle` or `standby` are
  background; user `run`/`exec`/`repeat` can preempt them.
- `transition.target=="user"` means the transition belongs to that foreground
  user target; later queued user work waits unless the user explicitly requests
  `--mode interrupt`.

`ready` is a read-only precheck with a finite loop. It never posts `/fixstand`
or `/standby_velocity`. If status is `passive`, it returns `ok:false` with
`next:"fixstand"`. If status is `fixstand`, it returns `ok:false` with
`next:"standby_velocity"`. Follow `next` only when the user has explicitly
requested that control command; otherwise report the server state. Ask for
operator intervention when `next=manual`.

`run`, `exec`, and `repeat` post directly to `/execute` and do not recover
control state. Their default mode is `queue`; only use `--mode interrupt` when
the user explicitly asks to interrupt the current user action. If the tracker
rejects execution from `passive` or `fixstand`, return the compact server error
and `next` without calling `fixstand` or `standby`.

`passive` is also explicit. The CLI sends `{"password":"..."}` only when
`ET1_PASSIVE_PASSWORD` or `passive --password VALUE` is provided; otherwise it
refuses locally before HTTP. Do not print or save the password. After
`passive`, do not automatically restore: the user must explicitly request
`fixstand`, and later explicitly request `standby`.

`idle set PATH...` only configures an idle pool. `idle clear` clears it.
Idle never creates a user id and never appears in `status?id=ID`.
Preset idle loading ("加载idle动作", "放松点", "别直挺挺站着" when context means
idle behavior) is only idle pool setup: copy cwd `preset-trk/idle/*.trk` into a
tracker-readable allowed/generated motion dir, then `idle set` the copied
paths. It is not stop, `fixstand`, or clear-idle.

Control command differences:

- `standby` / `/standby_velocity`: normal stand-still command; no user run,
  no queue entry, preserves idle config, and is the right choice for ordinary
  do-not-move, stop/relax/recover requests. For pure standby with no later
  idle, the user must explicitly ask for `idle clear` then `standby`, or an
  explicit clear-idle flow; do not infer idle clearing.
- `fixstand`: explicit fixed-configuration recovery/control step, especially
  after `passive`; use it only when the user asks for `fixstand`, to enter the
  stand configuration/进入站立构型, or an explicit control-state
  recovery/preparation step. Do not follow it with `standby` until the user
  separately asks.
- `passive`: explicit passworded safety sink that clears queued work and idle
  pool; never auto-restore from it.
- `stop --urgent` / `/stop`: urgent abort path only; clears/aborts active user,
  idle, holding, or transition work and does not play standby.

`run PATH --hold` only changes the `/execute` body by adding `hold:true`.
Omitting `--hold` sends no `hold` field.

`run --wait` and `wait` return success on `done` and on `holding`. Terminal
states other than `done` remain failures by default. Use
`wait ID --allow-terminal` only when non-`done` terminal states are expected and
acceptable.

`stop` requires CLI `--urgent` and immediately aborts active user, idle,
holding, or transition work. It clears idle/background work and does not play
`standby_ref.trk`. Use it only when the user directly and urgently says
stop/abort/kill/紧急停止/赶快停止. Ordinary
"停下/停止/放松/恢复/静止站立/不要动/站着别动" requests must use direct
`standby`/`standby_velocity` and preserve idle config.
