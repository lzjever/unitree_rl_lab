# State Machine

Hot path:

```bash
scripts/et1-trk2motion ready
scripts/et1-trk2motion run /abs/file.trk --wait
scripts/et1-trk2motion run /abs/file.trk --hold --wait
```

Control states:

- `passive`: only leave this state when the user explicitly calls `fixstand`,
  then explicitly calls `standby`.
- `fixstand`: only enter StandbyVelocity when the user explicitly calls
  `standby`.
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

`ready` is a read-only precheck with a finite loop. It never posts `/fixstand`
or `/standby_velocity`. If status is `passive`, it returns `ok:false` with
`next:"fixstand"`. If status is `fixstand`, it returns `ok:false` with
`next:"standby_velocity"`. Follow `next` only when the user has explicitly
requested that control command; otherwise report the server state. Ask for
operator intervention when `next=manual`.

`run` and `repeat` post directly to `/execute` and do not recover control
state. If the tracker rejects execution from `passive` or `fixstand`, return
the compact server error and `next` without calling `fixstand` or `standby`.

`passive` is also explicit. The CLI sends `{"password":"..."}` using default
`galaxy`, `ET1_PASSIVE_PASSWORD`, or `passive --password VALUE`; do not print
the password.

`idle set PATH...` only configures an idle pool. `idle clear` clears it.
Idle never creates a user id and never appears in `status?id=ID`.

`run PATH --hold` only changes the `/execute` body by adding `hold:true`.
Omitting `--hold` sends no `hold` field.

`run --wait` and `wait` return success on `done` and on `holding`. Terminal
states other than `done` remain failures by default. Use
`wait ID --allow-terminal` only when non-`done` terminal states are expected and
acceptable.

`stop` immediately aborts active user, idle, holding, or transition work.
`standby_ref.trk` is recorded, simulator accepted, and runtime-gated
internally; real-robot GA gates remain pending. Direct
`standby`/`standby_velocity` remains available.
