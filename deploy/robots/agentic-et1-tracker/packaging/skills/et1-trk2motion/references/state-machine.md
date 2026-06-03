# State Machine

Hot path:

```bash
scripts/et1-trk2motion ready
scripts/et1-trk2motion run /abs/file.trk --wait
scripts/et1-trk2motion run /abs/file.trk --hold --wait
```

Control states:

- `passive`: call `fixstand`, then `standby`.
- `fixstand`: call `standby`.
- `standby_velocity`: ready for `/execute` and idle auto-play.
- `running` with `active.kind=="idle"`: idle is active; user `run` preempts it.
- `running` with `active.kind=="user"`: wait by user id.
- `running` with `active.kind=="user"` and `state=="holding"`: the user run
  reached its last frame with `hold:true`; `wait` returns success and the id
  remains queryable until another user run or a control command releases it.
- `running` with `active.kind=="transition"`: internal synthetic transition to
  `transition.target`. Validated current local targets are `user` and `idle`;
  `standby` remains reserved because `standby_ref.trk` runtime playback and
  real-robot GA gates are pending, even though the asset is recorded and
  simulator accepted. It has no run id, does not enter queue/history, and does
  not consume `queue.limit`.
- `stopping`: poll `state`; `stop` remains idempotent.
- `fault` or `block:"lowcmd_occupied"`: manual/operator state.

`ready` performs the transitions above with a finite loop. If it returns
`ok:false`, follow `next` or ask for operator intervention when `next=manual`.

`idle set PATH...` only configures an idle pool. `idle clear` clears it.
Idle never creates a user id and never appears in `status?id=ID`.

`run PATH --hold` only changes the `/execute` body by adding `hold:true`.
Omitting `--hold` sends no `hold` field.

`run --wait` and `wait` return success on `done` and on `holding`. Terminal
states other than `done` remain failures by default. Use
`wait ID --allow-terminal` only when non-`done` terminal states are expected and
acceptable.

`stop` immediately aborts active user, idle, holding, or transition work.
`standby_ref.trk` is recorded and simulator accepted, but runtime playback and
real-robot GA gates remain pending; direct `standby`/`standby_velocity` remains
available without playback of that asset.
