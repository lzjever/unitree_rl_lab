# State Machine

Hot path:

```bash
scripts/et1-trk2motion ready
scripts/et1-trk2motion run /abs/file.trk --wait
```

Control states:

- `passive`: call `fixstand`, then `standby`.
- `fixstand`: call `standby`.
- `standby_velocity`: ready for `/execute` and idle auto-play.
- `running` with `active.kind=="idle"`: idle is active; user `run` preempts it.
- `running` with `active.kind=="user"`: wait by user id.
- `stopping`: poll `state`; `stop` remains idempotent.
- `fault` or `block:"lowcmd_occupied"`: manual/operator state.

`ready` performs the transitions above with a finite loop. If it returns
`ok:false`, follow `next` or ask for operator intervention when `next=manual`.

`idle set PATH...` only configures an idle pool. `idle clear` clears it.
Idle never creates a user id and never appears in `status?id=ID`.

`run --wait` and `wait` treat terminal states other than `done` as failures by
default. Use `wait ID --allow-terminal` only when non-`done` terminal states are
expected and acceptable.
