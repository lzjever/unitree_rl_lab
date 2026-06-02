# State Machine

Hot path:

```bash
scripts/et1-trk2motion ready
scripts/et1-trk2motion run /abs/file.trk --wait
```

Control states:

- `passive`: call `fixstand`, then `standby`.
- `fixstand`: call `standby`.
- `standby_velocity`: ready for `/execute`.

`ready` performs the transitions above with a finite loop. If it returns
`ok:false`, follow `next` or ask for operator intervention when `next=manual`.

`run --wait` and `wait` treat terminal states other than `done` as failures by
default. Use `wait ID --allow-terminal` only when non-`done` terminal states are
expected and acceptable.
