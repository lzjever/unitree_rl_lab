# Output Contract

The CLI prints exactly one compact JSON object per command, on one line.

Success shape:

```json
{"ok":true,"id":"run-1","state":"done","ctrl":"standby_velocity","ready":true,"err":null}
```

Held success shape:

```json
{"ok":true,"id":"run-1","state":"holding","frame":119,"frames":120,"progress":1,"ctrl":"running","ready":true,"err":null}
```

Short `state/status` shape:

```json
{"ok":true,"ctrl":"running","ready":true,"err":null,"active":{"kind":"transition","id":null},"transition":{"active":true,"target":"user","target_id":"run-2","frame":8,"frames":25,"progress":0.32},"idle":{"enabled":true,"n":2,"active":false,"current":null,"frame":0,"frames":0,"progress":0}}
```

Failure shape:

```json
{"ok":false,"error":{"code":"run_failed","message":"state=failed"},"next":"status","ctrl":"standby_velocity","ready":true,"err":null}
```

Use exit code `0` only when `ok:true`; `holding` is `ok:true` for `wait` and
`run --hold --wait`. Nonzero means inspect `error.code` and `next`.
`state`/`status` default to short output and omit large pose arrays. Use
`status --full` only when the user needs raw tracker details.

`active.kind` is `none|user|idle|transition`. Only `active.kind=="user"` with a
non-null `id` is waitable. Idle status is compact progress only and is not a
run id. Internal transitions are reported through compact
`transition.active/target/target_id/frame/frames/progress`; they have no id and
do not enter user queue or history. `standby_ref.trk` is recorded, simulator
accepted, and runtime-gated internally; real-robot GA gates remain pending.
