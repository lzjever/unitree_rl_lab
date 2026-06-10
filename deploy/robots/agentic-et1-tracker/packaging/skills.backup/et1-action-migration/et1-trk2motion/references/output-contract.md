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
{"ok":true,"ctrl":"running","ready":true,"err":null,"active":{"kind":"transition","id":null},"transition":{"active":true,"target":"user","target_id":"run-2","frame":8,"frames":25,"progress":0.32},"idle":{"enabled":true,"n":2,"active":false,"current":null,"frame":0,"frames":0,"progress":0},"exec":{"id":null,"state":null,"frame":0,"frames":0,"progress":0},"queue":{"ids":["run-3"],"n":1,"limit":8}}
```

Failure shape:

```json
{"ok":false,"error":{"code":"run_failed","message":"state=failed"},"next":"status","ctrl":"standby_velocity","ready":true,"err":null}
```

Control-state execution rejection shape:

```json
{"ok":false,"error":{"code":"CONTROL_STATE_CONFLICT","message":"passive cannot execute"},"next":"fixstand","ctrl":"passive","ready":false,"err":null}
```

Missing passive password shape:

```json
{"ok":false,"error":{"code":"missing_passive_password","message":"passive requires --password or ET1_PASSIVE_PASSWORD"},"next":"manual"}
```

Stop without urgent guard shape:

```json
{"ok":false,"error":{"code":"stop_requires_urgent","message":"stop requires --urgent"},"next":"standby_velocity"}
```

Use exit code `0` only when `ok:true`; `holding` is `ok:true` for `wait` and
`run --hold --wait`. Nonzero means inspect `error.code` and `next`.
`state`/`status` default to short output and omit large pose arrays. Use
`status --full` only when the user needs raw tracker details.
`ready` is read-only and can return `next:"fixstand"` or
`next:"standby_velocity"` without performing those commands. `run`, `exec`,
and `repeat` default to `mode:"queue"` and return compact `/execute` errors
and `next` without automatic control recovery. `passive` refuses before HTTP
unless `--password` or `ET1_PASSIVE_PASSWORD` is provided, and output never
includes the password. `stop` refuses before HTTP unless `--urgent` is present.

`active.kind` is `none|user|idle|transition`. Only `active.kind=="user"` with a
non-null `id` is waitable. Idle status is compact progress only and is not a
run id. Internal transitions are reported through compact
`transition.active/target/target_id/frame/frames/progress`; they have no id and
do not enter user queue or history. When `transition.target=="user"`, the target
user owns the foreground transition and later queued user work waits. When the
active work is idle/background or `transition.target` is `idle`/`standby`, user
`run` can preempt it. Compact `exec` and `queue` fields are included when the
tracker provides them so agents can judge user work without `--full`.
