# Output Contract

The CLI prints exactly one compact JSON object per command, on one line.

Success shape:

```json
{"ok":true,"id":"run-1","state":"done","ctrl":"standby_velocity","ready":true,"err":null}
```

Short `state/status` shape:

```json
{"ok":true,"ctrl":"running","ready":true,"err":null,"active":{"kind":"idle","id":null},"idle":{"enabled":true,"n":2,"active":true,"current":0,"frame":12,"frames":120,"progress":0.1}}
```

Failure shape:

```json
{"ok":false,"error":{"code":"run_failed","message":"state=failed"},"next":"status","ctrl":"standby_velocity","ready":true,"err":null}
```

Use exit code `0` only when `ok:true`; nonzero means inspect `error.code` and
`next`. `state`/`status` default to short output and omit large pose arrays.
Use `status --full` only when the user needs raw tracker details.
Only `active.kind=="user"` with a non-null `id` is waitable. Idle status is
compact progress only and is not a run id.
