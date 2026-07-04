# Output Contract

Every command writes one compact JSON object to stdout so bash-only agents can parse command results.

Common success fields are short: `ok`, `cmd`, `intent`, `seq_id`, `matched`, `state`, `ctrl`, `active`, `idle`, `segments`, `accepted`, `confirmed`, `urgent`, `hold`, `motion_mode`, `executor`, and `next`. These are common fields, not an exhaustive schema; individual commands may add compact command-specific fields.

Default failure output has:

```json
{"ok":false,"cmd":"sequence-status","error":{"code":"WORKER_DIED","message":"sequence worker process is not running"},"next":"sequence-status"}
```

`sequence-replace-tail` failures that touch already submitted tracker runs use `error.code:"TAIL_ALREADY_SUBMITTED"` and include `replaceable_count`, `submitted_count`, and `next:"sequence-interrupt"`.

CLI/skill `next` is always one command: `run-text`, `run-trk`, `sequence-start`, `sequence-status`, `sequence-append`, `sequence-replace-tail`, `sequence-cancel`, `sequence-interrupt`, `standby`, `fixstand`, `passive`, `motion-mode`, `status`, `urgent-stop`, `idle-load`, `idle-clear`, or `cache-clear`. Tracker HTTP response `next` is not CLI argv; for example HTTP `next:"urgent_stop"` is mapped by the CLI/skill layer to `urgent-stop`.

Do not rely on default output for full paths, prompts, durations, tracker status, or long logs.
For `run-text` and `run-trk`, direct non-`--wait` success means accepted/submitted only. The output keeps the tracker returned `state`, commonly `queued`, sets `accepted:true` and `confirmed:false`, and does not imply the run is already running or complete. If the tracker omits `state`, the CLI reports `state:"submitted"` rather than inventing `running`.
For `run-text --wait` and `run-trk --wait`, success means the submitted run reached `state:"done"` or `state:"holding"` and the output sets `accepted:true` and `confirmed:true`. Failed wait states such as `failed`, `canceled`, or `stopped` return `ok:false` with `error.code:"TRACKER_RUN_FAILED"`.
For `standby`, `state` is `standby` for ordinary standby and may be `idle` when loaded idle motions immediately take over; that active-idle takeover is confirmable only when public status has `ctrl:"running"`, `active.kind:"idle"`, and `idle.active:true`. `idle.enabled=true` means the idle configuration is still retained. Release tracker status uses `ctrl:"standby"` for ordinary standby. If the tracker does not reach confirmable ordinary standby or active-idle takeover before the command timeout, output may fail with `error.code:"STANDBY_NOT_CONFIRMED"`.
For `idle-load`, successful output means the idle pool was configured successfully. It does not imply idle is currently playing, has started, or has completed; use `status` and the compact `idle` fields to inspect playback.
For `idle-clear`, the CLI first sends tracker `POST /idle {"paths":[]}` and then requests ordinary `POST /standby`. Successful output uses `cmd:"idle-clear"`, `intent:"cancel_to_still"`, and the compact tracker `idle` fields after confirming ordinary standby and cleared idle config. Active idle is not an `idle-clear` success state. HTTP idle clear is any-state, but CLI `idle-clear` is not an any-state success guarantee: if the later standby request cannot be confirmed from passive, fault, fixstand, or another blocked state, the command may fail after the clear request was issued.
For `fixstand` and `passive`, successful output keeps compatibility `state:"fixstand"` or `state:"passive"` and adds `accepted:true, confirmed:false`; it is an accepted control request, not a confirmed `/status` state.
For `urgent-stop --urgent`, successful output uses `state:"urgent_stop"`, adds `urgent:true`, and maps to tracker `POST /urgent_stop`.
