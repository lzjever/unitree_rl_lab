# Output Contract

Every command writes one compact JSON object to stdout so bash-only agents can parse command results.

Default success fields are short: `ok`, `cmd`, `intent`, `seq_id`, `state`, `ctrl`, `active`, `idle`, `segments`, `accepted`, `confirmed`, `urgent`, `hold`, `motion_mode`, `executor`, and `next`.

Default failure output has:

```json
{"ok":false,"cmd":"sequence-status","error":{"code":"WORKER_DIED","message":"sequence worker process is not running"},"next":"sequence-status"}
```

`next` is always one command: `run-text`, `run-trk`, `sequence-start`, `sequence-status`, `sequence-append`, `sequence-replace-tail`, `sequence-cancel`, `sequence-interrupt`, `standby`, `fixstand`, `passive`, `motion-mode`, `status`, `urgent-stop`, `idle-load`, `idle-clear`, or `cache-clear`.

Do not rely on default output for full paths, prompts, durations, tracker status, or long logs.
For `standby`, `state` is `standby` for ordinary standby and may be `idle` when loaded idle motions immediately take over; `idle.enabled=true` means the idle configuration is still retained. Release tracker status uses `ctrl:"standby"` for ordinary standby.
For `idle-clear`, successful output uses `cmd:"idle-clear"`, `intent:"cancel_to_still"`, and the compact tracker `idle` fields after clearing idle config and entering standby.
For `fixstand` and `passive`, successful output keeps compatibility `state:"fixstand"` or `state:"passive"` and adds `accepted:true, confirmed:false`; it is an accepted control request, not a confirmed `/status` state.
For `urgent-stop --urgent`, successful output uses `state:"urgent_stop"`, adds `urgent:true`, and maps to tracker `POST /urgent_stop`.
