# Output Contract

Every command writes one compact JSON object to stdout so bash-only agents can parse command results.

Default success fields are short: `ok`, `cmd`, `intent`, `seq_id`, `state`, `ctrl`, `active`, `idle`, `segments`, `accepted`, `confirmed`, `urgent`, `hold`, `motion_mode`, `executor`, and `next`.

Default failure output has:

```json
{"ok":false,"cmd":"sequence-status","error":{"code":"WORKER_DIED","message":"sequence worker process is not running"},"next":"sequence-status"}
```

`next` is always one command: `run-text`, `run-trk`, `sequence-start`, `sequence-status`, `sequence-append`, `sequence-replace-tail`, `sequence-cancel`, `sequence-interrupt`, `standby`, `fixstand`, `passive`, `motion-mode`, `status`, `urgent-stop`, `idle-load`, or `cache-clear`.

Do not rely on default output for full paths, prompts, durations, tracker status, or long logs.
For `standby`, `state` may be `idle` when loaded idle motions immediately take over; `idle.enabled=true` means the idle configuration is still retained.
For `fixstand` and `passive`, successful output keeps compatibility `state:"fixstand"` or `state:"passive"` and adds `accepted:true, confirmed:false`; it is an accepted control request, not a confirmed `/status` state.
For `urgent-stop --urgent`, successful output keeps compatibility `state:"stopped"`, adds `urgent:true`, and maps to `/stop`.
