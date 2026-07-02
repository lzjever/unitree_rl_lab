# Sequence Workflow

`sequence-start` creates local JSON state and returns a `seq_id` immediately. A background worker prepares generated segments sequentially and keeps a small queue-ahead window submitted to the tracker while the current segment is playing.

P0 rules:

- First ready segment runs as soon as it is available.
- `sequence-start` cancels any prior active local sequence before starting the new one.
- Future segments are generated in order for serial continuity, then submitted up to `ET1_ACTION_QUEUE_AHEAD` unfinished tracker runs. Default queue-ahead is `3`.
- Queue-ahead reduces playback gaps. It also means `sequence-replace-tail` can only replace segments not yet submitted to the tracker.
- `sequence-append` adds unsubmitted local segments.
- `sequence-replace-tail` replaces only unsubmitted local segments.
- `sequence-cancel` cancels unsubmitted local work and calls standby.
- `sequence-interrupt --trk PATH` runs the ready `.trk` with mode `interrupt`.
- `sequence-interrupt --text TEXT` first calls standby, then generates and submits the new action with mode `interrupt`.
- Direct `run-text`, `run-trk`, and `standby` cancel active local sequences before calling the tracker, so old workers do not keep generating or submitting stale motions.
- Direct `run-text` and `run-trk` default to tracker interrupt as a skill product default. Raw HTTP `/execute` and `/execute_loco_upper` default to queue when `mode` is omitted.

Text segments accept:

- `duration`: physical duration seconds.
- `seed`: reproducible generation seed.
- `diffusion_steps`: advanced generation quality/speed knob.

Example:

```bash
scripts/et1-action sequence-start --plan-json '{"segments":[{"text":"A person walks forward cautiously.","duration":4,"seed":11},{"text":"A person turns right and keeps walking.","duration":4,"seed":12}]}'
```

`sequence-status` reports stale or dead workers using heartbeat and pid when available. It does not follow lower-level recovery suggestions automatically.

Agents should not keep the user turn open with `sleep && sequence-status`. Start the sequence, give a short acknowledgement, and only poll when the user asks for progress.
