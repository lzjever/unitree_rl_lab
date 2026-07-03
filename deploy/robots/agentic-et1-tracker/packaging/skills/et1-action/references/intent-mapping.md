# Intent Mapping

Map user interruptions to one workflow command:

- Switch to base/chassis/loco-upper mode: `motion-mode base`, then later motion commands use `/execute_loco_upper`.
- Switch back to normal whole-body tracking: `motion-mode fullbody`.
- Ordinary walking, turning, or motion prompts do not imply base mode; use the current `motion-mode`.
- If base/chassis/loco-upper mode returns `MODEL_NOT_READY`, report the error; do not retry through full-body `/execute`.
- Add after current work: `sequence-append`.
- Replace later work while current motion continues: `sequence-replace-tail`.
- New user motion request while anything is running: use `run-text` or a new `sequence-start`; these cancel old local sequences and submit the first motion with tracker interrupt. Direct `run-text` and `run-trk` also default to interrupt; use `--mode queue` only for explicit append-after-current tracker work. Raw HTTP `/execute` and `/execute_loco_upper` still default to queue when `mode` is omitted.
- Immediately change a known active sequence to a ready `.trk`: `sequence-interrupt --trk`.
- Immediately change a known active sequence to new text: `sequence-interrupt --text`; it cancels old tail work, generates, then submits with tracker interrupt.
- Ordinary stop, pause, resume, relax, "停下", "停止动作", "恢复", or "放松": `sequence-cancel` or `standby`; `standby` maps to tracker `POST /standby` and preserves idle config.
- "不要动", "站着别动", "do not move", "completely still", "no idle", or "别播放 idle": `idle-clear`; `idle-clear` maps to tracker `POST /idle {"paths":[]}` then `POST /standby` and does not use urgent stop.
- Emergency stop, abort, kill, explicit urgent/quick stop wording, "紧急停止", or "赶快停止": `urgent-stop --urgent`; it maps to tracker `POST /urgent_stop` and clears idle config.

Do not map hesitation, ordinary cancellation, or status checks to urgent stop.
