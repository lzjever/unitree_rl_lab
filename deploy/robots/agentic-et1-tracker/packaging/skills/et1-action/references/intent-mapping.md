# Intent Mapping

Map user interruptions to one workflow command:

- Add after current work: `sequence-append`.
- Replace later work while current motion continues: `sequence-replace-tail`.
- New user motion request while anything is running: use `run-text` or a new `sequence-start`; these cancel old local sequences and submit the first motion with tracker interrupt.
- Immediately change a known active sequence to a ready `.trk`: `sequence-interrupt --trk`.
- Immediately change a known active sequence to new text: `sequence-interrupt --text`; it cancels old tail work, generates, then submits with tracker interrupt.
- Ordinary stop, pause, or "do not move": `sequence-cancel` or `standby`.
- Emergency stop, abort, kill, or explicit urgent wording: `urgent-stop --urgent`.

Do not map hesitation, ordinary cancellation, or status checks to urgent stop.
