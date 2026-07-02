---
name: et1-action
description: Generate and run ET1 robot actions from text or .trk files, load idle motions, enter standby, perform explicit urgent stop, and manage local async action sequences.
---

# ET1 Action

Use this skill for ET1 robot motion. The agent uses the bash tool only; the entry point is:

```bash
scripts/et1-action ...
```

Do not depend on typed tools, plugins, curl, or multi-tool parallel calls.

Prefer natural-language commands through `run-text`; it checks bundled presets first, falls back to text-to-TRK generation, then runs the resulting `.trk`. Use `run-trk` only when the `.trk` path is already known. Direct `run-text`, `run-trk`, `sequence-start`, and `standby` cancel any active local sequence before continuing; new user intent must not leave old background generation running. `run-text` and `run-trk` submit with tracker `interrupt` mode by default because a new user instruction should replace the current foreground action; use `--mode queue` only when the user explicitly asks to add after current tracker work. This is a skill-level product default: raw HTTP `/execute` and `/execute_loco_upper` still default to `mode:"queue"` when `mode` is omitted.
For generated motions, translate user intent to a concise English Kimodo-style prompt and choose a short physical duration. Use `--seed` when the user asks to retry/compare variants or reproduce a result; use `--diffusion-steps` only when explicitly needed.
For precise left/right limb actions or final-pose requests, use `run-text ... --hold --no-preset`. The command also auto-detects common "hold the pose" phrasing and bypasses risky presets for precise left/right limb prompts.

Motion submission has two persistent skill modes. The default is `fullbody`, which submits TRK through the normal whole-body tracker (`/execute`). Only switch to `base` when the user explicitly asks to use base/chassis/loco-upper mode; then later `run-text`, `run-trk`, and sequence submissions use `/execute_loco_upper` until the user explicitly switches back to `fullbody`. Do not infer base mode from ordinary walking requests. If base/loco-upper execution returns `MODEL_NOT_READY`, report that compact error and do not fall back to `/execute`.

Common commands:

```bash
scripts/et1-action run-text "<concise English motion prompt>" --duration 3
scripts/et1-action run-text "<concise English motion prompt>" --duration 3 --mode queue  # explicit append after current tracker work
scripts/et1-action run-text "<concise English final-pose prompt>" --duration 4 --hold --no-preset
scripts/et1-action run-trk /abs/path/motion.trk
scripts/et1-action run-trk /abs/path/motion.trk --mode queue  # explicit append after current tracker work
scripts/et1-action run-trk relative/name-under-user-motion
scripts/et1-action motion-mode              # query current fullbody/base mode
scripts/et1-action motion-mode base         # explicit base/chassis/loco-upper mode
scripts/et1-action motion-mode fullbody     # explicit normal whole-body mode
scripts/et1-action status
scripts/et1-action standby
scripts/et1-action fixstand  # explicit "enter stand configuration" only
scripts/et1-action passive --password "$ET1_PASSIVE_PASSWORD"  # explicit authorized passive mode only
scripts/et1-action urgent-stop --urgent
scripts/et1-action idle-load
scripts/et1-action cache-clear
scripts/et1-action sequence-start --plan-json '{"segments":[{"text":"<segment prompt>","duration":3}]}'
scripts/et1-action sequence-status SEQ_ID
```

Normal stop requests map to `standby` or `sequence-cancel`; do not use `urgent-stop` unless the user clearly asks for emergency stop, abort, or kill. Use `fixstand` only when the user explicitly asks to enter the stand configuration. Use `passive --password ...` only when the user explicitly authorizes passive mode and provides the password; there is no default password lookup. Passive mode does not automatically recover, and the next normal recovery step is `fixstand`.
`standby` cancels active local sequences and tracker queued motions, but does not clear idle configuration. If idle motions are loaded, the tracker may enter idle background motion; if no idle is loaded, it stays in velocity0 standby. Trust the command JSON `state/ctrl/active/idle` fields instead of assuming pure velocity0.
`fixstand` and `passive` keep the compatibility `state:"fixstand"` or `state:"passive"`, but successful output also has `accepted:true` and `confirmed:false`; do not treat `state` alone as confirmation that `/status` has already reached the target state. `urgent-stop --urgent` keeps compatibility `state:"stopped"`, adds `urgent:true`, and is the only skill command that calls `/stop`.
Only clear generation cache when the user explicitly asks to clear/reset cache. Run `scripts/et1-action cache-clear`; add `--root PATH` only when clearing a non-default text-to-TRK root. Do not clear cache as a normal retry or motion-control step.

`run-trk` accepts absolute `.trk` file paths and relative user-motion names. Absolute paths must already exist, be files, and end in `.trk`; `.et1trk` is not supported. Relative paths are resolved under `Path.cwd()/generated/user-motion`, may include subdirectories, and may omit the `.trk` suffix. Relative paths must not contain `..`, and symlinks must resolve inside the user-motion root. If the exact relative file is not found, `scripts/find-user-motion` searches that root for a best `.trk` match: first a unique stem-exact match, then a unique basename/stem contains match. If no filename match exists, it scans `generated/user-motion/**/*.json` metadata and fuzzy-matches the JSON `name` field; a single best match with score `>= 0.80` plays the sibling/declared `.trk`. No match or ambiguous same-priority matches returns `REQUEST_INVALID`.

For long or multi-part requests, use `sequence-start` and return to the user immediately. The workflow has a background worker that prepares and submits a small queue-ahead window so motion generation overlaps current playback.
When a later user message asks for a new action, call `run-text` or a new `sequence-start`; do not append to the old sequence unless the user explicitly says to append/continue after current work.
Each generated segment may include `duration`, `seed`, and `diffusion_steps` in `--plan-json`.
Do not block the turn with `sleep && sequence-status`; poll status only when the user explicitly asks for progress or after a short non-blocking check is operationally necessary.

The skill writes generated/staged `.trk` files to `ET1_ACTION_STAGE_DIR` when set; otherwise it prefers the running tracker's first configured `motion_dirs` entry, then common local generated directories. If the tracker returns `TRK_PATH_NOT_ALLOWED`, set `ET1_ACTION_STAGE_DIR` to a tracker-readable `motion_dirs` directory and retry.
Generated text motions are organized as artifacts under the stage directory. Single `run-text` generations use a local `run_...` artifact id that is not sent to Kimodo. Sequences use the sequence `serial_id` (default: `seq_id`) as the artifact root, with each text segment written under that root. Each generated text segment includes the staged `.trk`, `prompt.txt`, and `meta.json`; sequence roots also maintain `manifest.json`.
Set `ET1_ACTION_DEBUG=1` (also accepts `true`, `yes`, or `on`) to export debug artifacts. Debug mode bypasses presets and invokes the vendored `et1-nl2trk` with `--fresh --debug-dir <artifact-dir>` so the artifact directory also receives `action.bvh`, `output.trk`, and `nl2trk-metadata.json`. Normal mode does not preserve BVH files and keeps the usual preset/cache behavior.

Read references only as needed:

- `references/motion-generation.md` for prompt, duration, seed, and sequence drafting.
- `references/intent-mapping.md` for user interruption mapping.
- `references/sequence-workflow.md` for sequence state and async behavior.
- `references/output-contract.md` for compact JSON output.
- `references/compatibility.md` for migration from older ET1 skills.
