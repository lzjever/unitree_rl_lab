# Motion Generation

Use this reference when the user asks for a generated motion rather than an existing `.trk`.

## Prompt Shape

Always send a concise English Kimodo-style prompt to `run-text`, even when the user speaks Chinese.

Good shape:

```text
A person <does one clear motion> <direction or object if needed> <style or tempo>.
```

Guidelines:

- Start with `A person ...` unless a short style subject makes the intent clearer.
- Keep one segment focused on one or at most two behaviors.
- Include action, direction, tempo/style, and simple object interaction when useful.
- Avoid long stories, camera language, robot joint names, and detailed per-limb choreography.
- Prefer common human behaviors: walk, turn, run, crouch, sit, stand, gesture, dance, simple object interaction, and simple combat.
- For long routines, split into short sequence segments instead of one overloaded prompt.

## Duration

Use `--duration` or per-segment `duration` when the user implies timing or when a better default is obvious.

Suggested durations:

| Motion | Duration |
| --- | ---: |
| Small gesture | `2-3` |
| Walk a few steps, turn, or wave while stepping | `3-5` |
| Crouch/pick up/stand, sit/stand transition | `4-6` |
| Short dance, stylized walk, cautious motion, simple combat | `5-8` |
| Two connected behaviors | `7-10` |

Keep each generated prompt at `1.0..10.0` seconds unless the user gives a shorter valid value.

## Seed And Quality

`--seed N` makes a generated request reproducible and is useful when retrying variants. If the user says "try another version", change the seed. If the user wants the same motion again, reuse the seed.

`--diffusion-steps N` is an advanced quality/speed knob. Higher values may improve generation but cost more time. Use only when requested or when explicitly tuning a failed generation.

When `--seed` or `--diffusion-steps` is set, `run-text` bypasses presets and calls generation directly so the requested control is real.

## Hold And Precise Limbs

Use `--hold` when the user asks to keep the final pose. For precise left/right limb requests, also use `--no-preset` so a loose preset match does not replace the intended motion.

Command shape:

```bash
scripts/et1-action run-text "<concise English final-pose prompt>" --duration 4 --hold --no-preset
```

## Pattern Hints

- For directional movement, include direction and rough distance.
- For turns, include turn direction and whether movement continues after the turn.
- For style, use one short adjective or adverbial phrase.
- For final-pose requests, include the final pose in the prompt and pass `--hold`.
- For left/right limb specificity, include the side in the prompt and pass `--no-preset`.

## Sequence Planning

For a multi-step user prompt, use `sequence-start` with short segments. Keep serial continuity on by leaving `--serial-id` unset; the skill assigns one sequence ID and passes it to generation.

Command shape:

```bash
scripts/et1-action sequence-start --plan-json '{"segments":[{"text":"<segment prompt>","duration":4},{"text":"<next segment prompt>","duration":4}]}'
```

Poll with:

```bash
scripts/et1-action sequence-status SEQ_ID
```
