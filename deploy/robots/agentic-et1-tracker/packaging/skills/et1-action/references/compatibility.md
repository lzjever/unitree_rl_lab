# Compatibility

`et1-action` replaces the older agent-facing ET1 action skills. Agents should not call older text-to-TRK, preset, or tracker-control skills directly.

Internal implementation may use bundled vendor scripts under `vendor/`, but those are not separate skills and should not be installed as active skills.

Packaging should expose:

- `skills/et1-action`
- `bin/et1-action`

Active local migration should move old ET1 action skills from `~/.agents/skills` and `~/.codex/skills` into a timestamped backup directory such as `skills.backup/et1-action-migration-YYYYmmddHHMMSS/`, then install only `et1-action`.
