# Standby Reference Asset

This directory contains the app-owned standby reference release asset for
`agentic-et1-tracker`:

- `standby_ref.trk`
- `ASSET_MANIFEST.yaml`

`standby_ref.trk` was copied byte-for-byte from the build-local
`build/standby_ref_candidate/standby_ref.candidate.trk` after simulator visual
review accepted that candidate on 2026-06-03.

The release asset is audit material for the gated standby reference transition.
It is not a user motion, idle pool entry, runtime fallback, downloader config,
or permission to read from the ET1 app tree. Runtime playback logic was not
changed by this asset promotion.

The real-robot/operator gate is still pending. Do not use this asset record to
claim overall GA or hardware acceptance.
