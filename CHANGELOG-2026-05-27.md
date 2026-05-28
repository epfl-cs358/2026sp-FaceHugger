# Changelog — 2026-05-27

Plain-language summary of the work done on `feat/animation-flow-integration`
today, in the same style as the earlier dated changelogs. Grouped by area.

## Remote-control app — play & manage clips, invert, connection

The app can now drive and curate animations directly, and survives sim
telemetry that the real robot wouldn't send.

- **Clip list on the Actions page.** On connect the app requests the flashed
  clip registry (`T:8`) and lists each clip as a button (tap to play, `T:7`).
  A three-way **All / Flashed / App** toggle shows on-robot clips, app-bundled
  clips, or both; a clip present in both gets side-by-side **Robot** and **App**
  play buttons. Fixed a bug where the flashed list never appeared — the `T:8`
  reply carries no `T` field, so the handler dropped it.
- **App-streamed clips (no flash).** The app bundles `clips_extra.json` and can
  stream a clip's servo frames itself over the socket (`T:4` per changed channel
  at the authored frame rate), so an animation is playable before it's flashed.
  This is a preview path — it depends on the live connection, unlike an on-robot
  clip.
- **Invert is now consistent everywhere.** App-streamed clips and the rest/neutral
  pose buttons send raw `T:4` angles the firmware invert flag can't touch, so the
  app mirrors them itself (thigh/knee → 180−angle, hip unchanged — the same
  `applyInvert` math), gated by an app-side flag kept in sync with `T:6`.
- **Invert confirmation** moved from an inline row that reflowed the page to a
  slide-up bottom sheet (dim backdrop, tap-outside / Cancel to dismiss).
- **Tap-to-retry connection.** The "Not Connected" badge is now tappable: it
  forces a fresh reconnect (re-running the connection effect so the message
  handler re-attaches) and shows a transient Retrying… / Couldn't-connect hint.
- **Telemetry crash fix.** The sim sends `T:10` with `d`/`a` as null (no
  ToF/AMU sensors); the status handler destructured the AMU array and crashed.
  Both arrays are now guarded.

## Firmware — trot mirror fix, clip loop, runtime smoothing

- **Trot left/right mirror fix.** The 2026-05-25 BR un-mirror and the FL
  "Change B" remount rewrote `translateToServo` and were applied in tandem to
  `tickGait`/`tickYawRotation`, but `tickTrot` was missed. Result: BR's rear
  shoulder swept the wrong way (both rear legs swept together → the back veered)
  and FL's front shoulder sat 25–60° off neutral. Adapted `tickTrot` to the new
  conventions so it reproduces the validated pre-branch (teammates' JS→C) trot:
  flipped BR's rear sweep sign, and re-anchored FL's HIP table to the new neutral
  while keeping the old sweep delta. Crab/walk go through `tickGait` and were
  already correct.
- **Clip loop.** `T:7` accepts an optional `"loop": true` — the clip replays from
  the start at each end instead of returning to neutral, until another motion
  command preempts it. Default stays play-once.
- **Runtime smoothing knob.** The clip-playback EMA alpha is now adjustable live
  via `T:11` (`{a: 0..0.95}`) instead of being a compile-time constant — tune
  snappy↔smooth without reflashing. Clip path only; clamped so it can't stall.

## Simulation — grippier feet

Raised the sim's foot friction (`FOOT_LATERAL_FRICTION` 1.2→2.5,
`FOOT_SPINNING_FRICTION` 0.05→0.3) to model the rubber/elastic bands on the real
feet — less slip on planted-feet moves and less pivot-in-place.

## Animation exporter — app clip bundle

The FH Clip Panel now also writes `clips_extra.json` (final servo frames for
every clip) and can copy it into the app's assets, mirroring how `clips_all.h`
is copied into the firmware tree. New panel toggles and an "App" folder button.

## CLI — clean shutdown

`facehugger.py sim --app` (and the standalone `app`) now run each child
(ws_sim + Expo) in its own process group and tear the whole tree down on a single
Ctrl-C, instead of orphaning Expo's Metro/node processes.

## Protocol

`code/API_SPEC.md` documents the `T:7` `loop` field and the new `T:11`
(Set Clip Smoothing) command.

## Testing

Firmware changes are covered by SIL tests (run against the compiled firmware):
rear-shoulder mirror + FL anchoring for the trot, one-shot-vs-loop clip
behaviour, and the smoothing knob changing playback (and clamping). The ESP32
build (`pio run`) and the app typecheck/jest suite pass. One pre-existing
golden (`dancing`) fails pending a clip re-export — unrelated to these changes.
