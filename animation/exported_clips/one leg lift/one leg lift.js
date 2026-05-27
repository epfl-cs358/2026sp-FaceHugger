// FaceHugger clip: one leg lift
// Generated 2026-05-27 from Blender animation
//
// HOW TO RUN (browser):
//   1. Join the robot Wi-Fi (FaceHugger_Net); robot at 192.168.4.1.
//   2. Open a console on a NON-HTTPS page (http://, file://, or
//      about:blank). ws:// is BLOCKED from https:// (mixed content)
//      — the #1 reason "nothing happens".
//   3. Paste this whole file. Call  fhStop()  to stop at any time.
//
// Wire shape matches origin/main CMD_CALIBRATE (T:4):
//   {T:4, id:<leg_id 0-3>, servo_id:<0-2>, a:<0-180>}
// Firmware does PCA-channel mapping via LEG_SERVO_CHANNEL.
//
// PLAYBACK SEMANTICS (Phase-1 parity with firmware tickClip, see spec
// doc/animation-pipeline/onboard-clip-player-design.md §3.1):
//   - DEFAULT: play the clip ONCE, then HOLD the final pose by
//     re-sending the last frame's servo angles every FRAME_MS — the
//     same as the firmware's hold-at-end behaviour.
//   - LOOP=true: replay from frame 0 instead of holding (diagnostic).
//   - fhStop(): explicit safe stop — clears the interval and closes
//     the socket. The robot keeps the last commanded servo positions
//     (servos hold their last commanded angle in hardware).

const LOOP = false;
// Playback wall-clock period — derived from the Blender scene FPS at
// bake time (median delta of `t` between consecutive frames), so this
// clip plays at the speed it was authored. Each scheduled tick sends
// the next CLIP[] frame.
const FRAME_MS = 42;
const CLIP_NAME = "one leg lift";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[90,131,32], fl:[89,50,149], br:[89,53,153], bl:[90,131,33] },
  { t: 83, fr:[89,131,32], fl:[88,51,150], br:[88,55,155], bl:[90,131,32] },
  { t: 125, fr:[88,130,31], fl:[87,51,150], br:[88,56,157], bl:[90,130,31] },
  { t: 167, fr:[87,130,30], fl:[86,52,151], br:[88,58,159], bl:[89,130,30] },
  { t: 208, fr:[87,129,30], fl:[86,53,151], br:[87,59,160], bl:[89,129,30] },
  { t: 250, fr:[86,129,29], fl:[85,54,151], br:[87,60,162], bl:[89,129,29] },
  { t: 292, fr:[85,128,28], fl:[84,55,152], br:[86,62,163], bl:[88,129,28] },
  { t: 333, fr:[85,128,28], fl:[83,56,152], br:[86,63,165], bl:[88,129,28] },
  { t: 375, fr:[84,127,27], fl:[83,57,152], br:[86,65,166], bl:[88,128,27] },
  { t: 417, fr:[83,126,26], fl:[82,57,152], br:[85,65,167], bl:[87,128,26] },
  { t: 458, fr:[83,126,26], fl:[82,58,152], br:[85,66,167], bl:[87,128,26] },
  { t: 500, fr:[82,125,25], fl:[81,59,152], br:[85,66,167], bl:[87,128,25] },
  { t: 542, fr:[81,124,24], fl:[80,59,151], br:[84,67,167], bl:[86,127,25] },
  { t: 583, fr:[81,124,24], fl:[80,60,151], br:[84,68,167], bl:[86,127,24] },
  { t: 625, fr:[81,123,23], fl:[80,61,151], br:[84,68,167], bl:[86,127,24] },
  { t: 667, fr:[81,123,23], fl:[80,63,152], br:[84,68,167], bl:[86,127,23] },
  { t: 708, fr:[81,123,23], fl:[80,64,152], br:[84,68,167], bl:[86,127,23] },
  { t: 750, fr:[81,123,22], fl:[80,65,153], br:[84,68,167], bl:[86,126,23] },
  { t: 792, fr:[81,122,22], fl:[80,67,153], br:[84,68,167], bl:[85,126,22] },
  { t: 833, fr:[81,122,22], fl:[80,68,153], br:[84,68,167], bl:[85,126,22] },
  { t: 875, fr:[81,122,21], fl:[80,69,153], br:[84,68,167], bl:[85,126,21] },
  { t: 917, fr:[81,122,21], fl:[80,70,153], br:[84,68,167], bl:[85,125,21] },
  { t: 958, fr:[81,121,21], fl:[80,71,153], br:[84,68,167], bl:[85,125,21] },
  { t: 1000, fr:[81,121,20], fl:[80,72,153], br:[84,68,167], bl:[85,125,20] },
  { t: 1042, fr:[82,121,20], fl:[80,72,153], br:[84,68,167], bl:[85,124,20] },
  { t: 1083, fr:[82,120,20], fl:[80,71,154], br:[84,68,167], bl:[85,124,19] },
  { t: 1125, fr:[82,120,19], fl:[80,71,154], br:[84,68,167], bl:[85,124,19] },
  { t: 1167, fr:[82,120,19], fl:[80,70,154], br:[84,69,167], bl:[85,124,19] },
  { t: 1208, fr:[82,121,20], fl:[81,70,155], br:[84,68,167], bl:[85,124,19] },
  { t: 1250, fr:[82,121,20], fl:[81,69,155], br:[84,68,167], bl:[85,124,20] },
  { t: 1292, fr:[83,122,21], fl:[81,68,156], br:[85,67,167], bl:[86,125,20] },
  { t: 1333, fr:[83,122,21], fl:[82,67,156], br:[85,67,167], bl:[86,125,21] },
  { t: 1375, fr:[84,123,22], fl:[82,67,156], br:[85,66,167], bl:[86,125,22] },
  { t: 1417, fr:[84,124,23], fl:[82,66,157], br:[85,66,167], bl:[86,125,22] },
  { t: 1458, fr:[84,124,23], fl:[83,65,157], br:[86,65,167], bl:[87,126,23] },
  { t: 1500, fr:[85,125,24], fl:[83,64,157], br:[86,65,167], bl:[87,126,23] },
  { t: 1542, fr:[85,125,25], fl:[84,63,157], br:[86,64,167], bl:[87,126,24] },
  { t: 1583, fr:[86,126,25], fl:[84,62,156], br:[86,64,167], bl:[88,127,25] },
  { t: 1625, fr:[86,126,26], fl:[84,61,156], br:[87,63,166], bl:[88,127,26] },
  { t: 1667, fr:[86,127,27], fl:[85,60,156], br:[87,62,164], bl:[88,127,26] },
  { t: 1708, fr:[87,127,27], fl:[85,59,155], br:[87,61,163], bl:[89,128,27] },
  { t: 1750, fr:[87,128,28], fl:[86,57,155], br:[87,60,162], bl:[89,128,28] },
  { t: 1792, fr:[88,128,29], fl:[86,56,154], br:[88,59,160], bl:[89,129,28] },
  { t: 1833, fr:[88,129,29], fl:[87,55,153], br:[88,58,159], bl:[89,129,29] },
  { t: 1875, fr:[89,129,30], fl:[87,54,152], br:[88,57,158], bl:[90,130,30] },
  { t: 1917, fr:[89,130,31], fl:[88,52,151], br:[88,55,156], bl:[90,130,31] },
  { t: 1958, fr:[90,130,31], fl:[88,51,150], br:[89,54,155], bl:[90,130,32] },
  { t: 2000, fr:[90,131,32], fl:[89,50,149], br:[89,53,153], bl:[90,131,33] },
  { t: 2042, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
];

// Servos accept 0..180; clamp defensively (extreme poses / a drifted
// convention can push the converted angle out of range).
const clamp = (v) => Math.max(0, Math.min(180, v | 0));

// Delta-encode: only emit a channel when its value changed since the last
// frame. Cuts redundant traffic and ends the hold-at-end resend flood
// (a held pose = unchanged angles = nothing sent). Reset to {} on restart
// so the first frame after a (re)start always sends all 12 channels.
let _last = {};

let _i = 0;
let _timer = null;
const ws = new WebSocket("ws://192.168.4.1:81");

function fhStop() {
  if (_timer !== null) { clearInterval(_timer); _timer = null; }
  try { ws.close(); } catch (e) {}
  console.log("FaceHugger: playback stopped (servos hold last commanded pose).");
}
globalThis.fhStop = fhStop;

function playFrame() {
  // End of clip: in LOOP mode wrap to the start; otherwise clamp to
  // the last frame and keep re-sending it (hold-at-end per spec §3.1).
  if (_i >= CLIP.length) {
    if (LOOP) {
      _i = 0;
      _last = {};  // reset delta cache so loop restart re-sends all channels
    } else {
      _i = CLIP.length - 1;
    }
  }
  const frame = CLIP[_i++];
  if (ws.readyState !== WebSocket.OPEN) return;
  for (const leg of ["fr", "fl", "br", "bl"]) {
    const angles = frame[leg];
    for (let j = 0; j < 3; j++) {
      const a = clamp(angles[j]);
      const key = leg + ":" + j;
      if (_last[key] === a) continue;
      _last[key] = a;
      const msg = { "T": 4, "id": LEG_IDS[leg], "servo_id": j, "a": a };
      ws.send(JSON.stringify(msg));
    }
  }
}

ws.onopen = () => {
  console.log("FaceHugger: connected — playing " + CLIP_NAME + " (" + CLIP.length + " frames). Hold-at-end is on by default; call fhStop() when done.");
  _timer = setInterval(playFrame, FRAME_MS);
};
ws.onerror = () => {
  alert("FaceHugger: could not connect to " + ws.url +
        " - check the robot IP / Wi-Fi network and reload.");
};
ws.onclose = (e) => { if (!e.wasClean) console.warn("FaceHugger WS closed", e.code); };
