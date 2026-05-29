// FaceHugger clip: one leg lift
// Generated 2026-05-28 from Blender animation
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
  { t: 0, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
  { t: 42, fr:[90,125,37], fl:[89,47,141], br:[89,66,143], bl:[90,125,30] },
  { t: 83, fr:[89,125,37], fl:[88,48,142], br:[88,68,145], bl:[90,125,29] },
  { t: 125, fr:[88,124,36], fl:[87,48,142], br:[88,69,147], bl:[90,124,28] },
  { t: 167, fr:[87,124,35], fl:[86,49,143], br:[88,71,149], bl:[89,124,27] },
  { t: 208, fr:[87,123,35], fl:[86,50,143], br:[87,72,150], bl:[89,123,27] },
  { t: 250, fr:[86,123,34], fl:[85,51,143], br:[87,73,152], bl:[89,123,26] },
  { t: 292, fr:[85,122,33], fl:[84,52,144], br:[86,75,153], bl:[88,123,25] },
  { t: 333, fr:[85,122,33], fl:[83,53,144], br:[86,76,155], bl:[88,123,25] },
  { t: 375, fr:[84,121,32], fl:[83,54,144], br:[86,78,156], bl:[88,122,24] },
  { t: 417, fr:[83,120,31], fl:[82,54,144], br:[85,78,157], bl:[87,122,23] },
  { t: 458, fr:[83,120,31], fl:[82,55,144], br:[85,79,157], bl:[87,122,23] },
  { t: 500, fr:[82,119,30], fl:[81,56,144], br:[85,79,157], bl:[87,122,22] },
  { t: 542, fr:[81,118,29], fl:[80,56,143], br:[84,80,157], bl:[86,121,22] },
  { t: 583, fr:[81,118,29], fl:[80,57,143], br:[84,81,157], bl:[86,121,21] },
  { t: 625, fr:[81,117,28], fl:[80,58,143], br:[84,81,157], bl:[86,121,21] },
  { t: 667, fr:[81,117,28], fl:[80,60,144], br:[84,81,157], bl:[86,121,20] },
  { t: 708, fr:[81,117,28], fl:[80,61,144], br:[84,81,157], bl:[86,121,20] },
  { t: 750, fr:[81,117,27], fl:[80,62,145], br:[84,81,157], bl:[86,120,20] },
  { t: 792, fr:[81,116,27], fl:[80,64,145], br:[84,81,157], bl:[85,120,19] },
  { t: 833, fr:[81,116,27], fl:[80,65,145], br:[84,81,157], bl:[85,120,19] },
  { t: 875, fr:[81,116,26], fl:[80,66,145], br:[84,81,157], bl:[85,120,18] },
  { t: 917, fr:[81,116,26], fl:[80,67,145], br:[84,81,157], bl:[85,119,18] },
  { t: 958, fr:[81,115,26], fl:[80,68,145], br:[84,81,157], bl:[85,119,18] },
  { t: 1000, fr:[81,115,25], fl:[80,69,145], br:[84,81,157], bl:[85,119,17] },
  { t: 1042, fr:[82,115,25], fl:[80,69,145], br:[84,81,157], bl:[85,118,17] },
  { t: 1083, fr:[82,114,25], fl:[80,68,146], br:[84,81,157], bl:[85,118,16] },
  { t: 1125, fr:[82,114,24], fl:[80,68,146], br:[84,81,157], bl:[85,118,16] },
  { t: 1167, fr:[82,114,24], fl:[80,67,146], br:[84,82,157], bl:[85,118,16] },
  { t: 1208, fr:[82,115,25], fl:[81,67,147], br:[84,81,157], bl:[85,118,16] },
  { t: 1250, fr:[82,115,25], fl:[81,66,147], br:[84,81,157], bl:[85,118,17] },
  { t: 1292, fr:[83,116,26], fl:[81,65,148], br:[85,80,157], bl:[86,119,17] },
  { t: 1333, fr:[83,116,26], fl:[82,64,148], br:[85,80,157], bl:[86,119,18] },
  { t: 1375, fr:[84,117,27], fl:[82,64,148], br:[85,79,157], bl:[86,119,19] },
  { t: 1417, fr:[84,118,28], fl:[82,63,149], br:[85,79,157], bl:[86,119,19] },
  { t: 1458, fr:[84,118,28], fl:[83,62,149], br:[86,78,157], bl:[87,120,20] },
  { t: 1500, fr:[85,119,29], fl:[83,61,149], br:[86,78,157], bl:[87,120,20] },
  { t: 1542, fr:[85,119,30], fl:[84,60,149], br:[86,77,157], bl:[87,120,21] },
  { t: 1583, fr:[86,120,30], fl:[84,59,148], br:[86,77,157], bl:[88,121,22] },
  { t: 1625, fr:[86,120,31], fl:[84,58,148], br:[87,76,156], bl:[88,121,23] },
  { t: 1667, fr:[86,121,32], fl:[85,57,148], br:[87,75,154], bl:[88,121,23] },
  { t: 1708, fr:[87,121,32], fl:[85,56,147], br:[87,74,153], bl:[89,122,24] },
  { t: 1750, fr:[87,122,33], fl:[86,54,147], br:[87,73,152], bl:[89,122,25] },
  { t: 1792, fr:[88,122,34], fl:[86,53,146], br:[88,72,150], bl:[89,123,25] },
  { t: 1833, fr:[88,123,34], fl:[87,52,145], br:[88,71,149], bl:[89,123,26] },
  { t: 1875, fr:[89,123,35], fl:[87,51,144], br:[88,70,148], bl:[90,124,27] },
  { t: 1917, fr:[89,124,36], fl:[88,49,143], br:[88,68,146], bl:[90,124,28] },
  { t: 1958, fr:[90,124,36], fl:[88,48,142], br:[89,67,145], bl:[90,124,29] },
  { t: 2000, fr:[90,125,37], fl:[89,47,141], br:[89,66,143], bl:[90,125,30] },
  { t: 2042, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
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
