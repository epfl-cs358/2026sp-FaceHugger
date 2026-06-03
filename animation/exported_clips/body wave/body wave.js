// FaceHugger clip: body wave
// Generated 2026-05-29 from Blender animation
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
const CLIP_NAME = "body wave";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,131,32], fl:[89,48,148], br:[89,52,152], bl:[91,132,34] },
  { t: 83, fr:[90,130,31], fl:[89,47,147], br:[90,53,153], bl:[91,133,35] },
  { t: 125, fr:[90,128,29], fl:[88,46,146], br:[90,55,155], bl:[92,134,36] },
  { t: 167, fr:[89,126,27], fl:[87,45,145], br:[91,57,157], bl:[93,135,37] },
  { t: 208, fr:[88,124,25], fl:[86,44,143], br:[92,59,160], bl:[94,136,38] },
  { t: 250, fr:[88,122,22], fl:[85,43,142], br:[92,62,162], bl:[95,137,40] },
  { t: 292, fr:[87,120,20], fl:[84,42,141], br:[93,64,164], bl:[96,138,41] },
  { t: 333, fr:[87,118,18], fl:[83,41,140], br:[93,66,166], bl:[97,139,41] },
  { t: 375, fr:[87,117,18], fl:[83,41,140], br:[93,67,167], bl:[97,139,41] },
  { t: 417, fr:[87,116,18], fl:[83,41,141], br:[93,67,167], bl:[97,139,41] },
  { t: 458, fr:[87,116,18], fl:[83,42,142], br:[93,67,167], bl:[97,138,39] },
  { t: 500, fr:[86,117,18], fl:[83,41,141], br:[93,68,167], bl:[97,136,37] },
  { t: 542, fr:[86,117,18], fl:[84,40,141], br:[92,69,167], bl:[97,134,35] },
  { t: 583, fr:[85,118,18], fl:[84,39,140], br:[91,70,167], bl:[97,131,32] },
  { t: 625, fr:[85,119,18], fl:[85,38,139], br:[91,71,167], bl:[97,129,29] },
  { t: 667, fr:[84,121,19], fl:[86,37,137], br:[90,72,167], bl:[97,126,26] },
  { t: 708, fr:[83,123,21], fl:[87,36,136], br:[89,73,167], bl:[97,123,23] },
  { t: 750, fr:[83,125,22], fl:[87,35,136], br:[88,74,167], bl:[96,120,20] },
  { t: 792, fr:[82,127,23], fl:[88,35,136], br:[88,75,167], bl:[96,118,18] },
  { t: 833, fr:[82,128,25], fl:[89,35,137], br:[87,75,167], bl:[95,117,18] },
  { t: 875, fr:[82,130,26], fl:[90,37,140], br:[86,75,167], bl:[94,116,18] },
  { t: 917, fr:[82,131,27], fl:[90,39,142], br:[86,74,167], bl:[94,115,18] },
  { t: 958, fr:[82,133,29], fl:[91,41,145], br:[85,73,167], bl:[93,114,18] },
  { t: 1000, fr:[83,134,30], fl:[91,44,148], br:[84,72,167], bl:[92,113,18] },
  { t: 1042, fr:[83,136,32], fl:[92,46,151], br:[83,71,167], bl:[91,112,18] },
  { t: 1083, fr:[83,138,34], fl:[92,48,153], br:[83,71,167], bl:[90,110,18] },
  { t: 1125, fr:[84,139,36], fl:[93,50,155], br:[82,70,167], bl:[89,109,18] },
  { t: 1167, fr:[84,141,38], fl:[93,52,157], br:[81,69,167], bl:[89,108,18] },
  { t: 1208, fr:[85,142,39], fl:[93,54,159], br:[80,69,167], bl:[88,106,18] },
  { t: 1250, fr:[86,144,41], fl:[93,56,161], br:[79,68,167], bl:[88,105,18] },
  { t: 1292, fr:[86,144,42], fl:[93,57,162], br:[79,68,167], bl:[87,105,18] },
  { t: 1333, fr:[86,145,42], fl:[93,58,163], br:[79,67,167], bl:[87,104,18] },
  { t: 1375, fr:[87,145,42], fl:[93,58,163], br:[78,67,167], bl:[87,104,18] },
  { t: 1417, fr:[87,144,41], fl:[93,59,163], br:[78,67,167], bl:[87,105,18] },
  { t: 1458, fr:[88,142,39], fl:[92,60,163], br:[79,66,167], bl:[87,106,18] },
  { t: 1500, fr:[88,140,36], fl:[91,61,163], br:[79,64,166], bl:[86,108,18] },
  { t: 1542, fr:[89,137,33], fl:[90,63,163], br:[79,61,163], bl:[85,110,18] },
  { t: 1583, fr:[90,134,30], fl:[89,64,163], br:[79,58,160], bl:[84,112,18] },
  { t: 1625, fr:[91,130,26], fl:[88,66,163], br:[79,54,157], bl:[83,114,18] },
  { t: 1667, fr:[91,127,23], fl:[87,69,163], br:[80,51,154], bl:[82,116,18] },
  { t: 1708, fr:[92,124,21], fl:[86,70,163], br:[80,48,151], bl:[81,117,18] },
  { t: 1750, fr:[92,122,19], fl:[85,72,163], br:[81,46,149], bl:[80,118,18] },
  { t: 1792, fr:[92,121,18], fl:[85,73,163], br:[81,44,147], bl:[79,119,18] },
  { t: 1833, fr:[92,120,18], fl:[85,73,163], br:[81,44,147], bl:[79,120,18] },
  { t: 1875, fr:[92,120,18], fl:[85,73,163], br:[82,44,147], bl:[79,120,18] },
  { t: 1917, fr:[91,120,18], fl:[85,72,163], br:[82,45,148], bl:[80,120,18] },
  { t: 1958, fr:[91,120,18], fl:[85,71,163], br:[83,45,148], bl:[80,121,18] },
  { t: 2000, fr:[91,121,18], fl:[85,69,163], br:[83,46,148], bl:[81,122,19] },
  { t: 2042, fr:[91,122,19], fl:[85,68,163], br:[84,46,149], bl:[82,123,20] },
  { t: 2083, fr:[91,123,21], fl:[86,66,163], br:[85,47,149], bl:[83,124,22] },
  { t: 2125, fr:[91,124,22], fl:[86,64,163], br:[85,48,150], bl:[84,126,24] },
  { t: 2167, fr:[91,125,24], fl:[87,63,163], br:[86,49,150], bl:[85,127,26] },
  { t: 2208, fr:[91,126,26], fl:[87,60,160], br:[87,49,150], bl:[86,128,27] },
  { t: 2250, fr:[91,128,28], fl:[88,57,157], br:[88,50,151], bl:[87,129,29] },
  { t: 2292, fr:[91,129,29], fl:[88,54,155], br:[88,51,151], bl:[88,130,30] },
  { t: 2333, fr:[91,130,31], fl:[89,52,152], br:[89,51,151], bl:[89,130,32] },
  { t: 2375, fr:[91,131,32], fl:[89,50,150], br:[89,51,151], bl:[90,131,33] },
  { t: 2417, fr:[91,131,33], fl:[89,49,149], br:[89,52,151], bl:[91,131,33] },
  { t: 2458, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
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
