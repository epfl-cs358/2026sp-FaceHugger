// FaceHugger clip: dog walk
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
const CLIP_NAME = "dog walk";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[117,126,35], fl:[63,45,143], br:[63,65,142], bl:[117,126,30] },
  { t: 42, fr:[117,126,35], fl:[63,45,143], br:[63,65,142], bl:[117,125,29] },
  { t: 83, fr:[117,125,34], fl:[63,45,143], br:[63,65,142], bl:[117,125,28] },
  { t: 125, fr:[117,124,32], fl:[63,45,143], br:[63,65,142], bl:[117,124,27] },
  { t: 167, fr:[117,122,31], fl:[63,44,143], br:[63,65,142], bl:[117,122,25] },
  { t: 208, fr:[117,121,29], fl:[63,44,143], br:[63,65,142], bl:[117,121,23] },
  { t: 250, fr:[117,119,27], fl:[63,44,144], br:[63,65,141], bl:[117,119,21] },
  { t: 292, fr:[117,117,25], fl:[63,44,144], br:[63,65,141], bl:[117,118,19] },
  { t: 333, fr:[117,115,24], fl:[63,44,144], br:[63,65,140], bl:[117,116,18] },
  { t: 375, fr:[117,114,23], fl:[63,44,144], br:[63,65,140], bl:[117,115,16] },
  { t: 417, fr:[117,113,23], fl:[63,43,145], br:[62,65,139], bl:[117,114,15] },
  { t: 458, fr:[117,112,23], fl:[63,43,145], br:[62,65,139], bl:[117,114,15] },
  { t: 500, fr:[117,111,23], fl:[63,43,146], br:[62,65,138], bl:[117,113,15] },
  { t: 542, fr:[117,111,23], fl:[64,42,146], br:[62,65,137], bl:[117,114,15] },
  { t: 583, fr:[117,112,24], fl:[64,42,146], br:[62,65,136], bl:[117,114,15] },
  { t: 625, fr:[118,113,26], fl:[64,42,147], br:[62,65,136], bl:[117,115,15] },
  { t: 667, fr:[118,114,28], fl:[64,41,147], br:[62,65,135], bl:[117,117,16] },
  { t: 708, fr:[118,116,30], fl:[64,41,147], br:[62,65,134], bl:[117,119,17] },
  { t: 750, fr:[118,117,32], fl:[64,40,147], br:[62,65,133], bl:[117,121,18] },
  { t: 792, fr:[118,119,35], fl:[64,40,148], br:[62,65,132], bl:[117,122,20] },
  { t: 833, fr:[118,121,37], fl:[64,39,148], br:[62,65,130], bl:[117,124,21] },
  { t: 875, fr:[118,122,39], fl:[64,39,148], br:[62,65,129], bl:[117,126,23] },
  { t: 917, fr:[118,124,40], fl:[65,38,148], br:[62,65,128], bl:[117,127,24] },
  { t: 958, fr:[117,125,41], fl:[65,38,148], br:[62,64,127], bl:[117,128,25] },
  { t: 1000, fr:[117,125,41], fl:[65,37,148], br:[62,64,126], bl:[117,128,26] },
  { t: 1042, fr:[117,125,40], fl:[65,38,149], br:[62,64,125], bl:[117,127,26] },
  { t: 1083, fr:[117,125,40], fl:[65,39,150], br:[62,65,126], bl:[117,127,27] },
  { t: 1125, fr:[117,125,39], fl:[64,41,151], br:[62,67,128], bl:[117,127,27] },
  { t: 1167, fr:[117,125,38], fl:[64,44,153], br:[62,69,131], bl:[117,127,28] },
  { t: 1208, fr:[117,125,38], fl:[63,48,154], br:[62,71,135], bl:[117,126,28] },
  { t: 1250, fr:[117,126,37], fl:[63,51,155], br:[62,73,138], bl:[117,126,28] },
  { t: 1292, fr:[117,126,37], fl:[63,54,155], br:[62,75,141], bl:[117,126,29] },
  { t: 1333, fr:[117,126,37], fl:[63,57,155], br:[62,77,145], bl:[117,126,29] },
  { t: 1375, fr:[117,126,36], fl:[63,59,154], br:[62,79,147], bl:[117,126,30] },
  { t: 1417, fr:[117,126,36], fl:[62,61,154], br:[62,80,150], bl:[117,126,30] },
  { t: 1458, fr:[117,126,36], fl:[62,62,153], br:[62,81,152], bl:[117,126,30] },
  { t: 1500, fr:[117,126,35], fl:[62,62,153], br:[62,81,154], bl:[117,125,31] },
  { t: 1542, fr:[117,127,35], fl:[62,62,153], br:[62,81,154], bl:[117,125,31] },
  { t: 1583, fr:[117,127,35], fl:[62,61,153], br:[62,80,155], bl:[117,125,31] },
  { t: 1625, fr:[117,127,35], fl:[62,60,152], br:[62,78,154], bl:[117,125,32] },
  { t: 1667, fr:[117,127,34], fl:[62,58,151], br:[62,77,153], bl:[117,125,32] },
  { t: 1708, fr:[117,127,34], fl:[62,56,149], br:[62,75,152], bl:[117,125,32] },
  { t: 1750, fr:[117,127,34], fl:[63,54,147], br:[63,73,151], bl:[117,125,32] },
  { t: 1792, fr:[117,127,34], fl:[63,52,145], br:[63,71,149], bl:[118,125,32] },
  { t: 1833, fr:[117,127,34], fl:[63,50,143], br:[63,69,147], bl:[118,125,33] },
  { t: 1875, fr:[117,127,34], fl:[63,49,141], br:[63,67,145], bl:[118,125,33] },
  { t: 1917, fr:[117,127,34], fl:[63,47,140], br:[63,66,144], bl:[118,125,33] },
  { t: 1958, fr:[117,127,34], fl:[63,46,139], br:[63,65,143], bl:[118,125,33] },
  { t: 2000, fr:[117,127,34], fl:[63,46,138], br:[63,65,143], bl:[118,125,33] },
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
