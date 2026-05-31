// FaceHugger clip: dog walk
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
const CLIP_NAME = "dog walk";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[117,132,30], fl:[63,48,151], br:[63,52,152], bl:[117,132,33] },
  { t: 42, fr:[117,129,29], fl:[63,47,151], br:[63,52,152], bl:[117,129,30] },
  { t: 83, fr:[117,126,28], fl:[63,47,152], br:[63,52,151], bl:[117,126,28] },
  { t: 125, fr:[118,123,27], fl:[63,47,152], br:[63,52,151], bl:[118,124,25] },
  { t: 167, fr:[118,121,27], fl:[63,47,152], br:[63,52,150], bl:[118,121,23] },
  { t: 208, fr:[118,118,27], fl:[63,46,153], br:[62,52,149], bl:[118,119,22] },
  { t: 250, fr:[118,116,28], fl:[63,46,153], br:[62,52,149], bl:[118,116,20] },
  { t: 292, fr:[118,114,29], fl:[63,46,153], br:[62,52,148], bl:[118,114,19] },
  { t: 333, fr:[118,113,31], fl:[63,46,154], br:[62,52,147], bl:[118,112,18] },
  { t: 375, fr:[118,111,33], fl:[64,45,154], br:[62,52,147], bl:[118,110,18] },
  { t: 417, fr:[118,111,35], fl:[64,45,154], br:[62,52,146], bl:[118,108,18] },
  { t: 458, fr:[118,110,39], fl:[64,45,155], br:[62,52,145], bl:[118,106,18] },
  { t: 500, fr:[118,110,43], fl:[64,44,155], br:[62,52,145], bl:[118,104,18] },
  { t: 542, fr:[118,111,41], fl:[64,44,155], br:[62,52,144], bl:[118,107,18] },
  { t: 583, fr:[118,112,40], fl:[64,43,155], br:[62,52,143], bl:[118,109,18] },
  { t: 625, fr:[118,113,39], fl:[64,43,156], br:[62,52,142], bl:[118,112,18] },
  { t: 667, fr:[118,115,39], fl:[64,43,156], br:[62,52,141], bl:[118,115,18] },
  { t: 708, fr:[118,116,38], fl:[64,42,156], br:[62,52,140], bl:[117,117,18] },
  { t: 750, fr:[118,118,38], fl:[64,42,156], br:[62,52,139], bl:[117,120,18] },
  { t: 792, fr:[118,120,39], fl:[65,41,156], br:[62,52,138], bl:[117,122,18] },
  { t: 833, fr:[118,122,39], fl:[65,41,156], br:[62,51,137], bl:[117,125,18] },
  { t: 875, fr:[118,124,40], fl:[65,41,156], br:[62,51,136], bl:[117,127,19] },
  { t: 917, fr:[118,126,41], fl:[65,40,157], br:[62,51,135], bl:[117,131,21] },
  { t: 958, fr:[118,129,43], fl:[65,40,157], br:[62,51,133], bl:[116,134,23] },
  { t: 1000, fr:[118,131,44], fl:[65,39,157], br:[62,50,132], bl:[116,137,26] },
  { t: 1042, fr:[118,131,43], fl:[64,43,160], br:[62,54,138], bl:[117,137,26] },
  { t: 1083, fr:[118,131,41], fl:[63,48,162], br:[62,57,143], bl:[117,136,27] },
  { t: 1125, fr:[118,131,40], fl:[63,52,163], br:[61,60,147], bl:[118,135,27] },
  { t: 1167, fr:[118,131,39], fl:[62,55,163], br:[61,62,151], bl:[118,135,28] },
  { t: 1208, fr:[119,131,37], fl:[62,58,163], br:[61,65,155], bl:[118,134,28] },
  { t: 1250, fr:[119,131,36], fl:[61,62,163], br:[61,67,158], bl:[119,134,29] },
  { t: 1292, fr:[119,131,35], fl:[61,65,163], br:[61,70,162], bl:[119,133,30] },
  { t: 1333, fr:[119,131,34], fl:[61,68,163], br:[61,72,165], bl:[119,133,30] },
  { t: 1375, fr:[119,131,33], fl:[61,71,163], br:[61,74,167], bl:[119,132,31] },
  { t: 1417, fr:[120,132,32], fl:[60,73,161], br:[60,75,167], bl:[120,132,32] },
  { t: 1458, fr:[120,132,31], fl:[60,75,158], br:[60,76,167], bl:[120,132,33] },
  { t: 1500, fr:[120,132,31], fl:[60,76,155], br:[60,77,167], bl:[120,131,34] },
  { t: 1542, fr:[120,133,30], fl:[60,74,154], br:[60,76,167], bl:[120,131,35] },
  { t: 1583, fr:[120,133,29], fl:[60,72,154], br:[60,75,167], bl:[120,131,36] },
  { t: 1625, fr:[121,133,29], fl:[60,70,153], br:[60,74,167], bl:[121,131,37] },
  { t: 1667, fr:[121,134,28], fl:[60,68,152], br:[59,72,166], bl:[121,131,38] },
  { t: 1708, fr:[121,134,27], fl:[59,66,151], br:[59,70,163], bl:[121,131,39] },
  { t: 1750, fr:[122,135,27], fl:[59,63,149], br:[59,67,160], bl:[121,131,41] },
  { t: 1792, fr:[122,135,26], fl:[59,61,148], br:[59,65,158], bl:[121,131,42] },
  { t: 1833, fr:[122,136,26], fl:[59,59,146], br:[59,63,154], bl:[121,131,44] },
  { t: 1875, fr:[123,137,26], fl:[59,56,144], br:[59,60,151], bl:[121,131,45] },
  { t: 1917, fr:[123,137,25], fl:[59,54,142], br:[59,58,147], bl:[121,132,47] },
  { t: 1958, fr:[124,138,25], fl:[59,51,139], br:[58,55,143], bl:[121,132,49] },
  { t: 2000, fr:[124,138,25], fl:[58,49,136], br:[58,52,139], bl:[122,133,51] },
  { t: 2042, fr:[124,134,22], fl:[58,49,136], br:[58,52,139], bl:[122,133,51] },
  { t: 2083, fr:[123,130,20], fl:[58,49,136], br:[58,52,139], bl:[122,133,51] },
  { t: 2125, fr:[123,127,18], fl:[58,49,136], br:[58,52,139], bl:[122,133,51] },
  { t: 2167, fr:[123,124,18], fl:[58,49,136], br:[58,52,139], bl:[122,132,51] },
  { t: 2208, fr:[122,121,18], fl:[58,49,136], br:[58,52,139], bl:[122,132,51] },
  { t: 2250, fr:[122,117,18], fl:[58,49,136], br:[58,52,139], bl:[122,132,51] },
  { t: 2292, fr:[122,114,18], fl:[58,49,136], br:[58,52,139], bl:[122,132,51] },
  { t: 2333, fr:[122,111,18], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2375, fr:[122,108,18], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2417, fr:[121,106,19], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2458, fr:[121,104,21], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2500, fr:[121,103,23], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2542, fr:[121,105,24], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2583, fr:[121,107,25], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2625, fr:[121,109,26], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2667, fr:[121,111,27], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2708, fr:[121,113,29], fl:[58,49,136], br:[58,52,139], bl:[122,132,50] },
  { t: 2750, fr:[121,116,30], fl:[58,49,136], br:[58,52,139], bl:[122,132,49] },
  { t: 2792, fr:[121,118,32], fl:[58,49,136], br:[58,52,139], bl:[122,132,49] },
  { t: 2833, fr:[121,121,34], fl:[58,49,136], br:[58,52,139], bl:[122,132,49] },
  { t: 2875, fr:[121,123,36], fl:[58,49,136], br:[58,52,139], bl:[122,132,49] },
  { t: 2917, fr:[122,126,39], fl:[58,49,136], br:[58,52,139], bl:[122,132,49] },
  { t: 2958, fr:[122,129,42], fl:[58,49,136], br:[58,52,139], bl:[122,132,49] },
  { t: 3000, fr:[122,131,45], fl:[58,49,136], br:[58,52,139], bl:[122,132,49] },
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
