// FaceHugger clip: fallingRobot
// Generated 2026-05-25 from Blender animation
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
const CLIP_NAME = "fallingRobot";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 83, fr:[93,131,35], fl:[87,49,146], br:[89,52,151], bl:[91,131,34] },
  { t: 125, fr:[95,130,39], fl:[85,50,142], br:[89,52,151], bl:[91,131,34] },
  { t: 167, fr:[97,130,45], fl:[83,50,136], br:[89,52,151], bl:[91,131,34] },
  { t: 208, fr:[100,133,56], fl:[80,47,125], br:[89,52,151], bl:[91,131,34] },
  { t: 250, fr:[102,140,77], fl:[78,40,104], br:[89,52,151], bl:[91,131,34] },
  { t: 292, fr:[104,137,77], fl:[76,43,104], br:[89,52,151], bl:[91,131,34] },
  { t: 333, fr:[106,134,77], fl:[74,46,104], br:[89,52,151], bl:[91,131,34] },
  { t: 375, fr:[107,131,77], fl:[73,49,104], br:[89,52,151], bl:[91,131,34] },
  { t: 417, fr:[109,128,77], fl:[71,52,104], br:[89,52,151], bl:[91,131,34] },
  { t: 458, fr:[110,126,77], fl:[70,54,104], br:[89,52,151], bl:[91,131,34] },
  { t: 500, fr:[111,124,77], fl:[69,56,104], br:[89,52,151], bl:[91,131,34] },
  { t: 542, fr:[111,122,77], fl:[69,58,104], br:[89,52,151], bl:[91,131,34] },
  { t: 583, fr:[112,121,77], fl:[68,59,104], br:[89,52,151], bl:[91,131,34] },
  { t: 625, fr:[112,119,77], fl:[68,61,104], br:[89,52,151], bl:[91,131,34] },
  { t: 667, fr:[113,118,77], fl:[67,62,104], br:[89,52,151], bl:[91,131,34] },
  { t: 708, fr:[113,117,77], fl:[67,63,104], br:[89,52,151], bl:[91,131,34] },
  { t: 750, fr:[114,116,77], fl:[66,64,104], br:[89,52,151], bl:[91,131,34] },
  { t: 792, fr:[114,115,77], fl:[66,65,104], br:[89,52,151], bl:[91,131,34] },
  { t: 833, fr:[114,114,77], fl:[66,66,104], br:[89,52,151], bl:[91,131,34] },
  { t: 875, fr:[114,114,77], fl:[66,66,104], br:[89,52,151], bl:[91,131,34] },
  { t: 917, fr:[114,113,77], fl:[66,67,104], br:[89,52,151], bl:[91,131,34] },
  { t: 958, fr:[114,112,77], fl:[66,68,104], br:[89,52,151], bl:[91,131,34] },
  { t: 1000, fr:[114,111,78], fl:[66,69,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1042, fr:[114,110,78], fl:[66,70,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1083, fr:[114,109,78], fl:[66,71,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1125, fr:[114,108,78], fl:[66,72,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1167, fr:[114,107,78], fl:[66,73,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1208, fr:[114,106,78], fl:[66,74,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1250, fr:[114,104,78], fl:[66,76,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1292, fr:[113,103,78], fl:[67,77,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1333, fr:[113,101,78], fl:[67,79,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1375, fr:[112,99,78], fl:[68,81,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1417, fr:[112,97,78], fl:[68,83,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1458, fr:[111,94,78], fl:[69,86,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1500, fr:[111,92,78], fl:[69,88,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1542, fr:[110,89,78], fl:[70,91,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1583, fr:[109,85,78], fl:[71,95,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1625, fr:[108,82,78], fl:[72,98,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1667, fr:[106,78,78], fl:[74,102,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1708, fr:[105,75,78], fl:[75,105,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1750, fr:[103,71,78], fl:[77,109,103], br:[89,52,151], bl:[91,131,34] },
  { t: 1792, fr:[101,70,80], fl:[79,110,101], br:[89,52,151], bl:[91,131,34] },
  { t: 1833, fr:[99,70,84], fl:[81,110,97], br:[89,52,151], bl:[91,131,34] },
  { t: 1875, fr:[97,70,88], fl:[83,110,93], br:[89,52,151], bl:[91,131,34] },
  { t: 1917, fr:[95,70,90], fl:[85,110,91], br:[89,52,151], bl:[91,131,34] },
  { t: 1958, fr:[94,70,91], fl:[86,110,90], br:[89,52,151], bl:[91,131,34] },
  { t: 2000, fr:[94,70,92], fl:[86,110,89], br:[89,52,151], bl:[91,131,34] },
  { t: 2042, fr:[94,70,92], fl:[86,110,89], br:[89,52,151], bl:[91,131,34] },
  { t: 2083, fr:[94,70,92], fl:[86,110,89], br:[89,52,151], bl:[91,131,34] },
  { t: 2125, fr:[94,70,92], fl:[86,110,89], br:[89,52,151], bl:[91,131,34] },
  { t: 2167, fr:[94,70,92], fl:[86,110,89], br:[89,52,151], bl:[91,131,34] },
  { t: 2208, fr:[94,70,92], fl:[86,110,89], br:[89,52,151], bl:[91,131,34] },
  { t: 2250, fr:[94,70,91], fl:[86,110,90], br:[89,52,151], bl:[91,131,34] },
  { t: 2292, fr:[94,70,91], fl:[86,110,90], br:[89,52,151], bl:[91,131,34] },
  { t: 2333, fr:[94,70,91], fl:[86,110,90], br:[89,52,151], bl:[91,131,34] },
  { t: 2375, fr:[94,70,91], fl:[86,110,90], br:[89,52,151], bl:[91,131,34] },
  { t: 2417, fr:[95,70,91], fl:[85,110,90], br:[89,52,151], bl:[91,131,34] },
  { t: 2458, fr:[95,70,90], fl:[85,110,91], br:[89,52,151], bl:[91,131,34] },
  { t: 2500, fr:[95,70,90], fl:[85,110,91], br:[89,52,151], bl:[91,131,34] },
  { t: 2542, fr:[96,70,90], fl:[84,110,91], br:[89,52,151], bl:[91,131,34] },
  { t: 2583, fr:[96,70,89], fl:[84,110,92], br:[89,52,151], bl:[91,131,34] },
  { t: 2625, fr:[96,70,89], fl:[84,110,92], br:[89,52,151], bl:[91,131,34] },
  { t: 2667, fr:[97,70,88], fl:[83,110,93], br:[89,52,151], bl:[91,131,34] },
  { t: 2708, fr:[97,70,88], fl:[83,110,93], br:[89,52,151], bl:[91,131,34] },
  { t: 2750, fr:[98,70,87], fl:[82,110,94], br:[89,52,151], bl:[91,131,34] },
  { t: 2792, fr:[98,70,86], fl:[82,110,95], br:[89,52,151], bl:[91,131,34] },
  { t: 2833, fr:[99,70,85], fl:[81,110,96], br:[89,52,151], bl:[91,131,34] },
  { t: 2875, fr:[100,70,85], fl:[80,110,96], br:[89,52,151], bl:[91,131,34] },
  { t: 2917, fr:[100,70,84], fl:[80,110,97], br:[89,52,151], bl:[91,131,34] },
  { t: 2958, fr:[101,70,83], fl:[79,110,98], br:[89,52,151], bl:[91,131,34] },
  { t: 3000, fr:[101,70,82], fl:[79,110,99], br:[89,52,151], bl:[91,131,34] },
  { t: 3042, fr:[102,70,80], fl:[78,110,101], br:[89,52,151], bl:[91,131,34] },
  { t: 3083, fr:[103,70,79], fl:[77,110,102], br:[89,52,151], bl:[91,131,34] },
  { t: 3125, fr:[103,70,78], fl:[77,110,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3167, fr:[104,72,78], fl:[76,108,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3208, fr:[105,73,78], fl:[75,107,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3250, fr:[105,75,78], fl:[75,105,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3292, fr:[106,77,78], fl:[74,103,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3333, fr:[106,79,78], fl:[74,101,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3375, fr:[107,82,78], fl:[73,98,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3417, fr:[107,84,78], fl:[73,96,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3458, fr:[107,87,78], fl:[73,93,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3500, fr:[108,90,78], fl:[72,90,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3542, fr:[108,92,78], fl:[72,88,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3583, fr:[108,95,78], fl:[72,85,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3625, fr:[109,98,78], fl:[71,82,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3667, fr:[109,100,78], fl:[71,80,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3708, fr:[109,102,78], fl:[71,78,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3750, fr:[109,105,78], fl:[71,75,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3792, fr:[109,106,78], fl:[71,74,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3833, fr:[109,108,78], fl:[71,72,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3875, fr:[110,109,78], fl:[70,71,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3917, fr:[110,110,78], fl:[70,70,103], br:[89,52,151], bl:[91,131,34] },
  { t: 3958, fr:[110,110,78], fl:[70,70,103], br:[89,52,151], bl:[91,131,34] },
  { t: 4000, fr:[110,111,78], fl:[70,69,103], br:[89,52,151], bl:[91,131,34] },
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
