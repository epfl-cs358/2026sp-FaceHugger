// FaceHugger clip: wave
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
const CLIP_NAME = "wave";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,132,34], fl:[89,50,146], br:[89,51,151], bl:[90,132,34] },
  { t: 83, fr:[91,132,35], fl:[88,52,144], br:[89,51,151], bl:[90,133,35] },
  { t: 125, fr:[92,133,36], fl:[88,52,141], br:[89,50,150], bl:[90,133,35] },
  { t: 167, fr:[92,133,36], fl:[88,53,137], br:[89,50,150], bl:[89,134,36] },
  { t: 208, fr:[92,134,37], fl:[88,52,132], br:[89,49,149], bl:[89,134,37] },
  { t: 250, fr:[93,134,38], fl:[87,51,125], br:[90,48,149], bl:[89,135,38] },
  { t: 292, fr:[93,135,39], fl:[87,47,113], br:[90,48,148], bl:[88,135,38] },
  { t: 333, fr:[93,135,40], fl:[87,43,104], br:[90,47,148], bl:[88,136,39] },
  { t: 375, fr:[94,136,41], fl:[87,45,104], br:[90,47,147], bl:[88,136,40] },
  { t: 417, fr:[94,136,43], fl:[87,47,104], br:[90,46,147], bl:[87,137,40] },
  { t: 458, fr:[94,137,44], fl:[86,50,104], br:[90,45,146], bl:[87,137,41] },
  { t: 500, fr:[95,138,45], fl:[86,53,104], br:[90,45,145], bl:[86,138,42] },
  { t: 542, fr:[95,138,46], fl:[86,56,104], br:[90,44,145], bl:[86,139,43] },
  { t: 583, fr:[95,139,47], fl:[86,59,104], br:[90,43,144], bl:[86,139,44] },
  { t: 625, fr:[96,140,49], fl:[86,62,104], br:[90,43,144], bl:[85,140,45] },
  { t: 667, fr:[96,140,50], fl:[86,65,104], br:[90,42,143], bl:[85,140,46] },
  { t: 708, fr:[96,141,51], fl:[85,68,103], br:[90,41,142], bl:[85,141,47] },
  { t: 750, fr:[97,142,53], fl:[85,70,103], br:[90,41,142], bl:[84,142,47] },
  { t: 792, fr:[97,142,53], fl:[85,72,103], br:[90,41,142], bl:[84,142,47] },
  { t: 833, fr:[97,142,53], fl:[85,74,103], br:[90,41,142], bl:[84,142,47] },
  { t: 875, fr:[98,142,54], fl:[84,76,103], br:[90,40,142], bl:[83,142,47] },
  { t: 917, fr:[98,142,54], fl:[84,78,103], br:[90,40,143], bl:[83,142,47] },
  { t: 958, fr:[98,142,54], fl:[84,80,103], br:[90,40,143], bl:[82,141,47] },
  { t: 1000, fr:[99,142,54], fl:[83,82,103], br:[90,40,143], bl:[82,141,47] },
  { t: 1042, fr:[99,142,55], fl:[83,84,103], br:[91,40,143], bl:[82,141,47] },
  { t: 1083, fr:[99,142,55], fl:[82,86,103], br:[91,40,144], bl:[81,141,47] },
  { t: 1125, fr:[100,142,55], fl:[82,88,103], br:[91,40,144], bl:[81,141,47] },
  { t: 1167, fr:[100,142,56], fl:[81,91,103], br:[91,40,144], bl:[81,141,46] },
  { t: 1208, fr:[100,142,56], fl:[81,93,103], br:[91,39,144], bl:[80,141,46] },
  { t: 1250, fr:[101,142,56], fl:[80,95,103], br:[91,39,145], bl:[80,141,46] },
  { t: 1292, fr:[101,142,57], fl:[80,97,103], br:[91,39,145], bl:[80,141,46] },
  { t: 1333, fr:[101,142,57], fl:[79,99,103], br:[91,39,145], bl:[79,141,46] },
  { t: 1375, fr:[101,142,57], fl:[79,101,103], br:[91,39,145], bl:[79,141,46] },
  { t: 1417, fr:[102,142,58], fl:[78,103,103], br:[91,39,145], bl:[79,141,46] },
  { t: 1458, fr:[102,142,58], fl:[77,104,103], br:[91,38,146], bl:[78,141,46] },
  { t: 1500, fr:[102,142,59], fl:[77,106,103], br:[91,38,146], bl:[78,141,46] },
  { t: 1542, fr:[102,142,57], fl:[77,106,103], br:[91,39,146], bl:[78,141,45] },
  { t: 1583, fr:[102,141,56], fl:[78,106,103], br:[91,39,147], bl:[77,140,45] },
  { t: 1625, fr:[101,141,55], fl:[79,107,103], br:[90,39,147], bl:[77,140,44] },
  { t: 1667, fr:[101,140,54], fl:[80,107,103], br:[90,40,148], bl:[77,140,43] },
  { t: 1708, fr:[101,139,53], fl:[81,107,103], br:[90,40,149], bl:[76,139,43] },
  { t: 1750, fr:[101,139,52], fl:[82,107,103], br:[89,40,149], bl:[76,139,42] },
  { t: 1792, fr:[100,138,51], fl:[83,107,103], br:[89,41,150], bl:[76,138,41] },
  { t: 1833, fr:[100,138,50], fl:[84,107,103], br:[89,41,150], bl:[76,138,41] },
  { t: 1875, fr:[100,137,50], fl:[84,107,103], br:[88,41,151], bl:[75,138,40] },
  { t: 1917, fr:[100,137,49], fl:[85,107,103], br:[88,42,151], bl:[75,137,40] },
  { t: 1958, fr:[99,136,48], fl:[86,107,103], br:[88,42,152], bl:[75,137,39] },
  { t: 2000, fr:[99,136,47], fl:[87,107,103], br:[87,42,152], bl:[74,137,38] },
  { t: 2042, fr:[99,136,46], fl:[87,107,103], br:[87,43,153], bl:[74,136,38] },
  { t: 2083, fr:[98,135,46], fl:[88,108,103], br:[87,43,153], bl:[74,136,37] },
  { t: 2125, fr:[98,135,45], fl:[88,108,103], br:[86,43,154], bl:[74,136,37] },
  { t: 2167, fr:[98,134,44], fl:[89,108,103], br:[86,44,154], bl:[73,135,36] },
  { t: 2208, fr:[98,134,43], fl:[89,108,103], br:[85,44,155], bl:[73,135,36] },
  { t: 2250, fr:[97,134,43], fl:[89,108,103], br:[85,44,155], bl:[73,134,35] },
  { t: 2292, fr:[97,133,42], fl:[90,109,103], br:[85,45,156], bl:[72,134,35] },
  { t: 2333, fr:[97,133,41], fl:[90,109,103], br:[84,45,156], bl:[72,134,34] },
  { t: 2375, fr:[96,132,41], fl:[91,109,103], br:[84,45,157], bl:[72,133,34] },
  { t: 2417, fr:[96,132,40], fl:[91,109,103], br:[84,46,157], bl:[72,133,33] },
  { t: 2458, fr:[96,132,39], fl:[91,109,103], br:[83,46,158], bl:[71,133,33] },
  { t: 2500, fr:[96,131,39], fl:[92,110,103], br:[83,46,158], bl:[71,132,32] },
  { t: 2542, fr:[96,131,39], fl:[91,109,103], br:[83,46,158], bl:[71,132,32] },
  { t: 2583, fr:[96,131,39], fl:[91,109,103], br:[84,46,158], bl:[72,132,32] },
  { t: 2625, fr:[97,131,39], fl:[91,109,103], br:[84,46,158], bl:[72,132,32] },
  { t: 2667, fr:[97,131,39], fl:[90,109,103], br:[85,46,158], bl:[72,132,32] },
  { t: 2708, fr:[97,131,39], fl:[90,109,103], br:[85,46,158], bl:[73,132,32] },
  { t: 2750, fr:[98,131,39], fl:[90,109,103], br:[86,46,158], bl:[73,133,32] },
  { t: 2792, fr:[98,131,39], fl:[89,109,103], br:[86,46,158], bl:[73,133,32] },
  { t: 2833, fr:[98,131,39], fl:[89,109,103], br:[87,46,158], bl:[74,133,32] },
  { t: 2875, fr:[99,131,40], fl:[89,109,103], br:[87,46,158], bl:[74,133,32] },
  { t: 2917, fr:[99,131,40], fl:[88,108,103], br:[87,46,158], bl:[75,133,32] },
  { t: 2958, fr:[100,131,40], fl:[88,108,103], br:[88,46,159], bl:[75,133,32] },
  { t: 3000, fr:[100,131,40], fl:[88,108,103], br:[88,46,159], bl:[75,133,32] },
  { t: 3042, fr:[100,131,40], fl:[87,108,103], br:[89,46,159], bl:[76,133,32] },
  { t: 3083, fr:[101,131,40], fl:[86,108,103], br:[89,46,159], bl:[76,133,32] },
  { t: 3125, fr:[101,131,40], fl:[85,108,103], br:[90,46,159], bl:[76,133,32] },
  { t: 3167, fr:[101,131,40], fl:[85,108,103], br:[90,46,159], bl:[77,133,32] },
  { t: 3208, fr:[102,131,40], fl:[84,108,103], br:[90,46,159], bl:[77,133,32] },
  { t: 3250, fr:[102,131,41], fl:[83,108,103], br:[91,46,159], bl:[77,133,32] },
  { t: 3292, fr:[102,131,41], fl:[82,108,103], br:[91,46,159], bl:[78,133,32] },
  { t: 3333, fr:[103,131,41], fl:[81,108,103], br:[92,46,159], bl:[78,133,32] },
  { t: 3375, fr:[103,131,41], fl:[81,108,103], br:[92,45,159], bl:[79,133,32] },
  { t: 3417, fr:[103,131,41], fl:[80,108,103], br:[93,45,159], bl:[79,133,32] },
  { t: 3458, fr:[104,131,41], fl:[79,108,103], br:[93,45,159], bl:[79,133,32] },
  { t: 3500, fr:[104,131,41], fl:[78,108,103], br:[94,45,159], bl:[80,133,32] },
  { t: 3542, fr:[104,131,41], fl:[78,108,103], br:[94,45,159], bl:[80,133,32] },
  { t: 3583, fr:[104,131,41], fl:[77,108,103], br:[94,45,159], bl:[80,133,32] },
  { t: 3625, fr:[105,131,41], fl:[77,108,103], br:[94,45,159], bl:[80,133,32] },
  { t: 3667, fr:[105,131,41], fl:[76,108,103], br:[95,45,159], bl:[80,133,32] },
  { t: 3708, fr:[105,131,42], fl:[76,108,103], br:[95,45,159], bl:[81,133,32] },
  { t: 3750, fr:[105,131,42], fl:[75,108,103], br:[95,45,159], bl:[81,133,32] },
  { t: 3792, fr:[105,131,42], fl:[75,108,103], br:[95,45,159], bl:[81,133,32] },
  { t: 3833, fr:[106,131,42], fl:[74,108,103], br:[96,45,159], bl:[81,133,32] },
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
