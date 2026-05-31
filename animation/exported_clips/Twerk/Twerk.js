// FaceHugger clip: Twerk
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
const CLIP_NAME = "Twerk";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[89,131,32], fl:[88,48,147], br:[88,53,152], bl:[90,132,34] },
  { t: 83, fr:[87,130,31], fl:[86,47,146], br:[88,54,153], bl:[89,133,35] },
  { t: 125, fr:[86,129,30], fl:[84,46,145], br:[87,54,154], bl:[88,134,36] },
  { t: 167, fr:[84,128,28], fl:[82,46,144], br:[86,55,154], bl:[87,135,37] },
  { t: 208, fr:[83,127,27], fl:[80,45,143], br:[85,56,155], bl:[86,137,39] },
  { t: 250, fr:[81,126,26], fl:[79,44,142], br:[84,57,155], bl:[85,138,40] },
  { t: 292, fr:[80,124,25], fl:[77,44,140], br:[83,57,156], bl:[84,139,41] },
  { t: 333, fr:[78,123,24], fl:[75,44,139], br:[82,58,156], bl:[83,140,43] },
  { t: 375, fr:[77,122,23], fl:[73,43,138], br:[82,58,156], bl:[82,142,44] },
  { t: 417, fr:[76,121,22], fl:[71,43,137], br:[81,59,157], bl:[81,143,46] },
  { t: 458, fr:[75,119,21], fl:[69,43,136], br:[80,59,157], bl:[80,144,48] },
  { t: 500, fr:[73,118,20], fl:[68,43,135], br:[79,60,157], bl:[79,146,50] },
  { t: 542, fr:[76,119,20], fl:[70,42,136], br:[81,60,158], bl:[82,145,49] },
  { t: 583, fr:[78,119,20], fl:[73,41,136], br:[84,61,159], bl:[85,145,48] },
  { t: 625, fr:[80,119,21], fl:[75,40,136], br:[86,61,160], bl:[88,144,48] },
  { t: 667, fr:[82,120,21], fl:[78,40,136], br:[88,62,161], bl:[91,143,47] },
  { t: 708, fr:[85,120,22], fl:[81,39,135], br:[91,62,162], bl:[94,143,47] },
  { t: 750, fr:[87,121,22], fl:[83,38,135], br:[93,63,162], bl:[97,142,46] },
  { t: 792, fr:[89,121,23], fl:[86,37,135], br:[95,63,163], bl:[99,141,46] },
  { t: 833, fr:[92,122,24], fl:[89,37,134], br:[98,63,163], bl:[102,140,46] },
  { t: 875, fr:[94,122,24], fl:[92,36,134], br:[100,64,164], bl:[105,140,46] },
  { t: 917, fr:[96,123,25], fl:[95,35,133], br:[102,64,164], bl:[107,139,46] },
  { t: 958, fr:[99,123,26], fl:[98,35,133], br:[104,65,164], bl:[110,138,46] },
  { t: 1000, fr:[101,124,27], fl:[101,34,132], br:[107,65,164], bl:[112,137,46] },
  { t: 1042, fr:[103,124,28], fl:[104,33,131], br:[109,66,165], bl:[115,137,47] },
  { t: 1083, fr:[100,123,27], fl:[100,34,132], br:[106,65,164], bl:[112,138,46] },
  { t: 1125, fr:[97,123,26], fl:[96,35,133], br:[103,65,164], bl:[108,139,46] },
  { t: 1167, fr:[94,122,24], fl:[92,36,134], br:[100,64,164], bl:[105,140,46] },
  { t: 1208, fr:[91,122,23], fl:[88,37,135], br:[97,63,163], bl:[101,141,46] },
  { t: 1250, fr:[88,121,22], fl:[85,38,135], br:[94,63,162], bl:[98,142,46] },
  { t: 1292, fr:[85,120,22], fl:[81,39,135], br:[91,62,162], bl:[94,143,47] },
  { t: 1333, fr:[82,120,21], fl:[77,40,136], br:[88,62,161], bl:[90,144,47] },
  { t: 1375, fr:[79,119,21], fl:[74,41,136], br:[85,61,160], bl:[86,144,48] },
  { t: 1417, fr:[76,119,20], fl:[71,42,136], br:[82,60,158], bl:[82,145,49] },
  { t: 1458, fr:[73,118,20], fl:[67,43,135], br:[79,60,157], bl:[78,146,50] },
  { t: 1500, fr:[70,118,20], fl:[64,43,135], br:[76,59,155], bl:[74,147,51] },
  { t: 1542, fr:[67,117,20], fl:[61,44,134], br:[73,58,154], bl:[70,147,52] },
  { t: 1583, fr:[70,118,20], fl:[64,44,135], br:[76,59,155], bl:[74,147,51] },
  { t: 1625, fr:[72,118,20], fl:[67,43,135], br:[78,60,157], bl:[77,146,50] },
  { t: 1667, fr:[75,118,20], fl:[70,42,136], br:[81,60,158], bl:[81,146,49] },
  { t: 1708, fr:[78,119,20], fl:[72,41,136], br:[84,61,159], bl:[85,145,48] },
  { t: 1750, fr:[80,119,21], fl:[75,40,136], br:[86,61,160], bl:[88,144,48] },
  { t: 1792, fr:[83,120,21], fl:[79,39,136], br:[89,62,161], bl:[92,143,47] },
  { t: 1833, fr:[86,120,22], fl:[82,38,135], br:[92,62,162], bl:[95,142,47] },
  { t: 1875, fr:[88,121,23], fl:[85,38,135], br:[95,63,163], bl:[98,141,46] },
  { t: 1917, fr:[91,122,23], fl:[89,37,135], br:[97,63,163], bl:[102,141,46] },
  { t: 1958, fr:[94,122,24], fl:[92,36,134], br:[100,64,164], bl:[105,140,46] },
  { t: 2000, fr:[97,123,25], fl:[96,35,133], br:[103,64,164], bl:[108,139,46] },
  { t: 2042, fr:[96,123,26], fl:[95,36,135], br:[102,63,163], bl:[106,138,45] },
  { t: 2083, fr:[96,124,26], fl:[95,38,137], br:[101,62,162], bl:[105,138,44] },
  { t: 2125, fr:[95,124,27], fl:[94,39,138], br:[100,61,161], bl:[103,138,43] },
  { t: 2167, fr:[95,125,27], fl:[94,40,139], br:[99,60,160], bl:[102,137,42] },
  { t: 2208, fr:[94,126,28], fl:[93,41,141], br:[98,59,159], bl:[100,136,41] },
  { t: 2250, fr:[94,127,28], fl:[92,43,142], br:[97,58,158], bl:[99,136,40] },
  { t: 2292, fr:[93,127,29], fl:[92,44,143], br:[95,57,157], bl:[98,135,39] },
  { t: 2333, fr:[93,128,30], fl:[91,45,144], br:[94,56,156], bl:[96,135,38] },
  { t: 2375, fr:[92,129,30], fl:[91,46,145], br:[93,55,155], bl:[95,134,37] },
  { t: 2417, fr:[92,130,31], fl:[90,47,146], br:[92,54,154], bl:[93,133,36] },
  { t: 2458, fr:[91,131,32], fl:[90,48,147], br:[91,53,153], bl:[92,132,35] },
  { t: 2500, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
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
