// FaceHugger clip: imbalance flip
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
const CLIP_NAME = "imbalance flip";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 83, fr:[91,131,33], fl:[89,48,148], br:[89,52,152], bl:[91,132,34] },
  { t: 125, fr:[91,131,33], fl:[89,48,148], br:[89,52,152], bl:[91,132,34] },
  { t: 167, fr:[91,131,33], fl:[89,48,148], br:[89,52,152], bl:[91,132,34] },
  { t: 208, fr:[91,131,32], fl:[89,48,148], br:[89,52,152], bl:[91,132,34] },
  { t: 250, fr:[91,131,32], fl:[89,48,147], br:[89,52,152], bl:[91,132,34] },
  { t: 292, fr:[90,131,32], fl:[89,48,147], br:[90,53,152], bl:[91,132,35] },
  { t: 333, fr:[90,131,32], fl:[89,47,147], br:[90,53,153], bl:[91,133,35] },
  { t: 375, fr:[90,130,31], fl:[89,47,146], br:[90,53,153], bl:[91,133,36] },
  { t: 417, fr:[90,130,31], fl:[89,46,145], br:[90,53,153], bl:[91,134,36] },
  { t: 458, fr:[90,130,31], fl:[89,46,145], br:[90,54,154], bl:[91,134,37] },
  { t: 500, fr:[90,129,30], fl:[88,45,144], br:[90,54,154], bl:[92,135,38] },
  { t: 542, fr:[90,129,30], fl:[88,45,143], br:[90,54,155], bl:[92,135,39] },
  { t: 583, fr:[90,129,29], fl:[87,44,142], br:[90,55,155], bl:[93,136,39] },
  { t: 625, fr:[90,128,29], fl:[86,43,141], br:[90,55,156], bl:[94,137,40] },
  { t: 667, fr:[90,128,28], fl:[85,43,140], br:[90,56,156], bl:[95,137,41] },
  { t: 708, fr:[90,127,27], fl:[83,42,139], br:[90,56,157], bl:[97,138,43] },
  { t: 750, fr:[90,127,27], fl:[82,41,138], br:[90,57,157], bl:[98,139,44] },
  { t: 792, fr:[91,127,26], fl:[80,40,136], br:[90,57,158], bl:[100,140,45] },
  { t: 833, fr:[91,126,25], fl:[78,39,135], br:[90,58,159], bl:[102,141,47] },
  { t: 875, fr:[91,126,24], fl:[76,38,133], br:[91,58,160], bl:[104,142,49] },
  { t: 917, fr:[92,126,24], fl:[73,37,131], br:[91,59,160], bl:[107,143,51] },
  { t: 958, fr:[92,125,23], fl:[71,36,128], br:[91,59,161], bl:[109,144,54] },
  { t: 1000, fr:[92,125,22], fl:[69,35,125], br:[91,60,162], bl:[111,145,57] },
  { t: 1042, fr:[93,125,22], fl:[66,33,121], br:[91,61,162], bl:[114,147,60] },
  { t: 1083, fr:[94,125,21], fl:[64,30,115], br:[91,61,163], bl:[116,150,66] },
  { t: 1125, fr:[94,125,21], fl:[62,30,114], br:[90,61,164], bl:[118,150,67] },
  { t: 1167, fr:[95,126,21], fl:[60,30,113], br:[90,62,164], bl:[120,150,68] },
  { t: 1208, fr:[96,126,22], fl:[58,30,113], br:[90,62,164], bl:[122,150,69] },
  { t: 1250, fr:[96,127,22], fl:[56,30,112], br:[90,62,164], bl:[124,150,70] },
  { t: 1292, fr:[97,128,23], fl:[55,30,112], br:[90,62,164], bl:[126,150,70] },
  { t: 1333, fr:[98,129,24], fl:[55,30,111], br:[90,61,164], bl:[127,150,71] },
  { t: 1375, fr:[99,130,26], fl:[55,30,111], br:[90,61,164], bl:[128,150,71] },
  { t: 1417, fr:[99,131,27], fl:[55,30,111], br:[90,61,163], bl:[129,150,71] },
  { t: 1458, fr:[100,132,29], fl:[55,30,112], br:[90,61,163], bl:[130,150,71] },
  { t: 1500, fr:[101,134,31], fl:[55,30,112], br:[90,60,162], bl:[131,150,71] },
  { t: 1542, fr:[101,135,33], fl:[55,30,112], br:[90,60,161], bl:[132,150,71] },
  { t: 1583, fr:[102,137,35], fl:[55,30,112], br:[90,59,161], bl:[132,150,71] },
  { t: 1625, fr:[102,138,38], fl:[55,30,113], br:[89,58,159], bl:[133,150,71] },
  { t: 1667, fr:[103,140,41], fl:[55,30,113], br:[89,57,158], bl:[134,150,71] },
  { t: 1708, fr:[103,142,44], fl:[55,30,113], br:[88,55,157], bl:[135,150,70] },
  { t: 1750, fr:[103,143,47], fl:[55,30,113], br:[88,54,155], bl:[135,150,70] },
  { t: 1792, fr:[104,145,50], fl:[55,30,113], br:[87,52,153], bl:[136,150,70] },
  { t: 1833, fr:[104,147,53], fl:[55,30,113], br:[86,50,151], bl:[136,150,70] },
  { t: 1875, fr:[105,149,57], fl:[55,30,113], br:[85,48,148], bl:[137,150,70] },
  { t: 1917, fr:[105,150,58], fl:[55,30,113], br:[84,46,146], bl:[137,150,70] },
  { t: 1958, fr:[106,150,58], fl:[55,30,113], br:[83,45,143], bl:[137,150,70] },
  { t: 2000, fr:[106,150,58], fl:[55,30,113], br:[82,43,140], bl:[137,150,70] },
  { t: 2042, fr:[107,150,58], fl:[55,30,113], br:[80,41,138], bl:[137,150,70] },
  { t: 2083, fr:[107,150,58], fl:[55,30,113], br:[79,39,135], bl:[137,150,70] },
  { t: 2125, fr:[108,150,58], fl:[55,30,113], br:[78,36,131], bl:[137,150,70] },
  { t: 2167, fr:[108,150,58], fl:[55,30,113], br:[76,34,127], bl:[137,150,70] },
  { t: 2208, fr:[109,150,58], fl:[55,30,113], br:[75,33,126], bl:[137,150,70] },
  { t: 2250, fr:[109,150,58], fl:[55,30,113], br:[74,33,126], bl:[137,150,70] },
  { t: 2292, fr:[110,150,58], fl:[55,30,113], br:[73,33,127], bl:[137,150,70] },
  { t: 2333, fr:[110,150,58], fl:[55,30,113], br:[72,33,127], bl:[137,150,70] },
  { t: 2375, fr:[110,150,58], fl:[55,30,113], br:[71,33,127], bl:[137,150,70] },
  { t: 2417, fr:[110,150,58], fl:[55,30,113], br:[70,33,127], bl:[137,150,70] },
  { t: 2458, fr:[110,150,58], fl:[55,30,113], br:[70,33,127], bl:[137,150,70] },
  { t: 2500, fr:[110,150,58], fl:[55,30,113], br:[70,33,127], bl:[137,150,70] },
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
