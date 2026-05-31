// FaceHugger clip: roll-ish
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
const CLIP_NAME = "roll-ish";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[90,131,31], fl:[90,49,150], br:[90,51,151], bl:[90,132,34] },
  { t: 83, fr:[87,130,28], fl:[93,50,153], br:[92,49,150], bl:[88,135,35] },
  { t: 125, fr:[84,129,23], fl:[96,51,158], br:[94,46,148], bl:[86,138,37] },
  { t: 167, fr:[79,128,20], fl:[101,52,161], br:[97,42,145], bl:[83,141,40] },
  { t: 208, fr:[75,127,18], fl:[105,53,163], br:[98,38,139], bl:[82,146,46] },
  { t: 250, fr:[71,127,18], fl:[109,53,163], br:[98,33,131], bl:[82,150,54] },
  { t: 292, fr:[69,125,18], fl:[111,55,163], br:[98,33,130], bl:[82,150,55] },
  { t: 333, fr:[69,122,18], fl:[111,58,163], br:[100,33,131], bl:[80,150,54] },
  { t: 375, fr:[70,119,18], fl:[110,61,163], br:[104,33,133], bl:[76,150,52] },
  { t: 417, fr:[71,115,18], fl:[109,65,163], br:[108,33,134], bl:[72,150,51] },
  { t: 458, fr:[71,112,18], fl:[109,68,163], br:[113,33,134], bl:[67,150,51] },
  { t: 500, fr:[71,109,18], fl:[109,71,163], br:[119,33,135], bl:[61,150,50] },
  { t: 542, fr:[71,106,18], fl:[109,74,163], br:[125,33,135], bl:[55,150,50] },
  { t: 583, fr:[71,105,19], fl:[109,75,162], br:[129,33,134], bl:[55,150,51] },
  { t: 625, fr:[71,104,21], fl:[109,76,160], br:[133,33,134], bl:[55,150,51] },
  { t: 667, fr:[71,103,23], fl:[109,77,158], br:[136,33,133], bl:[55,150,51] },
  { t: 708, fr:[71,102,24], fl:[109,78,157], br:[138,33,133], bl:[55,150,51] },
  { t: 750, fr:[71,102,26], fl:[109,78,155], br:[138,33,133], bl:[55,150,51] },
  { t: 792, fr:[71,101,26], fl:[109,79,155], br:[138,33,133], bl:[55,150,51] },
  { t: 833, fr:[72,100,26], fl:[108,80,155], br:[136,33,134], bl:[55,150,51] },
  { t: 875, fr:[72,100,26], fl:[108,80,155], br:[134,33,135], bl:[55,150,50] },
  { t: 917, fr:[72,99,25], fl:[108,81,156], br:[131,33,135], bl:[55,150,50] },
  { t: 958, fr:[71,99,24], fl:[109,81,157], br:[126,33,136], bl:[55,150,49] },
  { t: 1000, fr:[71,99,23], fl:[109,81,158], br:[121,33,137], bl:[59,150,48] },
  { t: 1042, fr:[71,100,21], fl:[109,80,160], br:[115,33,137], bl:[65,150,48] },
  { t: 1083, fr:[70,100,20], fl:[110,80,161], br:[109,33,136], bl:[71,150,49] },
  { t: 1125, fr:[70,102,19], fl:[110,78,162], br:[103,33,135], bl:[77,150,50] },
  { t: 1167, fr:[69,103,19], fl:[111,77,162], br:[98,33,132], bl:[82,150,53] },
  { t: 1208, fr:[68,105,18], fl:[112,75,163], br:[93,40,140], bl:[87,143,45] },
  { t: 1250, fr:[68,107,18], fl:[112,73,163], br:[89,48,147], bl:[91,136,38] },
  { t: 1292, fr:[67,109,18], fl:[113,71,163], br:[86,54,152], bl:[94,129,33] },
  { t: 1333, fr:[66,112,18], fl:[114,68,163], br:[84,60,155], bl:[96,123,30] },
  { t: 1375, fr:[65,114,19], fl:[115,66,162], br:[82,65,156], bl:[98,118,29] },
  { t: 1417, fr:[64,117,20], fl:[116,63,161], br:[80,69,156], bl:[100,114,29] },
  { t: 1458, fr:[63,119,21], fl:[117,61,160], br:[79,72,154], bl:[101,111,31] },
  { t: 1500, fr:[62,121,22], fl:[118,59,159], br:[78,75,151], bl:[102,109,34] },
  { t: 1542, fr:[61,123,24], fl:[119,57,157], br:[77,76,148], bl:[103,107,37] },
  { t: 1583, fr:[60,125,25], fl:[120,55,156], br:[76,76,145], bl:[104,107,40] },
  { t: 1625, fr:[60,127,26], fl:[120,53,155], br:[76,76,141], bl:[104,107,44] },
  { t: 1667, fr:[59,128,27], fl:[121,52,154], br:[75,76,138], bl:[105,107,47] },
  { t: 1708, fr:[59,128,28], fl:[121,52,153], br:[75,76,136], bl:[105,108,49] },
  { t: 1750, fr:[59,129,28], fl:[121,51,153], br:[75,76,135], bl:[105,108,50] },
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
