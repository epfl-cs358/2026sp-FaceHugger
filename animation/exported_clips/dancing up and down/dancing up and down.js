// FaceHugger clip: dancing up and down
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
const CLIP_NAME = "dancing up and down";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,146,56], fl:[89,34,125], br:[89,38,128], bl:[91,146,57] },
  { t: 42, fr:[92,143,51], fl:[91,37,130], br:[91,41,133], bl:[92,143,52] },
  { t: 83, fr:[93,140,47], fl:[92,40,135], br:[92,43,138], bl:[93,140,48] },
  { t: 125, fr:[95,138,43], fl:[93,42,139], br:[93,46,142], bl:[95,138,44] },
  { t: 167, fr:[96,135,40], fl:[95,44,142], br:[95,48,146], bl:[96,135,41] },
  { t: 208, fr:[97,133,37], fl:[96,46,146], br:[96,50,149], bl:[97,133,38] },
  { t: 250, fr:[99,131,34], fl:[97,48,149], br:[97,52,152], bl:[99,131,35] },
  { t: 292, fr:[100,129,32], fl:[99,50,151], br:[99,53,155], bl:[100,129,32] },
  { t: 333, fr:[101,127,29], fl:[100,52,154], br:[100,55,157], bl:[101,127,30] },
  { t: 375, fr:[102,126,27], fl:[101,54,157], br:[101,57,160], bl:[102,126,28] },
  { t: 417, fr:[104,124,25], fl:[103,55,159], br:[103,59,162], bl:[104,124,26] },
  { t: 458, fr:[105,122,23], fl:[104,57,161], br:[104,60,165], bl:[105,122,24] },
  { t: 500, fr:[106,120,21], fl:[105,59,163], br:[105,62,167], bl:[106,120,22] },
  { t: 542, fr:[107,121,22], fl:[106,58,162], br:[106,61,166], bl:[107,121,23] },
  { t: 583, fr:[109,121,24], fl:[108,57,161], br:[108,61,165], bl:[109,121,25] },
  { t: 625, fr:[110,122,25], fl:[109,57,160], br:[109,60,163], bl:[110,122,26] },
  { t: 667, fr:[111,123,27], fl:[110,56,159], br:[110,59,162], bl:[111,123,28] },
  { t: 708, fr:[112,124,29], fl:[112,55,158], br:[112,58,161], bl:[112,124,29] },
  { t: 750, fr:[113,125,30], fl:[113,54,156], br:[113,57,160], bl:[113,125,31] },
  { t: 792, fr:[115,126,32], fl:[114,53,155], br:[114,57,158], bl:[115,126,33] },
  { t: 833, fr:[116,127,34], fl:[115,53,154], br:[115,56,157], bl:[116,127,34] },
  { t: 875, fr:[117,128,36], fl:[116,52,152], br:[116,55,155], bl:[117,128,36] },
  { t: 917, fr:[117,128,37], fl:[116,51,151], br:[116,54,154], bl:[117,128,38] },
  { t: 958, fr:[117,129,39], fl:[116,50,149], br:[116,53,153], bl:[117,129,39] },
  { t: 1000, fr:[117,130,40], fl:[116,49,148], br:[116,52,151], bl:[117,130,41] },
  { t: 1042, fr:[117,129,39], fl:[116,50,149], br:[116,53,153], bl:[117,129,39] },
  { t: 1083, fr:[117,128,37], fl:[116,51,151], br:[116,54,154], bl:[117,128,38] },
  { t: 1125, fr:[117,128,36], fl:[116,52,152], br:[116,55,155], bl:[117,128,36] },
  { t: 1167, fr:[116,127,34], fl:[115,53,154], br:[115,56,157], bl:[116,127,34] },
  { t: 1208, fr:[114,126,32], fl:[114,53,155], br:[114,57,158], bl:[114,126,32] },
  { t: 1250, fr:[113,125,30], fl:[112,54,156], br:[112,57,160], bl:[113,125,31] },
  { t: 1292, fr:[112,124,28], fl:[111,55,158], br:[111,58,161], bl:[112,124,29] },
  { t: 1333, fr:[110,123,27], fl:[109,56,159], br:[109,59,162], bl:[110,123,27] },
  { t: 1375, fr:[109,122,25], fl:[108,56,160], br:[108,60,164], bl:[109,122,26] },
  { t: 1417, fr:[107,122,23], fl:[106,57,161], br:[106,61,165], bl:[107,122,24] },
  { t: 1458, fr:[106,121,22], fl:[105,58,162], br:[105,61,166], bl:[106,121,23] },
  { t: 1500, fr:[104,120,20], fl:[103,59,163], br:[103,62,167], bl:[104,120,21] },
  { t: 1542, fr:[103,121,21], fl:[102,58,163], br:[102,61,166], bl:[103,121,22] },
  { t: 1583, fr:[102,122,22], fl:[100,57,161], br:[100,61,165], bl:[102,122,23] },
  { t: 1625, fr:[100,123,23], fl:[99,56,160], br:[99,60,164], bl:[100,123,23] },
  { t: 1667, fr:[99,124,24], fl:[97,56,159], br:[97,59,163], bl:[99,124,24] },
  { t: 1708, fr:[97,125,24], fl:[96,55,158], br:[96,58,161], bl:[97,125,25] },
  { t: 1750, fr:[96,126,25], fl:[94,54,157], br:[94,57,160], bl:[96,126,26] },
  { t: 1792, fr:[94,127,26], fl:[93,53,155], br:[93,57,159], bl:[94,127,27] },
  { t: 1833, fr:[93,128,27], fl:[91,52,154], br:[91,56,157], bl:[93,128,28] },
  { t: 1875, fr:[91,128,28], fl:[90,52,153], br:[90,55,156], bl:[91,128,29] },
  { t: 1917, fr:[89,129,30], fl:[88,51,151], br:[88,54,155], bl:[89,129,30] },
  { t: 1958, fr:[88,130,31], fl:[87,50,150], br:[87,53,153], bl:[88,130,31] },
  { t: 2000, fr:[86,131,32], fl:[85,49,148], br:[85,52,151], bl:[86,131,33] },
  { t: 2042, fr:[85,130,30], fl:[84,50,149], br:[84,53,153], bl:[85,130,31] },
  { t: 2083, fr:[83,130,29], fl:[82,51,150], br:[82,54,154], bl:[83,130,30] },
  { t: 2125, fr:[82,129,28], fl:[81,52,151], br:[81,55,155], bl:[82,129,28] },
  { t: 2167, fr:[80,128,26], fl:[79,53,152], br:[79,56,156], bl:[80,128,27] },
  { t: 2208, fr:[79,127,25], fl:[78,54,153], br:[78,57,156], bl:[79,127,26] },
  { t: 2250, fr:[77,126,24], fl:[76,55,154], br:[76,58,157], bl:[77,126,25] },
  { t: 2292, fr:[76,125,23], fl:[75,56,155], br:[75,59,158], bl:[76,125,24] },
  { t: 2333, fr:[74,124,22], fl:[73,57,155], br:[73,60,159], bl:[74,124,23] },
  { t: 2375, fr:[73,124,21], fl:[72,58,156], br:[72,61,160], bl:[73,124,21] },
  { t: 2417, fr:[71,123,20], fl:[70,59,157], br:[70,62,160], bl:[71,123,20] },
  { t: 2458, fr:[70,122,19], fl:[69,59,157], br:[69,63,161], bl:[70,122,19] },
  { t: 2500, fr:[68,121,18], fl:[68,60,158], br:[68,64,161], bl:[68,121,19] },
  { t: 2542, fr:[67,122,19], fl:[66,60,156], br:[66,63,160], bl:[67,122,20] },
  { t: 2583, fr:[66,122,20], fl:[65,59,155], br:[65,62,158], bl:[66,122,21] },
  { t: 2625, fr:[64,123,22], fl:[64,58,153], br:[64,61,156], bl:[64,123,22] },
  { t: 2667, fr:[63,124,23], fl:[62,57,151], br:[62,60,155], bl:[63,124,24] },
  { t: 2708, fr:[61,125,24], fl:[61,56,149], br:[61,60,153], bl:[61,125,25] },
  { t: 2750, fr:[60,125,26], fl:[60,55,147], br:[60,59,151], bl:[60,125,27] },
  { t: 2792, fr:[59,126,27], fl:[59,54,145], br:[59,58,149], bl:[59,126,28] },
  { t: 2833, fr:[57,127,29], fl:[57,53,143], br:[57,57,147], bl:[57,127,30] },
  { t: 2875, fr:[56,128,31], fl:[56,52,141], br:[56,56,144], bl:[56,128,31] },
  { t: 2917, fr:[56,129,32], fl:[56,51,140], br:[56,54,143], bl:[56,129,33] },
  { t: 2958, fr:[56,130,33], fl:[56,50,138], br:[56,53,141], bl:[56,130,34] },
  { t: 3000, fr:[56,131,35], fl:[56,49,136], br:[56,52,140], bl:[56,131,35] },
  { t: 3042, fr:[56,130,33], fl:[56,50,138], br:[56,53,141], bl:[56,130,34] },
  { t: 3083, fr:[56,129,32], fl:[56,51,140], br:[56,54,143], bl:[56,129,33] },
  { t: 3125, fr:[56,128,31], fl:[56,52,141], br:[56,56,144], bl:[56,128,31] },
  { t: 3167, fr:[58,127,29], fl:[58,53,143], br:[58,57,147], bl:[58,127,30] },
  { t: 3208, fr:[59,126,27], fl:[59,54,146], br:[59,58,149], bl:[59,126,28] },
  { t: 3250, fr:[61,126,26], fl:[60,55,148], br:[60,59,151], bl:[61,126,26] },
  { t: 3292, fr:[62,125,24], fl:[62,56,150], br:[62,60,153], bl:[62,125,25] },
  { t: 3333, fr:[64,124,23], fl:[63,57,152], br:[63,60,155], bl:[64,124,24] },
  { t: 3375, fr:[65,123,22], fl:[65,58,154], br:[65,61,157], bl:[65,123,22] },
  { t: 3417, fr:[67,123,20], fl:[66,59,155], br:[66,62,159], bl:[67,123,21] },
  { t: 3458, fr:[68,122,19], fl:[68,60,157], br:[68,63,160], bl:[68,122,20] },
  { t: 3500, fr:[70,121,18], fl:[69,60,159], br:[69,64,162], bl:[70,121,18] },
  { t: 3542, fr:[72,123,20], fl:[71,58,157], br:[71,62,160], bl:[72,123,21] },
  { t: 3583, fr:[73,125,22], fl:[72,57,155], br:[72,60,158], bl:[73,125,23] },
  { t: 3625, fr:[75,126,24], fl:[74,55,153], br:[74,58,156], bl:[75,126,25] },
  { t: 3667, fr:[77,128,27], fl:[76,53,151], br:[76,56,154], bl:[77,128,28] },
  { t: 3708, fr:[79,130,30], fl:[77,51,149], br:[77,54,152], bl:[79,130,30] },
  { t: 3750, fr:[80,132,32], fl:[79,49,146], br:[79,52,150], bl:[80,132,33] },
  { t: 3792, fr:[82,134,35], fl:[81,47,143], br:[81,50,147], bl:[82,134,36] },
  { t: 3833, fr:[84,136,39], fl:[82,45,141], br:[82,48,144], bl:[84,136,39] },
  { t: 3875, fr:[85,138,42], fl:[84,42,137], br:[84,46,141], bl:[85,138,43] },
  { t: 3917, fr:[87,140,46], fl:[86,40,134], br:[86,43,137], bl:[87,140,47] },
  { t: 3958, fr:[89,143,51], fl:[88,37,130], br:[88,41,133], bl:[89,143,51] },
  { t: 4000, fr:[91,146,56], fl:[89,34,125], br:[89,38,128], bl:[91,146,57] },
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
