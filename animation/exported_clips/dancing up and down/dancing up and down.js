// FaceHugger clip: dancing up and down
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
const CLIP_NAME = "dancing up and down";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,140,61], fl:[89,31,117], br:[89,51,118], bl:[91,140,54] },
  { t: 42, fr:[92,137,56], fl:[91,34,122], br:[91,54,123], bl:[92,137,49] },
  { t: 83, fr:[93,134,52], fl:[92,37,127], br:[92,56,128], bl:[93,134,45] },
  { t: 125, fr:[95,132,48], fl:[93,39,131], br:[93,59,132], bl:[95,132,41] },
  { t: 167, fr:[96,129,45], fl:[95,41,134], br:[95,61,136], bl:[96,129,38] },
  { t: 208, fr:[97,127,42], fl:[96,43,138], br:[96,63,139], bl:[97,127,35] },
  { t: 250, fr:[99,125,39], fl:[97,45,141], br:[97,65,142], bl:[99,125,32] },
  { t: 292, fr:[100,123,37], fl:[99,47,143], br:[99,66,145], bl:[100,123,29] },
  { t: 333, fr:[101,121,34], fl:[100,49,146], br:[100,68,147], bl:[101,121,27] },
  { t: 375, fr:[102,120,32], fl:[101,51,149], br:[101,70,150], bl:[102,120,25] },
  { t: 417, fr:[104,118,30], fl:[103,52,151], br:[103,72,152], bl:[104,118,23] },
  { t: 458, fr:[105,116,28], fl:[104,54,153], br:[104,73,155], bl:[105,116,21] },
  { t: 500, fr:[106,114,26], fl:[105,56,155], br:[105,75,157], bl:[106,114,19] },
  { t: 542, fr:[107,115,27], fl:[106,55,154], br:[106,74,156], bl:[107,115,20] },
  { t: 583, fr:[109,115,29], fl:[108,54,153], br:[108,74,155], bl:[109,115,22] },
  { t: 625, fr:[110,116,30], fl:[109,54,152], br:[109,73,153], bl:[110,116,23] },
  { t: 667, fr:[111,117,32], fl:[110,53,151], br:[110,72,152], bl:[111,117,25] },
  { t: 708, fr:[112,118,34], fl:[112,52,150], br:[112,71,151], bl:[112,118,26] },
  { t: 750, fr:[113,119,35], fl:[113,51,148], br:[113,70,150], bl:[113,119,28] },
  { t: 792, fr:[115,120,37], fl:[114,50,147], br:[114,70,148], bl:[115,120,30] },
  { t: 833, fr:[116,121,39], fl:[115,50,146], br:[115,69,147], bl:[116,121,31] },
  { t: 875, fr:[117,122,41], fl:[116,49,144], br:[116,68,145], bl:[117,122,33] },
  { t: 917, fr:[117,122,42], fl:[116,48,143], br:[116,67,144], bl:[117,122,35] },
  { t: 958, fr:[117,123,44], fl:[116,47,141], br:[116,66,143], bl:[117,123,36] },
  { t: 1000, fr:[117,124,45], fl:[116,46,140], br:[116,65,141], bl:[117,124,38] },
  { t: 1042, fr:[117,123,44], fl:[116,47,141], br:[116,66,143], bl:[117,123,36] },
  { t: 1083, fr:[117,122,42], fl:[116,48,143], br:[116,67,144], bl:[117,122,35] },
  { t: 1125, fr:[117,122,41], fl:[116,49,144], br:[116,68,145], bl:[117,122,33] },
  { t: 1167, fr:[116,121,39], fl:[115,50,146], br:[115,69,147], bl:[116,121,31] },
  { t: 1208, fr:[114,120,37], fl:[114,50,147], br:[114,70,148], bl:[114,120,29] },
  { t: 1250, fr:[113,119,35], fl:[112,51,148], br:[112,70,150], bl:[113,119,28] },
  { t: 1292, fr:[112,118,33], fl:[111,52,150], br:[111,71,151], bl:[112,118,26] },
  { t: 1333, fr:[110,117,32], fl:[109,53,151], br:[109,72,152], bl:[110,117,24] },
  { t: 1375, fr:[109,116,30], fl:[108,53,152], br:[108,73,154], bl:[109,116,23] },
  { t: 1417, fr:[107,116,28], fl:[106,54,153], br:[106,74,155], bl:[107,116,21] },
  { t: 1458, fr:[106,115,27], fl:[105,55,154], br:[105,74,156], bl:[106,115,20] },
  { t: 1500, fr:[104,114,25], fl:[103,56,155], br:[103,75,157], bl:[104,114,18] },
  { t: 1542, fr:[103,115,26], fl:[102,55,155], br:[102,74,156], bl:[103,115,19] },
  { t: 1583, fr:[102,116,27], fl:[100,54,153], br:[100,74,155], bl:[102,116,20] },
  { t: 1625, fr:[100,117,28], fl:[99,53,152], br:[99,73,154], bl:[100,117,20] },
  { t: 1667, fr:[99,118,29], fl:[97,53,151], br:[97,72,153], bl:[99,118,21] },
  { t: 1708, fr:[97,119,29], fl:[96,52,150], br:[96,71,151], bl:[97,119,22] },
  { t: 1750, fr:[96,120,30], fl:[94,51,149], br:[94,70,150], bl:[96,120,23] },
  { t: 1792, fr:[94,121,31], fl:[93,50,147], br:[93,70,149], bl:[94,121,24] },
  { t: 1833, fr:[93,122,32], fl:[91,49,146], br:[91,69,147], bl:[93,122,25] },
  { t: 1875, fr:[91,122,33], fl:[90,49,145], br:[90,68,146], bl:[91,122,26] },
  { t: 1917, fr:[89,123,35], fl:[88,48,143], br:[88,67,145], bl:[89,123,27] },
  { t: 1958, fr:[88,124,36], fl:[87,47,142], br:[87,66,143], bl:[88,124,28] },
  { t: 2000, fr:[86,125,37], fl:[85,46,140], br:[85,65,141], bl:[86,125,30] },
  { t: 2042, fr:[85,124,35], fl:[84,47,141], br:[84,66,143], bl:[85,124,28] },
  { t: 2083, fr:[83,124,34], fl:[82,48,142], br:[82,67,144], bl:[83,124,27] },
  { t: 2125, fr:[82,123,33], fl:[81,49,143], br:[81,68,145], bl:[82,123,25] },
  { t: 2167, fr:[80,122,31], fl:[79,50,144], br:[79,69,146], bl:[80,122,24] },
  { t: 2208, fr:[79,121,30], fl:[78,51,145], br:[78,70,146], bl:[79,121,23] },
  { t: 2250, fr:[77,120,29], fl:[76,52,146], br:[76,71,147], bl:[77,120,22] },
  { t: 2292, fr:[76,119,28], fl:[75,53,147], br:[75,72,148], bl:[76,119,21] },
  { t: 2333, fr:[74,118,27], fl:[73,54,147], br:[73,73,149], bl:[74,118,20] },
  { t: 2375, fr:[73,118,26], fl:[72,55,148], br:[72,74,150], bl:[73,118,18] },
  { t: 2417, fr:[71,117,25], fl:[70,56,149], br:[70,75,150], bl:[71,117,17] },
  { t: 2458, fr:[70,116,24], fl:[69,56,149], br:[69,76,151], bl:[70,116,16] },
  { t: 2500, fr:[68,115,23], fl:[68,57,150], br:[68,77,151], bl:[68,115,16] },
  { t: 2542, fr:[67,116,24], fl:[66,57,148], br:[66,76,150], bl:[67,116,17] },
  { t: 2583, fr:[66,116,25], fl:[65,56,147], br:[65,75,148], bl:[66,116,18] },
  { t: 2625, fr:[64,117,27], fl:[64,55,145], br:[64,74,146], bl:[64,117,19] },
  { t: 2667, fr:[63,118,28], fl:[62,54,143], br:[62,73,145], bl:[63,118,21] },
  { t: 2708, fr:[61,119,29], fl:[61,53,141], br:[61,73,143], bl:[61,119,22] },
  { t: 2750, fr:[60,119,31], fl:[60,52,139], br:[60,72,141], bl:[60,119,24] },
  { t: 2792, fr:[59,120,32], fl:[59,51,137], br:[59,71,139], bl:[59,120,25] },
  { t: 2833, fr:[57,121,34], fl:[57,50,135], br:[57,70,137], bl:[57,121,27] },
  { t: 2875, fr:[56,122,36], fl:[56,49,133], br:[56,69,134], bl:[56,122,28] },
  { t: 2917, fr:[56,123,37], fl:[56,48,132], br:[56,67,133], bl:[56,123,30] },
  { t: 2958, fr:[56,124,38], fl:[56,47,130], br:[56,66,131], bl:[56,124,31] },
  { t: 3000, fr:[56,125,40], fl:[56,46,128], br:[56,65,130], bl:[56,125,32] },
  { t: 3042, fr:[56,124,38], fl:[56,47,130], br:[56,66,131], bl:[56,124,31] },
  { t: 3083, fr:[56,123,37], fl:[56,48,132], br:[56,67,133], bl:[56,123,30] },
  { t: 3125, fr:[56,122,36], fl:[56,49,133], br:[56,69,134], bl:[56,122,28] },
  { t: 3167, fr:[58,121,34], fl:[58,50,135], br:[58,70,137], bl:[58,121,27] },
  { t: 3208, fr:[59,120,32], fl:[59,51,138], br:[59,71,139], bl:[59,120,25] },
  { t: 3250, fr:[61,120,31], fl:[60,52,140], br:[60,72,141], bl:[61,120,23] },
  { t: 3292, fr:[62,119,29], fl:[62,53,142], br:[62,73,143], bl:[62,119,22] },
  { t: 3333, fr:[64,118,28], fl:[63,54,144], br:[63,73,145], bl:[64,118,21] },
  { t: 3375, fr:[65,117,27], fl:[65,55,146], br:[65,74,147], bl:[65,117,19] },
  { t: 3417, fr:[67,117,25], fl:[66,56,147], br:[66,75,149], bl:[67,117,18] },
  { t: 3458, fr:[68,116,24], fl:[68,57,149], br:[68,76,150], bl:[68,116,17] },
  { t: 3500, fr:[70,115,23], fl:[69,57,151], br:[69,77,152], bl:[70,115,15] },
  { t: 3542, fr:[72,117,25], fl:[71,55,149], br:[71,75,150], bl:[72,117,18] },
  { t: 3583, fr:[73,119,27], fl:[72,54,147], br:[72,73,148], bl:[73,119,20] },
  { t: 3625, fr:[75,120,29], fl:[74,52,145], br:[74,71,146], bl:[75,120,22] },
  { t: 3667, fr:[77,122,32], fl:[76,50,143], br:[76,69,144], bl:[77,122,25] },
  { t: 3708, fr:[79,124,35], fl:[77,48,141], br:[77,67,142], bl:[79,124,27] },
  { t: 3750, fr:[80,126,37], fl:[79,46,138], br:[79,65,140], bl:[80,126,30] },
  { t: 3792, fr:[82,128,40], fl:[81,44,135], br:[81,63,137], bl:[82,128,33] },
  { t: 3833, fr:[84,130,44], fl:[82,42,133], br:[82,61,134], bl:[84,130,36] },
  { t: 3875, fr:[85,132,47], fl:[84,39,129], br:[84,59,131], bl:[85,132,40] },
  { t: 3917, fr:[87,134,51], fl:[86,37,126], br:[86,56,127], bl:[87,134,44] },
  { t: 3958, fr:[89,137,56], fl:[88,34,122], br:[88,54,123], bl:[89,137,48] },
  { t: 4000, fr:[91,140,61], fl:[89,31,117], br:[89,51,118], bl:[91,140,54] },
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
