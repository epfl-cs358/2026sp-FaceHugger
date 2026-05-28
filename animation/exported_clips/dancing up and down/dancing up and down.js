// FaceHugger clip: dancing up and down
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
const CLIP_NAME = "dancing up and down";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,146,56], fl:[89,34,125], br:[89,38,128], bl:[91,146,57] },
  { t: 42, fr:[93,143,52], fl:[92,37,130], br:[92,41,134], bl:[93,143,52] },
  { t: 83, fr:[95,140,47], fl:[94,40,135], br:[94,43,138], bl:[95,140,48] },
  { t: 125, fr:[98,138,44], fl:[96,42,139], br:[96,46,142], bl:[98,138,45] },
  { t: 167, fr:[100,135,41], fl:[99,44,142], br:[99,48,146], bl:[100,135,42] },
  { t: 208, fr:[102,133,38], fl:[101,46,146], br:[101,50,149], bl:[102,133,39] },
  { t: 250, fr:[104,131,36], fl:[103,48,149], br:[103,52,152], bl:[104,131,36] },
  { t: 292, fr:[107,129,33], fl:[106,50,151], br:[106,53,155], bl:[107,129,34] },
  { t: 333, fr:[109,127,31], fl:[108,52,154], br:[108,55,157], bl:[109,127,32] },
  { t: 375, fr:[111,125,30], fl:[110,54,156], br:[110,57,160], bl:[111,125,30] },
  { t: 417, fr:[113,123,28], fl:[112,56,158], br:[112,59,162], bl:[113,123,29] },
  { t: 458, fr:[115,121,26], fl:[114,57,161], br:[114,61,164], bl:[115,121,27] },
  { t: 500, fr:[117,119,25], fl:[117,59,162], br:[117,63,166], bl:[117,119,26] },
  { t: 542, fr:[119,120,26], fl:[119,59,162], br:[119,62,165], bl:[119,120,27] },
  { t: 583, fr:[121,120,28], fl:[121,59,161], br:[121,62,164], bl:[121,120,29] },
  { t: 625, fr:[123,121,30], fl:[123,58,159], br:[123,62,163], bl:[123,121,30] },
  { t: 667, fr:[125,121,32], fl:[125,58,158], br:[125,61,162], bl:[125,121,32] },
  { t: 708, fr:[126,122,33], fl:[127,58,157], br:[127,61,160], bl:[126,122,34] },
  { t: 750, fr:[128,123,35], fl:[129,57,156], br:[129,61,159], bl:[128,123,36] },
  { t: 792, fr:[130,123,38], fl:[130,57,155], br:[130,60,158], bl:[130,123,38] },
  { t: 833, fr:[132,124,40], fl:[132,57,153], br:[132,60,157], bl:[132,124,40] },
  { t: 875, fr:[133,125,42], fl:[134,56,152], br:[134,60,155], bl:[133,125,43] },
  { t: 917, fr:[135,126,45], fl:[136,56,150], br:[136,59,153], bl:[135,126,45] },
  { t: 958, fr:[136,127,48], fl:[137,55,148], br:[137,59,152], bl:[136,127,48] },
  { t: 1000, fr:[138,128,51], fl:[139,55,147], br:[139,58,150], bl:[138,128,51] },
  { t: 1042, fr:[137,128,49], fl:[139,55,148], br:[139,59,151], bl:[137,128,50] },
  { t: 1083, fr:[137,127,47], fl:[138,56,149], br:[138,59,152], bl:[137,127,48] },
  { t: 1125, fr:[136,126,46], fl:[137,56,150], br:[137,60,153], bl:[136,126,47] },
  { t: 1167, fr:[136,125,44], fl:[137,57,151], br:[137,60,154], bl:[136,125,45] },
  { t: 1208, fr:[135,124,43], fl:[136,57,152], br:[136,61,155], bl:[135,124,44] },
  { t: 1250, fr:[135,124,42], fl:[136,58,153], br:[136,61,156], bl:[135,124,42] },
  { t: 1292, fr:[134,123,40], fl:[135,58,154], br:[135,62,157], bl:[134,123,41] },
  { t: 1333, fr:[134,122,39], fl:[135,59,155], br:[135,62,158], bl:[134,122,40] },
  { t: 1375, fr:[133,122,38], fl:[134,59,155], br:[134,62,159], bl:[133,122,39] },
  { t: 1417, fr:[133,121,37], fl:[133,60,156], br:[133,63,160], bl:[133,121,37] },
  { t: 1458, fr:[132,121,35], fl:[133,60,157], br:[133,63,161], bl:[132,121,36] },
  { t: 1500, fr:[132,120,34], fl:[132,60,158], br:[132,64,161], bl:[132,120,35] },
  { t: 1542, fr:[131,120,34], fl:[132,60,158], br:[132,63,161], bl:[131,120,35] },
  { t: 1583, fr:[130,121,34], fl:[131,60,158], br:[131,63,161], bl:[130,121,35] },
  { t: 1625, fr:[130,121,34], fl:[130,59,157], br:[130,63,161], bl:[130,121,35] },
  { t: 1667, fr:[129,121,34], fl:[130,59,157], br:[130,62,161], bl:[129,121,35] },
  { t: 1708, fr:[129,122,34], fl:[129,58,157], br:[129,62,160], bl:[129,122,35] },
  { t: 1750, fr:[128,122,35], fl:[129,58,157], br:[129,61,160], bl:[128,122,35] },
  { t: 1792, fr:[128,122,35], fl:[128,58,156], br:[128,61,160], bl:[128,122,35] },
  { t: 1833, fr:[127,123,35], fl:[127,57,156], br:[127,61,160], bl:[127,123,35] },
  { t: 1875, fr:[126,123,35], fl:[127,57,156], br:[127,60,159], bl:[126,123,35] },
  { t: 1917, fr:[126,123,35], fl:[126,56,156], br:[126,60,159], bl:[126,123,35] },
  { t: 1958, fr:[125,124,35], fl:[125,56,155], br:[125,59,159], bl:[125,124,36] },
  { t: 2000, fr:[125,124,35], fl:[125,56,155], br:[125,59,158], bl:[125,124,36] },
  { t: 2042, fr:[124,124,35], fl:[124,55,155], br:[124,59,158], bl:[124,124,36] },
  { t: 2083, fr:[123,125,35], fl:[123,55,154], br:[123,58,158], bl:[123,125,36] },
  { t: 2125, fr:[123,125,35], fl:[123,54,154], br:[123,58,157], bl:[123,125,36] },
  { t: 2167, fr:[122,125,36], fl:[122,54,154], br:[122,57,157], bl:[122,125,36] },
  { t: 2208, fr:[121,126,36], fl:[121,54,153], br:[121,57,157], bl:[121,126,36] },
  { t: 2250, fr:[121,126,36], fl:[121,53,153], br:[121,57,156], bl:[121,126,37] },
  { t: 2292, fr:[120,127,36], fl:[120,53,153], br:[120,56,156], bl:[120,127,37] },
  { t: 2333, fr:[120,127,36], fl:[119,52,152], br:[119,56,156], bl:[120,127,37] },
  { t: 2375, fr:[119,127,36], fl:[119,52,152], br:[119,55,155], bl:[119,127,37] },
  { t: 2417, fr:[118,128,37], fl:[118,52,152], br:[118,55,155], bl:[118,128,37] },
  { t: 2458, fr:[118,128,37], fl:[117,51,151], br:[117,55,155], bl:[118,128,38] },
  { t: 2500, fr:[117,129,37], fl:[117,51,151], br:[117,54,154], bl:[117,129,38] },
  { t: 2542, fr:[116,129,37], fl:[116,50,150], br:[116,54,154], bl:[116,129,38] },
  { t: 2583, fr:[116,129,38], fl:[115,50,150], br:[115,53,153], bl:[116,129,38] },
  { t: 2625, fr:[115,130,38], fl:[114,50,149], br:[114,53,153], bl:[115,130,39] },
  { t: 2667, fr:[114,130,38], fl:[114,49,149], br:[114,53,152], bl:[114,130,39] },
  { t: 2708, fr:[114,131,39], fl:[113,49,149], br:[113,52,152], bl:[114,131,39] },
  { t: 2750, fr:[113,131,39], fl:[112,48,148], br:[112,52,151], bl:[113,131,40] },
  { t: 2792, fr:[112,131,39], fl:[112,48,148], br:[112,51,151], bl:[112,131,40] },
  { t: 2833, fr:[112,132,40], fl:[111,48,147], br:[111,51,150], bl:[112,132,40] },
  { t: 2875, fr:[111,132,40], fl:[110,47,146], br:[110,51,150], bl:[111,132,41] },
  { t: 2917, fr:[110,133,40], fl:[109,47,146], br:[109,50,149], bl:[110,133,41] },
  { t: 2958, fr:[109,133,41], fl:[109,46,145], br:[109,50,149], bl:[109,133,41] },
  { t: 3000, fr:[109,134,41], fl:[108,46,145], br:[108,49,148], bl:[109,134,42] },
  { t: 3042, fr:[108,134,41], fl:[107,46,144], br:[107,49,148], bl:[108,134,42] },
  { t: 3083, fr:[107,134,42], fl:[106,45,144], br:[106,48,147], bl:[107,134,42] },
  { t: 3125, fr:[107,135,42], fl:[106,45,143], br:[106,48,146], bl:[107,135,43] },
  { t: 3167, fr:[106,135,43], fl:[105,44,142], br:[105,48,146], bl:[106,135,43] },
  { t: 3208, fr:[105,136,43], fl:[104,44,142], br:[104,47,145], bl:[105,136,44] },
  { t: 3250, fr:[104,136,44], fl:[103,43,141], br:[103,47,144], bl:[104,136,44] },
  { t: 3292, fr:[104,137,44], fl:[103,43,140], br:[103,46,144], bl:[104,137,45] },
  { t: 3333, fr:[103,137,45], fl:[102,43,140], br:[102,46,143], bl:[103,137,45] },
  { t: 3375, fr:[102,138,45], fl:[101,42,139], br:[101,45,142], bl:[102,138,46] },
  { t: 3417, fr:[101,138,46], fl:[100,42,138], br:[100,45,142], bl:[101,138,46] },
  { t: 3458, fr:[101,139,46], fl:[99,41,138], br:[99,45,141], bl:[101,139,47] },
  { t: 3500, fr:[100,139,47], fl:[99,41,137], br:[99,44,140], bl:[100,139,48] },
  { t: 3542, fr:[99,140,48], fl:[98,40,136], br:[98,44,139], bl:[99,140,48] },
  { t: 3583, fr:[98,140,48], fl:[97,40,135], br:[97,43,138], bl:[98,140,49] },
  { t: 3625, fr:[98,141,49], fl:[96,39,134], br:[96,43,138], bl:[98,141,50] },
  { t: 3667, fr:[97,141,50], fl:[96,39,133], br:[96,42,137], bl:[97,141,50] },
  { t: 3708, fr:[96,142,50], fl:[95,38,132], br:[95,42,136], bl:[96,142,51] },
  { t: 3750, fr:[95,142,51], fl:[94,38,131], br:[94,41,135], bl:[95,142,52] },
  { t: 3792, fr:[95,143,52], fl:[93,37,130], br:[93,41,134], bl:[95,143,53] },
  { t: 3833, fr:[94,143,53], fl:[92,37,129], br:[92,40,133], bl:[94,143,53] },
  { t: 3875, fr:[93,144,54], fl:[92,36,128], br:[92,40,132], bl:[93,144,54] },
  { t: 3917, fr:[92,144,54], fl:[91,36,127], br:[91,39,130], bl:[92,144,55] },
  { t: 3958, fr:[91,145,55], fl:[90,35,126], br:[90,38,129], bl:[91,145,56] },
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
