// FaceHugger clip: wiggle
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
const CLIP_NAME = "wiggle";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
  { t: 42, fr:[91,125,38], fl:[90,46,140], br:[90,65,142], bl:[91,125,31] },
  { t: 83, fr:[92,125,38], fl:[90,46,140], br:[90,65,142], bl:[92,125,30] },
  { t: 125, fr:[93,125,38], fl:[92,46,141], br:[92,65,142], bl:[93,125,30] },
  { t: 167, fr:[95,125,38], fl:[93,46,141], br:[93,65,143], bl:[95,125,30] },
  { t: 208, fr:[97,125,38], fl:[95,46,142], br:[95,65,143], bl:[97,125,30] },
  { t: 250, fr:[99,124,38], fl:[98,46,142], br:[98,66,144], bl:[99,124,30] },
  { t: 292, fr:[102,124,38], fl:[100,47,143], br:[100,66,144], bl:[102,124,30] },
  { t: 333, fr:[104,123,38], fl:[103,47,143], br:[103,66,145], bl:[104,123,31] },
  { t: 375, fr:[107,123,38], fl:[106,47,144], br:[106,67,145], bl:[107,123,31] },
  { t: 417, fr:[110,122,39], fl:[109,48,144], br:[109,67,145], bl:[110,122,31] },
  { t: 458, fr:[113,122,39], fl:[112,48,144], br:[112,68,146], bl:[113,122,32] },
  { t: 500, fr:[116,121,40], fl:[116,49,145], br:[116,68,146], bl:[116,121,33] },
  { t: 542, fr:[119,121,41], fl:[119,49,145], br:[119,69,146], bl:[119,121,33] },
  { t: 583, fr:[122,121,42], fl:[122,50,145], br:[122,69,146], bl:[122,121,34] },
  { t: 625, fr:[124,120,43], fl:[124,51,144], br:[124,70,146], bl:[124,120,36] },
  { t: 667, fr:[127,120,44], fl:[127,51,144], br:[127,70,145], bl:[127,120,37] },
  { t: 708, fr:[129,120,45], fl:[129,52,144], br:[129,71,145], bl:[129,120,38] },
  { t: 750, fr:[131,120,47], fl:[131,52,143], br:[131,71,145], bl:[131,120,39] },
  { t: 792, fr:[132,120,48], fl:[133,52,143], br:[133,72,144], bl:[132,120,40] },
  { t: 833, fr:[134,120,49], fl:[135,53,142], br:[135,72,144], bl:[134,120,42] },
  { t: 875, fr:[135,120,50], fl:[136,53,142], br:[136,72,143], bl:[135,120,43] },
  { t: 917, fr:[136,120,51], fl:[137,53,142], br:[137,72,143], bl:[136,120,43] },
  { t: 958, fr:[136,120,51], fl:[137,53,142], br:[137,72,143], bl:[136,120,44] },
  { t: 1000, fr:[137,120,51], fl:[138,53,142], br:[138,72,143], bl:[137,120,44] },
  { t: 1042, fr:[136,120,51], fl:[137,53,142], br:[137,72,143], bl:[136,120,44] },
  { t: 1083, fr:[135,120,50], fl:[136,53,142], br:[136,72,144], bl:[135,120,42] },
  { t: 1125, fr:[134,120,49], fl:[135,53,143], br:[135,72,144], bl:[134,119,41] },
  { t: 1167, fr:[132,119,47], fl:[133,53,144], br:[133,72,145], bl:[132,119,39] },
  { t: 1208, fr:[130,119,45], fl:[130,53,145], br:[130,72,146], bl:[130,118,36] },
  { t: 1250, fr:[127,119,43], fl:[128,53,146], br:[127,72,147], bl:[127,117,34] },
  { t: 1292, fr:[124,119,41], fl:[124,53,147], br:[123,72,149], bl:[124,117,31] },
  { t: 1333, fr:[120,119,39], fl:[120,52,148], br:[119,71,150], bl:[120,116,29] },
  { t: 1375, fr:[116,120,37], fl:[116,52,149], br:[115,71,150], bl:[116,116,26] },
  { t: 1417, fr:[112,120,36], fl:[112,52,149], br:[110,71,151], bl:[111,116,24] },
  { t: 1458, fr:[107,121,34], fl:[107,51,149], br:[105,71,152], bl:[107,116,22] },
  { t: 1500, fr:[102,122,33], fl:[102,51,149], br:[99,72,152], bl:[102,115,20] },
  { t: 1542, fr:[96,122,33], fl:[96,51,149], br:[94,72,152], bl:[97,115,19] },
  { t: 1583, fr:[91,123,32], fl:[91,50,148], br:[89,73,152], bl:[92,115,18] },
  { t: 1625, fr:[86,123,32], fl:[86,50,147], br:[84,73,152], bl:[87,115,17] },
  { t: 1667, fr:[80,124,32], fl:[81,50,146], br:[79,74,151], bl:[82,115,17] },
  { t: 1708, fr:[76,124,32], fl:[77,50,145], br:[74,75,150], bl:[78,115,17] },
  { t: 1750, fr:[71,124,33], fl:[73,49,143], br:[71,76,150], bl:[74,115,17] },
  { t: 1792, fr:[68,124,33], fl:[70,49,142], br:[67,76,149], bl:[70,114,17] },
  { t: 1833, fr:[64,124,34], fl:[67,49,140], br:[65,76,148], bl:[67,114,17] },
  { t: 1875, fr:[62,124,34], fl:[64,49,139], br:[62,77,147], bl:[65,114,17] },
  { t: 1917, fr:[60,124,34], fl:[63,49,138], br:[61,77,146], bl:[63,114,18] },
  { t: 1958, fr:[59,124,35], fl:[61,48,137], br:[60,77,146], bl:[62,114,18] },
  { t: 2000, fr:[58,123,35], fl:[61,48,137], br:[59,77,146], bl:[62,114,18] },
  { t: 2042, fr:[59,124,35], fl:[61,48,137], br:[59,77,146], bl:[62,114,18] },
  { t: 2083, fr:[59,124,35], fl:[62,48,137], br:[60,77,146], bl:[62,114,18] },
  { t: 2125, fr:[60,124,35], fl:[62,48,137], br:[60,77,146], bl:[63,114,18] },
  { t: 2167, fr:[61,124,35], fl:[63,48,138], br:[61,76,146], bl:[64,115,18] },
  { t: 2208, fr:[62,124,35], fl:[64,48,138], br:[62,76,146], bl:[65,115,19] },
  { t: 2250, fr:[63,124,35], fl:[65,48,138], br:[63,75,146], bl:[66,116,19] },
  { t: 2292, fr:[65,124,35], fl:[67,48,139], br:[65,75,146], bl:[67,116,19] },
  { t: 2333, fr:[66,125,35], fl:[68,48,139], br:[66,74,146], bl:[69,117,20] },
  { t: 2375, fr:[68,125,35], fl:[70,48,139], br:[68,73,145], bl:[71,117,20] },
  { t: 2417, fr:[70,125,35], fl:[71,48,140], br:[70,72,145], bl:[72,118,21] },
  { t: 2458, fr:[72,125,35], fl:[73,48,140], br:[71,72,145], bl:[74,119,22] },
  { t: 2500, fr:[74,125,35], fl:[75,48,140], br:[73,71,145], bl:[76,120,22] },
  { t: 2542, fr:[76,125,36], fl:[77,47,140], br:[75,70,145], bl:[78,120,23] },
  { t: 2583, fr:[78,126,36], fl:[78,47,141], br:[77,69,144], bl:[79,121,24] },
  { t: 2625, fr:[80,126,36], fl:[80,47,141], br:[79,68,144], bl:[81,122,25] },
  { t: 2667, fr:[82,126,36], fl:[82,47,141], br:[81,68,143], bl:[83,122,26] },
  { t: 2708, fr:[84,126,37], fl:[83,46,141], br:[82,67,143], bl:[84,123,27] },
  { t: 2750, fr:[86,126,37], fl:[85,46,141], br:[84,67,143], bl:[86,124,28] },
  { t: 2792, fr:[87,126,37], fl:[86,46,140], br:[86,66,142], bl:[87,124,28] },
  { t: 2833, fr:[88,126,37], fl:[87,46,140], br:[87,66,142], bl:[88,125,29] },
  { t: 2875, fr:[89,126,38], fl:[88,46,140], br:[88,65,142], bl:[89,125,30] },
  { t: 2917, fr:[90,125,38], fl:[89,46,140], br:[89,65,142], bl:[90,125,30] },
  { t: 2958, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,30] },
  { t: 3000, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
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
