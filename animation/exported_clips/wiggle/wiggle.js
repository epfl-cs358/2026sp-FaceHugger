// FaceHugger clip: wiggle
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
const CLIP_NAME = "wiggle";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,131,33], fl:[90,49,148], br:[90,52,152], bl:[91,131,34] },
  { t: 83, fr:[92,131,33], fl:[90,49,148], br:[90,52,152], bl:[92,131,33] },
  { t: 125, fr:[93,131,33], fl:[92,49,149], br:[92,52,152], bl:[93,131,33] },
  { t: 167, fr:[95,131,33], fl:[93,49,149], br:[93,52,153], bl:[95,131,33] },
  { t: 208, fr:[97,131,33], fl:[95,49,150], br:[95,52,153], bl:[97,131,33] },
  { t: 250, fr:[99,130,33], fl:[98,49,150], br:[98,53,154], bl:[99,130,33] },
  { t: 292, fr:[102,130,33], fl:[100,50,151], br:[100,53,154], bl:[102,130,33] },
  { t: 333, fr:[104,129,33], fl:[103,50,151], br:[103,53,155], bl:[104,129,34] },
  { t: 375, fr:[107,129,33], fl:[106,50,152], br:[106,54,155], bl:[107,129,34] },
  { t: 417, fr:[110,128,34], fl:[109,51,152], br:[109,54,155], bl:[110,128,34] },
  { t: 458, fr:[113,128,34], fl:[112,51,152], br:[112,55,156], bl:[113,128,35] },
  { t: 500, fr:[116,127,35], fl:[116,52,153], br:[116,55,156], bl:[116,127,36] },
  { t: 542, fr:[119,127,36], fl:[119,52,153], br:[119,56,156], bl:[119,127,36] },
  { t: 583, fr:[122,127,37], fl:[122,53,153], br:[122,56,156], bl:[122,127,37] },
  { t: 625, fr:[124,126,38], fl:[124,54,152], br:[124,57,156], bl:[124,126,39] },
  { t: 667, fr:[127,126,39], fl:[127,54,152], br:[127,57,155], bl:[127,126,40] },
  { t: 708, fr:[129,126,40], fl:[129,55,152], br:[129,58,155], bl:[129,126,41] },
  { t: 750, fr:[131,126,42], fl:[131,55,151], br:[131,58,155], bl:[131,126,42] },
  { t: 792, fr:[132,126,43], fl:[133,55,151], br:[133,59,154], bl:[132,126,43] },
  { t: 833, fr:[134,126,44], fl:[135,56,150], br:[135,59,154], bl:[134,126,45] },
  { t: 875, fr:[135,126,45], fl:[136,56,150], br:[136,59,153], bl:[135,126,46] },
  { t: 917, fr:[136,126,46], fl:[137,56,150], br:[137,59,153], bl:[136,126,46] },
  { t: 958, fr:[136,126,46], fl:[137,56,150], br:[137,59,153], bl:[136,126,47] },
  { t: 1000, fr:[137,126,46], fl:[138,56,150], br:[138,59,153], bl:[137,126,47] },
  { t: 1042, fr:[136,126,46], fl:[137,56,150], br:[137,59,153], bl:[136,126,47] },
  { t: 1083, fr:[135,126,45], fl:[136,56,150], br:[136,59,154], bl:[135,126,45] },
  { t: 1125, fr:[134,126,44], fl:[135,56,151], br:[135,59,154], bl:[134,125,44] },
  { t: 1167, fr:[132,125,42], fl:[133,56,152], br:[133,59,155], bl:[132,125,42] },
  { t: 1208, fr:[130,125,40], fl:[130,56,153], br:[130,59,156], bl:[130,124,39] },
  { t: 1250, fr:[127,125,38], fl:[128,56,154], br:[127,59,157], bl:[127,123,37] },
  { t: 1292, fr:[124,125,36], fl:[124,56,155], br:[123,59,159], bl:[124,123,34] },
  { t: 1333, fr:[120,125,34], fl:[120,55,156], br:[119,58,160], bl:[120,122,32] },
  { t: 1375, fr:[116,126,32], fl:[116,55,157], br:[115,58,160], bl:[116,122,29] },
  { t: 1417, fr:[112,126,31], fl:[112,55,157], br:[110,58,161], bl:[111,122,27] },
  { t: 1458, fr:[107,127,29], fl:[107,54,157], br:[105,58,162], bl:[107,122,25] },
  { t: 1500, fr:[102,128,28], fl:[102,54,157], br:[99,59,162], bl:[102,121,23] },
  { t: 1542, fr:[96,128,28], fl:[96,54,157], br:[94,59,162], bl:[97,121,22] },
  { t: 1583, fr:[91,129,27], fl:[91,53,156], br:[89,60,162], bl:[92,121,21] },
  { t: 1625, fr:[86,129,27], fl:[86,53,155], br:[84,60,162], bl:[87,121,20] },
  { t: 1667, fr:[80,130,27], fl:[81,53,154], br:[79,61,161], bl:[82,121,20] },
  { t: 1708, fr:[76,130,27], fl:[77,53,153], br:[74,62,160], bl:[78,121,20] },
  { t: 1750, fr:[71,130,28], fl:[73,52,151], br:[71,63,160], bl:[74,121,20] },
  { t: 1792, fr:[68,130,28], fl:[70,52,150], br:[67,63,159], bl:[70,120,20] },
  { t: 1833, fr:[64,130,29], fl:[67,52,148], br:[65,63,158], bl:[67,120,20] },
  { t: 1875, fr:[62,130,29], fl:[64,52,147], br:[62,64,157], bl:[65,120,20] },
  { t: 1917, fr:[60,130,29], fl:[63,52,146], br:[61,64,156], bl:[63,120,21] },
  { t: 1958, fr:[59,130,30], fl:[61,51,145], br:[60,64,156], bl:[62,120,21] },
  { t: 2000, fr:[58,129,30], fl:[61,51,145], br:[59,64,156], bl:[62,120,21] },
  { t: 2042, fr:[59,130,30], fl:[61,51,145], br:[59,64,156], bl:[62,120,21] },
  { t: 2083, fr:[59,130,30], fl:[62,51,145], br:[60,64,156], bl:[62,120,21] },
  { t: 2125, fr:[60,130,30], fl:[62,51,145], br:[60,64,156], bl:[63,120,21] },
  { t: 2167, fr:[61,130,30], fl:[63,51,146], br:[61,63,156], bl:[64,121,21] },
  { t: 2208, fr:[62,130,30], fl:[64,51,146], br:[62,63,156], bl:[65,121,22] },
  { t: 2250, fr:[63,130,30], fl:[65,51,146], br:[63,62,156], bl:[66,122,22] },
  { t: 2292, fr:[65,130,30], fl:[67,51,147], br:[65,62,156], bl:[67,122,22] },
  { t: 2333, fr:[66,131,30], fl:[68,51,147], br:[66,61,156], bl:[69,123,23] },
  { t: 2375, fr:[68,131,30], fl:[70,51,147], br:[68,60,155], bl:[71,123,23] },
  { t: 2417, fr:[70,131,30], fl:[71,51,148], br:[70,59,155], bl:[72,124,24] },
  { t: 2458, fr:[72,131,30], fl:[73,51,148], br:[71,59,155], bl:[74,125,25] },
  { t: 2500, fr:[74,131,30], fl:[75,51,148], br:[73,58,155], bl:[76,126,25] },
  { t: 2542, fr:[76,131,31], fl:[77,50,148], br:[75,57,155], bl:[78,126,26] },
  { t: 2583, fr:[78,132,31], fl:[78,50,149], br:[77,56,154], bl:[79,127,27] },
  { t: 2625, fr:[80,132,31], fl:[80,50,149], br:[79,55,154], bl:[81,128,28] },
  { t: 2667, fr:[82,132,31], fl:[82,50,149], br:[81,55,153], bl:[83,128,29] },
  { t: 2708, fr:[84,132,32], fl:[83,49,149], br:[82,54,153], bl:[84,129,30] },
  { t: 2750, fr:[86,132,32], fl:[85,49,149], br:[84,54,153], bl:[86,130,31] },
  { t: 2792, fr:[87,132,32], fl:[86,49,148], br:[86,53,152], bl:[87,130,31] },
  { t: 2833, fr:[88,132,32], fl:[87,49,148], br:[87,53,152], bl:[88,131,32] },
  { t: 2875, fr:[89,132,33], fl:[88,49,148], br:[88,52,152], bl:[89,131,33] },
  { t: 2917, fr:[90,131,33], fl:[89,49,148], br:[89,52,152], bl:[90,131,33] },
  { t: 2958, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,33] },
  { t: 3000, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
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
