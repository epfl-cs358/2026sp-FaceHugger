// FaceHugger clip: wiggle
// Generated 2026-05-21 from Blender animation
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
  { t: 0, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,34] },
  { t: 42, fr:[59,131,33], fl:[25,49,148], br:[60,52,152], bl:[179,131,34] },
  { t: 83, fr:[58,131,33], fl:[25,49,148], br:[60,52,152], bl:[178,131,33] },
  { t: 125, fr:[57,131,33], fl:[27,49,149], br:[58,52,152], bl:[177,131,33] },
  { t: 167, fr:[55,131,33], fl:[28,49,149], br:[57,52,153], bl:[175,131,33] },
  { t: 208, fr:[53,131,33], fl:[30,49,150], br:[55,52,153], bl:[173,131,33] },
  { t: 250, fr:[51,130,33], fl:[33,49,150], br:[52,53,154], bl:[171,130,33] },
  { t: 292, fr:[48,130,33], fl:[35,50,151], br:[50,53,154], bl:[168,130,33] },
  { t: 333, fr:[46,129,33], fl:[38,50,151], br:[47,53,155], bl:[166,129,34] },
  { t: 375, fr:[43,129,33], fl:[41,50,152], br:[44,54,155], bl:[163,129,34] },
  { t: 417, fr:[40,128,34], fl:[44,51,152], br:[41,54,155], bl:[160,128,34] },
  { t: 458, fr:[37,128,34], fl:[47,51,152], br:[38,55,156], bl:[157,128,35] },
  { t: 500, fr:[34,127,35], fl:[51,52,153], br:[34,55,156], bl:[154,127,36] },
  { t: 542, fr:[31,127,36], fl:[54,52,153], br:[31,56,156], bl:[151,127,36] },
  { t: 583, fr:[28,127,37], fl:[56,53,153], br:[28,56,156], bl:[148,127,37] },
  { t: 625, fr:[26,126,38], fl:[59,54,152], br:[26,57,156], bl:[146,126,39] },
  { t: 667, fr:[23,126,39], fl:[62,54,152], br:[23,57,155], bl:[143,126,40] },
  { t: 708, fr:[21,126,40], fl:[64,55,152], br:[21,58,155], bl:[141,126,41] },
  { t: 750, fr:[19,126,42], fl:[66,55,151], br:[19,58,155], bl:[139,126,42] },
  { t: 792, fr:[18,126,43], fl:[68,55,151], br:[17,59,154], bl:[138,126,43] },
  { t: 833, fr:[16,126,44], fl:[70,56,150], br:[15,59,154], bl:[136,126,45] },
  { t: 875, fr:[15,126,45], fl:[71,56,150], br:[14,59,153], bl:[135,126,46] },
  { t: 917, fr:[14,126,46], fl:[72,56,150], br:[13,59,153], bl:[134,126,46] },
  { t: 958, fr:[14,126,46], fl:[72,56,150], br:[13,59,153], bl:[134,126,47] },
  { t: 1000, fr:[13,126,46], fl:[73,56,150], br:[12,59,153], bl:[133,126,47] },
  { t: 1042, fr:[14,126,46], fl:[72,56,150], br:[13,59,153], bl:[134,126,47] },
  { t: 1083, fr:[15,126,45], fl:[71,56,150], br:[14,59,154], bl:[135,126,45] },
  { t: 1125, fr:[16,126,44], fl:[70,56,151], br:[15,59,154], bl:[136,125,44] },
  { t: 1167, fr:[18,125,42], fl:[68,56,152], br:[17,59,155], bl:[138,125,42] },
  { t: 1208, fr:[20,125,40], fl:[65,56,153], br:[20,59,156], bl:[140,124,39] },
  { t: 1250, fr:[23,125,38], fl:[63,56,154], br:[23,59,157], bl:[143,123,37] },
  { t: 1292, fr:[26,125,36], fl:[59,56,155], br:[27,59,159], bl:[146,123,34] },
  { t: 1333, fr:[30,125,34], fl:[55,55,156], br:[31,58,160], bl:[150,122,32] },
  { t: 1375, fr:[34,126,32], fl:[51,55,157], br:[35,58,160], bl:[154,122,29] },
  { t: 1417, fr:[38,126,31], fl:[47,55,157], br:[40,58,161], bl:[159,122,27] },
  { t: 1458, fr:[43,127,29], fl:[42,54,157], br:[45,58,162], bl:[163,122,25] },
  { t: 1500, fr:[48,128,28], fl:[37,54,157], br:[51,59,162], bl:[168,121,23] },
  { t: 1542, fr:[54,128,28], fl:[31,54,157], br:[56,59,162], bl:[173,121,22] },
  { t: 1583, fr:[59,129,27], fl:[26,53,156], br:[61,60,162], bl:[178,121,21] },
  { t: 1625, fr:[64,129,27], fl:[21,53,155], br:[66,60,162], bl:[183,121,20] },
  { t: 1667, fr:[70,130,27], fl:[16,53,154], br:[71,61,161], bl:[188,121,20] },
  { t: 1708, fr:[74,130,27], fl:[12,53,153], br:[76,62,160], bl:[192,121,20] },
  { t: 1750, fr:[79,130,28], fl:[8,52,151], br:[79,63,160], bl:[196,121,20] },
  { t: 1792, fr:[82,130,28], fl:[5,52,150], br:[83,63,159], bl:[200,120,20] },
  { t: 1833, fr:[86,130,29], fl:[2,52,148], br:[85,63,158], bl:[203,120,20] },
  { t: 1875, fr:[88,130,29], fl:[-1,52,147], br:[88,64,157], bl:[205,120,20] },
  { t: 1917, fr:[90,130,29], fl:[-2,52,146], br:[89,64,156], bl:[207,120,21] },
  { t: 1958, fr:[91,130,30], fl:[-4,51,145], br:[90,64,156], bl:[208,120,21] },
  { t: 2000, fr:[92,129,30], fl:[-4,51,145], br:[91,64,156], bl:[208,120,21] },
  { t: 2042, fr:[91,130,30], fl:[-4,51,145], br:[91,64,156], bl:[208,120,21] },
  { t: 2083, fr:[91,130,30], fl:[-3,51,145], br:[90,64,156], bl:[208,120,21] },
  { t: 2125, fr:[90,130,30], fl:[-3,51,145], br:[90,64,156], bl:[207,120,21] },
  { t: 2167, fr:[89,130,30], fl:[-2,51,146], br:[89,63,156], bl:[206,121,21] },
  { t: 2208, fr:[88,130,30], fl:[-1,51,146], br:[88,63,156], bl:[205,121,22] },
  { t: 2250, fr:[87,130,30], fl:[0,51,146], br:[87,62,156], bl:[204,122,22] },
  { t: 2292, fr:[85,130,30], fl:[2,51,147], br:[85,62,156], bl:[203,122,22] },
  { t: 2333, fr:[84,131,30], fl:[3,51,147], br:[84,61,156], bl:[201,123,23] },
  { t: 2375, fr:[82,131,30], fl:[5,51,147], br:[82,60,155], bl:[199,123,23] },
  { t: 2417, fr:[80,131,30], fl:[6,51,148], br:[80,59,155], bl:[198,124,24] },
  { t: 2458, fr:[78,131,30], fl:[8,51,148], br:[79,59,155], bl:[196,125,25] },
  { t: 2500, fr:[76,131,30], fl:[10,51,148], br:[77,58,155], bl:[194,126,25] },
  { t: 2542, fr:[74,131,31], fl:[12,50,148], br:[75,57,155], bl:[192,126,26] },
  { t: 2583, fr:[72,132,31], fl:[13,50,149], br:[73,56,154], bl:[191,127,27] },
  { t: 2625, fr:[70,132,31], fl:[15,50,149], br:[71,55,154], bl:[189,128,28] },
  { t: 2667, fr:[68,132,31], fl:[17,50,149], br:[69,55,153], bl:[187,128,29] },
  { t: 2708, fr:[66,132,32], fl:[18,49,149], br:[68,54,153], bl:[186,129,30] },
  { t: 2750, fr:[64,132,32], fl:[20,49,149], br:[66,54,153], bl:[184,130,31] },
  { t: 2792, fr:[63,132,32], fl:[21,49,148], br:[64,53,152], bl:[183,130,31] },
  { t: 2833, fr:[62,132,32], fl:[22,49,148], br:[63,53,152], bl:[182,131,32] },
  { t: 2875, fr:[61,132,33], fl:[23,49,148], br:[62,52,152], bl:[181,131,33] },
  { t: 2917, fr:[60,131,33], fl:[24,49,148], br:[61,52,152], bl:[180,131,33] },
  { t: 2958, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,33] },
  { t: 3000, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,34] },
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
