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
  { t: 42, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,34] },
  { t: 83, fr:[59,131,33], fl:[25,49,148], br:[60,52,151], bl:[179,131,34] },
  { t: 125, fr:[59,131,33], fl:[25,49,148], br:[60,52,152], bl:[179,131,34] },
  { t: 167, fr:[58,131,33], fl:[25,48,148], br:[60,52,152], bl:[178,131,34] },
  { t: 208, fr:[58,131,33], fl:[26,48,148], br:[59,52,152], bl:[178,131,34] },
  { t: 250, fr:[58,131,33], fl:[26,48,148], br:[59,52,152], bl:[178,131,34] },
  { t: 292, fr:[57,131,33], fl:[27,48,148], br:[58,52,152], bl:[177,131,34] },
  { t: 333, fr:[56,131,33], fl:[27,48,148], br:[58,52,152], bl:[176,131,34] },
  { t: 375, fr:[56,131,33], fl:[28,48,148], br:[57,52,152], bl:[176,131,34] },
  { t: 417, fr:[55,131,33], fl:[29,48,149], br:[56,52,152], bl:[175,131,34] },
  { t: 458, fr:[54,131,34], fl:[29,48,149], br:[56,52,152], bl:[174,131,34] },
  { t: 500, fr:[54,131,34], fl:[30,48,149], br:[55,52,152], bl:[174,131,34] },
  { t: 542, fr:[53,131,34], fl:[31,48,149], br:[54,52,152], bl:[173,131,34] },
  { t: 583, fr:[52,131,34], fl:[31,48,149], br:[54,52,152], bl:[172,131,35] },
  { t: 625, fr:[52,131,34], fl:[32,48,149], br:[53,52,152], bl:[172,131,35] },
  { t: 667, fr:[51,131,34], fl:[33,48,149], br:[52,52,152], bl:[171,131,35] },
  { t: 708, fr:[50,131,34], fl:[33,48,149], br:[52,52,152], bl:[171,131,35] },
  { t: 750, fr:[50,131,34], fl:[34,48,149], br:[51,52,152], bl:[170,131,35] },
  { t: 792, fr:[49,131,35], fl:[34,48,149], br:[51,52,152], bl:[169,131,35] },
  { t: 833, fr:[49,131,35], fl:[35,48,149], br:[50,52,152], bl:[169,131,35] },
  { t: 875, fr:[49,131,35], fl:[35,48,149], br:[50,52,152], bl:[169,131,35] },
  { t: 917, fr:[48,131,35], fl:[35,48,149], br:[50,52,152], bl:[168,131,35] },
  { t: 958, fr:[48,131,35], fl:[35,48,149], br:[50,52,152], bl:[168,131,36] },
  { t: 1000, fr:[48,131,35], fl:[36,48,149], br:[49,52,152], bl:[168,131,36] },
  { t: 1042, fr:[48,131,35], fl:[35,48,149], br:[50,52,152], bl:[168,131,36] },
  { t: 1083, fr:[49,131,35], fl:[35,48,149], br:[50,52,152], bl:[169,131,35] },
  { t: 1125, fr:[49,131,35], fl:[35,48,149], br:[50,52,152], bl:[169,131,35] },
  { t: 1167, fr:[50,131,34], fl:[34,48,149], br:[51,52,152], bl:[170,131,35] },
  { t: 1208, fr:[51,131,34], fl:[33,48,149], br:[52,52,152], bl:[171,131,35] },
  { t: 1250, fr:[52,131,34], fl:[32,48,149], br:[53,52,152], bl:[172,131,35] },
  { t: 1292, fr:[53,131,34], fl:[31,48,149], br:[54,52,152], bl:[173,131,35] },
  { t: 1333, fr:[54,131,34], fl:[30,48,149], br:[55,52,152], bl:[174,131,34] },
  { t: 1375, fr:[55,131,33], fl:[28,48,149], br:[57,52,152], bl:[175,131,34] },
  { t: 1417, fr:[57,131,33], fl:[27,48,148], br:[58,52,152], bl:[177,131,34] },
  { t: 1458, fr:[58,131,33], fl:[26,48,148], br:[59,52,152], bl:[178,131,34] },
  { t: 1500, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,34] },
  { t: 1542, fr:[61,132,33], fl:[23,49,148], br:[62,52,151], bl:[181,132,33] },
  { t: 1583, fr:[62,132,33], fl:[22,49,148], br:[63,52,151], bl:[182,132,33] },
  { t: 1625, fr:[63,132,32], fl:[20,49,148], br:[65,52,151], bl:[183,132,33] },
  { t: 1667, fr:[65,132,32], fl:[19,49,147], br:[66,52,151], bl:[185,132,33] },
  { t: 1708, fr:[66,132,32], fl:[18,49,147], br:[67,52,150], bl:[186,132,33] },
  { t: 1750, fr:[67,132,32], fl:[17,49,147], br:[68,52,150], bl:[187,132,33] },
  { t: 1792, fr:[68,132,32], fl:[16,49,147], br:[69,52,150], bl:[188,132,33] },
  { t: 1833, fr:[69,132,32], fl:[15,49,147], br:[70,52,150], bl:[189,132,33] },
  { t: 1875, fr:[70,132,32], fl:[14,49,146], br:[71,52,150], bl:[190,132,33] },
  { t: 1917, fr:[70,132,32], fl:[14,49,146], br:[71,52,150], bl:[190,132,33] },
  { t: 1958, fr:[70,132,32], fl:[13,49,146], br:[72,52,149], bl:[190,132,33] },
  { t: 2000, fr:[71,132,32], fl:[13,49,146], br:[72,52,149], bl:[191,132,33] },
  { t: 2042, fr:[70,132,32], fl:[13,49,146], br:[72,52,149], bl:[190,132,33] },
  { t: 2083, fr:[70,132,32], fl:[13,49,146], br:[72,52,150], bl:[190,132,33] },
  { t: 2125, fr:[70,132,32], fl:[14,49,146], br:[71,52,150], bl:[190,132,33] },
  { t: 2167, fr:[70,132,32], fl:[14,49,146], br:[71,52,150], bl:[190,132,33] },
  { t: 2208, fr:[69,132,32], fl:[14,49,146], br:[71,52,150], bl:[189,132,33] },
  { t: 2250, fr:[69,132,32], fl:[15,49,147], br:[70,52,150], bl:[189,132,33] },
  { t: 2292, fr:[68,132,32], fl:[15,49,147], br:[70,52,150], bl:[188,132,33] },
  { t: 2333, fr:[68,132,32], fl:[16,49,147], br:[69,52,150], bl:[188,132,33] },
  { t: 2375, fr:[67,132,32], fl:[17,49,147], br:[68,52,150], bl:[187,132,33] },
  { t: 2417, fr:[66,132,32], fl:[17,49,147], br:[68,52,150], bl:[186,132,33] },
  { t: 2458, fr:[66,132,32], fl:[18,49,147], br:[67,52,151], bl:[186,132,33] },
  { t: 2500, fr:[65,132,32], fl:[19,49,147], br:[66,52,151], bl:[185,132,33] },
  { t: 2542, fr:[64,132,32], fl:[19,49,147], br:[66,52,151], bl:[184,132,33] },
  { t: 2583, fr:[64,132,32], fl:[20,49,148], br:[65,52,151], bl:[184,132,33] },
  { t: 2625, fr:[63,132,33], fl:[21,49,148], br:[64,52,151], bl:[183,132,33] },
  { t: 2667, fr:[62,132,33], fl:[21,49,148], br:[64,52,151], bl:[182,132,33] },
  { t: 2708, fr:[62,132,33], fl:[22,49,148], br:[63,52,151], bl:[182,132,33] },
  { t: 2750, fr:[61,132,33], fl:[23,49,148], br:[62,52,151], bl:[181,132,33] },
  { t: 2792, fr:[61,132,33], fl:[23,49,148], br:[62,52,151], bl:[181,132,33] },
  { t: 2833, fr:[60,132,33], fl:[23,49,148], br:[62,52,151], bl:[180,132,33] },
  { t: 2875, fr:[60,131,33], fl:[24,49,148], br:[61,52,151], bl:[180,131,33] },
  { t: 2917, fr:[60,131,33], fl:[24,49,148], br:[61,52,151], bl:[180,131,34] },
  { t: 2958, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,34] },
  { t: 3000, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,34] },
];

// Servos accept 0..180; clamp defensively (extreme poses / a drifted
// convention can push the converted angle out of range).
const clamp = (v) => Math.max(0, Math.min(180, v | 0));

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
    _i = LOOP ? 0 : (CLIP.length - 1);
  }
  const frame = CLIP[_i++];
  if (ws.readyState !== WebSocket.OPEN) return;
  for (const leg of ["fr", "fl", "br", "bl"]) {
    const angles = frame[leg];
    for (let j = 0; j < 3; j++) {
      const msg = { "T": 4, "id": LEG_IDS[leg], "servo_id": j, "a": clamp(angles[j]) };
      ws.send(JSON.stringify(msg));
    }
  }
}

ws.onopen = () => {
  console.log("FaceHugger: connected — playing " + CLIP_NAME + " (" + CLIP.length + " frames). Hold-at-end is on by default; call fhStop() when done.");
  _timer = setInterval(playFrame, FRAME_MS);
};
ws.onerror = (e) => {
  console.error("FaceHugger: WebSocket error. On the robot Wi-Fi (FaceHugger_Net, ws://192.168.4.1:81)? Is this page http:// (NOT https:// — ws:// is blocked from https pages)?", e);
};
ws.onclose = () => { fhStop(); console.log("FaceHugger: socket closed."); };
