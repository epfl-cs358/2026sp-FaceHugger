// FaceHugger clip: lie down and stand up
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
const CLIP_NAME = "lie down and stand up";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
  { t: 42, fr:[91,123,36], fl:[89,48,142], br:[89,67,143], bl:[91,123,29] },
  { t: 83, fr:[91,121,34], fl:[89,50,144], br:[89,70,145], bl:[91,121,27] },
  { t: 125, fr:[91,118,33], fl:[89,53,145], br:[89,72,146], bl:[91,118,26] },
  { t: 167, fr:[90,116,32], fl:[90,55,146], br:[90,74,147], bl:[90,116,25] },
  { t: 208, fr:[90,114,31], fl:[90,57,147], br:[90,76,148], bl:[90,114,24] },
  { t: 250, fr:[90,112,30], fl:[90,59,148], br:[90,79,149], bl:[90,112,23] },
  { t: 292, fr:[90,110,30], fl:[90,61,148], br:[90,81,150], bl:[90,110,22] },
  { t: 333, fr:[90,108,29], fl:[90,63,149], br:[90,83,150], bl:[90,108,22] },
  { t: 375, fr:[90,106,29], fl:[90,65,149], br:[90,84,150], bl:[90,106,22] },
  { t: 417, fr:[90,104,29], fl:[90,67,149], br:[90,86,150], bl:[90,104,22] },
  { t: 458, fr:[90,103,30], fl:[90,68,148], br:[90,88,150], bl:[90,103,22] },
  { t: 500, fr:[90,101,30], fl:[90,70,148], br:[90,89,149], bl:[90,101,23] },
  { t: 542, fr:[90,100,31], fl:[90,71,147], br:[90,91,148], bl:[90,100,24] },
  { t: 583, fr:[90,98,32], fl:[90,73,146], br:[90,92,148], bl:[90,98,24] },
  { t: 625, fr:[90,97,33], fl:[90,74,145], br:[90,93,147], bl:[90,97,25] },
  { t: 667, fr:[90,96,34], fl:[90,75,144], br:[90,94,145], bl:[90,96,27] },
  { t: 708, fr:[90,96,36], fl:[90,75,142], br:[90,95,144], bl:[90,96,28] },
  { t: 750, fr:[90,95,37], fl:[90,76,141], br:[90,95,142], bl:[90,95,30] },
  { t: 792, fr:[90,94,39], fl:[90,77,139], br:[90,96,140], bl:[90,94,32] },
  { t: 833, fr:[90,94,42], fl:[90,77,136], br:[90,96,138], bl:[90,94,34] },
  { t: 875, fr:[90,94,44], fl:[90,77,134], br:[90,96,135], bl:[90,94,37] },
  { t: 917, fr:[90,94,47], fl:[90,77,131], br:[90,96,132], bl:[90,94,40] },
  { t: 958, fr:[90,95,51], fl:[90,76,127], br:[90,96,129], bl:[90,95,43] },
  { t: 1000, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1042, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1083, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1125, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1167, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1208, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1250, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1292, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1333, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1375, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1417, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1458, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1500, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1542, fr:[90,96,54], fl:[90,75,124], br:[90,95,125], bl:[90,96,47] },
  { t: 1583, fr:[90,95,52], fl:[90,76,126], br:[90,96,127], bl:[90,95,45] },
  { t: 1625, fr:[90,94,50], fl:[90,77,128], br:[90,97,129], bl:[90,94,43] },
  { t: 1667, fr:[90,93,49], fl:[90,78,129], br:[90,98,131], bl:[90,93,41] },
  { t: 1708, fr:[90,92,47], fl:[90,79,131], br:[90,98,132], bl:[90,92,40] },
  { t: 1750, fr:[90,91,45], fl:[90,80,133], br:[90,99,134], bl:[90,91,38] },
  { t: 1792, fr:[90,90,44], fl:[90,81,134], br:[90,100,136], bl:[90,90,36] },
  { t: 1833, fr:[90,90,42], fl:[90,81,136], br:[90,101,137], bl:[90,90,35] },
  { t: 1875, fr:[90,89,41], fl:[90,82,137], br:[90,102,139], bl:[90,89,33] },
  { t: 1917, fr:[90,88,39], fl:[90,83,139], br:[90,102,140], bl:[90,88,32] },
  { t: 1958, fr:[90,87,38], fl:[90,84,140], br:[90,103,142], bl:[90,87,30] },
  { t: 2000, fr:[90,87,36], fl:[90,84,142], br:[90,104,143], bl:[90,87,29] },
  { t: 2042, fr:[90,86,35], fl:[90,85,143], br:[90,104,144], bl:[90,86,28] },
  { t: 2083, fr:[90,86,34], fl:[90,85,144], br:[90,105,146], bl:[90,86,26] },
  { t: 2125, fr:[90,85,33], fl:[90,86,145], br:[90,105,147], bl:[90,85,25] },
  { t: 2167, fr:[90,84,31], fl:[90,87,147], br:[90,106,148], bl:[90,84,24] },
  { t: 2208, fr:[90,84,30], fl:[90,87,148], br:[90,107,149], bl:[90,84,23] },
  { t: 2250, fr:[90,83,29], fl:[90,88,149], br:[90,107,150], bl:[90,83,22] },
  { t: 2292, fr:[90,83,28], fl:[90,88,150], br:[90,108,152], bl:[90,83,20] },
  { t: 2333, fr:[90,82,27], fl:[90,89,151], br:[90,108,153], bl:[90,82,19] },
  { t: 2375, fr:[90,82,25], fl:[90,89,153], br:[90,109,154], bl:[90,82,18] },
  { t: 2417, fr:[90,81,24], fl:[90,90,154], br:[90,109,155], bl:[90,81,17] },
  { t: 2458, fr:[90,81,23], fl:[90,90,155], br:[90,110,156], bl:[90,81,16] },
  { t: 2500, fr:[90,80,23], fl:[90,91,155], br:[90,110,157], bl:[90,80,15] },
  { t: 2542, fr:[90,80,23], fl:[90,91,155], br:[90,110,157], bl:[90,80,15] },
  { t: 2583, fr:[90,80,23], fl:[90,91,155], br:[90,110,157], bl:[90,80,15] },
  { t: 2625, fr:[90,81,23], fl:[90,90,155], br:[90,110,157], bl:[90,81,15] },
  { t: 2667, fr:[90,81,23], fl:[90,90,155], br:[90,110,157], bl:[90,81,15] },
  { t: 2708, fr:[90,81,23], fl:[90,90,155], br:[90,110,157], bl:[90,81,15] },
  { t: 2750, fr:[90,81,23], fl:[90,90,155], br:[90,110,157], bl:[90,81,15] },
  { t: 2792, fr:[90,81,23], fl:[90,90,155], br:[90,109,157], bl:[90,81,15] },
  { t: 2833, fr:[90,82,23], fl:[90,89,155], br:[90,108,157], bl:[90,82,15] },
  { t: 2875, fr:[90,84,23], fl:[90,87,155], br:[90,107,157], bl:[90,84,15] },
  { t: 2917, fr:[90,85,23], fl:[90,86,155], br:[90,105,157], bl:[90,85,15] },
  { t: 2958, fr:[90,87,23], fl:[90,84,155], br:[90,104,157], bl:[90,87,15] },
  { t: 3000, fr:[90,88,23], fl:[90,83,155], br:[90,102,157], bl:[90,88,15] },
  { t: 3042, fr:[90,90,23], fl:[90,81,155], br:[90,101,157], bl:[90,90,15] },
  { t: 3083, fr:[90,91,23], fl:[90,80,155], br:[90,99,157], bl:[90,91,15] },
  { t: 3125, fr:[90,93,23], fl:[90,78,155], br:[90,98,157], bl:[90,93,15] },
  { t: 3167, fr:[90,94,23], fl:[90,77,155], br:[90,96,157], bl:[90,94,15] },
  { t: 3208, fr:[90,96,23], fl:[90,75,155], br:[90,95,157], bl:[90,96,15] },
  { t: 3250, fr:[90,97,23], fl:[90,74,155], br:[90,93,157], bl:[90,97,15] },
  { t: 3292, fr:[90,99,23], fl:[90,72,155], br:[90,92,157], bl:[90,99,15] },
  { t: 3333, fr:[90,100,23], fl:[90,71,155], br:[90,90,157], bl:[90,100,15] },
  { t: 3375, fr:[90,101,23], fl:[90,70,155], br:[90,89,157], bl:[90,101,15] },
  { t: 3417, fr:[90,103,23], fl:[90,68,155], br:[90,88,157], bl:[90,103,15] },
  { t: 3458, fr:[90,104,23], fl:[90,67,155], br:[90,86,157], bl:[90,104,15] },
  { t: 3500, fr:[90,105,23], fl:[90,66,155], br:[90,85,157], bl:[90,105,15] },
  { t: 3542, fr:[90,106,23], fl:[90,65,155], br:[90,84,157], bl:[90,106,15] },
  { t: 3583, fr:[90,108,23], fl:[90,63,155], br:[90,83,157], bl:[90,108,15] },
  { t: 3625, fr:[91,109,23], fl:[89,62,155], br:[89,82,157], bl:[91,109,15] },
  { t: 3667, fr:[91,111,24], fl:[89,60,154], br:[89,80,156], bl:[91,111,16] },
  { t: 3708, fr:[91,112,25], fl:[89,59,153], br:[89,78,154], bl:[91,112,18] },
  { t: 3750, fr:[91,114,27], fl:[89,57,151], br:[89,76,153], bl:[91,114,19] },
  { t: 3792, fr:[91,116,28], fl:[89,55,150], br:[89,74,151], bl:[91,116,21] },
  { t: 3833, fr:[91,118,30], fl:[89,53,148], br:[89,72,149], bl:[91,118,23] },
  { t: 3875, fr:[91,120,32], fl:[89,51,146], br:[89,71,148], bl:[91,120,24] },
  { t: 3917, fr:[91,122,34], fl:[89,49,144], br:[89,69,146], bl:[91,122,26] },
  { t: 3958, fr:[91,124,36], fl:[89,47,142], br:[89,67,144], bl:[91,124,28] },
  { t: 4000, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
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
