// FaceHugger clip: lie down and stand up
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
const CLIP_NAME = "lie down and stand up";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,129,31], fl:[89,51,150], br:[89,54,153], bl:[91,129,32] },
  { t: 83, fr:[91,127,29], fl:[89,53,152], br:[89,57,155], bl:[91,127,30] },
  { t: 125, fr:[91,124,28], fl:[89,56,153], br:[89,59,156], bl:[91,124,29] },
  { t: 167, fr:[90,122,27], fl:[90,58,154], br:[90,61,157], bl:[90,122,28] },
  { t: 208, fr:[90,120,26], fl:[90,60,155], br:[90,63,158], bl:[90,120,27] },
  { t: 250, fr:[90,118,25], fl:[90,62,156], br:[90,66,159], bl:[90,118,26] },
  { t: 292, fr:[90,116,25], fl:[90,64,156], br:[90,68,160], bl:[90,116,25] },
  { t: 333, fr:[90,114,24], fl:[90,66,157], br:[90,70,160], bl:[90,114,25] },
  { t: 375, fr:[90,112,24], fl:[90,68,157], br:[90,71,160], bl:[90,112,25] },
  { t: 417, fr:[90,110,24], fl:[90,70,157], br:[90,73,160], bl:[90,110,25] },
  { t: 458, fr:[90,109,25], fl:[90,71,156], br:[90,75,160], bl:[90,109,25] },
  { t: 500, fr:[90,107,25], fl:[90,73,156], br:[90,76,159], bl:[90,107,26] },
  { t: 542, fr:[90,106,26], fl:[90,74,155], br:[90,78,158], bl:[90,106,27] },
  { t: 583, fr:[90,104,27], fl:[90,76,154], br:[90,79,158], bl:[90,104,27] },
  { t: 625, fr:[90,103,28], fl:[90,77,153], br:[90,80,157], bl:[90,103,28] },
  { t: 667, fr:[90,102,29], fl:[90,78,152], br:[90,81,155], bl:[90,102,30] },
  { t: 708, fr:[90,102,31], fl:[90,78,150], br:[90,82,154], bl:[90,102,31] },
  { t: 750, fr:[90,101,32], fl:[90,79,149], br:[90,82,152], bl:[90,101,33] },
  { t: 792, fr:[90,100,34], fl:[90,80,147], br:[90,83,150], bl:[90,100,35] },
  { t: 833, fr:[90,100,37], fl:[90,80,144], br:[90,83,148], bl:[90,100,37] },
  { t: 875, fr:[90,100,39], fl:[90,80,142], br:[90,83,145], bl:[90,100,40] },
  { t: 917, fr:[90,100,42], fl:[90,80,139], br:[90,83,142], bl:[90,100,43] },
  { t: 958, fr:[90,101,46], fl:[90,79,135], br:[90,83,139], bl:[90,101,46] },
  { t: 1000, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1042, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1083, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1125, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1167, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1208, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1250, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1292, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1333, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1375, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1417, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1458, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1500, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1542, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1583, fr:[90,101,47], fl:[90,79,134], br:[90,83,137], bl:[90,101,48] },
  { t: 1625, fr:[90,100,45], fl:[90,80,136], br:[90,84,139], bl:[90,100,46] },
  { t: 1667, fr:[90,99,44], fl:[90,81,137], br:[90,85,141], bl:[90,99,44] },
  { t: 1708, fr:[90,98,42], fl:[90,82,139], br:[90,85,142], bl:[90,98,43] },
  { t: 1750, fr:[90,97,40], fl:[90,83,141], br:[90,86,144], bl:[90,97,41] },
  { t: 1792, fr:[90,96,39], fl:[90,84,142], br:[90,87,146], bl:[90,96,39] },
  { t: 1833, fr:[90,96,37], fl:[90,84,144], br:[90,88,147], bl:[90,96,38] },
  { t: 1875, fr:[90,95,36], fl:[90,85,145], br:[90,89,149], bl:[90,95,36] },
  { t: 1917, fr:[90,94,34], fl:[90,86,147], br:[90,89,150], bl:[90,94,35] },
  { t: 1958, fr:[90,93,33], fl:[90,87,148], br:[90,90,152], bl:[90,93,33] },
  { t: 2000, fr:[90,93,31], fl:[90,87,150], br:[90,91,153], bl:[90,93,32] },
  { t: 2042, fr:[90,92,30], fl:[90,88,151], br:[90,91,154], bl:[90,92,31] },
  { t: 2083, fr:[90,92,29], fl:[90,88,152], br:[90,92,156], bl:[90,92,29] },
  { t: 2125, fr:[90,91,28], fl:[90,89,153], br:[90,92,157], bl:[90,91,28] },
  { t: 2167, fr:[90,90,26], fl:[90,90,155], br:[90,93,158], bl:[90,90,27] },
  { t: 2208, fr:[90,90,25], fl:[90,90,156], br:[90,94,159], bl:[90,90,26] },
  { t: 2250, fr:[90,89,24], fl:[90,91,157], br:[90,94,160], bl:[90,89,25] },
  { t: 2292, fr:[90,89,23], fl:[90,91,158], br:[90,95,162], bl:[90,89,23] },
  { t: 2333, fr:[90,88,22], fl:[90,92,159], br:[90,95,163], bl:[90,88,22] },
  { t: 2375, fr:[90,88,20], fl:[90,92,161], br:[90,96,164], bl:[90,88,21] },
  { t: 2417, fr:[90,87,19], fl:[90,93,162], br:[90,96,165], bl:[90,87,20] },
  { t: 2458, fr:[90,87,18], fl:[90,93,163], br:[90,97,166], bl:[90,87,19] },
  { t: 2500, fr:[90,86,18], fl:[90,94,163], br:[90,97,167], bl:[90,86,18] },
  { t: 2542, fr:[90,86,18], fl:[90,94,163], br:[90,97,167], bl:[90,86,18] },
  { t: 2583, fr:[90,86,18], fl:[90,94,163], br:[90,97,167], bl:[90,86,18] },
  { t: 2625, fr:[90,87,18], fl:[90,93,163], br:[90,97,167], bl:[90,87,18] },
  { t: 2667, fr:[90,87,18], fl:[90,93,163], br:[90,97,167], bl:[90,87,18] },
  { t: 2708, fr:[90,87,18], fl:[90,93,163], br:[90,97,167], bl:[90,87,18] },
  { t: 2750, fr:[90,87,18], fl:[90,93,163], br:[90,97,167], bl:[90,87,18] },
  { t: 2792, fr:[90,87,18], fl:[90,93,163], br:[90,96,167], bl:[90,87,18] },
  { t: 2833, fr:[90,88,18], fl:[90,92,163], br:[90,95,167], bl:[90,88,18] },
  { t: 2875, fr:[90,90,18], fl:[90,90,163], br:[90,94,167], bl:[90,90,18] },
  { t: 2917, fr:[90,91,18], fl:[90,89,163], br:[90,92,167], bl:[90,91,18] },
  { t: 2958, fr:[90,93,18], fl:[90,87,163], br:[90,91,167], bl:[90,93,18] },
  { t: 3000, fr:[90,94,18], fl:[90,86,163], br:[90,89,167], bl:[90,94,18] },
  { t: 3042, fr:[90,96,18], fl:[90,84,163], br:[90,88,167], bl:[90,96,18] },
  { t: 3083, fr:[90,97,18], fl:[90,83,163], br:[90,86,167], bl:[90,97,18] },
  { t: 3125, fr:[90,99,18], fl:[90,81,163], br:[90,85,167], bl:[90,99,18] },
  { t: 3167, fr:[90,100,18], fl:[90,80,163], br:[90,83,167], bl:[90,100,18] },
  { t: 3208, fr:[90,102,18], fl:[90,78,163], br:[90,82,167], bl:[90,102,18] },
  { t: 3250, fr:[90,103,18], fl:[90,77,163], br:[90,80,167], bl:[90,103,18] },
  { t: 3292, fr:[90,105,18], fl:[90,75,163], br:[90,79,167], bl:[90,105,18] },
  { t: 3333, fr:[90,106,18], fl:[90,74,163], br:[90,77,167], bl:[90,106,18] },
  { t: 3375, fr:[90,107,18], fl:[90,73,163], br:[90,76,167], bl:[90,107,18] },
  { t: 3417, fr:[90,109,18], fl:[90,71,163], br:[90,75,167], bl:[90,109,18] },
  { t: 3458, fr:[90,110,18], fl:[90,70,163], br:[90,73,167], bl:[90,110,18] },
  { t: 3500, fr:[90,111,18], fl:[90,69,163], br:[90,72,167], bl:[90,111,18] },
  { t: 3542, fr:[90,112,18], fl:[90,68,163], br:[90,71,167], bl:[90,112,18] },
  { t: 3583, fr:[90,114,18], fl:[90,66,163], br:[90,70,167], bl:[90,114,18] },
  { t: 3625, fr:[91,115,18], fl:[89,65,163], br:[89,69,167], bl:[91,115,18] },
  { t: 3667, fr:[91,117,19], fl:[89,63,162], br:[89,67,166], bl:[91,117,19] },
  { t: 3708, fr:[91,118,20], fl:[89,62,161], br:[89,65,164], bl:[91,118,21] },
  { t: 3750, fr:[91,120,22], fl:[89,60,159], br:[89,63,163], bl:[91,120,22] },
  { t: 3792, fr:[91,122,23], fl:[89,58,158], br:[89,61,161], bl:[91,122,24] },
  { t: 3833, fr:[91,124,25], fl:[89,56,156], br:[89,59,159], bl:[91,124,26] },
  { t: 3875, fr:[91,126,27], fl:[89,54,154], br:[89,58,158], bl:[91,126,27] },
  { t: 3917, fr:[91,128,29], fl:[89,52,152], br:[89,56,156], bl:[91,128,29] },
  { t: 3958, fr:[91,130,31], fl:[89,50,150], br:[89,54,154], bl:[91,130,31] },
  { t: 4000, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
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
