// FaceHugger clip: lie down and stand up
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
const CLIP_NAME = "lie down and stand up";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,130,32], fl:[89,50,149], br:[89,54,152], bl:[91,130,33] },
  { t: 83, fr:[91,128,32], fl:[89,52,149], br:[89,55,153], bl:[91,128,32] },
  { t: 125, fr:[91,127,31], fl:[89,53,150], br:[89,57,153], bl:[91,127,32] },
  { t: 167, fr:[90,125,31], fl:[90,55,150], br:[90,58,153], bl:[90,125,32] },
  { t: 208, fr:[90,124,31], fl:[90,56,150], br:[90,60,153], bl:[90,124,32] },
  { t: 250, fr:[90,122,31], fl:[90,58,150], br:[90,61,153], bl:[90,122,32] },
  { t: 292, fr:[90,121,31], fl:[90,59,150], br:[90,62,153], bl:[90,121,32] },
  { t: 333, fr:[90,120,32], fl:[90,60,149], br:[90,63,153], bl:[90,120,32] },
  { t: 375, fr:[90,119,32], fl:[90,61,149], br:[90,64,152], bl:[90,119,33] },
  { t: 417, fr:[90,118,33], fl:[90,62,148], br:[90,65,151], bl:[90,118,34] },
  { t: 458, fr:[90,117,34], fl:[90,63,147], br:[90,66,151], bl:[90,117,34] },
  { t: 500, fr:[90,116,35], fl:[90,64,146], br:[90,67,150], bl:[90,116,35] },
  { t: 542, fr:[90,116,36], fl:[90,64,145], br:[90,68,149], bl:[90,116,36] },
  { t: 583, fr:[90,115,37], fl:[90,65,144], br:[90,68,147], bl:[90,115,38] },
  { t: 625, fr:[90,114,39], fl:[90,66,142], br:[90,69,146], bl:[90,114,39] },
  { t: 667, fr:[90,114,40], fl:[90,66,141], br:[90,69,144], bl:[90,114,41] },
  { t: 708, fr:[90,114,42], fl:[90,66,139], br:[90,69,142], bl:[90,114,43] },
  { t: 750, fr:[90,114,44], fl:[90,66,137], br:[90,69,140], bl:[90,114,45] },
  { t: 792, fr:[90,114,47], fl:[90,66,134], br:[90,69,138], bl:[90,114,47] },
  { t: 833, fr:[90,114,49], fl:[90,66,132], br:[90,69,135], bl:[90,114,50] },
  { t: 875, fr:[90,115,53], fl:[90,65,128], br:[90,68,132], bl:[90,115,53] },
  { t: 917, fr:[90,116,57], fl:[90,64,124], br:[90,67,128], bl:[90,116,57] },
  { t: 958, fr:[90,118,62], fl:[90,62,119], br:[90,66,123], bl:[90,118,62] },
  { t: 1000, fr:[90,121,70], fl:[90,59,111], br:[90,63,115], bl:[90,121,70] },
  { t: 1042, fr:[90,118,66], fl:[90,62,115], br:[90,65,118], bl:[90,118,67] },
  { t: 1083, fr:[90,116,63], fl:[90,64,118], br:[90,67,121], bl:[90,116,64] },
  { t: 1125, fr:[90,114,61], fl:[90,66,120], br:[90,69,123], bl:[90,114,62] },
  { t: 1167, fr:[90,113,59], fl:[90,67,122], br:[90,71,125], bl:[90,113,60] },
  { t: 1208, fr:[90,111,57], fl:[90,69,124], br:[90,72,127], bl:[90,111,58] },
  { t: 1250, fr:[90,110,56], fl:[90,70,125], br:[90,74,128], bl:[90,110,57] },
  { t: 1292, fr:[90,108,55], fl:[90,72,126], br:[90,75,130], bl:[90,108,55] },
  { t: 1333, fr:[90,107,54], fl:[90,73,127], br:[90,76,131], bl:[90,107,54] },
  { t: 1375, fr:[90,106,52], fl:[90,74,129], br:[90,77,132], bl:[90,106,53] },
  { t: 1417, fr:[90,105,52], fl:[90,75,129], br:[90,79,133], bl:[90,105,52] },
  { t: 1458, fr:[90,104,51], fl:[90,76,130], br:[90,80,134], bl:[90,104,51] },
  { t: 1500, fr:[90,103,50], fl:[90,77,131], br:[90,81,134], bl:[90,103,51] },
  { t: 1542, fr:[90,102,49], fl:[90,78,132], br:[90,82,135], bl:[90,102,50] },
  { t: 1583, fr:[90,100,46], fl:[90,80,135], br:[90,83,138], bl:[90,100,47] },
  { t: 1625, fr:[90,98,43], fl:[90,82,138], br:[90,85,141], bl:[90,98,44] },
  { t: 1667, fr:[90,97,40], fl:[90,83,141], br:[90,86,144], bl:[90,97,41] },
  { t: 1708, fr:[90,96,38], fl:[90,84,143], br:[90,88,147], bl:[90,96,38] },
  { t: 1750, fr:[90,94,35], fl:[90,86,146], br:[90,89,149], bl:[90,94,36] },
  { t: 1792, fr:[90,93,33], fl:[90,87,148], br:[90,90,152], bl:[90,93,33] },
  { t: 1833, fr:[90,92,30], fl:[90,88,151], br:[90,91,154], bl:[90,92,31] },
  { t: 1875, fr:[90,91,28], fl:[90,89,153], br:[90,92,156], bl:[90,91,29] },
  { t: 1917, fr:[90,90,26], fl:[90,90,155], br:[90,93,158], bl:[90,90,27] },
  { t: 1958, fr:[90,89,24], fl:[90,91,157], br:[90,94,160], bl:[90,89,25] },
  { t: 2000, fr:[90,88,22], fl:[90,92,159], br:[90,95,162], bl:[90,88,23] },
  { t: 2042, fr:[90,87,20], fl:[90,93,161], br:[90,96,164], bl:[90,87,21] },
  { t: 2083, fr:[90,87,18], fl:[90,93,163], br:[90,97,166], bl:[90,87,19] },
  { t: 2125, fr:[90,86,18], fl:[90,94,163], br:[90,97,167], bl:[90,86,18] },
  { t: 2167, fr:[90,87,18], fl:[90,93,163], br:[90,97,167], bl:[90,87,18] },
  { t: 2208, fr:[90,87,18], fl:[90,93,163], br:[90,97,167], bl:[90,87,18] },
  { t: 2250, fr:[90,87,18], fl:[90,93,163], br:[90,97,167], bl:[90,87,18] },
  { t: 2292, fr:[90,89,18], fl:[90,91,163], br:[90,94,167], bl:[90,89,18] },
  { t: 2333, fr:[90,91,18], fl:[90,89,163], br:[90,92,167], bl:[90,91,18] },
  { t: 2375, fr:[90,94,18], fl:[90,86,163], br:[90,90,167], bl:[90,94,18] },
  { t: 2417, fr:[90,96,18], fl:[90,84,163], br:[90,87,167], bl:[90,96,18] },
  { t: 2458, fr:[90,99,18], fl:[90,81,163], br:[90,85,167], bl:[90,99,18] },
  { t: 2500, fr:[90,101,18], fl:[90,79,163], br:[90,82,167], bl:[90,101,18] },
  { t: 2542, fr:[90,103,18], fl:[90,77,163], br:[90,80,167], bl:[90,103,18] },
  { t: 2583, fr:[90,106,18], fl:[90,74,163], br:[90,78,167], bl:[90,106,18] },
  { t: 2625, fr:[90,108,18], fl:[90,72,163], br:[90,76,167], bl:[90,108,18] },
  { t: 2667, fr:[90,110,18], fl:[90,70,163], br:[90,73,167], bl:[90,110,18] },
  { t: 2708, fr:[90,112,18], fl:[90,68,163], br:[90,71,167], bl:[90,112,18] },
  { t: 2750, fr:[90,114,18], fl:[90,66,163], br:[90,69,167], bl:[90,114,18] },
  { t: 2792, fr:[91,116,19], fl:[89,64,162], br:[89,67,165], bl:[91,116,20] },
  { t: 2833, fr:[91,119,21], fl:[89,61,160], br:[89,64,163], bl:[91,119,22] },
  { t: 2875, fr:[91,122,24], fl:[89,58,157], br:[89,61,161], bl:[91,122,24] },
  { t: 2917, fr:[91,125,26], fl:[89,55,155], br:[89,58,158], bl:[91,125,27] },
  { t: 2958, fr:[91,128,30], fl:[89,52,151], br:[89,55,155], bl:[91,128,30] },
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
