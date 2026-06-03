// FaceHugger clip: one leg lift
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
const CLIP_NAME = "one leg lift";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[90,132,34], fl:[89,50,150], br:[89,52,152], bl:[90,132,34] },
  { t: 83, fr:[89,132,34], fl:[88,52,151], br:[88,53,153], bl:[90,132,35] },
  { t: 125, fr:[89,133,35], fl:[88,54,153], br:[88,54,153], bl:[89,133,35] },
  { t: 167, fr:[88,133,36], fl:[87,56,154], br:[88,54,154], bl:[89,134,36] },
  { t: 208, fr:[87,134,37], fl:[86,57,155], br:[87,55,155], bl:[88,134,36] },
  { t: 250, fr:[87,134,38], fl:[86,59,156], br:[87,55,155], bl:[88,135,37] },
  { t: 292, fr:[86,135,38], fl:[85,61,157], br:[86,56,156], bl:[87,135,38] },
  { t: 333, fr:[85,135,39], fl:[85,62,157], br:[86,56,156], bl:[87,136,39] },
  { t: 375, fr:[85,135,40], fl:[84,63,158], br:[86,57,157], bl:[87,137,39] },
  { t: 417, fr:[84,136,41], fl:[84,65,158], br:[85,57,158], bl:[86,137,40] },
  { t: 458, fr:[83,136,42], fl:[84,66,158], br:[85,58,158], bl:[86,138,41] },
  { t: 500, fr:[83,136,42], fl:[83,67,158], br:[85,58,159], bl:[86,139,42] },
  { t: 542, fr:[82,137,43], fl:[83,68,157], br:[84,58,159], bl:[85,140,43] },
  { t: 583, fr:[82,137,44], fl:[82,69,157], br:[84,59,160], bl:[85,141,44] },
  { t: 625, fr:[82,137,45], fl:[83,72,157], br:[84,58,160], bl:[84,141,45] },
  { t: 667, fr:[82,138,45], fl:[83,74,157], br:[84,58,160], bl:[84,141,45] },
  { t: 708, fr:[83,138,46], fl:[83,76,157], br:[84,58,160], bl:[83,142,46] },
  { t: 750, fr:[83,138,46], fl:[83,77,156], br:[84,57,160], bl:[83,142,46] },
  { t: 792, fr:[84,139,47], fl:[83,79,155], br:[84,57,160], bl:[83,142,47] },
  { t: 833, fr:[84,139,48], fl:[84,80,154], br:[84,57,160], bl:[82,143,48] },
  { t: 875, fr:[84,140,48], fl:[84,81,153], br:[84,56,160], bl:[82,143,48] },
  { t: 917, fr:[85,140,49], fl:[84,82,151], br:[84,56,160], bl:[81,143,49] },
  { t: 958, fr:[85,140,50], fl:[84,83,149], br:[84,55,160], bl:[81,144,49] },
  { t: 1000, fr:[86,141,50], fl:[84,84,147], br:[84,55,160], bl:[80,144,50] },
  { t: 1042, fr:[86,141,51], fl:[84,83,145], br:[84,55,160], bl:[80,145,51] },
  { t: 1083, fr:[87,141,52], fl:[84,83,143], br:[84,54,159], bl:[79,145,51] },
  { t: 1125, fr:[87,142,53], fl:[85,82,141], br:[83,54,159], bl:[79,145,52] },
  { t: 1167, fr:[87,142,54], fl:[85,81,139], br:[83,53,159], bl:[78,146,53] },
  { t: 1208, fr:[88,142,52], fl:[85,83,141], br:[84,53,159], bl:[79,145,51] },
  { t: 1250, fr:[88,141,51], fl:[85,85,142], br:[84,53,159], bl:[79,144,50] },
  { t: 1292, fr:[88,140,50], fl:[86,85,146], br:[84,53,159], bl:[80,143,48] },
  { t: 1333, fr:[88,140,48], fl:[86,86,150], br:[84,53,158], bl:[81,142,47] },
  { t: 1375, fr:[88,139,47], fl:[86,86,153], br:[85,53,158], bl:[81,141,46] },
  { t: 1417, fr:[88,139,46], fl:[86,85,156], br:[85,53,158], bl:[82,140,45] },
  { t: 1458, fr:[89,138,45], fl:[86,84,158], br:[85,53,158], bl:[82,140,44] },
  { t: 1500, fr:[89,138,44], fl:[86,83,160], br:[85,53,157], bl:[83,139,42] },
  { t: 1542, fr:[89,137,43], fl:[87,81,161], br:[86,53,157], bl:[83,138,42] },
  { t: 1583, fr:[89,137,42], fl:[87,80,162], br:[86,53,157], bl:[84,137,41] },
  { t: 1625, fr:[89,136,41], fl:[87,78,163], br:[86,53,156], bl:[85,137,40] },
  { t: 1667, fr:[89,136,40], fl:[87,75,163], br:[86,53,156], bl:[85,136,39] },
  { t: 1708, fr:[90,135,39], fl:[87,73,163], br:[87,53,155], bl:[86,136,38] },
  { t: 1750, fr:[90,135,38], fl:[88,70,163], br:[87,53,155], bl:[86,135,37] },
  { t: 1792, fr:[90,134,38], fl:[88,68,162], br:[87,53,155], bl:[87,134,37] },
  { t: 1833, fr:[90,134,37], fl:[88,65,160], br:[88,53,154], bl:[88,134,36] },
  { t: 1875, fr:[90,133,36], fl:[88,62,159], br:[88,53,154], bl:[88,133,36] },
  { t: 1917, fr:[90,133,35], fl:[89,58,157], br:[88,52,153], bl:[89,133,35] },
  { t: 1958, fr:[90,132,34], fl:[89,55,154], br:[89,52,153], bl:[89,132,34] },
  { t: 2000, fr:[91,132,34], fl:[89,52,151], br:[89,52,152], bl:[90,132,34] },
  { t: 2042, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
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
