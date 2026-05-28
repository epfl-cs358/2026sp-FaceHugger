// FaceHugger clip: tiny wiggle
// Generated 2026-05-25 from Blender animation
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
const CLIP_NAME = "tiny wiggle";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 83, fr:[91,131,33], fl:[90,49,148], br:[90,52,151], bl:[91,131,34] },
  { t: 125, fr:[91,131,33], fl:[90,49,148], br:[90,52,152], bl:[91,131,34] },
  { t: 167, fr:[92,131,33], fl:[90,48,148], br:[90,52,152], bl:[92,131,34] },
  { t: 208, fr:[92,131,33], fl:[91,48,148], br:[91,52,152], bl:[92,131,34] },
  { t: 250, fr:[92,131,33], fl:[91,48,148], br:[91,52,152], bl:[92,131,34] },
  { t: 292, fr:[93,131,33], fl:[92,48,148], br:[92,52,152], bl:[93,131,34] },
  { t: 333, fr:[94,131,33], fl:[92,48,148], br:[92,52,152], bl:[94,131,34] },
  { t: 375, fr:[94,131,33], fl:[93,48,148], br:[93,52,152], bl:[94,131,34] },
  { t: 417, fr:[95,131,33], fl:[94,48,149], br:[94,52,152], bl:[95,131,34] },
  { t: 458, fr:[96,131,34], fl:[94,48,149], br:[94,52,152], bl:[96,131,34] },
  { t: 500, fr:[96,131,34], fl:[95,48,149], br:[95,52,152], bl:[96,131,34] },
  { t: 542, fr:[97,131,34], fl:[96,48,149], br:[96,52,152], bl:[97,131,34] },
  { t: 583, fr:[98,131,34], fl:[96,48,149], br:[96,52,152], bl:[98,131,35] },
  { t: 625, fr:[98,131,34], fl:[97,48,149], br:[97,52,152], bl:[98,131,35] },
  { t: 667, fr:[99,131,34], fl:[98,48,149], br:[98,52,152], bl:[99,131,35] },
  { t: 708, fr:[100,131,34], fl:[98,48,149], br:[98,52,152], bl:[100,131,35] },
  { t: 750, fr:[100,131,34], fl:[99,48,149], br:[99,52,152], bl:[100,131,35] },
  { t: 792, fr:[101,131,35], fl:[99,48,149], br:[99,52,152], bl:[101,131,35] },
  { t: 833, fr:[101,131,35], fl:[100,48,149], br:[100,52,152], bl:[101,131,35] },
  { t: 875, fr:[101,131,35], fl:[100,48,149], br:[100,52,152], bl:[101,131,35] },
  { t: 917, fr:[102,131,35], fl:[100,48,149], br:[100,52,152], bl:[102,131,35] },
  { t: 958, fr:[102,131,35], fl:[100,48,149], br:[100,52,152], bl:[102,131,36] },
  { t: 1000, fr:[102,131,35], fl:[101,48,149], br:[101,52,152], bl:[102,131,36] },
  { t: 1042, fr:[102,131,35], fl:[100,48,149], br:[100,52,152], bl:[102,131,36] },
  { t: 1083, fr:[101,131,35], fl:[100,48,149], br:[100,52,152], bl:[101,131,35] },
  { t: 1125, fr:[101,131,35], fl:[100,48,149], br:[100,52,152], bl:[101,131,35] },
  { t: 1167, fr:[100,131,34], fl:[99,48,149], br:[99,52,152], bl:[100,131,35] },
  { t: 1208, fr:[99,131,34], fl:[98,48,149], br:[98,52,152], bl:[99,131,35] },
  { t: 1250, fr:[98,131,34], fl:[97,48,149], br:[97,52,152], bl:[98,131,35] },
  { t: 1292, fr:[97,131,34], fl:[96,48,149], br:[96,52,152], bl:[97,131,35] },
  { t: 1333, fr:[96,131,34], fl:[95,48,149], br:[95,52,152], bl:[96,131,34] },
  { t: 1375, fr:[95,131,33], fl:[93,48,149], br:[93,52,152], bl:[95,131,34] },
  { t: 1417, fr:[93,131,33], fl:[92,48,148], br:[92,52,152], bl:[93,131,34] },
  { t: 1458, fr:[92,131,33], fl:[91,48,148], br:[91,52,152], bl:[92,131,34] },
  { t: 1500, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 1542, fr:[89,132,33], fl:[88,49,148], br:[88,52,151], bl:[89,132,33] },
  { t: 1583, fr:[88,132,33], fl:[87,49,148], br:[87,52,151], bl:[88,132,33] },
  { t: 1625, fr:[87,132,32], fl:[85,49,148], br:[85,52,151], bl:[87,132,33] },
  { t: 1667, fr:[85,132,32], fl:[84,49,147], br:[84,52,151], bl:[85,132,33] },
  { t: 1708, fr:[84,132,32], fl:[83,49,147], br:[83,52,150], bl:[84,132,33] },
  { t: 1750, fr:[83,132,32], fl:[82,49,147], br:[82,52,150], bl:[83,132,33] },
  { t: 1792, fr:[82,132,32], fl:[81,49,147], br:[81,52,150], bl:[82,132,33] },
  { t: 1833, fr:[81,132,32], fl:[80,49,147], br:[80,52,150], bl:[81,132,33] },
  { t: 1875, fr:[80,132,32], fl:[79,49,146], br:[79,52,150], bl:[80,132,33] },
  { t: 1917, fr:[80,132,32], fl:[79,49,146], br:[79,52,150], bl:[80,132,33] },
  { t: 1958, fr:[80,132,32], fl:[78,49,146], br:[78,52,149], bl:[80,132,33] },
  { t: 2000, fr:[79,132,32], fl:[78,49,146], br:[78,52,149], bl:[79,132,33] },
  { t: 2042, fr:[80,132,32], fl:[78,49,146], br:[78,52,149], bl:[80,132,33] },
  { t: 2083, fr:[80,132,32], fl:[78,49,146], br:[78,52,150], bl:[80,132,33] },
  { t: 2125, fr:[80,132,32], fl:[79,49,146], br:[79,52,150], bl:[80,132,33] },
  { t: 2167, fr:[80,132,32], fl:[79,49,146], br:[79,52,150], bl:[80,132,33] },
  { t: 2208, fr:[81,132,32], fl:[79,49,146], br:[79,52,150], bl:[81,132,33] },
  { t: 2250, fr:[81,132,32], fl:[80,49,147], br:[80,52,150], bl:[81,132,33] },
  { t: 2292, fr:[82,132,32], fl:[80,49,147], br:[80,52,150], bl:[82,132,33] },
  { t: 2333, fr:[82,132,32], fl:[81,49,147], br:[81,52,150], bl:[82,132,33] },
  { t: 2375, fr:[83,132,32], fl:[82,49,147], br:[82,52,150], bl:[83,132,33] },
  { t: 2417, fr:[84,132,32], fl:[82,49,147], br:[82,52,150], bl:[84,132,33] },
  { t: 2458, fr:[84,132,32], fl:[83,49,147], br:[83,52,151], bl:[84,132,33] },
  { t: 2500, fr:[85,132,32], fl:[84,49,147], br:[84,52,151], bl:[85,132,33] },
  { t: 2542, fr:[86,132,32], fl:[84,49,147], br:[84,52,151], bl:[86,132,33] },
  { t: 2583, fr:[86,132,32], fl:[85,49,148], br:[85,52,151], bl:[86,132,33] },
  { t: 2625, fr:[87,132,33], fl:[86,49,148], br:[86,52,151], bl:[87,132,33] },
  { t: 2667, fr:[88,132,33], fl:[86,49,148], br:[86,52,151], bl:[88,132,33] },
  { t: 2708, fr:[88,132,33], fl:[87,49,148], br:[87,52,151], bl:[88,132,33] },
  { t: 2750, fr:[89,132,33], fl:[88,49,148], br:[88,52,151], bl:[89,132,33] },
  { t: 2792, fr:[89,132,33], fl:[88,49,148], br:[88,52,151], bl:[89,132,33] },
  { t: 2833, fr:[90,132,33], fl:[88,49,148], br:[88,52,151], bl:[90,132,33] },
  { t: 2875, fr:[90,131,33], fl:[89,49,148], br:[89,52,151], bl:[90,131,33] },
  { t: 2917, fr:[90,131,33], fl:[89,49,148], br:[89,52,151], bl:[90,131,34] },
  { t: 2958, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
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
