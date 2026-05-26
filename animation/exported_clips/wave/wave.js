// FaceHugger clip: wave
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
const CLIP_NAME = "wave";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 42, fr:[91,132,33], fl:[89,49,148], br:[89,52,151], bl:[91,132,34] },
  { t: 83, fr:[91,132,33], fl:[90,49,148], br:[89,52,151], bl:[91,132,34] },
  { t: 125, fr:[91,132,33], fl:[90,50,149], br:[89,52,151], bl:[90,132,34] },
  { t: 167, fr:[91,132,34], fl:[91,51,149], br:[89,51,152], bl:[90,132,34] },
  { t: 208, fr:[92,132,34], fl:[91,53,149], br:[89,51,152], bl:[90,132,34] },
  { t: 250, fr:[92,132,35], fl:[92,54,149], br:[89,51,152], bl:[89,132,34] },
  { t: 292, fr:[93,132,35], fl:[93,56,149], br:[90,50,152], bl:[89,133,34] },
  { t: 333, fr:[93,133,36], fl:[94,58,148], br:[90,50,152], bl:[88,133,35] },
  { t: 375, fr:[94,133,36], fl:[94,61,147], br:[90,50,152], bl:[88,133,35] },
  { t: 417, fr:[94,133,37], fl:[95,63,145], br:[90,49,152], bl:[87,134,35] },
  { t: 458, fr:[95,133,38], fl:[96,64,141], br:[90,49,152], bl:[87,134,36] },
  { t: 500, fr:[95,134,39], fl:[97,64,134], br:[90,48,152], bl:[86,134,36] },
  { t: 542, fr:[96,134,40], fl:[98,62,124], br:[90,47,151], bl:[85,135,36] },
  { t: 583, fr:[96,134,41], fl:[98,55,104], br:[90,47,151], bl:[85,135,37] },
  { t: 625, fr:[97,135,42], fl:[99,58,104], br:[90,46,151], bl:[84,135,37] },
  { t: 667, fr:[98,135,43], fl:[99,61,104], br:[90,46,151], bl:[83,136,38] },
  { t: 708, fr:[98,136,44], fl:[100,64,104], br:[90,45,151], bl:[83,136,38] },
  { t: 750, fr:[99,136,45], fl:[100,66,104], br:[91,44,151], bl:[82,137,39] },
  { t: 792, fr:[99,136,46], fl:[100,69,103], br:[91,44,151], bl:[81,137,39] },
  { t: 833, fr:[100,137,47], fl:[100,72,103], br:[91,43,150], bl:[81,137,40] },
  { t: 875, fr:[100,137,48], fl:[100,74,103], br:[91,43,150], bl:[80,138,40] },
  { t: 917, fr:[101,138,49], fl:[100,77,103], br:[91,42,150], bl:[80,138,40] },
  { t: 958, fr:[101,138,50], fl:[101,80,103], br:[91,42,150], bl:[79,138,41] },
  { t: 1000, fr:[101,138,51], fl:[101,83,103], br:[91,41,150], bl:[79,138,41] },
  { t: 1042, fr:[102,139,52], fl:[101,78,87], br:[91,41,149], bl:[78,139,42] },
  { t: 1083, fr:[102,139,52], fl:[101,75,76], br:[91,40,149], bl:[78,139,42] },
  { t: 1125, fr:[102,139,53], fl:[100,73,69], br:[91,40,149], bl:[78,139,42] },
  { t: 1167, fr:[102,139,53], fl:[99,71,61], br:[91,40,149], bl:[78,139,42] },
  { t: 1208, fr:[102,139,53], fl:[98,69,55], br:[91,40,149], bl:[78,139,42] },
  { t: 1250, fr:[102,139,53], fl:[96,68,50], br:[91,40,149], bl:[78,139,42] },
  { t: 1292, fr:[102,139,53], fl:[93,67,46], br:[91,40,149], bl:[78,139,42] },
  { t: 1333, fr:[102,139,53], fl:[90,67,43], br:[91,40,149], bl:[78,139,42] },
  { t: 1375, fr:[102,139,53], fl:[87,69,43], br:[91,40,149], bl:[78,139,42] },
  { t: 1417, fr:[102,139,53], fl:[83,70,43], br:[91,40,149], bl:[78,139,42] },
  { t: 1458, fr:[102,139,53], fl:[79,70,43], br:[91,40,149], bl:[78,139,42] },
  { t: 1500, fr:[102,139,52], fl:[75,70,43], br:[91,41,149], bl:[78,139,42] },
  { t: 1542, fr:[102,139,52], fl:[71,70,43], br:[91,41,149], bl:[78,139,42] },
  { t: 1583, fr:[102,139,52], fl:[67,69,43], br:[91,41,149], bl:[79,139,42] },
  { t: 1625, fr:[102,138,51], fl:[64,68,43], br:[91,41,149], bl:[79,138,41] },
  { t: 1667, fr:[101,138,51], fl:[61,68,43], br:[91,41,150], bl:[79,138,41] },
  { t: 1708, fr:[101,138,50], fl:[58,68,46], br:[91,41,150], bl:[79,138,41] },
  { t: 1750, fr:[101,138,50], fl:[57,68,48], br:[91,42,150], bl:[79,138,41] },
  { t: 1792, fr:[101,138,50], fl:[55,69,51], br:[91,42,150], bl:[79,138,41] },
  { t: 1833, fr:[101,138,49], fl:[55,69,53], br:[91,42,150], bl:[80,138,40] },
  { t: 1875, fr:[101,137,49], fl:[55,70,54], br:[91,42,150], bl:[80,138,40] },
  { t: 1917, fr:[100,137,48], fl:[55,70,56], br:[91,43,150], bl:[80,138,40] },
  { t: 1958, fr:[100,137,48], fl:[55,71,57], br:[91,43,150], bl:[80,137,40] },
  { t: 2000, fr:[100,137,47], fl:[56,71,58], br:[91,43,150], bl:[81,137,40] },
  { t: 2042, fr:[100,137,47], fl:[57,71,58], br:[91,43,150], bl:[81,137,39] },
  { t: 2083, fr:[99,136,46], fl:[59,71,59], br:[91,44,151], bl:[81,137,39] },
  { t: 2125, fr:[99,136,46], fl:[61,71,59], br:[91,44,151], bl:[82,137,39] },
  { t: 2167, fr:[99,136,45], fl:[63,72,59], br:[91,44,151], bl:[82,137,39] },
  { t: 2208, fr:[99,136,45], fl:[65,72,59], br:[90,44,151], bl:[82,137,39] },
  { t: 2250, fr:[98,136,44], fl:[67,72,60], br:[90,45,151], bl:[82,136,38] },
  { t: 2292, fr:[98,135,44], fl:[68,72,60], br:[90,45,151], bl:[83,136,38] },
  { t: 2333, fr:[98,135,43], fl:[70,72,61], br:[90,45,151], bl:[83,136,38] },
  { t: 2375, fr:[98,135,43], fl:[72,73,61], br:[90,46,151], bl:[83,136,38] },
  { t: 2417, fr:[97,135,42], fl:[73,73,61], br:[90,46,151], bl:[84,136,37] },
  { t: 2458, fr:[97,135,42], fl:[74,73,60], br:[90,46,151], bl:[84,135,37] },
  { t: 2500, fr:[97,135,41], fl:[75,72,57], br:[90,47,151], bl:[84,135,37] },
  { t: 2542, fr:[97,134,41], fl:[77,71,53], br:[90,47,151], bl:[85,135,37] },
  { t: 2583, fr:[96,134,40], fl:[79,71,48], br:[90,47,151], bl:[85,135,37] },
  { t: 2625, fr:[96,134,40], fl:[81,71,43], br:[90,47,151], bl:[85,135,36] },
  { t: 2667, fr:[96,134,39], fl:[84,73,43], br:[90,48,152], bl:[86,135,36] },
  { t: 2708, fr:[95,134,39], fl:[87,76,43], br:[90,48,152], bl:[86,134,36] },
  { t: 2750, fr:[95,134,39], fl:[91,78,43], br:[90,48,152], bl:[86,134,36] },
  { t: 2792, fr:[95,133,38], fl:[96,80,43], br:[90,48,152], bl:[86,134,36] },
  { t: 2833, fr:[95,133,38], fl:[100,80,43], br:[90,49,152], bl:[87,134,36] },
  { t: 2875, fr:[94,133,37], fl:[103,79,43], br:[90,49,152], bl:[87,134,35] },
  { t: 2917, fr:[94,133,37], fl:[106,76,43], br:[90,49,152], bl:[87,134,35] },
  { t: 2958, fr:[94,133,37], fl:[106,72,43], br:[90,49,152], bl:[88,133,35] },
  { t: 3000, fr:[93,133,36], fl:[104,68,48], br:[90,50,152], bl:[88,133,35] },
  { t: 3042, fr:[93,133,36], fl:[101,68,61], br:[90,50,152], bl:[88,133,35] },
  { t: 3083, fr:[93,133,36], fl:[98,74,84], br:[90,50,152], bl:[88,133,35] },
  { t: 3125, fr:[93,132,35], fl:[96,79,103], br:[90,50,152], bl:[89,133,35] },
  { t: 3167, fr:[93,132,35], fl:[94,75,103], br:[90,50,152], bl:[89,133,34] },
  { t: 3208, fr:[92,132,35], fl:[93,72,103], br:[89,51,152], bl:[89,132,34] },
  { t: 3250, fr:[92,132,35], fl:[93,69,103], br:[89,51,152], bl:[89,132,34] },
  { t: 3292, fr:[92,132,34], fl:[93,66,104], br:[89,51,152], bl:[89,132,34] },
  { t: 3333, fr:[92,132,34], fl:[93,64,104], br:[89,51,152], bl:[90,132,34] },
  { t: 3375, fr:[92,132,34], fl:[93,61,104], br:[89,51,152], bl:[90,132,34] },
  { t: 3417, fr:[91,132,34], fl:[92,57,104], br:[89,51,152], bl:[90,132,34] },
  { t: 3458, fr:[91,132,34], fl:[92,54,104], br:[89,51,152], bl:[90,132,34] },
  { t: 3500, fr:[91,132,33], fl:[92,50,104], br:[89,52,152], bl:[90,132,34] },
  { t: 3542, fr:[91,132,33], fl:[92,55,123], br:[89,52,151], bl:[90,132,34] },
  { t: 3583, fr:[91,132,33], fl:[91,57,135], br:[89,52,151], bl:[90,132,34] },
  { t: 3625, fr:[91,132,33], fl:[91,56,141], br:[89,52,151], bl:[91,132,34] },
  { t: 3667, fr:[91,132,33], fl:[90,53,145], br:[89,52,151], bl:[91,132,34] },
  { t: 3708, fr:[91,132,33], fl:[90,51,147], br:[89,52,151], bl:[91,132,34] },
  { t: 3750, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
  { t: 3792, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
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
