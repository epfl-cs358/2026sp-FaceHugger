// FaceHugger clip: wave
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
const CLIP_NAME = "wave";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
  { t: 42, fr:[91,126,39], fl:[89,47,138], br:[89,64,141], bl:[90,126,31] },
  { t: 83, fr:[91,126,40], fl:[88,49,136], br:[89,64,141], bl:[90,127,32] },
  { t: 125, fr:[92,127,41], fl:[88,49,133], br:[89,63,140], bl:[90,127,32] },
  { t: 167, fr:[92,127,41], fl:[88,50,129], br:[89,63,140], bl:[89,128,33] },
  { t: 208, fr:[92,128,42], fl:[88,49,124], br:[89,62,139], bl:[89,128,34] },
  { t: 250, fr:[93,128,43], fl:[87,48,117], br:[90,61,139], bl:[89,129,35] },
  { t: 292, fr:[93,129,44], fl:[87,44,105], br:[90,61,138], bl:[88,129,35] },
  { t: 333, fr:[93,129,45], fl:[87,40,96], br:[90,60,138], bl:[88,130,36] },
  { t: 375, fr:[94,130,46], fl:[87,42,96], br:[90,60,137], bl:[88,130,37] },
  { t: 417, fr:[94,130,48], fl:[87,44,96], br:[90,59,137], bl:[87,131,37] },
  { t: 458, fr:[94,131,49], fl:[86,47,96], br:[90,58,136], bl:[87,131,38] },
  { t: 500, fr:[95,132,50], fl:[86,50,96], br:[90,58,135], bl:[86,132,39] },
  { t: 542, fr:[95,132,51], fl:[86,53,96], br:[90,57,135], bl:[86,133,40] },
  { t: 583, fr:[95,133,52], fl:[86,56,96], br:[90,56,134], bl:[86,133,41] },
  { t: 625, fr:[96,134,54], fl:[86,59,96], br:[90,56,134], bl:[85,134,42] },
  { t: 667, fr:[96,134,55], fl:[86,62,96], br:[90,55,133], bl:[85,134,43] },
  { t: 708, fr:[96,135,56], fl:[85,65,95], br:[90,54,132], bl:[85,135,44] },
  { t: 750, fr:[97,136,58], fl:[85,67,95], br:[90,54,132], bl:[84,136,44] },
  { t: 792, fr:[97,136,58], fl:[85,69,95], br:[90,54,132], bl:[84,136,44] },
  { t: 833, fr:[97,136,58], fl:[85,71,95], br:[90,54,132], bl:[84,136,44] },
  { t: 875, fr:[98,136,59], fl:[84,73,95], br:[90,53,132], bl:[83,136,44] },
  { t: 917, fr:[98,136,59], fl:[84,75,95], br:[90,53,133], bl:[83,136,44] },
  { t: 958, fr:[98,136,59], fl:[84,77,95], br:[90,53,133], bl:[82,135,44] },
  { t: 1000, fr:[99,136,59], fl:[83,79,95], br:[90,53,133], bl:[82,135,44] },
  { t: 1042, fr:[99,136,60], fl:[83,81,95], br:[91,53,133], bl:[82,135,44] },
  { t: 1083, fr:[99,136,60], fl:[82,83,95], br:[91,53,134], bl:[81,135,44] },
  { t: 1125, fr:[100,136,60], fl:[82,85,95], br:[91,53,134], bl:[81,135,44] },
  { t: 1167, fr:[100,136,61], fl:[81,88,95], br:[91,53,134], bl:[81,135,43] },
  { t: 1208, fr:[100,136,61], fl:[81,90,95], br:[91,52,134], bl:[80,135,43] },
  { t: 1250, fr:[101,136,61], fl:[80,92,95], br:[91,52,135], bl:[80,135,43] },
  { t: 1292, fr:[101,136,62], fl:[80,94,95], br:[91,52,135], bl:[80,135,43] },
  { t: 1333, fr:[101,136,62], fl:[79,96,95], br:[91,52,135], bl:[79,135,43] },
  { t: 1375, fr:[101,136,62], fl:[79,98,95], br:[91,52,135], bl:[79,135,43] },
  { t: 1417, fr:[102,136,63], fl:[78,100,95], br:[91,52,135], bl:[79,135,43] },
  { t: 1458, fr:[102,136,63], fl:[77,101,95], br:[91,51,136], bl:[78,135,43] },
  { t: 1500, fr:[102,136,64], fl:[77,103,95], br:[91,51,136], bl:[78,135,43] },
  { t: 1542, fr:[102,136,62], fl:[77,103,95], br:[91,52,136], bl:[78,135,42] },
  { t: 1583, fr:[102,135,61], fl:[78,103,95], br:[91,52,137], bl:[77,134,42] },
  { t: 1625, fr:[101,135,60], fl:[79,104,95], br:[90,52,137], bl:[77,134,41] },
  { t: 1667, fr:[101,134,59], fl:[80,104,95], br:[90,53,138], bl:[77,134,40] },
  { t: 1708, fr:[101,133,58], fl:[81,104,95], br:[90,53,139], bl:[76,133,40] },
  { t: 1750, fr:[101,133,57], fl:[82,104,95], br:[89,53,139], bl:[76,133,39] },
  { t: 1792, fr:[100,132,56], fl:[83,104,95], br:[89,54,140], bl:[76,132,38] },
  { t: 1833, fr:[100,132,55], fl:[84,104,95], br:[89,54,140], bl:[76,132,38] },
  { t: 1875, fr:[100,131,55], fl:[84,104,95], br:[88,54,141], bl:[75,132,37] },
  { t: 1917, fr:[100,131,54], fl:[85,104,95], br:[88,55,141], bl:[75,131,37] },
  { t: 1958, fr:[99,130,53], fl:[86,104,95], br:[88,55,142], bl:[75,131,36] },
  { t: 2000, fr:[99,130,52], fl:[87,104,95], br:[87,55,142], bl:[74,131,35] },
  { t: 2042, fr:[99,130,51], fl:[87,104,95], br:[87,56,143], bl:[74,130,35] },
  { t: 2083, fr:[98,129,51], fl:[88,105,95], br:[87,56,143], bl:[74,130,34] },
  { t: 2125, fr:[98,129,50], fl:[88,105,95], br:[86,56,144], bl:[74,130,34] },
  { t: 2167, fr:[98,128,49], fl:[89,105,95], br:[86,57,144], bl:[73,129,33] },
  { t: 2208, fr:[98,128,48], fl:[89,105,95], br:[85,57,145], bl:[73,129,33] },
  { t: 2250, fr:[97,128,48], fl:[89,105,95], br:[85,57,145], bl:[73,128,32] },
  { t: 2292, fr:[97,127,47], fl:[90,106,95], br:[85,58,146], bl:[72,128,32] },
  { t: 2333, fr:[97,127,46], fl:[90,106,95], br:[84,58,146], bl:[72,128,31] },
  { t: 2375, fr:[96,126,46], fl:[91,106,95], br:[84,58,147], bl:[72,127,31] },
  { t: 2417, fr:[96,126,45], fl:[91,106,95], br:[84,59,147], bl:[72,127,30] },
  { t: 2458, fr:[96,126,44], fl:[91,106,95], br:[83,59,148], bl:[71,127,30] },
  { t: 2500, fr:[96,125,44], fl:[92,107,95], br:[83,59,148], bl:[71,126,29] },
  { t: 2542, fr:[96,125,44], fl:[91,106,95], br:[83,59,148], bl:[71,126,29] },
  { t: 2583, fr:[96,125,44], fl:[91,106,95], br:[84,59,148], bl:[72,126,29] },
  { t: 2625, fr:[97,125,44], fl:[91,106,95], br:[84,59,148], bl:[72,126,29] },
  { t: 2667, fr:[97,125,44], fl:[90,106,95], br:[85,59,148], bl:[72,126,29] },
  { t: 2708, fr:[97,125,44], fl:[90,106,95], br:[85,59,148], bl:[73,126,29] },
  { t: 2750, fr:[98,125,44], fl:[90,106,95], br:[86,59,148], bl:[73,127,29] },
  { t: 2792, fr:[98,125,44], fl:[89,106,95], br:[86,59,148], bl:[73,127,29] },
  { t: 2833, fr:[98,125,44], fl:[89,106,95], br:[87,59,148], bl:[74,127,29] },
  { t: 2875, fr:[99,125,45], fl:[89,106,95], br:[87,59,148], bl:[74,127,29] },
  { t: 2917, fr:[99,125,45], fl:[88,105,95], br:[87,59,148], bl:[75,127,29] },
  { t: 2958, fr:[100,125,45], fl:[88,105,95], br:[88,59,149], bl:[75,127,29] },
  { t: 3000, fr:[100,125,45], fl:[88,105,95], br:[88,59,149], bl:[75,127,29] },
  { t: 3042, fr:[100,125,45], fl:[87,105,95], br:[89,59,149], bl:[76,127,29] },
  { t: 3083, fr:[101,125,45], fl:[86,105,95], br:[89,59,149], bl:[76,127,29] },
  { t: 3125, fr:[101,125,45], fl:[85,105,95], br:[90,59,149], bl:[76,127,29] },
  { t: 3167, fr:[101,125,45], fl:[85,105,95], br:[90,59,149], bl:[77,127,29] },
  { t: 3208, fr:[102,125,45], fl:[84,105,95], br:[90,59,149], bl:[77,127,29] },
  { t: 3250, fr:[102,125,46], fl:[83,105,95], br:[91,59,149], bl:[77,127,29] },
  { t: 3292, fr:[102,125,46], fl:[82,105,95], br:[91,59,149], bl:[78,127,29] },
  { t: 3333, fr:[103,125,46], fl:[81,105,95], br:[92,59,149], bl:[78,127,29] },
  { t: 3375, fr:[103,125,46], fl:[81,105,95], br:[92,58,149], bl:[79,127,29] },
  { t: 3417, fr:[103,125,46], fl:[80,105,95], br:[93,58,149], bl:[79,127,29] },
  { t: 3458, fr:[104,125,46], fl:[79,105,95], br:[93,58,149], bl:[79,127,29] },
  { t: 3500, fr:[104,125,46], fl:[78,105,95], br:[94,58,149], bl:[80,127,29] },
  { t: 3542, fr:[104,125,46], fl:[78,105,95], br:[94,58,149], bl:[80,127,29] },
  { t: 3583, fr:[104,125,46], fl:[77,105,95], br:[94,58,149], bl:[80,127,29] },
  { t: 3625, fr:[105,125,46], fl:[77,105,95], br:[94,58,149], bl:[80,127,29] },
  { t: 3667, fr:[105,125,46], fl:[76,105,95], br:[95,58,149], bl:[80,127,29] },
  { t: 3708, fr:[105,125,47], fl:[76,105,95], br:[95,58,149], bl:[81,127,29] },
  { t: 3750, fr:[105,125,47], fl:[75,105,95], br:[95,58,149], bl:[81,127,29] },
  { t: 3792, fr:[105,125,47], fl:[75,105,95], br:[95,58,149], bl:[81,127,29] },
  { t: 3833, fr:[106,125,47], fl:[74,105,95], br:[96,58,149], bl:[81,127,29] },
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
