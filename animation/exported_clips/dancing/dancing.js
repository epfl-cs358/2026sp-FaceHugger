// FaceHugger clip: dancing
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
const CLIP_NAME = "dancing";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
  { t: 42, fr:[91,127,41], fl:[89,44,137], br:[89,63,139], bl:[91,127,33] },
  { t: 83, fr:[91,129,44], fl:[89,42,134], br:[89,61,136], bl:[91,129,36] },
  { t: 125, fr:[91,131,47], fl:[89,40,131], br:[89,59,132], bl:[91,131,40] },
  { t: 167, fr:[91,133,50], fl:[89,38,128], br:[89,57,129], bl:[91,133,43] },
  { t: 208, fr:[91,136,54], fl:[89,35,124], br:[89,55,125], bl:[91,136,47] },
  { t: 250, fr:[91,138,59], fl:[89,33,119], br:[89,52,120], bl:[91,138,52] },
  { t: 292, fr:[90,134,52], fl:[88,33,120], br:[90,57,127], bl:[92,138,51] },
  { t: 333, fr:[89,130,46], fl:[87,33,121], br:[91,61,133], bl:[93,138,50] },
  { t: 375, fr:[88,126,41], fl:[86,33,122], br:[92,64,138], bl:[94,138,49] },
  { t: 417, fr:[88,122,36], fl:[85,33,123], br:[92,68,143], bl:[95,138,48] },
  { t: 458, fr:[87,119,32], fl:[84,33,124], br:[93,71,147], bl:[96,138,47] },
  { t: 500, fr:[87,116,28], fl:[83,34,125], br:[93,75,151], bl:[97,137,46] },
  { t: 542, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 583, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 625, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 667, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 708, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 750, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 792, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 833, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 875, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 917, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 958, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 1000, fr:[98,140,58], fl:[94,57,151], br:[82,50,121], bl:[86,114,20] },
  { t: 1042, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 1083, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 1125, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 1167, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 1208, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 1250, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 1292, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 1333, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 1375, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 1417, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 1458, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 1500, fr:[87,116,28], fl:[83,34,125], br:[93,75,151], bl:[97,137,46] },
  { t: 1542, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 1583, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 1625, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 1667, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 1708, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 1750, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 1792, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 1833, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 1875, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 1917, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 1958, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 2000, fr:[98,140,58], fl:[94,57,151], br:[82,50,121], bl:[86,114,20] },
  { t: 2042, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 2083, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 2125, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 2167, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 2208, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 2250, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 2292, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 2333, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 2375, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 2417, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 2458, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 2500, fr:[87,116,28], fl:[83,34,125], br:[93,75,151], bl:[97,137,46] },
  { t: 2542, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 2583, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 2625, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 2667, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 2708, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 2750, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 2792, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 2833, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 2875, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 2917, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 2958, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 3000, fr:[98,140,58], fl:[94,57,151], br:[82,50,121], bl:[86,114,20] },
  { t: 3042, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 3083, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 3125, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 3167, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 3208, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 3250, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 3292, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 3333, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 3375, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 3417, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 3458, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 3500, fr:[87,116,28], fl:[83,34,125], br:[93,75,151], bl:[97,137,46] },
  { t: 3542, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 3583, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 3625, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 3667, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 3708, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 3750, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 3792, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 3833, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 3875, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 3917, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 3958, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 4000, fr:[98,140,58], fl:[94,57,151], br:[82,50,121], bl:[86,114,20] },
  { t: 4042, fr:[97,140,58], fl:[93,54,147], br:[83,50,121], bl:[87,117,24] },
  { t: 4083, fr:[96,140,58], fl:[93,50,143], br:[84,51,121], bl:[87,121,28] },
  { t: 4125, fr:[94,139,58], fl:[92,47,139], br:[86,51,121], bl:[88,124,32] },
  { t: 4167, fr:[93,139,58], fl:[91,43,134], br:[87,51,122], bl:[89,128,37] },
  { t: 4208, fr:[92,138,58], fl:[91,39,129], br:[88,52,122], bl:[89,132,42] },
  { t: 4250, fr:[91,138,57], fl:[90,35,123], br:[89,53,122], bl:[90,136,48] },
  { t: 4292, fr:[90,133,51], fl:[89,35,123], br:[90,57,129], bl:[91,136,48] },
  { t: 4333, fr:[89,129,45], fl:[88,34,123], br:[91,61,134], bl:[92,137,47] },
  { t: 4375, fr:[88,126,40], fl:[87,34,124], br:[92,65,139], bl:[93,137,47] },
  { t: 4417, fr:[88,122,36], fl:[85,34,124], br:[92,68,143], bl:[95,137,47] },
  { t: 4458, fr:[87,119,32], fl:[84,34,124], br:[93,72,147], bl:[96,137,46] },
  { t: 4500, fr:[87,116,28], fl:[83,34,125], br:[93,75,151], bl:[97,137,46] },
  { t: 4542, fr:[87,116,29], fl:[84,35,126], br:[93,74,150], bl:[96,136,44] },
  { t: 4583, fr:[87,117,30], fl:[85,36,128], br:[93,73,149], bl:[95,135,43] },
  { t: 4625, fr:[88,118,31], fl:[85,37,129], br:[92,72,148], bl:[95,134,41] },
  { t: 4667, fr:[88,119,32], fl:[86,38,131], br:[92,71,148], bl:[94,133,40] },
  { t: 4708, fr:[88,120,33], fl:[86,39,132], br:[92,70,147], bl:[94,132,38] },
  { t: 4750, fr:[89,121,33], fl:[87,40,134], br:[91,69,146], bl:[93,131,37] },
  { t: 4792, fr:[89,122,34], fl:[87,41,135], br:[91,68,145], bl:[93,130,36] },
  { t: 4833, fr:[89,123,35], fl:[88,42,136], br:[91,67,144], bl:[92,129,34] },
  { t: 4875, fr:[90,124,36], fl:[88,43,138], br:[90,67,143], bl:[92,128,33] },
  { t: 4917, fr:[90,125,37], fl:[89,44,139], br:[90,66,142], bl:[91,127,32] },
  { t: 4958, fr:[91,125,38], fl:[89,46,140], br:[89,65,141], bl:[91,125,31] },
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
