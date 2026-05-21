// FaceHugger clip: wave
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
const CLIP_NAME = "wave";

// Blender leg name -> firmware LegId (see movements.h enum LegId on
// origin/main). The wire `id` field is this leg_id; the firmware maps
// to a PCA channel via LEG_SERVO_CHANNEL[id][servo_id].
const LEG_IDS = { fr: 0, fl: 1, br: 2, bl: 3 };

const CLIP = [
  { t: 0, fr:[59,131,33], fl:[24,49,148], br:[61,52,151], bl:[179,131,34] },
  { t: 42, fr:[59,132,33], fl:[24,49,148], br:[61,52,151], bl:[179,132,34] },
  { t: 83, fr:[59,132,33], fl:[25,49,148], br:[61,52,151], bl:[179,132,34] },
  { t: 125, fr:[59,132,33], fl:[25,50,149], br:[61,52,151], bl:[180,132,34] },
  { t: 167, fr:[59,132,34], fl:[26,51,149], br:[61,51,152], bl:[180,132,34] },
  { t: 208, fr:[58,132,34], fl:[26,53,149], br:[61,51,152], bl:[180,132,34] },
  { t: 250, fr:[58,132,35], fl:[27,54,149], br:[61,51,152], bl:[181,132,34] },
  { t: 292, fr:[57,132,35], fl:[28,56,149], br:[60,50,152], bl:[181,133,34] },
  { t: 333, fr:[57,133,36], fl:[29,58,148], br:[60,50,152], bl:[182,133,35] },
  { t: 375, fr:[56,133,36], fl:[29,61,147], br:[60,50,152], bl:[182,133,35] },
  { t: 417, fr:[56,133,37], fl:[30,63,145], br:[60,49,152], bl:[183,134,35] },
  { t: 458, fr:[55,133,38], fl:[31,64,141], br:[60,49,152], bl:[183,134,36] },
  { t: 500, fr:[55,134,39], fl:[32,64,134], br:[60,48,152], bl:[184,134,36] },
  { t: 542, fr:[54,134,40], fl:[33,62,124], br:[60,47,151], bl:[185,135,36] },
  { t: 583, fr:[54,134,41], fl:[33,55,104], br:[60,47,151], bl:[185,135,37] },
  { t: 625, fr:[53,135,42], fl:[34,58,104], br:[60,46,151], bl:[186,135,37] },
  { t: 667, fr:[52,135,43], fl:[34,61,104], br:[60,46,151], bl:[187,136,38] },
  { t: 708, fr:[52,136,44], fl:[35,64,104], br:[60,45,151], bl:[187,136,38] },
  { t: 750, fr:[51,136,45], fl:[35,66,104], br:[59,44,151], bl:[188,137,39] },
  { t: 792, fr:[51,136,46], fl:[35,69,103], br:[59,44,151], bl:[189,137,39] },
  { t: 833, fr:[50,137,47], fl:[35,72,103], br:[59,43,150], bl:[189,137,40] },
  { t: 875, fr:[50,137,48], fl:[35,74,103], br:[59,43,150], bl:[190,138,40] },
  { t: 917, fr:[49,138,49], fl:[35,77,103], br:[59,42,150], bl:[190,138,40] },
  { t: 958, fr:[49,138,50], fl:[36,80,103], br:[59,42,150], bl:[191,138,41] },
  { t: 1000, fr:[49,138,51], fl:[36,83,103], br:[59,41,150], bl:[191,138,41] },
  { t: 1042, fr:[48,139,52], fl:[36,78,87], br:[59,41,149], bl:[192,139,42] },
  { t: 1083, fr:[48,139,52], fl:[36,75,76], br:[59,40,149], bl:[192,139,42] },
  { t: 1125, fr:[48,139,53], fl:[35,73,69], br:[59,40,149], bl:[192,139,42] },
  { t: 1167, fr:[48,139,53], fl:[34,71,61], br:[59,40,149], bl:[192,139,42] },
  { t: 1208, fr:[48,139,53], fl:[33,69,55], br:[59,40,149], bl:[192,139,42] },
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
