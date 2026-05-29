# Calibration

Calibration ensures every servo's physical position matches what the firmware
believes it to be. Skipping this step will result in uneven standing poses,
asymmetric gaits, and joints that bind at the extremes of their range.

---

## How servos know their position

Servos do not have absolute encoders — they have no way of knowing where they
physically are when powered on. Instead, they respond blindly to PWM pulse widths:
a 1500µs pulse means "go to 90°", a 1000µs pulse means "go to 0°", and so on. The
firmware sends these pulses based on its own internal angle model, defined in
`config.h`.

This means that if a servo horn is pressed onto the spline slightly off-centre —
which is almost inevitable during assembly, since the spline has discrete teeth —
the physical angle and the firmware's assumed angle will be misaligned by one or
more teeth. The robot will stand and walk asymmetrically as a result.

The solution is to **separate the horn from the servo**, command the servo to its
reference position electronically, and only then press the horn back on at the
correct physical angle. This way the firmware's model and reality are guaranteed
to match.

---

## Procedure

!!! warning "Before you start"
    The robot must be fully assembled and powered. Follow the full power-on
    sequence in [Running](software.md#power-on-sequence) before proceeding.
    You will need a small Phillips-head screwdriver.

### Step 1 — Remove all servo horns

With the robot powered and standing, unscrew the centre screw from each of the
12 servo horns. Once the screw is out, gently pull the horn straight off the
spline — it may be tight; wiggle it slightly if needed. Do not use a lever or
excessive force, as the spline teeth are delicate.

!!! tip
    Keep the 12 screws somewhere safe — a small cup or tray works well. They are
    easy to lose.

After this step all 12 servo shafts will be exposed with no horns attached.

### Step 2 — Command the reference pose

In the remote control app, navigate to the **Actions** tab and press
**Reset Pose**. The firmware will send a 1500µs pulse (90°) to all 12 servos
simultaneously. You will hear the servos spin briefly to find their reference
position and then hold.

This is the intended behaviour — the servos spinning freely with no horns
attached is exactly what allows the calibration to work correctly.

### Step 3 — Realign and reattach the horns

With all servos holding their electronic 90° reference, physically reattach each
horn at the correct mechanical angle before screwing it back down.

The target geometry is:

- **Hip joints** — when viewed from above, each hip should be angled at **45°
  relative to the horizontal plane**, pointing diagonally outward from the body
  corners
- **Thigh and knee joints** — the legs should lie **completely flat on the
  table**, fully extended, with no joint bent upward or downward

![Correct calibration pose — top view showing 45° hip angles and flat legs](../assets/img/assembly/Top_overview_with_shell_clean.jpg)

Press each horn onto its spline at the closest tooth position that achieves the
target angle, then drive the centre screw back in finger-tight before giving it a
final quarter turn.

!!! warning "Choose the closest tooth, not the perfect angle"
    The spline has a fixed number of teeth, so you may not be able to hit exactly
    45° or exactly flat. Choose the tooth position that gets closest and accept a
    small residual offset — this can be trimmed later in `config.h` by adjusting
    the default angle for that specific servo channel.

### Step 4 — Verify

Once all 12 horns are reattached, press **Reset Pose** in the app a second time.
The robot should rise into a clean, symmetric standing pose with:

- All four legs extended flat
- The body level and centred
- No joint visibly cocked or twisted relative to its neighbours

If one leg looks obviously wrong, that servo's horn is likely off by one spline
tooth. Unscrew that horn, rotate it by one tooth in the required direction, and
repeat.

---

## Fine-tuning in `config.h`

For residual offsets that cannot be resolved by re-seating the horn (sub-tooth
errors), the default angle for each servo channel can be adjusted directly in
`shared/config.h`. Each channel has a defined default angle that is sent on boot
and on **Reset Pose**. Incrementing or decrementing these values by a few degrees
will trim the standing pose without requiring physical disassembly.