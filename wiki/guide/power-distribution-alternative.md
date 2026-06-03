# Alternative Power Distribution: Copper Board

!!! info "Context - why this page exists"
    This page documents an alternative power distribution setup that was explored
    during development but was not carried forward into the final build. It is
    preserved here so that future builders who run into the same power distribution
    problems we faced have a concrete, validated alternative to reference. It is
    one of several approaches that could work, not a recommendation.

---

## The problem

The standard approach of routing servo power through the PCA9685's onboard V+
and GND traces has a fundamental limitation: those traces are not rated for the
combined current draw of 12 high-torque servos under load. Under aggressive
movement or simultaneous multi-leg actuation, this causes voltage drops, brownouts,
and in the worst case, permanent damage to the multiplexer board. In the end, we burnt 
two PCA Multiplexers.

The question is how to deliver raw LiPo current to all 12 servos reliably,
without routing it through the PCA9685 traces, while keeping the assembly compact
enough to fit inside the shell.

---

## The solution: custom copper power board

The approach taken here was to fabricate a small custom power distribution board
using copper stripboard. The board acts as a dedicated power bus, sitting between
the BMS output and the servo connectors, completely bypassing the PCA9685's power
traces. 

**Key design decisions:**

- **Leg blocks spaced to mirror the PCA9685 pin layout** - the servo connector
  positions on the copper board match the physical layout of the multiplexer,
  keeping cable runs short and tidy
- **BMS power cables soldered directly to the board** - eliminates an intermediate
  connector and reduces resistance in the high-current path
- **Only two rows of pin headers soldered** - V+ and GND only; PWM is handled
  separately (see below)
- **PWM routed independently** - each servo's PWM signal wire connects directly
  from the servo's female connector to the PCA9685 using a male-to-female jumper,
  keeping the signal path entirely separate from the power path
- **GND bridge between copper board and PCA9685** - a dedicated GND wire ties the
  copper board's ground plane to the PCA9685's GND, ensuring a common reference
  for the PWM signals. This is easy to forget and causes erratic servo behaviour
  if omitted

---

## Build notes

This is not a step-by-step guide, but the following points are worth knowing
before attempting this approach:

- Plan the strip layout carefully before soldering - cut the copper strips between
  V+ and GND rows to prevent shorts before any wire is attached
- Test continuity and check for shorts with a multimeter before connecting the BMS
- The board is compact enough that the shell fits over the assembly without
  modification
- Cable management becomes more involved with this setup due to the additional
  PWM jumper wires - allow extra time for routing and securing them

Here are a few pictures of the setup :
[Top of copper board](../assets/img/copper-board/top_board.jpg)
[Bottom of copper board](../assets/img/copper-board/bottom_board.jpg)
[Full setup top view](../assets/img/copper-board/full_setup.jpg)

---

## What we learned

The setup was validated by running all previously working functionality after the
rebuild and everything behaved correctly. The main takeaways for anyone considering
this approach:

- It works, but it adds complexity to the wiring compared to using terminal blocks
- The GND bridge between the copper board and the PCA9685 is **not optional** -
  without it the PWM signals have no common ground reference and servo behaviour
  is undefined
- Spacing the connector rows to match the PCA9685 layout is worth the extra
  planning time - it makes the final wiring significantly cleaner
- Other valid alternatives exist: Wago connectors, Domino terminal blocks, or a
  purpose-made PCB would all solve the same problem with different trade-offs in
  cost, complexity, and space

---

## See also

- [Parts & Materials - Electronics](parts.md#electronics) for the recommended
  final build power distribution approach
- [Assembly - Body + Sensors](assembly.md) for how power wiring fits into the
  full build sequence