<!-- Sections below are suggestions, not requirements - keep what's useful, drop or merge the rest. -->
# Electronics & Wiring

!!! todo "Stub - to be written"
    How everything is wired: power, servo channels, ESP32 pinout, and the
    circuit schematic.

!!! warning "Pinout may change"
    The exact pinout is not final. Treat pin assignments here as the current
    proposal, not a contract. Confirm against the firmware `config.h` before wiring.

## Schematic

!!! todo
    Export the circuit from KiCad as SVG and embed it here. Walk through it
    block by block (power, ESP32, servo bus, sensors).

## Power distribution

!!! todo
    Battery -> regulation -> servos / ESP32. Current budget and protection.

## Servo wiring

!!! todo
    Which servo connects to which channel/pin. Map to servo IDs in
    [servo conventions](../reference/conventions.md).

## ESP32 pinout

!!! todo
    Pin table. Flag clearly as provisional (see warning above).

## Battery & connector safety

!!! danger "Read before powering on"
    !!! todo
        LiPo handling, polarity, connector keying, when to connect power (last).
