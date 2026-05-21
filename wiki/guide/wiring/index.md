# Electronics & Wiring

!!! todo "Stub — to be written"
    How everything is wired: servo channels, power distribution, and the ESP32
    pinout. The circuit diagram is on the [Schematic](schematic.md) page.

!!! warning "Pinout may change"
    The exact pinout is not final and may change as the hardware evolves.
    Treat the pin assignments here as the current proposal, not a contract —
    confirm against the firmware `config.h` before wiring.

## Power distribution

!!! todo
    Battery → regulation → servos / ESP32. Current budget and protection.

## Servo wiring

!!! todo
    Which servo connects to which channel/pin. Map to servo IDs in
    [servo conventions](../../reference/firmware/servo-conventions.md).

## ESP32 pinout

!!! todo
    Pin table. Flag clearly as provisional (see warning above).

!!! danger "Battery & connector safety"
    !!! todo
        LiPo handling, polarity, connector keying, what not to do.
