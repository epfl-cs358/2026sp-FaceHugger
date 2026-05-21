# Setup

!!! todo "Stub - to be written"
    Getting the toolchain installed and the firmware flashed. Commands adapted
    from `CLAUDE.md` and `code/firmware/`.

## PlatformIO

!!! todo
    Install PlatformIO, open the `code/firmware/` project.

```bash
pio run                 # build for the upesy_wroom ESP32
pio run -t upload       # flash
pio device monitor      # 115200 baud
```

## Flashing the ESP32

!!! todo
    Board selection, USB driver notes, first flash.

## Host-side tools (optional)

!!! todo
    Python env for the simulation (`uv` / conda for PyBullet), Blender for
    animation. Cross-reference [Running it](running.md).
