# Flashing the firmware

The firmware is a PlatformIO project targeting the `upesy_wroom` ESP32 board.
Build, flash, and open the serial monitor with the three commands below. Run them
from the `code/firmware/` directory.

```bash
cd code/firmware
pio run -e upesy_wroom
pio run -t upload
pio device monitor
```

`pio device monitor` opens a 115200-baud serial console with the PlatformIO
exception decoder. Watch for FSM state transitions and any `[WARN] servo <ch>
clamped` lines during testing.

## Connecting to the robot

Once flashed, the ESP32 starts a Wi-Fi access point. Join it from your laptop or
phone before sending commands.

- Network: `FaceHugger_Net`
- Password: `12345678`
- WebSocket endpoint: `ws://192.168.4.1:81`

Commands are JSON objects sent over the WebSocket. For example, open a browser
console on a non-HTTPS page and run:

```js
const ws = new WebSocket("ws://192.168.4.1:81");
ws.send(JSON.stringify({ T: 2, s: 0 }));   // transition to IDLE
```

See [Reference - API](../../reference/api.md) for the full command protocol.
