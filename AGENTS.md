# AGENTS.md

## Cursor Cloud specific instructions

### Product

ESP32 firmware (PlatformIO / Arduino) for the Spartan 3 v2 motor-bay display and BLE gateway. There is no host-side web server, database, or Docker stack — the ESP32 **is** the runtime.

### Build targets

| Environment | Board | Default? | Notes |
|-------------|-------|----------|-------|
| `motorraum` | `esp32dev` | Yes | Web GUI (`Spartan3-Setup` AP), BLE hub (`Spartan3-Hub`), demo mode |
| `m5_motorraum` | `m5stack-core-esp32` | No | M5 display variant; Web GUI and BLE hub disabled |

See `README.md` for wiring, CAN/UART details, and flash commands.

### Cloud VM workflow

1. **Build (default target):** `pio run` or `pio run -e motorraum`
2. **Build M5 target:** `pio run -e m5_motorraum` — currently fails to compile because `updateUart()` calls BLE-only helpers (`runTuneReadDump`, `sendTuneCommand`) while `ENABLE_BLE_HUB=0` in that environment. Fix requires a small `#if ENABLE_BLE_HUB` guard in `src/main.cpp`.
3. **Flash (local hardware only):** `pio run -e motorraum -t upload --upload-port <PORT>` — `platformio.ini` defaults to `COM9`; override the port on Linux (e.g. `/dev/ttyUSB0`).
4. **Serial monitor:** `pio device monitor --baud 115200 --port <PORT>`

### What runs in the cloud vs on hardware

| Action | Cloud VM | With ESP32 attached |
|--------|----------|---------------------|
| Compile firmware | Yes | Yes |
| Flash firmware | No (no USB device) | Yes |
| Web GUI at `192.168.4.1` | No | Yes (join `Spartan3-Setup` / `lambda123`) |
| BLE hub / LCD / CAN | No | Yes |

`pio device list` returns empty in the cloud VM — expected.

### Lint / tests

No automated lint or unit-test targets are configured. **Compile success for `motorraum` is the primary CI check** for this repo.

### Bench hello-world (hardware)

After flashing `motorraum` with demo mode (`ENABLE_SPARTAN_DEMO=1` in `platformio.ini`):

1. Power the ESP32 and connect the 16×2 I2C LCD (GPIO 21/22, address `0x27`).
2. LCD should show simulated lambda, e.g. `LAM 1.023 DEMO` / `780C OK`.
3. Join WiFi AP `Spartan3-Setup` (password `lambda123`) and open `http://192.168.4.1/` — live `/state` JSON with demo lambda readings.
