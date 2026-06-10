# AGENTS.md

Guidance for cloud agents and automated development environments working on this repository.

## Project overview

ESP32 embedded firmware (PlatformIO + Arduino) for the **Spartan 3 v2 Motorraum Display**. The primary environment is `motorraum` (ESP32 Dev + 16×2 I2C LCD + WiFi setup AP + BLE hub). A secondary variant is `m5_motorraum` (M5Stack display).

There is no host-side server, Docker stack, npm/pip application runtime, or automated test suite. The firmware binary **is** the application; it runs on physical ESP32 hardware after flash.

## Prerequisites

- Python 3
- PlatformIO Core (`pip install --user platformio`)
- USB serial access to an ESP32 board for flash/monitor (not available in typical cloud VMs)

Use PlatformIO via either:

```bash
export PATH="$HOME/.local/bin:$PATH"
pio run -e motorraum
```

or (no PATH change needed):

```bash
python3 -m platformio run -e motorraum
```

First build downloads the Espressif 32 platform, Xtensa toolchain, Arduino framework, and libraries into `~/.platformio/` and `.pio/`.

## Common commands

| Task | Command |
|------|---------|
| Build (default) | `python3 -m platformio run -e motorraum` |
| Build M5 variant | `python3 -m platformio run -e m5_motorraum` |
| Flash | `python3 -m platformio run -e motorraum -t upload --upload-port /dev/ttyUSB0` |
| Serial monitor | `python3 -m platformio device monitor --port /dev/ttyUSB0 --baud 115200` |
| Memory report | `python3 -m platformio run -e motorraum -t size` |
| List USB devices | `python3 -m platformio device list` |

Adjust the upload/monitor port for your OS (`COM9` on Windows per `platformio.ini`, `/dev/ttyUSB0` or `/dev/ttyACM0` on Linux).

## Lint and tests

- **Lint:** Not configured (no `.clang-format`, `clang-tidy`, cppcheck, or CI).
- **Tests:** No `test/` tree or PlatformIO test environments. Validation is manual on hardware (serial log, LCD, `http://192.168.4.1/state`, BLE clients). See `docs/quick-start-tomorrow.md`.

In cloud VMs without hardware, **a successful `motorraum` build** is the primary dev-environment check.

## Hardware runtime (full E2E)

After flashing `motorraum` to an ESP32 Dev board:

1. Power the board; connect a 16×2 I2C LCD on GPIO 21/22 (address `0x27`).
2. Join WiFi AP **Spartan3-Setup** / password **lambda123**.
3. Open **http://192.168.4.1/** for the setup dashboard and live `/state` JSON.
4. With `ENABLE_SPARTAN_DEMO=1` (default in `platformio.ini`), the LCD shows simulated Lambda values tagged `DEMO`.
5. Serial monitor at 115200 baud shows boot diagnostics and BLE hub address.

Full vehicle/gateway testing additionally needs Spartan 3 v2 + CAN transceiver, and optionally 123TUNE+ / M5 Dial BLE clients.

## Build flags

Compile-time options live in `platformio.ini` under each environment's `build_flags` (e.g. `ENABLE_SPARTAN_DEMO`, `ENABLE_WEB_GUI`, `ENABLE_BLE_HUB`). The `m5_motorraum` environment disables Web GUI and BLE hub.

## Known build note

As of setup verification, `motorraum` builds cleanly. `m5_motorraum` may fail with undeclared `runTuneReadDump` / `sendTuneCommand` in `updateUart()` when BLE hub code is disabled — treat as a separate firmware fix if that environment is needed.

## Cursor Cloud specific instructions

- **No long-running host service** to start; skip `docker compose`, `npm dev`, etc.
- **PATH:** PlatformIO installs to `~/.local/bin`. Prefer `python3 -m platformio` in scripts to avoid PATH issues.
- **Cloud VM limitation:** Without an ESP32 on USB, agents can build and inspect `/.pio/build/motorraum/firmware.bin` but cannot flash, run serial monitor, or hit the on-device Web GUI. Confirm builds with `python3 -m platformio run -e motorraum`.
- **Artifacts:** Release firmware is at `.pio/build/motorraum/firmware.bin` (~1.1 MB). Embedded strings include `Spartan3-Setup`, `Spartan3-Hub`, and the demo Web GUI HTML.
- **Upload port in `platformio.ini`:** Defaults to `COM9` (Windows bench setup). Override with `--upload-port` on Linux/macOS.
- **Dependencies refresh:** `pip install --user platformio` plus any `pio run` invocation re-syncs PlatformIO packages and `lib_deps` automatically; no separate install script exists in the repo.
