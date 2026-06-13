# AGENTS.md

## Cursor Cloud / Agent Einstieg

### Produkt

ESP32-Firmware (PlatformIO / Arduino) für Spartan 3 v2 Motorraum-Hub: CAN Lambda, 123TUNE+ BLE, ESP-NOW Cockpit-Broadcast, Web-GUI Setup.

**Aktueller Stand:** siehe `docs/STATUS_2026-06-13.md`  
**Branch:** `main` (einzige aktive Linie)

### Build

| Environment | Board | Default | Notes |
|-------------|-------|---------|-------|
| `motorraum` | `esp32dev` | Ja | Web GUI, 123TUNE BLE, ESP-NOW, BM6, COM16 |
| `motorraum_s3_devkitc` | ESP32-S3 | Nein | S3 Bring-up, andere Pins |
| `m5_motorraum` | M5Stack Core | Nein | BLE/Web aus |

```bash
pio run -e motorraum
pio run -e motorraum -t upload --upload-port COM16   # lokal
pio device monitor --port COM16 --baud 115200
```

### Wichtige Dateien

- `src/main.cpp` — gesamte Firmware
- `include/spartan_cockpit_frame.h` — ESP-NOW Frame (auch für M5/Waveshare)
- `platformio.ini` — Pins, Feature-Flags
- `docs/road-live-debug.md` — Fahrt-Debug über Handy-Hotspot

### Verwandte Repos

| Repo | Branch |
|------|--------|
| `m5stack-123` | `feature/spartan-live-display` |
| `waveshare-vdo-clock` | `cursor/webgui-ota-c56e` |
