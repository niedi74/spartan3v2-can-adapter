# Waveshare 2.8″ Round Cockpit / VDO Clock

ESP32-S3 round display: ESP-NOW listener for Spartan hub cockpit data, VDO/T2B clock screensaver.

## Build

```bash
pio run -e waveshare_round
pio run -e waveshare_round -t upload
```

## Hub link

- Keep `include/spartan_cockpit_frame.h` in sync with `norbi-espnow` branch `hub`.
- WiFi channel **6** (same as `Spartan3-Setup` AP).
- See `docs/espnow-gateway-architecture.md`.

## Assets

T2B/VDO reference clock faces in `assets/t2b-clock/` (copied from hub on push).
