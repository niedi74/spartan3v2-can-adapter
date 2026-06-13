# norbi-espnow — Spartan 3 v2 Cockpit Monorepo

Branch-per-project layout for Motorraum hub and cockpit displays.

| Branch | Project | Board | Role |
|--------|---------|-------|------|
| `main` | This README | — | Monorepo index |
| `hub` | Motorraum ESP32 hub | ESP32 (`motorraum`) | CAN Lambda, 123TUNE BLE, ESP-NOW broadcast |
| `m5` | M5Stack Dial cockpit | M5 StampS3 | Round display, gateway/direct 123TUNE |
| `waveshare` | Waveshare 2.8″ round | ESP32-S3 | VDO clock + ESP-NOW cockpit |

## Clone one project

```bash
git clone -b hub https://github.com/niedi74/norbi-espnow.git spartan-hub
cd spartan-hub && pio run -e motorraum

git clone -b m5 https://github.com/niedi74/norbi-espnow.git m5-cockpit
cd m5-cockpit && pio run -e m5stack-stamps3

git clone -b waveshare https://github.com/niedi74/norbi-espnow.git waveshare-cockpit
cd waveshare-cockpit && pio run -e waveshare_round
```

## Shared protocol

`include/spartan_cockpit_frame.h` on branch `hub` — 14-byte ESP-NOW frame, CRC-8, channel 6.

## Legacy repos

| Repo | Monorepo branch |
|------|-----------------|
| `spartan3v2-can-adapter` | `hub` |
| `m5stack-123` (`feature/spartan-live-display`) | `m5` |
| `waveshare-vdo-clock` | `waveshare` |

## Initial push

From `spartan3v2-can-adapter` on `main`:

```bash
bash scripts/push-norbi-espnow.sh
```
