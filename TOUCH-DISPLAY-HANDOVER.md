# Handover: 2.8" ESP32-S3 Touch Display fürs Spartan-Cockpit

> Übergabe-Doku für einen **separaten Chat**. Ziel: ein 2.8"-Touch-Display, das die
> Motordaten anzeigt wie das originale 123TUNE+ Dashboard. Das Display ist ein
> **eigenständiges Gerät** und holt die Daten vom Hub — der Hub selbst bleibt unverändert
> (Single-Purpose-123-Client, siehe unten).

## Status (Stand 2026-06-23) — **HTTP-Poll bereits implementiert und verifiziert**

Die Kernarbeit ist erledigt. Das Display läuft auf Branch **`claude/cranky-proskuriakova-cafad7`**
im Repo **`waveshare-vdo-clock`** (`D:\_claude\waveshare-vdo-clock`). Aktueller Commit:
`9eb22e0 Add graphical cockpit dashboard + HTTP/CAN data paths + WebGUI WiFi entry`.

Was bereits funktioniert (live getestet 2026-06-23):
- Grafisches Cockpit-Dashboard (Zeiger-Instrumente) läuft auf dem Display.
- **HTTP-Poll auf `GET /api/status`** des Hubs liefert Daten: Lambda, RPM, ADV, VOLT, TEMP, AMP.
- CAN-Listen auf `id=0x510` (GPIO43/44, mode=listen) ist bereit.
- WebGUI zum Einstellen der Hub-IP (WLAN-Profil wechseln: AP `192.168.4.1` ↔ Heimnetz).
- Der frühere Architektur-Konflikt (Display versuchte selbst 123-BLE-Client → `err=13`) ist behoben —
  Display holt Daten jetzt **ausschließlich** per HTTP vom Hub.

## Noch offen / nächste Schritte

1. **BLE-Scan im Display abschalten** — das Display scannt noch ins Leere nach `Spartan3-Hub`
   (BLE rx = 0). Da HTTP die Quelle ist, kann der BLE-Client im Display entfernt werden.
2. **CAN von Hub-Seite aktivieren** — Hub sendet noch keine CAN-Frames (kein `0x400`-Sender am Bus).
   Wenn der Spartan/Lambda-Sender läuft: Hub-seitig `rx_err=128` / `source=TEST` werden sich klären.
3. **Gauge-Layout verfeinern** — Rundinstrumente auf dem 480×480-Panel weiter optimieren.
4. **Stromversorgung** — Einbau: 12 V (Auto) → 5 V → Display-Board.

## Firmware-Stand im Repo

- **Repo:** `waveshare-vdo-clock` (separates Repo, NICHT `spartan3v2-can-adapter`)
- **Aktiver Branch:** `claude/cranky-proskuriakova-cafad7`
- **Parallel-Branch:** `cursor/webgui-ota-c56e` (OTA-/WebGUI-Arbeit, Uhr-Fix committed)
- **Veralteter Branch:** `claude/esp32-s3-touch-display-abuiy3` — ältere Variante, nicht mehr relevant

## Architektur (implementiert)

Der Hub (ESP32-S3, Env `motorraum_s3_test_com14`) ist bewusst **minimal**: nur 123-BLE +
Lambda(CAN) + Logging + Web/API. ESP-NOW und BM6 sind dort **wegkompiliert**.

Das Display holt Daten per **HTTP-Poll `GET /api/status`** (alle ~200–500 ms):
- Hub im Auto = **AP-only**: SSID `Spartan3-TestHub`, Passwort `lambda123`, `http://192.168.4.1/`
- Hub am Schreibtisch = zusätzlich im Heimnetz (STA), z. B. `http://192.168.0.91/`
- Display-IP im Heimnetz: `192.168.0.109`, Hostname `esp-touch2.8`

## Verfügbare Daten (`GET /api/status`, JSON)

| Feld | Bedeutung | 123TUNE+ Instrument |
| --- | --- | --- |
| `rpm` | Drehzahl | U/min |
| `advance` | Zündvorverstellung ° | Kurbelwelle Vorverstellung |
| `map` | Saugrohrdruck kPa | kPa |
| `volt` | Bordspannung (123 0x41) | VOLT |
| `tune_amp` | Zündspulenstrom A (123 0x35) | AMP |
| `tune_temp` | Verteiler-Temp °C (123 0x33) | TEMP °C |
| `lambda` | Lambda (Spartan über CAN) | (statt GPS) |
| `temperature` | Abgas-/Sondentemp °C | — |
| `speed_kmh` / `speed_hz` | Reed-Geschwindigkeit | GPS-Tempo-Ersatz |
| `tune_connected` / `tune_link_state` | 123-Verbindung | Status |
| `status` / `source` | Lambda-Status / Quelle | — |
| `can_ready` / `can_state` | CAN-Status | — |
| `device_hours`/`engine_hours`/`sensor_hours` | Betriebsstunden | — |

(Komplettes Beispiel-JSON per `curl http://192.168.4.1/api/status` ziehen.)

## Ziel-Layout (Original 123TUNE+ Dashboard nachbauen)

Fünf Rundinstrumente + zentral Drehzahl:
- **Kurbelwelle Vorverstellung °** (`advance`, ~0–50)
- **U/min** (`rpm`, 0–8000) — zentral, groß; darin klein die **Reed-Geschwindigkeit** (`speed_kmh`)
- **kPa** (`map`, 0–200)
- **VOLT / AMP** (`volt` 12–14 V, `tune_amp` 0–3 A) — kombiniert
- **TEMP °C** (`tune_temp`, -20…100)
- Zusätzlich prominenter **Lambda**-Wert (statt GPS)

## Hardware — ERMITTELT (COM13, 2026-06-23)

Board: **Waveshare ESP32-S3-Touch-LCD-2.8C** (Serial-Boot meldet sich als „Waveshare 2.8C VDO Clock").
- **SoC:** ESP32-S3 (QFN56) rev v0.2 — WiFi+BLE, **8 MB PSRAM**, **16 MB Flash**, 40 MHz Crystal.
  MAC `a0:f2:62:e3:a9:84`, Hostname `esp-touch2.8`.
- **Display: RGB-Panel** (NICHT SPI/ST7789!) — RGB-Interface über das S3-LCD-Peripheral,
  Steuerleitungen über **PCA9554 I/O-Expander** (I2C), Init per SPI. Framebuffer im PSRAM.
  → Treiber: ESP-IDF `esp_lcd` RGB-Panel bzw. LVGL mit RGB-Bus (Klasse ST7701S). **Kein TFT_eSPI.**
- **Touch: GT911** (I2C-Adresse 0x5D, id 911).
- **IMU: QMI8658** (I2C 0x6B). **Onboard-RTC** (läuft). PCA9554 für Backlight/Reset/CS.
- Pin-Hinweise aus Boot-Log: CAN cockpit `TX=GPIO43 RX=GPIO44`. RGB/PCA-Pins aus Waveshare-Doku
  bzw. existierender Firmware übernehmen.

## Kontext „warum der Hub minimal ist" (Hintergrund)

Die 123-BLE-Verbindung war instabil, weil der NimBLE-Host durch WiFi/ESP-NOW/BM6
verhungerte (ACL-Buffer-Erschöpfung) und ein Write-MIT-Response-Ping mit der Notify-Flut
kollidierte. Fix: Minimal-Modus (BM6/ESP-NOW raus, Buffer verdoppelt) + Write-OHNE-Response.
Darum soll der Hub schlank bleiben — **Display zieht Daten, statt den Hub zu belasten.**

## Repo-Konventionen

- PlatformIO; neue Env im `waveshare-vdo-clock`-Repo (eigenes Board, eigene platformio.ini).
- **Keine Secrets committen** (WLAN-Passwörter etc.). AP-Passwort `lambda123` ist das
  Geräte-AP-Passwort und bereits im Repo — kein echtes Geheimnis.
- Hub-Repo (`spartan3v2-can-adapter`) bleibt unverändert — kein Grund, dort etwas anzufassen.
