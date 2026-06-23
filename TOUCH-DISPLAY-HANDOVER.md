# Handover: 2.8" ESP32-S3 Touch Display fürs Spartan-Cockpit

> Übergabe-Doku für einen **separaten Chat**. Ziel: ein 2.8"-Touch-Display, das die
> Motordaten anzeigt wie das originale 123TUNE+ Dashboard. Das Display ist ein
> **eigenständiges Gerät** und holt die Daten vom Hub — der Hub selbst bleibt unverändert
> (Single-Purpose-123-Client, siehe unten).

## Zuerst prüfen
- **Branch `claude/esp32-s3-touch-display-abuiy3`** auf GitHub (`niedi74/spartan3v2-can-adapter`)
  könnte bereits Vorarbeit enthalten — vor Neubeginn ansehen/auschecken.
- Aktueller, sauberer Stand der Firmware liegt auf Branch **`test-hub-s3`**.

## Architektur-Entscheidung (Datenquelle)
Der Hub (ESP32-S3, Env `motorraum_s3_test_com14`) ist bewusst **minimal**: nur 123-BLE +
Lambda(CAN) + Logging + Web/API. ESP-NOW und BM6 sind dort **wegkompiliert**
(`build_unflags` → `=0`). Für das Display daher der einfachste, robuste Weg:

**HTTP-Poll der Hub-JSON-API** — das Display verbindet sich mit dem Hub-WLAN und pollt
`GET /api/status` (alle ~200–500 ms). Kein zusätzlicher Funk-Stress, kein Pairing.

- Hub im Auto = **AP-only**: SSID `Spartan3-TestHub`, Passwort `lambda123`, `http://192.168.4.1/`
- Hub am Schreibtisch = zusätzlich im Heimnetz (STA), z. B. `http://192.168.0.91/`
- Alternative wäre ESP-NOW (Hub hat `include/spartan_cockpit_frame.h` + Broadcast-Code,
  aber auf dem Minimal-Hub deaktiviert). HTTP-Poll ist für ein einzelnes Display einfacher.
  → **Empfehlung: HTTP-Poll.** ESP-NOW nur wenn mehrere Displays/sehr niedrige Latenz nötig.

## Verfügbare Daten (`GET /api/status`, JSON)
Relevante Felder (Stand test-hub-s3):
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

Die bestehende Web-GUI des Hubs zeigt diese Werte bereits als Kacheln (Live-Tab) — als
Referenz für Wertebereiche/Beschriftung nutzen.

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

### Es existiert bereits Firmware auf dem Board!
Die aktuelle Firmware (vermutlich Branch `claude/esp32-s3-touch-display-abuiy3`) kann schon:
- VDO-Uhr-Zifferblatt zeichnen (nutzt `assets/t2b-clock/` PNGs), RGB-Panel + GT911 + IMU + RTC laufen.
- **CAN-RX** auf `id=0x510` (mode=listen) — TX=43, RX=44.
- WiFi (STA), WebGUI auf Port 80, NTP.
- **ABER:** versucht, **selbst** den 123 per BLE zu verbinden (`mode=123 dir mac=ef:a8:b2:de:e0:9e`)
  und scheitert mit `connect FAIL err=13`.

### ⚠️ Architektur-Konflikt unbedingt zuerst lösen
Das Display darf **NICHT** zusätzlich 123-BLE-Client sein. Der **Hub hält die EINZIGE
123-Verbindung** (123/Emu erlaubt nur einen Central). Doppelter Client = Dauerkampf → `err=13`.
**Lösung:** im Display den eigenen 123-BLE-Client **entfernen/deaktivieren** und Daten stattdessen
vom Hub ziehen:
- **Variante A (empfohlen):** HTTP-Poll `GET http://<hub>/api/status` (Felder s. o.).
- **Variante B:** CAN — der Display hört eh schon auf `0x510`. Dann muss der **Hub** die 123-Werte
  als CAN-Frame senden (aktuell sendet der Minimal-Hub das nicht). Mehr Firmware-Arbeit am Hub.
→ Start mit Variante A (Hub unverändert lassen).

### Offen
- Stromversorgung/Einbau (Auto 12 V → 5 V).
- Rundes 480×480-Panel: Gauge-Layout aufs runde Format anpassen (passt gut zum VDO-Look).

## Repo-Konventionen
- PlatformIO; neue Env z. B. `[env:touch_display]` (eigenes Board).
- **Keine Secrets committen** (WLAN-Passwörter etc.). AP-Passwort `lambda123` ist das
  Geräte-AP-Passwort und bereits im Repo — kein echtes Geheimnis.
- Toolchain für die zugehörige Android-WebView-App liegt unter `android/hubdisplay/`
  (nicht relevant fürs Display, nur zur Info).

## Kontext „warum der Hub minimal ist" (Hintergrund)
Die 123-BLE-Verbindung war instabil, weil der NimBLE-Host durch WiFi/ESP-NOW/BM6
verhungerte (ACL-Buffer-Erschöpfung) und ein Write-MIT-Response-Ping mit dem Notify-Flut
kollidierte. Fix: Minimal-Modus (BM6/ESP-NOW raus, Buffer verdoppelt) + Write-OHNE-Response.
Darum soll der Hub schlank bleiben — **Display zieht Daten, statt den Hub zu belasten.**
