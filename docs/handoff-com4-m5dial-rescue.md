# Handoff: COM4 / M5Dial Bootloop-Rescue

> **So benutzen:** Neue Claude-Code-Web-Session starten, als Repo **`niedi74/m5stack-123`**
> (Branch `feature/spartan-live-display`) wählen, und **dieses komplette Dokument** als
> ersten Prompt einfügen. Dann arbeitet der Agent autonom an COM4.

---

## TL;DR (Ziel dieser Session)

Der **M5Dial (COM4)** bootet nicht mehr stabil: nach Reset kurz hell, dann dunkel,
**Bootloop / Blinken**, kein stabiler Serial-Output. Ursache mit hoher Wahrscheinlichkeit:
Die **OPI-PSRAM-Konfiguration** (`board_build.arduino.memory_type = qio_opi`) wurde aus der
`platformio.ini` entfernt. Aufgabe: **stabilen Boot wiederherstellen**, mit einem
Minimal-Rescue-Build (nur Display + Power-Hold), dann BLE/123-direkt schrittweise zurück.

**Kein** ESP-NOW, **kein** WiFi, **kein** Hub-Bezug in dieser Session. Nur: M5Dial bootet sauber.

---

## Hardware-Fakten M5Dial

| Eigenschaft | Wert |
|---|---|
| MCU-Modul | ESP32-S3-WROOM-1 **N16R8** |
| Flash | 16 MB, **QIO** |
| PSRAM | 8 MB, **OPI (Octal)** ← entscheidend |
| Display | GC9A01 rund, 240×240, SPI, Backlight |
| USB | **natives USB** am USB-C (kein separater UART-Chip) |
| Port | COM4 |
| MAC | `48:ca:43:b7:6b:f4` |

**Wichtig:** N16R8 **muss** mit `memory_type = qio_opi` gebaut werden. Ohne das wird die
OPI-PSRAM falsch initialisiert → `M5.begin()` legt den Display-Framebuffer in die PSRAM →
Crash direkt nach Backlight-An → Reset → Bootloop ("kurz hell, dann dunkel").

Referenzen:
- N16R8 braucht qio_opi: <https://lonelybinary.com/en-us/blogs/esp32-family-getting-started-guide/02_09_platformio>
- PlatformIO Flash/PSRAM-Configs: <https://github.com/sivar2311/ESP32-PlatformIO-Flash-and-PSRAM-configurations>

---

## Symptom (aus Fahrt-Status 2026-06-19)

- M5 blinkt weiter, nach Reset kurz hell, dann dunkel.
- Kein stabiler Serial-Output nach Flash/Reset.
- Bereits versucht (im m5-Repo): PSRAM-OPI-Flag **entfernt** (`memory_type = qio_opi` raus),
  Battery-Hold hart auf ON, Rescue-Flag `M5_RESCUE_DIRECT_ONLY=1`.
  → Hat **nicht** geholfen; Verdacht: das Entfernen von `qio_opi` ist genau die Ursache.

---

## Root-Cause-Analyse (zwei Verdachtspunkte)

### 1. PSRAM-Modus falsch (Hauptverdacht → Bootloop)
`memory_type = qio_opi` fehlt → OPI-PSRAM im falschen Modus → Crash in `M5.begin()`.
**Fix:** Flag zurück in `platformio.ini`.

### 2. Serial über falschen Pfad (→ "kein Serial-Output")
M5Dial hängt per **nativem USB** am USB-C. Wenn der Build `ARDUINO_USB_MODE=0` /
`ARDUINO_USB_CDC_ON_BOOT=0` hat, geht `Serial` auf UART0-Pins statt auf USB-C → am COM4
ist nichts zu sehen, selbst wenn das Board bootet.
**Fix:** `ARDUINO_USB_MODE=1` + `ARDUINO_USB_CDC_ON_BOOT=1`.

---

## Fix 1 — `platformio.ini` (M5Dial-Env)

Env so setzen (Board-agnostisch, erzwingt QIO-Flash + OPI-PSRAM explizit):

```ini
[env:m5dial]
platform = espressif32
framework = arduino
board = esp32-s3-devkitc-1
board_build.mcu = esp32s3
board_build.flash_mode = qio
board_build.psram_type = opi
board_build.arduino.memory_type = qio_opi   ; <-- DER FIX (war entfernt)
board_upload.flash_size = 16MB
board_build.flash_size = 16MB
board_build.partitions = default_16MB.csv
upload_port = COM4
monitor_port = COM4
monitor_speed = 115200
build_flags =
  -DBOARD_HAS_PSRAM
  -DARDUINO_USB_MODE=1          ; M5Dial = natives USB -> Serial über USB-C
  -DARDUINO_USB_CDC_ON_BOOT=1
  -DM5_RESCUE_DIRECT_ONLY=1     ; Rescue: kein ESP-NOW/WiFi (vorhandenes Flag beibehalten)
lib_deps =
  m5stack/M5Unified@^0.2.2
```

> Falls das bestehende Env anders heißt (z.B. `m5stack-123` / `m5dial_motorraum`): **nicht**
> ein neues anlegen, sondern das **bestehende** Env anpassen. Nur die obigen Keys ergänzen/setzen,
> restliche projektspezifische Flags behalten.

---

## Fix 2 — Minimal-Rescue-Sketch (Boot-Beweis)

Zuerst **nur** das hier flashen, um zu beweisen, dass der Boot steht (kein BLE/WiFi):

```cpp
#include <M5Unified.h>

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);                 // hält beim Dial den Power-Latch selbst
  M5.Display.setBrightness(255);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_GREEN);
  M5.Display.setTextSize(3);
  M5.Display.setCursor(30, 100);
  M5.Display.print("M5 BOOT OK");
  Serial.begin(115200);
  Serial.println("M5 BOOT OK");
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 1000) {
    t = millis();
    Serial.println("alive");
  }
  M5.update();
}
```

**Erwartung:** "M5 BOOT OK" bleibt stehen, im Serial-Monitor (COM4, 115200) kommt jede
Sekunde `alive`. Bleibt das stabil → PSRAM-Config war die Ursache.

---

## Schrittplan für die Session

1. `git fetch` + auf Branch `feature/spartan-live-display` (oder den aktiven M5-Branch) gehen.
2. **Bestehende** `platformio.ini` lesen: aktuelles M5-Env, Board, Flags, lib_deps erfassen.
3. Fix 1 anwenden: `memory_type = qio_opi` + USB-CDC-Flags ergänzen.
4. Rescue-Pfad sicherstellen: wenn `M5_RESCUE_DIRECT_ONLY=1`, im Code nur Display + Power-Hold
   aktiv, **BLE/ESP-NOW/WiFi aus**. Falls kein sauberer Rescue-Zweig existiert, den
   Minimal-Sketch (Fix 2) temporär als `src/main.cpp` verwenden (alten als `.bak` sichern).
5. **Build verifizieren:** `pio run -e <m5-env>` muss grün sein.
6. Commit + Push auf den M5-Branch (siehe Git-Regeln unten). **Nicht** flashen können wir aus
   der Cloud — Flash macht der User lokal: `pio run -e <m5-env> -t upload`.
7. User flasht + schickt Boot-Log/Foto. Wenn stabil:
8. **Stufenweise zurück:** Rescue-Flag aus → 123-direkt-BLE wieder rein → testen →
   erst danach optional ESP-NOW/WiFi. Nach jeder Stufe Boot prüfen.

---

## Verifikation / Akzeptanzkriterien

- [ ] `pio run -e <m5-env>` Build **grün** (Cloud).
- [ ] Nach Flash (lokal): Display zeigt dauerhaft "M5 BOOT OK", **kein** Blinken/Reset.
- [ ] Serial-Monitor COM4 @115200 zeigt regelmäßig `alive` (USB-CDC funktioniert).
- [ ] Danach mit 123-direkt-BLE: bootet weiterhin stabil, verbindet zur 123TUNE+.
- [ ] Heap stabil, kein `Guru Meditation` / `rst:` Bootloop im Log.

---

## Git-Regeln (wichtig)

- Entwicklung auf dem aktiven M5-Branch (vermutlich `feature/spartan-live-display`) —
  **nicht** auf `main` ohne Rücksprache.
- Eigene Commits sauber authoren (Agent-Commits als Claude), **fremde/bestehende Commits
  des Users NICHT amenden/reauthoren**.
- Minimaler Diff. Bestehende projektspezifische Flags/Pins **nicht** löschen.
- Keine Secrets committen.

---

## Kontext / Verweise (Hub-Seite, nur zur Orientierung — NICHT Aufgabe hier)

- Hub-Repo: `niedi74/spartan3v2-can-adapter` (Branch-Arbeit dort separat).
- Strategischer roter Faden (aus Planung 2026-06-19):
  **Phase 1 Hub Core stabil → Phase 2 CAN-Output → Phase 3 Displays.**
  M5 bleibt langfristig **123-direkt Remote-Display** (kein Hub-Zwischenschritt).
- COM4-Rescue (dieses Dokument) ist der erste, isolierte Schritt: **M5 bootet wieder.**
