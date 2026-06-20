# Handoff für Claude Code – M5Dial (COM4) wieder stabil + Testhub-Stand

Stand: 2026-06-20 · Autor: Übergabe aus der Codex-Session (Nutzungslimit erreicht)

> Zweck: Eine frische Claude-Code-Session soll den M5Dial an COM4 wieder
> stabil zum Laufen bringen. Diese Datei fasst Ziel, gesicherten Wissensstand
> und Root-Cause-Analyse zusammen, damit **nicht von vorne geraten** werden muss.

---

## 1. Auftrag (klar und eng halten)

Genau **zwei** Betriebsarten am M5Dial, **ohne automatische Fallback-Kette**:

1. **Modus 1 – 123 direkt:** RPM/123-Werte direkt per BLE vom 123TUNE+ am M5;
   Lambda (und BM6) kommen vom Hub.
2. **Modus 2 – Alles vom Testhub:** sämtliche Werte ausschließlich vom Testhub
   (Protokoll egal – ESP-NOW **oder** BLE, Hauptsache stabil).

Anforderung des Nutzers wörtlich: *„einfach simple, stabil und straight forward,
kein Schnickschnack, keine Fallback-Lösung, einigermaßen flüssige Datenübertragung."*

Ausdrücklich erlaubt: **kompletter Neuanfang** des M5-Firmware-Pfads anhand der
offiziellen M5Dial-Beispiele (siehe Referenzen §6), statt den gewachsenen Code
weiter zu flicken.

---

## 2. WICHTIG: Wo liegt welcher Code?

- **Dieses Repo** (`spartan3v2-can-adapter`) = **Hub / Spartan-CAN-Adapter / Testhub (COM14)**.
- Die **M5Dial-Firmware** (PlatformIO-Env `m5stack-stamps3`, `#include <M5Unified.h>`)
  liegt in einem **separaten Projekt** (ESP123 / SynologyDrive-Verzeichnis), **nicht** hier.
  → Die folgende Boot-Loop-Analyse betrifft jenes Projekt; hier ist nur das *Wissen* gesichert.
- Board: **M5Dial v1.1**, MCU **StampS3 (ESP32-S3)**, USB = **COM4**.

---

## 3. M5Dial COM4 Boot-Loop – Root-Cause-Analyse (das Wichtigste)

**Symptom:** Dauernder Reboot, ~2,7 s Zyklus. COM4 verschwindet/erscheint
periodisch auf dem USB. Resetgrund anfangs `RESET_REASON=1` (Power-On),
nach Power-Hold-Fix `RESET_REASON=9` (**Brownout**).

### Gesicherte Erkenntnisse (in dieser Reihenfolge gefunden)

1. **Power-Hold GPIO46 – Reihenfolge ist entscheidend.**
   M5Dial hält seine eigene Versorgung über GPIO46. Korrekt (wie M5Unified):
   **erst HIGH setzen, dann auf OUTPUT schalten.** Umgekehrt entsteht ein kurzer
   LOW-Impuls → Gerät läuft noch ~2,7 s aus dem Kondensator und startet neu.
   - `digitalWrite()` auf einem Input setzt nur den Pull-up, **nicht** sicher das
     Ausgangslatch. Manuelle Nachbildung (gpio_set_level / out1_w1ts) hat auf
     **diesem konkreten M5Dial v1.1 nicht gereicht.**
   - **Konsequenz: Power-Hold ausschließlich M5Unified erledigen lassen**
     (`M5.begin()` mit Default-Powerinit), nicht selbst nachbauen.

2. **Einziger nachweislich stabiler Baseline-Test:**
   *Nur* `M5.begin()` (offizielle M5Unified Power/Display-Init) **+ statisches Display**,
   **ohne** zweite Display-Init, **ohne** BLE, **ohne** ESP-NOW
   → **20 s ohne einen einzigen Aussetzer.** Damit sind Hardware, Kabel und
   Versorgung **in Ordnung**. Der Fehler liegt in der App-Init, nicht in der HW.

3. **Eigene zweite Display-Init nach `M5.begin()` löst Reset aus.**
   → App komplett auf das bereits initialisierte **`M5.Display`** umstellen,
   keine separate TFT-/Display-Initialisierung mehr.

4. **Flash/Memory-Config:** Nur **`qio_opi`** lief stabil.
   **`qio_qspi` läuft auf diesem M5Dial NICHT** (Boot-Loop). Also `qio_opi` pinnen.

5. **Restfehler nach Power-Hold-Fix = Brownout (reason 9) beim Funkstart.**
   Tritt exakt bei `NimBLEDevice::init()` bzw. ESP-NOW-Init auf → kurze
   **Einschalt-Stromspitze**. Versuchte Milderung: Funk bei **ausgeschalteter
   Hintergrundbeleuchtung** starten, kurze 100-ms-Warmlaufphase, **danach** Helligkeit
   hoch (Default 100, nicht 200). Brownout-Wächter temporär überbrücken wurde
   getestet, aber bewusst **verworfen** (keine Dauerlösung – behandelt das Symptom).

6. **NimBLE-Version war nicht gepinnt.** `^2.0.0` zog automatisch 2.5.0; der
   „2 Wochen stabile" Stand war nicht darauf festgelegt.
   → **NimBLE-Version exakt pinnen** (nicht `^`).

### Daraus abgeleiteter Plan für den Neuanfang (Empfehlung)

1. Minimal-Sketch: nur `M5.begin()` + `M5.Display` statisch → bestätigen: stabil.
2. **Einen** Funkweg dazu (für Modus 2: ESP-NOW; für Modus 1: genau **eine** BLE-Verbindung
   zu 123). Funk mit Display dunkel starten, dann Helligkeit hoch.
3. Erst wenn ein Funkweg stabil ist, den zweiten Datenweg ergänzen.
4. **Keine** Fallback-Kette, kein WebGUI/HTTP-Polling/SPIFFS im M5, solange es um
   Stabilität geht. `qio_opi`, NimBLE gepinnt, Power-Hold nur via M5Unified.

---

## 4. Gesicherter Recovery-Punkt (M5)

- Branch `recovery/m5dial-stable-core2`, Commit **`1e44167`**.
- Arduino Core **2.0.17**, 8 MB Flash, **kein PSRAM**, 123-BLE direkt,
  **kein** ESP-NOW-/Fallback-Geflecht.
- Hinweis aus der Session: der Sprung auf **Arduino Core 3** war eine
  Hauptverdächtige für das frühere „Blinken"/Instabilität → Core 2.0.17 bleiben.

---

## 5. Testhub COM14 – aktueller Stand (in DIESEM Repo)

Aus `/api/status` (Hub-AP `192.168.4.1`, Spartan3-Setup):

- **BM6:** `bm6_build:false` → diese Testhub-Firmware enthält **keinen BM6-Client**.
  Deshalb bleibt „BM6 nicht aktiv", **unabhängig** von der Adresse `...d0:bc`.
  → BM6 muss im COM14-Build wirklich **einkompiliert** werden (Build-Flag suchen/setzen).
- **CAN:** `can_ready:true`, aber `can_rx_total:0`, `can_tx_errors:128`,
  > 329 000 TX-Fehler. Controller startet, aber **keine bestätigte CAN-Kommunikation**.
  Prüfen: Verkabelung, **gemeinsame Masse**, **Bitrate**, **Busabschluss (120 Ω)**.
  `stale rx_timeout` bedeutet nur „keine frischen Nutzdaten", **kein** Defektbeweis.
- **SPIFFS/CSV defekt:** Log spammt alle 500 ms `fopen(/spiffs/drive.csv) failed` /
  `Logs: create failed`. → SPIFFS-Logpartition reparieren/neu formatieren.
- 123-Emulator bestätigt: `conn=1`, RX > 36 000, RPM 700–2900 plausibel.

Details siehe `docs/HANDOFF_COM14_ANDROID_2026-06-20.md` (COM14-Umbau + Android Live-App).

---

## 6. Referenzen (vom Nutzer für Neuanfang genannt)

- M5Dial BSP (Espressif): https://github.com/espressif/esp-bsp/tree/master/bsp/m5dial
- M5Dial V1.1 Doku: https://docs.m5stack.com/en/core/M5Dial%20V1.1
- M5Dial Hardware K130: https://github.com/m5stack/M5_Hardware/tree/master/Products/K130_Dial/Structures
- Beispiel-FW (stabil): https://github.com/fbiego/esp32-c3-mini
- Beispiel-FW (stabil): https://github.com/AH2005NA/m5stick-shark

---

## 7. Praktische Diagnose-Regeln (aus der Session)

- COM4 **passiv** lesen: seriellen Port mit `dtr=False, rts=False` öffnen, sonst
  löst das Öffnen selbst einen USB-Reset am M5 aus.
- Resetgrund zur Diagnose **als erste Boot-Zeile** ausgeben → ein Zyklus reicht
  zur Unterscheidung Power-On (1) / Brownout (9) / Watchdog / Software.
- COM4-Reboot-Erkennung ohne den Port zu stören: zyklisch prüfen, ob `COM4` in
  `[IO.Ports.SerialPort]::GetPortNames()` auftaucht/verschwindet.
- NVS gezielt löschen (nur WLAN/Settings, Firmware bleibt):
  `esptool erase-region 0x9000 0x5000` (NVS lag bei 0x9000, 20 KB – Partitionstabelle
  vorher mit `gen_esp32part.py` verifizieren).
