# Hub: WLAN-Stabilität, Variante A & W25Q-Config-Backup

Referenz für die Hub-Firmware-Arbeit rund um WLAN-Stabilität (Stand 2026-06-30,
Branch `test-hub-s3`). Ergänzt die Commit-Messages — hier steht das **Warum** und
**Wie**, damit man es später nicht aus dem Git-Log rekonstruieren muss.

Geräte im Setup:
- **Hub** (COM24): liest Spartan über CAN, verbindet 123 über BLE, Web-GUI, speist
  Remote-Displays (Touch 2.8", M5) über CAN/0x510. Env `motorraum_s3_test_com14`.
- **123-Emulator** (COM22): eigenständiger ESP, advertised als `123\TUNE+`, füttert
  den Hub mit Test-RPM/Adv/MAP. Env `emu_com17` (`-D ENABLE_EMU123=1`).

---

## 1. WLAN: AP-Kanal folgt dem Router + scan-freier Connect

**Problem (Single-Radio):** Der ESP32-S3 hat nur **eine** 2,4-GHz-Funkeinheit. SoftAP
(für die Displays) und STA (Heim-/S24-WLAN) müssen sich **denselben Kanal** teilen.
Früher war der AP **fest auf Kanal 6**; lag der Router auf einem anderen Kanal,
assoziierte der Hub zwar (Auth OK), bekam aber **keine IP**.

**Lösung:**
- `hubApChannel` (NVS-Key `ap_chan`, Default 6) ersetzt den festen Kanal in allen
  `WiFi.softAP()`-Aufrufen → AP kann dem Router-Kanal folgen, übersteht Reboots.
- **Scan-frei** (`connectHomeWifiAligned()`): statt eines Scans (der durch BLE+AP-Koexistenz
  unzuverlässig war, „Scan fehlgeschlagen") setzt der AP für den Connect **kurz aus**
  (`WIFI_STA`, ~5–20 s), die STA holt sich auf reiner STA die DHCP-IP, danach kommt der
  AP über `WiFi.channel()` auf genau dem Router-Kanal zurück (loop-Kontext, NVS-Merk).
- **Klartext-Grund** im Dev-Tab „WLAN-Kanal (STA / AP)":
  `201` AP nicht gefunden · `202/204/15` Passwort? · `8` Kanal/DHCP · `210` kein Passwort.

Commits: `d0f21b7` (Kanal-Follow), `9d5cdf8` (scan-frei + Klartext).

> **Wichtige Lehre (Z00-Station):** Der reale „assoziiert, keine IP"-Fall bei Z00 lag am
> Ende **nicht** am Kanal (Z00 = Kanal 6 = AP-Kanal 6) und **nicht** am Passwort, sondern
> an **veralteten DHCP-Leases in der FRITZ!Box** (Mesh: 7590 + 310). Fix: alte
> „spartan"-Geräte in der FRITZ!Box löschen + Mesh-Knoten neu starten → sofort IP
> (`192.168.0.87`, −36 dBm). Bei „assoziiert, keine IP" mit **passendem Kanal** zuerst
> Router/Lease prüfen. Details: `../CAN-bus/Hub-WLAN-Z00-Problem-und-Loesung.md`.

---

## 2. Variante A — Motor-Gate (stabiler AP/Touch während der Fahrt)

**Anforderung:** Läuft der Motor, muss der Hub-AP den Clients (Touch/Displays)
**jederzeit** stabil bleiben. Jeder STA-Versuch (Scan/Connect/Kanalwechsel) kann den
AP kurz blippen — das darf **fahrend** nicht passieren.

**Umsetzung** (`vehicleActive()`):
- „Fahrzeug aktiv" = `tuneRpm > ENGINE_RUNNING_RPM_THRESHOLD` (650) **oder** Reed-Speed
  > 0,5 km/h.
- **Emu-Ausnahme:** im Emu-Modus (`hubFeatEmu123`) liefert `vehicleActive()` immer
  `false` — die Emu-RPM ist selbst erzeugt, kein echter Motor (sonst blockiert der Emu
  sein eigenes Heim-WLAN).
- Fahrend: `WiFi.setAutoReconnect(false)` (IDF sucht **nicht** nach Verbindungsabriss) +
  alle expliziten Connect-Trigger (Boot, Reconnect, `connectHomeWifiAligned`) sind
  hinter `!vehicleActive()` gesperrt. Eine **bestehende** Verbindung bleibt; fällt sie
  fahrend weg, kommt sie erst beim nächsten Halt zurück.
- Im Stand: Auto-Reconnect wieder an + einmaliger Connect-Versuch.

| Zustand | Hub |
|---|---|
| Motor läuft / fährt | nur AP, kein STA/Scan/Kanalwechsel → Touch bockfest |
| Motor aus / Standgas (< 650 rpm, steht) | darf Heim/S24 verbinden, dann halten |
| AP | immer an |

Commits: `8155e9b` (Gate), `59509e9` (Emu-Ausnahme).

---

## 3. W25Q-Config-Backup (überlebt jeden Firmware-Flash)

Externer **W25Q128** (16 MB SPI-NOR, FSPI: CS=13 CLK=14 DI/MOSI=15 DO/MISO=18, 3V3)
sichert die Config zusätzlich zum internen NVS — als Roh-Blob im **Sektor 0**, ohne
Dateisystem.

- **Gesichert:** WLAN-Profile (p1/p2 SSID+Pass), aktives Profil, Legacy-SSID/Pass,
  AP-SSID/Pass/IP, `ap_chan`, mDNS-Name. Blob mit **Magic + Version + CRC32**.
- **Restore** (`restoreConfigFromW25Q()`, läuft in `loadHubFeatures()` vor dem NVS-Lesen):
  nur bei **frischem NVS** (Sentinel-Key `cfg_w25q` fehlt, z.B. nach `erase_flash`) und
  gültigem Blob → schreibt die Werte zurück ins NVS.
- **Backup** (`saveConfigToW25Q()`): bei Config-Änderung wird `hubCfgDirty` gesetzt und in
  `updateWebGui()` einmalig geschrieben (Erase ~50 ms, selten/user-getriggert).

**Fallback ohne Chip** (`w25qDetected == false`) — alle Chip-Zugriffe sind gesperrt:

| Szenario | mit W25Q | ohne W25Q (nur NVS) |
|---|---|---|
| Reboot / Stromaus | bleibt | bleibt |
| Normaler Firmware-Flash / OTA | bleibt | bleibt |
| `erase_flash` / NVS gelöscht / Partition geändert | **aus Backup wiederhergestellt** | **weg (neu eingeben)** |
| Hub-Betrieb (CAN/AP/BLE/Web) | normal | völlig normal |

Kein Crash/Hänger ohne Chip; ein Falsch-Restore ist durch Magic/Version/CRC ausgeschlossen
(liest der Hub Müll/0xFF, nimmt er die NVS-Defaults).

Commits: `daea4a4` (Erkennung), `4738db1` (Dev-Tab-Status), `8c41b71` + `5b96974` (Backup).

---

## 4. 123-Emulator: 300-rpm-Button

`/emu` hat jetzt Buttons **Sweep / 300 / 800 / 1500 / 2500 / 4000**. **300** liegt unter
dem Motor-Gate-Schwellwert (650) → der Hub sieht „Motor aus / Stand" und **erlaubt** den
Heim-/S24-Connect. Damit lässt sich Variante A bequem vom Tisch aus testen (300 = Stand,
≥800 = „fährt" → AP-only). Commit `3a29c53`.

---

## 5. Diagnose / Sichtbarkeit

**`/api/status`-Felder (neu):** `wifi_sta_channel`, `wifi_sta_rssi`, `wifi_ap_channel`,
`wifi_sta_reason`, `wifi_home_not_found`, `vehicle_active`, `flash_ext_detected`,
`cfg_backup`.

**Dev-Tab „System"-Karte (neu):**
- `Ext. Flash (W25Q)` — erkannt? (JEDEC/Größe/Hersteller)
- `Config-Backup (W25Q)` — gesichert ✓ / Chip da, noch kein Backup / kein Chip
- `WLAN-Kanal (STA / AP)` — verbunden (Kanal + dBm) bzw. Fehlergrund im Klartext
- `Fahrt-Gate (Variante A)` — aktiv (pausiert) / Stand (frei/verbunden)

Jede dieser Zeilen hat eine kurze Inline-Beschreibung in der GUI.

---

## Geräte-/Pin-Kurzreferenz (Test-Hub)

| Funktion | Pin |
|---|---|
| Status-LED | 2 |
| I2C SDA/SCL | 8 / 9 |
| CAN RX/TX (TWAI 500 kbit/s) | 10 / 11 |
| Reed-Speed | 12 |
| W25Q CS/CLK/MOSI/MISO | 13 / 14 / 15 / 18 |
| Spartan UART RX/TX | 16 / 17 |

CAN: RX Spartan `0x400`, TX Cockpit `0x510` (10 Hz, 8 Byte BE) — braucht einen ACK-Knoten
(Spartan), der Touch ist listen-only.
