# Handoff: Spartan Hub — Fahrt-Stabilität, ESP-NOW, Client-Log, Live-Debug

> **Zweck:** Dieses Dokument als **gesamten Prompt** in eine lokale Cursor/Codex-Session einfügen.
> Repo: `https://github.com/niedi74/spartan3v2-can-adapter`

---

## Kontext / Ausgangslage

### Git-Stand (wichtig — zuerst prüfen!)

- **`main` auf GitHub ist veraltet** (letzter Commit: 26. Mai 2026).
- **Aktuelle Arbeit liegt auf Branch `work`** (~22 Commits vor `main`), inkl. Fahrzeug-Features vom 2.–12. Juni 2026.
- Cloud-Agent-PR **#7** (`cursor/espnow-client-hub-594a`) basiert auf **altem `main`** — **nicht** auf `work` mergen ohne Rebase!
- **Vor jeder Implementierung:** `git fetch origin` und auf `work` arbeiten (oder `work` → `main` mergen).

```bash
git fetch origin
git checkout work
git pull origin work
git log origin/main..work --oneline   # was fehlt in main
git status                          # lokale uncommittete Änderungen?
```

Falls lokal noch unpushed Commits: **zuerst pushen**, dann weiterarbeiten.

---

## Probleme aus der Fahrt (12. Juni 2026)

1. **123TUNE+ BLE-Verbindung baut unterwegs nicht stabil auf**
   - Hub findet/verbindet 123TUNE+ unzuverlässig im Auto.
   - `reason=520` nach Zündung AUS ist erwartbar — entscheidend ist Reconnect bei Motorlauf.
   - Vermutete Ursache: **BLE-Radio-Überlast** (Hub = Peripheral für Displays + Central für 123TUNE + optional BM6 + WiFi-AP).

2. **BLE/WiFi mit 2+ Display-Clients im Auto unzuverlässig**
   - Mehrere GATT-Subscriber + 123-Central konkurieren um dasselbe ESP32-Funkmodul.

3. **Fehlendes Client-Log**
   - Brauche **Log aller Clients** (BLE-Displays, 123TUNE, BM6, WiFi-AP, HTTP-Poller, später ESP-NOW).

4. **Live-Eingriff unterwegs**
   - Hub soll mit Laptop/Handy-Hotspot verbunden sein, damit ich **während der Fahrt** per Browser/Cursor diagnostizieren und später fixen kann.

---

## Was auf `work` bereits existiert (nicht neu erfinden!)

Branch `origin/work` enthält u.a.:

| Feature | Details |
|---------|---------|
| Erweiterte Web-GUI | Live + Setup + Diagnose + CSV-Logging Tabs |
| `ble_hub_clients[]` | BLE-Display-Clients mit MAC, MTU, Intervall, age_ms |
| `ble_scan[]` | Gesehene BLE-Geräte während Scan (RSSI, tune/bm6-Flag) |
| `wifi_ap_stations[]` | Clients am `Spartan3-Setup` AP |
| `wifi_http_pollers[]` | HTTP-Clients die `/state` pollen |
| CSV Fahrt-Log | SPIFFS, konfigurierbare Spalten |
| BM6 dual-slot | Batterie + Zusatzbatterie rotierend |
| NimBLE max connections | `sdkconfig.defaults`: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=5` |
| NTP / time_epoch | Hub als Zeitmaster |
| Reed-Speed, Heater-ADC | GPIO 27 etc. |
| HOME_WIFI | Default `Android-AP1` in `platformio.ini` |

**Bewusst vertagt (laut alter Handoff):**
- `/log` Ringbuffer für Events
- Binary BLE Payload + CRC (teilweise auf work schon erweitert)
- OTA-Update

---

## Zielbild — vier Arbeitspakete

### AP1: Git aufräumen (Pflicht, zuerst)

1. Lokalen Stand committen + `git push origin work`
2. PR `work` → `main` erstellen und mergen (oder direkt mergen wenn solo)
3. ESP-NOW-Branch **auf `work` rebasen**, nicht auf altem `main`:
   ```bash
   git fetch origin
   git checkout cursor/espnow-client-hub-594a
   git rebase origin/work
   # Konflikte in src/main.cpp lösen
   git push --force-with-lease origin cursor/espnow-client-hub-594a
   ```

### AP2: Display-Clients auf ESP-NOW (statt BLE GATT)

**Warum:** Broadcast an beliebig viele ESP32-Displays ohne GATT-Connect/Subscribe.

**Hub-Seite** (teilweise in PR #7 skizziert, auf `work` integrieren):

| Flag | Wert | Bedeutung |
|------|------|-----------|
| `ENABLE_BLE_HUB` | `1` | 123TUNE BLE Central bleibt |
| `ENABLE_BLE_DISPLAY` | `0` | Kein GATT-Server für M5/Waveshare |
| `ENABLE_ESP_NOW_HUB` | `1` | Cockpit-Broadcast |
| `ESP_NOW_WIFI_CHANNEL` | `6` | Gleich wie AP-Kanal |

**Frame:** `include/spartan_cockpit_frame.h` — 14 Bytes, CRC-8, Felder: lambda, rpm, advance, map, flags.

**M5/Waveshare (separates Repo):** Gateway-Modus auf ESP-NOW-Empfang umstellen; `spartan_cockpit_frame.h` kopieren; WiFi Kanal 6; BLE-Scan zu `Spartan3-Hub` im Gateway-Modus entfernen.

**Doku:** `docs/espnow-gateway-architecture.md` (in PR #7)

### AP3: 123TUNE BLE stabilisieren

Konkrete Firmware-Änderungen auf `work`:

1. **Radio-Priorität:** Während 123-Scan/Connect **kein** Display-BLE-Advertising (wenn BLE-Display noch aktiv)
2. **Nach ESP-NOW-Umstellung:** 123TUNE ist einziger BLE-Central — deutlich weniger Konflikt
3. **Reconnect-State-Machine** mit klaren Zuständen:
   - `IDLE → SCANNING → CONNECTING → SUBSCRIBED → STREAMING`
   - Bei Disconnect: sofort Scan (kein langer Backoff beim ersten Versuch)
   - Backoff nur bei wiederholten Fehlschlägen (z.B. 5s, 10s, 30s max)
4. **Stale-Erkennung:** Wenn `tune_connected` aber `tune_age_ms > 3000` und Motor läuft → Resubscribe oder Reconnect
5. **Jeden State-Wechsel ins Event-Log** (siehe AP4)
6. **BM6-Rotation** nicht während aktivem 123-Connect scannen lassen (falls noch Konflikt)

Bekannte Diagnose-Regeln (aus Handoff):
- `notify len=5` und steigendes `tune_rx` = gut
- Nur `0D`-Notifies ohne Motor = normal
- `!read` über USB-Serial für Read-Dump: `v@`, `10@`, `11@`, `12@`, `13@`

### AP4: Einheitliches Client- + Event-Log

**Ziel:** Ein Ringbuffer + HTTP-Endpoint für alles was am Hub passiert.

```
GET /log/events        → JSON Array (letzte N Events)
GET /log/events.csv    → Download
POST /log/clear        → Leeren (optional, mit Confirm)
```

**Event-Typen (mindestens):**

| type | payload |
|------|---------|
| `tune_state` | `scan\|connect\|subscribe\|disconnect\|stale` + reason + MAC |
| `ble_client` | `connect\|disconnect` + addr + handle |
| `wifi_ap` | station join/leave + MAC + IP |
| `http_poll` | IP + X-Device header |
| `espnow_tx` | seq + ok/fail |
| `espnow_rx` | optional ACK von Display |
| `bm6_state` | connect/disconnect/decode |

**Implementierung:**
- Feste Größe Ringbuffer im RAM (z.B. 200 Events à ~80 Bytes) oder SPIFFS-Ring
- Timestamps: `millis()` + wenn NTP valid → `time_epoch`
- Web-GUI Diagnose-Tab: Live-Tabelle + Auto-Refresh
- Bestehende Felder in `/state` (`ble_hub_clients`, `wifi_ap_stations`, …) **behalten**

---

## AP5: Live-Debug unterwegs (Laptop ↔ Hub)

### Setup im Auto (Ziel-Workflow)

```
[Laptop/Cursor] ──WiFi──► [Handy-Hotspot Android-AP1] ◄──STA── [Spartan Hub]
```

1. Handy-Hotspot an (`Android-AP1` / Passwort in `platformio.ini` oder Preferences)
2. Hub bootet → verbindet STA zum Hotspot
3. Laptop ebenfalls ins Hotspot
4. Hub-IP aus Serial oder Router: `http://<wifi_ip>/`
5. Diagnose-Tab + `/state` + `/log/events` live beobachten

### Was sofort geht (ohne neue Firmware)

- Web-GUI Diagnose-Tab (`/state` JSON)
- CSV-Log nach Fahrt von SPIFFS laden
- USB-Serial am Motorraum-ESP (wenn Kabel erreichbar): `pio device monitor --port COM16`

### Was noch gebaut werden soll (Priorität)

| Feature | Nutzen |
|---------|--------|
| `/log/events` Ringbuffer | Nachvollziehen was während Fahrt schiefging |
| WebSocket `/ws/events` (optional) | Live-Tail im Browser ohne Polling |
| `GET /serial` TCP-Bridge (optional) | `pio device monitor` über WiFi — nur wenn sicher (Passwort!) |
| **ArduinoOTA** oder HTTPS-OTA | Fix flashen ohne USB nach Rückkehr oder an Haltepunkt |

**Sicherheit:** Kein offenes OTA/Serial ohne Auth — mindestens AP-Passwort + optional Setup-Token.

### Cursor/Agent live eingreifen — realistische Erwartung

- Cloud-Agent kann **nicht direkt** ins Auto-WLAN.
- Workflow: **Laptop lokal** mit Cursor, Hub-IP im Browser, `/state` + Events kopieren oder Screenshots → Agent analysiert → Fix → `pio run -t upload` per USB oder OTA.
- Optional: kleines Python-Script auf Laptop das `/state` jede Sekunde loggt → `drive_YYYYMMDD_HHMM.jsonl`

---

## Dateien / Referenzen

| Pfad | Inhalt |
|------|--------|
| `src/main.cpp` | Hauptfirmware (~3500 Zeilen auf `work`) |
| `platformio.ini` | `motorraum` = COM16, `huge_app`, HOME_WIFI, BM6 |
| `sdkconfig.defaults` | NimBLE max connections = 5 |
| `docs/handoff_2026-05-26.md` | Alter Stand, 123-BLE Diagnose |
| `docs/ble-gateway-architecture.md` | BLE-Rollen (Display → ESP-NOW migrieren) |
| `docs/espnow-gateway-architecture.md` | ESP-NOW Plan (PR #7) |
| `include/spartan_cockpit_frame.h` | Binary Frame (PR #7) |

**Build & Flash:**
```bash
pio run -e motorraum
pio run -e motorraum -t upload --upload-port COM16
pio device monitor --port COM16 --baud 115200
```

**Hardware (work branch Pins — von main abweichend!):**
- CAN: RX=25, TX=26
- UART: RX=16, TX=17
- Speed Reed: GPIO 27
- Heater ADC: GPIO 35

---

## Akzeptanzkriterien (Fahrzeug-Test)

### 123TUNE BLE
- [ ] Nach Zündung EIN: innerhalb 30s `tune_connected=true`
- [ ] Bei Motorlauf: `tune_rx` steigt, `tune_age_ms < 2000`
- [ ] Nach Zündung AUS/AUS: Disconnect erwartbar; nach EIN wieder Connect
- [ ] Event-Log zeigt klare State-Transitions

### ESP-NOW Displays
- [ ] 2 Displays gleichzeitig empfangen Frames (`seq` steigt)
- [ ] Kein BLE-Connect der Displays nötig
- [ ] `/state`: `esp_now_tx` steigt, `esp_now_tx_fail` bleibt niedrig

### Client-Log
- [ ] `/log/events` listet 123-, BLE-, WiFi-, HTTP-Events
- [ ] Diagnose-Tab zeigt Live-Events
- [ ] Nach 30min Fahrt: Ringbuffer nicht crashed, Heap stabil

### Live-Debug
- [ ] Hub im Handy-Hotspot erreichbar (`wifi_ip` in `/state`)
- [ ] Laptop kann `/state` und Web-GUI öffnen während Motor läuft

---

## Implementierungs-Reihenfolge (für die Session)

```
1. git: work pushen / als Basis checkout
2. ESP-NOW auf work integrieren (PR #7 rebasen)
3. 123TUNE Reconnect + Radio-Priorität härten
4. /log/events Ringbuffer + Diagnose-UI
5. Doku: docs/road-live-debug.md (Hotspot-Workflow)
6. Optional: OTA, WebSocket
7. Fahrzeug-Test mit Checkliste oben
```

**Scope-Regeln:**
- Minimaler Diff, bestehende `work`-Features nicht löschen
- Keine Secrets in Git committen (HOME_WIFI Passwort aus platformio.ini in Preferences/Web-GUI verschieben)
- Tests: `pio run -e motorraum` muss grün sein

---

## Prompt für Cursor (Kurzversion — nur diesen Block kopieren)

```
Repo: spartan3v2-can-adapter, Branch: work (NICHT main — main ist veraltet).

Aufgabe in dieser Reihenfolge:
1. git fetch && checkout work && sicherstellen alles gepusht ist
2. ESP-NOW für Display-Clients integrieren (PR #7 auf work rebasen):
   ENABLE_BLE_DISPLAY=0, ENABLE_ESP_NOW_HUB=1, spartan_cockpit_frame.h
3. 123TUNE BLE stabilisieren: Radio-Priorität, Reconnect-State-Machine, kein Scan/Advertising-Konflikt
4. Einheitliches /log/events Ringbuffer für alle Clients (123, BLE, WiFi, HTTP, ESP-NOW) + Diagnose-Tab
5. docs/road-live-debug.md: Laptop über Handy-Hotspot (Android-AP1) an Hub

Kontext Fahrt 12.06: 123 BLE instabil, 2+ BLE-Displays unzuverlässig.
work hat schon: ble_hub_clients, wifi_ap_stations, wifi_http_pollers, CSV-Log, BM6, NTP.

Nicht auf altem main bauen. platformio.ini motorraum = COM16, CAN 25/26, UART 16/17.
Akzeptanz: tune_connected <30s nach Zündung EIN, esp_now_tx steigt für 2 Displays, /log/events zeigt State-Wechsel.
```
