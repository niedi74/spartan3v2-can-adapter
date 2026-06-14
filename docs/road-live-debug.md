# Road Live Debug — Laptop via Handy-Hotspot

Ziel: Während der Fahrt oder am Straßenrand den Spartan Motorraum-Hub per Laptop debuggen, ohne USB-Kabel am ESP32.

## Netzwerk-Setup (Android-AP1)

1. **Handy-Hotspot starten** — SSID `Android-AP1`, Passwort wie in `platformio.ini` (`REDACTED`).
2. **Spartan ESP32** bootet und verbindet sich per STA zum Hotspot (eingebautes Profil oder Web-GUI Setup).
3. **Laptop** verbindet sich mit demselben Hotspot.
4. **Hub-IP finden:**
   - Web-GUI Tab *Setup → WLAN* zeigt `ESP32 IP`
   - oder `/state` JSON: Feld `wifi_ip`
   - oder USB-Serial beim Boot: `Home WiFi: connected ...`

Typische URLs:

| Dienst | URL |
| --- | --- |
| Web-GUI | `http://<wifi_ip>/` |
| Status JSON | `http://<wifi_ip>/state` |
| Event-Log | `http://<wifi_ip>/log/events` |
| CSV Fahrt-Log | `http://<wifi_ip>/download` |

## Was live beobachten

### Diagnose-Tab

- **123Tune BLE:** Verbindung, RX-Alter, Scan-Kandidaten, State-Machine (`tune_link_state` in JSON)
- **ESP-NOW:** `esp_now_tx` steigt (~4/s), `esp_now_tx_fail` bleibt niedrig
- **BLE Hub Clients:** leer wenn `ENABLE_BLE_DISPLAY=0` (Cockpit nutzt ESP-NOW)
- **WiFi AP / HTTP Pollers:** wer am Hub hängt
- **Event-Log:** State-Transitions (`tune_state`, `ble_client`, `wifi_http`, `espnow_tx`)

### Serial (wenn Kabel erreichbar)

```bash
pio device monitor --port COM16 --baud 115200
```

## AP-only Fallback (ohne Internet)

Wenn kein Hotspot verfügbar:

1. Laptop oder Display mit `Spartan3-Setup` verbinden (Passwort `lambda123`)
2. Hub unter `http://192.168.4.1/` — NTP bleibt aus, Event-Log mit Boot-`millis()` funktioniert trotzdem

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Laptop findet Hub nicht | Gleiches WLAN? `wifi_connected` in `/state`? |
| 123 BLE instabil | Event-Log: `tune_state` disconnect/stale; Scan während Connect? |
| Displays ohne Daten | ESP-NOW Kanal 6 = Hub-AP-Kanal; Display-Firmware auf ESP-NOW? |
| Event-Log leer | Seite neu laden; `/log/events` direkt im Browser |

## Lokaler Log auf Laptop (optional)

```bash
python -c "
import json, time, urllib.request
url = 'http://192.168.1.42/state'  # wifi_ip anpassen
while True:
    d = json.load(urllib.request.urlopen(url, timeout=3))
    print(time.strftime('%H:%M:%S'), d.get('tune_connected'), d.get('rpm'), d.get('esp_now_tx'))
    time.sleep(1)
"
```

Ersetze `192.168.1.42` durch die tatsächliche `wifi_ip` aus dem ersten erfolgreichen `/state`-Aufruf.
