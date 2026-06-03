# Known-good Stand - 2026-06-03

Dieser Stand fasst die Tests vom 2026-06-02 und 2026-06-03 zusammen.

## Erfolgreich getestet

| Bereich | Status | Notiz |
|---|---|---|
| Spartan Lambda via CAN | OK | CAN ID 1024 / `0x400`, Quelle `CAN`, Lambda am M5 Dial sichtbar |
| Spartan UART | OK | Direkte Befehle wie `GETFW`, CAN-Config und Spartan-Einstellungen getestet |
| Reed Speed | OK | Frequenz konnte erzeugt werden, GPIO27, 10 Pulse pro Radumdrehung |
| 123TUNE+ BLE | OK | Verbindung im Auto erfolgreich, Werte nur mit Zuendung sinnvoll |
| BM6 BLE | OK | Batteriemonitor im Hub/WebGUI vorgesehen |
| M5 Dial Gateway | OK | Dial zeigt Gateway-Daten, Lambda sichtbar |
| 16x2 LCD | OK | Mit 5 V Display-Versorgung und 3.3 V I2C-Logik/ESP stabil lesbar |
| WebGUI | OK | Mobile-freundlicher Live/Setup-Aufbau, Roh-JSON eingeklappt |
| Betriebsstunden | OK | Gesamt, Engine und Sensorstunden in `/state` sichtbar |

## WebGUI-IP aus letztem Test

| Geraet | IP |
|---|---|
| Spartan ESP32 | `10.155.167.69` |
| M5 Dial | `10.155.167.107` |
| Setup AP | `192.168.4.1` |

IPs koennen sich bei Handy-Hotspot/Home-WLAN aendern.

## Wichtige Hardware-Details

- Spartan Grau: GND gemeinsam mit ESP32.
- Spartan Gelb/Orange wurden final direkt bzw. mit Pegelanpassung getestet; bei neuer Hardware wieder vorsichtig am Tisch pruefen.
- CAN Adapter: CAN H/L am Spartan, RX/TX am ESP.
- Spartan braunes Kabel: Heater/Status analog an GPIO35.
- Gruenes Analog-Kabel ist derzeit nicht aktiv (`ENABLE_SPARTAN_ANALOG=0`), weil CAN die Lambda-Daten liefert.

## Logging-Ziel

Das Setup ist primaer fuer echtes Logging gedacht: Lambda, 123TUNE+, BM6 und Geschwindigkeit synchron erfassen. Die Sonde sitzt nach dem Endtopf vor der letzten Biegung; Falschluft im Abgastrakt kann Lambda magerer erscheinen lassen als die echte Verbrennung.
