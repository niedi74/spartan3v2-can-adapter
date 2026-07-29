# CHT-Sensor (satellites/cht-sensor)

Zylinderkopftemperatur (CHT) am VW T3 Boxer, gemessen mit MAX6675-Thermoelement-
Modulen in den originalen M10x1-Bohrungen nahe der Zündkerze. Läuft auf einem
eigenständigen ESP32-C6 (WiFi + eingebauter CAN-Controller/TWAI) als Satellit
neben dem eigentlichen Hub aus diesem Repo -- eigenes PlatformIO-Projekt,
separat bauen: `cd satellites/cht-sensor && pio run`.

## Hardware

- ESP32-C6-DevKitC-1
- bis zu 4x MAX6675 (SPI-Read-only, SCK+SO geteilt, je Fühler eigene CS-Leitung)
- CAN-Anbindung vorbereitet (Frame 0x520), aktiv erst mit SN65HVD230-Transceiver

## Pin-Plan

| Funktion | GPIO |
|---|---|
| MAX6675 SCK | 6 |
| MAX6675 SO | 7 |
| CS Zylinder 1 | 10 |
| CS Zylinder 2 | 11 |
| CS Zylinder 3 | 18 |
| CS Zylinder 4 | 19 |
| CAN TX (vorbereitet) | 2 |
| CAN RX (vorbereitet) | 3 |

Vollständiger Plan auch live in der WebGUI unter Tab „Anschlussplan".

## WLAN

- Eigener Access Point (SSID/Passwort/IP-Range in der WebGUI änderbar,
  Default `CHT-Sensor` / `chtboxer1` / `192.168.7.1`)
- Zusätzlich Verbindung ins Heimnetz möglich (DHCP oder statische IP,
  MAC-Override optional)
- WebGUI erreichbar unter `/` und `/head`

## CAN (vorbereitet, noch inaktiv)

Frame `0x520`, 8 Byte, big-endian: 4x `int16` Zylinderkopftemperatur × 10
(0.1 °C Auflösung), `INT16_MIN` = Fühler offen/kein Kontakt. Aktivierung über
Build-Flag `ENABLE_CAN=1`, sobald der Transceiver verbaut ist.

## API

`GET /api/status` liefert JSON mit den aktuellen Temperaturen, WLAN- und
CAN-Status.

## Verwandtes im Projekt

- Hub-Firmware (Lambda/CAN/BLE): Repo-Root dieses Repos
- [spartan3-hub-app](https://github.com/niedi74/spartan3-hub-app) — Android-Displays
- CAN-Kontrakt der Cockpit-Frames (0x510/0x511/0x512): `../../docs/lambda-status-logik.md`
