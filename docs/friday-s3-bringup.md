# ESP32-S3 Motorraum Bring-up - Freitag 2026-06-05

Ziel: Den bekannten Motorraum-Hub auf dem neuen ESP32-S3 WROOM-1-N16R8 / ESP32-S3-DevKitC-1 starten und zuerst am Schreibtisch, danach im Auto testen.

## Build

- Branch: `codex/esp32s3-motorraum-hub`
- PlatformIO env: `motorraum_s3_devkitc`
- Build-Befehl: `pio run -e motorraum_s3_devkitc`
- Upload-Befehl: `pio run -e motorraum_s3_devkitc -t upload`
- Setup-AP: `Spartan3-Setup`
- Setup-AP Passwort: `lambda123`

Hinweis: Das neue Env nutzt `esp32-s3-devkitc-1` mit 16 MB Flash-Override. PSRAM ist absichtlich noch nicht aktiviert, weil der Motorraum-Code sie nicht braucht.

## Pin-Plan fuer ersten Test

| Signal | GPIO | Hinweis |
|---|---:|---|
| Status LED | 2 | Bei S3-Board ggf. pruefen, ob LED dort passt |
| LCD SDA | 21 | I2C Display 16x2 |
| LCD SCL | 22 | I2C Display 16x2 |
| CAN RX | 25 | SN65HVD230 / VP230 RX |
| CAN TX | 26 | SN65HVD230 / VP230 TX |
| Spartan UART RX | 16 | ESP empfaengt vom Spartan |
| Spartan UART TX | 17 | ESP sendet zum Spartan |
| Reed Speed | 27 | 10 Magnete pro Radumdrehung |
| Spartan Heater Analog | 35 | braunes Spartan-Kabel, ADC |

Vor dem festen Verloeten am S3-Expansion-Board die Beschriftung der Pins gegen das DevKitC-1 Pinout pruefen.

## Reihenfolge am Tisch

1. S3 ohne Spartan/CAN flashen.
2. Serial Monitor mit 115200 starten.
3. AP `Spartan3-Setup` sehen und WebGUI auf `http://192.168.4.1/` oeffnen.
4. WLAN in der WebGUI setzen: zuerst Handy-Hotspot `Android-AP1`, alternativ Home `ZOO_station`.
5. 16x2 LCD testen: Boottext, IP, Datenstatus.
6. Spartan CAN anschliessen und Lambda-Quelle `CAN` pruefen.
7. Reed-Frequenz einspeisen und km/h/Hz im WebGUI pruefen.
8. UART Orange/Gelb/Grau anschliessen und `GETFW`, `GETAFRM`, `GETLAMZEROV`, `GETLAMFIVEV` testen.
9. BM6 Scan/Verbindung pruefen.
10. 123TUNE+ nur mit Zuendung/Auto pruefen.
11. M5 Dial als Client verbinden und Gateway-Payload pruefen.

## Auto-Test

- ESP32-S3 in den Motorraum, Laptop/M5 Dial vorne.
- Handy-Hotspot als zentraler AP mit Internet/NTP.
- Erst im Stand pruefen: Lambda, BM6, Reed, 123 BLE.
- Danach kurze Fahrt mit Log; Lambda-Werte wegen Sondenposition am Ende des Auspuffs auf Falschluft-Einfluss interpretieren.

## Bekannte Eigenheit

Frische S3-Hardware hat noch keine gespeicherten WLAN-Daten. `ZOO_station` ist als Preset vorbereitet, aber das Passwort wird nicht ins Git geschrieben. Einmal ueber `Spartan3-Setup` speichern.
