# Waveshare Round Cockpit / T2B-Uhr

Zielidee: Das Waveshare ESP32-S3 2.8 Zoll runde IPS-Touchscreen-Board soll vorne ins Cockpit. Es soll als Gateway-Anzeige laufen und im Ruhezustand eine Uhr im Stil der originalen 123TUNE+/T2B-Uhr zeigen.

## Gefundener Stand

- Die T2B/VDO-Uhrbilder wurden im GitHub-Branch `codex/spartan3v2-can-adapter` gefunden und in diesen Branch uebernommen.
- Asset-Ordner: `assets/t2b-clock/`
- `vdo_clock_preview.png`: komplette Uhr mit goldenem Ring.
- `vdo_clock_nobezel.png`: Zifferblatt ohne Ring, gut als Render-/Overlay-Basis.
- `vdo_clock_sizes.png`: Groessenvergleich fuer kleines/mittleres/volles Display.
- In `D:\_claude\M5stack` wurde keine weitere passende T2B-/Uhr-Grafik gefunden.
- Gefunden wurde nur eine Code-Referenz aus der dekompilierten 123Tune-App:
  `D:\_claude\M5stack\xapk_123tune_8_2_0_analysis\decompiled\123TunePlus\_123TunePlus.Dashboard.Views\ClockView.cs`
- Das lokale Projekt `C:\Users\niedi01\SynologyDrive\Dokumente\PlatformIO\Projects\waveshare` ist kein fertiges Round-Cockpit-Projekt, sondern wirkt wie ein anderer/alter Waveshare-Stand.

## Analoge VDO/T2B-Optik

- Schwarzes Zifferblatt mit weissen Zahlen und Strichen.
- Roter Sekundenzeiger, weisse Stunden-/Minutenzeiger.
- Dezente Beschriftung `VDO`, `Quartz-Zeit`, `12V 1.2W`.
- Goldener Ring in der Preview fuer die T2B-Optik.
- Fuer das runde Waveshare-Display ist `vdo_clock_nobezel.png` wahrscheinlich die beste Basis, damit Zeiger dynamisch gezeichnet werden koennen.

## Digitale Optik aus `ClockView.cs`

Zusatzreferenz aus der 123Tune-App:
- Drei Segment-Labels: Stunden, Doppelpunkt, Minuten.
- Uhrzeitformat: `HH:mm`.
- Aktive Segmentfarbe: `SegmentTextColor`, in der App-Referenz Azure/hellblau.
- Hintergrund-Segmente: `88` bzw. `00` als dunkle, transparente Geistersegmente.
- Doppelpunkt blinkt im Sekundentakt per Fade.
- 5 Taps innerhalb von 5 Sekunden schalten Developer Mode.

## Umsetzungsvorschlag fuer das runde Display

- Fullscreen-Uhr als Screensaver mit `vdo_clock_nobezel.png` oder `vdo_clock_preview.png`.
- Zeiger dynamisch ueber das Zifferblatt rendern, damit die Uhr wirklich laeuft.
- Optional zweiter Screensaver-Modus: digitale 7-Segment-Uhr nach 123Tune-App-Referenz.
- Touch weckt zur Live-Anzeige.
- Lange Inaktivitaet geht zur Uhr zurueck.
- Spaeter: 5-Tap-Shortcut fuer Debug/Dev-Menue uebernehmen.

## Noch offen

- Exaktes Waveshare-Board-Pinout und Displaytreiber pruefen.
- Eigenes PlatformIO-Env/Projekt fuer das Round-Cockpit anlegen.
- Entscheiden, ob es BLE-Gateway-Daten vom Motorraum-Hub liest oder zusaetzlich WLAN/WebSocket/MQTT nutzt.
