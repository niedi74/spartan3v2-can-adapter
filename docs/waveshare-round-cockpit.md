# Waveshare Round Cockpit / T2B-Uhr

Zielidee: Das Waveshare ESP32-S3 2.8 Zoll runde IPS-Touchscreen-Board soll vorne ins Cockpit. Es soll als Gateway-Anzeige laufen und im Ruhezustand eine Uhr im Stil der originalen 123TUNE+/T2B-Uhr zeigen.

## Gefundener Stand

- Im ESP123-Repo liegen bisher keine T2B-Uhr-Bildassets.
- In `D:\_claude\M5stack` wurde keine passende T2B-/Uhr-Grafik gefunden.
- Gefunden wurde nur eine Code-Referenz aus der dekompilierten 123Tune-App:
  `D:\_claude\M5stack\xapk_123tune_8_2_0_analysis\decompiled\123TunePlus\_123TunePlus.Dashboard.Views\ClockView.cs`
- Das lokale Projekt `C:\Users\niedi01\SynologyDrive\Dokumente\PlatformIO\Projects\waveshare` ist kein fertiges Round-Cockpit-Projekt, sondern wirkt wie ein anderer/alter Waveshare-Stand.

## Optik aus `ClockView.cs`

- Drei Segment-Labels: Stunden, Doppelpunkt, Minuten.
- Uhrzeitformat: `HH:mm`.
- Aktive Segmentfarbe: `SegmentTextColor`, in der App-Referenz Azure/hellblau.
- Hintergrund-Segmente: `88` bzw. `00` als dunkle, transparente Geistersegmente.
- Doppelpunkt blinkt im Sekundentakt per Fade.
- 5 Taps innerhalb von 5 Sekunden schalten Developer Mode.

## Umsetzungsvorschlag fuer das runde Display

- Kein Bildasset noetig: Die Uhr als echte 7-Segment-Grafik rendern.
- Fullscreen-Uhr als Screensaver mit schwarzem Hintergrund, hellblauen Segmenten und schwachen Geistersegmenten.
- Touch weckt zur Live-Anzeige.
- Lange Inaktivitaet geht zur Uhr zurueck.
- Spaeter: 5-Tap-Shortcut fuer Debug/Dev-Menue uebernehmen.

## Noch offen

- Exaktes Waveshare-Board-Pinout und Displaytreiber pruefen.
- Eigenes PlatformIO-Env/Projekt fuer das Round-Cockpit anlegen.
- Entscheiden, ob es BLE-Gateway-Daten vom Motorraum-Hub liest oder zusaetzlich WLAN/WebSocket/MQTT nutzt.
