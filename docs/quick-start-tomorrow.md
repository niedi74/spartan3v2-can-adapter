# Quick Start Tomorrow (Fahrzeugtest)

## GitHub Links
- Repo: `https://github.com/niedi74/spartan3v2-can-adapter`
- Issues: `https://github.com/niedi74/spartan3v2-can-adapter/issues`
- Letzter Doku-Stand: `https://github.com/niedi74/spartan3v2-can-adapter/blob/main/docs/handoff_2026-05-26.md`
- Diese Quick-Start Datei: `https://github.com/niedi74/spartan3v2-can-adapter/blob/main/docs/quick-start-tomorrow.md`
- Hauptcode: `https://github.com/niedi74/spartan3v2-can-adapter/blob/main/src/main.cpp`
- Wichtige Commits:
  - `https://github.com/niedi74/spartan3v2-can-adapter/commit/eed24d8`
  - `https://github.com/niedi74/spartan3v2-can-adapter/commit/a192ff9`
  - `https://github.com/niedi74/spartan3v2-can-adapter/commit/7dd8abc`

1. Board an 12V, COM9 anschliessen, Monitor starten: `pio device monitor --port COM9 --baud 115200`.
2. Zuendung EIN, warten bis Spartan bootet und `123TUNE BLE: scan 10s...` erscheint.
3. Pruefen: `123TUNE BLE: found ...` und danach `subscribe OK` + `CCCD read len=2 : 01 00`.
4. Pruefen: `BLE hub: advertising started` erst nach 123-Connect (oder nach Fallback).
5. M5 in `Spartan Gateway` Modus, Verbindung auf `Spartan3-Hub` kontrollieren.
6. Web pruefen: `http://192.168.0.80/` (oder AP `http://192.168.4.1/`) und Diagnosefelder beobachten.
7. In Web-Hauptkarte muss `123 RPM / ADV / MAP` sichtbar sein.
8. Im 16x2 Display muss ein kurzer 123 Status erscheinen: `123:OK`, `123:SCAN` oder `123:DISC`.
9. Bei fehlenden Livewerten `!read` im COM9-Monitor senden (v@,10@,11@,12@,13@ Dump).
10. Motor starten und 2-3 Minuten laufen lassen; auf `notify`-Frames und steigendes `tune_rx` achten.
11. Erfolgskriterium: stabile RPM/ADV/MAP im Spartan `/state` und auf dem M5-Display.
12. Bei Abbruch/Timeout: kurze Logs sichern (Zeitpunkt + letzte 30 Zeilen), dann Zuendung kurz AUS/EIN und neu testen.
