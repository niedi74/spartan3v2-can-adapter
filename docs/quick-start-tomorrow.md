# Quick Start Tomorrow (Fahrzeugtest)

1. Board an 12V, COM9 anschliessen, Monitor starten: `pio device monitor --port COM9 --baud 115200`.
2. Zündung EIN, warten bis Spartan bootet und `123TUNE BLE: scan 10s...` erscheint.
3. Prüfen: `123TUNE BLE: found ...` und danach `subscribe OK` + `CCCD read len=2 : 01 00`.
4. Prüfen: `BLE hub: advertising started` erst nach 123-Connect (oder nach Fallback).
5. M5 in `Spartan Gateway` Modus, Verbindung auf `Spartan3-Hub` kontrollieren.
6. Web prüfen: `http://192.168.0.80/` (oder AP `http://192.168.4.1/`) und Diagnosefelder beobachten.
7. Bei fehlenden Livewerten `!read` im COM9-Monitor senden (v@,10@,11@,12@,13@ Dump).
8. Motor starten und 2-3 Minuten laufen lassen; auf `notify`-Frames und steigendes `tune_rx` achten.
9. Erfolgskriterium: stabile RPM/ADV/MAP im Spartan `/state` und auf dem M5-Display.
10. Bei Abbruch/Timeout: kurze Logs sichern (Zeitpunkt + letzte 30 Zeilen), dann Zündung kurz AUS/EIN und neu testen.

