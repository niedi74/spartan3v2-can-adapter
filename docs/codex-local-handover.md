# Codex Local Handover (Kurz)

## Projekt
- Repo: `https://github.com/niedi74/spartan3v2-can-adapter`
- Branch lokal: `codex/spartan3v2-can-adapter`
- Ziel: Spartan als BLE Gateway (123TUNE + Lambda -> M5/Waveshare)

## Einstieg in 2 Minuten
1. `cd C:\Users\niedi01\SynologyDrive\Dokumente\ESP123`
2. `pio run -e motorraum`
3. `pio run -e motorraum -t upload --upload-port COM9`
4. `pio device monitor --port COM9 --baud 115200`

## Wichtige Dateien
- `src/main.cpp` (BLE, CAN, Web, Display)
- `docs/handoff_2026-05-26.md` (voller Verlauf/Stand)
- `docs/quick-start-tomorrow.md` (Fahrzeug-Checkliste)

## Bereits umgesetzt
- 123 BLE Connect + Subscribe + Decode stabil im Normalfall
- Dashboard-V2 mit Diagnose
- LCD zeigt 123 Kurzstatus (`123:OK/SCAN/DISC`) und wechselt auf 123 Werte-Seite
- Read-Dump Kommandos: `!read`, `!v@`, `!10@`, `!11@`, `!12@`, `!13@`

## Offene Punkte (bewusst vertagt)
- Binary BLE Payload + CRC (M5 Kompatibilitaet beachten)
- BLE Bonding/Security erst nach stabilem Livebetrieb
- Optional `/log` Ringbuffer Endpoint

## Praktische Diagnose-Regeln
- `reason=520` nach Zuendung AUS ist erwartbar.
- Wichtig ist, dass bei Motorlauf `notify len=5` kommt und `tune_rx` steigt.
- Web und Display muessen dieselben Kernwerte zeigen (RPM/ADV/MAP).

