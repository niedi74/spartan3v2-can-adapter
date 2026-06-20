# Handoff: COM14 Tischtest-Hub und Android Live-App

Stand: 2026-06-20

## Ziel

- Nur den ESP32-S3 Test-Hub auf COM14 aendern. Der Live-Hub im Bus bleibt unberuehrt.
- Externer 123TUNE-Emulator liefert echte BLE-Frames an den Test-Hub.
- Echter BM6 mit MAC `3c:ab:72:7f:d0:bc` bleibt als BLE-Client aktiv.
- Lambda kann am Tisch fest auf 1.000 oder als Sweep 0.85 bis 1.15 simuliert werden.
- Der Hub schreibt weiterhin das normale CSV mit Zeit, Lambda, 123 und BM6.
- Eine minimale Android-App zeigt RPM, Lambda und BM6 Volt gross an und haelt den Bildschirm an.

## Git und Arbeitsverzeichnis

- Repo: `D:\_claude\spartan3v2-can-adapter`
- Branch: `codex/com14-test-apk`
- Ausgangspunkt: lokaler Branch `work` bei `2220861`
- `work` war 16 Commits vor `origin/work`; nicht auf `origin/main` zuruecksetzen.
- Vorhandene lokale Arbeit wurde beibehalten:
  - `include/spartan_cockpit_frame.h`: ESP-NOW Frame v2 mit 123 Volt/Temp/Coil
  - `sdkconfig.defaults`: NimBLE max connections 5; Encoding inzwischen von UTF-16 nach UTF-8/ASCII korrigiert

## COM14 Ist-Zustand vor neuem Flash

Passiv mit 115200 Baud gelesen:

- 123 BLE verbunden: `conn=1`
- RX-Zahl stieg kontinuierlich, zuletzt groesser 36000
- Emulatorwerte liefen, z. B. RPM 700 bis 2900, ADV und MAP plausibel
- 123 Ping meldete `OK`
- Logging war defekt und spamte alle 500 ms:
  - `fopen(/spiffs/drive.csv) failed`
  - `Logs: create failed`

Damit ist der externe 123-Emulator bereits bestaetigt. Der Hauptfehler auf COM14 ist SPIFFS/CSV.

## Bereits implementiert, noch nicht final getestet

### Hub

- Neues PlatformIO-Environment `motorraum_s3_test` mit Upload/Monitor COM14.
- BM6-Defaultziel auf `3c:ab:72:7f:d0:bc` gesetzt.
- Persistenter Lambda-Testmodus:
  - `off`
  - `fixed`: Lambda 1.000, 780 C, Status OK
  - `sweep`: Dreieck 0.85 bis 1.15, 20 Sekunden Gesamtperiode
- Testdaten verwenden Quelle `TEST` und laufen durch den normalen Snapshot.
- API-Felder:
  - `lambda_test_mode`
  - `lambda_test_active`
  - `log_error`
- API: `POST /lambda_test` mit `mode=off|fixed|sweep`.
- WebGUI: Setup-Karte mit AUS, Fest 1.000 und Sweep.
- Serial:
  - `lambda test off`
  - `lambda test fixed`
  - `lambda test sweep`
- Logging-Recovery:
  - Mount und Schreibprobe `/.rwtest`
  - Einmaliges Format nur wenn keine bestehenden Logs vorhanden sind
  - Fehler werden einmal gemeldet und als `log_error` gespeichert
  - `POST /log/fs_reset` und Serial `hub logfs reset`

### Noch offen

1. Hub-Build `pio run -e motorraum_s3_test` fertig laufen lassen. Erster Aufruf erreichte nur das 120-s-Timeout.
2. Compilerfehler korrigieren, falls der laengere Build welche zeigt.
3. Aktuelle COM14-Konfiguration sichern, soweit ueber `/state` oder Serial erreichbar.
4. COM14 vollstaendig loeschen und Test-Environment flashen.
5. Features nach Erase setzen:
   - `emu123=off`
   - `ble123=on`
   - `blebm6=on`
   - `log=on`
   - BM6 `3c:ab:72:7f:d0:bc`
6. SPIFFS, CSV, 123, BM6 und Lambda-Test auf Hardware pruefen.
7. Android-App unter `android-app/` erstellen.
8. Android-SDK ist auf diesem Rechner derzeit nicht gefunden; Toolchain muss eingerichtet werden.
9. APK bauen und auf S24+ installieren/testen.
10. Committen und Branch pushen.

## Build-, Flash- und Monitorbefehle

```powershell
cd D:\_claude\spartan3v2-can-adapter
pio run -e motorraum_s3_test
pio run -e motorraum_s3_test -t erase --upload-port COM14
pio run -e motorraum_s3_test -t upload --upload-port COM14
pio device monitor --port COM14 --baud 115200
```

Vor einem rekursiven oder vollstaendigen Erase nochmals sicherstellen, dass COM14 der Tisch-Hub ist.

## Android-App Vertrag

- Native Kotlin-App im Ordner `android-app/`.
- Polling alle 500 ms:
  - `GET http://<hub>/api/status?client=android-live`
- Hubadresse persistent speichern.
- Preset `192.168.4.1` plus freie LAN-/Hotspot-IP.
- Gross: RPM, Lambda, BM6 Volt.
- Klein: ADV, MAP, BM6 Temperatur, Verbindungen, Logstatus und Loggroesse.
- Nach 2 Sekunden ohne Antwort: offline und letzte Werte abgeblendet.
- `FLAG_KEEP_SCREEN_ON` in der Live-Activity.
- Kein lokales App-Logging und kein CSV-Download in Version 1.

## Abnahmetests

1. Keine wiederholten SPIFFS/fopen-Fehler im Serial-Log.
2. `/api/status`: `log_ready=true`, `log_error=""`, `log_current_bytes` steigt.
3. 123: verbunden, `tune_rx` steigt, RPM/ADV/MAP aendern sich.
4. BM6: verbunden, RX steigt, Spannung und Temperatur sind frisch.
5. Lambda fixed zeigt exakt 1.000; Sweep bleibt zwischen 0.85 und 1.15.
6. CSV enthaelt Zeit, Quelle TEST, Lambda, 123 und BM6 in derselben Zeile.
7. Neustart behaelt Lambda-Modus und BLE-Ziele.
8. Android-App funktioniert ueber Hub-AP und LAN-IP; Bildschirm bleibt an.

