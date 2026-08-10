# Feature-Ideen / Backlog

Ideen für später, noch nicht umgesetzt — kein Anspruch auf Priorität oder Reihenfolge.

## USB-only-123ignition per Hub anbinden (kein Front-Heck-Kabel)

**Status:** Idee/Backlog, nicht umgesetzt.

Manche 123ignition-Einheiten haben keine BLE-Variante, sondern nur einen USB-Anschluss
(Szenario: Motorbauer-Freund des Nutzers, eigenes Fahrzeug — nicht das eigene Setup des
Nutzers). Ziel: die Auswertung/Anzeige soll im Cockpit sitzen, die 123 wie üblich im
Motorraum — ohne ein dediziertes Kabel quer durchs Fahrzeug (hinten→vorne) verlegen zu
müssen.

**Favorisierter Ansatz:** Kein eigenes "USB→BLE-Dongle"-Gadget bauen. Stattdessen einen
normalen Hub (wie dieses Projekt) direkt neben der 123 im Motorraum aufbauen, per kurzem
lokalem USB-Kabel mit der 123 verbunden. Der Hub überträgt dann nach vorne über die
bereits vorhandenen, bewährten Wege (WiFi-AP und/oder CAN-Bus 0x510-0x514) — analog zum
bestehenden eigenen Fahrzeug-Setup (Hub im Motorraum, Display vorne, kein dediziertes
Kabel dazwischen).

**Offener Kernpunkt (vor Umsetzung zu klären):** Der Hub-Code kennt aktuell **kein
USB-Protokoll für die 123ignition selbst**:
- Die bestehende `Serial2`-UART-Anbindung (`sendSpartanUartCommand()`, `GETFW`/`GETCANID`
  etc. in `src/main.cpp`) spricht mit dem **Spartan**-Controller, nicht mit der 123.
- Die 123-Anbindung im Hub läuft ausschließlich über **BLE** (`include/hub_ble.h`,
  `tuneClient`, NUS-Protokoll — siehe Kommentare zu Feldern 0x30/0x31/0x32/0x33/0x41/0x42).
- Es gibt im Repo keine Dokumentation/Referenz zum tatsächlichen Byte-Protokoll, das eine
  123ignition-Einheit über ihre USB-Buchse spricht (z. B. für die offizielle
  "123map"-PC-Software). Muss vor einer echten Umsetzung recherchiert/reverse-engineered
  werden.

## Klopfsensor (Knock Sensor) am Hub

**Status:** Idee/Backlog, nicht umgesetzt. Recherche abgeschlossen, Hardware noch nicht
beschafft.

**Ziel:** Klopferkennung als Monitoring-/Log-Funktion am Hub, fürs Boxer-Motor-Setup
(VW T3, 2 Zylinderbänke à 2 Zylinder).

**Wichtige Einschränkung:** Die 123ignition-Einheiten (123\TUNE+ etc.) haben **keinen
eigenen Klopfsensor-Eingang** — reine Zündsteuerung ohne Closed-Loop-Klopfregelung. Ein
Klopfsensor am Hub kann den Zündwinkel also **nicht automatisch** schützen, nur
**anzeigen/loggen** (Display, CSV-Log, ggf. Alarm). Eine automatische Rückkopplung auf den
Zündwinkel müsste als eigene Hub-Logik über den bestehenden Live-Tune-Pfad (0x513 /
`/api/tune/live`) gebaut werden — technisch möglich, da der Hub dort ohnehin schon
schreibend auf die 123 zugreift, aber ein zusätzliches Stück Arbeit, kein Selbstläufer.

### Sensor

- **Entscheidung (Stand dieser Session): Bosch 0 261 231 095/096** (breitbandiger
  "Flat Response"-Universalsensor, in der DIY-ECU-Szene verbreitet) — statt des von Bylund
  fürs Knock-Shield-Referenzdesign empfohlenen KS4-P (0 261 231 173).
  **Zu beachten:** Das Bylund-Referenzdesign (s.u.) ist auf den KS4-P abgestimmt
  (Eingangsverstärkung per R1-R4, Standard-Gain 1x bei 100k-Widerständen laut Technical
  Manual). Bei abweichender Empfindlichkeit des 095/096 ggf. R1-R4 anpassen (Formel:
  A_IN = R2/R1 bzw. R4/R3) oder per SPI-Programmable-Gain (0.111-2.000) ausgleichen — vor
  dem Bau/Bestücken gegenchecken, nicht blind übernehmen.
- 1 Sensor pro Zylinderbank (2 Stück für den Boxer), zentral zwischen den beiden Zylindern
  einer Bank direkt ins Kurbelgehäuse verschraubt (Metallkontakt, kein Dichtmittel
  drunter). Beim VW-T3-Spätmodell/Porsche-914-Motor (Typ 4, baugleiche Motorfamilie) ist
  dafür die Bohrung bekannt, an der sonst der Temperatursensor sitzt — ggf. Platzkonflikt
  mit dem CHT-Sensor-Projekt (`spartan3-headtemp`) prüfen, falls dieselbe Bohrung gemeint
  ist.
- Verworfen: Bosch 0 261 231 038 (resonant/frequenzfest auf ein bestimmtes OEM-Motor-
  Modell abgestimmt — für einen Boxer-Umbau ohne bekannte Resonanzfrequenz eher
  ungeeignet), Magneti Marelli 064836009010 (Kompatibilität ungeprüft, keine belastbaren
  Daten gefunden), Bosch KS4-P (0 261 231 173, Bylund-Empfehlung — Board discontinued,
  siehe unten, Sensor selbst aber weiterhin separat erhältlich).

### Auswerte-Hardware: "Knock Shield for Arduino" (Bylund Automotive)

**Status-Update (Bylund-Produkttabelle geprüft): BEIDE Knock-Shield-Versionen (6 000 000 011
und die neuere 6 000 000 021) sind "Discontinued"** — beim Hersteller aktuell NICHT mehr
kaufbar. Das "fertig bestückt kaufen, kein Löten nötig"-Vorhaben funktioniert also nicht
mehr wie ursprünglich gedacht. Optionen:
- Gebraucht/NOS (eBay, Kleinanzeigen, Restposten) suchen.
- **DIY via PCBA-Assembly-Service** (JLCPCB/PCBWay bestücken nach dem offen dokumentierten
  Schaltplan, s.u.) bleibt der verlässliche Weg — der TPIC8101-Chip selbst ist bei
  Mouser/DigiKey weiterhin regulär lieferbar, nur Bylunds fertiges Board ist es nicht.

Ursprünglich fertig bestücktes Board (Referenz, aktuell discontinued, s.o.):
<https://www.bylund-automotive.com/educative/knock/>

- **Chip:** TI **TPIC8101DW**, pinkompatibel zu Renesas HIP9011ABZ.
- **2 Kanäle** (Channel 1/2 per SPI-Kommando wählbar) — passt exakt zu 2 Sensoren/Bänken.
- Bauform: Arduino-Uno-Shield (Stiftleisten) — für den ESP32-Hub nicht aufstecken, sondern
  per Drahtbrücke direkt verdrahten:
  - **SPI** (MISO/MOSI/CLK/CS) über Header X1.
  - **1 digitaler GPIO** als Mess-Fenster-Trigger ("HOLD"-Pin, im Beispielcode Arduino-Pin
    4) — Fenster start/stop synchron zur Zündung/zum Kurbelwinkel.
  - **1 Analog-Eingang** liest den Signalpegel (`UA`-Ausgang über Header X2) — am ESP32
    ein ADC-Pin.
  - 5V-Versorgung über Header X4.
- Anschluss-Klemme X5 (4-polig) für die Sensoren: Pin1=Signal Kanal1, Pin2/3=GND,
  Pin4=Signal Kanal2.
- SPI-Protokoll komplett im Technical Manual dokumentiert (Prescaler/Kanalwahl/
  Bandpass-Frequenz/Gain/Integrationszeit als einzelne SPI-Bytes, mit fertigen
  Werte-Tabellen) — ließe sich direkt in Hub-Code übernehmen.
- Bandpass-Frequenz-Richtwert per Formel aus dem Bohrungsdurchmesser:
  `f_calc[kHz] = 3 * C / (2 * π * Bohrung[mm])`, C ≈ 1200 m/s (nur Startwert, echte
  Abstimmung braucht Messung/Feintuning).
- Referenz-Beispielcode (Arduino, GPLv3, als Vorlage für eine ESP32-Portierung geeignet):
  - <https://github.com/Bylund/Knock-Shield-Example> — einfache Version, Kanäle
    abwechselnd per festem Zeitfenster (3 ms) abgefragt, SD-Karten-Logging optional.
  - <https://github.com/Bylund/Knock-Shield-Interrupt-Example> — Tacho-/RPM-synchrone
    Version: Interrupt auf einem Tacho-Signal-Pin, Mess-Fenster-Timing und
    Integrationszeitkonstante werden abhängig von der Drehzahl angepasst (Tabelle
    1000-9000 RPM -> 40-320 µs Integrationszeit). Für den Hub bietet sich an, statt eines
    separaten Tacho-Kabels die ohnehin schon vom 123 per BLE ankommende Drehzahl
    (`tuneRpm`) als Zeitbasis zu nutzen — kein zusätzliches Kabel zum Kurbelwellensensor
    nötig, dafür kein exakt kurbelwinkelsynchrones Fenster (grobe Näherung statt
    Cycle-genauer Messung pro Zylinder).
- Fertigungs-/Bezugsalternative, falls das fertige Shield nicht verfügbar/zu teuer ist:
  eigenes kleines TPIC8101-Board nach Bylund-Referenzschaltung (KiCad) via
  JLCPCB-/PCBWay-SMT-Assembly-Service bestücken lassen (kein Handlöten, Kosten für
  Kleinserie überschaubar).

### Offene Punkte

- Magneti-Marelli-Sensor-Kompatibilität ungeprüft.
- Genaue Bohrposition am eigenen Motorblock noch nicht verifiziert (nur die
  914/Typ-4-Referenzposition als Anhaltspunkt).
- Kein CAN-Frame/Cockpit-Feld für Klopfpegel vorgesehen — müsste bei Umsetzung neu
  vergeben werden (nächste freie Ext-ID nach den bestehenden 0x510-0x514, siehe
  Absprache mit dem Display/Hub-Team wegen ID-Kollisionen).
- Automatische Zündwinkel-Rückkopplung bei Klopfen ist NICHT Teil der Grundidee (siehe
  Einschränkung oben) — müsste als separate Entscheidung/Sicherheitsdesign behandelt
  werden, falls gewünscht.
