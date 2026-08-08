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
