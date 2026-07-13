# Lambda-Status-Logik (`status` / `status_code`)

Dokumentiert, wie das Feld `status` (Text) bzw. `status_code` (Zahl 0–3) in
`/api/status` und `/state` zustande kommt — und warum es nach dem Einschalten
eine Weile dauert, bis dort `OK` steht.

## Die vier Zustände

Quelle: `statusTextC()` in [`src/main.cpp`](../src/main.cpp) (Suche nach `statusTextC`).

| `status_code` | `status` (Text) | Bedeutung |
|---|---|---|
| `0` (oder jeder unbekannte Wert) | `ERR` | Fehler / kein gültiger Status vom Controller |
| `1` | `WAIT` | Sonde noch nicht bereit (Aufwärmphase, ganz am Anfang) |
| `2` | `HEAT` | Sonde heizt (Wideband-Lambdasonde braucht Betriebstemperatur) |
| `3` | `OK` | Sonde bereit, Messwert gültig und belastbar |

```cpp
const char *statusTextC(uint8_t status)
{
  switch (status) {
    case 1: return "WAIT";
    case 2: return "HEAT";
    case 3: return "OK";
    default: return "ERR";
  }
}
```

## Woher der Wert kommt (je nach aktiver Datenquelle)

Der Hub hat vier mögliche Datenquellen für die Lambda-Messung (`source`-Feld in
`/api/status`: `CAN`, `DEMO`, `TEST`, `ADC`). Jede setzt `status` anders:

### 1. CAN (echter Spartan 3 v2, Normalbetrieb) — `hub_can.h`

Der `status`-Wert kommt **direkt vom Spartan-Controller selbst**, unverändert
aus **Byte 3** des empfangenen CAN-Frames (ID = `spartanCanIdCfg`, Default
`0x400`):

```cpp
fresh.status = message.data[3];   // Byte 3 des CAN-Frames = Status, roh vom Spartan
```

Der Hub interpretiert dieses Byte **nicht selbst** — er gibt exakt weiter, was
der Spartan meldet. `WAIT`→`HEAT`→`OK` ist also die **Aufwärmsequenz der
Lambdasonde selbst** (eine beheizte Breitband-Lambdasonde braucht nach dem
Einschalten typischerweise **10–30 Sekunden**, bis sie ihre Betriebstemperatur
erreicht und zuverlässige Werte liefert — die genaue Dauer bestimmt der
Spartan-Controller, nicht die Hub-Firmware).

**Bis `OK` erscheint, tut der Hub nichts weiter** als abzuwarten und
weiterzuleiten — es gibt keinen Retry/Timeout auf Hub-Seite, der das
beschleunigt. Das ist normales, erwartetes Verhalten der Sonde.

### 2. DEMO-Modus (`updateDemo()`, kein CAN-Modul vorhanden)

Simuliert die Aufwärmsequenz zeitgesteuert, unabhängig von echten Daten:

```cpp
if (now < 8000) {          // erste 8 Sekunden nach Boot
  fresh.status = 2;        // HEAT
} else {
  fresh.status = 3;        // danach dauerhaft OK
}
```

### 3. TEST-Modus (`updateLambdaTest()`, Lambda-Test fixed/sweep über die WebGUI)

Simuliert seit dem 2026-07-13-Update dieselbe Aufwärmsequenz wie echte CAN-
Hardware — ab dem Zeitpunkt, an dem der Testmodus aktiviert wird
(`lambdaTestStartMs`, jede Neuaktivierung startet die Sequenz neu):

| Zeit seit Aktivierung | `status_code` | `status` |
|---|---|---|
| 0–3 s | `1` | `WAIT` |
| 3–10 s | `2` | `HEAT` |
| ab 10 s | `3` | `OK` |

Damit lässt sich die `WAIT`→`HEAT`→`OK`-Sequenz **auch am Schreibtisch ohne
echten Spartan** beobachten — einfach `/lambda_test` mit `mode=fixed` oder
`mode=sweep` aufrufen und `status`/`status_code` in `/api/status` für ~10 s
verfolgen.

### 4. ADC-Fallback (`updateAnalog()`, analoger Spannungseingang statt CAN)

Meldet ebenfalls **immer `OK`** (`status = 3`) — die Analogspannung liefert
keine eigene Statusinformation, nur den Lambdawert selbst.

## CAN-Cockpit-Frame (0x510) — eigener Wire-Weg zu Displays

Displays, die **direkt am CAN-Bus** hängen (statt per HTTP `/api/status` zu
pollen), bekommen den Status über den 0x510-Cockpit-Frame, den der Hub sendet
(`cockpitCanIdCfg`, ~10 Hz). Das ist ein **eigenes, schlankes 8-Byte-Format**
(NICHT das reichhaltigere `SpartanCockpitFrame` aus
[`include/spartan_cockpit_frame.h`](../include/spartan_cockpit_frame.h) — das
ist 17 Byte, passt nicht in einen einzelnen klassischen CAN-Frame mit DLC=8,
und wird im Hub-Code aktuell nirgends aufgerufen).

**Bug gefunden + gefixt (2026-07-13):** `flags & 0x01` (`kCockpitFlagLambdaValid`)
bedeutete bisher nur "irgendein Lambda-Wert kam an" — das ist während
`WAIT`/`HEAT` genauso `true` wie bei `OK`, weil `SpartanReading.valid` in
jedem Lesepfad (CAN/Demo/Test/ADC) unconditional gesetzt wird. Ein Display,
das nur `flags & 0x01` prüft, konnte die Sonden-Aufwärmphase über CAN also
**nicht** erkennen — anders als über HTTP, wo `status`/`status_code` immer
den echten Wert trägt.

**Fix:** `status_code` (0–3) wird jetzt zusätzlich in **Bits 2-3** des
`flags`-Bytes gepackt (Byte 7 des Frames), rückwärtskompatibel (Bit 0/1
unverändert):

```
Byte 7 (flags):  Bit0=LambdaValid  Bit1=TuneFresh  Bits2-3=status_code  Bits4-7=reserviert
```

Display-seitig zum Auswerten:
```
status_code = (flags >> 2) & 0x03;   // 0=ERR 1=WAIT 2=HEAT 3=OK
bereit      = status_code == 3;      // entspricht dem HTTP-Feld status=="OK"
```

Komplettes Byte-Layout: siehe Kommentar am Kopf von
[`include/hub_can.h`](../include/hub_can.h).

## Wichtig: nicht verwechseln mit `heater_status_code`

Es gibt ein **zweites, unabhängiges** Statusfeld: `heater_status_code`
(`updateHeaterAnalog()`), nur aktiv wenn `ENABLE_SPARTAN_HEATER_ANALOG=1`.
Das liest eine **separate Analogspannung** (Heizungs-Status-Pin,
`SPARTAN_HEATER_PIN`) und klemmt sie auf 0–3:

```cpp
if      (heaterStatusVolts < 0.5f) heaterStatusCode = 0;
else if (heaterStatusVolts < 1.5f) heaterStatusCode = 1;
else if (heaterStatusVolts < 2.5f) heaterStatusCode = 2;
else                                heaterStatusCode = 3;
```

Das ist ein **Fallback-Signal für die Heizungssteuerung**, kein Ersatz für den
CAN-`status`. Wird u. a. für `sensorActive` (Motor-/Sensor-Aktivitätserkennung)
kombiniert genutzt:

```cpp
const bool sensorActive = snapshot.status == 3 || heaterStatusCode >= 2 || snapshot.temperatureC >= 700;
```

## Verifikationsstand

- **Live-Setup (Fahrzeug, echter Spartan 3 v2 am CAN-Bus):** bereits vom User
  verifiziert — der `/api/status`-Pfad inkl. `status`/`status_code` funktioniert
  dort mit echten CAN-Frames.
- **Tischtest-Aufbau (Schreibtisch, Test-Hub):** hier hängt kein echter Spartan,
  nur der `emu123`-BLE-Emulator (treibt RPM/ADV/MAP/Volt über BLE nach) und der
  WebGUI-Lambda-Testmodus (`source: TEST`, meldet immer sofort `OK`) — beides
  simuliert, durchläuft nicht den CAN-Decode-Pfad (`message.data[3]`). Das ist
  am Schreibtisch ohne echtes Spartan-Gerät auch gar nicht anders möglich.

## Kurz gesagt

- **`status`/`status_code` = 1:1 vom Spartan-Controller über CAN** (im
  Normalbetrieb) — der Hub erfindet nichts, wartet nur ab.
- **`WAIT`→`HEAT`→`OK` ist die Sonden-Aufwärmphase**, typischerweise wenige
  Sekunden bis unter einer halben Minute nach Zündung/Stromversorgung an.
- Bleibt es dauerhaft bei `WAIT`/`HEAT`/`ERR` und wird nie `OK`: das deutet auf
  ein Problem **am Spartan/an der Sonde selbst** hin (Heizung defekt,
  Verkabelung, Sonde kalt wegen Unterspannung), nicht auf einen Hub-Fehler —
  der Hub gibt nur weiter, was er über CAN empfängt.
