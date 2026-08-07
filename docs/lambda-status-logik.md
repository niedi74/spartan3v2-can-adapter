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
Byte 7 (flags):  Bit0=LambdaValid  Bit1=TuneFresh  Bits2-3=status_code  Bit4=RealCan  Bits5-7=reserviert
```

Display-seitig zum Auswerten:
```
status_code = (flags >> 2) & 0x03;   // 0=ERR 1=WAIT 2=HEAT 3=OK
bereit      = status_code == 3;      // entspricht dem HTTP-Feld status=="OK"
```

## CAN-Empfangs-Frame 0x513 — Live-Zündwinkel per CAN bedienen (Display → Hub)

**Neu (2026-07-29):** Damit das Display die 123-Live-Zündwinkel-Steuerung auch
ohne WLAN bedienen kann, nimmt der Hub jetzt zusätzlich zum bestehenden
`/api/tune/live` (HTTP) auch ein CAN-Frame entgegen — **derselbe, bereits
abgesicherte Codepfad**, keine neue Sicherheitslogik: Dead-Man-Timeout 60s
ohne Kommando, nur Einzelschritte (kein Sprung auf beliebigen Wert), nur
wirksam bei aktiv streamender 123.

```
ID: Cockpit-ID + 3 (Default 0x513)
Byte 0: Kommando -- 0=Ping(Dead-Man), 1=Schritt hoch, 2=Schritt runter,
        3=Reset auf 0, 4=Modus umschalten (Live-Tuning an/aus)
```

Display-seitig: einfach das passende Byte senden, kein Antwort-Frame nötig.
Ohne Kommando >60s fällt der Live-Modus automatisch ab (Dead-Man), genau wie
beim HTTP-Weg.

**Ping-Intervall:** Dead-Man-Timeout ist exakt 60.000 ms ohne jedes Kommando
(Ping zählt genauso wie ein echter Schritt). Empfehlung: alle 10–15s ein
Ping (Byte0=0) senden, solange der Live-Modus aktiv bleiben soll.

**Bestätigung (seit 2026-07-29, Bits neu belegt):**
- **0x510 Byte 7, Bit 5** (`kCockpitFlagTuneModeActive`): 1 = Live-Tuning-Modus
  ist gerade aktiv. Direkte Antwort auf Kommando 4 (Modus umschalten).
- **0x511 Byte 7** (`tune_adv_steps`, int8, vorher reserviert/0): aktuell
  kommandierte Zündwinkel-Schritte relativ zum Basiswert (0 = kein Offset).
  Direkte Antwort auf Kommando 1/2/3 (hoch/runter/reset).

```js
const tuneModeActive = (flags0x510 & 0x20) !== 0;   // aus 0x510 Byte 7
const tuneAdvSteps = int8FromByte(data0x511[7]);     // aus 0x511 Byte 7
```

Zusätzlich weiterhin indirekt: der echte `advance`-Wert in 0x510 (Byte 4-5)
spiegelt die tatsächlich von der 123 gemeldete Zündeinstellung wider, mit
BLE-Rundlaufzeit (~100-300ms) — die expliziten Bits oben sind aber die
verlässlichere, sofortige Bestätigung "Kommando angekommen".

## ⚠️ Sicherheitsbug gefunden + gefixt (2026-07-14): Demo/Test-Daten nicht von echten unterscheidbar

**Vorfall:** Am 2026-07-14 fiel während der Fahrt CAN zum Spartan zeitweise
aus. Der Hub fiel automatisch auf den DEMO-Modus zurück (simulierte, feste
Werte) — aber **weder das Display noch der CAN-Cockpit-Frame zeigten das
an**. Der Fahrer hielt die simulierten Werte für echte Messwerte und **hat
danach den Vergaser real verstellt** — auf Basis von Fake-Daten. Kein
Personenschaden, aber ein reales Fehlbedienungsrisiko.

**Root Cause:** `flags & 0x01` (Bit0, `LambdaValid`) sagt nur "irgendein
Lambda-Wert kam an" — das ist bei DEMO/TEST/ADC-Fallback genauso `true` wie
bei echtem CAN (`SpartanReading.valid` wird in **jedem** Lesepfad gesetzt,
nicht nur bei echten CAN-Frames). Über HTTP existierte das `source`-Feld
(`"CAN"`/`"DEMO"`/`"TEST"`/`"ADC"`/`"NONE"`) bereits lange und macht das
korrekt sichtbar — **aber der CAN-Cockpit-Frame hatte kein äquivalentes Bit**,
und offenbar wertete auch das Display das vorhandene `source`-Feld nicht
sichtbar genug (oder gar nicht) aus.

**Fix:** neues **Bit 4** (`kCockpitFlagRealCan`) im `flags`-Byte — `1` **nur**
wenn der Wert wirklich aus einem echten Spartan-CAN-Frame stammt
(`SpartanReading.fromCan`), `0` bei DEMO/TEST/ADC-Fallback:

```
Byte 7 (flags):  ... Bit4=RealCan (1=echtes CAN, 0=SIMULIERT/Ersatzwert -- NICHT vertrauen!)
```

**Display-seitig zwingend zu prüfen (beide Wege, je nachdem was das Display nutzt):**
```
// Über HTTP /api/status:
istEcht = (source === "CAN");

// Über CAN-Cockpit-Frame 0x510, Byte 7:
istEcht = (flags & 0x10) !== 0;
```

**Wenn `istEcht == false`: das Display MUSS das deutlich sichtbar machen**
(z. B. "SIMULIERT"/"DEMO" statt der Lambda-Zahl, oder ein auffälliges
Warnsymbol) — niemals stillschweigend Zahlen anzeigen, die nicht von der
echten Sonde kommen. Genau das Fehlen dieser Anzeige hat den Vorfall
verursacht.

Komplettes Byte-Layout: siehe Kommentar am Kopf von
[`include/hub_can.h`](../include/hub_can.h).

## Ext-Cockpit-Frame (ID = Cockpit-ID+1, z.B. 0x511) — 123-Volt/Temp/Coil + Speed über CAN

**Neu (2026-07-19):** Manche Einbauorte haben nur schwaches Hub-WLAN zum Display
(Metallblech im Weg, Abstand). Bisher gingen 123-Spannung/-Temperatur/-Spulenstrom
und Geschwindigkeit NUR per HTTP `/api/status` raus — bei schwachem WLAN fehlten
diese Werte am Display, obwohl CAN robust lief. Jetzt gibt es dafür ein zweites
CAN-Frame, gesendet mit derselben ~10-Hz-Rate wie 0x510, ID = Cockpit-ID + 1
(Default also **0x511**):

```
Byte 0-1: tune_volt_x100  (uint16, big-endian, 0 = 123 nicht verbunden/nicht frisch)
Byte 2:   tune_temp_c     (int8)
Byte 3:   tune_coil_x10   (uint8)
Byte 4-5: speed_kmh_x10   (uint16, big-endian)
Byte 6:   flags: Bit0 = TuneFresh (123-Daten <3s alt)
Byte 7:   reserviert/0
```

Display-seitig:
```
volt  = (data[0]<<8 | data[1]) / 100.0
temp  = (int8_t)data[2]
coil  = data[3] / 10.0
speed = (data[4]<<8 | data[5]) / 10.0
tuneFresh = (data[6] & 0x01) != 0
```

Wenn `tuneFresh == false`: Volt/Temp/Coil sind 0/veraltet — genauso behandeln wie
`tune_link_state != "streaming"` über HTTP (nicht als aktuelle Werte anzeigen).

## Ext2-Cockpit-Frame (ID = Cockpit-ID+2, z.B. 0x512) — Odo/Trip/Motorstunden

**Neu (2026-07-19):** Das Display ist das Fahrt-Frontend — Kilometerstand,
Trip und Motorstunden sollen bei schwachem WLAN nicht fehlen. Uhrzeit und
Live-Tuning-Schreibzugriff bleiben bewusst HTTP-only (Uhrzeit unkritisch,
Tuning-Schreiben braucht ohnehin einen Rückkanal, kein reines Sende-Frame).

```
Byte 0-3: odo_km_x10    (uint32, big-endian, Gesamtstrecke, Lebensdauer)
Byte 4-5: trip_km_x10   (uint16, Teilstrecke)
Byte 6-7: engine_hours_x10 (uint16, Motorstunden)
```

Display-seitig:
```
odo   = (data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3]) / 10.0
trip  = (data[4]<<8 | data[5]) / 10.0
hours = (data[6]<<8 | data[7]) / 10.0
```

Geräte-/Sensorstunden (nur Wartungsinfo, nicht Fahrt-relevant) bleiben HTTP-only.

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
