# VW T2 Typ4 Einspritzanlage – Teile-Analyse und Umbaukonzept

## Konzept

Der VW T2 Typ4 (2.0L Boxer, luftgekühlt) trägt eine originale Bosch L-Jetronic Einspritzanlage.
Ziel des Umbaus: **Komplette moderne EFI-Technik einbauen, aber die originale Optik vollständig
erhalten.** Die alten Gehäuse und Verkleidungen bleiben — neue Sensorik, neue Steuerung,
neues Leben dahinter.

```
Original-Optik (Hüllen/Gehäuse)
       +
Moderne EFI-Technik (Sensorik, ECU, Lambda, Zündung)
       =
Restomod-Einspritzung mit authentischem Look
```

---

## Originale Komponenten der L-Jetronic (Typ4)

### Luftführung und Messung

| Bauteil | Funktion original | Zustand / Verwendung |
|---|---|---|
| **Bosch LMM** (0 280 200 xxx) | Luftmengenmessung per Klappe + Poti | Gehäuse behalten → moderner Sensor rein |
| Luftfilterkasten | Luftansaugung | Behalten oder optisch angepasst |
| Ansaugschläuche | Luftführung | Behalten, ggf. erneuern |

### Kraftstoff

| Bauteil | Funktion original | Verwendung |
|---|---|---|
| Kraftstoffverteiler (Fuel Rail) | Kraftstoffverteilung zu Injektoren | Behalten als Hülle / optisches Element |
| Injektoren (Bosch) | Einspritzung | Ersetzen durch moderne low-Z Injektoren |
| Druckregler | Kraftstoffdruck ~2.5 bar | Ersetzen durch modernen Druckregler |
| Kraftstoffpumpe | Förderpumpe | Ersetzen durch moderne Inline-Pumpe |

### Elektronik und Steuerung

| Bauteil | Funktion original | Verwendung |
|---|---|---|
| Bosch L-Jetronic ECU | Steuergerät | **Komplett ersetzen** durch moderne ECU |
| Kabelbaum | Verdrahtung | Neu aufbauen |
| Kaltstart-Einspritzventil | Kaltstart-Anreicherung | Entfällt mit moderner ECU |
| Thermozeitschalter | Kaltstartsteuerung | Entfällt |
| Druckfühler | Saugrohrdruck (D-Jetronic Anteil) | Entfällt / durch MAP-Sensor ersetzt |

### Sensorik

| Sensor original | Ersatz modern |
|---|---|
| LMM-Poti (Luftmenge) | MAP-Sensor + optionaler MAF im LMM-Gehäuse |
| LMM-NTC (Ansauglufttemperatur) | Moderner NTC IAT-Sensor |
| Wassertemperaturfühler | Moderner NTC CLT-Sensor |
| Drosselklappenschalter | TPS (Drosselklappenpositionssensor) |
| Kurbelwellenposition | Zahnrad + Hall-Sensor oder VR-Sensor |
| Lambda (originale Breitband fehlt) | **Spartan 3 v2 Wideband** ✓ bereits vorhanden |

---

## Bosch LMM – Detailanalyse

**Gehäuse:** Stabiles Aluminium-Druckguss mit Rippenstruktur — sehr gut als Hülle geeignet.

**6-Pin-Stecker:**

| Pin | Funktion original | Neu belegen |
|---|---|---|
| 1 | Masse (GND) | GND |
| 2 | +12V Referenz | +5V oder +12V je nach neuem Sensor |
| 3 | Luftmengen-Ausgang (0–5V Poti) | Analogausgang MAP oder MAF |
| 4 | Kraftstoffpumpe Schaltausgang | Freier Ausgang oder ungenutzt |
| 5 | IAT-Sensor NTC + | IAT-Sensor (neuer NTC) |
| 6 | IAT-Sensor NTC – | IAT-Sensor Rückleiter |

> Belegung vor Umbau am Originalstecker mit Multimeter verifizieren — Varianten möglich.

**Empfehlung für Einbau in LMM-Gehäuse:**

- **Option A (einfach):** MAP-Sensor (z.B. GM 3-bar, ~15€) + neuer NTC IAT-Sensor
  - Kein Umbau der Luftführung nötig
  - MAP-Sensor misst Saugrohrdruck, kein Durchströmungserfordernis
  - Sofort machbar

- **Option B (präziser):** Bosch HFM5/HFM6 Hot-Film MAF (aus Golf IV / A4, ~15–25€ gebraucht)
  - Echter Luftmassenwert wie beim Original, nur digital und präzise
  - Braucht 3D-gedruckten Einbauadapter im LMM-Gehäuse
  - Ausgangssignal 0–5V oder Frequenz, bekannte Kennlinie

---

## Zündung

Die 123TUNE+ ersetzt den originalen Zündverteiler und liefert:
- Programmierbare Zündkennfelder
- BLE-Ausgabe (RPM, Zündwinkel, MAP) → bereits in Spartan ESP integriert

---

## Gesamtsystem Übersicht (Zielzustand)

```
Ansaugluft
    │
    ▼
[LMM-Gehäuse] ──── MAP/MAF-Sensor ──────────────────────────────────┐
    │                                                                 │
    ▼                                                                 ▼
[Drosselklappe] ── TPS ────────────────────────────────────► [Moderne ECU]
    │                                                         (z.B. MegaSquirt,
    ▼                                                          SpeedUino, Haltech)
[Einlassverteilung]                                                   │
    │                                                                 │
    ▼                                                         [Injektoren]
[Typ4 Motor]                                                  [Zündung via 123TUNE+]
    │
    ▼
[Abgasanlage] ── [Spartan 3 v2 Wideband Lambda]
                        │
                        ▼ CAN (500 kbit/s, ID 0x400)
                 [Spartan ESP (dieser Adapter)]
                        │
                        ├── LCD: Lambda + 123TUNE-Daten
                        ├── Web-Dashboard
                        └── BLE → M5/Waveshare Cockpit-Display
```

---

## Empfohlene moderne ECU-Kandidaten

| ECU | Vorteil | Preis |
|---|---|---|
| **SpeedUino** | Open Source, günstig, große Community, DIY-freundlich | ~100–150€ |
| **MegaSquirt MS1/MS2** | Bewährt, viel Doku, gut für Sauger | ~150–300€ |
| **Haltech Nexus R3** | Professionell, Plug-and-Play-Kabel verfügbar | ~600€+ |
| **Adaptronic** | Kompakt, modern | ~400€+ |

Für den VW T2 Typ4 und DIY-Ansatz: **SpeedUino** oder **MegaSquirt MS2** empfohlen.

---

## Offene Punkte (Hardware)

- [ ] Innendurchmesser LMM-Luftstutzen messen (für Adapter-Auslegung HFM5)
- [ ] Steckerbelegung LMM am echten Bauteil mit Multimeter messen
- [ ] Entscheidung: MAP-Sensor (sofort) oder HFM5 (präziser, mehr Aufwand)
- [ ] Wahl der modernen ECU
- [ ] Injektoren-Impedanz prüfen (Original L-Jetronic = high-Z ~16Ω, moderne low-Z brauchen Widerstandsbox)
- [ ] Kraftstoffdruck: Original ~2.5 bar, moderne ECU-Anforderung prüfen

## Offene Punkte (Software / Spartan ESP)

- [ ] Analog-Eingang GPIO 34 für MAP/MAF-Signal aktivieren (ENABLE_SPARTAN_ANALOG)
- [ ] Zweiter ADC-Kanal GPIO 35 für IAT-Sensor (falls gewünscht)
- [ ] IAT/MAP-Werte ins BLE-Payload und Web-Dashboard aufnehmen
- [ ] Daten an ECU weitergeben (falls CAN-Bus der ECU genutzt wird)
