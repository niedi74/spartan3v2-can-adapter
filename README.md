# Spartan 3 v2 Motorraum Display

> **Entwicklungsstand:** siehe [Releases](https://github.com/niedi74/spartan3v2-can-adapter/releases) · Branch: **`main`**

Display firmware for a dedicated ESP32 Dev adapter board connected to a 14Point7 Spartan 3 v2. The primary measurement input is CAN. UART is available for Spartan configuration, and the analog output is prepared as an optional fallback.

## Hardware targets

### Dedicated ESP32 Dev adapter

- Port: `COM16` (default in `platformio.ini`)
- USB bridge: Silicon Labs CP210x USB to UART Bridge
- Chip: ESP32-D0WD-V3, revision 3.1
- MAC: `6C:C8:40:06:73:58`
- Display: 16x2 I2C LCD, address `0x27`
- Spartan input: CAN through an external 3.3 V CAN transceiver
- Optional interfaces: protected UART and scaled analog input
- Current bring-up mode: simulated Spartan readings until the CAN module arrives
- Setup Web GUI: ESP32 access point `Spartan3-Setup`, password `lambda123`
- Road hotspot default: `Android-AP1` / `REDACTED` (2.4 GHz)
- Cockpit link: **HTTP poll** — cockpit displays/apps poll `GET /api/status` (JSON) over WiFi (ESP-NOW cockpit link was removed)
- BLE: 123TUNE+ central + optional BM6; setup name `Spartan3-Hub` for diagnostics only

### M5

The `m5_motorraum` environment uses `M5Unified` and currently targets `m5stack-core-esp32`. Adjust the PlatformIO board if the actual M5 model differs.

## GPIO Pinbelegung (Uebersicht)

| GPIO | Funktion | Richtung | Schutz |
| --- | --- | --- | --- |
| 2 | Status-LED | OUT | - |
| 16 | Spartan UART RX (Serial2) | IN | Spannungsteiler 10k/18k (5V->3.3V) |
| 17 | Spartan UART TX (Serial2) | OUT | direkt |
| 21 | LCD SDA (I2C) | I/O | - |
| 22 | LCD SCL (I2C) | OUT | - |
| 25 | CAN RX (TWAI, via SN65HVD230) | IN | Transceiver |
| 26 | CAN TX (TWAI, via SN65HVD230) | OUT | Transceiver |
| 27 | Reed Speed-Sensor | IN (PULLUP) | gegen GND |
| 34 | Spartan Analog (optional) | ADC | Spannungsteiler |
| 35 | Heater Status Analog (optional) | ADC | - |

## LCD wiring

| LCD | ESP32 Dev |
| --- | --- |
| VCC | 5V for usable contrast on this module |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Many 16x2 I2C backpacks use address `0x27`; some use `0x3F`.

## Spartan CAN wiring

The ESP32 contains a TWAI/CAN controller but requires a physical CAN transceiver. For the first adapter board, use a 3.3 V transceiver module such as an SN65HVD230 breakout.

| ESP32 Dev | CAN transceiver | Spartan 3 v2 |
| --- | --- | --- |
| 3V3 | VCC | - |
| GND | GND | Black electronics ground |
| GPIO 26 | TXD / D | - |
| GPIO 25 | RXD / R | - |
| - | CANH | Blue CAN High |
| - | CANL | Purple CAN Low |

Keep Spartan electronics ground and the ESP32/transceiver ground connected. Power the Spartan according to its manual from switched 12 V through the provided 5 A fuse; do not power it from the ESP32.

The firmware expects the Spartan factory CAN measurement format:

| Setting | Value |
| --- | --- |
| Baud rate | 500 kbit/s |
| 11-bit CAN ID | `0x400` (`1024`) |
| Payload | 4 bytes, big-endian |
| Data rate | 50 Hz |

| CAN byte | Measurement |
| --- | --- |
| `0..1` | Lambda x 1000 |
| `2` | Sensor temperature / 10 |
| `3` | Status: `1` waiting, `2` heating, `3` normal |

## Spartan UART via ESP32

Spartan UART is intended here for setup commands such as `GETFW`, `GETCANID`, or `GETCANBAUD`; runtime Lambda data comes from CAN. The adapter exposes Spartan UART through the ESP32: the Web GUI sends ASCII commands over ESP32 `Serial2`, and the ESP32 USB serial monitor can do the same with commands prefixed by `>`.

| ESP32 Dev | Protected interface | Spartan 3 v2 |
| --- | --- | --- |
| GPIO 16 RX | Level shifter or divider | Orange UART TX |
| GPIO 17 TX | Level shifter | Yellow UART RX |
| GND | Common ground | Grey UART Ground |

The Spartan UART side must not feed a 5 V signal directly into an ESP32 GPIO. Use a bidirectional level shifter or at least a safe divider on Spartan TX to ESP32 RX. UART runs at `9600 8N1`.

The supplied Spartan USB converter is still useful for bench testing, but the normal adapter path is:

```text
Browser / ESP32 USB monitor -> ESP32 -> level shifter -> Spartan UART
```

This means the adapter needs only the ESP32 USB connection for setup from a laptop. Spartan's Orange/Yellow/Grey UART wires can be brought directly to a small protected header on the ESP32 adapter board:

| Adapter header | ESP32 Dev | Purpose |
| --- | --- | --- |
| Spartan UART TX in | GPIO 16 RX | Receives Spartan replies through level shift/divider |
| Spartan UART RX out | GPIO 17 TX | Sends commands to Spartan through level shift |
| UART GND | GND | Common ground |

Do not use ESP32 `GPIO 1/3` for Spartan UART on this board; those are normally tied to the ESP32 USB serial/programming port. `Serial2` on `GPIO 16/17` keeps programming/debug USB and Spartan configuration separate.

## Speed Reed Input

The planned speed pickup uses one reed contact against ground on `GPIO 27`, with the ESP32 internal pull-up enabled. The current magnet ring target is:

| Setting | Value |
| --- | --- |
| Reed input | `GPIO 27` to GND |
| Pulses per wheel revolution | `10` |
| Tire | `205/80 R14` |
| Calculated circumference | `2147 mm` |
| Start calibration | `4657 pulses/km` |

The tire value is a geometric starting point. Final speed should be calibrated from GPS or a measured road distance because real rolling circumference changes with load, pressure and tire model.

## BM6 Battery Monitor

The motorraum ESP can also connect to the Leagend BM6 V2.0 battery monitor over BLE. The live-tested target address is `3C:AB:72:80:06:6A`; `/state` exposes `bm6_connected`, `bm6_voltage`, `bm6_temperature`, `bm6_rx_count`, `bm6_decode_fail` and `bm6_age_ms`.

## BLE gateway mode

The `motorraum` build also starts a NimBLE GATT peripheral named `Spartan3-Hub`. This is the first step toward using the Spartan ESP as the central engine-bay gateway:

```text
123TUNE+ (BLE)     ─┐
Spartan 3 v2 (CAN) ─┤  Spartan ESP32   ──BLE──>  M5 Dial (Gateway)
BM6 Battery (BLE)  ─┤  "Spartan3-Hub"            8 Display-Pages
Reed Speed (GPIO27)─┘  Web GUI (Live|Setup)
```

The firmware publishes a compact text payload via BLE Notify every 250 ms:

```
L<lambda>R<rpm>A<adv>M<map>V<bm6_volt>S<speed_kmh>I<123_volt>T<123_temp>C<coil_A>
```

Example: `L0.98R850A14M100V12.38S65I12.8T45C3.2`

| Field | Source | Description |
| --- | --- | --- |
| `L` | Spartan CAN | Lambda (0.68..3.40) |
| `R` | 123TUNE+ BLE | RPM |
| `A` | 123TUNE+ BLE | Advance (degrees) |
| `M` | 123TUNE+ BLE | MAP (kPa) |
| `V` | BM6 BLE | Battery voltage |
| `S` | Reed GPIO 27 | Speed (km/h, rounded) |
| `I` | 123TUNE+ BLE | Internal voltage |
| `T` | 123TUNE+ BLE | Internal temperature |
| `C` | 123TUNE+ BLE | Coil current (A) |

Fields `V`, `S`, `I`, `T`, `C` are optional — omitted when the source is not connected. The M5 parser handles any subset.

### BLE UUIDs

| UUID | Type |
| --- | --- |
| `7f510001-5a6b-4d2a-9f20-14a7f3e20000` | Service |
| `7f510002-5a6b-4d2a-9f20-14a7f3e20000` | Status Notify (compact payload) |
| `7f510003-5a6b-4d2a-9f20-14a7f3e20000` | Command Write |

See [docs/ble-gateway-architecture.md](docs/ble-gateway-architecture.md) for the full architecture.

## Optional analog fallback

The green Spartan high-performance analog output is `0-5 V` and cannot be connected directly to an ESP32 ADC input. A future fallback input is allocated on `GPIO 34`; enable it with `-D ENABLE_SPARTAN_ANALOG=1` only after installing a divider. The current calculation expects:

| Connection | Value |
| --- | --- |
| Spartan green output to GPIO 34 | `10 kOhm` series/high-side resistor |
| GPIO 34 to ground | `20 kOhm` low-side resistor |

At factory output settings, `0 V` represents Lambda `0.68` and `5 V` represents Lambda `1.36`. CAN remains preferred because it avoids ADC and ground-offset measurement error.

## Commands

```powershell
pio run -e motorraum
pio run -e motorraum -t upload
pio run -e spartan_uart_test -t upload
pio run -e m5_motorraum
```

## Spartan UART Test Wiring

Use the `spartan_uart_test` environment when connecting the Spartan 3 v2 UART wires directly to the ESP32 DevKit.

```text
Spartan Orange TX -> 10k -> ESP32 GPIO16 RX2
                         |
                        18k
                         |
                        GND

Spartan Yellow RX -> ESP32 GPIO17 TX2
Spartan Grey GND  -> ESP32 GND
```

Orange is a 5V UART signal. Do not connect it directly to an ESP32 GPIO.

## Bring-up / Demo mode

`ENABLE_SPARTAN_DEMO=0` is the default (real CAN data). Set `=1` in `platformio.ini` for bench testing without a sensor. Demo cycles Lambda values around `1.000` and marks them as `DEMO`.

```text
LAM 1.023 DEMO
780C OK
```

CAN reception remains compiled in. A valid Spartan CAN message takes priority over demo data. When the CAN module is installed for normal operation, set `ENABLE_SPARTAN_DEMO=0` in `platformio.ini` so a missing CAN connection is shown as an error rather than concealed by test values.

## Web GUI

Two-tab layout: **Live** and **Setup**.

| Setting | Value |
| --- | --- |
| WiFi AP | `Spartan3-Setup` / `lambda123` |
| AP address | `http://192.168.4.1/` |
| Road hotspot | `Android-AP1` / `REDACTED` |

## Time master

The hub carries a battery-backed **DS3231 RTC** (I2C, GPIO4=SDA/GPIO5=SCL) as its own always-available time source — no phone/NTP needed on the road:

1. On boot, the hub reads the DS3231 and sets its system clock (UTC) immediately, independent of WiFi.
2. If NTP becomes available (hub joins a WiFi network with internet), it resyncs and writes the fresh time back into the DS3231.
3. Cockpit displays/apps poll `GET /api/status` and copy `time_epoch` whenever `time_valid:true` — the hub is always the time master, the RTC just makes that available without needing a phone hotspot.
4. Fallback: a cockpit display with its own battery-backed RTC can push a bootstrap time via `POST /api/rtc/set` (`epoch=<unix seconds>`) if the hub's own RTC is ever missing/uninitialized.

`/api/status` fields: `time_valid`, `time_epoch` (Unix seconds), `time_text`/`ntp_time` (local string), `ntp_synced` (true once NTP has confirmed the time at least once), `rtc_present`, `rtc_valid`, `timezone`.

JSON endpoint: `/state` (also `/api/status`)

### Live tab

- Lambda hero card (value, source, status, sensor temp)
- 123 RPM / ADV / MAP
- 60 s charts (Lambda, RPM, Temp)
- 123Tune BLE Diagnose (connection, RX frames, scan stats)
- BM6 Batteriemonitor (voltage, temperature, RX count)
- Geschwindigkeit Reed (Hz, km/h, pulses, GPS trim form)

### Setup tab

- System Diagnose (CAN state, Heap, AP IP, JSON dump)
- BLE Hub / Gateway (UUIDs, client count, road-AP hints)
- WLAN / Hotspot (presets, manual entry, clear)
- Spartan UART (GETFW, GETCANID, expert commands, CAN/Output config)

## Spartan Reference

Local Spartan 3 v2 documentation from `D:\_claude\M5stack\spartan` has been condensed into [docs/spartan-3-v2-reference.md](docs/spartan-3-v2-reference.md).

With valid CAN data, the LCD shows:

```text
LAM 1.000 CAN
780C OK
```

Without CAN messages, it stays on `warte Daten...`. If analog fallback is deliberately enabled and CAN data is absent, it shows `ADC` instead of `CAN`.

## Official M5Dial documentation

- M5Dial main documentation: https://docs.m5stack.com/en/core/M5Dial
- M5Dial Arduino quick start: https://docs.m5stack.com/en/arduino/m5dial/program
- M5Stack docs home: https://docs.m5stack.com/
- M5Dial product page (official store): https://shop.m5stack.com/products/m5dial

License note: official third-party docs/files should be linked, not copied into this repository, unless their license explicitly allows redistribution.
