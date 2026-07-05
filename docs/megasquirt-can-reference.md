# MegaSquirt CAN Reference

Consolidated from the official msextra.com documentation. Relevant because the Spartan 3 v2 factory CAN format (`SETCANFORMAT0`) is the "MegaSquirt 3 ECU" format, and because a MegaSquirt/Microsquirt ECU is the natural future data source on the same 500 kbit/s bus this adapter already listens on.

Sources:

- Manuals index: <https://www.msextra.com/manuals/>
- Microsquirt Hardware Guide 3.4 (2016-01-19): <https://www.msextra.com/doc/pdf/html/Microsquirt_Hardware-3.4.pdf/Microsquirt_Hardware-3.4.html> (PDF: `Microsquirt_Hardware-3.4.pdf`)
- MegaSquirt CAN realtime data broadcast protocol (2016-02-17): <http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf>
- MegaSquirt 29-bit CAN protocol (2015-01-20): <http://www.msextra.com/doc/pdf/Megasquirt_29bit_CAN_Protocol-2015-01-20.pdf>
- A `.dbc` file for the broadcast format is available at <http://www.msextra.com/doc/pdf/>

## Protocol Overview

MegaSquirt uses two unrelated CAN mechanisms on the same bus:

| Protocol | Headers | Direction | Purpose |
| --- | --- | --- | --- |
| Realtime data broadcasting | 11-bit, 500 kbit/s, big-endian | ECU transmits, no handshake | Dashes, loggers, displays |
| 29-bit MegaSquirt protocol | 29-bit | Request/response, no broadcasting | Device-to-device data exchange, remote tuning passthrough |

Broadcasting requires MS2/Extra 3.4.x (MS2, Microsquirt, MSPNP2, Microsquirt-module) or MS3 1.4.x (MS3, MS3-Pro, MS3-Gold, MSPNP-Pro). The 29-bit doc applies to MS2/Extra 3.3.x / MS3 1.3.x and later. msextra strongly recommends receive-only devices (dashes, this adapter) use the 11-bit broadcast, not the 29-bit protocol.

## Simplified Dash Broadcasting (11-bit)

Reduced pre-defined data set intended for third-party dashes. 11-bit headers, 500 kbit/s, sequential IDs, big-endian. Default base identifier **1512 (0x5E8)**. In `Automatic` mode the ID is locked to 1512 and the rate is 50 Hz on MS3, 20 Hz on MS2. `Advanced` mode allows changing ID and rate.

Real value = raw value × Multiply ÷ Divide (no Add offsets in this set).

| CAN ID | Byte offset | Size | Signed | Name | Function | Units | Divide | MS2? |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1512 (0x5E8) | 0 | 2 | Y | `map` | Manifold air pressure | kPa | 10 | Y |
| 1512 | 2 | 2 | - | `rpm` | Engine RPM | RPM | 1 | Y |
| 1512 | 4 | 2 | Y | `clt` | Coolant temperature | deg F | 10 | Y |
| 1512 | 6 | 2 | Y | `tps` | Throttle position | % | 10 | Y |
| 1513 (0x5E9) | 0 | 2 | - | `pw1` | Main pulsewidth bank 1 | ms | 1000 | Y |
| 1513 | 2 | 2 | - | `pw2` | Main pulsewidth bank 2 | ms | 1000 | Y |
| 1513 | 4 | 2 | Y | `mat` | Manifold air temperature | deg F | 10 | Y |
| 1513 | 6 | 2 | Y | `adv_deg` | Final ignition spark advance | deg BTDC | 10 | Y |
| 1514 (0x5EA) | 0 | 1 | - | `afrtgt1` | Bank 1 AFR target | AFR | 10 | Y |
| 1514 | 1 | 1 | - | `AFR1` | AFR cyl#1 | AFR | 10 | (Y) |
| 1514 | 2 | 2 | Y | `EGOcor1` | EGO correction cyl#1 | % | 10 | (Y) |
| 1514 | 4 | 2 | Y | `egt1` | EGT 1 | deg F | 10 | - |
| 1514 | 6 | 2 | Y | `pwseq1` | Sequential pulsewidth cyl#1 | ms | 1000 | Y |
| 1515 (0x5EB) | 0 | 2 | Y | `batt` | Battery voltage | V | 10 | Y |
| 1515 | 2 | 2 | Y | `sensors1` | Generic sensor input 1 (gpioadc0 on MS2) | - | 10 | Y |
| 1515 | 4 | 2 | Y | `sensors2` | Generic sensor input 2 (gpioadc1 on MS2) | - | 10 | Y |
| 1515 | 6 | 1 | - | `knk_rtd` | Knock retard | deg | 10 | Y |
| 1516 (0x5EC) | 0 | 2 | - | `VSS1` | Vehicle speed 1 | m/s | 10 | - |
| 1516 | 2 | 2 | Y | `tc_retard` | Traction control retard | deg | 10 | - |
| 1516 | 4 | 2 | Y | `launch_timing` | Launch control timing | deg | 10 | - |
| 1516 | 6 | 2 | - | - | Not used | - | - | - |

Notes from the manual: EGT, VSS and knock may require additional hardware; MS2-based ECUs use a subset of these fields.

TunerStudio configuration: `CAN-Bus/Testmodes -> CAN Parameters -> Master enable = On/Enable`, then `CAN-Bus/Testmodes -> Dash Broadcasting -> Enable = On`.

## Advanced Real-Time Data Broadcast (11-bit)

Full internal data set. 11-bit headers, 500 kbit/s, big-endian. Default base identifier **1520 (0x5F0)**, user-selectable. Each enabled group of 8 bytes is transmitted at `base ID + group number` (group 17 → 1537/0x601). Groups are enabled individually with a per-group broadcast frequency; offsets never change. With base 1520 the groups 0–63 occupy IDs **1520–1583 (0x5F0–0x62F)**.

Real value = raw × Multiply ÷ Divide (Add is 0 for all fields). Detailed field list for the dash-relevant groups:

| Group (ID @ base 1520) | Offset | Size | Signed | Name | Function | Units | Mult/Div |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 (0x5F0) | 0 | 2 | - | `seconds` | Seconds ECU has been on | s | 1/1 |
| 0 | 2 | 2 | - | `pw1` | Main pulsewidth bank 1 | ms | 1/1000 |
| 0 | 4 | 2 | - | `pw2` | Main pulsewidth bank 2 | ms | 1/1000 |
| 0 | 6 | 2 | - | `rpm` | Engine RPM | RPM | 1/1 |
| 1 (0x5F1) | 0 | 2 | Y | `adv_deg` | Final ignition spark advance | deg BTDC | 1/10 |
| 1 | 2 | 1 | - | `squirt` | Bitfield of batch fire injector events | - | 1/1 |
| 1 | 3 | 1 | - | `engine` | Bitfield of engine status | - | 1/1 |
| 1 | 4 | 1 | - | `afrtgt1` | Bank 1 AFR target | AFR | 1/10 |
| 1 | 5 | 1 | - | `afrtgt2` | Bank 2 AFR target | AFR | 1/10 |
| 1 | 6-7 | 1+1 | - | `wbo2_en1/2` | Not used | - | - |
| 2 (0x5F2) | 0 | 2 | Y | `baro` | Barometric pressure | kPa | 1/10 |
| 2 | 2 | 2 | Y | `map` | Manifold air pressure | kPa | 1/10 |
| 2 | 4 | 2 | Y | `mat` | Manifold air temperature | deg F | 1/10 |
| 2 | 6 | 2 | Y | `clt` | Coolant temperature | deg F | 1/10 |
| 3 (0x5F3) | 0 | 2 | Y | `tps` | Throttle position | % | 1/10 |
| 3 | 2 | 2 | Y | `batt` | Battery voltage | V | 1/10 |
| 3 | 4 | 2 | Y | `afr1_old` | AFR1 (deprecated on MS3) | AFR | 1/10 |
| 3 | 6 | 2 | Y | `afr2_old` | AFR2 (deprecated on MS3) | AFR | 1/10 |
| 4 (0x5F4) | 0 | 2 | Y | `knock` | Indication of knock input | % | 1/10 |
| 4 | 2 | 2 | Y | `egocor1` | EGO bank 1 correction | % | 1/10 |
| 4 | 4 | 2 | Y | `egocor2` | EGO bank 2 correction | % | 1/10 |
| 4 | 6 | 2 | Y | `aircor` | Air density correction | % | 1/10 |
| 5 (0x5F5) | 0 | 2 | Y | `warmcor` | Warmup correction | % | 1/10 |
| 5 | 2 | 2 | Y | `tpsaccel` | TPS-based acceleration | % | 1/10 |
| 5 | 4 | 2 | Y | `tpsfuelcut` | TPS-based fuel cut | % | 1/10 |
| 5 | 6 | 2 | Y | `barocor` | Barometric fuel correction | % | 1/10 |
| 6 (0x5F6) | 0 | 2 | Y | `totalcor` | Total fuel correction | % | 1/10 |
| 6 | 2 | 2 | Y | `ve1` | VE value table/bank 1 | % | 1/10 |
| 6 | 4 | 2 | Y | `ve2` | VE value table/bank 2 | % | 1/10 |
| 6 | 6 | 2 | Y | `iacstep` | Stepper idle step number (step, 1/1) or PWM idle duty (%, 392/1000) | step or % | see fn |
| 7 (0x5F7) | 0 | 2 | Y | `cold_adv_deg` | Cold advance | deg | 1/10 |
| 7 | 2 | 2 | Y | `TPSdot` | Rate of change of TPS | %/s | 1/10 |
| 7 | 4 | 2 | Y | `MAPdot` | Rate of change of MAP | kPa/s | 1/1 |
| 7 | 6 | 2 | Y | `RPMdot` | Rate of change of RPM | RPM/s | 10/1 |
| 8 (0x5F8) | 0 | 2 | Y | `MAFload` | Synthetic 'load' from MAF | % | 1/10 |
| 8 | 2 | 2 | Y | `fuelload` | 'Load' for fuel table lookup (= MAP in speed-density) | % | 1/10 |
| 8 | 4 | 2 | Y | `fuelcor` | Adjustment to fuel from Flex | % | 1/10 |
| 8 | 6 | 2 | Y | `MAF` | Mass air flow (scaling range-dependent, 650 g/s shown) | g/s | 1/100 |
| 9 (0x5F9) | 0 | 2 | Y | `egoV1` | Voltage from O2#1 (deprecated on MS3) | V | 1/100 |
| 9 | 2 | 2 | Y | `egoV2` | Voltage from O2#2 (deprecated on MS3) | V | 1/100 |
| 9 | 4 | 2 | - | `dwell` | Main ignition dwell | ms | 1/10 |
| 9 | 6 | 2 | - | `dwell_trl` | Trailing ignition dwell | ms | 1/10 |
| 10 (0x5FA) | 0-3 | 1 each | - | `status1..status4` | ECU status bitfields (status4 not typically used) | - | 1/1 |
| 10 | 4 | 2 | Y | `status5` | Not typically used | - | 1/1 |
| 10 | 6-7 | 1+1 | - | `status6/status7` | ECU status bitfields (MS3 only) | - | 1/1 |
| 11 (0x5FB) | 0 | 2 | Y | `fuelload2` | 'Load' for fuelling on modified table | % | 1/10 |
| 11 | 2 | 2 | Y | `ignload` | 'Load' for ignition table lookup | % | 1/10 |
| 11 | 4 | 2 | Y | `ignload2` | 'Load' for modifier ignition table lookup | % | 1/10 |
| 11 | 6 | 2 | Y | `airtemp` | Estimated intake air temperature | deg F | 1/10 |
| 12 (0x5FC) | 0 / 4 | 4+4 | Y | `wallfuel1/2` | Calculated fuel volume on intake walls from EAE | us | 1/100 |
| 13 (0x5FD) | 0/2/4/6 | 2 each | Y | `sensors1..4` | Generic sensor inputs 1-4 (gpioadc0-3 on MS2) | - | 1/10 |
| 14 (0x5FE) | 0/2/4/6 | 2 each | Y | `sensors5..8` | Generic sensor inputs 5-8 (gpioadc4-7 on MS2) | - | 1/10 |
| 15 (0x5FF) | 0/2/4/6 | 2 each | Y | `sensors9..12` | Generic sensor inputs 9-12 (9/10 = adc6/7 on MS2; 11/12 MS3 only) | - | 1/10 |
| 16 (0x600) | 0/2/4/6 | 2 each | Y | `sensors13..16` | Generic sensor inputs 13-16 (MS3 only) | - | 1/10 |
| 17 (0x601) | 0 | 2 | Y | `boost_targ_1` | Target boost - channel 1 | kPa | 1/10 |
| 17 | 2 | 2 | Y | `boost_targ_2` | Target boost - channel 2 (MS3 only) | kPa | 1/10 |
| 17 | 4 | 1 | - | `boostduty` | Duty cycle on boost solenoid 1 | % | 1/1 |
| 17 | 5 | 1 | - | `boostduty2` | Duty cycle on boost solenoid 2 (MS3 only) | % | 1/1 |
| 17 | 6 | 2 | Y | `maf_volts` | MAF voltage (synthesised for frequency MAFs) | V | 1/1000 |

Higher groups, compact map (all big-endian; MS3-only unless noted):

| Group | IDs @ base 1520 | Contents | Scaling |
| --- | --- | --- | --- |
| 18-21 | 0x602-0x605 | `pwseq1..16` sequential pulsewidth per cylinder, s16 each (MS2: cyl 1-4 / group 18 only) | ms, 1/1000 |
| 22-25 | 0x606-0x609 | `egt1..16`, s16 each | deg F, 1/10 |
| 26 | 0x60A | `nitrous1_duty` u8 %, `nitrous2_duty` u8 %, `nitrous_timer_out` u16 s 1/1000, `n2o_addfuel` s16 ms 1/1000 (MS2: Y), `n2o_retard` s16 deg 1/10 (MS2: Y) | mixed |
| 27 | 0x60B | `canpwmin1..4` PWM periods from remote board, s16 each (MS2: Y) | 1/1 |
| 28 | 0x60C | `cl_idle_targ_rpm` u16 RPM, `tpsadc` s16 ADC counts, `eaeload` s16 % 1/10, `afrload` s16 % 1/10 (all MS2: Y) | mixed |
| 29 | 0x60D | `EAEfcor1/2` u16 % 1/10 (MS2: Y), `VSS1dot`/`VSS2dot` s16 m/s^2 1/10 | mixed |
| 30 | 0x60E | `accelx/y/z` external accelerometer s16 m/s^2 1/1000, `stream_level` u8 %, `water_duty` u8 % | mixed |
| 31-32 | 0x60F-0x610 | `AFR1..16`, u8 each | AFR, 1/10 |
| 33 | 0x611 | `duty_pwm1..6` u8 %, `gear` s8, `status8` u8 bitfield | 1/1 |
| 34-37 | 0x612-0x615 | `EGOv1..16` O2 voltages, s16 each | V, x489/100000 |
| 38-41 | 0x616-0x619 | `EGOcor1..16` per-cylinder EGO correction, s16 each | %, 1/10 |
| 42 | 0x61A | `VSS1..4` vehicle speeds, u16 each | m/s, 1/10 |
| 43 | 0x61B | `synccnt` u8 (MS2: Y), `syncreason` u8 (MS2: Y), `sd_filenum` u16, `sd_error` u8, `sd_phase` u8, `sd_status` u8, `timing_err` s8 % (MS2: Y) | 1/1 |
| 44-45 | 0x61C-0x61D | `vvt_ang1..4` / `vvt_target1..4`, s16 each | deg, 1/10 |
| 46 | 0x61E | `vvt_duty1..4` u8 x392/1000, `inj_timing_pri`/`inj_timing_sec` s16 deg BTDC 1/10 (MS2: Y) | mixed |
| 47 | 0x61F | `fuel_pct` s16 % 1/10 (Flex ethanol content, MS2: Y), `tps_accel` s16 % 1/10, `SS1`/`SS2` shaft speeds u16 RPM x10 | mixed |
| 48-49 | 0x620-0x621 | `knock_cyl1..16`, u8 each | %, x4/10 |
| 50 | 0x622 | `map_accel` s16 % 1/10, `total_accel` s16 % 1/10, `launch_timer` u16 s 1/1000, `launch_retard` s16 deg 1/10 | mixed |
| 51 | 0x623 | CPU port bitfields `porta`, `portb`, `porteh`, `portk`, `portmj`, `portp`, `portt` u8 each, `cel_errorcode` u8 | 1/1 |
| 52 | 0x624 | `canin1/canin2/canout` bitfields u8 (MS2: Y), `knk_rtd` u8 deg 1/10 (MS2: Y), `fuelflow` u16 cc/min, `fuelcons` u16 l/km | mixed |
| 53 | 0x625 | `fuel_press1/2` s16 kPa 1/10, `fuel_temp1/2` s16 deg F 1/10 | 1/10 |
| 54 | 0x626 | `batt_cur` s16 A 1/10, `cel_status` u16 bitfield, `fp_duty` u8 x392/1000, `alt_duty` u8 %, `load_duty` u8 %, `alt_targv` u8 V 1/10 | mixed |
| 55 | 0x627 | `looptime` u16 us (MS2: Y), `fueltemp_cor` u16 % 1/10, `fuelpress_cor` u16 % 1/10, `ltt_cor` s8 % 1/10, `sp1` unused | mixed |
| 56 | 0x628 | `tc_retard`, `cel_retard`, `fc_retard` s16 deg 1/10, `als_addfuel` s16 ms 1/1000 | mixed |
| 57 | 0x629 | `base_advance`, `idle_cor_advance`, `mat_retard`, `flex_advance` s16 deg 1/10 (all MS2: Y) | 1/10 |
| 58 | 0x62A | `adv1..adv4` timing lookups from tables 1-4, s16 deg 1/10 (adv1-3 MS2: Y) | 1/10 |
| 59 | 0x62B | `revlim_retard` s16 deg 1/10 (MS2: Y), `als_timing` s16 deg 1/10, `ext_advance` s16 deg 1/10 (MS2: Y), `deadtime1` s16 ms 1/1000 (MS2: Y) | mixed |
| 60 | 0x62C | `launch_timing`, `step3_timing`, `vsslaunch_retard` s16 deg 1/10, `cel_status2` u16 | mixed |
| 61-62 | 0x62D-0x62E | External GPS: lat/lon (deg s8/u8, min u8, milli-min u16), `gps_outstatus` u8 (bit 0 = E/W), altitude (km s8 + m u16), `gps_speed` u16 m/s 1/10, `gps_course` u16 deg 1/10 | mixed |
| 63 | 0x62F | `generic_pid_duty1/2` closed-loop duties, u8 x392/1000; bytes 2-7 unused | x392/1000 |

TunerStudio configuration: `CAN-Bus/Testmodes -> CAN Parameters -> Master enable = On/Enable`, then `CAN-Bus/Testmodes -> CAN Realtime Data Broadcasting -> Enable = On`, set base identifier if needed, and set a broadcast frequency per required group.

### DBW control messages (provisional)

Work-in-progress in MS3 1.5.x only, not MS2. 11-bit, 500 kbit/s, big-endian, default base 256 (0x100). MS3 listens for a pedal-position message (IDs 256-257 from DBW controller) and broadcasts throttle targets (IDs 260-261). The manual carries an explicit safety warning: the MS3 side is not safety-critical; an external safety-critical throttle controller is mandatory. Not relevant to this adapter — documented only so the IDs are known if such traffic appears on a shared bus.

## 29-bit MegaSquirt Protocol (Summary)

Proprietary request/response protocol between MegaSquirt devices (device-to-device data exchange and tuning passthrough). No broadcasting. The 29-bit identifier carries all addressing:

| Identifier bits | Field | Width |
| --- | --- | --- |
| 28-18 | `offset` — data offset within the table | 11 |
| 17-15 | `msg_type` | 3 |
| 14-11 | `from_id` — MegaSquirt CANid of sender | 4 |
| 10-7 | `to_id` — MegaSquirt CANid of destination | 4 |
| 6-3 | `table` bits 3-0 | 4 |
| 2 | `table` bit 4 | 1 |
| 1-0 | spare | 2 |

Note: table bit 4 is *not* contiguous with bits 3-0 — for tables 0-15 the identifier can be built as `(offset<<18) | (type<<15) | (from<<11) | (to<<7) | (table<<3)`.

MegaSquirt CANid well-known values: 0 = master ECU (always), 1 = GPIO transmission controller, 2 = GPIO board, 4 = JBPerf TinyIOx, 5 = JBPerf IO-x, 7 = Microsquirt transmission controller.

Message types: 0 `MSG_CMD` (write/poke data), 1 `MSG_REQ` (request data; 3 data bytes name the reply table/offset/count), 2 `MSG_RSP` (reply, same format as CMD), 4 `MSG_BURN` (burn table to flash), 5/6 `OUTMSG_REQ/RSP` (pre-configured data groups), 7 `MSG_XTND` (extended; real type in data byte 0: 8 `MSG_FWD`, 9 `MSG_CRC`, 12 `MSG_REQX` for tables >31, 14 `MSG_BURNACK`, 0x80 `MSG_PROT`, 0x82 `MSG_SPND`).

Realtime data lives in table 7 (`outpc`); e.g. RPM is table 7 offset 6 on the master. Worked example from the spec — CANid 7 fetching RPM from CANid 0: request `id=0x18B838, DLC=3, data 07 00 42`, reply `id=0x903B8, DLC=2, data 14 FE` (5374 RPM).

The spec's own advice for third-party receive-only devices: prefer the 11-bit broadcast. This adapter follows that advice — 29-bit (extended) frames are ignored in `updateCan()` via the `message.extd` filter.

## Microsquirt Hardware CAN Notes

From the Microsquirt Hardware Guide 3.4 (V3 hardware, MS2/Extra 3.4.x):

- 35-way connector pin 2 = **CANH** (Blue/Yellow wire), pin 3 = **CANL** (Blue/Red wire).
- CAN forms a bus with a **120 R terminator at each end**; devices hang as short drops off the bus.
- The Microsquirt has an **internal terminating resistor** — no additional resistor needed at its end of the bus.
- CANH/L are dedicated pins ("Always CANH/L signals"), used for add-on units such as transmission control, CANEGT interfaces, data capture or compatible dashboards.

## Relevance To This Adapter

Known 11-bit IDs if this adapter ever shares a bus with a MegaSquirt ECU (all at 500 kbit/s, matching our TWAI config):

| 11-bit ID | Transmitter | Content |
| --- | --- | --- |
| 0x100-0x105 (256-261) | DBW controller / MS3 | Provisional drive-by-wire messages |
| 0x400 (1024) | Spartan 3 v2 (`SETCANFORMAT0`, factory default) | Lambda frame this firmware decodes (`SPARTAN_CAN_ID`) |
| 0x510 (1296) | This hub | Cockpit frame TX (`COCKPIT_CAN_ID`) — no collision with MS IDs |
| 0x5E8-0x5EC (1512-1516) | MS ECU | Simplified dash broadcast |
| 0x5F0-0x62F (1520-1583) | MS ECU | Advanced realtime broadcast, groups 0-63 |

- The Spartan's "MegaSquirt 3 ECU" format at 0x400 is a frame the *ECU consumes* (CAN EGO input); it is not part of the ECU's own broadcast. Both can coexist on one bus.
- RPM, MAP, CLT, TPS, advance and battery voltage — currently sourced from 123TUNE+ BLE — are all available from the dash broadcast (0x5E8-0x5EB) if a MegaSquirt ECU is fitted, at 20-50 Hz with no pairing.
- All MS broadcast data is big-endian, same as the Spartan frame; temperatures are **deg F x10** (`degC = (raw/10 - 32) / 1.8`), AFR fields are **AFR x10** (`lambda = raw / 10 / 14.7` for petrol).
- 29-bit request/response traffic between MS devices is expected background noise on such a bus; the existing `message.extd` filter drops it correctly.
