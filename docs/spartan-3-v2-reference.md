# Spartan 3 v2 Reference

Local source folder scanned: `D:\_claude\M5stack\spartan`

Primary source for this adapter:

- `Spartan_3_v2_User_Manual_0364360d-3d7e-4f94-96f7-cd243c5a24de.pdf`
- Manual metadata: `Spartan 3 v2 User Manual June 27 2025`

Related but not identical sources:

- `SPartan_3_OEM_Manual_d659f67e-71fa-4680-b3a3-7c1220668c07.pdf`
- `Spartan2_OEM_Manual_...` files and examples
- `14Point7_MS_Linear_Output_Compensation.xls`

## Safety And Installation

- Do not connect or disconnect the lambda sensor while Spartan 3 is powered.
- Sensor gets very hot in normal operation.
- Do not power the unit before the engine is running; condensation can thermally shock a heated sensor.
- In an active exhaust stream, the sensor must be controlled by Spartan 3. An unpowered sensor in active exhaust can foul from carbon.
- Leaded fuel sensor life is listed as roughly 100 to 500 hours.
- Install the sensor between 10 o'clock and 2 o'clock, less than 60 degrees from vertical.
- Install before the catalytic converter.
- Normally aspirated engines: about 2 ft from exhaust port.
- Turbocharged engines: after the turbocharger.
- Supercharged engines: about 3 ft from exhaust port.

## Power

- Red wire: switched 12 V through the supplied 5 A fuse holder.
- Recommended switched source: fuel pump relay.
- 12 V should be live only when the engine is running.
- Spartan 3 cannot be powered through USB.
- Black wire: electronics ground, connect where the interfacing device is grounded.
- White wire: heater ground, connect to chassis or engine block.
- If powered without heater ground connected, Spartan 3 enters bootloader mode and normal operation will not start.

## Wiring

| Wire | Function | Notes |
| --- | --- | --- |
| Red | Power | Switched 12 V, fused, live only when engine runs |
| Black | Electronics ground | Ground where ESP32/interfacing device is grounded |
| White | Heater ground | Chassis or engine block |
| Green | High-performance analog output | Default linear lambda output |
| Brown | Standard-performance analog output | Default heater status output |
| Blue | CAN High | To CAN transceiver CANH |
| Purple | CAN Low | To CAN transceiver CANL |
| Orange | UART TX | To USB converter `Rx-Orange`; to ESP32 RX only through level shifting |
| Yellow | UART RX | To USB converter `Tx-Yellow`; from ESP32 TX through level shifting |
| Grey | UART ground | To USB converter `Gnd-Grey`; common UART ground |

## Analog Outputs

High-performance analog output, green wire:

- Default mode: linear lambda output.
- Default scale: `0 V = Lambda 0.68`, `5 V = Lambda 1.36`.
- Equivalent gasoline AFR scale: `10 AFR` to `20 AFR`.
- ESP32 ADC input must not receive 5 V directly. Use a divider before `GPIO 34`.

Standard-performance analog output, brown wire:

- Default mode: heater status.
- `0 V`: waiting for trigger before heating sensor.
- `1 V`: sensor heatup started.
- `2 V`: sensor heatup complete and heater is in closed-loop control.
- Can be switched to simulated narrowband mode with `SETNBMODEx`.

Optional spreadsheet:

- `14Point7_MS_Linear_Output_Compensation.xls` calculates compensated TunerStudio points from measured AFR.
- Example in the sheet: input conversion factor `14.7`, measured `13` and `17.1`, ideal `13.328` and `16.66`.
- Calculated compensation: slope `0.812682927`, offset `2.763121951`.
- Example TunerStudio points: `0 V = 10.88995122 AFR`, `5 V = 19.01678049 AFR`.
- This is optional analog/MS calibration material, not needed for CAN decoding.

## UART / USB

- Spartan 3 includes a USB serial converter based on an FTDI chipset.
- To enter serial commands, black electronics ground, white heater ground, and red power must all be connected and Spartan 3 must be powered.
- USB does not power Spartan 3.
- Serial terminal settings: `9600 baud`.
- The v2 manual lists UART commands for configuration. It does not list the OEM manual's UART realtime datalog command.
- All v2 commands are ASCII; case does not matter; spaces do not matter.
- UART electrical level in the manuals is treated as 5 V in the Spartan 3 OEM document. Use level shifting/protection before ESP32 RX.

## Serial Commands

| Command | Purpose | Default / Notes |
| --- | --- | --- |
| `GETHW` | Get hardware version | v2 manual |
| `GETFW` | Get firmware version | v2 manual |
| `SETTYPEx` | Set LSU sensor type | `x=0` Bosch LSU 4.9, `x=1` Bosch LSU ADV; default `x=0` |
| `GETTYPE` | Get LSU sensor type |  |
| `SETCANFORMATx` | Set CAN format | `0` default, `1` Link, `2` Adaptronic, `3` Haltech, `4` percent oxygen x100, `5` extended |
| `GETCANFORMAT` | Get CAN format |  |
| `SETCANIDx` | Set 11-bit CAN ID | Factory `1024`; examples `SETCANID1024`, `SETCANID128` |
| `GETCANID` | Get 11-bit CAN ID |  |
| `SETCANBAUDx` | Set CAN baud rate | Factory `500000`; example `SETCANBAUD1000000` |
| `GETCANBAUD` | Get CAN baud rate |  |
| `SETCANRx` | Enable/disable CAN termination resistor | `x=1` enabled, `x=0` disabled; factory enabled |
| `GETCANR` | Get CAN termination state | `1` enabled, `0` disabled |
| `SETAFRMxx.x` | Set AFR multiplier for Torque app | Factory `14.7`; examples in manual show `SETAFM14.7` typo/form |
| `GETAFRM` | Get AFR multiplier |  |
| `SETLAMFIVEVx.xx` | Lambda value at 5 V linear output | Factory `1.36`; range `0.60` to `3.40` |
| `GETLAMFIVEV` | Get lambda at 5 V |  |
| `SETLAMZEROVx.xx` | Lambda value at 0 V linear output | Factory `0.68`; range `0.60` to `3.40` |
| `GETLAMZEROV` | Get lambda at 0 V |  |
| `SETPERFx` | Performance mode | `0` standard 20 ms, `1` high performance 10 ms, `2` lean operation; factory `1` |
| `GETPERFx` | Get performance | Manual includes trailing `x` in command name |
| `SETSLOWHEATx` | Sensor heat strategy | `0` normal, `1` one-third rate, `2` wait for MS3 CAN RPM up to 10 min, `3` wait for exhaust to heat sensor to 350 C up to 10 min |
| `GETSLOWHEAT` | Get slowheat setting |  |
| `MEMRESET` | Reset factory settings |  |
| `SETLINOUTx.xxx` | Temporarily set high-performance linear output voltage | `0.000 < x.xxx < 5.000`; resumes normal operation after reboot |
| `DOCAL` | Free-air calibration | Pull sensor out of exhaust, power about 5 min, then issue command; recommended for clone sensors only |
| `GETCAL` | Get free-air calibration value | Firmware 1.04+ |
| `RESETCAL` | Reset calibration to `1.00` | Firmware 1.04+ |
| `SETCANDRx` | Set CAN data rate in Hz | Firmware 1.04+; factory `50`; manual says optimal performance at `200 Hz` |
| `GETCANDR` | Get CAN data rate | Firmware 1.04+ |
| `SETNBMODEx` | Brown output mode | `0` simulated narrowband, `2` heater status; factory `2` |
| `GETNBMODE` | Get brown output mode |  |
| `SETNBSWLAMx.xxx` | Simulated narrowband switch point lambda | Firmware 1.08+; factory `1.000`; example `SETNBSWLAM1.005` |
| `GETNBSWLAM` | Get simulated narrowband switch point |  |

## CAN Default Lambda Format

- 11-bit addressing.
- Default baud rate: `500 kbit/s`.
- Default CAN termination resistor: enabled.
- Default CAN ID: `1024` decimal, `0x400`.
- Data length code: `4`.
- Default data rate: `50 Hz`, one frame every `20 ms`.
- Byte order: big-endian.

Payload:

| Byte | Meaning |
| --- | --- |
| `Data[0]` | Lambda x1000 high byte |
| `Data[1]` | Lambda x1000 low byte |
| `Data[2]` | LSU temperature / 10 |
| `Data[3]` | Status |

Formulas:

```text
lambda = ((Data[0] << 8) + Data[1]) / 1000
sensor_temperature_c = Data[2] * 10
```

Status values:

| Value | Meaning |
| --- | --- |
| `0` | Reserved |
| `1` | Waiting for trigger before heating up |
| `2` | Sensor is heating up |
| `3` | Sensor in normal operation |
| `4+` | Reserved |

## CAN Formats Listed In v2 Manual

| Format | Format Command | CAN ID Command | Baud Command | Notes |
| --- | --- | --- | --- | --- |
| Link ECU | `SETCANFORMAT1` | `SETCANID950` | `SETCANBAUD1000000` | Link G4+ document referenced |
| Adaptronic ECU | `SETCANFORMAT2` | `SETCANID1024` | `SETCANBAUD1000000` |  |
| MegaSquirt 3 ECU | `SETCANFORMAT0` | `SETCANID1024` | `SETCANBAUD500000` | Factory defaults |
| Haltech ECU | `SETCANFORMAT3` | Not required | `SETCANBAUD1000000` | Emulates Haltech WBC1 |
| YourDyno dyno controller | `SETCANFORMAT0` | `SETCANID1024` | `SETCANBAUD1000000` |  |
| MaxxECU | `SETCANFORMAT0` | `SETCANID1024` | `SETCANBAUD500000` |  |
| Extended CAN | `SETCANFORMAT5` | See extended CAN doc | See extended CAN doc | Firmware 1.08+ |

## CAN Termination Guidance

- If Spartan 3 is the only slave on the CAN bus, enable Spartan termination with `SETCANR1`.
- Factory default is enabled.
- If multiple slaves share the bus, enable termination on the slave farthest from the master by wire length and disable/disconnect it on other slaves.
- The manual notes that in practice it may often still work, but correct termination is recommended for highest reliability.

## Notes For This ESP32 Adapter

- The firmware's current CAN defaults match the v2 factory lambda format: `500 kbit/s`, ID `0x400`, DLC `4`, big-endian lambda.
- ESP32 requires a CAN transceiver; do not connect CANH/CANL directly to ESP32 pins.
- Keep electronics ground common between Spartan and ESP32/CAN transceiver.
- Do not feed the green or brown 0-5 V analog outputs directly into ESP32 ADC pins.
- Do not feed Spartan UART TX directly into ESP32 RX without level shifting/protection.
- UART is currently best treated as configuration/control. CAN is the preferred runtime measurement path.
- The adapter can replace the supplied Spartan USB converter for configuration: browser or ESP32 USB serial monitor commands are relayed through ESP32 `Serial2` to Spartan UART.

## Older Spartan 2 / I2C Material

The folder also contains Spartan 2 OEM and Spartan 2 OEM I2C documents. These are not the target hardware for this adapter.

Useful only for historical context:

- Spartan 2 OEM I2C has I2C-only communication and lookup tables for lambda and temperature.
- Spartan 2 OEM analog linear output used the same default `0 V = Lambda 0.68`, `5 V = Lambda 1.36` concept.
- Do not copy Spartan 2 I2C addresses, lookup tables, or pinout into Spartan 3 v2 firmware.
