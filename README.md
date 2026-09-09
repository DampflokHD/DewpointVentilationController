# Dewpoint Ventilation Controller

Low-power dew point ventilation controller for an ATtiny85 and two BME280 sensors. The controller compares the dew point of indoor air with the dew point of outdoor air and switches a ventilation relay when outdoor air can reduce indoor moisture.

The reference implementation is intended for the Arduino IDE and has been developed for and tested on an ATtiny85. Other Arduino-compatible microcontrollers may require different pin definitions, I2C hardware, clock settings, board support, or code changes and are not covered by the tested reference configuration.

## Features

- Two BME280 sensors on one I2C bus
- Indoor/outdoor dew point comparison
- Relay control with configurable threshold and hysteresis
- Temperature limits to prevent unsuitable ventilation
- Plausibility checks for temperature and humidity
- Relay-off failsafe when sensor data or dew point calculations are invalid
- Error output that remains active until a complete valid control cycle succeeds
- Low-power power-down sleep between measurement cycles
- Watchdog-based wake-up timing
- Lightweight software/USI I2C and TX-only serial output for ATtiny85
- No external Arduino sensor library required by the included implementation

## Important: Sensor Assignment

The application logic uses a fixed logical assignment:

| Logical sensor | Location | I2C address | Meaning |
|---|---|---:|---|
| Sensor 1 | Indoor | `0x76` | Air in the room being ventilated |
| Sensor 2 | Outdoor | `0x77` | Outside reference air |

This assignment is used throughout the program. Sensor 1 is used as the indoor value and sensor 2 as the outdoor value for:

- dew point difference calculation
- temperature minimums
- calibration offsets
- diagnostic output

The physical sensor wiring must match this assignment. The BME280 address is selected by the sensor's SDO/ADR connection. If the physical sensors are reversed, exchange the two address definitions in both configurations:

```cpp
#define BME1_ADDRESS 0x77 // indoor sensor
#define BME2_ADDRESS 0x76 // outdoor sensor
```

Do not change only one of the two project variants.

## How the Control Algorithm Works

For each measurement cycle, the firmware performs the following steps:

1. Wake from power-down sleep.
2. Initialize or reconnect each BME280 sensor.
3. Trigger a forced-mode measurement on each sensor.
4. Read temperature, humidity, and pressure registers.
5. Apply the configured temperature and humidity offsets.
6. Reject missing, non-finite, or implausible temperature/humidity values.
7. Calculate both dew points.
8. Calculate the difference:

   ```text
   dew point difference = indoor dew point - outdoor dew point
   ```

9. Switch the relay according to the threshold, hysteresis, and temperature limits.
10. Release the I2C bus and enter power-down sleep.

### Current relay settings

The current release configuration uses:

```cpp
#define TRIGGERMIN 4
#define HYSTERESIS 1
#define TEMP1_MIN 10
#define TEMP2_MIN -10
```

This means:

- The relay switches **on** only when the indoor dew point is more than `5 °C` above the outdoor dew point.
- The relay switches **off** when the difference falls below `4 °C`.
- Between `4 °C` and `5 °C`, the previous relay state is retained.
- Ventilation is disabled when the indoor temperature is below `10 °C`.
- Ventilation is disabled when the outdoor temperature is below `-10 °C`.

This hysteresis prevents the relay from rapidly switching when the measured difference is close to the threshold.

## Hardware

### Required components

- ATtiny85 or compatible target controller
- Two BME280 sensors with separate I2C addresses
- Relay module or relay driver stage
- Transistor suitable for the relay coil or relay module input
- Pull-down resistor on the transistor control/input node
- External I2C pull-up resistors on SDA and SCL
- Stable power supply suitable for the controller, sensors, and relay hardware
- Optional USB-to-serial adapter for the TX-only diagnostic output

### ATtiny85 pin assignment

| ATtiny85 pin | Port | Function |
|---:|---|---|
| 5 | PB0 | I2C SDA, fixed by the USI peripheral |
| 6 | PB1 | Relay control output |
| 7 | PB2 | I2C SCL, fixed by the USI peripheral |
| 2 | PB3 | TX-only debug output |
| 3 | PB4 | Error LED or error output |
| 4 | GND | Ground |
| 8 | VCC | Supply voltage |

The ATtiny85 USI peripheral fixes I2C to PB0 and PB2 in this implementation. SDA and SCL are not configurable application pins.

![ATTiny-Pinout Grafik](attiny_pinout.jpeg)

### I2C wiring

Connect both BME280 sensors in parallel to the I2C bus:

```text
ATtiny85 PB0 (SDA) ---- BME280 indoor SDA
                    \--- BME280 outdoor SDA

ATtiny85 PB2 (SCL) ---- BME280 indoor SCL
                    \--- BME280 outdoor SCL
```

Use external pull-up resistors from SDA to the bus supply and from SCL to the bus supply. The exact resistance depends on the bus voltage, wiring length, capacitance, and sensor modules. Verify that the selected pull-ups are suitable for the chosen voltage and I2C speed.

During sleep, the firmware releases SDA and SCL as inputs and disables the ATtiny's internal pull-ups. The external pull-ups keep the bus HIGH without adding the parallel current of internal pull-ups.

### BME280 address selection

The two sensors must have different I2C addresses:

- `0x76`: logical indoor sensor 1
- `0x77`: logical outdoor sensor 2

Set the SDO/ADR connection on one sensor so that it uses `0x76` and on the other so that it uses `0x77`. Never connect two devices with the same address to the same bus unless an I2C multiplexer is used.

### Relay driver

Do not drive a relay coil directly from an ATtiny GPIO pin. Use a transistor or a suitable relay module with the required protection components. The relay control output is active HIGH in the firmware:

```text
ATtiny PB1 HIGH  -> relay driver active
ATtiny PB1 LOW   -> relay driver inactive
```

A hardware pull-down is recommended so that the relay driver remains inactive while the ATtiny is resetting, unpowered, or not yet configured. Confirm the polarity and electrical limits of the actual relay driver circuit before connecting mains or other hazardous voltages.

The firmware controls only the low-voltage relay input. It does not provide galvanic isolation or mains safety by itself.

## Configuration

Edit the configuration section near the beginning of:

```text
DewpointController/DewpointController.ino
```

The self-contained Arduino IDE sketch is the recommended starting point. The modular source files can also be integrated into PlatformIO or another build system when required, using a configuration suitable for the selected controller.

### Configuration values

| Macro | Current value | Description |
|---|---:|---|
| `BME1_ADDRESS` | `0x76` | Indoor BME280 address |
| `BME2_ADDRESS` | `0x77` | Outdoor BME280 address |
| `I2C_CLOCK_KHZ` | `10` | I2C clock in kHz |
| `SERIAL_BAUD` | `9600` | TX diagnostic baud rate |
| `SLEEP_WATCHDOG_CYCLES` | `8` | Approximately 64 seconds between cycles |
| `OFFSET_TEMP_1` | `0` | Indoor temperature correction in °C |
| `OFFSET_TEMP_2` | `0` | Outdoor temperature correction in °C |
| `OFFSET_HUM_1` | `0` | Indoor humidity correction in %RH |
| `OFFSET_HUM_2` | `0` | Outdoor humidity correction in %RH |
| `TRIGGERMIN` | `4` | Relay-off dew point difference in °C |
| `HYSTERESIS` | `1` | Relay switching deadband in °C |
| `TEMP1_MIN` | `10` | Minimum indoor temperature in °C |
| `TEMP2_MIN` | `-10` | Minimum outdoor temperature in °C |
| `TEMP_CRIT_HIGH` | `65` | Maximum accepted temperature in °C |
| `TEMP_CRIT_LOW` | `-30` | Minimum accepted temperature in °C |
| `HUM_CRIT_HIGH` | `100` | Maximum accepted humidity in %RH |
| `HUM_CRIT_LOW` | `0` | Lower humidity boundary; currently exclusive |

Temperature and humidity offsets should only be changed after comparing the BME280 readings with a calibrated reference instrument. Do not use offsets to compensate for incorrect sensor placement or an incorrectly assigned I2C address.

## Arduino IDE

The Arduino IDE version is the self-contained sketch in:

```text
DewpointController/DewpointController.ino
```

### Setup

1. Install the Arduino IDE.
2. Install a board core that supports the selected ATtiny85 board and its USI peripheral, such as ATTinyCore where appropriate.
3. Select the correct ATtiny85 board, clock, programmer, and processor settings.
4. Open `DewpointController.ino` from the `DewpointController` directory.
5. Verify that the selected clock configuration matches `F_CPU` and the actual fuse settings.
6. Select the programmer used by the hardware.
7. Compile and upload the sketch.

The `.ino` contains the application, serial output, BME280 driver, dew point calculation, watchdog setup, and sleep handling in one file. It is intended for users who want to work directly with the Arduino IDE.

Select the appropriate ATtiny85 board support, processor, clock, programmer, and upload settings in the Arduino IDE. These settings depend on the selected board core and programmer and are not prescribed by this repository.

The debug output is TX-only and uses the configured baud rate. Connect the serial adapter's RX input to ATtiny PB3 and connect grounds together. Do not connect a higher-voltage serial signal directly to the ATtiny.

## Clock and Fuse Configuration

The reference configuration targets an ATtiny85 at 1 MHz (for most power-saving). Select the matching clock and fuse settings in the Arduino IDE according to the installed board core and hardware.

Confirm the oscillator source and clock division before programming. An incorrect clock configuration affects:

- software serial timing
- I2C timing delays
- watchdog interval documentation
- delay functions

The documented sleep interval is approximate. The watchdog oscillator has its own tolerance, so measure the real wake-up interval if precise timing is required.

## ATtiny Installation & Board Configuration (Arduino IDE)

This library supports the classic ATtiny series (**ATtiny25, ATtiny45, and ATtiny85**). To program these chips in the Arduino IDE, you need to install the **ATTinyCore** hardware package.

### 1. Install ATTinyCore
1. Open the **Arduino IDE**.
2. Go to **File** → **Preferences** (on macOS: *Arduino* → *Preferences*).
3. Find the field **Additional Boards Manager URLs**.
4. Copy and paste the following URL into the field:
   ```text
   https://descartes.net/package_drazzy.com_index.json
   ```
   *(Note: If there are already other URLs there, separate them with a comma or place them on a new line).*
5. Click **OK**.
6. Navigate to **Tools** → **Board** → **Boards Manager...**
7. Type **ATTinyCore** into the search bar, locate the entry by *Spence Konde*, and click **Install**.

### 2. Configure Your Board Settings
Open the **Tools** menu in your Arduino IDE and adjust the configuration to match your hardware setup. Here is how to configure it correctly:

* **Board:** Select `ATTinyCore` → `ATtiny25/45/85 (No bootloader)`. *Choosing the "No bootloader" version is ideal since you are uploading your code directly via an ISP programmer.*
* **Port:** Select the serial port that your ISP programmer is connected to.
* **B.O.D. Level:** Choose according to your project's power and hardware preferences.
* **Chip:** Select the **exact chip** you are using (`ATtiny25`, `ATtiny45`, or `ATtiny85`).
* **Clock Source:** Choose your desired clock speed (e.g., `1 MHz (internal)` for maximum power savings or `8 MHz (internal)` for standard performance). You can also use an external crystal (quartz) if your hardware requires it.
* **Save EEPROM:** Select `EEPROM retained` if you want to keep your data stored in the EEPROM when uploading new sketches. *(Note: Burning the bootloader will always erase the EEPROM, but enabling this option ensures your data persists during normal code uploads).*.
* **LTO:** Select `Enabled`. Link Time Optimization (LTO) significantly reduces the flash memory occupied by your code. It is highly recommended for space-constrained chips like the ATtiny and works flawlessly with this library.
* **millis()/micros():** You can leave this `Enabled` or change it to `Disabled` to save a massive amount of flash memory. *Note: This library is highly optimized and works perfectly even with millis/micros disabled.*
* **Timer 1 Clock:** Keep this on the default setting: `CPU (CPU frequency)`.
* **Programmer:** Select the ISP programmer you are using to connect to the chip (e.g., `Arduino as ISP` or `USBtinyISP`).


### 3. Crucial Step: Apply Settings to the Hardware (Fuses)
Before uploading your actual sketch for the first time—or whenever you change core hardware options like the **Clock Source** or **B.O.D. Level**—you must write these settings onto the physical chip. 

1. Connect your **ISP Programmer** to the pins of your ATtiny chip.
2. Ensure your correct programmer is selected under **Tools** → **Programmer**.
3. Click **Burn Bootloader** (at the very bottom of the *Tools* menu).
   *(Don't worry: this does not actually load a heavy bootloader onto the chip; it simply configures the internal hardware registers and fuses to match your selected settings).*

⚠️ **Note:** This step only needs to be done once for a new chip, or whenever you modify any of the hardware-defining settings in the **Tools** menu. For standard code uploads, you can skip this step.


### 4. Upload Your Sketch to the ATtiny
Since your chip does not use a bootloader, you must upload your sketch using your ISP programmer.

1. Open your sketch in the Arduino IDE.
2. Go to the **Sketch** menu.
3. Click **Upload Using Programmer** (or press `Ctrl + Shift + U` / `Cmd + Shift + U` on macOS).

Your code is now running and ready to go on your ATtiny!

## Error Handling and Failsafe Behavior

The relay is forced OFF at startup. During operation, the relay is forced OFF when:

- a sensor does not respond
- sensor initialization fails
- a measurement cannot be read
- temperature or humidity is outside the configured plausibility range
- a dew point calculation is not finite
- the dew point difference is not finite

The error output remains active until both sensors provide a complete valid control cycle.

The watchdog is used here as a periodic wake-up source for power-down sleep. It is not intended as an application crash-recovery mechanism. The external relay pull-down and transistor circuit are part of the hardware safety design and must be verified independently.

## Sensor Placement

For meaningful control decisions:

- Place the indoor sensor in representative room air.
- Keep it away from direct sunlight, heaters, radiators, ventilation outlets, and condensation surfaces.
- Protect the outdoor sensor from rain and direct weather exposure while allowing free air exchange.
- Avoid placing both sensors inside the same enclosure or close to heat-producing electronics.
- Keep the sensor wiring short or verify signal quality with the selected I2C speed and pull-ups.

The dew point describes the moisture content of air more reliably for this use case than relative humidity alone. Relative humidity changes significantly with temperature even when the absolute moisture content stays similar.

## Troubleshooting

### Both sensors are reported as missing

- Check common ground and supply voltage.
- Confirm SDA is on PB0 and SCL is on PB2.
- Verify external pull-ups on both bus lines.
- Check that the sensors use different addresses.
- Confirm the selected BME280 modules are actually BME280 devices and not incompatible variants.
- Verify the selected clock and fuse configuration.

### Humidity is always 100%

- Confirm that the current firmware was rebuilt and uploaded.
- Check the BME280 calibration readout and I2C signal quality.
- Verify that the humidity register bytes are read in the correct order.
- Ensure that the sensor is powered correctly and not exposed to condensation.

### The relay never switches on

- Check that the indoor dew point exceeds the outdoor dew point by more than `TRIGGERMIN + HYSTERESIS`.
- Verify that indoor temperature is at least `TEMP1_MIN`.
- Verify that outdoor temperature is at least `TEMP2_MIN`.
- Check the error output and serial diagnostics.
- Verify the transistor driver polarity and relay power supply.

### The relay switches too frequently

- Increase `HYSTERESIS`.
- Increase the measurement interval if the application permits it.
- Check sensor placement and airflow.
- Avoid using large calibration offsets without a reference measurement.

## Project Structure

```text
.
├── DewpointController/
│   └── DewpointController.ino    # Complete Arduino IDE sketch
├── include/
│   ├── ATTinyBME280.h             # BME280 driver interface
│   ├── ATTinySerial.h             # TX-only serial interface
│   ├── header.h                   # Application declarations
│   └── settings.h                 # Modular configuration header
├── src/
│   ├── ATTinyBME280.cpp           # USI/I2C BME280 implementation
│   ├── ATTinySerial.cpp           # TX-only serial implementation
│   ├── deepsleep.cpp              # Watchdog and power-down sleep
│   ├── dewpoint.cpp               # Dew point calculation
│   └── main.cpp                   # Modular application entry point
├── LICENSE                        # GPLv3 license
├── README.md                      # Project documentation
└── .gitignore                     # Ignored build and editor files
```

The Arduino IDE sketch in `DewpointController/` is the primary self-contained release target. The modular `src/` and `include/` files are provided for users who want to create their own build setup; no PlatformIO project configuration is included.

## Limitations

- The reference implementation targets and has been tested on the ATtiny85 USI/I2C hardware arrangement.
- Other Arduino-compatible controllers are not guaranteed to be compatible without adapting the pins, I2C implementation, clock, and board configuration.
- SDA and SCL are fixed to PB0 and PB2.
- The serial interface is transmit-only.
- The program does not provide a web interface, data logging, remote configuration, or real-time clock.
- The pressure value is read and compensated by the BME280 driver but is not used for relay control.
- The relay and mains-side safety must be implemented in suitable external hardware.
- Flash space on the ATtiny85 is limited; avoid adding unnecessary features without checking the final firmware size.

## License

Copyright (c) 2026 DampflokHD.

This project is released under the MIT License. See the `LICENSE` file in the repository for the complete license text.

The MIT License permits use, modification, distribution, and commercial use, provided that the copyright and license notice are retained. Hardware, power supply, relay, and mains safety remain the responsibility of the user and the system integrator.

## Disclaimer

This project is provided as open-source software and hardware guidance without warranty. Test the complete system under controlled conditions before using it for permanent ventilation control. Never work on mains-voltage wiring without appropriate qualifications, isolation, protection, and compliance with applicable regulations.
