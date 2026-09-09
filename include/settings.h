// ============================================================================
// Configuration: edit these values for the target hardware and installation.
// Pin numbers refer to ATtiny85 port B pins (PB0..PB4).
// ============================================================================

// Hardware pins. The USI peripheral fixes I2C to PB0 (SDA) and PB2 (SCL);
// these two pins are therefore not configurable.
#define RELAYPIN 1       // PB1: relay output; HIGH switches the relay on
#define ERRORPIN 4       // PB4: error LED/output; HIGH indicates an error
#define TX_PIN 3         // PB3: TX-only debug output

// BME280 I2C addresses. Sensor 1 is the indoor sensor, sensor 2 outdoor.
// The address depends on the sensor SDO/ADR wiring: 0x76 or 0x77.
#define BME1_ADDRESS 0x76 // indoor sensor
#define BME2_ADDRESS 0x77 // outdoor sensor

// Communication and sleep timing.
#define I2C_CLOCK_KHZ 10           // I2C bus speed in kHz; supported range: 1..100
#define SERIAL_BAUD 9600           // debug TX baud rate; must match the serial monitor
#define SLEEP_WATCHDOG_CYCLES 8    // watchdog periods per cycle; about 64 s between measurements

// Calibration offsets added to each sensor result before validation and control.
// Temperature offsets are in degrees Celsius; humidity offsets are in percent RH.
#define OFFSET_TEMP_1 0            // indoor temperature correction
#define OFFSET_TEMP_2 0            // outdoor temperature correction
#define OFFSET_HUM_1 0             // indoor humidity correction
#define OFFSET_HUM_2 0             // outdoor humidity correction

// Relay control based on dew-point difference: indoor dew point minus outdoor.
// The relay turns on above TRIGGERMIN + HYSTERESIS and off below TRIGGERMIN.
#define TRIGGERMIN 4               // minimum dew-point difference in degrees Celsius
#define HYSTERESIS 1               // switching deadband in degrees Celsius
#define TEMP1_MIN 10               // indoor temperature below this disables ventilation
#define TEMP2_MIN -10              // outdoor temperature below this disables ventilation

// Plausibility limits. Measurements outside these limits trigger the failsafe.
// Humidity uses a strict lower comparison in validMeasurement(), so 0% is invalid.
#define TEMP_CRIT_HIGH 65          // maximum accepted temperature in degrees Celsius
#define TEMP_CRIT_LOW -30          // minimum accepted temperature in degrees Celsius
#define HUM_CRIT_HIGH 100          // maximum accepted relative humidity in percent
#define HUM_CRIT_LOW 0             // lower humidity boundary in percent (exclusive)

// Port operations. Keep these macros unchanged unless the hardware wiring changes.

#define RELAY_INIT (DDRB |= (1 << RELAYPIN))
#define RELAY_ON (PORTB |= (1 << RELAYPIN))
#define RELAY_OFF (PORTB &= ~(1 << RELAYPIN))

#define ERROR_INIT (DDRB |= (1 << ERRORPIN))
#define ERROR_ON (PORTB |= (1 << ERRORPIN))
#define ERROR_OFF (PORTB &= ~(1 << ERRORPIN))

// Current relay state retained across sleep cycles.
bool is_relay_on = false;