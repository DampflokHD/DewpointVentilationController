/**
 * @file main.cpp
 * @author DampflokHD
 * @brief Low-power dew point ventilation controller for ATtiny85.
 *
 * Reads two BME280 sensors over the ATtiny85 USI/I2C interface, calculates
 * both dew points and switches a relay only when both sensor values are
 * valid and all configured temperature conditions are met.
 *
 * Hardware defaults:
 * - BME280 sensor 1: I2C address 0x76, indoor measurement
 * - BME280 sensor 2: I2C address 0x77, outdoor measurement
 * - SDA: PB0, SCL: PB2, relay: PB1, error LED: PB4, debug TX: PB3
 * - I2C clock: 10 kHz, debug output: 9600 baud
 *
 * Safety behavior:
 * - Relay is initialized and forced LOW at startup.
 * - Missing, invalid or incomplete sensor data keeps the relay OFF.
 * - The error LED remains ON until a complete valid control cycle succeeds.
 * - The BME280 bus is released before the ATtiny enters power-down sleep.
 *
 * @version 1.0.0
 * @date 2026-09-08
 * @copyright Copyright (c) 2026 DampflokHD. Licensed under the MIT License.
 */


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include "header.h"
#include "settings.h"
#include "ATTinySerial.h"
#include "ATTinyBME280.h"


volatile uint8_t watchdog_counter = 0;

ISR(WDT_vect) {
  watchdog_counter++;
}

SensorData sensorData1;
SensorData sensorData2;

ATTinySerial debugSerial(TX_PIN);  // TX Pin for debugging output

ATTinyBME280 sensor1(BME1_ADDRESS);
ATTinyBME280 sensor2(BME2_ADDRESS);

bool sensor1_ready = false;
bool sensor2_ready = false;

static bool validMeasurement(const SensorData& data) {
  return isfinite(data.temperature) && isfinite(data.humidity) &&
         data.temperature >= TEMP_CRIT_LOW && data.temperature <= TEMP_CRIT_HIGH &&
         data.humidity > HUM_CRIT_LOW && data.humidity <= HUM_CRIT_HIGH;
}

static void printHexByte(uint8_t value) {
  const char hex[] = "0123456789ABCDEF";
  debugSerial.print(F("0x"));
  debugSerial.print(hex[(value >> 4) & 0x0F]);
  debugSerial.print(hex[value & 0x0F]);
}

int main(void) {
  ERROR_INIT;  // initialize error pin as output
  ERROR_ON;  // stay in error state until a complete valid cycle succeeds
  RELAY_INIT;  // initialize relay pin as output
  RELAY_OFF;  // turn off relay
  debugSerial.begin(SERIAL_BAUD);

  // Initialize both sensors and load their factory calibration data.
  sensor1_ready = sensor1.begin(I2C_CLOCK_KHZ);
  sensor2_ready = sensor2.begin(I2C_CLOCK_KHZ);

  if (!sensor1_ready) {
    ERROR_ON;
    debugSerial.print(F("ERROR: Sensor 1 address "));
    printHexByte(BME1_ADDRESS);
    debugSerial.println(F(" missing"));
  }
  
  if (!sensor2_ready) {
    ERROR_ON;
    debugSerial.print(F("ERROR: Sensor 2 address "));
    printHexByte(BME2_ADDRESS);
    debugSerial.println(F(" missing"));
  }
  
  setupWatchdog();

  while (1) {
    loop();  // call the main loop function
  }
}

void loop() {

  debugSerial.println(F("----- ATTiny is Awake! -----"));

  if (!sensor1_ready) sensor1_ready = sensor1.begin(I2C_CLOCK_KHZ);
  if (!sensor2_ready) sensor2_ready = sensor2.begin(I2C_CLOCK_KHZ);

  bool sensor1_ok = false;
  bool sensor2_ok = false;

    if (!sensor1_ready || !sensor1.readData()) {
    sensor1_ready = false;
    debugSerial.println(F("ERROR: Sensor 1 unavailable!"));
  } else {
    sensorData1.humidity = sensor1.humidity + OFFSET_HUM_1;
    sensorData1.temperature = sensor1.temperature + OFFSET_TEMP_1;
    sensor1_ok = validMeasurement(sensorData1);
    if (!sensor1_ok) {
      sensor1_ready = false;
      debugSerial.println(F("ERROR: Sensor 1 value invalid!"));
    }
  }

    if (!sensor2_ready || !sensor2.readData()) {
    sensor2_ready = false;
    debugSerial.println(F("ERROR: Sensor 2 unavailable!"));
  } else {
    sensorData2.humidity = sensor2.humidity + OFFSET_HUM_2;
    sensorData2.temperature = sensor2.temperature + OFFSET_TEMP_2;
    sensor2_ok = validMeasurement(sensorData2);
    if (!sensor2_ok) {
      sensor2_ready = false;
      debugSerial.println(F("ERROR: Sensor 2 value invalid!"));
    }
  }

  // **** calculate dew points and delta of dew points ****
  float dewpoint1 = 0.0f;
  float dewpoint2 = 0.0f;
  float delta_dp = 0.0f;
  bool dewpoint1_ok = false;
  bool dewpoint2_ok = false;

  if (sensor1_ok) {
    dewpoint1 = dewpoint(sensorData1.temperature, sensorData1.humidity);
    dewpoint1_ok = isfinite(dewpoint1);
  }
  if (sensor2_ok) {
    dewpoint2 = dewpoint(sensorData2.temperature, sensorData2.humidity);
    dewpoint2_ok = isfinite(dewpoint2);
  }

  bool control_ok = sensor1_ok && sensor2_ok && dewpoint1_ok && dewpoint2_ok;
  if (control_ok) {
    delta_dp = dewpoint1 - dewpoint2;
    control_ok = isfinite(delta_dp);
  }

  // Clear the error indicator only when both sensors and control values are valid.
  if (control_ok) {
    ERROR_OFF;
  } else {
    ERROR_ON;
    is_relay_on = false;
    RELAY_OFF;
    debugSerial.println(F("RELAY OFF FAILSAFE"));
  }

  // **** some debbugging if debugSerial monitor is connected ****
  debugSerial.println(F("\nMeasurements:"));
  if (sensor1_ok) {
    debugSerial.print(F("Sensor-1: Humidity: "));
    debugSerial.print(sensorData1.humidity);
    debugSerial.print(F(" % Temperature: "));
    debugSerial.print(sensorData1.temperature);
    debugSerial.print(F(" °C Dewpoint: "));
    if (dewpoint1_ok) {
      debugSerial.print(dewpoint1);
      debugSerial.println(F(" °C"));
    } else {
      debugSerial.println(F(" invalid"));
    }
  } else {
    debugSerial.println(F("Sensor-1: unavailable"));
  }
  // ********************************************************
  if (sensor2_ok) {
    debugSerial.print(F("Sensor-2: Humidity: "));
    debugSerial.print(sensorData2.humidity);
    debugSerial.print(F(" % Temperature: "));
    debugSerial.print(sensorData2.temperature);
    debugSerial.print(F(" °C Dewpoint: "));
    if (dewpoint2_ok) {
      debugSerial.print(dewpoint2);
      debugSerial.println(F(" °C"));
    } else {
      debugSerial.println(F(" invalid"));
    }
  } else {
    debugSerial.println(F("Sensor-2: unavailable"));
  }
  // ********************************************************

  // **** preferences of the dew point ventilation  ****
  if (control_ok) {
    if (delta_dp > (TRIGGERMIN + HYSTERESIS)) is_relay_on = true;
    if (delta_dp < (TRIGGERMIN)) is_relay_on = false;
    if (sensorData1.temperature < TEMP1_MIN) is_relay_on = false;
    if (sensorData2.temperature < TEMP2_MIN) is_relay_on = false;
  }

  // **** switch relay ****
  if (is_relay_on == true) {
    RELAY_ON;  // turn on relay
    debugSerial.println(F("Relay is ON"));
  } else {
    RELAY_OFF;  // turn off relay
    debugSerial.println(F("Relay is OFF"));
  }

  debugSerial.println(F("\n\n"));

  sensor1.prepareSleep();
  sensor2.prepareSleep();
  deepSleep(SLEEP_WATCHDOG_CYCLES);
}
