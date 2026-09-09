/**
 * @file DewpointController.ino
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


#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <math.h>

class __FlashStringHelper;

#ifndef F
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
#endif

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

// ============================================================================
// Small sensor/application data types
// ============================================================================

struct SensorData {
  float humidity;
  float temperature;
};

volatile uint8_t watchdog_counter = 0;
SensorData sensorData1 = {0.0f, 0.0f};
SensorData sensorData2 = {0.0f, 0.0f};
bool is_relay_on = false;

// ============================================================================
// Minimal TX-only serial output
// ============================================================================

class ATTinySerial {
public:
  explicit ATTinySerial(uint8_t pin) : _pin(pin), _delay_cycles(0) {}

  void begin(uint32_t baudrate) {
    _delay_cycles = 0;
    if (baudrate == 0 || _pin >= 8) return;

    uint32_t cycles = (uint32_t)F_CPU / baudrate / 4;
    if (cycles > 3) cycles -= 3;
    if (cycles == 0 || cycles > UINT16_MAX) return;

    DDRB |= (1 << _pin);
    PORTB |= (1 << _pin);
    _delay_cycles = (uint16_t)cycles;
  }

  void write(char value) {
    if (_delay_cycles == 0) return;

    volatile uint8_t* port = &PORTB;
    const uint8_t mask = (uint8_t)(1 << _pin);
    const uint8_t inverse_mask = (uint8_t)~mask;
    const uint16_t delay = _delay_cycles;
    uint8_t data = (uint8_t)value;
    uint8_t status = SREG;
    cli();

    *port &= inverse_mask;
    _delay_loop_2(delay);

    for (uint8_t bit = 8; bit > 0; --bit) {
      if (data & 0x01) *port |= mask;
      else *port &= inverse_mask;
      _delay_loop_2(delay);
      data >>= 1;
    }

    *port |= mask;
    SREG = status;
    _delay_loop_2(delay);
  }

  void print(char value) { write(value); }

  void print(const __FlashStringHelper* text) {
    if (text == nullptr) return;
    PGM_P pointer = reinterpret_cast<PGM_P>(text);
    char value;
    while ((value = pgm_read_byte(pointer++))) write(value);
  }

  void print(uint32_t value) {
    if (value == 0) {
      write('0');
      return;
    }

    char buffer[10];
    uint8_t index = 0;
    while (value > 0) {
      buffer[index++] = (char)('0' + value % 10);
      value /= 10;
    }
    while (index > 0) write(buffer[--index]);
  }

  void print(int32_t value) {
    if (value < 0) {
      write('-');
      print((uint32_t)(-(value + 1)) + 1);
    } else {
      print((uint32_t)value);
    }
  }

  void print(float value, uint8_t decimals = 2) {
    if (isnan(value)) {
      print(F("nan"));
      return;
    }
    if (isinf(value)) {
      print(value < 0.0f ? F("-inf") : F("inf"));
      return;
    }
    if (value < 0.0f) {
      write('-');
      value = -value;
    }
    if (decimals > 4) decimals = 4;

    uint32_t integer_part = (uint32_t)value;
    print(integer_part);
    if (decimals == 0) return;

    write('.');
    uint32_t scale = 1;
    for (uint8_t i = 0; i < decimals; ++i) scale *= 10;
    uint32_t fraction = (uint32_t)((value - integer_part) * scale + 0.5f);
    if (fraction >= scale) {
      integer_part++;
      fraction = 0;
    }

    uint32_t divisor = scale / 10;
    while (divisor > 0) {
      write((uint8_t)(fraction / divisor) + '0');
      fraction %= divisor;
      divisor /= 10;
    }
  }

  void println() {
    write('\r');
    write('\n');
  }

  void println(const __FlashStringHelper* text) {
    print(text);
    println();
  }

  void println(float value, uint8_t decimals = 2) {
    print(value, decimals);
    println();
  }

private:
  uint8_t _pin;
  uint16_t _delay_cycles;
};

ATTinySerial debugSerial(TX_PIN);

// ============================================================================
// Lightweight USI BME280 driver
// ============================================================================

class ATTinyBME280 {
public:
  explicit ATTinyBME280(uint8_t address)
      : temperature(0.0f), pressure(0.0f), humidity(0.0f),
        _address(address), _address_valid(address <= 0x7F),
        _half_period_us(20), _transfer_ok(true), _t_fine(0) {}

  bool begin(uint16_t clock_khz = 25) {
    if (!_address_valid) return false;
    if (clock_khz == 0) clock_khz = 1;
    if (clock_khz > 100) clock_khz = 100;

    _delay_ms(10);
    _half_period_us = 500 / clock_khz;
    if (_half_period_us == 0) _half_period_us = 1;

    USIDR = 0xFF;
    USICR = (1 << USIWM1) | (1 << USICS1) | (1 << USICLK);
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

    uint8_t chip_id = 0;
    if (!readRegisters(0xD0, &chip_id, 1) || chip_id != 0x60) return false;
    if (!readCalibration()) return false;
    if (!writeRegister(0xF2, 0x01)) return false;
    return writeRegister(0xF4, 0x25);
  }

  bool readData() {
    if (!_address_valid || !writeRegister(0xF4, 0x25)) return false;
    _delay_ms(10);

    uint8_t raw[8];
    if (!readRegisters(0xF7, raw, 8)) return false;

    int32_t adc_p = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_t = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_h = ((int32_t)raw[6] << 8) | raw[7];
    if (adc_t == 0x80000 || adc_p == 0x80000) return false;

    int32_t var1_t = ((((adc_t >> 3) - ((int32_t)_dig_T1 << 1))) * _dig_T2) >> 11;
    int32_t var2_t = (((((adc_t >> 4) - _dig_T1) * ((adc_t >> 4) - _dig_T1)) >> 12) * _dig_T3) >> 14;
    _t_fine = var1_t + var2_t;
    temperature = ((_t_fine * 5 + 128) >> 8) / 100.0f;

    int32_t var1_p = (_t_fine >> 1) - 64000;
    int32_t var2_p = (((var1_p >> 2) * (var1_p >> 2)) >> 11) * _dig_P6;
    var2_p += (var1_p * _dig_P5) << 1;
    var2_p = (var2_p >> 2) + (_dig_P4 << 16);
    var1_p = (((_dig_P3 * (((var1_p >> 2) * (var1_p >> 2)) >> 13)) >> 3) + ((_dig_P2 * var1_p) >> 1)) >> 18;
    var1_p = ((32768 + var1_p) * _dig_P1) >> 15;

    if (var1_p == 0) {
      pressure = 0.0f;
    } else {
      uint32_t p = (((uint32_t)(1048576 - adc_p) - (var2_p >> 12))) * 3125;
      if (p < 0x80000000) p = (p << 1) / (uint32_t)var1_p;
      else p = (p / (uint32_t)var1_p) * 2;
      var1_p = (_dig_P9 * (int32_t)(((p >> 3) * (p >> 3)) >> 13)) >> 12;
      var2_p = (((int32_t)(p >> 2)) * _dig_P8) >> 13;
      p = (uint32_t)((int32_t)p + ((var1_p + var2_p + _dig_P7) >> 4));
      pressure = p / 100.0f;
    }

    int32_t v_x1 = _t_fine - 76800;
    v_x1 = (((((adc_h << 14) - (((int32_t)_dig_H4) << 20) - ((int32_t)_dig_H5 * v_x1)) +
              16384) >> 15) *
            (((((((v_x1 * (int32_t)_dig_H6) >> 10) *
                 (((v_x1 * (int32_t)_dig_H3) >> 11) + 32768)) >> 10) +
               2097152) * (int32_t)_dig_H2 + 8192) >> 14));
    v_x1 -= (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)_dig_H1) >> 4);
    if (v_x1 < 0) v_x1 = 0;
    if (v_x1 > 419430400) v_x1 = 419430400;
    humidity = (uint32_t)(v_x1 >> 12) / 1024.0f;
    return true;
  }

  void prepareSleep() {
    // Release the bus lines; external I2C pull-ups keep them HIGH during sleep.
    DDRB &= ~((1 << PB0) | (1 << PB2));
    PORTB &= ~((1 << PB0) | (1 << PB2));
  }

  float temperature;
  float pressure;
  float humidity;

private:
  static const uint8_t SDA = PB0;
  static const uint8_t SCL = PB2;
  static const uint8_t USI_STROBE = (1 << USIWM1) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);

  uint8_t _address;
  bool _address_valid;
  uint16_t _half_period_us;
  bool _transfer_ok;
  int32_t _t_fine;
  uint16_t _dig_T1, _dig_P1;
  int16_t _dig_T2, _dig_T3, _dig_P2, _dig_P3, _dig_P4, _dig_P5, _dig_P6, _dig_P7, _dig_P8, _dig_P9;
  uint8_t _dig_H1, _dig_H3;
  int16_t _dig_H2, _dig_H4, _dig_H5;
  int8_t _dig_H6;

  void delayBus() {
    uint16_t count = _half_period_us;
    while (count--) _delay_us(1);
  }

  uint8_t transfer(uint8_t mode) {
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | mode;
    _transfer_ok = true;
    while (!(USISR & (1 << USIOIF))) {
      delayBus();
      USICR = USI_STROBE;
      uint16_t timeout = 1000;
      while (!(PINB & (1 << SCL)) && --timeout) _delay_us(1);
      if (timeout == 0) {
        _transfer_ok = false;
        PORTB &= ~(1 << SCL);
        recoverBus();
        return 0;
      }
      delayBus();
      USICR = USI_STROBE;
    }
    delayBus();
    uint8_t value = USIDR;
    USIDR = 0xFF;
    DDRB |= (1 << SDA);
    return value;
  }

  bool start() {
    PORTB |= (1 << SCL) | (1 << SDA);
    DDRB |= (1 << SCL) | (1 << SDA);
    uint16_t timeout = 1000;
    while (!(PINB & (1 << SCL)) && --timeout) _delay_us(1);
    if (timeout == 0) {
      recoverBus();
      return false;
    }
    PORTB &= ~(1 << SDA);
    delayBus();
    PORTB &= ~(1 << SCL);
    PORTB |= (1 << SDA);
    return true;
  }

  bool stop() {
    PORTB &= ~(1 << SDA);
    PORTB |= (1 << SCL);
    uint16_t timeout = 1000;
    while (!(PINB & (1 << SCL)) && --timeout) _delay_us(1);
    if (timeout == 0) {
      recoverBus();
      return false;
    }
    delayBus();
    PORTB |= (1 << SDA);
    delayBus();
    return true;
  }

  bool recoverBus() {
    bool recovered = true;
    USICR = 0;
    PORTB |= (1 << SDA) | (1 << SCL);
    DDRB &= ~((1 << SDA) | (1 << SCL));

    for (uint8_t pulse = 0; pulse < 9; ++pulse) {
      uint16_t timeout = 1000;
      while (!(PINB & (1 << SCL)) && --timeout) _delay_us(1);
      if (timeout == 0) {
        recovered = false;
        break;
      }
      delayBus();
      DDRB |= (1 << SCL);
      PORTB &= ~(1 << SCL);
      delayBus();
      DDRB &= ~(1 << SCL);
      PORTB |= (1 << SCL);
    }

    if (recovered) {
      DDRB |= (1 << SDA);
      PORTB &= ~(1 << SDA);
      uint16_t timeout = 1000;
      while (!(PINB & (1 << SCL)) && --timeout) _delay_us(1);
      if (timeout == 0) {
        recovered = false;
      } else {
        delayBus();
        DDRB &= ~(1 << SDA);
        PORTB |= (1 << SDA);
        delayBus();
      }
    }

    USIDR = 0xFF;
    USICR = (1 << USIWM1) | (1 << USICS1) | (1 << USICLK);
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);
    return recovered;
  }

  bool writeByte(uint8_t value) {
    PORTB &= ~(1 << SCL);
    USIDR = value;
    transfer(0x00);
    if (!_transfer_ok) return false;
    DDRB &= ~(1 << SDA);
    uint8_t ack = transfer(0x0E);
    return _transfer_ok && !(ack & 0x01);
  }

  uint8_t readByte(bool ack) {
    DDRB &= ~(1 << SDA);
    uint8_t value = transfer(0x00);
    if (!_transfer_ok) return 0;
    USIDR = ack ? 0x00 : 0xFF;
    transfer(0x0E);
    return value;
  }

  bool writeRegister(uint8_t reg, uint8_t value) {
    if (!start()) return false;
    if (!writeByte(_address << 1)) {
      stop();
      return false;
    }
    if (!writeByte(reg) || !writeByte(value)) {
      stop();
      return false;
    }
    return stop();
  }

  bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    if (buffer == nullptr || length == 0 || !start()) return false;
    if (!writeByte(_address << 1) || !writeByte(reg) || !start() || !writeByte((_address << 1) | 1)) {
      stop();
      return false;
    }
    for (uint8_t i = 0; i < length; ++i) {
      buffer[i] = readByte(i < length - 1);
      if (!_transfer_ok) {
        stop();
        return false;
      }
    }
    return stop();
  }

  bool readCalibration() {
    uint8_t first[26], second[7];
    if (!readRegisters(0x88, first, 26) || !readRegisters(0xE1, second, 7)) return false;

    _dig_T1 = (first[1] << 8) | first[0];
    _dig_T2 = (first[3] << 8) | first[2];
    _dig_T3 = (first[5] << 8) | first[4];
    _dig_P1 = (first[7] << 8) | first[6];
    _dig_P2 = (first[9] << 8) | first[8];
    _dig_P3 = (first[11] << 8) | first[10];
    _dig_P4 = (first[13] << 8) | first[12];
    _dig_P5 = (first[15] << 8) | first[14];
    _dig_P6 = (first[17] << 8) | first[16];
    _dig_P7 = (first[19] << 8) | first[18];
    _dig_P8 = (first[21] << 8) | first[20];
    _dig_P9 = (first[23] << 8) | first[22];
    _dig_H1 = first[25];
    _dig_H2 = (second[1] << 8) | second[0];
    _dig_H3 = second[2];
    _dig_H4 = (int16_t)((second[3] << 4) | (second[4] & 0x0F));
    _dig_H5 = (int16_t)((second[5] << 4) | (second[4] >> 4));
    if (_dig_H4 & 0x0800) _dig_H4 |= (int16_t)0xF000;
    if (_dig_H5 & 0x0800) _dig_H5 |= (int16_t)0xF000;
    _dig_H6 = second[6];
    return true;
  }
};

ATTinyBME280 sensor1(BME1_ADDRESS);
ATTinyBME280 sensor2(BME2_ADDRESS);
bool sensor1_ready = false;
bool sensor2_ready = false;

// ============================================================================
// Application helpers
// ============================================================================

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

static float dewpoint(float temperature, float humidity) {
  float a = temperature >= 0.0f ? 7.5f : 7.6f;
  float b = temperature >= 0.0f ? 237.3f : 240.7f;
  float saturation = 6.1078f * pow(10.0f, (a * temperature) / (b + temperature));
  float vapor = saturation * humidity / 100.0f;
  float value = log10(vapor / 6.1078f);
  return (b * value) / (a - value);
}

static void setupWatchdog() {
  cli();
  wdt_reset();
  MCUSR &= ~(1 << WDRF);
  WDTCR |= (1 << WDCE) | (1 << WDE);
  WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
  sei();
}

static void deepSleep(uint8_t cycles) {
  watchdog_counter = 0;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  while (watchdog_counter < cycles) {
    sleep_enable();
    cli();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
}

ISR(WDT_vect) {
  watchdog_counter++;
}

// ============================================================================
// Arduino application
// ============================================================================

void loop();

int main(void) {
  ERROR_INIT;
  ERROR_ON;
  RELAY_INIT;
  RELAY_OFF;
  debugSerial.begin(SERIAL_BAUD);

  sensor1_ready = sensor1.begin(I2C_CLOCK_KHZ);
  sensor2_ready = sensor2.begin(I2C_CLOCK_KHZ);

  if (!sensor1_ready) {
    debugSerial.print(F("ERROR: Sensor 1 address "));
    printHexByte(BME1_ADDRESS);
    debugSerial.println(F(" missing"));
  }
  if (!sensor2_ready) {
    debugSerial.print(F("ERROR: Sensor 2 address "));
    printHexByte(BME2_ADDRESS);
    debugSerial.println(F(" missing"));
  }

  setupWatchdog();

  while (1) {
    loop();
  }

  return 0;
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
    sensorData1.temperature = sensor1.temperature + OFFSET_TEMP_1;
    sensorData1.humidity = sensor1.humidity + OFFSET_HUM_1;
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
    sensorData2.temperature = sensor2.temperature + OFFSET_TEMP_2;
    sensorData2.humidity = sensor2.humidity + OFFSET_HUM_2;
    sensor2_ok = validMeasurement(sensorData2);
    if (!sensor2_ok) {
      sensor2_ready = false;
      debugSerial.println(F("ERROR: Sensor 2 value invalid!"));
    }
  }

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

  if (control_ok) {
    ERROR_OFF;
    if (delta_dp > TRIGGERMIN + HYSTERESIS) is_relay_on = true;
    if (delta_dp < TRIGGERMIN) is_relay_on = false;
    if (sensorData1.temperature < TEMP1_MIN) is_relay_on = false;
    if (sensorData2.temperature < TEMP2_MIN) is_relay_on = false;
  } else {
    ERROR_ON;
    is_relay_on = false;
    RELAY_OFF;
    debugSerial.println(F("RELAY OFF FAILSAFE"));
  }

  debugSerial.println();
  debugSerial.println(F("Measurements:"));
  if (sensor1_ok) {
    debugSerial.print(F("Sensor-1: T="));
    debugSerial.print(sensorData1.temperature);
    debugSerial.print(F("°C H="));
    debugSerial.print(sensorData1.humidity);
    debugSerial.print(F("% DP="));
    if (dewpoint1_ok) debugSerial.println(dewpoint1);
    else debugSerial.println(F("invalid"));
  } else {
    debugSerial.println(F("Sensor-1: unavailable"));
  }
  if (sensor2_ok) {
    debugSerial.print(F("Sensor-2: T="));
    debugSerial.print(sensorData2.temperature);
    debugSerial.print(F("°C H="));
    debugSerial.print(sensorData2.humidity);
    debugSerial.print(F("% DP="));
    if (dewpoint2_ok) debugSerial.println(dewpoint2);
    else debugSerial.println(F("invalid"));
  } else {
    debugSerial.println(F("Sensor-2: unavailable"));
  }

  if (is_relay_on) {
    RELAY_ON;
    debugSerial.println(F("Relay is ON"));
  } else {
    RELAY_OFF;
    debugSerial.println(F("Relay is OFF"));
  }

  debugSerial.println(F("\n\n"));

  sensor1.prepareSleep();
  sensor2.prepareSleep();
  deepSleep(SLEEP_WATCHDOG_CYCLES);
}
