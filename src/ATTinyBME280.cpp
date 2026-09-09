/**
 * @file ATTinyBME280.cpp
 * @author DampflokHD
 * @brief Implementation of the lightweight USI-based BME280 library for ATtiny MCUs.
 * @version 1.0.0
 * @date 2026-09-08
 * 
 * @copyright Copyright (c) 2026 DampflokHD. Licensed under the MIT License.
 */

#include "ATTinyBME280.h"
#include <util/delay.h>

#if !defined(USICR) || !defined(USISR) || !defined(USIDR)
#error "ATTinyBME280 requires a MCU with USI support, such as ATtiny25/45/85."
#endif

// Hardware pin definitions for ATtiny25/45/85 USI
#define DDR_USI   DDRB
#define PORT_USI  PORTB
#define PIN_USI   PINB
#define PORT_SDA  PB0
#define PORT_SCL  PB2

// USI Control Register preset for 2-wire I2C clock strobing
#define USI_CLK_STROBE ((1 << USIWM1) | (0 << USIWM0) | (1 << USICS1) | (0 << USICS0) | (1 << USICLK) | (1 << USITC))

ATTinyBME280::ATTinyBME280(uint8_t i2c_addr) 
    : temperature(0.0f), pressure(0.0f), humidity(0.0f), 
        _addr(i2c_addr), _address_valid(i2c_addr <= 0x7F),
        _half_period_us(20), _transfer_ok(true), t_fine(0) {}

void ATTinyBME280::bus_delay() {
    uint16_t count = _half_period_us;
    while (count--) {
        _delay_us(1);
    }
}

uint8_t ATTinyBME280::usi_transfer(uint8_t mode) {
    // Clear all USI flags and preset the counter (0x00 = 8 bits, 0x0E = 1 bit).
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) |
            (1 << USIDC) | mode;
    _transfer_ok = true;
    uint8_t usi_ctrl = USI_CLK_STROBE;

    while (!(USISR & (1 << USIOIF))) {
        bus_delay();
        USICR = usi_ctrl; // SCL High
        
        // Clock stretching detection with simple loop bound to avoid infinite hangs
        uint16_t timeout = 1000;
        while (!(PIN_USI & (1 << PORT_SCL)) && --timeout) {
            _delay_us(1);
        }

        if (timeout == 0) {
            _transfer_ok = false;
            PORT_USI &= ~(1 << PORT_SCL);
            recoverBus();
            return 0;
        }

        bus_delay();
        USICR = usi_ctrl; // SCL Low
    }

    bus_delay();
    uint8_t data = USIDR;
    USIDR = 0xFF;               // Release SDA line
    DDR_USI |= (1 << PORT_SDA); // Set SDA back to output
    return data;
}

bool ATTinyBME280::usi_start() {
    // Release SCL first so a repeated START can be generated safely.
    PORT_USI |= (1 << PORT_SCL) | (1 << PORT_SDA);
    DDR_USI  |= (1 << PORT_SCL) | (1 << PORT_SDA);
    uint16_t timeout = 1000;
    while (!(PIN_USI & (1 << PORT_SCL)) && --timeout) {
        _delay_us(1);
    }
    if (timeout == 0) {
        recoverBus();
        return false;
    }

    PORT_USI &= ~(1 << PORT_SDA); // SDA Low
    bus_delay();
    PORT_USI &= ~(1 << PORT_SCL); // SCL Low
    PORT_USI |= (1 << PORT_SDA);  // Release SDA for data transfer
    return true;
}

bool ATTinyBME280::usi_stop() {
    PORT_USI &= ~(1 << PORT_SDA); // SDA Low
    PORT_USI |= (1 << PORT_SCL);  // SCL High

    uint16_t timeout = 1000;
    while (!(PIN_USI & (1 << PORT_SCL)) && --timeout) {
        _delay_us(1);
    }

    if (timeout == 0) {
        recoverBus();
        return false;
    }

    bus_delay();
    PORT_USI |= (1 << PORT_SDA);  // SDA High
    bus_delay();
    return true;
}

bool ATTinyBME280::recoverBus() {
    const uint8_t sda_mask = (1 << PORT_SDA);
    const uint8_t scl_mask = (1 << PORT_SCL);
    bool recovered = true;

    // Disable USI before temporarily driving the bus as GPIO.
    USICR = 0;

    // Release both lines before generating recovery clocks.
    PORT_USI |= sda_mask | scl_mask;
    DDR_USI &= ~(sda_mask | scl_mask);

    for (uint8_t pulse = 0; pulse < 9; ++pulse) {
        uint16_t timeout = 1000;
        while (!(PIN_USI & scl_mask) && --timeout) {
            _delay_us(1);
        }
        if (timeout == 0) {
            recovered = false;
            break;
        }

        bus_delay();
        DDR_USI |= scl_mask;
        PORT_USI &= ~scl_mask;
        bus_delay();
        DDR_USI &= ~scl_mask;
        PORT_USI |= scl_mask;
    }

    if (recovered) {
        // Generate STOP: SDA low, release SCL high, then release SDA high.
        DDR_USI |= sda_mask;
        PORT_USI &= ~sda_mask;
        uint16_t timeout = 1000;
        while (!(PIN_USI & scl_mask) && --timeout) {
            _delay_us(1);
        }
        if (timeout == 0) {
            recovered = false;
        } else {
            bus_delay();
            DDR_USI &= ~sda_mask;
            PORT_USI |= sda_mask;
            bus_delay();
        }
    }

    // Restore USI two-wire mode after temporary GPIO clocking.
    USIDR = 0xFF;
    USICR = (1 << USIWM1) | (1 << USICS1) | (1 << USICLK);
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);
    return recovered;
}

bool ATTinyBME280::usi_write(uint8_t data) {
    PORT_USI &= ~(1 << PORT_SCL);
    USIDR = data;
    usi_transfer(0x00); // Send 8 bits
    if (!_transfer_ok) return false;
    
    DDR_USI &= ~(1 << PORT_SDA);       // Set SDA as input to receive ACK
    uint8_t ack = usi_transfer(0x0E); // Read 1 bit ACK
    return _transfer_ok && !(ack & 0x01); // 0 means ACK received
}

uint8_t ATTinyBME280::usi_read(bool ack) {
    DDR_USI &= ~(1 << PORT_SDA); // Set SDA as input
    uint8_t data = usi_transfer(0x00);
    if (!_transfer_ok) return 0;

    USIDR = ack ? 0x00 : 0xFF;   // Send ACK (0x00) or NACK (0xFF)
    usi_transfer(0x0E);          // Send 1 bit response
    return data;
}

bool ATTinyBME280::writeRegister(uint8_t reg, uint8_t value) {
    if (!usi_start()) return false;
    if (!usi_write(_addr << 1)) {
        usi_stop();
        return false;
    }
    if (!usi_write(reg) || !usi_write(value)) {
        usi_stop();
        return false;
    }
    return usi_stop();
}

bool ATTinyBME280::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    if (buffer == nullptr || len == 0 || !usi_start()) return false;
    if (!usi_write(_addr << 1) || !usi_write(reg) || !usi_start() ||
        !usi_write((_addr << 1) | 1)) {
        usi_stop();
        return false;
    }
    
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = usi_read(i < (len - 1));
        if (!_transfer_ok) {
            usi_stop();
            return false;
        }
    }
    return usi_stop();
}

bool ATTinyBME280::begin(uint16_t clock_khz) {
    if (!_address_valid) return false;
    if (clock_khz == 0) clock_khz = 1;
    if (clock_khz > 100) clock_khz = 100;

    // Allow the sensor to complete its power-on reset before reading chip ID.
    _delay_ms(10);
    
    // Half-period delay calculation: delay = 1000 / (2 * kHz)
    _half_period_us = 500 / clock_khz;
    if (_half_period_us == 0) _half_period_us = 1;

    // Configure USI hardware for 2-wire I2C mode
    USIDR = 0xFF; // Keep SDA released until the first data byte is sent.
    USICR = (1 << USIWM1) | (0 << USIWM0) | (1 << USICS1) | (0 << USICS0) | (1 << USICLK);
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

    uint8_t chip_id = 0;
    if (!readRegisters(0xD0, &chip_id, 1) || chip_id != 0x60) return false; // Verify BME280 signature

    if (!readCalibration()) return false;

    if (!writeRegister(0xF2, 0x01)) return false; // Humidity oversampling x1
    if (!writeRegister(0xF4, 0x25)) return false; // Pressure x1, Temperature x1, Forced Mode

    return true;
}

bool ATTinyBME280::isConnected() {
    if (!_address_valid) return false;
    uint8_t chip_id = 0;
    return readRegisters(0xD0, &chip_id, 1) && chip_id == 0x60;
}

void ATTinyBME280::prepareSleep() {
    // Release the bus lines; external I2C pull-ups keep them HIGH during sleep.
    DDR_USI  &= ~((1 << PORT_SDA) | (1 << PORT_SCL));
    PORT_USI &= ~((1 << PORT_SDA) | (1 << PORT_SCL));
}

bool ATTinyBME280::readCalibration() {
    uint8_t calib1[26], calib2[7];
    if (!readRegisters(0x88, calib1, 26) || !readRegisters(0xE1, calib2, 7)) {
        return false;
    }

    dig_T1 = (calib1[1] << 8) | calib1[0];
    dig_T2 = (calib1[3] << 8) | calib1[2];
    dig_T3 = (calib1[5] << 8) | calib1[4];

    dig_P1 = (calib1[7] << 8) | calib1[6];
    dig_P2 = (calib1[9] << 8) | calib1[8];
    dig_P3 = (calib1[11] << 8) | calib1[10];
    dig_P4 = (calib1[13] << 8) | calib1[12];
    dig_P5 = (calib1[15] << 8) | calib1[14];
    dig_P6 = (calib1[17] << 8) | calib1[16];
    dig_P7 = (calib1[19] << 8) | calib1[18];
    dig_P8 = (calib1[21] << 8) | calib1[20];
    dig_P9 = (calib1[23] << 8) | calib1[22];

    dig_H1 = calib1[25];
    dig_H2 = (calib2[1] << 8) | calib2[0];
    dig_H3 = calib2[2];
    dig_H4 = static_cast<int16_t>((calib2[3] << 4) | (calib2[4] & 0x0F));
    dig_H5 = static_cast<int16_t>((calib2[5] << 4) | (calib2[4] >> 4));
    if (dig_H4 & 0x0800) dig_H4 |= static_cast<int16_t>(0xF000);
    if (dig_H5 & 0x0800) dig_H5 |= static_cast<int16_t>(0xF000);
    dig_H6 = calib2[6];
    return true;
}

bool ATTinyBME280::readData() {
    if (!_address_valid) return false;
    // Trigger measurement in forced mode; 10 ms matches x1 oversampling.
    if (!writeRegister(0xF4, 0x25)) return false;
    _delay_ms(10); // Wait for sensor conversion

    uint8_t raw[8];
    if (!readRegisters(0xF7, raw, 8)) return false;

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8) | raw[7];

    if (adc_T == 0x80000 || adc_P == 0x80000) return false; // Invalid or uninitialized measurement

    // 1. Temperature Calculation
    int32_t var1_T = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2_T = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1_T + var2_T;
    temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;

    // 2. Pressure Calculation
    int32_t var1_P = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    int32_t var2_P = (((var1_P >> 2) * (var1_P >> 2)) >> 11) * ((int32_t)dig_P6);
    var2_P = var2_P + ((var1_P * ((int32_t)dig_P5)) << 1);
    var2_P = (var2_P >> 2) + (((int32_t)dig_P4) << 16);
    var1_P = (((dig_P3 * (((var1_P >> 2) * (var1_P >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1_P) >> 1)) >> 18;
    var1_P = ((((32768 + var1_P)) * ((int32_t)dig_P1)) >> 15);

    if (var1_P == 0) {
        pressure = 0.0f;
    } else {
        uint32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2_P >> 12))) * 3125;
        if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1_P);
        else p = (p / (uint32_t)var1_P) * 2;
        
        var1_P = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
        var2_P = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
        p = (uint32_t)((int32_t)p + ((var1_P + var2_P + dig_P7) >> 4));
        pressure = p / 100.0f;
    }

    // 3. Humidity Calculation
    int32_t v_x1 = (t_fine - ((int32_t)76800));
    v_x1 = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1)) + 
              ((int32_t)16384)) >> 15) * (((((((v_x1 * ((int32_t)dig_H6)) >> 10) * 
              (((v_x1 * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + 
              ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1 = (v_x1 < 0) ? 0 : v_x1;
    v_x1 = (v_x1 > 419430400) ? 419430400 : v_x1;
    humidity = (uint32_t)(v_x1 >> 12) / 1024.0f;

    return true;
}
