/**
 * @file ATTinyBME280.h
 * @author DampflokHD
 * @brief Lightweight, USI-based BME280 library tailored for ATtiny MCUs with low power consumption and customizable I2C clock frequency.
 * @version 1.0.0
 * @date 2026-09-08
 * 
 * @copyright Copyright (c) 2026 DampflokHD. Licensed under the MIT License.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

class ATTinyBME280 {
public:
    explicit ATTinyBME280(uint8_t i2c_addr = 0x76); // Creates object with default address 0x76 [e.g. ATTinyBME280 sensor1; or ATTinyBME280 sensor2(0x77);]

    bool begin(uint16_t clock_khz = 25); // Initializes I2C bus with clock speed in kHz (default: 25 kHz) and verifies sensor ID

    bool isConnected(); // Checks whether the sensor responds with the BME280 chip ID

    bool readData(); // Triggers measurement in Forced Mode and updates public measurement variables

    float temperature; // Public variable storing measured temperature in degrees Celsius (°C)
    float pressure;    // Public variable storing measured atmospheric pressure in hectopascals (hPa)
    float humidity;    // Public variable storing measured relative humidity in percent (%RH)

    void prepareSleep(); // Sets SDA and SCL to high-impedance mode before deep sleep to prevent leakage currents

private:
    uint8_t _addr;       // I2C target address of the BME280 sensor
    bool _address_valid;
    uint16_t _half_period_us;  // Half-period bus delay in microseconds, calculated from the target frequency
    bool _transfer_ok;

    // Factory compensation parameters retrieved from sensor NVM
    uint16_t dig_T1, dig_P1;
    int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3;
    int16_t  dig_H2, dig_H4, dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;     // Fine temperature reading required for pressure and humidity calculation

    // Low-Level USI (Universal Serial Interface) routines
    void bus_delay();                     // Generates timing delay to maintain configured I2C clock speed
    uint8_t usi_transfer(uint8_t mode);   // Controls USI shift register and generates SCL clock pulses
    bool usi_start();                     // Generates I2C START condition on the bus
    bool usi_stop();                      // Generates I2C STOP condition on the bus
    bool recoverBus();                    // Releases a stuck I2C bus with up to nine SCL pulses
    bool usi_write(uint8_t data);         // Writes a single byte to the bus and returns true if ACK received
    uint8_t usi_read(bool ack);           // Reads a byte from the sensor and sends ACK (more bytes) or NACK (last byte)

    // Register access helpers
    bool writeRegister(uint8_t reg, uint8_t value);                 // Writes a single byte to a sensor control register
    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);  // Reads multiple consecutive register bytes into a buffer
    bool readCalibration();                                         // Reads all 33 factory calibration parameters on startup
};
