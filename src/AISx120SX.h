/*
 * AISx120SX.h
 *
 *  Created on: 2021-04-24
 *      Author: Joshua Cayetano-Emond
 * 
 * This library provides the necessary functions to interface with an
 * AIS1120SX (1-axis) or AIS2120SX (2-axis) sensor from STMicroelectronics with
 * an Arduino or similar board.
 * 
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "AIS_regs.h"
#include "CRC.h"

typedef enum
{
  _400Hz,
  _800Hz,
  _1600Hz
} bandwidth;

typedef enum
{
  initialization,
  normal,
  test,
  error
} statusTypes;

// AISx120SX class definition
class AISx120SX
{
private:
  uint8_t _CS;        // Chip select pin assignment
  uint8_t axisNumber; // 1 for AIS1120SX and 2 for AIS2120SX

  SPISettings AISSettings;

  // REG_STATUS_0 flags
  statusTypes status;
  bool testmode_enabled;
  bool reg_ctrl_0_wr_err;
  bool loss_cap;
  bool end_of_pwrup;
  bool rst_active;

  // REG_STATUS_1 flags
  bool spi_err;
  bool eeprom_err;
  bool off_canc_chy_err;
  bool off_canc_chx_err;
  bool reg_config_wr_err;
  bool reg_ctrl_1_wr_err;

  // REG_STATUS_2 flags
  bool a2d_sat_chy;
  bool a2d_sat_chx;
  bool charge_pump_err;
  bool vreg_low_volt_det;
  bool vreg_high_volt_det;
  bool vdd_low_volt_det;
  bool vdd_high_volt_det;

  // Find parity (of any width up to the width of an unsigned)
  int calcEvenParityBit(unsigned par, unsigned width);

  // finds if a given payload has even parity
  int calculateEvenParity(uint8_t *payload, int size);

  // reads the bits in data given a bit mask
  uint8_t readBitMask(uint8_t data, uint8_t mask);

  // write the bits from message in data given a bit mask
  uint8_t writeBitMask(uint8_t data, uint8_t mask, uint8_t message);

  // reads or writes to a register. returns 0xFF if there were any errors
  // returns 0 if it was write operation. else returns read content
  uint8_t readWriteReg(uint8_t address, uint8_t data);

  // updates the given status register
  void updateStatus(uint8_t address);

  // performs self tests on the device
  void selfTest();

public:
  // Constructor with configurable chip select pin
  AISx120SX(uint8_t CS);

  // Destructor
  ~AISx120SX();

  // verifies there were no boot errors and sets up the device
  bool setup(bandwidth bandwidthX, bandwidth bandwidthY,
             bool x_offset_monitor, bool x_offset_canc,
             bool y_offset_monitor, bool y_offset_canc);

  // perform a soft reset of the device
  bool reset();

  // reads the acceleration values from the sensor
  int16_t *readAccel();
};