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


// AISx120SX class definition
class AISx120SX
{
private:
  // Chip select pin assignment
  int _CS;

  // 

  // Find parity (of any width up to the width of an unsigned)
  int calcEvenParityBit(unsigned par, unsigned width);

  // finds if a given payload has even parity
  int calculateEvenParity(char *payload, int size);
  
  // reads the bits in data given a bit mask
  uint8_t readBitMask(uint8_t data, uint8_t mask);

  // write the bits in data given a bit mask and a full width message
  uint8_t writeBitMask(uint8_t data, uint8_t mask, uint8_t message);
  
  // reads from a register
  uint8_t readReg(uint8_t address);

  // writes to a register. returns 0 if there were any errors
  bool writeReg(uint8_t address, uint8_t message);
  
  // updates the given status register
  uint8_t updateStatus(uint8_t address);
  
  // performs self tests on the device
  void selfTest();

public:
  // Constructor with configurable chip select pin
  AISx120SX(int CS);

  // Destructor
  ~AISx120SX();

  // verifies there were no boot errors and sets up the device
  bool setup();

  // reads the acceleration values from the sensor
  int16_t *readAccel();

};