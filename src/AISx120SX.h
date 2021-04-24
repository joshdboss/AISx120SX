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

// AISx120SX class definition
class AISx120SX
{
private:

  // Chip select pin assignment
  int _CS;

public:
  // Constructor with configurable chip select pin

  AISx120SX(int CS);

  // Destructor
  ~AISx120SX();
};