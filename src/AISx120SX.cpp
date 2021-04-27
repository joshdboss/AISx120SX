/*
 * AISx120SX.cpp
 *
 *  Created on: 2021-04-24
 *      Author: Joshua Cayetano-Emond
 */

#include "AISx120SX.h"

AISx120SX::AISx120SX(int CS)
{
  _CS = CS;
}

// Empty and unused destructor
AISx120SX::~AISx120SX()
{
}

// Find parity (of any width up to the width of an unsigned)
int AISx120SX::calcEvenParityBit(unsigned par, unsigned width)
{
  while (width > 1)
  {
    par ^= par >> (width / 2);
    width -= width / 2;
  }

  // Only return Least Significant Bit
  return par % 2;
}

int AISx120SX::calculateEvenParity(char *payload, int size)
{
  unsigned char r = 0;
  int i;
  for (i = 0; i < size; i++)
  {
    r ^= payload[i];
  }
  return calcEvenParityBit(r, CHAR_BIT);
}

bool AISx120SX::setup(){
    // check power is done
    // check registers for errors
    // setup the acquisition rate
    // perform a self test
    // start acquisition by setting END_OF_INIT flag to 1
    writeReg(uint8_t address, uint8_t message)}

// reads the acceleration values from the sensor
int16_t *AISx120SX::readAccel()
{
}

uint8_t AISx120SX::readBitMask(uint8_t data, uint8_t mask)
{
  //( x & ~(x-1) ) // returns the lowest set bit of the mask to know how many bits to shift
}

// reads from a register
uint8_t AISx120SX::readReg(uint8_t address)
{
  
}

// writes to a register. returns 0 if there were any errors
bool AISx120SX::writeReg(uint8_t address, uint8_t message)
{
}

// updates the given status register
uint8_t AISx120SX::updateStatus(uint8_t address)
{
}

// performs self tests on the device
void AISx120SX::selfTest()
{
}