/*
 * AISx120SX.cpp
 *
 *  Created on: 2021-04-24
 *      Author: Joshua Cayetano-Emond
 */

#include "AISx120SX.h"

AISx120SX::AISx120SX(uint8_t CS)
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

bool AISx120SX::setup(bandwidth bandwidthX, bandwidth bandwidthY)
{
  uint8_t command; // empty command for later use

  updateStatus(REG_STATUS_0);
  updateStatus(REG_STATUS_1);
  updateStatus(REG_STATUS_2);

  // check if power up is complete
  if (end_of_pwrup)
  {
    // check that there were no errors
    if (!loss_cap && !charge_pump_err && !vreg_low_volt_det &&
        !vreg_high_volt_det && !vdd_low_volt_det && !vdd_high_volt_det)
    {
      // set the acquistion rate
      writeBitMask(command, FIR_BW_SEL_CHY, bandwidthY); // y axis bandwidth
      writeBitMask(command, FIR_BW_SEL_CHX, bandwidthX); // x axis bandwidth
      writeReg(uint8_t REG_CTRL_1, uint8_t command);

      // find the sensor type
      uint8_t sensorTypeRead = readWriteReg(REG_ID_SENSOR_TYPE);
      if (sensorTypeRead == 0x1A) // single axis sensor
      {
        axisNumber = 1;
      }
      else if (sensorTypeRead == 0x2A) // dual axis sensor
      {
        axisNumber = 2;
      }
      else // error
      {
        return false;
      }

      selfTest(); // perform a self test

      // start acquisition by setting END_OF_INIT flag to 1
      readWriteReg(REG_CTRL_0, 1U);
      return true;
    }
  }
  return false;
}

bool reset()
{
  readWriteReg(SOFT_RST, 2U);
  delay(500);
  readWriteReg(SOFT_RST, 1U);
  delay(500);
  readWriteReg(SOFT_RST, 2U);
  delay(500);
  updateStatus(REG_STATUS_0);
  return rst_active;
}

uint32_t AISx120SX::readBitMask(uint32_t data, uint32_t mask)
{
  uint32_t maskedData = data & mask; // mask the data

  // figure out how many bits to shift the data
  // correspondings to trailing zeros in the mask
  trailingZeroCount = 0;
  while (mask != 0)
  {
    if (mask & 1 == 1)
    { // reached the first set bit
      break;
    }
    else
    {
      trailingZeroCount++;
      mask = mask >> 1;
    }
  }

  return maskedData >> trailingZeroCount; // shift data all the way right
}

bool AISx120SX::writeBitMask(uint32_t &data, uint32_t mask, uint32_t message)
{
  uint32_t tempData = data;    // don't work on the original data yet
  tempData = tempData & ~mask; // set bits in the mask equal to zero

  // figure out how many bits to shift the data
  // correspondings to trailing zeros in the mask
  trailingZeroCount = 0;
  while (mask != 0)
  {
    if (mask & 1 == 1)
    { // reached the first set bit
      break;
    }
    else
    {
      trailingZeroCount++;
      mask = mask >> 1;
    }
  }

  if (message > mask) // the message is wider than the mask
  {
    return false;
  }
  else
  {
    message = message << trailingZeroCount; // align message with mask
    data = tempData | message;              // insert message into data
    return true;
  }
}

// reads from a register
uint8_t AISx120SX::readWriteReg(uint8_t address, uint8_t data = 0)
{
  // get the operation mode
  uint8_t operationMode = 3U; // binary 11
  if (Write)
  {
    operationMode = 1U; // binary 01
  }

  // create the command to send to the device
  uint32_t command;                                          // empty command
  writeBitMask(command, NON_ACC_SDI_OP_CODE, operationMode); // set op mode
  writeBitMask(command, NON_ACC_SDI_ADDRESS, address);       // set address
  writeBitMask(command, NON_ACC_SDI_DATA, data);             // set data

  if (calculateEvenParity(command, sizeof(command))) // if even
  {
    writeBitMask(command, NON_ACC_SDI_PARITY, 1U); // set parity to odd
  }

  // calculate and set the CRC
  unsigned char crc = Fast_CRC_Cal8Bits(0, sizeof(command), command);
  writeBitMask(command, NON_ACC_SDI_CRC, crc);

  // send the request to the device
  // start SPI communication with the device
  SPI.beginTransaction(AISSettings);
  digitalWrite(_CS, LOW);

  SPI.transfer(command, sizeof(command)); // transfer the command

  // end SPI communication
  digitalWrite(_CS, HIGH);
  SPI.endTransaction();

  // uncomment this delay if getting errors
  // should not be necessary as digitalWrite is slow enough to give time
  // to the device to process the request
  // delayMicroseconds(10); // wait a bit before reading

  uint32_t buffer; // buffer where to store the reading

  // receive the request to the device
  // start SPI communication with the device
  SPI.beginTransaction(AISSettings);
  digitalWrite(_CS, LOW);

  SPI.transfer(buffer, sizeof(buffer)); // receive the reading

  // end SPi communication
  digitalWrite(_CS, HIGH);
  SPI.endTransaction();

  // verify the CRC
  crc = readBitMask(buffer, NON_ACC_SDO_CRC); // get the CRC
  writeBitMask(buffer, NON_ACC_SDO_CRC, 0);   // clear the crc from the buffer

  // if CRCs match and there are no error flags
  if (crc == Fast_CRC_Cal8Bits(0, sizeof(buffer), buffer) &&
      (readBitMask(buffer, NON_ACC_SDO_ERROR)) == 0)
  {
    return readBitMask(buffer, NON_ACC_SDO_DATA); // will be zero for a write
  }
  else
  {
    return 0xFF;
  }
}

// reads the acceleration values from the sensor
int16_t *AISx120SX::readAccel()
{
  int16_t accel[2] = {0}; // initialize acceleration array

  for (size_t i = 0; i < axisNumber; i++) // iterate through the available axes
  {
    // create the command to send to the device
    uint32_t command;                            // empty command
    writeBitMask(command, ACC_SDI_CH_SELECT, i); // set channel
    writeBitMask(command, ACC_SDI_SEN, 1U);      // set sensor data

    if (calculateEvenParity(command, sizeof(command))) // if even
    {
      writeBitMask(command, ACC_SDI_PARITY, 1U); // set parity to odd
    }

    // calculate and set the CRC
    unsigned char crc = Fast_CRC_Cal8Bits(0, sizeof(command), command);
    writeBitMask(command, ACC_SDI_CRC, crc);

    // send the request to the device
    // start SPI communication with the device
    SPI.beginTransaction(AISSettings);
    digitalWrite(_CS, LOW);

    SPI.transfer(command, sizeof(command)); // transfer the command

    // end SPI communication
    digitalWrite(_CS, HIGH);
    SPI.endTransaction();

    // uncomment this delay if getting errors
    // should not be necessary as digitalWrite is slow enough to give time
    // to the device to process the request
    // delayMicroseconds(10); // wait a bit before reading

    uint32_t buffer; // buffer where to store the reading

    // receive the request to the device
    // start SPI communication with the device
    SPI.beginTransaction(AISSettings);
    digitalWrite(_CS, LOW);

    SPI.transfer(buffer, sizeof(buffer)); // receive the reading

    // end SPi communication
    digitalWrite(_CS, HIGH);
    SPI.endTransaction();

    // verify the CRC
    crc = readBitMask(buffer, ACC_SDO_CRC); // get the CRC
    writeBitMask(buffer, ACC_SDO_CRC, 0);   // clear the crc from the buffer

    // if CRCs match and there are no error flags
    if (crc == Fast_CRC_Cal8Bits(0, sizeof(buffer), buffer) &&
        (readBitMask(buffer, ACC_SDO_ERROR)) == 0)
    {
      // get acceleration and left shift 2 bits to get align 14 bits on 16
      // Note that putting the acceleration as a 16-bit integer
      // effectively means that the magnitude of the acceleration is 4 times
      // larger and should be normalized by a quarter of the resolution.
      accel[i] = readBitMask(buffer, ACC_SDO_DATA) << 2;
    }
    else
    {
      accel[i] = 0xFFFF;
    }
  }
  return accel;
}

// updates the given status register
uint8_t AISx120SX::updateStatus(uint8_t address)
{
  uint8_t reading = readWriteReg(address);
  switch (address)
  {
  case REG_STATUS_0:
    status = readBitMask(reading, STATUS);
    testmode_enabled = readBitMask(reading, TESTMODE_ENABLED);
    reg_ctrl_0_wr_err = readBitMask(reading, REG_CTRL_0_WR_ERR_LATCH);
    loss_cap = readBitMask(reading, LOSSCAP_ERR_LATCH);
    end_of_pwrup = readBitMask(reading, END_OF_PWRUP_LATCH);
    rst_active = readBitMask(reading, RST_ACTIVE_LATCH);
    break;

  case REG_STATUS_1:
    spi_err = readBitMask(reading, SPI_ERR);
    eeprom_err = readBitMask(reading, EEPROM_ERR_LATCH);
    off_canc_chy_err = readBitMask(reading, OFF_CANC_CHY_ERR);
    off_canc_chx_err = readBitMask(reading, OFF_CANC_CHx_ERR);
    reg_config_wr_err = readBitMask(reading, REG_CONFIG_WR_ERR_LATCH);
    reg_ctrl_1_wr_err = readBitMask(reading, REG_CTRL_1_WR_ERR_LATCH);
    break;

  case REG_STATUS_2:
    a2d_sat_chy = readBitMask(reading, A2D_SAT_CHY_ERR);
    a2d_sat_chx = readBitMask(reading, A2D_SAT_CHX_ERR);
    charge_pump_err = readBitMask(reading, CHARGE_PUMP_ERR_LATCH);
    vreg_low_volt_det = readBitMask(reading, VREG_LOW_ERR);
    vreg_high_volt_det = readBitMask(reading, VREG_HIGH_ERR);
    vdd_low_volt_det = readBitMask(reading, VDD_LOW_ERR);
    vdd_high_volt_det = readBitMask(reading, VDD_HIGH_ERR);
    break;
  }
}

// performs self tests on the device
void AISx120SX::selfTest()
{
  uint8_t command; // empty command

  // x-axis
  writeBitMask(command, SELF_TEST_CMD, 1U); // x axis positive voltage
  readWriteReg(REG_CTRL1, command);
  delay(250);                              // give the sensor time to react
  writeBitMask(command, SELF_TEST_CMD, 0); // 0g
  readWriteReg(REG_CTRL1, command);
  delay(250);                               // give the sensor time to react
  writeBitMask(command, SELF_TEST_CMD, 2U); // x axis negative voltage
  readWriteReg(REG_CTRL1, command);
  delay(250);                              // give the sensor time to react
  writeBitMask(command, SELF_TEST_CMD, 0); // 0g
  readWriteReg(REG_CTRL1, command);
  delay(250); // give the sensor time to react

  // y-axis
  if (axisCount == 2)
  {
    writeBitMask(command, SELF_TEST_CMD, 5U); // y axis positive voltage
    readWriteReg(REG_CTRL1, command);
    delay(250);                              // give the sensor time to react
    writeBitMask(command, SELF_TEST_CMD, 0); // 0g
    readWriteReg(REG_CTRL1, command);
    delay(250);                               // give the sensor time to react
    writeBitMask(command, SELF_TEST_CMD, 6U); // y axis negative voltage
    readWriteReg(REG_CTRL1, command);
    delay(250);                              // give the sensor time to react
    writeBitMask(command, SELF_TEST_CMD, 0); // 0g
    readWriteReg(REG_CTRL1, command);
    delay(250); // give the sensor time to react
  }
}