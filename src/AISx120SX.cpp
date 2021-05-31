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
  pinMode(_CS, OUTPUT); // Set CS pin to be an output
  SPISettings AISSettings(5000000, MSBFIRST, SPI_MODE3);
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

int AISx120SX::calculateEvenParity(uint8_t *payload, int size)
{
  uint8_t r = 0;
  int i;
  for (i = 0; i < size; i++)
  {
    r ^= payload[i];
  }
  return calcEvenParityBit(r, 8);
}

bool AISx120SX::setup(bandwidth bandwidthX, bandwidth bandwidthY,
                      bool x_offset_monitor, bool x_offset_canc,
                      bool y_offset_monitor, bool y_offset_canc)

{
  reset();
  uint8_t command = 0; // empty command for later use

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
      command = writeBitMask(command, FIR_BW_SEL_CHY, bandwidthY); // y axis
      command = writeBitMask(command, FIR_BW_SEL_CHX, bandwidthX); // x axis
      // set the offset monitor
      command = writeBitMask(command, DIS_OFF_MON_CHY, !y_offset_monitor);
      command = writeBitMask(command, DIS_OFF_MON_CHX, !x_offset_monitor);
      // set the offset cancellation
      command = writeBitMask(command, DIS_OFF_CANC_CHY, !y_offset_canc);
      command = writeBitMask(command, DIS_OFF_CANC_CHX, !x_offset_canc);
      readWriteReg(REG_CONFIG, command);

      // find the sensor type
      uint8_t sensorTypeRead = readWriteReg(REG_ID_SENSOR_TYPE, 0xFF);
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

      selfTest();
      // start acquisition by setting END_OF_INIT flag to 1
      readWriteReg(REG_CTRL_0, 1U);
      return true;
    }
  }
  return false;
}

bool AISx120SX::reset()
{
  readWriteReg(REG_RESET, 0);
  delay(500);
  readWriteReg(REG_RESET, 2U);
  delay(500);
  readWriteReg(REG_RESET, 1U);
  delay(500);
  readWriteReg(REG_RESET, 2U);
  delay(500);
  readWriteReg(REG_RESET, 0);
  delay(500);
  updateStatus(REG_STATUS_0);
  return rst_active;
}

uint8_t AISx120SX::readBitMask(uint8_t data, uint8_t mask)
{
  uint8_t maskedData = data & mask; // mask the data

  // figure out how many bits to shift the data
  // correspondings to trailing zeros in the mask
  uint8_t trailingZeroCount = 0;
  while (mask != 0)
  {
    if ((mask & 1) == 1)
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

uint8_t AISx120SX::writeBitMask(uint8_t data, uint8_t mask, uint8_t message)
{
  data = data & ~mask; // set bits in the mask equal to zero

  // figure out how many bits to shift the data
  // correspondings to trailing zeros in the mask
  uint8_t trailingZeroCount = 0;
  while (mask != 0)
  {
    if ((mask & 1) == 1)
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
    return 0xFF;
  }
  else
  {
    message = message << trailingZeroCount; // align message with mask
    return data | message;                  // insert message into data
  }
}

// reads from a register
uint8_t AISx120SX::readWriteReg(uint8_t address, uint8_t data)
{
  // get the operation mode
  uint8_t operationMode = 3U; // binary 11
  if (data != 0xFF)
  {
    operationMode = 1U; // binary 01
  }

  // create the command to send to the device
  uint8_t command[3] = {0};                       // empty command
  command[0] = command[0] | (operationMode << 6); // set op mode
  // sensor mode is zero so not set
  command[0] = command[0] | (address >> 3); // first two bits address
  command[1] = command[1] | (address << 5); // last three bits address
  command[1] = command[1] | (data >> 3);    // first 5 bits of data
  command[2] = command[2] | (data << 5);    // last 3 bits of data

  if (!calculateEvenParity(command, sizeof(command))) // if even
  {
    bitSet(command[0], 4); // set parity to odd
  }

  // calculate and set the CRC
  uint8_t crc_out = Fast_CRC_Cal8Bits(0, sizeof(command), command);

  // try the request multiple times since SPI bus errors do happen
  for (size_t i = 0; i < 5; i++)
  {
    // send the request to the device
    // start SPI communication with the device
    SPI.beginTransaction(AISSettings);
    digitalWrite(_CS, LOW);

    // send the command
    SPI.transfer(command[0]);
    SPI.transfer(command[1]);
    SPI.transfer(command[2]);
    SPI.transfer(crc_out);

    // end SPI communication
    digitalWrite(_CS, HIGH);
    SPI.endTransaction();

    // delay to allow for the chip select to be set properly
    // interrupt-safe, though the thread is occupied in the loop for the
    // delay duration. as it is very short, should not be a big problem
    uint32_t delayPrevCheck = micros();
    uint32_t delayDuration = 5;
    while (micros() - delayPrevCheck < delayDuration)
      ; // do nothing while waiting

    uint8_t buffer[3]; // buffer where to store the reading
    uint8_t crc_in;

    // receive the request to the device
    // start SPI communication with the device
    SPI.beginTransaction(AISSettings);
    digitalWrite(_CS, LOW);

    // receive the reading
    buffer[0] = SPI.transfer(0);
    buffer[1] = SPI.transfer(0);
    buffer[2] = SPI.transfer(0);
    crc_in = SPI.transfer(0);

    // end SPi communication
    digitalWrite(_CS, HIGH);
    SPI.endTransaction();

    // if CRCs match and there are no error flags
    if (Fast_CRC_Cal8Bits(0, sizeof(buffer), buffer) == crc_in &&
        (buffer[2] & 0x0F) == 0)
    {
      uint8_t output = {0};
      output = buffer[1] << 3;            // first 5 bits of data
      output = output | (buffer[2] >> 5); // last 3 bits of data

      return output;
    }
  }
  return 0xFF;
}

// reads the acceleration values from the sensor
int16_t *AISx120SX::readAccel()
{
  static int16_t accel[2] = {0}; // initialize acceleration array

  for (size_t i = 0; i < axisNumber; i++) // iterate through the available axes
  {
    // create the command to send to the device
    uint8_t command[3] = {0};            // empty command
    command[0] = command[0] | (i << 6);  // channel
    command[0] = command[0] | (1U << 5); // sensor sata mode

    if (!calculateEvenParity(command, sizeof(command))) // if even
    {
      bitSet(command[0], 4); // set parity to odd
    }

    // calculate and set the CRC
    uint8_t crc_out = Fast_CRC_Cal8Bits(0, sizeof(command),
                                        command);

    // send the request to the device
    // start SPI communication with the device
    SPI.beginTransaction(AISSettings);
    digitalWrite(_CS, LOW);

    // send the command
    SPI.transfer(command[0]);
    SPI.transfer(command[1]);
    SPI.transfer(command[2]);
    SPI.transfer(crc_out);

    // end SPI communication
    digitalWrite(_CS, HIGH);
    SPI.endTransaction();

    // delay to allow for the chip select to be set properly
    // interrupt-safe, though the thread is occupied in the loop for the
    // delay duration. as it is very short, should not be a big problem
    uint32_t delayPrevCheck = micros();
    uint32_t delayDuration = 5;
    while (micros() - delayPrevCheck < delayDuration)
      ; // do nothing while waiting

    uint8_t buffer[3]; // buffer where to store the reading

    // receive the request to the device
    // start SPI communication with the device
    SPI.beginTransaction(AISSettings);
    digitalWrite(_CS, LOW);

    // receive the reading
    buffer[0] = SPI.transfer(0);
    buffer[1] = SPI.transfer(0);
    buffer[2] = SPI.transfer(0);
    uint8_t crc_in = SPI.transfer(0);

    // end SPi communication
    digitalWrite(_CS, HIGH);
    SPI.endTransaction();

    // if CRCs match and there are no error flags
    if (Fast_CRC_Cal8Bits(0, sizeof(buffer), buffer) == crc_in &&
        (buffer[2] & 0x0F) == 0)
    {
      // get acceleration
      accel[i] = ((uint16_t)buffer[0] & 0x03) << 12;      // first 2 bits acc
      accel[i] = accel[i] | (((uint16_t)buffer[1]) << 4); // central 8 bits acc
      accel[i] = accel[i] | ((uint16_t)buffer[2] >> 4);   // last 4 bits acc
      // left shift 2 bits to get align 14 bits on 16
      // Note that putting the acceleration as a 16-bit integer
      // effectively means that the magnitude of the acceleration is 4 times
      // larger and should be normalized by a quarter of the resolution.
      // get the acceleration
      accel[i] = accel[i] << 2;
    }
    else
    {
      accel[i] = 0xFFFF;
    }
  }
  return accel;
}

// updates the given status register
void AISx120SX::updateStatus(uint8_t address)
{
  uint8_t reading = readWriteReg(address, 0xFF);
  switch (address)
  {
  case REG_STATUS_0:
    status = static_cast<statusTypes>(readBitMask(reading, STATUS));
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
    off_canc_chx_err = readBitMask(reading, OFF_CANC_CHX_ERR);
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

  // x-axis
  readWriteReg(REG_CTRL_1, 0x00); // zero self test
  delay(250); // give the sensor time to react

  readWriteReg(REG_CTRL_1, 0x01); // positive self test
  delay(250); // give the sensor time to react

  readWriteReg(REG_CTRL_1, 0x00); // zero self test
  delay(250); // give the sensor time to react

  readWriteReg(REG_CTRL_1, 0x02); // negative self test
  delay(250); // give the sensor time to react

  readWriteReg(REG_CTRL_1, 0x00); // zero self test
  delay(250); // give the sensor time to react

  // y-axis
  if (axisNumber == 2)
  {
    readWriteReg(REG_CTRL_1, 0x00); // zero self test
    delay(250); // give the sensor time to react

    readWriteReg(REG_CTRL_1, 0x05); // positive self test
    delay(250); // give the sensor time to react

    readWriteReg(REG_CTRL_1, 0x00); // zero self test
    delay(250); // give the sensor time to react

    readWriteReg(REG_CTRL_1, 0x06); // negative self test
    delay(250); // give the sensor time to react

    readWriteReg(REG_CTRL_1, 0x00); // zero self test
    delay(250); // give the sensor time to react
  }
}