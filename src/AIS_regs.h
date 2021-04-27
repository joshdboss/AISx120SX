/*
 * AIS_regs.h
 *
 *  Created on: 2021-04-27
 *      Author: Joshua Cayetano-Emond
 * 
 */

// File to define the register addresses and bits according to fig 16

// Define the accessible registers addresses
#define REG_CTRL_0              0x00
#define REG_CTRL_1              0x01
#define REG_CONFIG              0x02
#define REG_STATUS_0            0x03
#define REG_STATUS_1            0x04
#define REG_STATUS_2            0x05
#define REG_CHID_REVID          0x06
#define REG_ACC_CHX_LOW         0x07
#define REG_ACC_CHX_HIGH        0x08
#define REG_ACC_CHY_LOW         0x09
#define REG_ACC_CHY_HIGH        0x0A
#define REG_OSC_COUNTER         0x0B
#define REG_ID_SENSOR_TYPE      0x0C
#define REG_ID_VEH_MANUF        0x0D
#define REG_ID_SENSOR_MANUF     0x0E
#define REG_ID_LOT0             0x0F
#define REG_ID_LOT1             0x10
#define REG_ID_LOT2             0x11
#define REG_ID_LOT3             0x12
#define REG_ID_WAFER            0x13
#define REG_ID_COOR_X           0x14
#define REG_ID_COOR_Y           0x15
#define REG_RESET               0x16
#define OFF_CHX_HIGH            0x17
#define OFF_CHX_LOW             0x18
#define OFF_CHY_HIGH            0x19
#define OFF_CHY_LOW             0x1A

// Bit masks for REG_CTRL_0
#define END_OF_INIT             0b0000'0001

// Bit masks for REG_CTRL_1
#define SELF_TEST_CMD           0b0000'0111

// Bit masks for REG_CONFIG
#define FIR_BW_SEL_CHY          0b1100'0000
#define FIR_BW_SEL_CHX          0b0011'0000
#define DIS_OFF_MON_CHY         0b0000'1000
#define DIS_OFF_MON_CHX         0b0000'0100
#define DIS_OFF_CANC_CHY        0b0000'0010
#define DIS_OFF_CANC_CHX        0b0000'0001

// Bit masks for REG_STATUS_0
#define STATUS                  0b1100'0000
#define TESTMODE_ENABLED        0b0010'0000
#define REG_CTRL_0_WR_ERR_LATCH 0b0001'0000
#define LOSSCAP_ERR_LATCH       0b0000'0100
#define END_OF_PWRUP_LATCH      0b0000'0010
#define RST_ACTIVE_LATCH        0b0000'0001

// Bit masks for REG_STATUS_1
#define SPI_ERR                 0b1000'0000
#define EEPROM_ERR_LATCH        0b0100'0000
#define OFF_CANC_CHY_ERR        0b0000'1000
#define OFF_CANC_CHX_ERR        0b0000'0100
#define REG_CONFIG_WR_ERR_LATCH 0b0000'0010
#define REG_CTRL_1_WR_ERR_LATCH 0b0000'0001

// Bit masks for REG_STATUS_2
#define A2D_SAT_CHY_ERR         0b1000'0000
#define A2D_SAT_CHX_ERR         0b0100'0000
#define CHARGE_PUMP_ERR_LATCH   0b0001'0000
#define VREG_LOW_ERR            0b0000'1000
#define VREG_HIGH_ERR           0b0000'0100
#define VDD_LOW_ERR             0b0000'0010
#define VDD_HIGH_ERR            0b0000'0001

// Bit masks for REG_CHID_REVID
#define CHY_ACTIVE              0b0010'0000
#define CHX_ACTIVE              0b0001'0000
#define REVID                   0b0000'0111

// Bit masks for REG_RESET
#define SOFT_RST                0b0000'0011

// other register bit masks were not included as they are not used in the code
// if required, they can be found from fig 16 in the manual and defined similar
// to the above bit masks