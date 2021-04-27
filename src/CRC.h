// 8-bit CRC Model - sourcer32@gmail.com
//  Copyright (C) 1988-2020, All Rights Reserved

#define POLY 0x97 // # 0x197 = x^8 + x^7 + x^4 + x^2 + x^1 +1  (0x97 -> 0x197)

unsigned char Slow_CRC_Cal8Bits(unsigned char crc, int Size, unsigned char *Buffer);
unsigned char Quick_CRC_Cal8Bits(unsigned char crc, int Size, unsigned char *Buffer);
unsigned char Fast_CRC_Cal8Bits(unsigned char crc, int Size, unsigned char *Buffer);