// ---------------------------------------------------------------------------- 
// crc8
//
// Copyright (c) 2002 Colin O'Flynn
// Minor changes by M.Thomas 9/2004 
// ----------------------------------------------------------------------------

#ifndef __have_crc8_h__
#define __have_crc8_h__

#include <stdint.h>

// ----------------------------------------------------------------------------

#define CRC8INIT    0x00
#define CRC8POLY    0x18              //0X18 = X^8+X^5+X^4+X^0

// ----------------------------------------------------------------------------

extern uint8_t crc8(uint8_t *data, uint16_t data_length);

// ----------------------------------------------------------------------------

#endif // __have_crc8_h__
