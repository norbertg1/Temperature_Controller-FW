/*
Library:				STM32F40x Internal FLASH read/write
Written by:			Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Last modified:	15/03/2019
Description:
							MY_FLASH library implements the following basic functionalities
								- Set sectos address
								- Flash Sector Erase
								- Flash Write
								- Flash Read
								
* Copyright (C) 2019 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.								
*/

#include "stm32f3xx_hal.h"

//Typedefs
//1. data size
typedef enum
{
	DATA_TYPE_16=0,
	DATA_TYPE_32,
	DATA_TYPE_64,
}DataTypeDef;

//functions prototypes
//1. Erase Sector
static void flash_EraseSector(void);

//2. Set Sector Adress
void flash_SetSectorAddrs(uint8_t sector, uint32_t addrs);
//3. Write Flash
void flash_WriteN(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType);
//4. Read Flash
void flash_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType);
