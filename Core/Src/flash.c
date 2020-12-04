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

#include "flash.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_110   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_111   /* End @ of user Flash area */

//Private variables
//1. sector start address
static uint32_t MY_SectorAddrs;

uint32_t PageError = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;


//functions definitions
//1. Erase Sector
static void flash_EraseSector(void)
{
	HAL_StatusTypeDef ret;

	HAL_FLASH_Unlock();
	//Erase the required Flash sector
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	ret = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock();
}


//3. Write Flash
void flash_WriteN(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = FLASH_USER_START_ADDR + idx;
	HAL_StatusTypeDef ret;

	//Erase sector before write
	flash_EraseSector();

	//Unlock Flash
	ret = HAL_FLASH_Unlock();
	//Write to Flash
	switch(dataType)
	{
		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flashAddress , ((uint16_t *)wrBuf)[i]);
					flashAddress++;
				}
				if(ret != HAL_OK){
					ret = HAL_FLASH_GetError();
				}
			break;

		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress , ((uint32_t *)wrBuf)[i]);
					flashAddress+=2;
				}
				if(ret != HAL_OK){
					ret = HAL_FLASH_GetError();
				}
			break;

		case DATA_TYPE_64:
				for(uint32_t i=0; i<Nsize; i++)
				{
					ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flashAddress , ((uint64_t *)wrBuf)[i]);
					flashAddress+=4;
				}
				if(ret != HAL_OK){
					ret = HAL_FLASH_GetError();
				}
			break;
	}

	//Lock the Flash space
	ret = HAL_FLASH_Lock();

}
//4. Read Flash
void flash_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = FLASH_USER_START_ADDR + idx;

	switch(dataType)
	{
		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint16_t *)rdBuf + i) = *(uint16_t *)flashAddress;
					flashAddress++;
				}
			break;

		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint32_t *)rdBuf + i) = *(uint32_t *)flashAddress;
					flashAddress+=2;
				}
			break;

		case DATA_TYPE_64:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint64_t *)rdBuf + i) = *(uint64_t *)flashAddress;
					flashAddress+=4;
				}
			break;
	}

}
