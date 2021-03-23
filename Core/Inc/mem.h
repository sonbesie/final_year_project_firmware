/**
  ******************************************************************************
  * @file           : mem.h
  * @brief          : The file running the cameras
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 & 2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#ifndef INC_MEM_H_
#define INC_MEM_H_

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f7xx_hal.h"

typedef struct
{
	// filesystem
	uint32_t CurrentChip;
	uint16_t FileSystemCounter;

	// io
	GPIO_TypeDef *MemoryCardBank;
	SPI_HandleTypeDef *Spi;
	uint16_t MemoryCardPin1, MemoryCardPin2;

	// status
	int StatusLength;
	uint8_t *Status;

	// data
	int DataLength;
	uint8_t *Data;
	RTC_DateTypeDef Date; 	// date
	RTC_TimeTypeDef Time;	// time

} MEM_InitTypeDef;

typedef enum
{
	  MEM_OK       = 0x00U,
	  MEM_ERROR    = 0x01U
} MEM_StatusTypeDef;

// memory elements
#define MANUFACTURER_ID 0xEFU
#define DEVICE_ID 0x15U
#define DEVICE_SIZE 0x400000U
#define SECTOR_SIZE 0x1000U

// memory card
#define NO_ELEMENTS 0x04U
#define CARD_SIZE 0xFFFFFFU

// status registers
#define STATUS_1 0x00U
#define STATUS_2 0x01U
#define STATUS_3 0x02U

// metadata
//#define METADATA_LENGTH 0x04U
#define METADATA_LENGTH 0x0CU

// level 1 functions: hardware
MEM_StatusTypeDef ReadManufacturerDevice(MEM_InitTypeDef *);
MEM_StatusTypeDef WriteEnable(MEM_InitTypeDef *);
MEM_StatusTypeDef ChipErase(MEM_InitTypeDef *);
MEM_StatusTypeDef PageProgram(MEM_InitTypeDef *, uint32_t, uint8_t *, uint16_t);
MEM_StatusTypeDef ReadData(MEM_InitTypeDef *, uint32_t, uint16_t);
MEM_StatusTypeDef ReadStatusRegister(MEM_InitTypeDef *, uint8_t);

// level 2 functions: card
MEM_StatusTypeDef InitMem(MEM_InitTypeDef *);
MEM_StatusTypeDef CardTest(MEM_InitTypeDef *);
MEM_StatusTypeDef CardRead(MEM_InitTypeDef *, uint32_t, uint16_t);
MEM_StatusTypeDef CardWrite(MEM_InitTypeDef *, uint32_t, uint8_t *, uint16_t);
MEM_StatusTypeDef CardErase(MEM_InitTypeDef *);

// level 3 functions: filesystem
MEM_StatusTypeDef StoreImage(MEM_InitTypeDef *, uint8_t *, uint16_t, uint16_t, RTC_DateTypeDef, RTC_TimeTypeDef);
MEM_StatusTypeDef RecalImage(MEM_InitTypeDef *, uint16_t);

// helper functions
void ChipSelect(MEM_InitTypeDef *);
void DriveChipSelectA(MEM_InitTypeDef *);
void DriveChipSelectB(MEM_InitTypeDef *);

#endif /* INC_MEM_H_ */
