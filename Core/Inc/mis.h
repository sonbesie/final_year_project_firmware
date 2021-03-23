/**
  ******************************************************************************
  * @file           : mis.h
  * @brief          : This file contains general functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#ifndef INC_MIS_H_
#define INC_MIS_H_

#include <stdlib.h>
#include <math.h>
#include "stm32f7xx_hal.h"

#define DEFAULT_SENSITIVITY 0xFFU;

typedef struct
{
	DAC_HandleTypeDef *Dacx;
	uint8_t Status, Sensitivity, Armed;
} MIS_InitTypeDef;

typedef enum
{
	  MIS_OK       = 0x00U,
	  MIS_ERROR    = 0x01U
} MIS_StatusTypeDef;

MIS_StatusTypeDef DecodeSevenSegment(MIS_InitTypeDef* );
MIS_StatusTypeDef InitialiseTrigger(MIS_InitTypeDef* );
MIS_StatusTypeDef SetTriggerSensitivity(MIS_InitTypeDef* );
MIS_StatusTypeDef SetTriggerArmedStatus(MIS_InitTypeDef* );

#endif /* INC_MIS_H_ */
