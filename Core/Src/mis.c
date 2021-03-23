/**
  ******************************************************************************
  * @file           : mis.c
  * @brief          : This file contains general functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */


#include "mis.h"

MIS_StatusTypeDef DecodeSevenSegment(MIS_InitTypeDef* Misx)
{
	// set the enable pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

	switch (Misx->Status)
	{
		case  0:	// 0
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  1: // 1
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  2: // 2
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  3: // 3
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  4: // 4
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  5:  // 5
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  6: // 6
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  7: // 7
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_RESET);	// D
			break;
		}
		case  8: // 8
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
		case  9: // 9
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
		case 10: // extra 1
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
		case 11: // extra 2
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
		case 12: // extra 3
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
		case 13: // extra 4
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
		case 14: // extra 5
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
		case 15: // blank
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET	);	// A
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET	);	// B
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET	);	// C
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 , GPIO_PIN_SET	);	// D
			break;
		}
	}

	return MIS_OK;
}

MIS_StatusTypeDef InitialiseTrigger(MIS_InitTypeDef* Misx)
{
	// reset the comparator trigger
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 , GPIO_PIN_RESET);

	// reset the interrupt
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	// set the sensitivity to default
	Misx->Sensitivity = DEFAULT_SENSITIVITY;

	if (HAL_DAC_Start(Misx->Dacx, DAC_CHANNEL_1) != HAL_OK)
	{
		return MIS_ERROR;
	}

	if (HAL_DAC_SetValue(Misx->Dacx, DAC_CHANNEL_1, DAC_ALIGN_8B_R, Misx->Sensitivity) != HAL_OK)
	{
		return MIS_ERROR;
	}

	return MIS_OK;
}

MIS_StatusTypeDef SetTriggerSensitivity(MIS_InitTypeDef* Misx)
{
	// change the sensitivity scale
	Misx->Sensitivity = Misx->Sensitivity * 10;

	if (HAL_DAC_SetValue(Misx->Dacx, DAC_CHANNEL_1, DAC_ALIGN_8B_R, Misx->Sensitivity) != HAL_OK)
	{
		return MIS_ERROR;
	}

	return MIS_OK;
}

MIS_StatusTypeDef SetTriggerArmedStatus(MIS_InitTypeDef* Misx)
{
	if (Misx->Armed == 1)
	{
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 , GPIO_PIN_SET  );
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 , GPIO_PIN_RESET);
	}
	return MIS_OK;
}
