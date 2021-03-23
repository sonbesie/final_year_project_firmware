/**
  ******************************************************************************
  * @file           : network.h
  * @brief          : The file running the network communication system
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

// todo: documentation

#ifndef INC_NETWORK_H_
#define INC_NETWORK_H_

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stm32f7xx_hal.h"

typedef struct
{
	uint8_t DestinMAC[6];
	uint8_t SourceMAC[6];
	uint8_t Rx_Buff[ETH_RX_BUF_SIZE];
	uint8_t Tx_Buff[ETH_TX_BUF_SIZE];
} COM_SettingsTypeDef;

typedef struct
{
	ETH_HandleTypeDef *Heth;
	ETH_DMADescTypeDef DMARxDscrTab;
	ETH_DMADescTypeDef DMATxDscrTab;
	COM_SettingsTypeDef Settings;

} COM_InitTypeDef;

typedef enum
{
	COM_OK		= 0x00U,
	COM_ERROR	= 0x01U
} COM_StatusTypeDef;

#define MAX_DATA_PAYLOAD 1514

COM_StatusTypeDef InitCom(COM_InitTypeDef *);
COM_StatusTypeDef SetDestinationMacAddress(COM_InitTypeDef *);
COM_StatusTypeDef TransmitData(COM_InitTypeDef *, uint8_t*, uint32_t, uint8_t*, uint32_t);

void TransmitDelay(void);

// HAL
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth);
void NetworkErrorHandler(COM_InitTypeDef *);

#endif /* INC_NETWORK_H_ */
