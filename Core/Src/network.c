/**
  ******************************************************************************
  * @file           : network.c
  * @brief          : The file running the network communication system
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Pieter Schoeman.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  */

#include "network.h"

COM_StatusTypeDef InitCom(COM_InitTypeDef *Comx)
{
	// set the source and destination address of the ethernet frame
	// set the ethernet frame type
	if (SetDestinationMacAddress(Comx) != COM_OK)
	{
		NetworkErrorHandler(Comx);
		return COM_ERROR;
	}

	// initialise the HAL driver transmit buffers
	if (HAL_ETH_DMATxDescListInit(Comx->Heth, &Comx->DMATxDscrTab, &Comx->Settings.Tx_Buff[0], 1) != HAL_OK)
	{
		NetworkErrorHandler(Comx);
		return COM_ERROR;
	}

	// initialise the HAL driver receive buffers
	if (HAL_ETH_DMARxDescListInit(Comx->Heth, &Comx->DMARxDscrTab, &Comx->Settings.Rx_Buff[0], 1) != HAL_OK)
	{
		NetworkErrorHandler(Comx);
		return COM_ERROR;
	}

	// start the transmission / reception process, and return result
	if (HAL_ETH_Start(Comx->Heth) != HAL_OK)
	{
		NetworkErrorHandler(Comx);
		return COM_ERROR;
	}
	else
	{
		return COM_OK;
	}
}

COM_StatusTypeDef SetDestinationMacAddress(COM_InitTypeDef *Comx)
{
	// set the destination  MAC address
	for (int index = 0; index < 6; index++)
	{
		Comx->Settings.Tx_Buff[index+0] = Comx->Settings.DestinMAC[index];
	}

	// set the transmission MAC address
	for (int index = 0; index < 6; index++)
	{
		Comx->Settings.Tx_Buff[index+6] = Comx->Settings.SourceMAC[index];
	}

	// set transmission type
	Comx->Settings.Tx_Buff[12] = 0xEE;
	Comx->Settings.Tx_Buff[13] = 0xFA;

	// set the default preamble
	Comx->Settings.Tx_Buff[14] = 0x00;		// command 1
	Comx->Settings.Tx_Buff[15] = 0x00;		// command 2
	Comx->Settings.Tx_Buff[15] = 0x00;		// execution error code
	Comx->Settings.Tx_Buff[16] = 0x00;		// data packet number : 0 to 255
	Comx->Settings.Tx_Buff[17] = 0x00;		// total data packets : 0 to 255

	return COM_OK;
}

COM_StatusTypeDef TransmitData(COM_InitTypeDef *Comx, uint8_t* Preamble, uint32_t PreambleLength, uint8_t* Data, uint32_t DataLength)
{
    int Count = 0, Len = 0, Times = 0;
    Times = DataLength/(MAX_DATA_PAYLOAD-14-PreambleLength);

    // update the preamble
    Preamble[3] = 0;
    Preamble[4] = Times;

    while (Count <= Times)
    {
    	// save the number of the current loop
    	Preamble[3] = Count;

    	// calculate the current buffer length
        if (Count == Times)
        	Len = DataLength-(Count*(MAX_DATA_PAYLOAD-14-PreambleLength));
        else
        	Len = (MAX_DATA_PAYLOAD-14-PreambleLength);

        // fill the packet buffer
        // the first thirteen bytes are already filled
        memcpy(&(Comx->Settings.Tx_Buff[14]), Preamble, PreambleLength);
        memcpy(&(Comx->Settings.Tx_Buff[14 + PreambleLength]), &Data[Count*(MAX_DATA_PAYLOAD-14-PreambleLength)], Len);

        // wait for transmitter to be ready
        while (HAL_ETH_GetState(Comx->Heth) != HAL_ETH_STATE_READY) {};

    	// transmit data
        if (HAL_ETH_TransmitFrame(Comx->Heth, (Len+14+PreambleLength)) != HAL_OK)
        {
        	NetworkErrorHandler(Comx);
        	return COM_ERROR;
        }
        ++Count;

        // todo replace with acknowledge
        TransmitDelay();
    }
    return COM_OK;
}

void TransmitDelay(void)
{
	long delay = 0;
	while (delay < 10000)
	{
		++delay;
	}
}

void NetworkErrorHandler(COM_InitTypeDef *Comx)
{
	// TODO: implement log file
	while (1)
	{
		// ask for hardware reset
		break;
	}
}
