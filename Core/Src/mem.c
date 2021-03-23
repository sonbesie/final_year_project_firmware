/**
  ******************************************************************************
  * @file           : mem.c
  * @brief          : The file running the cameras
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 & 2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#include "mem.h"

// level 3 functions: filesystem

MEM_StatusTypeDef StoreImage(MEM_InitTypeDef *Memx, uint8_t *Data, uint16_t DataLength, uint16_t No, RTC_DateTypeDef Date, RTC_TimeTypeDef Time)
{
	// sanity check
	if ((DataLength > SECTOR_SIZE*4) || (1))
	{
		return MEM_ERROR;
	}

	// data variables
	uint32_t DataAddress = (No+2) * SECTOR_SIZE*4;

	// metadata variables
	uint8_t *MetaData;
	uint16_t MetaDataLength = METADATA_LENGTH;
	uint32_t MetaDataAddress = ((No+1) * METADATA_LENGTH);	// cant write to the first metadata address ??

	// read data at that address
	if ((CardRead(Memx, MetaDataAddress, MetaDataLength)) != MEM_OK)
	{
		return MEM_ERROR;
	}

	// ensure the data has been erased
	for (int index = 0; index < MetaDataLength; index++)
	{
		if (Memx->Data[index] != 0xFF)
		{
			return MEM_ERROR;
		}
	}

	// assign metadata
	MetaData = calloc(MetaDataLength , sizeof(uint8_t));
	// metadata: length
	MetaData[0 ] = (DataLength & 0b1111111100000000) >> 8;
	MetaData[1 ] = (DataLength & 0b0000000011111111) >> 0;
	// metadata: date
	MetaData[2 ] = ((Date.Year   /16)*10)+(Date.Year   %16);
	MetaData[3 ] = ((Date.Month  /16)*10)+(Date.Month  %16);
	MetaData[4 ] = ((Date.Date   /16)*10)+(Date.Date   %16);
	// metadata: time
	MetaData[5 ] = ((Time.Hours  /16)*10)+(Time.Hours  %16);
	MetaData[6 ] = ((Time.Minutes/16)*10)+(Time.Minutes%16);
	MetaData[7 ] = ((Time.Seconds/16)*10)+(Time.Seconds%16);
	// metadata: accurate time
	MetaData[8 ] = (Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
	MetaData[9 ] = (Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
	MetaData[10] = (Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
	MetaData[11] = (Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

	// write image metadata
	if (CardWrite(Memx, MetaDataAddress, MetaData, MetaDataLength) != MEM_OK)
	{
		free(MetaData);
		return MEM_ERROR;
	}

	// write image data
	if (CardWrite(Memx, DataAddress, Data, DataLength) != MEM_OK)
	{
		free(MetaData);
		return MEM_ERROR;
	}

	free(MetaData);
	return MEM_OK;
}

MEM_StatusTypeDef RecalImage(MEM_InitTypeDef *Memx, uint16_t No)
{
	// metadata variables
	uint16_t MetaDataLength = METADATA_LENGTH;
	uint32_t MetaDataAddress = ((No+1) * METADATA_LENGTH);

	// read data at that address
	if ((CardRead(Memx, MetaDataAddress, MetaDataLength)) != MEM_OK)
	{
		return MEM_ERROR;
	}

	// find image length
	uint16_t DataLength = ((Memx->Data[0]) << 8)|((Memx->Data[1]) << 0);
	uint32_t DataAddress = (No+2) * SECTOR_SIZE*4;

	// set date
	Memx->Date.Year 		= Memx->Data[2];
	Memx->Date.Month 		= Memx->Data[3];
	Memx->Date.Date 		= Memx->Data[4];

	// set time
	Memx->Time.Hours 		= Memx->Data[5];
	Memx->Time.Minutes 		= Memx->Data[6];
	Memx->Time.Seconds 		= Memx->Data[7];

	Memx->Time.SubSeconds 	= ((Memx->Data[8]) << 24)|((Memx->Data[9]) << 16)|((Memx->Data[10]) << 8)|((Memx->Data[11]) << 0);

	if ((CardRead(Memx, DataAddress, DataLength)) != MEM_OK)
	{
		return MEM_ERROR;
	}

	return MEM_OK;
}

// level 2 functions: card

MEM_StatusTypeDef InitMem(MEM_InitTypeDef *Memx)
{
	// select the first memory chip
	Memx->CurrentChip = 0;
	ChipSelect(Memx);

	// set data output to NULL
	Memx->Data = NULL;
	Memx->DataLength = 0;

	// set status output to NULL
	Memx->Status = NULL;
	Memx->StatusLength = 0;

	// initialise from configuration memory after reset
	// this is where the cameras will start to try and save new data
	Memx->FileSystemCounter = 0;

	// return ok
	return MEM_OK;
}

MEM_StatusTypeDef CardTest(MEM_InitTypeDef *Memx)
{
	for (int index = 0; index < NO_ELEMENTS; index++)
	{
		// select chip
		Memx->CurrentChip = index;

		// perform operation
		if (ReadManufacturerDevice(Memx) != MEM_OK)
		{
			return MEM_ERROR;
		}

		// analyse results
		if ((Memx->Data[0] != MANUFACTURER_ID) && (Memx->Data[1] != DEVICE_ID))
		{
			return MEM_ERROR;
		}
	}
	return MEM_OK;
}

MEM_StatusTypeDef CardRead(MEM_InitTypeDef *Memx, uint32_t Address, uint16_t Length)
{
	// sanity check the address and length of data to read
	// cannot read more data than is available on all four chips
	if ((Address+Length-1) > CARD_SIZE)
	{
		return MEM_ERROR;
	}

	uint8_t *Receive = NULL;
	uint16_t Len;
	int SubChip = 0, SubAddress = 0, Remaining = Length, Position = 0;
	SubChip = Address/DEVICE_SIZE;
	SubAddress = Address - (DEVICE_SIZE * SubChip);

	// allocate enough memory for all data to be received
	Receive = (uint8_t*)calloc(Length, sizeof(uint8_t));

	while (Remaining > 0)
	{
		// select chip
		Memx->CurrentChip = SubChip;

		// determine data length
		if ((SubAddress+Remaining-1) > (DEVICE_SIZE-1))
		{
			Len = SubAddress - DEVICE_SIZE;
			Remaining = Remaining - Len;
		}
		else
		{
			Len = Remaining;
			Remaining = 0;
		}

		// perform operation
		if (ReadData(Memx, SubAddress, Len) != MEM_OK)
		{
			return MEM_ERROR;
		}
		memcpy(&Receive[Position], Memx->Data, Len);

		// increment
		SubChip = SubChip + 1;
		SubAddress = 0x000000;
		Position = Position + Len;
	}

	// free memory in chip receive buffer
	free(Memx->Data);
	Memx->Data = NULL;
	Memx->DataLength = 0;

	// make receive buffer accessible to external functions
	Memx->Data = Receive;
	Memx->DataLength = Length;

	return MEM_OK;
}

MEM_StatusTypeDef CardWrite(MEM_InitTypeDef *Memx, uint32_t Address, uint8_t * Data, uint16_t Length)
{
	// sanity check the address and length of data to write
	// cannot write more data than is available on all four chips
	if ((Address+Length-1) > CARD_SIZE)
	{
		return MEM_ERROR;
	}

	uint16_t Len;
	int SubChip = 0, SubAddress = 0, Remaining = Length, Position = 0;
	SubChip = Address/DEVICE_SIZE;
	SubAddress = Address - (DEVICE_SIZE * SubChip);

	while (Remaining > 0)
	{
		// select chip
		Memx->CurrentChip = SubChip;

		// determine data length
		if ((SubAddress+Remaining-1) > (DEVICE_SIZE-1))
		{
			Len = SubAddress - DEVICE_SIZE;
			Remaining = Remaining - Len;
		}
		else
		{
			Len = Remaining;
			Remaining = 0;
		}

		// perform operation
		if (PageProgram(Memx, SubAddress, &Data[Position], Len) != MEM_OK)
		{
			return MEM_ERROR;
		}

		// increment
		SubChip = SubChip + 1;
		SubAddress = 0x000000;
		Position = Position + Len;
	}

	return MEM_OK;
}

MEM_StatusTypeDef CardErase(MEM_InitTypeDef *Memx)
{
	for (int index = 0; index < NO_ELEMENTS; index++)
	{
		// select chip
		Memx->CurrentChip = index;

		// perform operation
		if (ChipErase(Memx) != MEM_OK)
		{
			return MEM_ERROR;
		}

		// analyse results
		// no results to analyse
	}
	return MEM_OK;
}

// level 1 functions: hardware

MEM_StatusTypeDef ReadManufacturerDevice(MEM_InitTypeDef *Memx)
{
	uint8_t *Transmt, *Receive;
	uint16_t Len;

	if (Memx->Data != NULL)
	{
		free(Memx->Data);
		Memx->Data = NULL;
	}

	// allocate memory
	Len = 6;
	Transmt = (uint8_t*)calloc(Len, sizeof(uint8_t));
	Receive = (uint8_t*)calloc(Len, sizeof(uint8_t));

	// generate the transmit command
	Transmt[0] = 0x90;
	Transmt[1] = 0x00;
	Transmt[2] = 0x00;
	Transmt[3] = 0x00;

	// check the busy flag
	do
	{
		if ((ReadStatusRegister(Memx, STATUS_1)) != MEM_OK)
		{
			return MEM_ERROR;
		}
	}
	while ((Memx->Status[0] & 0b00000001) == 0b1);

	DriveChipSelectA(Memx);
	if (HAL_SPI_TransmitReceive(Memx->Spi, Transmt, Receive, Len, 500) != HAL_OK)
	{
		return MEM_ERROR;
	}
	DriveChipSelectB(Memx);

	// assign returned values to output
	Memx->DataLength = 2;
	Memx->Data = calloc(Memx->DataLength, sizeof(uint8_t));
	memcpy(Memx->Data, &Receive[4], Memx->DataLength);

	free(Transmt);
	free(Receive);

	return MEM_OK;
}

MEM_StatusTypeDef WriteEnable(MEM_InitTypeDef *Memx)
{
	uint8_t *Transmt;
	uint16_t Len;

	// allocate memory
	Len = 1;
	Transmt = (uint8_t*)calloc(Len, sizeof(uint8_t));

	// generate the transmit command
	Transmt[0] = 0x06;						// instruction

	// check the busy flag
	do
	{
		if ((ReadStatusRegister(Memx, STATUS_1)) != MEM_OK)
		{
			return MEM_ERROR;
		}
	}
	while ((Memx->Status[0] & 0b00000001) == 0b1);

	DriveChipSelectA(Memx);
	if (HAL_SPI_Transmit(Memx->Spi, Transmt, Len, 500) != HAL_OK)
	{
		return MEM_ERROR;
	}
	DriveChipSelectB(Memx);

	free(Transmt);

	return MEM_OK;
}

MEM_StatusTypeDef ChipErase(MEM_InitTypeDef * Memx)
{
	uint8_t *Transmt;
	uint16_t Len;

	// perform operation
	if (WriteEnable(Memx) != MEM_OK)
	{
		return MEM_ERROR;
	}

	// allocate memory
	Len = 1;
	Transmt = (uint8_t*)calloc(Len, sizeof(uint8_t));

	// generate the transmit command
	Transmt[0] = 0x60;						// instruction

	// check the busy flag
	do
	{
		if ((ReadStatusRegister(Memx, STATUS_1)) != MEM_OK)
		{
			return MEM_ERROR;
		}
	}
	while ((Memx->Status[0] & 0b00000001) == 0b1);

	// transmit the command
	DriveChipSelectA(Memx);
	if (HAL_SPI_Transmit(Memx->Spi, Transmt, Len, 500) != HAL_OK)
	{
		return MEM_ERROR;
	}
	DriveChipSelectB(Memx);

	free(Transmt);

	return MEM_OK;
}

MEM_StatusTypeDef PageProgram(MEM_InitTypeDef *Memx, uint32_t Address, uint8_t * Data, uint16_t Length)
{
	// sanity check the address and length of data to write
	if ((Address+Length-1) > 0x3FFFFF)
	{
		return MEM_ERROR;
	}

	uint8_t *Transmt = NULL;
	uint16_t Len;
	int Remaining = Length;
	int Counter = 0;

	while (Remaining > 0)
	{
		if (Remaining > 256)
			Len = 256 + 4;
		else
			Len = Remaining + 4;

		// perform operation
		if (WriteEnable(Memx) != MEM_OK)
		{
			return MEM_ERROR;
		}

		// allocate memory
		Transmt = (uint8_t*)calloc(Len, sizeof(uint8_t));

		// generate the transmit command
		Transmt[0] = 0x02;														// instruction
		Transmt[1] = (Address & 0b111111110000000000000000) >> 16;				// 24-bit address msb
		Transmt[2] = (Address & 0b000000001111111100000000) >>  8;				// 24-bit address
		Transmt[3] = (Address & 0b000000000000000011111111) >>  0;				// 24-bit address lsb

		// copy data into transmit buffer
		memcpy(&Transmt[4], &Data[256*Counter], Len-4);

		// check the busy flag
		do
		{
			if ((ReadStatusRegister(Memx, STATUS_1)) != MEM_OK)
			{
				return MEM_ERROR;
			}
		}
		while ((Memx->Status[0] & 0b00000001) == 0b1);

		DriveChipSelectA(Memx);
		if (HAL_SPI_Transmit(Memx->Spi, Transmt, Len, 500) != HAL_OK)
		{
			return MEM_ERROR;
		}
		DriveChipSelectB(Memx);

		free(Transmt);
		Address = Address + Len - 4;
		Remaining = Remaining - 256;

		++Counter;
	}

	return MEM_OK;
}

MEM_StatusTypeDef ReadData(MEM_InitTypeDef *Memx, uint32_t Address, uint16_t Length)
{
	// sanity check the address and length of data to read
	if ((Address+Length-1) > 0x3FFFFF)
	{
		return MEM_ERROR;
	}

	uint8_t *Transmt = NULL, *Receive = NULL;
	uint16_t Len;

	if (Memx->Data != NULL)
	{
		free(Memx->Data);
		Memx->Data = NULL;
	}

	// allocate memory
	Len = Length + 4;
	Transmt = (uint8_t*)calloc(Len, sizeof(uint8_t));
	Receive = (uint8_t*)calloc(Len, sizeof(uint8_t));

	// generate the transmit command
	Transmt[0] = 0x03;														// instruction
	Transmt[1] = (Address & 0b111111110000000000000000) >> 16;				// 24-bit address
	Transmt[2] = (Address & 0b000000001111111100000000) >>  8;				// 24-bit address
	Transmt[3] = (Address & 0b000000000000000011111111) >>  0;				// 24-bit address

	// check the busy flag
	do
	{
		if ((ReadStatusRegister(Memx, STATUS_1)) != MEM_OK)
		{
			return MEM_ERROR;
		}
	}
	while ((Memx->Status[0] & 0b00000001) == 0b1);

	DriveChipSelectA(Memx);
	if (HAL_SPI_TransmitReceive(Memx->Spi, Transmt, Receive, Len, 500) != HAL_OK)
	{
		return MEM_ERROR;
	}
	DriveChipSelectB(Memx);

	// assign returned values to output
	Memx->DataLength = Length;
	Memx->Data = calloc(Memx->DataLength, sizeof(uint8_t));
	memcpy(Memx->Data, &Receive[4], Memx->DataLength);

	free(Transmt);
	free(Receive);

	return MEM_OK;
}

MEM_StatusTypeDef ReadStatusRegister(MEM_InitTypeDef *Memx, uint8_t RegisterNo)
{
	// sanity check
	if (RegisterNo > 2)
	{
		return MEM_ERROR;
	}

	uint8_t *Transmt, *Receive;
	uint16_t Len;

	if (Memx->Status != NULL)
	{
		free(Memx->Status);
		Memx->Status = NULL;
	}

	// allocate memory
	Len = 2;
	Transmt = (uint8_t*)calloc(Len, sizeof(uint8_t));
	Receive = (uint8_t*)calloc(Len, sizeof(uint8_t));

	// generate the transmit command
	switch(RegisterNo)
	{
	case 0:
		Transmt[0] = 0x05;
		break;
	case 1:
		Transmt[0] = 0x35;
		break;
	case 2:
		Transmt[0] = 0x15;
		break;
	default:
		Transmt[0] = 0x05;
		break;
	}

	DriveChipSelectA(Memx);
	if (HAL_SPI_TransmitReceive(Memx->Spi, Transmt, Receive, Len, 500) != HAL_OK)
	{
		return MEM_ERROR;
	}
	DriveChipSelectB(Memx);

	// assign returned values to output
	Memx->StatusLength = 1;
	Memx->Status = calloc(Memx->StatusLength, sizeof(uint8_t));
	memcpy(Memx->Status, &Receive[1], Memx->StatusLength);

	free(Transmt);
	free(Receive);

	return MEM_OK;
}

// helper functions

void ChipSelect(MEM_InitTypeDef *Memx)
{
	switch (Memx->CurrentChip)
	{
		case 0:
		{
			// select chip 1
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin1, 0);	// LSB
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin2, 0);	// MSB
			break;
		}
		case 1:
		{
			// select chip 2
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin1, 1);	// LSB
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin2, 0);	// MSB
			break;
		}
		case 2:
		{
			// select chip 3
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin1, 0);	// LSB
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin2, 1);	// MSB
			break;
		}
		case 3:
		{
			// select chip 4
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin1, 1);	// LSB
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin2, 1);	// MSB
			break;
		}
		default:
		{
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin1, 0);	// LSB
			HAL_GPIO_WritePin(Memx->MemoryCardBank, Memx->MemoryCardPin2, 0);	// MSB
			break;
		}
	}
}

void DriveChipSelectA(MEM_InitTypeDef *Memx)
{
	ChipSelect(Memx);
}

void DriveChipSelectB(MEM_InitTypeDef *Memx)
{
	++Memx->CurrentChip;
	ChipSelect(Memx);
	--Memx->CurrentChip;
}
