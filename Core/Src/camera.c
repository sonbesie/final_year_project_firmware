/**
  ******************************************************************************
  * @file           : camera.c
  * @brief          : The file running the cameras.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 & 2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#include "camera.h"

// standard functions (during runtime)

CAM_StatusTypeDef InitCam(CAM_InitTypeDef *Camx)
{
	// declare function variables
	int found;
	int count;

	// reset camera status
	Camx->Done = 1;
	Camx->ImageCounter = 0;

	// validate camera address input
	found = 0;
	count = 0;
	while (found == 0 && count < 2)
	{
		if (count == Camx->Settings.CameraAddress)
		{
			found = 1;
		}
		++count;
	}
	if (found != 1)
	{
		CameraErrorHandler(Camx);		// log the error
		return CAM_ERROR;
	}

	// validate baud rate input
	switch(Camx->Settings.BaudRate)
	{
	case 0x00:
		if (Camx->Uart->Init.BaudRate != 0x02580)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	case 0x01:
		if (Camx->Uart->Init.BaudRate != 0x04B00)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	case 0x02:
		if (Camx->Uart->Init.BaudRate != 0x09600)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	case 0x03:
		if (Camx->Uart->Init.BaudRate != 0x0E100)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	case 0x04:
		if (Camx->Uart->Init.BaudRate != 0x1C200)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	case 0x05:
		if (Camx->Uart->Init.BaudRate != 0x38400)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	case 0x06:
		if (Camx->Uart->Init.BaudRate != 0x70800)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	case 0x07:
		if (Camx->Uart->Init.BaudRate != 0xE1000)
		{
			CameraErrorHandler(Camx);
			return CAM_ERROR;
		}
		break;
	default:
		CameraErrorHandler(Camx);		// log the error
		return CAM_ERROR;
	}

	// initialise the image length array
	count = 0;
	while (count < 6)
	{
		Camx->ImageLength[count] = 0x00;
		++count;
	}

	// generate the command and success array prefixes
	Camx->Settings.ComPrefix[0] = 0x56;
	Camx->Settings.ComPrefix[1] = Camx->Settings.CameraAddress;
	Camx->Settings.SucPrefix[0] = 0x76;
	Camx->Settings.SucPrefix[1] = Camx->Settings.CameraAddress;

	// if no errors are present return success
	return CAM_OK;
}

CAM_StatusTypeDef CaptureImage(CAM_InitTypeDef *Camx, RTC_HandleTypeDef *Timx)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[5];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 5, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0xBB;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x36;
	Command[3] = 0x01;
	Command[4] = 0x00;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x36;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// get time and date
	do
	{
		HAL_RTC_GetTime(Timx, &Camx->Time[0], RTC_FORMAT_BCD);
		HAL_RTC_GetDate(Timx, &Camx->Date[0], RTC_FORMAT_BCD);
	}
	while (Camx->Time[0].Seconds == 0xFF);

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef CaptureMultipleImages(CAM_InitTypeDef *Camx, RTC_HandleTypeDef *Timx, uint32_t Time, uint8_t Number)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[7];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 7, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT;

	// validate number of images to be taken
	// only values 2, 3, 5 and 5 will be accepted
	if (Number < 2 || Number > 5)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// validate the time between each image
	// only accept values between 10 and 1000ms
	if (Time < 1 || Time > 1000)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x88;
	Command[3] = 0x03;
	Command[4] = Number;
	Command[5] = 0x00;
	Command[6] = Time;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x88;
	Success[3] = 0x01;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, 5000) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	for (int index = 1; index < Number+1; index++)
	{
		// get time and date
		do
		{
			HAL_RTC_GetTime(Timx, &Camx->Time[index], RTC_FORMAT_BCD);
			HAL_RTC_GetDate(Timx, &Camx->Date[index], RTC_FORMAT_BCD);
		}
		while (Camx->Time[index].Seconds == 0xFF);

		// add time between images
		Camx->Time[index].SubSeconds = Camx->Time[1].SubSeconds + ((index-1)*40);
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef ReadImageDataLength(CAM_InitTypeDef *Camx, uint8_t Number)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[5];
	uint8_t Result[9];
	uint8_t Success[5];
	uint16_t TraSize = 5, RecSize = 9, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x34;
	Command[3] = 0x01;
	Command[4] = Number;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x34;
	Success[3] = 0x00;
	Success[4] = 0x04;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the image length in bytes
	Camx->ImageLength[Number] = (Result[5]<<24)|(Result[6]<<16)|(Result[7]<<8)|(Result[8]);

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef ReadImageData(CAM_InitTypeDef *Camx, uint8_t Number)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[16];
	uint16_t TraSize = 16, RecSize;
	uint32_t TraTimeout = TRA_TIMEOUT;
	uint64_t StartAddress, StopAddress;

	// free previously allocated memory
	if (Camx->ImageData[Number] != 0)
	{
		free(Camx->ImageData[Number]);
		Camx->ImageData[Number] = 0;
	}

	// disable camera done check
	Camx->Done = 0;

	// generate start and stop addresses
	StartAddress = 0x00;
	StopAddress = Camx->ImageLength[Number];

	// allocate memory for the image
	RecSize = (Camx->ImageLength[Number])+10;
	Camx->ImageData[Number] = calloc(RecSize, sizeof(uint8_t));

	// generate the command array
	// section 1 (start)
	Command[0 ] = Camx->Settings.ComPrefix[0];
	Command[1 ] = Camx->Settings.ComPrefix[1];
	Command[2 ] = 0x32;
	Command[3 ] = 0x0C;
	Command[4 ] = Number;
	Command[5 ] = 0x0A;
	// section 2 (start address)
	Command[6 ] = (StartAddress >> 24)&0xFF;
	Command[7 ] = (StartAddress >> 16)&0xFF;
	Command[8 ] = (StartAddress >>  8)&0xFF;
	Command[9 ] = (StartAddress      )&0xFF;
	// section 3 (stop  address)
	Command[10] = (StopAddress  >> 24)&0xFF;
	Command[11] = (StopAddress  >> 16)&0xFF;
	Command[12] = (StopAddress  >>  8)&0xFF;
	Command[13] = (StopAddress       )&0xFF;
	// section 4 (end)
	Command[14] = 0x00;
	Command[15] = 0xFF;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive_IT(Camx->Uart, Camx->ImageData[Number], RecSize) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef StopCapture(CAM_InitTypeDef *Camx)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[5];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 5, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x36;
	Command[3] = 0x01;
	Command[4] = 0x03;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x36;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef SetImageCompressionRatio(CAM_InitTypeDef *Camx, uint8_t Ratio)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[9];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 9, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// validate the image compression ratio input
	if (Ratio < 54 || Ratio > 144)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x31;
	Command[3] = 0x05;
	Command[4] = 0x01;
	Command[5] = 0x01;
	Command[6] = 0x12;
	Command[7] = 0x04;
	Command[8] = Ratio;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x31;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef SetImageResolution(CAM_InitTypeDef *Camx, uint8_t Resolution)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[9];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 9, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x31;
	Command[3] = 0x05;
	Command[5] = 0x01;
	Command[6] = 0x00;
	Command[7] = 0x19;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x31;
	Success[3] = 0x01;
	Success[4] = 0x00;

	// validate and update command array
	switch(Resolution)
	{
	case 0:
		Command[4] = 0x04;
		Command[8] = 0x22;
		break;
	case 1:
		Command[4] = 0x04;
		Command[8] = 0x11;
		break;
	case 2:
		Command[4] = 0x04;
		Command[8] = 0x00;
		break;
	case 3:
		Command[4] = 0x05;
		Command[8] = 0x33;
		break;
	case 4:
		Command[4] = 0x05;
		Command[8] = 0x44;
		break;
	case 5:
		Command[4] = 0x05;
		Command[8] = 0x55;
		break;
	case 6:
		Command[4] = 0x05;
		Command[8] = 0x66;
		break;
	default:
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef SetCameraBaud(CAM_InitTypeDef *Camx, uint8_t Baud)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[10];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 10, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x31;
	Command[3] = 0x06;

	Command[5] = 0x02;
	Command[6] = 0x00;
	Command[7] = 0x08;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x31;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// validate and update command array
	switch(Baud)
	{
	case 0:
		Command[4] = 0x04;
		Command[8] = 0xAE;
		Command[9] = 0xC8;
		Camx->Uart->Init.BaudRate =   9600;
		break;
	case 1:
		Command[4] = 0x04;
		Command[8] = 0x56;
		Command[9] = 0xE4;
		Camx->Uart->Init.BaudRate =  19200;
		break;
	case 2:
		Command[4] = 0x04;
		Command[8] = 0x2A;
		Command[9] = 0xF2;
		Camx->Uart->Init.BaudRate =  38400;
		break;
	case 3:
		Command[4] = 0x04;
		Command[8] = 0x1C;
		Command[9] = 0x4C;
		Camx->Uart->Init.BaudRate =  57600;
		break;
	case 4:
		Command[4] = 0x04;
		Command[8] = 0x0D;
		Command[9] = 0xA6;
		Camx->Uart->Init.BaudRate = 115200;
		break;
	case 5:
		Command[4] = 0x05;
		Command[8] = 0xEE;
		Command[9] = 0xA1;
		Camx->Uart->Init.BaudRate = 230400;
		break;
	case 6:
		Command[4] = 0x05;
		Command[8] = 0xEE;
		Command[9] = 0xA2;
		Camx->Uart->Init.BaudRate = 460800;
		break;
	case 7:
		Command[4] = 0x05;
		Command[8] = 0xEE;
		Command[9] = 0xA3;
		Camx->Uart->Init.BaudRate = 921600;
		break;
	default:
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// update the board baud rate
	if (HAL_UART_Init(Camx->Uart) != HAL_OK)
	{
		CameraErrorHandler(Camx);
	}

	// restart the cameras
	RestartCamera(Camx);

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef EnableDetectMotion(CAM_InitTypeDef *Camx)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[5];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 5, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0xBB;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x37;
	Command[3] = 0x01;
	Command[4] = 0x01;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x37;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	//----------------------------------------------------------------------------

	// section to start waiting for data indicating motion detection

	// free previously allocated memory
	if (Camx->InteruptResult != 0)
	{
		free(Camx->InteruptResult);
		Camx->InteruptResultLength = 0;
	}

	// disable camera done check
	Camx->Done = 0;

	// allocate memory
	Camx->InteruptResultLength = RecSize;
	Camx->InteruptResult = calloc(RecSize, sizeof(uint8_t));

	// receive the motion detection ping
	if (HAL_UART_Receive_IT(Camx->Uart, Camx->InteruptResult, RecSize) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef DisableDetectMotion(CAM_InitTypeDef *Camx)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[5];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 5, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0xBB;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x37;
	Command[3] = 0x01;
	Command[4] = 0x00;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x37;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef SetMotionDetectionSensitivity(CAM_InitTypeDef *Camx, uint8_t Sensitivity)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[9];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 9, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x31;
	Command[3] = 0x05;
	Command[4] = 0x01;
	Command[5] = 0x01;
	Command[6] = 0x1A;
	Command[7] = 0x6E;
	Command[8] = Sensitivity;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x31;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef StartCamera(CAM_InitTypeDef *Camx)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint16_t RecSize = 76;

	// free previously allocated memory
	if (Camx->InteruptResult != 0)
	{
		free(Camx->InteruptResult);
		Camx->InteruptResultLength = 0;
	}

	// disable camera done check
	Camx->Done = 0;

	// allocate memory
	Camx->InteruptResultLength = RecSize;
	Camx->InteruptResult = calloc(RecSize, sizeof(uint8_t));

	// save the response from the camera (do not check, will result in endless loop)
	if (HAL_UART_Receive_IT(Camx->Uart, Camx->InteruptResult, RecSize) != HAL_OK)
	{
		//
	}

	// turn the relays on
	switch(Camx->Settings.CameraAddress)
	{
		case CAMERA_1:
		{
			// turn on
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET  );
			break;
		}
		case CAMERA_2:
		{
			// turn on
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET  );
			break;
		}
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef StopCamera(CAM_InitTypeDef *Camx)
{
	// turn the relays off
	switch(Camx->Settings.CameraAddress)
	{
		case CAMERA_1:
		{
			// turn off
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
			break;
		}
		case CAMERA_2:
		{
			// turn off
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
			break;
		}
	}

	// return command successful
	return CAM_OK;
}

CAM_StatusTypeDef RestartCamera(CAM_InitTypeDef *Camx)
{
	int timeout_st = 0, timeout_sp = 0;

restart_start:

	// stop camera
	StopCamera(Camx);
	// wait
	CameraResetDelay();
	// start camera
	StartCamera(Camx);

	timeout_st = HAL_GetTick();
	timeout_sp = timeout_st;
	while (Camx->Done == 0)
	{
		// if timeout, keep on trying forever
		if (((timeout_sp-timeout_st)/1000) > 5)
		{
			goto restart_start;
		}
		timeout_sp = HAL_GetTick();
	 }

	// return command successful
	// upon a successful camera restart
	return CAM_OK;
}

// standard functions (not during runtime)

CAM_StatusTypeDef SetCameraAddress(CAM_InitTypeDef *Camx, uint8_t Address)
{
	// flush the receive buffer
	__HAL_UART_FLUSH_DRREGISTER(Camx->Uart);

	// declare function variables
	uint8_t Command[9];
	uint8_t Result[5];
	uint8_t Success[5];
	uint16_t TraSize = 9, RecSize = 5, SucSize = 5;
	uint32_t TraTimeout = TRA_TIMEOUT, RecTimeout = REC_TIMEOUT;

	// Initialise the result array
	for (int item = 0; item < RecSize; ++item)
	{
		Result[item] = 0;
	}

	// generate the command array
	Command[0] = Camx->Settings.ComPrefix[0];
	Command[1] = Camx->Settings.ComPrefix[1];
	Command[2] = 0x31;
	Command[3] = 0x05;
	Command[4] = 0x04;
	Command[5] = 0x01;
	Command[6] = 0x00;
	Command[7] = 0x06;
	Command[8] = Address;

	// generate the success array
	Success[0] = Camx->Settings.SucPrefix[0];
	Success[1] = Camx->Settings.SucPrefix[1];
	Success[2] = 0x31;
	Success[3] = 0x00;
	Success[4] = 0x00;

	// transmit the command
	if (HAL_UART_Transmit(Camx->Uart, Command, TraSize, TraTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// save the response from the camera
	if (HAL_UART_Receive (Camx->Uart, Result,  RecSize, RecTimeout) != HAL_OK)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// analyse the response
	if (ContainsResult(Result, RecSize, Success, SucSize) != 1)
	{
		CameraErrorHandler(Camx);
		return CAM_ERROR;
	}

	// return command successful
	return CAM_OK;
}

// void functions

void CameraErrorHandler(CAM_InitTypeDef *Camx)
{
	// restart the camera
	RestartCamera(Camx);
}

void CameraResetDelay(void)
{
	// wait for cameras to reset, set this value to drain power from camera and reset memory
	long delay = 0;
	while (delay < 2000000)
	{
		++delay;
	}
}

// helper functions

int ContainsResult(uint8_t* Haystack, uint16_t HaystackLength, const uint8_t*Needle, uint16_t NeedleLength)
{
	for (int i = 0; i < HaystackLength-NeedleLength+1; i++)
	{
		int Found = 1;
		for (int j = 0; j < NeedleLength; j++)
		{
			if (Needle[j] != Haystack[i+j])
			{
				Found = 0;
				break;
			}
		}
		if (Found == 1)
		{
			return 1;
		}
	}
	return 0;
}

