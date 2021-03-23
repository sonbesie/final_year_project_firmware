/**
  ******************************************************************************
  * @file           : camera.h
  * @brief          : The file running the cameras.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 & 2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f7xx_hal.h"

typedef struct
{
	uint8_t ComPrefix[2];
	uint8_t SucPrefix[2];
	uint8_t BaudRate;
	uint8_t CameraAddress;
} CAM_SettingsTypeDef;

typedef struct
{
	// structures
	UART_HandleTypeDef *Uart;
	CAM_SettingsTypeDef Settings;

	// image (one index for each image)
	uint64_t ImageLength[6];
	uint8_t *ImageData[6];

	// general interrupt result
	uint16_t InteruptResultLength;
	uint8_t *InteruptResult;

	// date
	RTC_DateTypeDef Date[6];
	// time
	RTC_TimeTypeDef Time[6];

	// general flags
	int Done, ImageCounter;

} CAM_InitTypeDef;

typedef enum
{
	  CAM_OK       = 0x00U,
	  CAM_ERROR    = 0x01U
} CAM_StatusTypeDef;

#define CAMERA_1 0x00U
#define CAMERA_2 0x01U

#define BAUD_009600 0x00U
#define BAUD_019200 0x01U
#define BAUD_038400 0x02U
#define BAUD_057600 0x03U
#define BAUD_115200 0x04U
#define BAUD_230400 0x05U
#define BAUD_460800 0x06U
#define BAUD_921600 0x07U

#define RES_0160_0120 0x00U
#define RES_0320_0240 0x01U
#define RES_0640_0480 0x02U
#define RES_1024_0768 0x03U
#define RES_1280_0720 0x04U
#define RES_1280_0960 0x05U
#define RES_1920_1080 0x06U

#define TRA_TIMEOUT 5000		// define timeout for data transmit
#define REC_TIMEOUT 5000		// define timeout for data receive

// standard functions (during runtime)
CAM_StatusTypeDef InitCam(CAM_InitTypeDef *);
CAM_StatusTypeDef CaptureImage(CAM_InitTypeDef *, RTC_HandleTypeDef*);
CAM_StatusTypeDef CaptureMultipleImages(CAM_InitTypeDef *, RTC_HandleTypeDef *, uint32_t, uint8_t);
CAM_StatusTypeDef ReadImageDataLength(CAM_InitTypeDef *, uint8_t);
CAM_StatusTypeDef ReadImageData(CAM_InitTypeDef *, uint8_t);
CAM_StatusTypeDef StopCapture(CAM_InitTypeDef *);
CAM_StatusTypeDef SetImageCompressionRatio(CAM_InitTypeDef *, uint8_t);
CAM_StatusTypeDef SetImageResolution(CAM_InitTypeDef *, uint8_t);
CAM_StatusTypeDef SetCameraBaud(CAM_InitTypeDef *, uint8_t);
CAM_StatusTypeDef EnableDetectMotion(CAM_InitTypeDef *);
CAM_StatusTypeDef DisableDetectMotion(CAM_InitTypeDef *);
CAM_StatusTypeDef SetMotionDetectionSensitivity(CAM_InitTypeDef *, uint8_t);
CAM_StatusTypeDef StartCamera(CAM_InitTypeDef *);
CAM_StatusTypeDef StopCamera(CAM_InitTypeDef *);
CAM_StatusTypeDef RestartCamera(CAM_InitTypeDef *);

// standard functions (not during runtime)
CAM_StatusTypeDef SetCameraAddress(CAM_InitTypeDef *, uint8_t);

// void functions
void SerialTestFunction(CAM_InitTypeDef *);
void CameraErrorHandler(CAM_InitTypeDef *);
void CameraResetDelay(void);

// helper functions
int ContainsResult(uint8_t*, uint16_t, const uint8_t*, uint16_t);

#endif /* INC_CAMERA_H_ */
