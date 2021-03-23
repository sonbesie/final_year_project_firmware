/**
  ******************************************************************************
  * @file           : image.h
  * @brief          : This file contains the image analysis algorithm.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#ifndef INC_IMAGE_H_
#define INC_IMAGE_H_

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f7xx_hal.h"

typedef struct
{
	#define MAX_LEN 32		// length of the shift register
	#define MAX_DIF 5		// number of standard deviations

	// shift register of image lengths
	uint16_t Length[MAX_LEN];
	uint8_t Index;

	 // motion
	uint8_t Motion;

} IMA_MotionTypeDef;

typedef struct
{
	#define MAX_COR_TEM 32			// number of minimums coordinates to save
	#define TRA_COR_TEM 2

	// result grid
	uint32_t **Result;

	// local minimums
	uint16_t *Row, *Col;
	uint32_t *Val;

	// output
	uint16_t OutputRow[TRA_COR_TEM], OutputCol[TRA_COR_TEM];

} IMA_TemplateTypeDef;

typedef struct
{
	#define MAX_COR_MSK 32			// number of minimums coordinates to save
	#define TRA_COR_MSK 2

	// result grid
	uint32_t **Result;

	// local minimums
	uint16_t *Row, *Col;
	uint32_t *Val;

	// output
	uint16_t OutputRow[TRA_COR_MSK], OutputCol[TRA_COR_MSK];

	// threshold
	uint8_t Threshold;

	// trigger
	uint8_t Diff1[6], Diff2[6];

} IMA_MaskTypeDef;

typedef struct
{
	uint8_t **Image;					// main: image two dimensional array

	IMA_MotionTypeDef Motion;			// main: motion detection
	IMA_TemplateTypeDef Temp;			// main: template matching
	IMA_MaskTypeDef Mask;				// main: create mask

} IMA_InitTypeDef;

typedef enum
{
	  IMA_OK	= 0x00U,
	  IMA_ERROR	= 0x01U

} IMA_StatusTypeDef;

typedef enum
{
	 IMA_SQDIFF	= 0x00U

} IMA_AlgorithmTypeDef;

// user
IMA_StatusTypeDef InitIma(IMA_InitTypeDef*);
IMA_StatusTypeDef MatchTemplate(IMA_InitTypeDef*, uint8_t[][8][8], uint16_t, IMA_AlgorithmTypeDef);
IMA_StatusTypeDef CreateDetectionMask(IMA_InitTypeDef*, uint8_t[][8][8], uint16_t);
IMA_StatusTypeDef DetectsMotion(IMA_InitTypeDef*, uint16_t);
IMA_StatusTypeDef GetDetectionCoordinates(IMA_InitTypeDef*);
IMA_StatusTypeDef GetShouldersCoordinates(IMA_InitTypeDef*, uint8_t);

//
void free_memory(IMA_InitTypeDef*);

// helper functions
IMA_StatusTypeDef ConvertsImage(IMA_InitTypeDef*, uint8_t[][8][8], uint16_t);
IMA_StatusTypeDef MatchTemplate_SQDIFF(IMA_InitTypeDef*, uint16_t);
void find_local_minimum(uint32_t**, uint16_t, uint16_t, uint32_t*, uint16_t*, uint16_t*, uint16_t);
void find_local_maximum(uint32_t**, uint16_t, uint16_t, uint32_t*, uint16_t*, uint16_t*, uint16_t);
void sort_low_to_high(uint16_t*, uint16_t);
void sort_high_to_low(uint16_t*, uint16_t);

#endif /* INC_IMAGE_H_ */
