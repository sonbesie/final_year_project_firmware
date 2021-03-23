/**
  ******************************************************************************
  * @file           : jfif.h
  * @brief          : The file running the jfif decoding algorithm.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#ifndef INC_JFIF_H_
#define INC_JFIF_H_

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f7xx_hal.h"

//------------------------------------------------------------------------------
// DEFINE THE RESOLUTION
#define R0160_0120
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define LU 0
#define CR 1
#define CB 2
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// CAMERA CONSTANTS
#define HUFFMAN_TABLE_LENGTH 4			// number of huffman tables
#define HUFFMAN_TREE_LENGTH 16			// length of each huffman table (standard value)
#define HUFFMAN_OFFSET 5

#define QUANTIS_TABLE_LENGTH 2			// number of quantis tables
#define QUANTIS_TREE_LENGTH 8			// length of each quantis table (standard value) (QUANTIS_TREE_LENGTH X QUANTIS_TREE_LENGTH)
#define QUANTIS_OFFSET 5

#define MCU_TABLES_LU 4
#define MCU_TABLES_CR 1
#define MCU_TABLES_CB 1
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// 08X08 CONSTANTS
#ifdef R0160_0120

#define MCU 80							// number of MCU's in the image

#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#ifdef R0320_0240
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#ifdef R0640_0480
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#ifdef R1024_0768
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#ifdef R1280_0720
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#ifdef R1280_0960
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#ifdef R1920_1080
#endif
//------------------------------------------------------------------------------

// structure containing the quantisation table
typedef struct
{
	// temp structure
	uint8_t *quantisTable[QUANTIS_TABLE_LENGTH];
	int length[QUANTIS_TABLE_LENGTH];

	// result
	uint16_t code[QUANTIS_TABLE_LENGTH][QUANTIS_TREE_LENGTH][QUANTIS_TREE_LENGTH];

} DEC_QuantisationTable;

// structure containing the huffman table
typedef struct
{
	// temp structure
	uint8_t  *huffmanTable[HUFFMAN_TABLE_LENGTH];
	int length[HUFFMAN_TABLE_LENGTH];

	// result
	uint16_t *tree[HUFFMAN_TABLE_LENGTH];
	uint16_t *code[HUFFMAN_TABLE_LENGTH];
	uint16_t *leng[HUFFMAN_TABLE_LENGTH];
	int tree_length[HUFFMAN_TABLE_LENGTH];
	int code_length[HUFFMAN_TABLE_LENGTH];
	int leng_length[HUFFMAN_TABLE_LENGTH];

} DEC_HuffmanTable;

// structure containing the frequency domain structure (dct)
typedef struct
{
	// lu
	int16_t *mcu_lu[MCU*MCU_TABLES_LU];
	int mcu_lu_length[MCU*MCU_TABLES_LU];
	// cr
	int16_t *mcu_cr[MCU*MCU_TABLES_CR];
	int mcu_cr_length[MCU*MCU_TABLES_CR];
	// cb
	int16_t *mcu_cb[MCU*MCU_TABLES_CB];
	int mcu_cb_length[MCU*MCU_TABLES_CB];

} DEC_McuTable;

typedef struct
{
	DEC_QuantisationTable QuantisTable;
	DEC_HuffmanTable HuffmanTable;
	DEC_McuTable McuTable;

	// input data to work with
	// user sets befor initialisation
	uint8_t *ImageData;
	int ImageLength;

	// pixel output
	// normally uninitialised
	uint8_t PixelsLu[MCU*MCU_TABLES_LU][8][8];
	uint8_t PixelsCr[MCU*MCU_TABLES_CR][8][8];
	uint8_t PixelsCb[MCU*MCU_TABLES_CB][8][8];

	// status flags
	int Quantis, Huffman;

} DEC_InitTypeDef;

typedef enum
{
	  DEC_OK       = 0x00U,
	  DEC_ERROR    = 0x01U
} DEC_StatusTypeDef;

// private

DEC_StatusTypeDef SplitEncodedData(DEC_InitTypeDef *);
DEC_StatusTypeDef DecodeHuffman(DEC_InitTypeDef *);
DEC_StatusTypeDef DecodeQuantis(DEC_InitTypeDef *);
DEC_StatusTypeDef DecodePicture(DEC_InitTypeDef *);
DEC_StatusTypeDef GenerateTables(DEC_InitTypeDef *);
DEC_StatusTypeDef InverseDCT(DEC_InitTypeDef *, uint16_t, uint8_t);

// user

DEC_StatusTypeDef InitDec(DEC_InitTypeDef *);
DEC_StatusTypeDef Decode(DEC_InitTypeDef *);

#endif /* INC_JFIF_H_ */
