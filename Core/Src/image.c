/**
  ******************************************************************************
  * @file           : image.c
  * @brief          : This file contains the image analysis algorithm.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#include "image.h"
#include "image_hard.h"

// user

IMA_StatusTypeDef InitIma(IMA_InitTypeDef* Imax)
{
	//////////////////////////////////////////////////////
	// image
	Imax->Image = NULL;
	//////////////////////////////////////////////////////

	//////////////////////////////////////////////////////
	// motion
	Imax->Motion.Index = 0;
	Imax->Motion.Motion = 0;
	for (int index = 0; index < MAX_LEN; index++)
	{
		Imax->Motion.Length[index] = 0;
	}
	//////////////////////////////////////////////////////

	//////////////////////////////////////////////////////
	// template
	Imax->Temp.Result = NULL;
	Imax->Temp.Val = NULL;
	Imax->Temp.Row = NULL;
	Imax->Temp.Col = NULL;
	//////////////////////////////////////////////////////

	//////////////////////////////////////////////////////
	// mask
	Imax->Mask.Result = NULL;
	Imax->Mask.Val = NULL;
	Imax->Mask.Row = NULL;
	Imax->Mask.Col = NULL;
	Imax->Mask.Threshold = 0;
	for (int index = 0; index < 6; index++)
	{
		Imax->Mask.Diff1[index] = 0xFF;
		Imax->Mask.Diff2[index] = 0xFF;
	}
	//////////////////////////////////////////////////////

	// return ok
	return IMA_ERROR;
}

IMA_StatusTypeDef MatchTemplate(IMA_InitTypeDef* Imax, uint8_t Image[][8][8], uint16_t Length, IMA_AlgorithmTypeDef Algorithm)
{
	uint16_t Ver = 0;

	free_memory(Imax);

	// create image
	if (ConvertsImage(Imax, Image, Length) != IMA_OK) return IMA_ERROR;

	// algorithm
	switch (Algorithm)
	{
	case IMA_SQDIFF:
		if (MatchTemplate_SQDIFF(Imax, Length) != IMA_OK)
		{
			return IMA_ERROR;
		}
		break;

	// add more algorithms

	default:
		return IMA_ERROR;
		break;
	}

	// resolution
	switch (Length)
	{
	// 160 x 120
	case 0x050: 		// 080 (cr & cb)
		Ver   = 0x040;	// pixels
		break;
	case 0x140:			// 160 (lu & lu)
		Ver   = 0x080;	// pixels
		break;

	// more resolutions

	default:
		return IMA_ERROR;
		break;
	}

	// free allocated memory
	for (int index = 0; index < Ver; index++)
	{
		free(Imax->Image[index]);
		Imax->Image[index] = NULL;
	}
	free(Imax->Image);
	Imax->Image = NULL;

	// return ok
	return IMA_OK;
}

IMA_StatusTypeDef CreateDetectionMask(IMA_InitTypeDef* Imax, uint8_t Image[][8][8], uint16_t Length)
{
	uint16_t Hor = 0, Ver = 0, Row = 0, Col = 0, CompRow1 = 0, CompRow2 = 0, CompCol1 = 0, CompCol2 = 0;

	free_memory(Imax);

	// create image
	if (ConvertsImage(Imax, Image, Length) != IMA_OK) return IMA_ERROR;

	// resolution
	switch (Length)
	{
	// 160 x 120
	case 0x050: 		// 080 (cr & cb)
		Hor   = 0x050;	// pixels x
		Ver   = 0x040;	// pixels y
		CompRow1 = (Imax->Temp.OutputRow[0]/2)-8;
		CompRow2 = (Imax->Temp.OutputRow[1]/2)+8;
		CompCol1 = (Imax->Temp.OutputCol[0]/2)-8;
		CompCol2 = (Imax->Temp.OutputCol[1]/2)+8;
		break;
	case 0x140:			// 160 (lu & lu)
		Hor   = 0x0A0;	// pixels x
		Ver   = 0x080;	// pixels y
		CompRow1 = (Imax->Temp.OutputRow[0]/1)-8;
		CompRow2 = (Imax->Temp.OutputRow[1]/1)+8;
		CompCol1 = (Imax->Temp.OutputCol[0]/1)-8;
		CompCol2 = (Imax->Temp.OutputCol[1]/1)+8;
		break;

	// more resolutions

	default:
		return IMA_ERROR;
		break;
	}

	// allocate memory
	Imax->Mask.Result = (uint32_t**)calloc(Ver, sizeof(uint32_t*));
	for (int index = 0; index < Ver; index++)
	{
		Imax->Mask.Result[index] = (uint32_t*)calloc(Hor, sizeof(uint32_t*));
	}

	// maximise all pixels outsize the detection mask
	Row = 0;
	while (Row < Ver)
	{
		Col = 0;
		while (Col < Hor)
		{
			if ((Row < CompRow1) || (Row > CompRow2))
			{
				Imax->Mask.Result[Row][Col] = 0xFF;
			}
			else
			if ((Col < CompCol1) || (Col > CompCol2))
			{
				Imax->Mask.Result[Row][Col] = 0xFF;
			}
			else
			{
				Imax->Mask.Result[Row][Col] = Imax->Image[Row][Col];
			}
			Col++;
		}
		Row++;
	}

	// assign threshold
	if (Imax->Mask.Threshold == 0) Imax->Mask.Threshold = Imax->Mask.Result[(Imax->Temp.OutputRow[0]/2)][(Imax->Temp.OutputCol[0]/2)];

	// find local minimums
	Imax->Mask.Val = calloc(MAX_COR_TEM, sizeof(uint32_t));
	Imax->Mask.Row = calloc(MAX_COR_TEM, sizeof(uint16_t));
	Imax->Mask.Col = calloc(MAX_COR_TEM, sizeof(uint16_t));
	find_local_minimum(Imax->Mask.Result, Ver, Hor, Imax->Mask.Val, Imax->Mask.Col, Imax->Mask.Row, MAX_COR_MSK);

	// free allocated memory
	for (int index = 0; index < Ver; index++)
	{
		free(Imax->Mask.Result[index]);
		Imax->Mask.Result[index] = NULL;
	}
	free(Imax->Mask.Result);
	Imax->Mask.Result = NULL;

	// free allocated memory
	for (int index = 0; index < Ver; index++)
	{
		free(Imax->Image[index]);
		Imax->Image[index] = NULL;
	}
	free(Imax->Image);
	Imax->Image = NULL;

	// return ok
	return IMA_OK;
}

IMA_StatusTypeDef DetectsMotion(IMA_InitTypeDef* Imax, uint16_t Length)
{
	// if array is full
	if (Imax->Motion.Index > MAX_LEN - 1)
	{
		// find sum
		int Sum = 0;
		for (int index = 0; index < MAX_LEN; index++) Sum = Sum + Imax->Motion.Length[index];

		// find average
		int Avg = Sum/(MAX_LEN+0);

		// find squared difference
		int Sqd = 0;
		for (int index = 0; index < MAX_LEN; index++)
		{
			int diff = Imax->Motion.Length[index] - Avg;
			Sqd = Sqd + pow(diff, 2);
		}

		// find variance
		int Var = Sqd/(MAX_LEN-1);

		// find standard deviation
		int Dev = sqrt(Var);

		// check difference
		if ((abs(Avg - Length)) > MAX_DIF*Dev)
		{
			// set detect
			Imax->Motion.Motion = 1;
		}
		else
		{
			// shift the register and add the new value at the start
			for (int index = MAX_LEN - 1; index > 0; index--)
			{
				Imax->Motion.Length[index] = Imax->Motion.Length[index-1];
			}
			Imax->Motion.Length[0] = Length;
		}
	}
	else
	{
		// add new length to length array
		Imax->Motion.Length[Imax->Motion.Index] = Length;
		++Imax->Motion.Index;
	}

	// return ok
	return IMA_OK;
}

IMA_StatusTypeDef GetDetectionCoordinates(IMA_InitTypeDef* Imax)
{
	uint16_t a_row[MAX_COR_TEM] = {0}, a_col[MAX_COR_TEM] = {0};
	uint8_t  len_a_cor = 0;

	// detect unique coordinates
	for (int a = 0; a < MAX_COR_TEM; a++)
	{
		uint8_t flag = 1;

		for (int b = 0; b < len_a_cor; b++)
		{
			uint32_t length = sqrt(pow(Imax->Temp.Row[a]-a_row[b], 2) + pow(Imax->Temp.Col[a]-a_col[b], 2));
			if (length < 10)
			{
				flag = 0;
			}
		}

		if (flag == 1)
		{
			a_row[len_a_cor] = Imax->Temp.Row[a];
			a_col[len_a_cor] = Imax->Temp.Col[a];
			len_a_cor++;
		}
	}

	uint16_t u_row[MAX_COR_TEM] = {0};
	uint8_t  len_u_row = 0;

	// get unique rows
	for (int a = 0; a < len_a_cor; a++)
	{
		uint8_t flag = 1;

		for (int b = 0; b < len_u_row; b++)
		{
			uint16_t length = sqrt( pow(a_row[a]-u_row[b], 2) );
			if (length < 10)
			{
				flag = 0;
			}
		}

		if (flag == 1)
		{
			u_row[len_u_row] = a_row[a];
			len_u_row++;
		}
	}

	uint16_t u_col[MAX_COR_TEM] = {0};
	uint8_t  len_u_col = 0;

	// get unique cols
	for (int a = 0; a < len_a_cor; a++)
	{
		uint8_t flag = 1;

		for (int b = 0; b < len_u_col; b++)
		{
			uint16_t length = sqrt( pow(a_col[a]-u_col[b], 2) );
			if (length < 10)
			{
				flag = 0;
			}
		}

		if (flag == 1)
		{
			u_col[len_u_col] = a_col[a];
			len_u_col++;
		}
	}

	// update result
	for (int index = 0; index < TRA_COR_TEM; index++)
	{
		Imax->Temp.OutputRow[index] = u_row[index] + TEMPLATE_V/2;
		Imax->Temp.OutputCol[index] = u_col[index] + TEMPLATE_H/2;
	}

	// sort result array
	sort_low_to_high(Imax->Temp.OutputRow, TRA_COR_TEM);
	sort_low_to_high(Imax->Temp.OutputCol, TRA_COR_TEM);

	// return ok
	return IMA_OK;
}

IMA_StatusTypeDef GetShouldersCoordinates(IMA_InitTypeDef* Imax, uint8_t Camera)
{
	uint16_t a_row[MAX_COR_MSK] = {0}, a_col[MAX_COR_MSK] = {0};
	uint8_t  len_a_cor = 0;
	int diff1 = 0, diff2 = 0;

	// detect unique coordinates
	for (int a = 0; a < MAX_COR_MSK; a++)
	{
		uint8_t flag = 1;

		for (int b = 0; b < len_a_cor  ; b++)
		{
			uint32_t length = sqrt(pow(Imax->Mask.Row[a]-a_row[b], 2) + pow(Imax->Mask.Col[a]-a_col[b], 2));
			if (length < 20)
			{
				flag = 0;
			}
		}
		if ((flag == 1) && (abs(Imax->Mask.Threshold - Imax->Mask.Val[a]) > 35))
		{
			a_row[len_a_cor] = Imax->Mask.Row[a];
			a_col[len_a_cor] = Imax->Mask.Col[a];
			len_a_cor++;
		}
	}

	// update result
	for (int index = 0; index < TRA_COR_MSK; index++)
	{
		Imax->Mask.OutputRow[index] = (a_row[index]* 2);
		Imax->Mask.OutputCol[index] = (a_col[index]* 2);
	}

	// sort result array
	sort_low_to_high(Imax->Mask.OutputRow, TRA_COR_MSK);
	sort_low_to_high(Imax->Mask.OutputCol, TRA_COR_MSK);

	// detect: shoulder 1
	for (int index = 6-1; index > 0; index--) Imax->Mask.Diff1[index] = Imax->Mask.Diff1[index-1];
	if (Imax->Mask.OutputCol[0] != 0)
	{
		switch(Camera)
		{
		case 0:
			diff1 = Imax->Mask.OutputCol[0] - Imax->Temp.OutputCol[0];
			break;
		case 1:
			diff1 = Imax->Temp.OutputCol[1] - Imax->Mask.OutputCol[0];
			break;
		}

		Imax->Mask.Diff1[0] = diff1;
		if (diff1 < 0) Imax->Mask.Diff1[0] = 0xFF;
	}
	else
	{
		Imax->Mask.Diff1[0] = 0x0FF;
	}

	// detect: shoulder 2
	for (int index = 6-1; index > 0; index--) Imax->Mask.Diff2[index] = Imax->Mask.Diff2[index-1];
	if (Imax->Mask.OutputCol[1] != 0)
	{
		switch(Camera)
		{
		case 0:
			diff2 = Imax->Mask.OutputCol[1] - Imax->Temp.OutputCol[0];
			break;
		case 1:
			diff2 = Imax->Temp.OutputCol[1] - Imax->Mask.OutputCol[1];
			break;
		}

		Imax->Mask.Diff2[0] = diff2;
		if (diff2 < 0) Imax->Mask.Diff2[0] = 0xFF;
	}
	else
	{
		Imax->Mask.Diff2[0] = 0x0FF;
	}

	// return ok
	return IMA_OK;
}

// memory management

void free_memory(IMA_InitTypeDef* Imax)
{
	if (Imax->Temp.Val != NULL) free(Imax->Temp.Val);
	if (Imax->Temp.Row != NULL) free(Imax->Temp.Row);
	if (Imax->Temp.Col != NULL) free(Imax->Temp.Col);

	if (Imax->Mask.Val != NULL) free(Imax->Mask.Val);
	if (Imax->Mask.Row != NULL) free(Imax->Mask.Row);
	if (Imax->Mask.Col != NULL) free(Imax->Mask.Col);
}

// helper functions

IMA_StatusTypeDef ConvertsImage(IMA_InitTypeDef* Imax, uint8_t Image[][8][8], uint16_t Length)
{
	uint16_t g_row = 0, g_col = 0, hor = 0, ver = 0, mcu = 0;
	uint8_t  m_col = 0, flg = 0, state = 1;

	switch (Length)
	{
	// 160 x 120
	case 0x050: 		// 080 (cr & cb)
		hor   = 0x050;	// pixels
		ver   = 0x040;	// pixels
		flg   = 0;		//

		m_col = 0x00A;	//

		mcu   = Length;	//

		break;
	case 0x140: 		// 320 (lu & lu)
		hor   = 0x0A0;	// pixels
		ver   = 0x080;	// pixels
		flg   = 1;		//

		m_col = 0x00A;	//

		mcu   = Length;	//

		break;

	// more resolutions

	default:
		return IMA_ERROR;
		break;
	}

	Imax->Image = (uint8_t **)calloc(ver, sizeof(uint8_t *));
	for (int index = 0; index < ver; index++)
	{
		Imax->Image[index] = (uint8_t *)calloc(hor, sizeof(uint8_t *));
	}

	for (int index = 0; index < mcu; index++)
	{
		switch (state)
		{
		case 1:

			// fill (loc 1)
			for (int row = 0; row < 8; row++)
			{
				for (int col = 0; col < 8; col++)
				{
					Imax->Image[row + g_row + 0][col + g_col + 0] = Image[index][row][col];
				}
			}

			// change state
			if (flg == 1)
			{
				state = 2;
			}
			else
			if (flg == 0)
			{
				state = 1;

				// check maximum cols
				if ((g_col/8) < m_col - 1)
				{
					g_col = g_col + 8;
				}
				else
				{
					g_col = 0;
					g_row = g_row + 8;
				}
			}

			break;
		case 2:

			// fill (loc 2)
			for (int row = 0; row < 8; row++)
			{
				for (int col = 0; col < 8; col++)
				{
					Imax->Image[row + g_row + 0][col + g_col + 8] = Image[index][row][col];
				}
			}

			// change state
			state = 3;

			break;
		case 3:

			// fill (loc 3)
			for (int row = 0; row < 8; row++)
			{
				for (int col = 0; col < 8; col++)
				{
					Imax->Image[row + g_row + 8][col + g_col + 0] = Image[index][row][col];
				}
			}

			// change state
			state = 4;

			break;
		case 4:

			// fill (loc 4)
			for (int row = 0; row < 8; row++)
			{
				for (int col = 0; col < 8; col++)
				{
					Imax->Image[row + g_row + 8][col + g_col + 8] = Image[index][row][col];
				}
			}

			// change state
			if (flg == 1)
			{
				state = 1;

				// check maximum cols
				if ((g_col/16) < m_col - 1)
				{
					g_col = g_col + 16;
				}
				else
				{
					g_col = 0;
					g_row = g_row + 16;
				}
			}

			break;
		default:
			// return error
			return IMA_ERROR;
			break;
		}
	}

	// return ok
	return IMA_OK;
}

IMA_StatusTypeDef MatchTemplate_SQDIFF(IMA_InitTypeDef* Imax, uint16_t Length)
{
	uint16_t Hor = 0, Ver = 0, Row = 0, Col = 0;

	// resolution
	switch (Length)
	{
	// 160 x 120
	case 0x050: 		  				// 080 (cr & cb)
		Hor   = 0x050 - TEMPLATE_H;		// pixels x
		Ver   = 0x040 - TEMPLATE_V;		// pixels y
		break;
	case 0x140:							// 160 (lu & lu)
		Hor   = 0x0A0 - TEMPLATE_H;		// pixels x
		Ver   = 0x080 - TEMPLATE_V;		// pixels y
		break;

	// more resolutions

	default:
		return IMA_ERROR;
		break;
	}

	// allocate memory
	Imax->Temp.Result = (uint32_t**)calloc(Ver, sizeof(uint32_t*));
	for (int index = 0; index < Ver; index++)
	{
		Imax->Temp.Result[index] = (uint32_t*)calloc(Hor, sizeof(uint32_t*));
	}

	// squared difference
	Row = 0;
	while (Row < Ver)
	{
		Col = 0;
		while (Col < Hor)
		{
			uint32_t sum1 = 0;
			uint32_t sum2 = 0;
			for (int a = 0; a < TEMPLATE_V; a++)
			{
				for (int b = 0; b < TEMPLATE_H; b++)
				{
					sum1 = sum1 + pow((TemplateMarker1[a][b] - Imax->Image[Row + a][Col + b]), 2);
					sum2 = sum2 + pow((TemplateMarker2[a][b] - Imax->Image[Row + a][Col + b]), 2);
				}
			}

			// save sum
			if (sum1 < sum2)
				Imax->Temp.Result[Row][Col] = sum1;
			else
				Imax->Temp.Result[Row][Col] = sum2;

			Col++;
		}
		Row++;
	}

	// find local minimums
	Imax->Temp.Val = calloc(MAX_COR_TEM, sizeof(uint32_t));
	Imax->Temp.Row = calloc(MAX_COR_TEM, sizeof(uint16_t));
	Imax->Temp.Col = calloc(MAX_COR_TEM, sizeof(uint16_t));
	find_local_minimum(Imax->Temp.Result, Ver, Hor, Imax->Temp.Val, Imax->Temp.Col, Imax->Temp.Row, MAX_COR_TEM);

	// free allocated memory
	for (int index = 0; index < Ver; index++)
	{
		free(Imax->Temp.Result[index]);
		Imax->Temp.Result[index] = NULL;
	}
	free(Imax->Temp.Result);
	Imax->Temp.Result = NULL;

	// return ok
	return IMA_OK;
}

void find_local_minimum(uint32_t** Input, uint16_t InputRows, uint16_t InputCols, uint32_t* Result, uint16_t* ResultCols, uint16_t *ResultRows, uint16_t ResultLength)
{
	uint16_t Row = 0, Col = 0;

	// initialise
	memset(Result, 0xFFFFFFFF, ResultLength*sizeof(uint32_t));

	// get minimums
	Row = 0;
	while (Row < InputRows)
	{
		Col = 0;
		while (Col < InputCols)
		{
			for (int a = 0; a < ResultLength; a++)
			{
				if (Input[Row][Col] < Result[a])
				{
					for (int b = ResultLength-1; b > a; b--)
					{
						Result[b] = Result[b-1];
						ResultRows[b] = ResultRows[b-1];
						ResultCols[b] = ResultCols[b-1];
					}

					// add new
					Result[a] = Input[Row][Col];
					ResultRows[a] = Row;
					ResultCols[a] = Col;

					break;
				}
			}
			Col++;
		}
		Row++;
	}
}

void find_local_maximum(uint32_t** Input, uint16_t InputRows, uint16_t InputCols, uint32_t* Result, uint16_t* ResultCols, uint16_t *ResultRows, uint16_t ResultLength)
{
	uint16_t Row = 0, Col = 0;

	Result = malloc(ResultLength*sizeof(uint32_t));
	memset(Result, 0xFFFFFFFF, ResultLength*sizeof(uint32_t));

	ResultRows = calloc(ResultLength, sizeof(uint16_t));
	ResultCols = calloc(ResultLength, sizeof(uint16_t));

	// get minimums
	Row = 0;
	while (Row < InputRows)
	{
		Col = 0;
		while (Col < InputCols)
		{
			for (int a = 0; a < ResultLength; a++)
			{
				if (Input[Row][Col] > Result[a])
				{
					for (int b = ResultLength-1; b > a; b--)
					{
						Result[b] = Result[b-1];
						ResultRows[b] = ResultRows[b-1];
						ResultCols[b] = ResultCols[b-1];
					}

					// add new
					Result[a] = Input[Row][Col];
					ResultRows[a] = Row;
					ResultCols[a] = Col;

					break;
				}
			}
			Col++;
		}
		Row++;
	}
}

void sort_low_to_high(uint16_t* Array, uint16_t Length)
{
	// low to high
	for (int a = 0; a < Length; a++)
	{
		for (int b = (a + 1); b < Length; b++)
		{
			if (Array[b] <= Array[a])
			{
				uint16_t temp = Array[a];
				Array[a] = Array[b];
				Array[b] = temp;
			}
		}
	}
}

void sort_high_to_low(uint16_t* Array, uint16_t Length)
{
	// high to low
	for (int a = 0; a < Length; a++)
	{
		for (int b = (a + 1); b < Length; b++)
		{
			if (Array[b] >= Array[a])
			{
				uint16_t temp = Array[a];
				Array[a] = Array[b];
				Array[b] = temp;
			}
		}
	}
}
