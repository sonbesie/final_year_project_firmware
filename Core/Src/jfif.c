/**
  ******************************************************************************
  * @file           : jfif.c
  * @brief          : The file running the jfif decoding algorithm.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
  */

#include "jfif.h"
#include "jfif_hard.h"

// private

DEC_StatusTypeDef SplitEncodedData(DEC_InitTypeDef *Decx)
{
	for (int index = 0; index < Decx->ImageLength; index++)
	{
		// quantisation table
		if ((Decx->ImageData[index+0] == 0xFF) && (Decx->ImageData[index+1] == 0xDB))
		{
			int length = 0, size = 0, prev = 0;
			uint16_t upper = (Decx->ImageData[index+2] << 8);
			uint16_t lower = (Decx->ImageData[index+3] << 0);
			length = ((int)(upper | lower));

			for (int counter = 0; counter < QUANTIS_TABLE_LENGTH; counter++)
			{
				// determine the size of each quantis table
				size = QUANTIS_TREE_LENGTH*QUANTIS_TREE_LENGTH;

				// allocate the memory and store the table size
				Decx->QuantisTable.length[counter] = size;
				Decx->QuantisTable.quantisTable[counter] = malloc(size*sizeof(uint8_t));

				for (int position = 0; position < size; position++)
				{
					Decx->QuantisTable.quantisTable[counter][position] = Decx->ImageData[(prev+index+QUANTIS_OFFSET)+(position+counter)];
				}

				// update the running size counter
				prev = prev + size;
			}
			index = index + length;
		}

		// dc and ac huffman tables
		if ((Decx->ImageData[index+0] == 0xFF) && (Decx->ImageData[index+1] == 0xC4))
		{
			int length = 0, size = 0, prev = 0;
			uint16_t upper = (Decx->ImageData[index+2] << 8);
			uint16_t lower = (Decx->ImageData[index+3] << 0);
			length = ((int)(upper | lower));

			for (int counter = 0; counter < HUFFMAN_TABLE_LENGTH; counter++)
			{
				// determine the size of each huffman table
				size = 0;
				for (int position = 0; position < HUFFMAN_TREE_LENGTH; position++)
				{
					size = size + Decx->ImageData[(prev+index+5)+(position+counter)];
				}
				size = size + HUFFMAN_TREE_LENGTH;

				// allocate the memory and store the table size
				Decx->HuffmanTable.length[counter] = size;
				Decx->HuffmanTable.huffmanTable[counter] = malloc(size*sizeof(uint8_t));

				for (int position = 0; position < size; position++)
				{
					Decx->HuffmanTable.huffmanTable[counter][position] = Decx->ImageData[(prev+index+HUFFMAN_OFFSET)+(position+counter)];
				}

				// update the running size counter
				prev = prev + size;
			}
			index = index + length;
		}

		// break at the end of tables
		if ((Decx->ImageData[index+0] == 0xFF) && (Decx->ImageData[index+1] == 0xDA))
		{
			break;
		}
	}

	return DEC_OK;
}

DEC_StatusTypeDef DecodeHuffman(DEC_InitTypeDef *Decx)
{
	for (int index = 0; index < HUFFMAN_TABLE_LENGTH; index++)
	{
		int code = Decx->HuffmanTable.length[index]-HUFFMAN_TREE_LENGTH;
		Decx->HuffmanTable.tree_length[index] = code;
		Decx->HuffmanTable.code_length[index] = code;
		Decx->HuffmanTable.leng_length[index] = code;
		Decx->HuffmanTable.tree[index] = malloc(code*sizeof(uint16_t));	// tree
		Decx->HuffmanTable.code[index] = malloc(code*sizeof(uint16_t));	// code
		Decx->HuffmanTable.leng[index] = malloc(code*sizeof(uint16_t));


		// generate the tree
		int global = 0;
		for (int position = 0; position < HUFFMAN_TREE_LENGTH; position++)
		{
			int branches = 0, counter = 1;
			for (int a = position-1; a > -1; a--)
			{
				branches = branches + Decx->HuffmanTable.huffmanTable[index][a] * pow(2, counter);
				++counter;
			}
			for (int b = branches; b < branches + Decx->HuffmanTable.huffmanTable[index][position]; b++)
			{
				Decx->HuffmanTable.tree[index][global] = b;
				Decx->HuffmanTable.leng[index][global] = position+1;	// store the number of bits
				++global;
			}
		}

		// generate the code
		for (int position = 0; position < code; position++)
		{
			Decx->HuffmanTable.code[index][position] = Decx->HuffmanTable.huffmanTable[index][position+HUFFMAN_TREE_LENGTH];
		}
	}

	// free unused memory
	for (int index = 0; index < HUFFMAN_TABLE_LENGTH; index++)
	{
		free(Decx->HuffmanTable.huffmanTable[index]);
		Decx->HuffmanTable.huffmanTable[index] = NULL;
		Decx->HuffmanTable.length[index] = 0;
	}

	// set initialised to true
	Decx->Huffman = 1;

	return DEC_OK;
}

DEC_StatusTypeDef DecodeQuantis(DEC_InitTypeDef *Decx)
{
	// generate the data
	for (int index = 0; index < QUANTIS_TABLE_LENGTH; index++)
	{
		for (int position = 0; position < (QUANTIS_TREE_LENGTH*QUANTIS_TREE_LENGTH); position++)
		{
			if (QUANTIS_TREE_LENGTH == 8)
			{
				Decx->QuantisTable.code[index][ROW_08X08[position]][COL_08X08[position]] =
						Decx->QuantisTable.quantisTable[index][position];
			}
		}
	}

	// free unused memory
	for (int index = 0; index < QUANTIS_TABLE_LENGTH; index++)
	{
		free(Decx->QuantisTable.quantisTable[index]);
		Decx->QuantisTable.quantisTable[index] = NULL;
		Decx->QuantisTable.length[index] = 0;
	}

	// set initialised to false
	Decx->Quantis = 1;

	return DEC_OK;
}

DEC_StatusTypeDef DecodePicture(DEC_InitTypeDef *Decx)
{
	if ((Decx->Huffman) && (Decx->Quantis))
	{
		// the total number of bits and bytes processed by the function
		int bitcounter, bytecounter;

		// row & col of the result
		uint16_t mcu_row_lu = 0, mcu_row_cr = 0, mcu_row_cb = 0, mcu_col = 0;
		// jfif special variables
		uint16_t zrl = 0, final =  0, mask = 0, TempData[64] = {0};

		// mcu counters
		uint8_t mc = 0, in = 0;
		// mc ->
		// in ->

		// pre, post and target length counter
		uint8_t pre = 0, pst = 0, len = 0;
		// pre ->
		// pst ->
		// length ->

		// flags
		uint8_t lu = 1, cr = 0, cb = 0, dc = 1, st = 0;
		// lu ->
		// cr ->
		// cb ->
		// dc -> use the ac or dc huffman decoding tables
		// st -> start the data saving process

		// save
		uint8_t sv_lu = 0, sv_cr = 0, sv_cb = 0;
		// sv_lu -> save lu
		// sv_cr -> save cr
		// sv_cb -> save cb

		bytecounter = 0;
		while ((bytecounter < Decx->ImageLength)) // loop through all the bytes
		{

			// zrl
			if ((Decx->ImageData[bytecounter-1] == 0xFF) &&
				(Decx->ImageData[bytecounter+0] == 0x00)
				)
			{
				bytecounter = bytecounter + 1;
			}

			// end of the file
			if ((Decx->ImageData[bytecounter+0] == 0xFF) &&
				(Decx->ImageData[bytecounter+1] == 0xD9)
				)
			{
				break;
			}

			bitcounter = 0;
			while ((bitcounter < 8)) // loop through all the bits in each byte
			{
				// add new bit to the temporary data buffer
				uint8_t comp1 = ((Decx->ImageData[bytecounter+0] & (uint8_t)pow(2, 7-bitcounter)) >> (7-bitcounter));
				final = final | comp1;

				if (st == 1) // get data when flag is set
				{
					// create the mask
					mask = mask + pow(2, pst);

					// advance pst bit counter
					++pst;

					// is the length of the captured data equal to the requested length
					if (pst == len)
					{
						// save the data
						if (((final >> (pst-1)) & 0b1) == 0b0)
							TempData[mcu_col] = -1* (final ^ mask);
						else
							TempData[mcu_col]  = final;
						++mcu_col;

						// toggle dc flag
						if ((dc == 1) && (sv_lu == 0) && (sv_cr == 0) && (sv_cb == 0))
						{
							if (lu == 1)
								in = 1;
							else
							if (cr == 1)
								in = 3;
							else
							if (cb ==1)
								in = 3;
							dc = 0;
						}

						// set the capture data flag
						st = 0;
						// clear the mask
						mask = 0;
						// clear the temporary data buffer
						final = 0;
						// clear counters
						pst = 0;
						pre = 0;
						len = 0;
						zrl = 0;
					}
				}
				else // compare current buffer with huffman table values
				{
					// advance the pre bit counter
					++pre;

					// for all elements in the huffman table
					for (int b = 0; b < Decx->HuffmanTable.leng_length[in]; b++)
					{
						if ((pre == Decx->HuffmanTable.leng[in][b]) && (final == Decx->HuffmanTable.tree[in][b]))
						{
							// set the capture data flag
							st = 0;
							// clear the mask
							mask = 0;
							// clear the temporary data buffer
							final = 0;
							// clear counters
							pst = 0;
							pre = 0;
							len = 0;
							zrl = 0;

							if (sv_lu == 1)
							{
								// save luminance table
								Decx->McuTable.mcu_lu_length[mcu_row_lu] = mcu_col;
								Decx->McuTable.mcu_lu[mcu_row_lu] = calloc(Decx->McuTable.mcu_lu_length[mcu_row_lu], sizeof(uint16_t));
								memcpy(Decx->McuTable.mcu_lu[mcu_row_lu], TempData, Decx->McuTable.mcu_lu_length[mcu_row_lu]*sizeof(uint16_t));
								mcu_row_lu++;

								// set save
								sv_lu = 0;
								// clear the data array
								memset(TempData, 0, sizeof(TempData));
								// reset the col counter of the temp data array
								mcu_col = 0;
							}
							else
							if (sv_cr == 1)
							{
								// save chrominance 1 table
								Decx->McuTable.mcu_cr_length[mcu_row_cr] = mcu_col;
								Decx->McuTable.mcu_cr[mcu_row_cr] = calloc(Decx->McuTable.mcu_cr_length[mcu_row_cr], sizeof(uint16_t));
								memcpy(Decx->McuTable.mcu_cr[mcu_row_cr], TempData, Decx->McuTable.mcu_cr_length[mcu_row_cr]*sizeof(uint16_t));
								mcu_row_cr++;

								// set save
								sv_cr = 0;
								// clear the data array
								memset(TempData, 0, sizeof(TempData));
								// reset the col counter of the temp data array
								mcu_col = 0;
							}
							else
							if (sv_cb == 1)
							{
								// save chrominance 2 table
								Decx->McuTable.mcu_cb_length[mcu_row_cb] = mcu_col;
								Decx->McuTable.mcu_cb[mcu_row_cb] = calloc(Decx->McuTable.mcu_cb_length[mcu_row_cb], sizeof(uint16_t));
								memcpy(Decx->McuTable.mcu_cb[mcu_row_cb], TempData, Decx->McuTable.mcu_cb_length[mcu_row_cb]*sizeof(uint16_t));
								mcu_row_cb++;

								// set save
								sv_cb = 0;
								// clear the data array
								memset(TempData, 0, sizeof(TempData));
								// reset the col counter of the temp data array
								mcu_col = 0;
							}

							if ((Decx->HuffmanTable.code[in][b] == 0x00) && (dc == 1))
							{
								// toggle dc flag
								if ((dc == 1) && (sv_lu == 0) && (sv_cr == 0) && (sv_cb == 0))
								{
									if (lu == 1)
										in = 1;
									else
									if (cr == 1)
										in = 3;
									else
									if (cb ==1)
										in = 3;
									dc = 0;
								}

								// add the dc zero to the data array
								TempData[mcu_col] = 0;
								++mcu_col;

								// set the capture data flag
								st = 0;
							}
							else
							if ((Decx->HuffmanTable.code[in][b] == 0xF0) && (dc == 0))
							{
								len = 00;
								zrl = 15;

								// zrl
								if (zrl > 0)
								{
									for (int times = 0; times < zrl; times++)
									{
										TempData[mcu_col] = 0;
										++mcu_col;
									}
									zrl = 0;
								}

								// add the dc zero to the data array
								TempData[mcu_col] = 0;
								++mcu_col;

								// set the capture data flag
								st = 0;
							}
							else
							{
								len = (Decx->HuffmanTable.code[in][b] & 0b1111);
								zrl = (Decx->HuffmanTable.code[in][b] & 0b1111111111110000) >> 4;

								// zrl
								if (zrl > 0)
								{
									for (int times = 0; times < zrl; times++)
									{
										TempData[mcu_col] = 0;
										++mcu_col;
									}
									zrl = 0;
								}

								// set the capture data flag
								if (Decx->HuffmanTable.code[in][b] == 0x00)
									st = 0;
								else
									st = 1;

								// eob
								if ( ((mcu_col + 1) == 64) || (Decx->HuffmanTable.code[in][b] == 0x00) )
								{
									// advance to next huffman decoding table
									++mc;
									switch (mc)
									{
										case 1:
										{
											lu = 1;	// lum 2
											cr = 0;
											cb = 0;

											dc = 1;
											in = 0;

											sv_lu = 1;

											break;
										}
										case 2:
										{
											lu = 1;	// lum 3
											cr = 0;
											cb = 0;

											dc = 1;
											in = 0;

											sv_lu = 1;

											break;
										}
										case 3:
										{
											lu = 1; // lum 4
											cr = 0;
											cb = 0;

											dc = 1;
											in = 0;

											sv_lu = 1;

											break;
										}
										case 4:
										{
											lu = 0;	// chr 1
											cr = 1;
											cb = 0;

											dc = 1;
											in = 2;

											sv_lu = 1;

											break;
										}
										case 5:
										{
											lu = 0;	// chr 2
											cr = 0;
											cb = 1;

											dc = 1;
											in = 2;

											sv_cr = 1;

											break;
										}
										case 6:
										{
											lu = 1;	// lum 1
											cr = 0;
											cb = 0;

											dc = 1;
											in = 0;

											sv_cb = 1;

											mc = 0;	// reset the mcu counter
											break;
										}
									}
								}
							}

							// break the loop to save execution time
							break;
						}
					}
				}

				// increment shift register
				final = final << 1;

				// increment the local bitcounter
				++bitcounter;
			}
			// increment the local byte bytecounter
			bytecounter = bytecounter + 1;
		}

		// save last table
		Decx->McuTable.mcu_cb_length[mcu_row_cb] = mcu_col;
		Decx->McuTable.mcu_cb[mcu_row_cb] = calloc(Decx->McuTable.mcu_cb_length[mcu_row_cb], sizeof(uint16_t));
		memcpy(Decx->McuTable.mcu_cb[mcu_row_cb], TempData, Decx->McuTable.mcu_cb_length[mcu_row_cb]*sizeof(uint16_t));
		mcu_row_cb++;

		// decoding error
		if ((mcu_row_lu != 320) || (mcu_row_cr != 80) || (mcu_row_cr != 80))
		{
			return DEC_ERROR;
		}
		else // decoding success
		{
			return DEC_OK;
		}
	}
	else
	{
		// if the huffman and quantisation tables have not been initialised return error
		// these arrays need to be initialised for this function to work properly
		return DEC_ERROR;
	}
}

DEC_StatusTypeDef GenerateTables(DEC_InitTypeDef * Decx)
{
	if ((Decx->Huffman) && (Decx->Quantis))
	{

		InverseDCT(Decx, MCU*MCU_TABLES_LU, LU);
		InverseDCT(Decx, MCU*MCU_TABLES_CR, CR);
		InverseDCT(Decx, MCU*MCU_TABLES_CB, CB);

		return DEC_OK;
	}
	else
	{
		// if the huffman and quantisation tables have not been initialised return error
		// these arrays need to be initialised for this function to work properly
		return DEC_ERROR;
	}
}

DEC_StatusTypeDef InverseDCT(DEC_InitTypeDef * Decx, uint16_t Length, uint8_t Type)
{
	int16_t TempData[Length][8][8];
	float sum = 0.0, average = 0.0, p_average = 0.0;
	float K[8][8] = {0};

	// clear tempdata
	memset(TempData, 0, sizeof(TempData));

	// generate the data
	for (int index = 0; index < Length; index++)
	{
		for (int position = 0; position < (QUANTIS_TREE_LENGTH*QUANTIS_TREE_LENGTH); position++)
		{
			switch (Type)
			{
			case LU:
				// break early
				if (position > Decx->McuTable.mcu_lu_length[index]-1)
				{
					break;
				}
				else
				{
					TempData[index][ROW_08X08[position]][COL_08X08[position]] =
					(Decx->McuTable.mcu_lu[index][position] * Decx->QuantisTable.code[0][ROW_08X08[position]][COL_08X08[position]]);
					break;
				}
			case CR:
				// break early
				if (position > Decx->McuTable.mcu_cr_length[index]-1)
				{
					break;
				}
				else
				{
					TempData[index][ROW_08X08[position]][COL_08X08[position]] =
					(Decx->McuTable.mcu_cr[index][position] * Decx->QuantisTable.code[1][ROW_08X08[position]][COL_08X08[position]]);
					break;
				}
			case CB:
				// break early
				if (position > Decx->McuTable.mcu_cb_length[index]-1)
				{
					break;
				}
				else
				{
					TempData[index][ROW_08X08[position]][COL_08X08[position]] =
					(Decx->McuTable.mcu_cb[index][position] * Decx->QuantisTable.code[1][ROW_08X08[position]][COL_08X08[position]]);
					break;
				}
			}
		}
	}

	// calculate the idct
	for (int index = 0; index < Length; index++)
	{
		// zero the average
		average = 0;

		// first multiplication
		for (int a = 0; a < 8; a++)
		{
			for (int b = 0; b < 8; b++)
			{
				sum = 0.0;
				for (int c = 0; c < 8; c++)
				{
					sum = sum + I[a][c] * TempData[index][c][b];
				}
				K[a][b] = sum;
			}
		}

		// second multiplication
		for (int a = 0; a < 8; a++)
		{
			for (int b = 0; b < 8; b++)
			{
				sum = 0.0;
				for (int c = 0; c < 8; c++)
				{
					sum = sum + K[a][c] * T[c][b];
				}
				switch (Type)
				{
				case LU:
					Decx->PixelsLu[index][a][b] = (sum + p_average + 128.0);
					break;
				case CR:
					Decx->PixelsCr[index][a][b] = (sum + p_average + 128.0);
					break;
				case CB:
					Decx->PixelsCb[index][a][b] = (sum + p_average + 128.0);
					break;
				}
				average = average + p_average + sum;
			}
		}

		// calculate the running average
		average = average / 64.0;
		p_average = average;

	}

	// free unused memory
	for (int index = 0; index < Length; index++)
	{
		switch (Type)
		{
		case LU:
			free(Decx->McuTable.mcu_lu[index]);
			break;
		case CR:
			free(Decx->McuTable.mcu_cr[index]);
			break;
		case CB:
			free(Decx->McuTable.mcu_cb[index]);
			break;
		}
	}

	return DEC_OK;
}

// user

DEC_StatusTypeDef InitDec(DEC_InitTypeDef *Decx)
{
	// initialise table length
	for (int index = 0; index < QUANTIS_TABLE_LENGTH; index++)
	{
		Decx->QuantisTable.length[index] = 0;
	}
	// initialise table length
	for (int index = 0; index < HUFFMAN_TABLE_LENGTH; index++)
	{
		Decx->HuffmanTable.length[index] = 0;
	}

	// set initialised to false
	Decx->Quantis = 0;
	Decx->Huffman = 0;

	return DEC_OK;
}

DEC_StatusTypeDef Decode(DEC_InitTypeDef *Decx)
{
	// decode the file format
	if (DecodePicture(Decx) != DEC_OK)
	{
		// return error
		return DEC_ERROR;
	}

	// generate the final image tables
	if (GenerateTables(Decx) != DEC_OK)
	{
		// return error
		return DEC_ERROR;
	}

	// return ok
	return DEC_OK;
}
