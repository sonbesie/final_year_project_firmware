/**
  ******************************************************************************
  * @file           : image_hard.h
  * @brief          : This file contains hardcoded data for the image processing
  * 				  algorithm.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ******************************************************************************
*/

#ifndef INC_IMAGE_HARD_H_
#define INC_IMAGE_HARD_H_

#define TEMPLATE_H 9
#define TEMPLATE_V 7

////////////////////////////////////////////////////////////////////////
// pattern 1
const uint8_t TemplateMarker1[TEMPLATE_V][TEMPLATE_H] = {
		{196,204,201,189,198,193,187,208,199},
		{198,207,178, 98, 91,119,187,188,198},
		{205,194, 73,103,102, 97,137,194,195},
		{202,198, 92, 90,105,102,109,198,195},
		{202,200,140,105, 95, 89,142,200,199},
		{207,203,203,170,143,188,191,207,201},
		{200,204,207,201,196,199,205,197,201}
};

// pattern 2
const uint8_t TemplateMarker2[TEMPLATE_V][TEMPLATE_H] = {
		{247,243,244,227,226,231,238,248,241},
		{242,249,233,171,156,151,231,236,243},
		{245,243,152,163,163,152,174,238,242},
		{245,251,157,150,149,161,146,245,237},
		{243,244,239,142,148,142,195,237,234},
		{248,245,243,234,204,213,233,239,231},
		{244,238,234,240,239,227,227,223,227}
};
////////////////////////////////////////////////////////////////////////


#endif /* INC_IMAGE_HARD_H_ */
