/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2017 NXP
 *
 ******************************************************************************
 *
 * edid_parser.c
 *
 ******************************************************************************
 */

#include "edid_parser.h"

static EDID_PARSER_RESULT edid_parse_dtd(S_DTD_DATA *descriptor,
					 unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;

	descriptor->header.type = DESCRIPTOR_TYPE_DTD;
	descriptor->header.tag = 0;

	descriptor->pixel_clock = raw_data[raw_data_index];
	descriptor->pixel_clock +=
	    (unsigned short)raw_data[raw_data_index + 1] << 8;

	descriptor->horizontal_addressable_video = raw_data[raw_data_index + 2];
	descriptor->horizontal_addressable_video +=
	    ((unsigned short)raw_data[raw_data_index + 4] & 0xF0) << 4;
	descriptor->horizontal_blanking = raw_data[raw_data_index + 3];
	descriptor->horizontal_blanking +=
	    ((unsigned short)raw_data[raw_data_index + 4] & 0x0F) << 8;

	descriptor->vertical_addressable_video = raw_data[raw_data_index + 5];
	descriptor->vertical_addressable_video +=
	    ((unsigned short)raw_data[raw_data_index + 7] & 0xF0) << 4;
	descriptor->vertical_blanking = raw_data[raw_data_index + 6];
	descriptor->vertical_blanking +=
	    ((unsigned short)raw_data[raw_data_index + 7] & 0x0F) << 8;

	descriptor->horizontal_front_porch = raw_data[raw_data_index + 8];
	descriptor->horizontal_front_porch +=
	    ((unsigned short)raw_data[raw_data_index + 11] & 0xC0) << 2;
	descriptor->horizontal_sync_pulse_width = raw_data[raw_data_index + 9];
	descriptor->horizontal_sync_pulse_width +=
	    ((unsigned short)raw_data[raw_data_index + 11] & 0x30) << 4;

	descriptor->vertical_front_porch =
	    (raw_data[raw_data_index + 10] & 0xF0) >> 4;
	descriptor->vertical_front_porch +=
	    (raw_data[raw_data_index + 11] & 0x0C) << 2;
	descriptor->vertical_sync_pulse_width =
	    raw_data[raw_data_index + 10] & 0x0F;
	descriptor->vertical_sync_pulse_width +=
	    (raw_data[raw_data_index + 11] & 0x03) << 4;

	descriptor->horizontal_addressable_video_image_size =
	    raw_data[raw_data_index + 12];
	descriptor->horizontal_addressable_video_image_size +=
	    ((unsigned short)raw_data[raw_data_index + 14] & 0xF0) << 4;
	descriptor->vertical_addressable_video_image_size =
	    raw_data[raw_data_index + 13];
	descriptor->vertical_addressable_video_image_size +=
	    ((unsigned short)raw_data[raw_data_index + 14] & 0x0F) << 8;

	descriptor->horizontal_border = raw_data[raw_data_index + 15];
	descriptor->vertical_border = raw_data[raw_data_index + 16];

	descriptor->signal_features = raw_data[raw_data_index + 17];

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_serial_number(S_SERIAL_NUMBER_DATA *
						   descriptor,
						   unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;
	int idx;

	descriptor->header.type = DESCRIPTOR_TYPE_SERIAL_NUMBER;
	descriptor->header.tag = 0xFF;

	for (idx = 0; idx < 13; idx++)
		descriptor->serial_number[idx] =
		    raw_data[raw_data_index + 5 + idx];

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_data_string(S_DATA_STRING_DATA *
						 descriptor,
						 unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;
	int idx;

	descriptor->header.type = DESCRIPTOR_TYPE_DATA_STRING;
	descriptor->header.tag = 0xFE;
	for (idx = 0; idx < 13; idx++)
		descriptor->data_string[idx] =
		    raw_data[raw_data_index + 5 + idx];

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_range_limits(S_RANGE_LIMITS_DATA *
						  descriptor,
						  unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;
	S_RANGE_LIMITS_VIDEO_TIMING_SECONDARY_GTF *timing_type_gtf;
	S_RANGE_LIMITS_VIDEO_TIMING_CVT *timing_type_cvt;

	descriptor->header.type = DESCRIPTOR_TYPE_RANGE_LIMITS;
	descriptor->header.tag = 0xFD;

	descriptor->offset_flags = raw_data[raw_data_index + 4];
	descriptor->min_vertical_rate = raw_data[raw_data_index + 5];
	descriptor->max_vertical_rate = raw_data[raw_data_index + 6];
	descriptor->min_horizontal_rate = raw_data[raw_data_index + 7];
	descriptor->max_horizontal_rate = raw_data[raw_data_index + 8];
	descriptor->max_pixel_clock = raw_data[raw_data_index + 9];

	switch (raw_data[raw_data_index + 10]) {
	case 0x00:
		descriptor->type = VIDEO_TIMING_DEFAULT_GTF;
		break;
	case 0x01:
		descriptor->type = VIDEO_TIMING_RANGE_LIMITS_ONLY;
		break;
	case 0x02:
		descriptor->type = VIDEO_TIMING_SECONDARY_GTF;
		timing_type_gtf = (S_RANGE_LIMITS_VIDEO_TIMING_SECONDARY_GTF *) descriptor->suport_flags;
		timing_type_gtf->start_break_frequency =
		    raw_data[raw_data_index + 12];
		timing_type_gtf->c = raw_data[raw_data_index + 13];
		timing_type_gtf->m = raw_data[raw_data_index + 14];
		timing_type_gtf->m +=
		    (unsigned short)raw_data[raw_data_index + 15] << 8;
		timing_type_gtf->k = raw_data[raw_data_index + 16];
		timing_type_gtf->j = raw_data[raw_data_index + 17];
		break;
	case 0x04:
		descriptor->type = VIDEO_TIMING_CVT;
		timing_type_cvt = (S_RANGE_LIMITS_VIDEO_TIMING_CVT *) descriptor->suport_flags;
		timing_type_cvt->cvt_version = raw_data[raw_data_index + 11];
		timing_type_cvt->additional_pixel_clock_precision =
		    raw_data[raw_data_index + 12] >> 2;
		timing_type_cvt->max_active_pixels =
		    raw_data[raw_data_index + 13];
		timing_type_cvt->max_active_pixels +=
		    (unsigned short)(raw_data[raw_data_index + 12] & 0x03) << 8;
		timing_type_cvt->supported_ar =
		    raw_data[raw_data_index + 14] >> 3;
		timing_type_cvt->preferred_ar =
		    raw_data[raw_data_index + 15] >> 5;
		timing_type_cvt->blanking_support =
		    (raw_data[raw_data_index + 15] & 0x18) >> 3;
		timing_type_cvt->supported_scalling =
		    raw_data[raw_data_index + 16] >> 4;
		timing_type_cvt->preferred_vertical_refresh_rate =
		    raw_data[raw_data_index + 17];
		break;
	}

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_product_name(S_PRODUCT_NAME_DATA *
						  descriptor,
						  unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;
	int idx;

	descriptor->header.type = DESCRIPTOR_TYPE_PRODUCT_NAME;
	descriptor->header.tag = 0xFC;
	for (idx = 0; idx < 13; idx++)
		descriptor->product_name[idx] =
		    raw_data[raw_data_index + 5 + idx];

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_color_point(S_COLOR_POINT_DATA *
						 descriptor,
						 unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;

	descriptor->header.type = DESCRIPTOR_TYPE_COLOR_POINT;
	descriptor->header.tag = 0xFB;
	descriptor->white_point_index_1 = raw_data[raw_data_index + 5];
	descriptor->white_x_1 = (raw_data[raw_data_index + 6] & 0x0C) >> 2;
	descriptor->white_x_1 +=
	    (unsigned short)raw_data[raw_data_index + 7] << 2;
	descriptor->white_y_1 = raw_data[raw_data_index + 6] & 0x03;
	descriptor->white_y_1 +=
	    (unsigned short)raw_data[raw_data_index + 8] << 2;
	descriptor->gamma_1 = raw_data[raw_data_index + 9];

	descriptor->white_point_index_2 = raw_data[raw_data_index + 10];
	descriptor->white_x_2 = (raw_data[raw_data_index + 11] & 0x0C) >> 2;
	descriptor->white_x_2 +=
	    (unsigned short)raw_data[raw_data_index + 12] << 2;
	descriptor->white_y_2 = raw_data[raw_data_index + 11] & 0x03;
	descriptor->white_y_2 +=
	    (unsigned short)raw_data[raw_data_index + 13] << 2;
	descriptor->gamma_2 = raw_data[raw_data_index + 14];

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_standard_timing(S_STANDARD_TIMING_DATA *
						     descriptor,
						     unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;
	int idx;

	descriptor->header.type = DESCRIPTOR_TYPE_STANDARD_TIMING;
	descriptor->header.tag = 0xFA;
	for (idx = 0; idx < 6; idx++) {
		descriptor->standard_timings[idx] =
		    raw_data[raw_data_index + 5 + 2 * idx];
		descriptor->standard_timings[idx] +=
		    (unsigned short)raw_data[raw_data_index + 5 + 2 * idx + 1];
	}

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_color_management(S_COLOR_MANAGEMENT_DATA *
						      descriptor,
						      unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;

	descriptor->header.type = DESCRIPTOR_TYPE_COLOR_MANAGEMENT;
	descriptor->header.tag = 0xF9;

	descriptor->version = raw_data[raw_data_index + 5];

	descriptor->red_a3 = raw_data[raw_data_index + 6];
	descriptor->red_a3 += (unsigned short)raw_data[raw_data_index + 7] << 8;
	descriptor->red_a2 = raw_data[raw_data_index + 8];
	descriptor->red_a2 += (unsigned short)raw_data[raw_data_index + 9] << 8;

	descriptor->green_a3 = raw_data[raw_data_index + 10];
	descriptor->green_a3 +=
	    (unsigned short)raw_data[raw_data_index + 11] << 8;
	descriptor->green_a2 = raw_data[raw_data_index + 12];
	descriptor->green_a2 +=
	    (unsigned short)raw_data[raw_data_index + 13] << 8;

	descriptor->blue_a3 = raw_data[raw_data_index + 14];
	descriptor->blue_a3 +=
	    (unsigned short)raw_data[raw_data_index + 15] << 8;
	descriptor->blue_a2 = raw_data[raw_data_index + 16];
	descriptor->blue_a2 +=
	    (unsigned short)raw_data[raw_data_index + 17] << 8;

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_cvt_timing_codes(S_CVT_TIMING_CODES_DATA *
						      descriptor,
						      unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;
	int idx;

	descriptor->header.type = DESCRIPTOR_TYPE_CVT_TIMING_CODES;
	descriptor->header.tag = 0xF8;
	descriptor->version = raw_data[raw_data_index + 5];

	for (idx = 0; idx < 4; idx++) {
		descriptor->addressable_lines[idx] =
		    raw_data[raw_data_index + 6 + idx * 3];
		descriptor->addressable_lines[idx] +=
		    (unsigned short)(raw_data[raw_data_index + 7 + idx * 3] &
				     0xF0) << 4;
		descriptor->aspect_ratio[idx] =
		    (raw_data[raw_data_index + 7 + idx * 3] & 0x0C) >> 2;
		descriptor->preferred_vertical_rate[idx] =
		    (raw_data[raw_data_index + 8 + idx * 3] & 0x60) >> 5;
		descriptor->supported_vertical_rate_and_blanking[idx] =
		    raw_data[raw_data_index + 8 + idx * 3] & 0x1F;
	}

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT
edid_parse_established_timings_3(S_ESTABLISHED_TIMINGS_3_DATA *descriptor,
				 unsigned char *raw_data)
{
	unsigned int raw_data_index = 0;
	int idx;

	descriptor->header.type = DESCRIPTOR_TYPE_ESTABLISHED_TIMINGS_3;
	descriptor->header.tag = 0xF7;
	descriptor->version = raw_data[raw_data_index + 5];
	for (idx = 0; idx < 6; idx++) {
		descriptor->established_timings[idx] =
		    raw_data[raw_data_index + 6 + idx];
	}

	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT edid_parse_dummy(S_DUMMY_DATA *descriptor,
					   unsigned char *raw_data)
{
	descriptor->header.type = DESCRIPTOR_TYPE_DUMMY;
	descriptor->header.tag = 0x10;
	return EDID_PARSER_SUCCESS;
}

static EDID_PARSER_RESULT
edid_parse_manufacturer_specific(S_MANUFACTURER_SPECIFIC_DATA *descriptor,
				 unsigned char *raw_data, unsigned char tag)
{
	descriptor->header.type = DESCRIPTOR_TYPE_MANUFACTURER_SPECIFIC;
	descriptor->header.tag = tag;

	return EDID_PARSER_SUCCESS;
}

EDID_PARSER_RESULT edid_parse(S_EDID_DATA *edid, unsigned char *raw_data,
			      unsigned int len)
{
	unsigned int raw_data_index = 0;
	unsigned char sum = 0;
	unsigned int descriptor_index;
	unsigned char tag;

	/* CHECK SUM OF BYTES IN BLOCK0 */
	for (raw_data_index = 0; raw_data_index < EDID_LENGTH; raw_data_index++)
		sum += raw_data[raw_data_index];

	if (sum != 0)
		return EDID_PARSER_ERROR;

	/* READ HEADER */
	for (raw_data_index = 0; raw_data_index < EDID_HEADER_LENGTH;
	     raw_data_index++)
		edid->header[raw_data_index] = raw_data[raw_data_index];

	/* READ VENDOR & PRODUCT IDENTIFICATION */
	/* manufacturer name */
	edid->manufacturer_name[0] = ((raw_data[8] & 0x7C) >> 2) + 0x40;
	edid->manufacturer_name[1] =
	    ((raw_data[8] & 0x03) << 3) + ((raw_data[9] & 0xE0) >> 5) + 0x40;
	edid->manufacturer_name[2] = ((raw_data[9] & 0x1F)) + 0x40;
	edid->manufacturer_name[3] = 0;

	/* product code */
	edid->product_code = (raw_data[10]);
	edid->product_code += ((unsigned short)raw_data[11]) << 8;

	/* serial number */
	edid->serial_number = raw_data[12];
	edid->serial_number += (unsigned int)raw_data[13] << 8;
	edid->serial_number += (unsigned int)raw_data[14] << 16;
	edid->serial_number += (unsigned int)raw_data[15] << 24;

	/* week of manufacture */
	edid->week = raw_data[16];

	/* year of manufacture */
	edid->year = raw_data[17];

	/* EDID STRUCTURE VERSION & REVISION */
	edid->edid_version = ((unsigned short)raw_data[18] << 8) + raw_data[19];

	/* BASIC DISPLAY PARAMETERS AND FEATURES
	* video input definition */
	edid->video_input_definition = raw_data[20];

	/* horizontal screen size */
	edid->horizontal_size = raw_data[21];

	/* vertical screen size */
	edid->vertical_size = raw_data[22];

	/* display transfer characteristic */
	edid->gamma = raw_data[23];

	/* feature support */
	edid->feature_support = raw_data[24];

	/* COLOR CHARACTERISTIC */
	/* red */
	edid->chromacity_coorditates_red_x = (raw_data[25] & 0xC0) >> 6;
	edid->chromacity_coorditates_red_x += (unsigned short)raw_data[27] << 2;
	edid->chromacity_coorditates_red_y = (raw_data[25] & 0x30) >> 4;
	edid->chromacity_coorditates_red_y += (unsigned short)raw_data[28] << 2;

	/* green */
	edid->chromacity_coorditates_green_x = (raw_data[25] & 0x0C) >> 2;
	edid->chromacity_coorditates_green_x +=
	    (unsigned short)raw_data[29] << 2;
	edid->chromacity_coorditates_green_y = (raw_data[25] & 0x03);
	edid->chromacity_coorditates_green_y +=
	    (unsigned short)raw_data[30] << 2;

	/* blue */
	edid->chromacity_coorditates_blue_x = (raw_data[26] & 0xC0) >> 6;
	edid->chromacity_coorditates_blue_x +=
	    (unsigned short)raw_data[31] << 2;
	edid->chromacity_coorditates_blue_y = (raw_data[26] & 0x30) >> 4;
	edid->chromacity_coorditates_blue_y +=
	    (unsigned short)raw_data[32] << 2;

	/* blue */
	edid->chromacity_coorditates_white_x = (raw_data[26] & 0x0C) >> 2;
	edid->chromacity_coorditates_white_x +=
	    (unsigned short)raw_data[33] << 2;
	edid->chromacity_coorditates_white_y = (raw_data[26] & 0x03);
	edid->chromacity_coorditates_white_y +=
	    (unsigned short)raw_data[34] << 2;

	/* ESTABLISHED TIMINGS */
	edid->established_timing_1 = raw_data[35];
	edid->established_timing_2 = raw_data[36];
	edid->manufacturer_timing = raw_data[37];

	/* STANDARD TIMINGS */
	for (raw_data_index = 0; raw_data_index < 8; raw_data_index++) {
		edid->standard_timings[raw_data_index] =
		    raw_data[38 + (2 * raw_data_index)];
		edid->standard_timings[raw_data_index] +=
		    (unsigned short)raw_data[38 + (2 * raw_data_index + 1)];
	}
	/* extensions */
	edid->extensions = raw_data[126];

	/* DESCRIPTORS */
	raw_data_index = 54;
	for (descriptor_index = 0; descriptor_index < 4; descriptor_index++) {
		if (raw_data[raw_data_index] == 0
		    && raw_data[raw_data_index] == 0) {
			/* display descriptor found */
			tag = raw_data[raw_data_index + 3];
			if (tag == 0xFF) {
				/* display product serial number */
				S_SERIAL_NUMBER_DATA *descriptor =
				    (S_SERIAL_NUMBER_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_serial_number
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xFE) {
				/* alphanumeric data string */
				S_DATA_STRING_DATA *descriptor =
				    (S_DATA_STRING_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_data_string
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xFD) {
				/* display range limits */
				S_RANGE_LIMITS_DATA *descriptor =
				    (S_RANGE_LIMITS_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_range_limits
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xFC) {
				/* display product name */
				S_PRODUCT_NAME_DATA *descriptor =
				    (S_PRODUCT_NAME_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_product_name
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xFB) {
				/* color point data */
				S_COLOR_POINT_DATA *descriptor =
				    (S_COLOR_POINT_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_color_point
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xFA) {
				/* standard timing identifications */
				S_STANDARD_TIMING_DATA *descriptor =
				    (S_STANDARD_TIMING_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_standard_timing
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xF9) {
				/* display color management (DCM) */
				S_COLOR_MANAGEMENT_DATA *descriptor =
				    (S_COLOR_MANAGEMENT_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_color_management
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xF8) {
				/* CVT 3 byte timing codes */
				S_CVT_TIMING_CODES_DATA *descriptor =
				    (S_CVT_TIMING_CODES_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_cvt_timing_codes
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0xF7) {
				/* established timings III */
				S_ESTABLISHED_TIMINGS_3_DATA *descriptor =
				    (S_ESTABLISHED_TIMINGS_3_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_established_timings_3
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag == 0x10) {
				/* dummy */
				S_DUMMY_DATA *descriptor =
				    (S_DUMMY_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_dummy
				    (descriptor,
				     raw_data + raw_data_index) !=
				    EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;

			} else if (tag <= 0x0F) {
				/* manufacturer specific data */
				S_MANUFACTURER_SPECIFIC_DATA *descriptor =
				    (S_MANUFACTURER_SPECIFIC_DATA *) edid->
				    descriptors[descriptor_index];
				if (edid_parse_manufacturer_specific
				    (descriptor, raw_data + raw_data_index,
				     tag) != EDID_PARSER_SUCCESS)
					return EDID_PARSER_ERROR;
			}
		} else {
			/* detailed timing definition */
			S_DTD_DATA *descriptor =
			    (S_DTD_DATA *) edid->descriptors[descriptor_index];
			if (edid_parse_dtd
			    (descriptor,
			     raw_data + raw_data_index) != EDID_PARSER_SUCCESS)
				return EDID_PARSER_ERROR;

		}
		raw_data_index += 18;
	}

	return EDID_PARSER_SUCCESS;
}
