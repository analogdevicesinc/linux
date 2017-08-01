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
 ******************************************************************************
 *
 * edid_parser.h
 *
 ******************************************************************************
 */

#ifndef EDID_PARSER_H
# define EDID_PARSER_H

# define MAX_DESCRIPTOR_LENGTH 36
# define MAX_RANGE_LIMITS_VIDEO_TIMING_LENGTH 12
# define EDID_HEADER_LENGTH 8
# define EDID_LENGTH 128

typedef enum {
	EDID_PARSER_SUCCESS,
	EDID_PARSER_ERROR,
} EDID_PARSER_RESULT;

typedef enum {
	DESCRIPTOR_TYPE_DTD,
	DESCRIPTOR_TYPE_SERIAL_NUMBER,
	DESCRIPTOR_TYPE_DATA_STRING,
	DESCRIPTOR_TYPE_RANGE_LIMITS,
	DESCRIPTOR_TYPE_PRODUCT_NAME,
	DESCRIPTOR_TYPE_COLOR_POINT,
	DESCRIPTOR_TYPE_STANDARD_TIMING,
	DESCRIPTOR_TYPE_COLOR_MANAGEMENT,
	DESCRIPTOR_TYPE_CVT_TIMING_CODES,
	DESCRIPTOR_TYPE_ESTABLISHED_TIMINGS_3,
	DESCRIPTOR_TYPE_DUMMY,
	DESCRIPTOR_TYPE_MANUFACTURER_SPECIFIC
} EDID_DESCRIPTOR_TYPE;

typedef enum {
	VIDEO_TIMING_DEFAULT_GTF,
	VIDEO_TIMING_RANGE_LIMITS_ONLY,
	VIDEO_TIMING_SECONDARY_GTF,
	VIDEO_TIMING_CVT,
} RANGE_LIMITS_VIDEO_TIMING_TYPE;

/**
 *  \brief Common descriptor header structure
 */
typedef struct {
	EDID_DESCRIPTOR_TYPE type;
	unsigned char tag;

} S_DESCRIPTOR_HEADER_DATA;
/**
 *  \brief Detailed Timing Descriptor (DTD) structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned short pixel_clock;
	unsigned short horizontal_addressable_video;
	unsigned short horizontal_blanking;
	unsigned short vertical_addressable_video;
	unsigned short vertical_blanking;
	unsigned short horizontal_front_porch;
	unsigned short horizontal_sync_pulse_width;
	unsigned short vertical_front_porch;
	unsigned short vertical_sync_pulse_width;
	unsigned short horizontal_addressable_video_image_size;
	unsigned short vertical_addressable_video_image_size;
	unsigned char horizontal_border;
	unsigned char vertical_border;
	unsigned char signal_features;
} S_DTD_DATA;

/**
 *  \brief Serial Number Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned char serial_number[13];

} S_SERIAL_NUMBER_DATA;

/**
 *  \brief Data String Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	char data_string[13];

} S_DATA_STRING_DATA;

/**
 *  \brief Range Limits Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned char offset_flags;
	unsigned char min_vertical_rate;
	unsigned char max_vertical_rate;
	unsigned char min_horizontal_rate;
	unsigned char max_horizontal_rate;
	unsigned char max_pixel_clock;
	RANGE_LIMITS_VIDEO_TIMING_TYPE type;
	unsigned char suport_flags[MAX_RANGE_LIMITS_VIDEO_TIMING_LENGTH];
} S_RANGE_LIMITS_DATA;

/**
 *  \brief Range Limits Secondary GTF Flags structure
 */
typedef struct {
	unsigned char start_break_frequency;
	unsigned char c;
	unsigned short m;
	unsigned char k;
	unsigned char j;

} S_RANGE_LIMITS_VIDEO_TIMING_SECONDARY_GTF;

/**
 *  \brief Range Limits CVT Flags structure
 */
typedef struct {
	unsigned char cvt_version;
	unsigned char additional_pixel_clock_precision;
	unsigned short max_active_pixels;
	unsigned char supported_ar;
	unsigned char preferred_ar;
	unsigned char blanking_support;
	unsigned char supported_scalling;
	unsigned char preferred_vertical_refresh_rate;
} S_RANGE_LIMITS_VIDEO_TIMING_CVT;

/**
 *  \brief Product Name Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	char product_name[13];

} S_PRODUCT_NAME_DATA;

/**
 *  \brief Color point Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned char white_point_index_1;
	unsigned short white_x_1;
	unsigned short white_y_1;
	unsigned char gamma_1;
	unsigned char white_point_index_2;
	unsigned short white_x_2;
	unsigned short white_y_2;
	unsigned char gamma_2;
} S_COLOR_POINT_DATA;

/**
 *  \brief Standard Timing Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned short standard_timings[6];
} S_STANDARD_TIMING_DATA;

/**
 *  \brief Color Management Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned char version;
	unsigned short red_a3;
	unsigned short red_a2;
	unsigned short green_a3;
	unsigned short green_a2;
	unsigned short blue_a3;
	unsigned short blue_a2;
} S_COLOR_MANAGEMENT_DATA;

/**
 *  \brief CVT 3 Byte Code Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned char version;
	unsigned short addressable_lines[4];
	unsigned char aspect_ratio[4];
	unsigned char preferred_vertical_rate[4];
	unsigned char supported_vertical_rate_and_blanking[4];

} S_CVT_TIMING_CODES_DATA;

/**
 *  \brief Established Timings 3 Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned char version;
	unsigned char established_timings[6];
} S_ESTABLISHED_TIMINGS_3_DATA;

/**
 *  \brief Dummy Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
} S_DUMMY_DATA;

/**
 *  \brief Manufacturer Specific Descriptor structure
 */
typedef struct {
	S_DESCRIPTOR_HEADER_DATA header;
	unsigned char desc_data[18];
} S_MANUFACTURER_SPECIFIC_DATA;

/**
 *  \brief CEA-861 extension structure
 */
typedef struct {
	unsigned char revision;
	unsigned char underscan;
	unsigned char audio;
} S_CEA861_DATA;

/**
 *  \brief Extended Display Identification Data (EDID) structure
 */
typedef struct {
	unsigned char header[8];
	char manufacturer_name[4];
	unsigned short product_code;
	unsigned int serial_number;
	unsigned char week;
	unsigned short year;
	unsigned short edid_version;
	unsigned char video_input_definition;
	unsigned char horizontal_size;
	unsigned char vertical_size;
	unsigned char gamma;
	unsigned char feature_support;
	unsigned short chromacity_coorditates_red_x;
	unsigned short chromacity_coorditates_red_y;
	unsigned short chromacity_coorditates_green_x;
	unsigned short chromacity_coorditates_green_y;
	unsigned short chromacity_coorditates_blue_x;
	unsigned short chromacity_coorditates_blue_y;
	unsigned short chromacity_coorditates_white_x;
	unsigned short chromacity_coorditates_white_y;
	unsigned char established_timing_1;
	unsigned char established_timing_2;
	unsigned char manufacturer_timing;
	unsigned short standard_timings[8];
	unsigned char descriptors[4][MAX_DESCRIPTOR_LENGTH];
	unsigned char extensions;
} S_EDID_DATA;

EDID_PARSER_RESULT edid_parse(S_EDID_DATA *edid, unsigned char *raw_data,
			      unsigned int len);

#endif /* EDID_PARSER_H */
