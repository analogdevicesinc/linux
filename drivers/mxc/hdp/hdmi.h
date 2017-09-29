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
 * hdmi.h
 *
 ******************************************************************************
 */

#ifndef _HDMI__
#define _HDMI__
/* ONLY ENUMS AND #DEFINES IN THIS FILE *
 * THIS FILE WILL BE USED IN HOST'S API */

#define EDID_SLAVE_ADDRESS                  0x50
#define EDID_SEGMENT_SLAVE_ADDRESS          0x30
#define SCDC_SLAVE_ADDRESS                  0x54

typedef enum {
	HDMI_TX_READ,
	HDMI_TX_WRITE,
	HDMI_TX_UPDATE_READ,
	HDMI_TX_EDID,
	HDMI_TX_EVENTS,
	HDMI_TX_HPD_STATUS,
	HDMI_TX_DEBUG_ECHO = 0xAA,
	HDMI_TX_TEST = 0xBB,
	HDMI_TX_EDID_INTERNAL = 0xF0,
} HDMI_TX_OPCODE;

typedef enum {
	HDMI_I2C_ACK,
	HDMI_I2C_NACK,
	HDMI_I2C_TO,
	HDMI_I2C_ARB_LOST,
	HDMI_I2C_RRTO,
	HDMI_I2C_RRT,
    /** when i2c hardware didn't respond after some time */
	HDMI_I2C_HW_TO,
	HDMI_I2C_ERR		//unspecified error
} HDMI_I2C_STATUS;

typedef enum {
	HDMI_RX_SET_EDID,
	HDMI_RX_SCDC_SET,
	HDMI_RX_SCDC_GET,
	HDMI_RX_READ_EVENTS,
	HDMI_RX_SET_HPD,

	HDMI_RX_DEBUG_ECHO = 0xAA,
	HDMI_RX_TEST = 0xBB,
} HDMI_RX_OPCODE;

typedef enum {
	HDMI_SCDC_SINK_VER,
	HDMI_SCDC_SOURCE_VER,
} HDMI_SCDC_FIELD;

typedef struct {
	u8 sink_ver;
	u8 manufacturer_oui_1;
	u8 manufacturer_oui_2;
	u8 manufacturer_oui_3;
	u8 devId[8];
	u8 hardware_major_rev;
	u8 hardware_minor_rev;
	u8 software_major_rev;
	u8 software_minor_rev;
	u8 manufacturerSpecific[34];
} S_HDMI_SCDC_SET_MSG;

typedef struct {
	u8 source_ver;
	u8 TMDS_Config;
	u8 config_0;
	u8 manufacturerSpecific[34];
} S_HDMI_SCDC_GET_MSG;

/* hpd events location */
#define HDMI_RX_EVENT_5V_HIGH            0
#define HDMI_RX_EVENT_5V_LOW             1
#define HDMI_TX_EVENT_reserved           2
#define HDMI_RX_EVENT_5V_VAL             3

#endif
