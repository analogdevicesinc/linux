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
 * API_AVI.c
 *
 ******************************************************************************
 */

#include "API_AVI.h"
#include "API_Infoframe.h"

CDN_API_STATUS CDN_API_Set_AVI(state_struct *state, VIC_MODES vicMode,
			       VIC_PXL_ENCODING_FORMAT colorMode,
			       BT_TYPE ITUver)
{
	u32 active_slot = vic_table[vicMode][H_BLANK];
	u32 line_width = vic_table[vicMode][H_TOTAL];
	u32 Hactive = line_width - active_slot + 1;
	u32 Vactive = vic_table[vicMode][V_ACTIVE] + 1;

	u32 Hactive_l = Hactive - 256 * ((u32) Hactive / 256);
	u32 Hactive_h = Hactive / 256;
	u32 Vactive_l = Vactive - 256 * ((u32) Vactive / 256);
	u32 Vactive_h = Vactive / 256;

	u32 packet_type = 0x82;
	u32 packet_version = 0x2;
	u32 packet_len = 0xD;
	u32 packet_Y = 0;
	u32 packet_C = 0;
	u32 packet_R = 0;
	u32 packet_VIC = 0;
	u32 packet_PR = 0;
	u8 packet[32];
	u8 len = sizeof(packet)/sizeof(u32);
	u32 packet_HB0 = 0;
	u32 packet_HB1 = 0;
	u32 packet_HB2 = 0;
	u32 packet_PB0 = 0;
	u32 packet_PB1 = 0;
	u32 packet_PB2 = 0;
	u32 packet_PB3 = 0;
	u32 packet_PB4 = 0;
	u32 packet_PB5 = 0;
	u32 packet_PB6 = 0;
	u32 packet_PB7 = 0;
	u32 packet_PB8 = 0;
	u32 packet_PB9 = 0;
	u32 packet_PB10 = 0;
	u32 packet_PB11 = 0;
	u32 packet_PB12 = 0;
	u32 packet_PB13 = 0;
	u32 PB1_13_chksum = 0;
	u32 packet_chksum = 0;

	u32 packet_A0 = 1;
	u32 packet_B = 0;
	u32 packet_S = 0;
	/* Picture Scsaling */
	u32 packet_SC = 0;
	/* Aspect Ratio: Nodata=0 4:3=1 16:9=2 */
	u32 packet_M = 0;
	/* Quantization Range Default=0 Limited Range=0x1 FullRange=0x2 Reserved 0x3 */
	u32 packet_Q = 0;
	/* Quantization Range 0=Limited Range  FullRange=0x1 Reserved 0x3/2 */
	u32 packet_YQ = 0;
	/* Extended Colorimetry xvYCC601=0x0 xvYCC709=1 All other Reserved */
	u32 packet_EC = 0;
	/* IT content nodata=0 ITcontent=1 */
	u32 packet_IT = 0;
	/* Content Type */
	u32 packet_CN = 0;

	/* Active Format Aspec Ratio:
	 * Same As Picture = 0x8 4:3(Center)=0x9 16:9=0xA 14:9=0xB */
	packet_R = vic_table[vicMode][VIC_R3_0];
	/* Video Code (CEA) */
	packet_VIC = vic_table[vicMode][VIC];
	/* Pixel Repetition 0 ... 9 (1-10) */
	packet_PR = vic_table[vicMode][VIC_PR];

	if (colorMode == PXL_RGB)
		packet_Y = 0;
	else if (colorMode == YCBCR_4_4_4)
		packet_Y = 2;
	else if (colorMode == YCBCR_4_2_2)
		packet_Y = 1;
	else if (colorMode == YCBCR_4_2_0)
		packet_Y = 3;

	/* Colorimetry:  Nodata=0 IT601=1 ITU709=2 */
	if (ITUver == BT_601)
		packet_C = 1;
	else if (ITUver == BT_709)
		packet_C = 2;
	else
		packet_C = 0;

	packet_HB0 = packet_type;
	packet_HB1 = packet_version;
	packet_HB2 = packet_len;

	packet_PB1 = 32 * packet_Y + 16 * packet_A0 + 4 * packet_B + packet_S;
	packet_PB2 = 64 * packet_C + 16 * packet_M + packet_R;
	packet_PB3 =
	    128 * packet_IT + 16 * packet_EC + 4 * packet_Q + packet_SC;
	packet_PB4 = packet_VIC;
	packet_PB5 = 64 * packet_YQ + 16 * packet_CN + packet_PR;
	packet_PB6 = 0;
	packet_PB7 = 0;
	packet_PB8 = Vactive_l;
	packet_PB9 = Vactive_h;
	packet_PB10 = 0;
	packet_PB11 = 0;
	packet_PB12 = Hactive_l;
	packet_PB13 = Hactive_h;

	PB1_13_chksum =
	    (packet_HB0 + packet_HB1 + packet_HB2 + packet_PB1 + packet_PB2 +
	     packet_PB3 + packet_PB4 + packet_PB5 + packet_PB6 + packet_PB7 +
	     packet_PB8 + packet_PB9 + packet_PB10 + packet_PB11 + packet_PB12 +
	     packet_PB13);
	packet_chksum =
	    256 - (PB1_13_chksum - 256 * ((u32) PB1_13_chksum / 256));
	packet_PB0 = packet_chksum;

	packet[0] = 0;
	packet[1] = packet_HB0;
	packet[2] = packet_HB1;
	packet[3] = packet_HB2;
	packet[4] = packet_PB0;
	packet[5] = packet_PB1;
	packet[6] = packet_PB2;
	packet[7] = packet_PB3;
	packet[8] = packet_PB4;
	packet[9] = packet_PB5;
	packet[10] = packet_PB6;
	packet[11] = packet_PB7;
	packet[12] = packet_PB8;
	packet[13] = packet_PB9;
	packet[14] = packet_PB10;
	packet[15] = packet_PB11;
	packet[16] = packet_PB12;
	packet[17] = packet_PB13;

	CDN_API_InfoframeSet(state, 0, len, (u32 *)&packet[0], packet_type);

	return CDN_OK;
}
