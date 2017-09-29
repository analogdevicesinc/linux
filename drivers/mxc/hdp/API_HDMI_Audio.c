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
 * API_HDMI_Audio.c
 *
 ******************************************************************************
 */

#include "API_HDMI_Audio.h"
#include "API_Infoframe.h"
#include "source_aif_decoder.h"
#include "source_aif_smpl2pckt.h"
#include "dptx_stream.h"
#include "address.h"
#include "util.h"
#include "aif_pckt2smp.h"
#include "dptx_framer.h"
#include "clock_meters.h"
#include "source_car.h"
#include "API_DPTX.h"
#include "mhl_hdtx_top.h"

CDN_API_STATUS CDN_API_HDMI_AudioSetInfoFrame(state_struct *state,
					      AUDIO_MUTE_MODE mode,
					      AUDIO_TYPE audioType,
					      int numOfChannels,
					      AUDIO_FREQ freq, int lanes,
					      int ncts)
{
	u32 packet_type = 0x84;
	u32 packet_version = 0x1;
	u32 packet_len = 0xA;
	u32 packet_HB0 = packet_type;
	u32 packet_HB1 = packet_version;
	u32 packet_HB2 = packet_len;
	u32 packet_PB0 = 0;
	u32 packet_PB1 = numOfChannels - 1;
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
	u32 packet_PB14 = 0;
	u32 packet_PB15 = 0;
	u32 packet_PB16 = 0;
	u32 packet_PB17 = 0;
	u32 packet_PB18 = 0;
	u32 packet_PB19 = 0;
	u32 packet_PB20 = 0;
	u32 packet_PB21 = 0;
	u32 packet_PB22 = 0;
	u32 packet_PB23 = 0;
	u32 packet_PB24 = 0;
	u32 packet_PB25 = 0;
	u32 packet_PB26 = 0;
	u32 packet_PB27 = 0;
	u32 PB1_13_chksum = 0;
	u32 packet_chksum = 0;
	u8 packet[32];

	if (numOfChannels == 2) {
		packet_PB4 = 0;
	} else if (numOfChannels == 8) {
		packet_PB4 = 0x13;
	}

	PB1_13_chksum =
	    (packet_HB0 + packet_HB1 + packet_HB2 + packet_PB1 + packet_PB2 +
	     packet_PB3 + packet_PB4 + packet_PB5 + packet_PB6 + packet_PB7 +
	     packet_PB8 + packet_PB9 + packet_PB10);
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
	packet[18] = packet_PB14;
	packet[19] = packet_PB15;
	packet[20] = packet_PB16;
	packet[21] = packet_PB17;
	packet[22] = packet_PB18;
	packet[23] = packet_PB19;
	packet[24] = packet_PB20;
	packet[25] = packet_PB21;
	packet[26] = packet_PB22;
	packet[27] = packet_PB23;
	packet[28] = packet_PB24;
	packet[29] = packet_PB25;
	packet[30] = packet_PB26;
	packet[31] = packet_PB27;

	CDN_API_InfoframeSet(state, 1, 28, (u32 *) &packet[0], packet_type);

	return CDN_OK;
}
