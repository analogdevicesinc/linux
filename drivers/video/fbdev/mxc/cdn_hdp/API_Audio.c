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
 * API_Audio.c
 *
 ******************************************************************************
 */
#include "API_Audio.h"
#include "API_HDMI_Audio.h"
#include "API_DPTX.h"
#include "API_General.h"
#include "source_aif_decoder.h"
#include "source_aif_smpl2pckt.h"
#include "dptx_stream.h"
#include "address.h"
#include "util.h"
#include "externs.h"
#include "aif_pckt2smp.h"
#include "dptx_framer.h"
#include "clock_meters.h"
#include "source_car.h"

CDN_API_STATUS CDN_API_AudioMute(AUDIO_MUTE_MODE mode)
{
	return (CDN_API_General_Write_Field
		(ADDR_DPTX_STREAM + (DP_VB_ID << 2), 4, 1, (1 - mode) << 4));
}

CDN_API_STATUS CDN_API_AudioMute_blocking(AUDIO_MUTE_MODE mode)
{
	internal_block_function(CDN_API_AudioMute(mode));
}

CDN_API_STATUS CDN_API_AudioMode(AUDIO_MODE mode)
{
	return (CDN_API_General_Write_Register
		(ADDR_DPTX_FRAMER + (AUDIO_PACK_CONTROL << 2),
		 F_AUDIO_PACK_EN(mode)));
}

CDN_API_STATUS CDN_API_AudioMode_blocking(AUDIO_MODE mode)
{
	internal_block_function(CDN_API_AudioMode(mode));
}

CDN_API_STATUS CDN_API_AudioConfigCore(AUDIO_TYPE audioType, int numOfChannels,
				       AUDIO_FREQ freq, int lanes,
				       AUDIO_WIDTH width)
{
	int i;
	int lanesParam;
	unsigned int I2S_DEC_PORT_EN_Val;

	if (numOfChannels == 2) {
		if (lanes == 1) {
			lanesParam = 1;
		} else {
			lanesParam = 3;
		}
	} else {
		lanesParam = 0;
	}

	if (audioType == AUDIO_TYPE_I2S) {
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (AUDIO_SRC_CNFG << 2),
			      0x20000);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (FIFO_CNTL << 2), 2);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (SMPL2PKT_CNFG << 2),
			      F_MAX_NUM_CH(numOfChannels -
					   1) |
			      F_NUM_OF_I2S_PORTS((numOfChannels / 2) -
						 1) | (1 << 8) | (lanesParam <<
								  11));

		if (numOfChannels == 2) {
			I2S_DEC_PORT_EN_Val = 1;
		} else if (numOfChannels == 4) {
			I2S_DEC_PORT_EN_Val = 3;
		} else {
			I2S_DEC_PORT_EN_Val = 0xF;
		}

		/* 24 bit configuration + number of channels according to config */
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (AUDIO_SRC_CNFG << 2),
			      0x01000 | F_AUDIO_SAMPLE_WIDTH(width) |
			      F_AUDIO_CH_NUM(numOfChannels -
					     1) |
			      F_I2S_DEC_PORT_EN(I2S_DEC_PORT_EN_Val));

		for (i = 0; i < (numOfChannels + 1) / 2; i++) {
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      ((STTS_BIT_CH01 + i) << 2),
				      F_WORD_LENGTH_CH0(0x2) |
				      F_WORD_LENGTH_CH1(0x2) |
				      F_CHANNEL_NUM_CH0(i *
							2) |
				      F_CHANNEL_NUM_CH1((i * 2) + 1));
		}

		/*set ch status bits */
		switch (freq) {

		case AUDIO_FREQ_32:
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      (COM_CH_STTS_BITS << 2),
				      4 | F_SAMPLING_FREQ(0x3) |
				      F_ORIGINAL_SAMP_FREQ(0xC));
			break;
		case AUDIO_FREQ_192:
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      (COM_CH_STTS_BITS << 2),
				      4 | F_SAMPLING_FREQ(0xE) |
				      F_ORIGINAL_SAMP_FREQ(0x1));
			break;

		case AUDIO_FREQ_48:
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      (COM_CH_STTS_BITS << 2),
				      4 | F_SAMPLING_FREQ(0x2) |
				      F_ORIGINAL_SAMP_FREQ(0xD));
			break;
		case AUDIO_FREQ_96:
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      (COM_CH_STTS_BITS << 2),
				      4 | F_SAMPLING_FREQ(0xA) |
				      F_ORIGINAL_SAMP_FREQ(0x5));
			break;
		case AUDIO_FREQ_44_1:
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      (COM_CH_STTS_BITS << 2),
				      4 | F_SAMPLING_FREQ(0x0) |
				      F_ORIGINAL_SAMP_FREQ(0xF));
			break;
		case AUDIO_FREQ_88_2:
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      (COM_CH_STTS_BITS << 2),
				      4 | F_SAMPLING_FREQ(0x8) |
				      F_ORIGINAL_SAMP_FREQ(0x7));
			break;
		case AUDIO_FREQ_176_4:
			cdn_apb_write(ADDR_SOURCE_AIF_DECODER +
				      (COM_CH_STTS_BITS << 2),
				      4 | F_SAMPLING_FREQ(0xC) |
				      F_ORIGINAL_SAMP_FREQ(0x3));
			break;
		}

		/* Enable I2S encoder */
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (AUDIO_SRC_CNTL << 2),
			      2);
		/* Enable smpl2pkt */
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (SMPL2PKT_CNTL << 2),
			      2);
	} else {

		/* set spidif 2c en */
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (SPDIF_CTRL_ADDR << 2),
			      0x1F0707);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (FIFO_CNTL << 2), 2);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (SMPL2PKT_CNFG << 2),
			      0x101 | (lanesParam << 11));
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (SMPL2PKT_CNTL << 2),
			      2);
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (SPDIF_CTRL_ADDR << 2),
			      0x3F0707);
	}
	return CDN_OK;
}

CDN_API_STATUS CDN_API_AudioConfigCore_blocking(AUDIO_TYPE audioType,
						int numOfChannels,
						AUDIO_FREQ freq, int lanes,
						AUDIO_WIDTH width)
{
	internal_block_function(CDN_API_AudioConfigCore
				(audioType, numOfChannels, freq, lanes, width));
}

CDN_API_STATUS CDN_API_AudioAutoConfig(AUDIO_TYPE audioType, int numOfChannels,
				       AUDIO_FREQ freq, int lanes,
				       AUDIO_WIDTH width,
				       CDN_PROTOCOL_TYPE protocol, int ncts,
				       AUDIO_MUTE_MODE mode)
{

	CDN_API_STATUS ret = CDN_BSY;
	unsigned int REF_CYC_Val;
	switch (state.tmp) {
	case 0:
		if (protocol == CDN_DPTX) {
			ret =
			    CDN_API_General_Write_Register(ADDR_DPTX_FRAMER +
							   (AUDIO_PACK_STATUS <<
							    2), 0x11 << 16);
		} else {
			ret = CDN_OK;
		}
		break;

	case 1:
		if (protocol == CDN_DPTX) {
			REF_CYC_Val = 0x8000;
			ret =
			    CDN_API_General_Write_Register(ADDR_CLOCK_METERS +
							   (CM_LANE_CTRL << 2),
							   REF_CYC_Val);
		} else {
			/* hdmi mode */
			ret = CDN_API_General_Write_Register(ADDR_CLOCK_METERS + (CM_CTRL << 2), 8);

		}
		break;
	case 2:
		if ((protocol == CDN_HDMITX_TYPHOON)
		    || (protocol == CDN_HDMITX_KIRAN)) {
			ret =
			    CDN_API_General_Write_Register(ADDR_CLOCK_METERS +
							   (CM_I2S_CTRL << 2),
							   ncts | 0x4000000);
		} else {
			ret = CDN_OK;
		}

		break;

	case 3:
		if ((protocol == CDN_HDMITX_TYPHOON)
		    || (protocol == CDN_HDMITX_KIRAN)) {
			ret = CDN_OK;
		} else {
			/* in dptx set audio on in dp framer */
			ret = CDN_API_AudioMode(1);
		}
		break;

	case 4:
		/* set car audio on _not reset */
		if (protocol == CDN_DPTX) {
			/* TODO DK: try to merge case 3 and 4 */
			ret = CDN_OK;
		} else {
			ret = CDN_OK;
		}
		break;

	case 5:
		if ((protocol == CDN_DPTX) && (audioType != AUDIO_TYPE_I2S)) {
			ret =
			    CDN_API_General_Write_Register(ADDR_SOURCE_CAR +
							   (SOURCE_AIF_CAR <<
							    2), 0xff);
		} else {
			ret = CDN_OK;
		}
		break;
	case 6:
		if (protocol == CDN_DPTX) {
			ret =
			    CDN_API_General_Write_Register(ADDR_CLOCK_METERS +
							   (CM_CTRL << 2), 0);
		} else {
			ret = CDN_OK;
		}
		break;

	case 7:
		ret =
		    CDN_API_AudioConfigCore(audioType, numOfChannels, freq,
					    lanes, width);
		break;
	case 8:
		if ((protocol == CDN_HDMITX_TYPHOON)
		    || (protocol == CDN_HDMITX_KIRAN)) {
			CDN_API_HDMI_AudioSetInfoFrame(mode, audioType,
						       numOfChannels, freq,
						       lanes, ncts);
		}
		ret = CDN_OK;
		break;

	}
	if (!state.tmp && ret == CDN_STARTED)
		return CDN_STARTED;
	switch (ret) {
	case CDN_OK:
		state.tmp++;
		break;
	case CDN_STARTED:
		return CDN_BSY;
		break;
	default:
		return ret;
	}
	if (state.tmp == 9) {
		state.tmp = 0;
		return CDN_OK;
	}
	return CDN_BSY;

}

CDN_API_STATUS CDN_API_AudioAutoConfig_blocking(AUDIO_TYPE audioType,
						int numOfChannels,
						AUDIO_FREQ freq, int lanes,
						AUDIO_WIDTH width,
						CDN_PROTOCOL_TYPE protocol,
						int ncts, AUDIO_MUTE_MODE mode)
{
	internal_block_function(CDN_API_AudioAutoConfig
				(audioType, numOfChannels, freq, lanes, width,
				 protocol, ncts, mode));
}

CDN_API_STATUS CDN_API_AudioOff(AUDIO_TYPE audioType)
{
	CDN_API_STATUS ret = CDN_BSY;

	switch (state.tmp) {
	case 0:
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (SPDIF_CTRL_ADDR << 2),
			      0x1F0707);
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (AUDIO_SRC_CNTL << 2),
			      0);
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (AUDIO_SRC_CNFG << 2),
			      0);
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (AUDIO_SRC_CNTL << 2),
			      1);
		cdn_apb_write(ADDR_SOURCE_AIF_DECODER + (AUDIO_SRC_CNTL << 2),
			      0);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (SMPL2PKT_CNTL << 2),
			      0);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (SMPL2PKT_CNTL << 2),
			      1);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (SMPL2PKT_CNTL << 2),
			      0);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (FIFO_CNTL << 2), 1);
		cdn_apb_write(ADDR_SOURCE_AIF_SMPL2PCKT + (FIFO_CNTL << 2), 0);
		ret = CDN_OK;

		break;
	case 1:
		ret =
		    CDN_API_General_Write_Register(ADDR_SOURCE_CAR +
						   (SOURCE_AIF_CAR << 2), 0x5f);
		break;
	case 2:
		ret =
		    CDN_API_General_Write_Register(ADDR_SOURCE_CAR +
						   (SOURCE_AIF_CAR << 2), 0x0f);
		break;
	case 3:
		ret = CDN_OK;
		break;
	}

	if (!state.tmp && ret == CDN_STARTED)
		return CDN_STARTED;
	switch (ret) {
	case CDN_OK:
		state.tmp++;
		break;
	case CDN_STARTED:
		return CDN_BSY;
		break;
	default:
		return ret;
	}
	if (state.tmp == 4) {
		state.tmp = 0;
		return CDN_OK;
	}
	return CDN_BSY;
}

CDN_API_STATUS CDN_API_AudioOff_blocking(AUDIO_TYPE audioType)
{
	internal_block_function(CDN_API_AudioOff(audioType));
}
