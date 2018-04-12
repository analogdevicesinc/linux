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
 * Copyright 2018 NXP
 *
 ******************************************************************************
 *
 * API_HDMI_RX_Audio.c
 *
 ******************************************************************************
 */

#include "API_HDMI_RX_Audio.h"
#include "sink_aif_encoder.h"
#include "aif_pckt2smp.h"
#include "address.h"
#include "util.h"
#include "API_General.h"

CDN_API_STATUS CDN_API_RX_AudioAutoConfig(
						state_struct *state,
						u8 max_ch_num,
						u8 i2s_ports_num,
						u8 dis_port3,
						u8 enc_sample_width,
						u8 i2s_sample_width)
{
	u32 regread;
	u8 num_of_pairs_of_channels_per_port = max_ch_num / (i2s_ports_num * 2);
	u8 enc_size_code;
	u8 i2s_size_code;
	u8 i2s_port3_dis = (dis_port3 != 0 && i2s_ports_num == 4) ? 1 : 0;

	/* Valid values: 1/2/4. */
	/* 3 ports can be emulated with 'i2s_ports_num = 4' and 'dis_port3 = 1'. */
	if (i2s_ports_num == 0 || i2s_ports_num == 3 || i2s_ports_num > 4)
		return CDN_ERR;

	/* 'dis_port3' makes sense only with 4 ports enabled */
	if (dis_port3 != 0 && i2s_ports_num < 4)
		return CDN_ERR;

	switch (enc_sample_width) {
	case 16:
		enc_size_code = 0x0;
		break;
	case 24:
		enc_size_code = 0x1;
		break;
	case 32:
		enc_size_code = 0x2;
		break;
	default:
		return CDN_ERR;
	}

	switch (i2s_sample_width) {
	case 16:
		i2s_size_code = 0x0;
		break;
	case 24:
		i2s_size_code = 0x1;
		break;
	case 32:
		i2s_size_code = 0x2;
		break;
	default:
		return CDN_ERR;
	}

	/* Maximum number of channels has to be in range from 2 to 32 */
	if (max_ch_num < 2 || max_ch_num > 32)
		return CDN_ERR;
	/* Maximum number of channels has to be power of 2 */
	else if (!(max_ch_num * (max_ch_num - 1)))
		return CDN_ERR;
	/* Each active port shall carry the same number of sub-channels */
	else if (max_ch_num % i2s_ports_num)
		return CDN_ERR;

	/* Disable ACR during configuration */
	if (cdn_apb_write(state, ADDR_AIF_ENCODER + (ACR_CFG << 2), F_ACR_SW_RESET(1)))
		return CDN_ERR;

	/* Configuring audio FIFO */
	if (cdn_apb_write(state,
				ADDR_AIF_ENCODER + ((0x40 + FIFO_CNTL_ADDR) << 2),
				F_CFG_FIFO_SW_RST(0) | F_CFG_INDEX_SYNC_EN(1) |
				F_CFG_FIFO_DIR(1) |   F_CFG_DIS_PORT3(i2s_port3_dis)))
		return CDN_ERR;

	/* Configuring audio parameters */
	if (cdn_apb_write(state,
				ADDR_AIF_ENCODER + ((0x40 + AUDIO_SINK_CNFG) << 2),
				F_ENC_LOW_INDEX_MSB(0) | F_SINK_AUDIO_CH_NUM(max_ch_num - 1) |
				F_ENC_SAMPLE_JUST(0x1) | F_ENC_SMPL_WIDTH(enc_size_code) |
				F_I2S_ENC_WL_SIZE(i2s_size_code) | F_CNTL_SMPL_ONLY_EN(1) |
				F_CNTL_TYPE_OVRD(0x0) | F_CNTL_TYPE_OVRD_EN(0) |
				F_I2S_ENC_PORT_EN((1 << i2s_ports_num) - 1) | F_WS_POLARITY(0)))
		return CDN_ERR;

	/* Waiting for N value... */
	do {
		if (cdn_apb_read(state,
					ADDR_AIF_ENCODER + (AIF_ACR_N_ST << 2), &regread))
			return CDN_ERR;
	} while (!(regread));

	/* Enable ACR */
	if (cdn_apb_write(state,
				ADDR_AIF_ENCODER + (ACR_CFG << 2), F_ACR_SW_RESET(0)))
		return CDN_ERR;

	/* Important: */
	/* Write to AIF_ACR_N_OFST_CFG register is interpreted as new N_CTS value. */
	/* The ACR has to be enabled (reset released) to register that event. */

	if (cdn_apb_write(state,
				ADDR_AIF_ENCODER + (AIF_ACR_N_OFST_CFG << 2),
				F_ACR_N_OFFSET(regread * (num_of_pairs_of_channels_per_port - 1))))
		return CDN_ERR;

	/* Enable sample decoder */
	if (cdn_apb_write(state,
				ADDR_AIF_ENCODER + (PKT2SMPL_CNTL << 2), F_PKT2SMPL_EN(1)))
		return CDN_ERR;

	/* Enable I2S encoder */
	if (cdn_apb_write(state,
				ADDR_AIF_ENCODER + ((0x40 + AUDIO_SINK_CNTL) << 2),	F_I2S_ENC_START(1)))
		return CDN_ERR;

	return CDN_OK;

}

CDN_API_STATUS CDN_API_RX_AudioAutoConfig_blocking(state_struct *state,
						   u8 max_ch_num,
						   u8 i2s_ports_num,
						   u8 dis_port3,
						   u8 enc_sample_width,
						   u8 i2s_sample_width)
{
	internal_block_function(&state->mutex,
							CDN_API_RX_AudioAutoConfig(state,
							max_ch_num,
							i2s_ports_num,
							dis_port3,
							enc_sample_width,
							i2s_sample_width));
}
