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
 * API_Audio.h
 *
 ******************************************************************************
 */

#ifndef API_AUDIO_H_
#define API_AUDIO_H_

#include "API_General.h"
/**
 * \addtogroup AUDIO_API
 * \{
 */
typedef enum {
	AUDIO_TYPE_I2S,
	AUDIO_TYPE_SPIDIF_INTERNAL,
	AUDIO_TYPE_SPIDIF_EXTERNAL,
} AUDIO_TYPE;

typedef enum {
	AUDIO_FREQ_32,
	AUDIO_FREQ_48,
	AUDIO_FREQ_96,
	AUDIO_FREQ_192,
	AUDIO_FREQ_44_1,
	AUDIO_FREQ_88_2,
	AUDIO_FREQ_176_4,
} AUDIO_FREQ;

typedef enum {
	AUDIO_WIDTH_16,
	AUDIO_WIDTH_24,
	AUDIO_WIDTH_32,
} AUDIO_WIDTH;

typedef enum {
	AUDIO_MODE_OFF,
	AUDIO_MODE_ON
} AUDIO_MODE;

typedef enum {
	AUDIO_MUTE_MODE_MUTE,
	AUDIO_MUTE_MODE_UNMUTE
} AUDIO_MUTE_MODE;

/**
 * \brief mute or unmute audio
 */
CDN_API_STATUS CDN_API_AudioMute(state_struct *state, AUDIO_MUTE_MODE mode);

/**
 * \brief blocking version of #CDN_API_AudioMute
 */
CDN_API_STATUS CDN_API_AudioMute_blocking(state_struct *state,
					  AUDIO_MUTE_MODE mode);

/**
 * \brief start playing audio with the input parameters
    ncts and mode are relevant only in HDMI TX mode , not relevant for DPTX mode
 */
CDN_API_STATUS CDN_API_AudioAutoConfig(state_struct *state,
				       AUDIO_TYPE audioType, int numOfChannels,
				       AUDIO_FREQ freq, int lanes,
				       AUDIO_WIDTH width,
				       CDN_PROTOCOL_TYPE protocol, int ncts,
				       AUDIO_MUTE_MODE mode);

/**
 * \brief blocking version of #CDN_API_AudioAutoConfig
 */
CDN_API_STATUS CDN_API_AudioAutoConfig_blocking(state_struct *state,
						AUDIO_TYPE audioType,
						int numOfChannels,
						AUDIO_FREQ freq, int lanes,
						AUDIO_WIDTH width,
						CDN_PROTOCOL_TYPE protocol,
						int ncts, AUDIO_MUTE_MODE mode);

/**
 * \brief audio off (use it to stop current audio and start new one using CDN_API_AudioAutoConfig)
 */
CDN_API_STATUS CDN_API_AudioOff(state_struct *state, AUDIO_TYPE audioType);

/**
 * \brief blocking version of #CDN_API_AudioOff
 */
CDN_API_STATUS CDN_API_AudioOff_blocking(state_struct *state,
					 AUDIO_TYPE audioType);

/**
 * \brief internal function to set audio on or off inside internal registers
 */
CDN_API_STATUS CDN_API_AudioMode(state_struct *state, AUDIO_MODE mode);

/**
 * \brief blocking version of #CDN_API_AudioMode
 */
CDN_API_STATUS CDN_API_AudioMode_blocking(state_struct *state,
					  AUDIO_MODE mode);

/**
 * \brief internal function to set audio core registers
 */
CDN_API_STATUS CDN_API_AudioConfigCore(state_struct *state,
				       AUDIO_TYPE audioType, int numOfChannels,
				       AUDIO_FREQ freq, int lanes,
				       AUDIO_WIDTH width);

/**
 * \brief blocking version of #CDN_API_AudioConfigCore
 */
CDN_API_STATUS CDN_API_AudioConfigCore_blocking(state_struct *state,
						AUDIO_TYPE audioType,
						int numOfChannels,
						AUDIO_FREQ freq, int lanes,
						AUDIO_WIDTH width);

#endif
