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
 * API_HDMIRX.h
 *
 ******************************************************************************
 */

#ifndef _API_HDMIRX_H_
#define _API_HDMIRX_H_

#include "API_General.h"
#include "hdmi.h"

/**
 * \addtogroup HDMI_RX_API
 * \{
 */

/**
 * \brief init hdmi rx registers
 * \returns status
 */
CDN_API_STATUS CDN_API_HDMIRX_Init_blocking(state_struct *state);
CDN_API_STATUS CDN_API_HDMIRX_Stop_blocking(state_struct *state);

/**
 * \brief get hdmi rx events (currently 5v events)
 * \returns status
 */
CDN_API_STATUS CDN_API_HDMIRX_ReadEvent(state_struct *state, u8 *Events_5v);

/**
 * \brief blocking version of CDN_API_HDMIRX_ReadEvent
 * \returns status
 */
CDN_API_STATUS CDN_API_HDMIRX_ReadEvent_blocking(state_struct *state, u8 *Events_5v);


/**
 *  \brief Cadence API for HDMI TX to set  EDID
 *  \param [in] segment - EDID segment to read
 *  \param [in] extension - EDID extension to read
 *  \param [in] edid_buff - pointer to buffer with 128 bytes of edid block
 *  please note, edid_buff should be allocated\clear by caller
 *  \return status
 *
 */
CDN_API_STATUS CDN_API_HDMIRX_SET_EDID(state_struct *state, u8 segment, u8 extension, u8 *edid_buff);
/**
 * \brief blocking version of #CDN_API_HDMIRX_SET_EDID
 */
CDN_API_STATUS CDN_API_HDMIRX_SET_EDID_blocking(state_struct *state, u8 segment, u8 extension, u8 *edid_buff);

/**
 *  \brief Cadence API for HDMI Rx to set general scdc information
 *  \param [in] scdcData - general slave scdc information
 *  \return status
 *
 */
CDN_API_STATUS CDN_API_HDMIRX_SET_SCDC_SLAVE(state_struct *state, S_HDMI_SCDC_SET_MSG *scdcData);

/**
 * \brief blocking version of #CDN_API_HDMIRX_SET_SCDC_SLAVE
 */
CDN_API_STATUS CDN_API_HDMIRX_SET_SCDC_SLAVE_blocking(state_struct *state, S_HDMI_SCDC_SET_MSG *scdcData);

/**
 *  \brief Cadence API for HDMI Rx to get general scdc information
 *  \param [in] source_ver  - SCDC register - Source Version
 *  \param [in] TMDS_Config - SCDC register - TMDS_Config
 *  \param [in] config_0    - SCDC register - Config_0
 *  \param [in] manufacturerSpecific - manufacturer specific data
 *  \return status
 *
 */
CDN_API_STATUS CDN_API_HDMIRX_GET_SCDC_SLAVE(state_struct *state, S_HDMI_SCDC_GET_MSG *scdcData);

/**
 * \brief blocking version of #CDN_API_HDMIRX_GET_SCDC_SLAVE
 */
CDN_API_STATUS CDN_API_HDMIRX_GET_SCDC_SLAVE_blocking(state_struct *state, S_HDMI_SCDC_GET_MSG *scdcData);

/**
 *  \brief Cadence API for HDMI Rx to set hpd
 *  \param [in] hpd - 0 or 1
 *  \return status
 *
 */
CDN_API_STATUS CDN_API_HDMIRX_SetHpd(state_struct *state, u8 hpd);

/**
 * \brief blocking version of #CDN_API_HDMIRX_SetHpd
 */
CDN_API_STATUS CDN_API_HDMIRX_SetHpd_blocking(state_struct *state, u8 hpd);

#endif
