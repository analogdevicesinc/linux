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
 * address.h
 *
 ******************************************************************************
 */

#ifndef ADDRESS_H_
# define ADDRESS_H_

# define ADDR_IMEM 0x10000
# define ADDR_DMEM 0x20000
# define ADDR_CIPHER 0x60000
# define BASE_CIPHER 0x600
# define ADDR_APB_CFG 0x00000
# define BASE_APB_CFG 0x000
# define ADDR_SOURCE_AIF_DECODER 0x30000
# define BASE_SOURCE_AIF_DECODER 0x300
# define ADDR_SOURCE_AIF_SMPL2PCKT 0x30080
# define BASE_SOURCE_AIF_SMPL2PCKT 0x300
# define ADDR_AIF_ENCODER 0x30000
# define BASE_AIF_ENCODER 0x300
# define ADDR_SOURCE_PIF 0x30800
# define BASE_SOURCE_PIF 0x308
# define ADDR_SINK_PIF 0x30800
# define BASE_SINK_PIF 0x308
# define ADDR_APB_CFG 0x00000
# define BASE_APB_CFG 0x000
# define ADDR_SOURCE_CSC 0x40000
# define BASE_SOURCE_CSC 0x400
# define ADDR_UCPU_CFG 0x00000
# define BASE_UCPU_CFG 0x000
# define ADDR_SOURCE_CAR 0x00900
# define BASE_SOURCE_CAR 0x009
# define ADDR_SINK_CAR 0x00900
# define BASE_SINK_CAR 0x009
# define ADDR_CLOCK_METERS 0x00A00
# define BASE_CLOCK_METERS 0x00A
# define ADDR_SOURCE_VIF 0x00b00
# define BASE_SOURCE_VIF 0x00b
# define ADDR_SINK_MHL_HD 0x01000
# define BASE_SINK_MHL_HD 0x010
# define ADDR_SINK_CORE 0x07800
# define BASE_SINK_CORE 0x078
# define ADDR_DPTX_PHY 0x02000
# define BASE_DPTX_PHY 0x020
# define ADDR_DPTX_HPD 0x02100
# define BASE_DPTX_HPD 0x021
# define ADDR_DPTX_FRAMER 0x02200
# define BASE_DPTX_FRAMER 0x022
# define ADDR_DPTX_STREAM 0x02200
# define BASE_DPTX_STREAM 0x022
# define ADDR_DPTX_GLBL 0x02300
# define BASE_DPTX_GLBL 0x023
# define ADDR_DPTX_HDCP 0x02400
# define BASE_DPTX_HDCP 0x024
# define ADDR_DP_AUX 0x02800
# define BASE_DP_AUX 0x028
# define ADDR_CRYPTO 0x05800
# define BASE_CRYPTO 0x058
# define ADDR_CIPHER 0x60000
# define BASE_CIPHER 0x600
# define ADDR_SOURCE_MHL_HD 0x01000

# define ADDR_AFE  (0x20000 * 4)
# define ADDR_SOURCD_PHY  (0x800)

#endif
