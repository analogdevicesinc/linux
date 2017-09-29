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
 * This file was auto-generated. Do not edit it manually.
 *
 ******************************************************************************
 *
 * vic_table.c
 *
 ******************************************************************************
 */
#include "vic_table.h"

const unsigned int vic_table[VIC_MODE_COUNT][27] = {
	{858, 720, 138, 62, 16, 60, 525, 480, 45, 6, 9, 30, 59, 27000,
	 PROGRESSIVE, ACTIVE_LOW, ACTIVE_LOW, 1, 65535, 1, 46, 65535, 65535, 3,
	 8, 0},
	{1650, 1280, 370, 40, 110, 220, 750, 720, 30, 5, 5, 20, 60, 74250,
	 PROGRESSIVE, ACTIVE_HIGH, ACTIVE_HIGH, 1, 65535, 1, 31, 65535, 65535,
	 4, 8, 0},
	{2200, 1920, 280, 44, 88, 148, 1125, 1080, 45, 5, 4,
	 36, 60, 148500, PROGRESSIVE, ACTIVE_HIGH,
	 ACTIVE_HIGH, 1, 65535, 1, 46, 65535, 65535, 16, 8, 0},
	{4400, 3840, 560, 88, 176, 296, 2250, 2160, 90, 10, 8, 72, 60,
	 594000, PROGRESSIVE, ACTIVE_HIGH, ACTIVE_HIGH, 4, 266, 262, 22, 525,
	 285, 97, 8, 0},
	{4400, 3840, 560, 88, 176, 296, 2250, 2160, 90, 10, 8, 72, 30,
	 297000, PROGRESSIVE, ACTIVE_HIGH, ACTIVE_HIGH, 4, 266, 262, 22, 525,
	 285, 95, 8, 0},
};
