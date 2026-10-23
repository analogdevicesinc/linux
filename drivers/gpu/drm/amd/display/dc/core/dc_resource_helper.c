/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "dc_types.h"
#include "sspl/dc_spl_types.h"
#include "resource.h"

enum upsp_mode resource_is_upsp_required(enum surface_pixel_format format,
		enum dc_scaling_linearity scaling_linearity)
{
	bool scaling_in_linear = (scaling_linearity == DC_SCALING_LINEARITY_LINEAR);
	bool is_420_format = (format >= SURFACE_PIXEL_FORMAT_VIDEO_BEGIN &&
		format <= SURFACE_PIXEL_FORMAT_VIDEO_420_10bpc_YCrCb);
	bool is_422_format = (format > SURFACE_PIXEL_FORMAT_VIDEO_420_10bpc_YCrCb &&
		format < SURFACE_PIXEL_FORMAT_SUBSAMPLE_END);

	/* UPSP (chroma upsampling) is only needed when subsampled YUV is scaled in
	 * linear space.
	 */
	if (scaling_in_linear && is_420_format)
		return UPSP_HORIZONTAL_VERTICAL_UPSAMPLING;
	if (scaling_in_linear && is_422_format)
		return UPSP_HORIZONTAL_UPSAMPLING_ONLY;
	return UPSP_BYPASS;
}
