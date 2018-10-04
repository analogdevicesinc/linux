/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2012-2013 by Tensilica Inc. ALL RIGHTS RESERVED.
 * Copyright 2018 NXP
 */

#ifndef _FSL_DSP_PLATFORM_H
#define _FSL_DSP_PLATFORM_H

#include "fsl_dsp_xaf_api.h"

struct dsp_data {
	struct xf_client      *client;
	struct xaf_pipeline   *p_pipe;
	struct xaf_pipeline    pipeline;
	struct xaf_comp        component[2];
	int codec_type;
	int status;
};

#endif /*_FSL_DSP_PLATFORM_H*/
