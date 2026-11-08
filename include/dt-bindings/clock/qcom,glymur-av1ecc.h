/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _DT_BINDINGS_CLK_QCOM_AV1E_CC_GLYMUR_H
#define _DT_BINDINGS_CLK_QCOM_AV1E_CC_GLYMUR_H

/* AV1E_CC clocks */
#define AV1E_CC_AHB_CLK						0
#define AV1E_CC_AV1E_CORE_APB_CLK				1
#define AV1E_CC_AV1E_CORE_AXI_CLK				2
#define AV1E_CC_AV1E_CORE_CLK					3
#define AV1E_CC_AV1E_CORE_CLK_SRC				4
#define AV1E_CC_AV1E_GDSC_NOC_AHB_CLK				5
#define AV1E_CC_AV1E_NOC_AHB_CLK				6
#define AV1E_CC_AV1E_NOC_AHB_DIV_CLK_SRC			7
#define AV1E_CC_AV1E_NOC_CORE_AXI_CLK				8
#define AV1E_CC_AV1E_NOC_XO_CLK					9
#define AV1E_CC_AV1E_XO_CLK_SRC					10
#define AV1E_CC_PLL0						11

/* AV1E_CC power domains */
#define AV1E_CC_CORE_GDSC					0

/* AV1E_CC resets */
#define AV1E_CC_AV1E_CORE_AXI_CLK_ARES				0
#define AV1E_CC_AV1E_CORE_CLK_ARES				1
#define AV1E_CC_AV1E_GDSC_NOC_AHB_CLK_ARES			2
#define AV1E_CC_AV1E_NOC_CORE_AXI_CLK_ARES			3
#define AV1E_CC_AV1E_NOC_XO_CLK_ARES				4
#define AV1E_CC_CORE_BCR					5
#define AV1E_CC_NOC_BCR						6

#endif
