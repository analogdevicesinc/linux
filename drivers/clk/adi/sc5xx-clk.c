// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock driver for ADSP-SC5xx processors
 *
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * Author: Qasim Ijaz <qasim.ijaz@analog.com>
 * Contact: linux@analog.com
 */

#include "clk.h"

static void adi_sc5xx_cgu_register(struct device_node *np, void *data)
{

}
	
static void __init __adi_sc5xx_cgu_register(struct device_node *np)
{
	adi_sc5xx_cgu_register(np, 
}

CLK_OF_DECLARE(sc598_cgu0, "adi,sc598-cgu0", adi_sc5xx_cgu_of_init);
CLK_OF_DECLARE(sc598_cgu1, "adi,sc598-cgu1", adi_sc5xx_cgu_of_init);
CLK_OF_DECLARE(sc598_cgu2, "adi,sc598-cgu2", adi_sc5xx_cgu_of_init);
CLK

CLK_OF_DECLARE(sc589_cgu0, "adi,sc589-cgu0", adi_sc5xx_cgu_of_init);
CLK_OF_DECLARE(sc589_cgu1, "adi,sc589-cgu1", adi_sc5xx_cgu_of_init);

CLK_OF_DECLARE(sc594_cgu0, "adi,sc594-cgu0", adi_sc5xx_cgu_of_init);
CLK_OF_DECLARE(sc594_cgu1, "adi,sc594-cgu1", adi_sc5xx_cgu_of_init);

CLK_OF_DECLARE(sc573_cgu0, "adi,sc573-cgu0", adi_sc5xx_cgu_of_init);
CLK_OF_DECLARE(sc573_cgu1, "adi,sc573-cgu1", adi_sc5xx_cgu_of_init);

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Clock driver for ADSP-SC5xx SoCs");
MODUULE_LICENSE("GPLv2");
