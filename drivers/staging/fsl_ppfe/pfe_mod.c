/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/dma-mapping.h>
#include "pfe_mod.h"

struct pfe *pfe;

/*
 * pfe_probe -
 */
int pfe_probe(struct pfe *pfe)
{
	int rc;

	if (pfe->ddr_size < DDR_MAX_SIZE) {
		pr_err("%s: required DDR memory (%x) above platform ddr memory (%x)\n",
		       __func__, (unsigned int)DDR_MAX_SIZE, pfe->ddr_size);
		rc = -ENOMEM;
		goto err_hw;
	}

	if (((int)(pfe->ddr_phys_baseaddr + BMU2_DDR_BASEADDR) &
			(8 * SZ_1M - 1)) != 0) {
		pr_err("%s: BMU2 base address (0x%x) must be aligned on 8MB boundary\n",
		       __func__, (int)pfe->ddr_phys_baseaddr +
			BMU2_DDR_BASEADDR);
		rc = -ENOMEM;
		goto err_hw;
	}

	pr_info("cbus_baseaddr: %lx, ddr_baseaddr: %lx, ddr_phys_baseaddr: %lx, ddr_size: %x\n",
		(unsigned long)pfe->cbus_baseaddr,
		(unsigned long)pfe->ddr_baseaddr,
		pfe->ddr_phys_baseaddr, pfe->ddr_size);

	pfe_lib_init(pfe->cbus_baseaddr, pfe->ddr_baseaddr,
		     pfe->ddr_phys_baseaddr, pfe->ddr_size);

	rc = pfe_hw_init(pfe, 0);
	if (rc < 0)
		goto err_hw;

	rc = pfe_hif_lib_init(pfe);
	if (rc < 0)
		goto err_hif_lib;

	rc = pfe_hif_init(pfe);
	if (rc < 0)
		goto err_hif;

	rc = pfe_firmware_init(pfe);
	if (rc < 0)
		goto err_firmware;

	rc = pfe_ctrl_init(pfe);
	if (rc < 0)
		goto err_ctrl;

	rc = pfe_eth_init(pfe);
	if (rc < 0)
		goto err_eth;

	rc = pfe_sysfs_init(pfe);
	if (rc < 0)
		goto err_sysfs;

	rc = pfe_debugfs_init(pfe);
	if (rc < 0)
		goto err_debugfs;

	return 0;

err_debugfs:
	pfe_sysfs_exit(pfe);

err_sysfs:
	pfe_eth_exit(pfe);

err_eth:
	pfe_ctrl_exit(pfe);

err_ctrl:
	pfe_firmware_exit(pfe);

err_firmware:
	pfe_hif_exit(pfe);

err_hif:
	pfe_hif_lib_exit(pfe);

err_hif_lib:
	pfe_hw_exit(pfe);

err_hw:
	return rc;
}

/*
 * pfe_remove -
 */
int pfe_remove(struct pfe *pfe)
{
	pr_info("%s\n", __func__);

	pfe_debugfs_exit(pfe);

	pfe_sysfs_exit(pfe);

	pfe_eth_exit(pfe);

	pfe_ctrl_exit(pfe);

#if defined(LS1012A_PFE_RESET_WA)
	pfe_hif_rx_idle(&pfe->hif);
#endif
	pfe_firmware_exit(pfe);

	pfe_hif_exit(pfe);

	pfe_hif_lib_exit(pfe);

	pfe_hw_exit(pfe);

	return 0;
}
