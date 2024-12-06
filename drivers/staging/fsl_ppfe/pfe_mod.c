// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include <linux/dma-mapping.h>
#include "pfe_mod.h"
#include "pfe_cdev.h"

unsigned int us;
module_param(us, uint, 0444);
MODULE_PARM_DESC(us, "0: module enabled for kernel networking (DEFAULT)\n"
			"1: module enabled for userspace networking\n");
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

	if (us)
		goto firmware_init;

	rc = pfe_hif_lib_init(pfe);
	if (rc < 0)
		goto err_hif_lib;

	rc = pfe_hif_init(pfe);
	if (rc < 0)
		goto err_hif;

firmware_init:
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

	if (us) {
		/* Creating a character device */
		rc = pfe_cdev_init();
		if (rc < 0)
			goto err_cdev;
	}

	return 0;

err_cdev:
	pfe_debugfs_exit(pfe);

err_debugfs:
	pfe_sysfs_exit(pfe);

err_sysfs:
	pfe_eth_exit(pfe);

err_eth:
	pfe_ctrl_exit(pfe);

err_ctrl:
	pfe_firmware_exit(pfe);

err_firmware:
	if (us)
		goto err_hif_lib;

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

	if (us)
		pfe_cdev_exit();

	pfe_debugfs_exit(pfe);

	pfe_sysfs_exit(pfe);

	pfe_eth_exit(pfe);

	pfe_ctrl_exit(pfe);

#if defined(LS1012A_PFE_RESET_WA)
	pfe_hif_rx_idle(&pfe->hif);
#endif
	pfe_firmware_exit(pfe);

	if (us)
		goto hw_exit;

	pfe_hif_exit(pfe);

	pfe_hif_lib_exit(pfe);

hw_exit:
	pfe_hw_exit(pfe);

	return 0;
}
