/*
 * FPGA Manager Driver for Altera Arria10 SoCFPGA
 *
 * Copyright (C) 2015 Altera Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fpga.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#define ALT_FPGAMGR_DCLKCNT_OFST				0x8
#define ALT_FPGAMGR_DCLKSTAT_OFST				0xc
#define ALT_FPGAMGR_IMGCFG_CTL_00_OFST				0x70
#define ALT_FPGAMGR_IMGCFG_CTL_01_OFST				0x74
#define ALT_FPGAMGR_IMGCFG_CTL_02_OFST				0x78
#define ALT_FPGAMGR_IMGCFG_STAT_OFST				0x80

#define ALT_FPGAMGR_DCLKSTAT_DCLKDONE_SET_MSK			0x1

#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_NCONFIG_SET_MSK	0x00000001
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_NSTATUS_SET_MSK	0x00000002
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_CONDONE_SET_MSK	0x00000004
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NCONFIG_SET_MSK		0x00000100
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NSTATUS_OE_SET_MSK	0x00010000
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_CONDONE_OE_SET_MSK	0x01000000

#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NENABLE_CONFIG_SET_MSK	0x00000001
#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_PR_REQUEST_SET_MSK	0x00010000
#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NCE_SET_MSK		0x01000000

#define ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_CTRL_SET_MSK		0x00000001
#define ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_DATA_SET_MSK		0x00000100
#define ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_SET_MSK		0x00030000
#define ALT_FPGAMGR_IMGCFG_CTL_02_CFGWIDTH_SET_MSK		0x01000000
#define ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_LSB			16

#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CRC_ERROR_SET_MSK		0x00000001
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_EARLY_USERMODE_SET_MSK	0x00000002
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_USERMODE_SET_MSK		0x00000004
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_INITDONE_OE_SET_MSK		0x00000008
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTATUS_PIN_SET_MSK		0x00000010
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTATUS_OE_SET_MSK		0x00000020
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_PIN_SET_MSK		0x00000040
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_OE_SET_MSK		0x00000080
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CVP_CONF_DONE_SET_MSK	0x00000100
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_READY_SET_MSK		0x00000200
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_DONE_SET_MSK		0x00000400
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_ERROR_SET_MSK		0x00000800
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NCONFIG_PIN_SET_MSK		0x00001000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NCEO_OE_SET_MSK		0x00002000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK		0x00010000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK		0x00020000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK		0x00040000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL_SET_MSD (\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_IMGCFG_FIFOEMPTY_SET_MSK	0x01000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_IMGCFG_FIFOFULL_SET_MSK	0x02000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_JTAGM_SET_MSK		0x10000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_EMR_SET_MSK			0x20000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_LSB        16

static u32 socfpga_a10_fpga_read_stat(struct fpga_manager *mgr)
{
	return fpga_mgr_reg_readl(mgr, ALT_FPGAMGR_IMGCFG_STAT_OFST);
}

static int socfpga_a10_fpga_status(struct fpga_manager *mgr, char *buf)
{
	u32 reg;

	reg = socfpga_a10_fpga_read_stat(mgr);

	if (reg & ALT_FPGAMGR_IMGCFG_STAT_F2S_USERMODE_SET_MSK)
		return sprintf(buf, "user mode\n");

	if (reg & ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_READY_SET_MSK)
		return sprintf(buf, "configuration phase\n");

	if (reg & ALT_FPGAMGR_IMGCFG_STAT_F2S_CRC_ERROR_SET_MSK)
		return sprintf(buf, "crc error\n");

	if ((reg & ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTATUS_PIN_SET_MSK) == 0)
		return sprintf(buf, "reset phase\n");

	return sprintf(buf, "undetermined\n");
}

struct fpga_manager_ops socfpga_a10_fpga_mgr_ops = {
	.status = socfpga_a10_fpga_status,
};

static int socfpga_a10_fpga_probe(struct platform_device *pdev)
{
	return register_fpga_manager(pdev, &socfpga_a10_fpga_mgr_ops,
				     "SoCFPGA Arria10 FPGA Manager", NULL);
}

static int socfpga_a10_fpga_remove(struct platform_device *pdev)
{
	remove_fpga_manager(pdev);
	return 0;
}

static const struct of_device_id socfpga_a10_fpga_of_match[] = {
	{ .compatible = "altr,socfpga-a10-fpga-mgr", },
	{},
};

MODULE_DEVICE_TABLE(of, socfpga_a10_fpga_of_match);

static struct platform_driver socfpga_a10_fpga_driver = {
	.remove = socfpga_a10_fpga_remove,
	.driver = {
		.name	= "socfpga_a10_fpga_manager",
		.owner	= THIS_MODULE,
		.of_match_table = socfpga_a10_fpga_of_match,
	},
};

static int __init socfpga_a10_fpga_init(void)
{
	return platform_driver_probe(&socfpga_a10_fpga_driver,
				     socfpga_a10_fpga_probe);
}

static void __exit socfpga_a10_fpga_exit(void)
{
	platform_driver_unregister(&socfpga_a10_fpga_driver);
}

module_init(socfpga_a10_fpga_init);
module_exit(socfpga_a10_fpga_exit);

MODULE_AUTHOR("Alan Tull <atull@opensource.altera.com>");
MODULE_DESCRIPTION("SoCFPGA Arria10 FPGA Manager");
MODULE_LICENSE("GPL v2");
