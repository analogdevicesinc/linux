/*
 * Xilinx AXI-JESD204B Interface Module
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "axi_jesd204b_v51.h"

struct jesd204b_state {
	struct device 		*dev;
	void __iomem		*regs;
	struct clk 		*clk;
	u32			lanes;
	u32			vers_id;
	u32			addr;
	u32			transmit;

	/* Used for probe ordering */
	struct clk_hw		dummy_clk;
};

/*
 * IO accessors
 */

static inline void jesd204b_write(struct jesd204b_state *st,
				  unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int jesd204b_read(struct jesd204b_state *st,
					 unsigned reg)
{
	return ioread32(st->regs + reg);
}

static ssize_t jesd204b_laneinfo_read(struct device *dev,
			struct device_attribute *attr,
			char *buf, unsigned lane)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);
	int ret;
	unsigned val1, val2, val3;


	val1 = jesd204b_read(st, XLNX_JESD204_REG_ID_L(lane));
	val2 = jesd204b_read(st, XLNX_JESD204_REG_LANE_F(lane));
	val3 = jesd204b_read(st, XLNX_JESD204_REG_SCR_S_HD_CF(lane));
	ret = sprintf(buf,
		"DID: %d, BID: %d, LID: %d, L: %d, SCR: %d, F: %d\n",
		XLNX_JESD204_LANE_DID(val1),
		XLNX_JESD204_LANE_BID(val1),
		XLNX_JESD204_LANE_LID(val1),
		XLNX_JESD204_LANE_L(val1),
		XLNX_JESD204_LANE_SCR(val3),
		XLNX_JESD204_LANE_F(val2));

	val1 = jesd204b_read(st, XLNX_JESD204_REG_LANE_K(lane));
	val2 = jesd204b_read(st, XLNX_JESD204_REG_M_N_ND_CS(lane));

	ret += sprintf(buf + ret,
		"K: %d, M: %d, N: %d, CS: %d, S: %d, N': %d, HD: %d\n",
		XLNX_JESD204_LANE_K(val1),
		XLNX_JESD204_LANE_M(val2),
		XLNX_JESD204_LANE_N(val2),
		XLNX_JESD204_LANE_CS(val2),
		XLNX_JESD204_LANE_S(val3),
		XLNX_JESD204_LANE_ND(val2),
		XLNX_JESD204_LANE_HD(val3));

	val1 = jesd204b_read(st, XLNX_JESD204_REG_FCHK(lane));
	ret += sprintf(buf + ret, "FCHK: 0x%X, CF: %d\n",
		XLNX_JESD204_LANE_FCHK(val1),
		XLNX_JESD204_LANE_CF(val3));

	val1 = jesd204b_read(st, XLNX_JESD204_REG_SC2_ADJ_CTRL(lane));
	val2 = jesd204b_read(st, XLNX_JESD204_REG_LANE_VERSION(lane));
	ret += sprintf(buf + ret,
		"ADJCNT: %d, PHADJ: %d, ADJDIR: %d, JESDV: %d, SUBCLASS: %d\n",
		XLNX_JESD204_LANE_ADJ_CNT(val1),
		XLNX_JESD204_LANE_PHASE_ADJ_REQ(val1),
		XLNX_JESD204_LANE_ADJ_CNT_DIR(val1),
		XLNX_JESD204_LANE_JESDV(val2),
		XLNX_JESD204_LANE_SUBCLASS(val2));

	ret += sprintf(buf + ret, "MFCNT : 0x%X\n",
		       jesd204b_read(st, XLNX_JESD204_REG_TM_MFC_CNT(lane)));
	ret += sprintf(buf + ret, "ILACNT: 0x%X\n",
		       jesd204b_read(st, XLNX_JESD204_REG_TM_ILA_CNT(lane)));
	ret += sprintf(buf + ret, "ERRCNT: 0x%X\n",
		       jesd204b_read(st, XLNX_JESD204_REG_TM_ERR_CNT(lane)));
	ret += sprintf(buf + ret, "BUFCNT: 0x%X\n",
		       jesd204b_read(st, XLNX_JESD204_REG_TM_BUF_ADJ(lane)));
	ret += sprintf(buf + ret, "LECNT: 0x%X\n",
		       jesd204b_read(st, XLNX_JESD204_REG_TM_LINK_ERR_CNT(lane)));

	ret += sprintf(buf + ret, "FC: %lu\n", clk_get_rate(st->clk));

	return ret;
}

#define JESD_LANE(_x) 						 \
static ssize_t jesd204b_lane##_x##_info_read(struct device *dev,	\
			struct device_attribute *attr,		\
			char *buf)				\
{								\
	return jesd204b_laneinfo_read(dev, attr, buf, _x);	\
}								\
static DEVICE_ATTR(lane##_x##_info, S_IRUSR, jesd204b_lane##_x##_info_read, NULL);

JESD_LANE(0);
JESD_LANE(1);
JESD_LANE(2);
JESD_LANE(3);
JESD_LANE(4);
JESD_LANE(5);
JESD_LANE(6);
JESD_LANE(7);

static ssize_t jesd204b_lane_syscstat_read(struct device *dev,
			struct device_attribute *attr,
			char *buf, unsigned lane)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);
	unsigned stat;

	stat = jesd204b_read(st, XLNX_JESD204_REG_SYNC_ERR_STAT);

	return sprintf(buf,
		"NOT_IN_TAB: %d, DISPARITY: %d, UNEXPECTED_K: %d\n",
		stat & XLNX_JESD204_SYNC_ERR_NOT_IN_TAB(lane),
		stat & XLNX_JESD204_SYNC_ERR_DISPARITY(lane),
		stat & XLNX_JESD204_SYNC_ERR_UNEXPECTED_K(lane));
}

#define JESD_SYNCSTAT_LANE(_x) 						 \
static ssize_t jesd204b_lane##_x##_syncstat_read(struct device *dev,	\
			struct device_attribute *attr,		\
			char *buf)				\
{								\
	return jesd204b_lane_syscstat_read(dev, attr, buf, _x);	\
}								\
static DEVICE_ATTR(lane##_x##_syncstat, S_IRUSR, jesd204b_lane##_x##_syncstat_read, NULL);

JESD_SYNCSTAT_LANE(0);
JESD_SYNCSTAT_LANE(1);
JESD_SYNCSTAT_LANE(2);
JESD_SYNCSTAT_LANE(3);
JESD_SYNCSTAT_LANE(4);
JESD_SYNCSTAT_LANE(5);
JESD_SYNCSTAT_LANE(6);
JESD_SYNCSTAT_LANE(7);


static ssize_t jesd204b_reg_write(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);
	unsigned val;
	int ret;

	ret = sscanf(buf, "%i %i", &st->addr, &val);

	if (ret == 2)
		jesd204b_write(st, st->addr, val);

	return count;
}
static ssize_t jesd204b_reg_read(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);

	return sprintf(buf, "0x%X\n", jesd204b_read(st, st->addr));
}

static DEVICE_ATTR(reg_access, S_IWUSR | S_IRUSR, jesd204b_reg_read, jesd204b_reg_write);

static const struct clk_ops clkout_ops = {};

/* Match table for of_platform binding */
static const struct of_device_id jesd204b_of_match[] = {
	{ .compatible = "xlnx,jesd204-5.1", .data = (void*) 51},
	{ .compatible = "xlnx,jesd204-5.2", .data = (void*) 51},
	{ .compatible = "xlnx,jesd204-6.0", .data = (void*) 51},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, jesd204b_of_match);

static int jesd204b_probe(struct platform_device *pdev)
{
	struct jesd204b_state *st;
	struct resource *mem; /* IO mem resources */
	struct clk *clk;
	struct clk_init_data init;
	const char *parent_name;
	const char *clk_name;
	struct clk *clk_out;
	unsigned frmcnt, bytecnt, subclass, val;
	int ret;

	const struct of_device_id *of_id =
			of_match_device(jesd204b_of_match, &pdev->dev);

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->dev = &pdev->dev;

	platform_set_drvdata(pdev, st);

	st->clk = clk;

	if (of_id && of_id->data)
		st->vers_id = (unsigned) of_id->data;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "xlnx,frames-per-multiframe", &frmcnt);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read required dt property\n");
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
				   "xlnx,bytes-per-frame", &bytecnt);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read required dt property\n");
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
				   "xlnx,subclass", &subclass);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read required dt property\n");
		return ret;
	}

	of_property_read_u32(pdev->dev.of_node,
			"xlnx,node-is-transmit", &st->transmit);

	ret = of_property_read_u32(pdev->dev.of_node,
			"xlnx,lanes", &st->lanes);
	if (ret)
		st->lanes = jesd204b_read(st, XLNX_JESD204_REG_LANES) + 1;


	jesd204b_write(st, XLNX_JESD204_REG_RESET,
		       XLNX_JESD204_RESET);

	jesd204b_write(st, XLNX_JESD204_REG_ILA_CTRL,
		       (of_property_read_bool(pdev->dev.of_node,
			"xlnx,lanesync-enable") ? XLNX_JESD204_ILA_EN : 0));

	jesd204b_write(st, XLNX_JESD204_REG_SCR_CTRL,
		       (of_property_read_bool(pdev->dev.of_node,
			"xlnx,scramble-enable") ? XLNX_JESD204_SCR_EN : 0));

	jesd204b_write(st, XLNX_JESD204_REG_SYSREF_CTRL,
		       (of_property_read_bool(pdev->dev.of_node,
			"xlnx,sysref-always-enable") ? XLNX_JESD204_ALWAYS_SYSREF_EN : 0));

	jesd204b_write(st, XLNX_JESD204_REG_ILA_MFC,
		       XLNX_JESD204_ILA_MFC(frmcnt));

	jesd204b_write(st, XLNX_JESD204_REG_OCTETS_PER_FRAME,
		       XLNX_JESD204_OCTETS_PER_FRAME(bytecnt));

	jesd204b_write(st, XLNX_JESD204_REG_FRAMES_PER_MFRAME,
		       XLNX_JESD204_FRAMES_PER_MFRAME(frmcnt));

	jesd204b_write(st, XLNX_JESD204_REG_SUBCLASS, subclass);

	device_create_file(&pdev->dev, &dev_attr_reg_access);

	switch (st->lanes) {
	case 8:
		device_create_file(&pdev->dev, &dev_attr_lane4_info);
		device_create_file(&pdev->dev, &dev_attr_lane5_info);
		device_create_file(&pdev->dev, &dev_attr_lane6_info);
		device_create_file(&pdev->dev, &dev_attr_lane7_info);
		if (!st->transmit) {
			device_create_file(&pdev->dev, &dev_attr_lane4_syncstat);
			device_create_file(&pdev->dev, &dev_attr_lane5_syncstat);
			device_create_file(&pdev->dev, &dev_attr_lane6_syncstat);
			device_create_file(&pdev->dev, &dev_attr_lane7_syncstat);
		}
		fallthrough;
	case 4:
		device_create_file(&pdev->dev, &dev_attr_lane2_info);
		device_create_file(&pdev->dev, &dev_attr_lane3_info);
		if (!st->transmit) {
			device_create_file(&pdev->dev, &dev_attr_lane2_syncstat);
			device_create_file(&pdev->dev, &dev_attr_lane3_syncstat);
		}
		fallthrough;
	case 2:
		device_create_file(&pdev->dev, &dev_attr_lane1_info);
		if (!st->transmit)
			device_create_file(&pdev->dev, &dev_attr_lane1_syncstat);
		fallthrough;
	case 1:
		device_create_file(&pdev->dev, &dev_attr_lane0_info);
		if (!st->transmit)
			device_create_file(&pdev->dev, &dev_attr_lane0_syncstat);
		break;
	default:

		break;
	}

	ret = of_property_read_string(pdev->dev.of_node, "clock-output-names",
		&clk_name);
	if (ret < 0)
		return ret;

	init.name = clk_name;
	init.ops = &clkout_ops;
	init.flags = CLK_SET_RATE_PARENT;

	parent_name = of_clk_get_parent_name(pdev->dev.of_node, 0);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	st->dummy_clk.init = &init;

	clk_out = clk_register(&pdev->dev, &st->dummy_clk);
	if (IS_ERR(clk_out))
		return PTR_ERR(clk_out);

	of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get, clk_out);

	val = jesd204b_read(st, XLNX_JESD204_REG_VERSION);

	dev_info(&pdev->dev, "AXI-JESD204B %d.%d Rev %d, at 0x%08llX mapped to 0x%p,",
		 XLNX_JESD204_VERSION_MAJOR(val),
		 XLNX_JESD204_VERSION_MINOR(val),
		 XLNX_JESD204_VERSION_REV(val),
		 (unsigned long long)mem->start, st->regs);

	return 0;
}

static int jesd204b_remove(struct platform_device *pdev)
{
	struct jesd204b_state *st = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);

	clk_disable_unprepare(st->clk);
	clk_put(st->clk);

	return 0;
}

static struct platform_driver jesd204b_v51_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = jesd204b_of_match,
	},
	.probe		= jesd204b_probe,
	.remove		= jesd204b_remove,
};

module_platform_driver(jesd204b_v51_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI-JESD204B Interface Module");
MODULE_LICENSE("GPL v2");
