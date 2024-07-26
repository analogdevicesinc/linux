// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timekeeping.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/init.h>

#include "ptp_private.h"
#include "ptp_adrv906x_tod.h"

MODULE_DESCRIPTION("Example driver for integrating the time-of-day in adrv906x to work with a clock pll");
MODULE_AUTHOR("Landau Zhang <landau.zhang@analog.com>");
MODULE_AUTHOR("Kim Holdt <kim.holdt@analog.com>");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");

struct adrv906x_hw_pll;

struct phc_pll_ops {
	int (*adjfine)(struct adrv906x_hw_pll *phc_clk, long scaled_ppm);
	int (*adjfreq)(struct adrv906x_hw_pll *phc_clk, s32 delta);
	int (*close)(struct adrv906x_hw_pll *phc_clk);
};

struct phc_pll_i2c_attr {
	u32 bus_addr;
	struct i2c_adapter *adpt;
};

struct adrv906x_hw_pll {
	long scaled_ppm;
	struct phc_pll_i2c_attr pll_i2c;
	spinlock_t reg_lock;
	struct phc_pll_ops pll_ops;
};

struct adrv906x_phc_pll {
	struct device *dev;
	struct adrv906x_hw_pll hw_pll;
};

#define AD9545_I2C_BUF_SIZE                     128

#define AD9545_ADDR_IO_UPDATE                   0x00F
#define AD9545_ADDR_NCO0_CENTER_FREQ_LSB        0x2800
#define AD9545_ADDR_MAX                         0x3A3F

#define ADDR_NCO0_CENTER_FREQ_CNT               7

#define ADRV906X_PHC_NCO_FREQ_TO_HZ(freq)           (freq >> 40)
#define ADRV906X_PHC_PPB_TO_PPT(ppb)                (ppb * 1000)

static int adrv906x_pll_i2c_read(struct adrv906x_hw_pll *hw_pll, u16 addr, u8 *buffer, size_t len)
{
	struct phc_pll_i2c_attr *pll_i2c = &hw_pll->pll_i2c;
	struct i2c_msg msg;
	int ret = -EIO;
	int rc = 0;
	u8 temp[2];

	if ((addr + (u16)len) >= AD9545_ADDR_MAX)
		return -EINVAL;

	temp[0] = (addr >> 8) & 0xFF;
	temp[1] = addr & 0xFF;

	msg.addr = pll_i2c->bus_addr;
	msg.flags = 0;
	msg.len = sizeof(temp);
	msg.buf = temp;

	rc = i2c_transfer(pll_i2c->adpt, &msg, 1);
	if (rc == 1)
		ret = 0;
	else if (rc > 0)
		return -EREMOTEIO;
	else
		return rc;

	msg.addr = pll_i2c->bus_addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = buffer;

	rc = i2c_transfer(pll_i2c->adpt, &msg, 1);
	if (rc == 1)
		ret = 0;
	else if (rc > 0)
		return -EREMOTEIO;
	else
		return rc;

	return ret;
}

static int adrv906x_pll_i2c_write(struct adrv906x_hw_pll *hw_pll, u16 addr, const u8 *buffer, size_t len)
{
	struct phc_pll_i2c_attr *pll_i2c = &hw_pll->pll_i2c;
	struct i2c_msg msg;
	int ret = -EIO;
	int rc = 0;
	int i;
	u8 temp[ADDR_NCO0_CENTER_FREQ_CNT + 2];

	if (((addr + (u16)len) >= AD9545_ADDR_MAX) || (len >= AD9545_I2C_BUF_SIZE - 2))
		return -EINVAL;

	temp[0] = (addr >> 8) & 0xFF;
	temp[1] = addr & 0xFF;
	for (i = 2; i < sizeof(temp); i++)
		temp[i] = buffer[i - 2];

	msg.addr = pll_i2c->bus_addr;
	msg.flags = 0;
	msg.len = sizeof(temp);
	msg.buf = temp;

	rc = i2c_transfer(pll_i2c->adpt, &msg, 1);
	if (rc == 1)
		ret = 0;
	else if (rc > 0)
		return -EREMOTEIO;
	else
		return rc;

	return ret;
}

static int adrv906x_pll_sync_ad9545(struct adrv906x_hw_pll *hw_pll)
{
	u8 sync = 0x01;

	return adrv906x_pll_i2c_write(hw_pll, AD9545_ADDR_IO_UPDATE, &sync, 1);
}

static int adrv906x_pll_get_freq_ad9545(struct adrv906x_hw_pll *hw_pll, u64 *freq)
{
	u8 data_bytes[8];
	int i;
	int ret =
		adrv906x_pll_i2c_read(hw_pll, AD9545_ADDR_NCO0_CENTER_FREQ_LSB,
				      data_bytes, ADDR_NCO0_CENTER_FREQ_CNT);

	*freq = 0;

	for (i = 0; i < 8; i++)
		*freq |= ((u64)data_bytes[i] << (8 * i));

	return ret;
}

static int adrv906x_pll_set_freq_ad9545(struct adrv906x_hw_pll *hw_pll, u64 freq)
{
	u8 data_bytes[ADDR_NCO0_CENTER_FREQ_CNT + 1];
	int i;

	for (i = 0; i < ADDR_NCO0_CENTER_FREQ_CNT; i++)
		data_bytes[i] = (freq >> (8 * i)) & 0xFF;

	return adrv906x_pll_i2c_write(hw_pll, AD9545_ADDR_NCO0_CENTER_FREQ_LSB,
				      data_bytes, ADDR_NCO0_CENTER_FREQ_CNT);
}

/*
 * @brief Adjusts the frequency of the hardware clock.
 *
 *  This function should be implemented by the user and depend on the clock chip used
 *  param:
 *       adrv906x_hw_pll     -   hardware clock operation information structure
 *       scaled_ppm -   Desired frequency offset from nominal frequency in parts per million, but with a
 *                      16 bit binary fractional field.
 */
static int adrv906x_pll_adjfine_ad9545(struct adrv906x_hw_pll *hw_pll, long scaled_ppm)
{
	int neg_adj = 0;
	static u64 org_freq;
	u64 updated_freq;
	u64 tar_freq;
	u64 freq_hz;
	u64 adj;
	s64 ppt;
	s32 ppb;
	int err;

	ppb = scaled_ppm_to_ppb(scaled_ppm);
	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	/*
	 * Get the original NCO value which is a 7 * 8(bit) = 56(bit) value
	 * Center frequency of AUX-PLL in ad9545:
	 * ---------------------------------------------------------------
	 * | 55 |    ...     | 40 | 39 |             ....            | 0 |
	 * | ->  INTEGER Hz    <- | ->          FRACTIONAL HZ         <- |
	 *                      |                                      |
	 *                     1 Hz                                2^(-40) Hz
	 */
	err = adrv906x_pll_get_freq_ad9545(hw_pll, &updated_freq);
	updated_freq &= ~__GENMASK(63, 56); /* Zero out the 8 most significant bits. */
	if (err)
		return err;
	if (ADRV906X_PHC_NCO_FREQ_TO_HZ(abs(updated_freq - org_freq)) >
	    ADRV906X_PHC_NCO_FREQ_TO_HZ(updated_freq) - 1)
		org_freq = updated_freq; /* Keeping only the base frequency for updates. */

	/* Convert ppb to ppt so that we can easily calculate the offset frequency value */
	ppt = ADRV906X_PHC_PPB_TO_PPT(ppb);

	freq_hz = ADRV906X_PHC_NCO_FREQ_TO_HZ(org_freq);        /* Get in the HZ */
	adj = freq_hz * ppt;                                    /* Find requested adjustment in ppt - i.e. LSB corresponds to approximately 2^(-40) Hz */

	/* Update the target NCO value */
	if (neg_adj == 1)
		tar_freq = org_freq - adj;
	else
		tar_freq = org_freq + adj;

	/* adjust the frequency */
	err = adrv906x_pll_set_freq_ad9545(hw_pll, tar_freq);
	if (err)
		return err;

	err = adrv906x_pll_sync_ad9545(hw_pll);

	return err;
}

static int adrv906x_pll_get_adapter(struct adrv906x_hw_pll *hw_pll)
{
	struct adrv906x_phc_pll *pll_phc = container_of(hw_pll, struct adrv906x_phc_pll, hw_pll);
	struct device *dev = pll_phc->dev;
	struct device_node *i2c_pll_node;
	struct device_node *i2c_mux_node;
	struct device_node *pll_np;

	pll_np = of_get_child_by_name(dev->of_node, "clock-pll");
	if (!pll_np)
		return -ENODEV;

	i2c_pll_node = of_parse_phandle(pll_np, "adi,i2c-clk", 0);
	if (!i2c_pll_node) {
		dev_err(dev, "No clk node is found");
		return -EINVAL;
	}
	of_property_read_u32(i2c_pll_node, "reg", &hw_pll->pll_i2c.bus_addr);

	i2c_mux_node = of_get_parent(i2c_pll_node);
	if (!i2c_mux_node) {
		dev_err(dev, "No parent device node of clk node is found");
		of_node_put(i2c_pll_node);
		return -EINVAL;
	}

	hw_pll->pll_i2c.adpt = of_find_i2c_adapter_by_node(i2c_mux_node);

	of_node_put(i2c_pll_node);
	of_node_put(i2c_mux_node);

	if (!hw_pll->pll_i2c.adpt) {
		dev_err(dev, "No adapter of the clk node is found");
		return -ENODEV;
	}

	return 0;
}

static int adrv906x_pll_i2c_probe(struct adrv906x_hw_pll *hw_pll)
{
	struct adrv906x_phc_pll *pll_phc = container_of(hw_pll, struct adrv906x_phc_pll, hw_pll);
	struct device *dev = pll_phc->dev;
	struct device_node *iic_pll_node;
	struct device_node *pll_np;
	int ret;

	pll_np = of_get_child_by_name(dev->of_node, "clock-pll");
	if (!pll_np)
		return -ENODEV;

	iic_pll_node = of_parse_phandle(pll_np, "adi,i2c-clk", 0);

	if (iic_pll_node) {
		ret = adrv906x_pll_get_adapter(hw_pll);
	} else {
		dev_err(dev, "clk node not found");
		ret = -ENODEV;
	}

	return ret;
}

static int adrv906x_pll_i2c_remove(struct adrv906x_hw_pll *hw_pll)
{
	i2c_put_adapter(hw_pll->pll_i2c.adpt);
	return 0;
}

struct phc_pll_ops adrv906x_pll_ops = {
	.adjfine	= &adrv906x_pll_adjfine_ad9545,
	.close		= &adrv906x_pll_i2c_remove,
};

static int adrv906x_phc_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct adrv906x_tod_counter *counter = container_of(ptp, struct adrv906x_tod_counter, caps);
	struct adrv906x_tod *tod = counter->parent;
	struct platform_device *pdev = container_of(tod->dev, struct platform_device, dev);
	struct adrv906x_phc_pll *pll_phc = platform_get_drvdata(pdev);
	struct adrv906x_hw_pll *hw_pll = &pll_phc->hw_pll;
	int err;

	if (hw_pll->pll_ops.adjfine) {
		err = hw_pll->pll_ops.adjfine(hw_pll, scaled_ppm);
	} else {
		dev_err(pll_phc->dev, "function not supported");
		err = -EOPNOTSUPP;
	}
	return err;
}

static struct ptp_clock_info adrv906x_pll_caps = {
	.owner		= THIS_MODULE,
	.name		= "adrv906x soc ptp",
	.max_adj	= 5000,
	.adjfine	= &adrv906x_phc_adjfine,
	.adjfreq	= NULL,
};

int adrv906x_phc_pll_probe(struct adrv906x_phc_pll *pll_phc)
{
	struct adrv906x_hw_pll *hw_pll = &pll_phc->hw_pll;
	struct device *dev = pll_phc->dev;
	struct device_node *pll_np;
	struct device_node *np;
	int ret;

	np = dev->of_node;
	pll_np = of_get_child_by_name(np, "clock-pll");
	if (!pll_np) {
		dev_err(dev, "miss clock pll device node");
		ret = -ENODEV;
		goto probe_error;
	}

	if (of_find_property(pll_np, "adi,i2c-clk", NULL)) {
		ret = adrv906x_pll_i2c_probe(hw_pll);
		if (ret == -ENODEV) {
			ret = -EPROBE_DEFER;
			dev_err(dev, "miss i2c clock device node");
			goto probe_error;
		}
		if (ret == 0) {
			hw_pll->pll_ops = adrv906x_pll_ops;
			goto probe_ok;
		}
	} else {
		dev_err(dev, "No valid phc hardware clock chip");
		ret = -ENODEV;
		goto probe_error;
	}

probe_ok:
	if (ret == 0)
		spin_lock_init(&hw_pll->reg_lock);
probe_error:
	if (ret == -EPROBE_DEFER)
		dev_err(dev, "No valid phc hardware clock chip, defer probing");
	else if (ret != 0)
		dev_err(dev, "PHC pll clock probe error");
	else
		dev_info(dev, "PHC pll clock probe ok");

	return ret;
}

int adrv906x_pll_remove(struct adrv906x_phc_pll *pll_phc)
{
	struct adrv906x_hw_pll *hw_pll = &pll_phc->hw_pll;

	if (hw_pll->pll_ops.close)
		return hw_pll->pll_ops.close(hw_pll);

	return 0;
}

static int adrv906x_ptp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adrv906x_phc_pll *pll_phc;
	int ret;

	pll_phc = devm_kzalloc(dev, sizeof(struct adrv906x_phc_pll), GFP_KERNEL);
	if (!pll_phc)
		return -ENOMEM;

	pll_phc->dev = dev;

	ret = adrv906x_tod_probe(pdev);
	if (ret)
		return ret;

	ret = adrv906x_phc_pll_probe(pll_phc);
	if (ret)
		goto err_out;

	ret = adrv906x_tod_register_pll(&adrv906x_pll_caps);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, pll_phc);
	return 0;

err_out:
	adrv906x_tod_remove(pdev);
	return ret;
}

static int adrv906x_ptp_remove(struct platform_device *pdev)
{
	struct adrv906x_phc_pll *pll_phc = platform_get_drvdata(pdev);
	int ret;

	ret = adrv906x_tod_remove(pdev);
	if (ret)
		return ret;

	return adrv906x_pll_remove(pll_phc);
}

static const struct of_device_id ptp_adrv906x_soc_of_match[] = {
	{ .compatible = "adi,adrv906x-ptp", },
	{},
};

MODULE_DEVICE_TABLE(of, ptp_adrv906x_soc_of_match);

static struct platform_driver ptp_adrv906x_soc_driver = {
	.driver			= {
		.name		= "adrv906x-ptp",
		.of_match_table = ptp_adrv906x_soc_of_match,
	},
	.probe			= adrv906x_ptp_probe,
	.remove			= adrv906x_ptp_remove,
};

module_platform_driver(ptp_adrv906x_soc_driver);
