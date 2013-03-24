/*
 * ADI-FFT Module
 *
 * Copyright 2012 Analog Devices Inc.
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
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/log2.h>

/***************************************************************************
 * register definitions (fft)
 * 5'h00:
 *  [31: 0]: version(32'h00010061)
 * 5'h01:
 *  [31: 0]: up_cfg_data, fft cfg data (refer Xilinx fft core for details).
 * 5'h02:
 *  [ 3: 3]: up_status_ovf, fft overflow (mapped to Xilinx fft core outputs).
 *  [ 2: 2]: up_status_lm, fft tlast missing (mapped to core).
 *  [ 1: 1]: up_status_lu, fft tlast unexpected (mapped to core).
 *  [ 0: 0]: up_status_fs, fft frame start (mapped to core).
 ***************************************************************************/

#define FFT_PCORE_VERSION	0x0
#define FFT_PCORE_CFG		0x4
#define FFT_PCORE_STAT		0x8
#define FFT_PCORE_POSTPROC	0xC
#define FFT_PCORE_IRSEL		0x10
#define FFT_PCORE_WINCFG	0x14

#define FFT_PCORE_CFG_DEFAULT	0xD5700
#define FFT_PCORE_CFG_FW_FFT	(1 << 8)
#define FFT_PCORE_CFG_INV_FFT	(0 << 8)

#define FFT_PCORE_POSTPROC_EN	0x1

#define FFT_PCORE_IRSEL_I(x)	((x) << 2)
#define FFT_PCORE_IRSEL_R(x)	((x) << 0)
#define FFT_PCORE_IRSEL_ZERO	0
#define FFT_PCORE_IRSEL_CH0	1
#define FFT_PCORE_IRSEL_CH1	2


#define FFT_PCORE_WINCFG_INC(x)	((x) & 0xFFFF)
#define FFT_PCORE_WINCFG_ENB	(1 << 16)

struct fft_state {
	struct device 			*dev;
	struct mutex			lock;
	struct completion		dma_complete;
	struct dma_chan			*rx_chan;
	struct dma_chan			*tx_chan;
	void __iomem			*regs;
	int				compl_stat;
};

struct fft_dma_params {
	struct device_node *of_node;
	enum dma_transfer_direction direction;
	int chan_id;
};

struct fft_state *fft_state_glob;

/*
 * IO accessors
 */

static inline void fft_write(struct fft_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int fft_read(struct fft_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static bool fft_dma_filter(struct dma_chan *chan, void *param)
{
	struct fft_dma_params *p = param;

	return chan->device->dev->of_node == p->of_node &&
		chan->chan_id == p->chan_id;
}

int fft_calculate(dma_addr_t src, dma_addr_t dest, unsigned int size, unsigned irsel)
{
	struct fft_state *st = fft_state_glob;
	struct dma_async_tx_descriptor *tx_desc, *rx_desc;
	dma_cookie_t tx_cookie, rx_cookie;
	unsigned nfft = ilog2(size);
	int ret = 0;

	if (st == NULL)
		return -ENODEV;

	if (((1 << nfft) != size) || !src || !dest || !irsel)
		return -EINVAL;

	switch (irsel) {
		case FFT_PCORE_IRSEL_CH0:
		case FFT_PCORE_IRSEL_CH1:
			irsel = FFT_PCORE_IRSEL_R(irsel);
			break;
		case FFT_PCORE_IRSEL_CH0 | FFT_PCORE_IRSEL_CH1:
			irsel = FFT_PCORE_IRSEL_R(FFT_PCORE_IRSEL_CH0) |
				FFT_PCORE_IRSEL_I(FFT_PCORE_IRSEL_CH1);
			break;
	}

	mutex_lock(&st->lock);
//	printk("%s: %d::: %d %x\n",__func__,__LINE__, size, irsel);
	fft_write(st, FFT_PCORE_WINCFG, 0);
	fft_write(st, FFT_PCORE_WINCFG, BIT(16) | (0x10000 >> nfft));
	fft_write(st, FFT_PCORE_POSTPROC, FFT_PCORE_POSTPROC_EN);
	fft_write(st, FFT_PCORE_IRSEL, irsel);
	fft_write(st, FFT_PCORE_STAT, 0xFF);
	fft_write(st, FFT_PCORE_CFG, FFT_PCORE_CFG_DEFAULT | nfft);

	tx_desc = dmaengine_prep_slave_single(st->tx_chan, src, size * 4,
					   DMA_TO_DEVICE, 0);
	if (!tx_desc) {
		dev_err(st->dev,
			"Failed to allocate a dma descriptor\n");
		ret = -ENOMEM;
		goto error_ret;
	}

	tx_cookie = dmaengine_submit(tx_desc);
	if (tx_cookie < 0) {
		dev_err(st->dev,
			"Failed to submit a dma transfer\n");
		ret = tx_cookie;
		goto error_ret;
	}

	rx_desc = dmaengine_prep_slave_single(st->rx_chan, dest, size * 4,
					   DMA_FROM_DEVICE, DMA_PREP_INTERRUPT);
	if (!rx_desc) {
		dev_err(st->dev,
			"Failed to allocate a dma descriptor\n");
		ret = -ENOMEM;
		goto error_ret;
	}

	rx_desc->callback = (dma_async_tx_callback) complete;
	rx_desc->callback_param = &st->dma_complete;

	rx_cookie = dmaengine_submit(rx_desc);
	if (rx_cookie < 0) {
		dev_err(st->dev,
			"Failed to submit a dma transfer\n");
		ret = rx_cookie;
		goto error_ret;
	}

	dma_async_issue_pending(st->tx_chan);
	dma_async_issue_pending(st->rx_chan);

	ret = wait_for_completion_interruptible_timeout(&st->dma_complete,
							4 * HZ);
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto error_ret;
	} else if (ret < 0) {
		goto error_ret;
	}

	ret = 0;

error_ret:
	mutex_unlock(&st->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(fft_calculate);

/**
 * fft_of_probe - probe method for the AIM device.
 * @of_dev:	pointer to OF device structure
 * @match:	pointer to the structure used for matching a device
 *
 * This function probes the AIM device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the AIM device, or a negative
 * value if there is an error.
 */
static int fft_of_probe(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct fft_state *st;
	struct resource r_mem; /* IO mem resources */
	struct fft_dma_params dma_params;
	struct of_phandle_args dma_spec;
	dma_cap_mask_t mask;
	resource_size_t remap_size, phys_addr;
	int ret;

	dev_info(dev, "Device Tree Probing \'%s\'\n",
		 op->dev.of_node->name);

	st = devm_kzalloc(&op->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	st->dev = dev;
	dev_set_drvdata(dev, st);
	mutex_init(&st->lock);

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
	if (ret) {
		dev_err(dev, "invalid address\n");
		return ret;
	}

	phys_addr = r_mem.start;
	remap_size = resource_size(&r_mem);
	if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto failed1;
	}

	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto failed2;
	}

	/* Get dma channel for the device */
	ret = of_parse_phandle_with_args(op->dev.of_node, "dma-request",
					 "#dma-cells", 0, &dma_spec);
	if (ret) {
		dev_err(dev, "Couldn't parse dma-request\n");
		goto failed2;
	}

	dma_params.of_node = dma_spec.np;
	dma_params.chan_id = dma_spec.args[0];

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

	st->tx_chan = dma_request_channel(mask, fft_dma_filter, &dma_params);
	if (!st->tx_chan) {
		dev_err(dev, "failed to find tx dma device\n");
		goto failed2;
	}

	/* Get dma channel for the device */
	ret = of_parse_phandle_with_args(op->dev.of_node, "dma-request",
					 "#dma-cells", 1, &dma_spec);
	if (ret) {
		dev_err(dev, "Couldn't parse dma-request\n");
		goto failed2;
	}

	dma_params.of_node = dma_spec.np;
	dma_params.chan_id = dma_spec.args[0];

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

	st->rx_chan = dma_request_channel(mask, fft_dma_filter, &dma_params);
	if (!st->rx_chan) {
		dev_err(dev, "failed to find rx dma device\n");
		goto failed2;
	}

	init_completion(&st->dma_complete);

	dev_info(dev, "ADI-FFT (0x%X) at 0x%08llX mapped to 0x%p,"
		 " DMA-%d, DMA-%d probed\n",
		 fft_read(st, FFT_PCORE_VERSION),
		 (unsigned long long)phys_addr, st->regs,
		 st->rx_chan->chan_id,  st->tx_chan->chan_id);

	fft_state_glob = st;

	return 0;

failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	dev_set_drvdata(dev, NULL);

	return ret;
}

/**
 * fft_of_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int fft_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem; /* IO mem resources */
	struct fft_state *st = dev_get_drvdata(dev);

	dma_release_channel(st->rx_chan);
	dma_release_channel(st->tx_chan);

	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	dev_set_drvdata(dev, NULL);
	fft_state_glob = NULL;

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id fft_of_match[] = {
	{ .compatible = "xlnx,cf-fft-core-1.00.a", },
	{ .compatible = "xlnx,axi-fft-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, fft_of_match);

static struct platform_driver fft_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = fft_of_match,
	},
	.probe		= fft_of_probe,
	.remove		= fft_of_remove,
};

module_platform_driver(fft_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices CF FFT");
MODULE_LICENSE("GPL v2");
