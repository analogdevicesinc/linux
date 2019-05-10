// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Analog Devices Inc.
 * ADI Frame Buffer Driver
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <media/media-device.h>
#include <linux/fpga/adi-axi-common.h>

/* DMA defines */
#define DMAC_REG_CTRL				0x400
#define DMAC_REG_START_TRANSFER			0x408
#define DMAC_REG_FLAGS				0x40c
#define DMAC_REG_DEST_ADDRESS			0x410
#define DMAC_REG_SRC_ADDRESS			0x414
#define DMAC_REG_X_LENGTH			0x418
#define DMAC_REG_Y_LENGTH			0x41c
#define DMAC_REG_DEST_STRIDE			0x420
#define DMAC_REG_SRC_STRIDE			0x424
#define DMAC_REG_FRAME_LOCK_CONFIG		0x454
#define DMAC_REG_FRAME_LOCK_STRIDE		0x458

#define DMAC_CTRL_ENABLE			BIT(0)
#define DMAC_TRANSFER_SUBMIT			BIT(0)
#define DMAC_FLOCK_WAIT_WRITER			BIT(9)

#define DMAC_FLAGS_CYCLIC			BIT(0)
#define DMAC_FLAGS_TLAST			BIT(1)
#define DMAC_FLAGS_FLOCK			BIT(3)

#define DMAC_DEFAULT_FLAGS (DMAC_FLAGS_FLOCK | DMAC_FLAGS_CYCLIC | \
			    DMAC_FLAGS_TLAST)

struct hdl_subdev {
	void __iomem *tx_dma_regs;
	void __iomem *rx_dma_regs;
};

struct frame_buffer {
	struct media_device media_dev;

	struct resource video_ram_buf;
	struct hdl_subdev hdl_subdev;
	u32 num_frames;
	u32 mode;
	u32 distance;
	u32 line_stride;
	u32 frame_stride;
	/* resolution[0] width, resolution[1] height */
	u32 resolution[2];
	u32 dwidth;
};

enum {
	TX_DMA,
	RX_DMA
};

static u32 adi_fb_reg_read(void __iomem *base_addr, u32 addr)
{
	return ioread32(base_addr + addr);
}

static void adi_fb_reg_write(void __iomem *base_addr, u32 addr, u32 value)
{
	iowrite32(value, base_addr + addr);
}

static void adi_fb_reg_clr(void __iomem *base_addr, u32 addr, u32 clr)
{
	adi_fb_reg_write(base_addr, addr,
			 adi_fb_reg_read(base_addr, addr) & ~clr);
}

static void adi_fb_reg_set(void __iomem *base_addr, u32 addr, u32 set)
{
	adi_fb_reg_write(base_addr, addr,
			 adi_fb_reg_read(base_addr, addr) | set);
}

static void adi_fb_init(struct frame_buffer *buff, int dma_dir)
{
	void __iomem *base_addr;
	u32 stride, dir, flock_cfg;

	if (dma_dir == RX_DMA) {
		base_addr = buff->hdl_subdev.rx_dma_regs;
		dir = DMAC_REG_DEST_ADDRESS;
		stride = DMAC_REG_DEST_STRIDE;
		flock_cfg = ((buff->distance) << 16) | ((buff->mode) << 8) |
			    (buff->num_frames);
	} else {
		base_addr = buff->hdl_subdev.tx_dma_regs;
		dir = DMAC_REG_SRC_ADDRESS;
		stride = DMAC_REG_SRC_STRIDE;
		flock_cfg = ((buff->distance) << 16) | ((buff->mode) << 8) |
			    DMAC_FLOCK_WAIT_WRITER | (buff->num_frames);
	}

	/* reset DMAC */
	adi_fb_reg_clr(base_addr, DMAC_REG_CTRL, DMAC_CTRL_ENABLE);

	/* Init DMAC */
	adi_fb_reg_set(base_addr, DMAC_REG_CTRL, DMAC_CTRL_ENABLE);
	adi_fb_reg_write(base_addr, DMAC_REG_FLAGS, DMAC_DEFAULT_FLAGS);

	adi_fb_reg_write(base_addr, dir, buff->video_ram_buf.start);
	/* h size */
	adi_fb_reg_write(base_addr, DMAC_REG_X_LENGTH,
			 ((buff->resolution[0] * buff->dwidth) - 1));
	/* h offset */
	adi_fb_reg_write(base_addr, stride, buff->line_stride);
	/* v size */
	adi_fb_reg_write(base_addr, DMAC_REG_Y_LENGTH,
			 (buff->resolution[1] - 1));

	adi_fb_reg_write(base_addr, DMAC_REG_FRAME_LOCK_CONFIG, flock_cfg);
	/* total active */
	adi_fb_reg_write(base_addr, DMAC_REG_FRAME_LOCK_STRIDE,
			buff->frame_stride);
	/* submit transfer */
	adi_fb_reg_set(base_addr, DMAC_REG_START_TRANSFER,
		       DMAC_TRANSFER_SUBMIT);
}

static void frame_buffer_media_unregister(void *data)
{
	struct frame_buffer *frm_buff = data;

	media_device_unregister(&frm_buff->media_dev);
}

static int frame_buffer_probe(struct platform_device *pdev)
{
	struct frame_buffer *frm_buff;
	struct device_node *np;
	struct resource *res;
	int ret;
	u32 tmp;

	frm_buff = devm_kzalloc(&pdev->dev, sizeof(struct frame_buffer),
				GFP_KERNEL);
	if (!frm_buff)
		return -ENOMEM;

	platform_set_drvdata(pdev, frm_buff);

	/* Get reserved memory region from Device-tree */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		/* Get physical address of FB from reg property */
		res = platform_get_resource_byname(pdev,
						IORESOURCE_MEM, "fb_mem");
		if (!res) {
			dev_err(&pdev->dev, "No frame buffer memory.\n");
			return -EFAULT;
		}
		frm_buff->video_ram_buf = *res;
	} else {
		ret = of_address_to_resource(np, 0, &frm_buff->video_ram_buf);
		if (ret) {
			dev_err(&pdev->dev,
				"No memory address assigned to the region\n");
			return ret;
		}
	}
	dev_info(&pdev->dev, "Allocated reserved memory, paddr: 0x%0X\n",
		 (unsigned int)frm_buff->video_ram_buf.start);

	/* Get physical address of DMAs*/
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tx_dma");
	frm_buff->hdl_subdev.tx_dma_regs =
		devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(frm_buff->hdl_subdev.tx_dma_regs))
		return PTR_ERR(frm_buff->hdl_subdev.tx_dma_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rx_dma");
	frm_buff->hdl_subdev.rx_dma_regs =
		devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(frm_buff->hdl_subdev.rx_dma_regs))
		return PTR_ERR(frm_buff->hdl_subdev.rx_dma_regs);

	/* Get frames number */
	ret = device_property_read_u32(&pdev->dev, "adi,flock-frm-buf-nr",
					&tmp);
	if (ret) {
		frm_buff->num_frames = 3;
		dev_info(&pdev->dev, "No frames number specified. Using %d\n",
			 frm_buff->num_frames);
	} else {
		frm_buff->num_frames = tmp;
	}

	/* Get frame distance */
	ret = device_property_read_u32(&pdev->dev, "adi,flock-distance", &tmp);
	if (ret) {
		frm_buff->distance = 0;
		dev_info(&pdev->dev, "No distance specified. Using %d\n",
			 frm_buff->distance);
	} else {
		frm_buff->distance = tmp;
	}

	/* Get operating mode */
	ret = device_property_read_u32(&pdev->dev, "adi,flock-mode", &tmp);
	if (ret) {
		frm_buff->mode = 0;
		dev_info(&pdev->dev,
			 "No operating mode specified. Using framelock\n");
	} else {
		frm_buff->mode = tmp;
	}

	/* Get data width */
	ret = device_property_read_u32(&pdev->dev, "adi,flock-dwidth", &tmp);
	if (ret) {
		frm_buff->dwidth = 4;
		dev_info(&pdev->dev, "No data width specified. Using %d byte\n",
			 frm_buff->dwidth);
	} else {
		frm_buff->dwidth = tmp;
	}

	/* Get resolution mode */
	ret = device_property_read_u32_array(&pdev->dev, "adi,flock-resolution",
					     frm_buff->resolution, 2);
	if (ret) {
		frm_buff->resolution[0] = 1920;
		frm_buff->resolution[1] = 1080;
		dev_info(&pdev->dev,
			 "No resolution specified. Using default %d x %d\n",
			 frm_buff->resolution[0], frm_buff->resolution[1]);
	}

	/* Get optional line stride*/
	ret = device_property_read_u32(&pdev->dev, "adi,flock-line-stride",
					&tmp);
	frm_buff->line_stride = frm_buff->resolution[0] * frm_buff->dwidth;
	if (ret) {
		dev_info(&pdev->dev, "No line stride specified. Using %d bytes\n",
			 frm_buff->line_stride);
	} else {
		if (tmp > frm_buff->line_stride)
			frm_buff->line_stride = tmp;
	}

	/* Get optional frame stride*/
	ret = device_property_read_u32(&pdev->dev, "adi,flock-frm-stride",
					&tmp);
	frm_buff->frame_stride =
		(frm_buff->line_stride * frm_buff->resolution[1]);
	if (ret) {
		dev_info(&pdev->dev, "No frame stride specified. Using %d bytes\n",
			 frm_buff->frame_stride);
	} else {
		if (tmp > frm_buff->frame_stride)
			frm_buff->frame_stride = tmp;
	}

	if ((frm_buff->video_ram_buf.end - frm_buff->video_ram_buf.start) <
			frm_buff->frame_stride * frm_buff->num_frames) {
		dev_err(&pdev->dev, "FB does not fit in reserved memory\n");
		return -ENOMEM;
	}

	adi_fb_init(frm_buff, TX_DMA);
	adi_fb_init(frm_buff, RX_DMA);

	frm_buff->media_dev.dev = &pdev->dev;

	strlcpy(frm_buff->media_dev.model, "ADI AXI Frame Buffer",
		sizeof(frm_buff->media_dev.model));

	frm_buff->media_dev.hw_revision = 0;

	media_device_init(&frm_buff->media_dev);

	ret = media_device_register(&frm_buff->media_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register media_device\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&pdev->dev,
		frame_buffer_media_unregister, frm_buff);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id frame_buffer_of_match[] = {
	{ .compatible = "adi,axi-framebuffer-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, frame_buffer_of_match);

static struct platform_driver frame_buffer_driver = {
	.driver			= {
		.name		= "axi-framebuffer",
		.of_match_table = frame_buffer_of_match,
	},
	.probe			= frame_buffer_probe,
};
module_platform_driver(frame_buffer_driver);

MODULE_AUTHOR("Bogdan Togoean <bogdan.togorean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI Frame Buffer");
MODULE_LICENSE("GPL");
