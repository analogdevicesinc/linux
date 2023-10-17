// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2023 NXP
 */

#include <linux/regmap.h>

#include <media/mipi-csi2.h>

#include "imx8-isi-core.h"

/* -----------------------------------------------------------------------------
 * i.MX8MN and i.MX8MP gasket
 */

#define GASKET_BASE(n)				(0x0060 + (n) * 0x30)

#define GASKET_CTRL				0x0000
#define GASKET_CTRL_DATA_TYPE(dt)		((dt) << 8)
#define GASKET_CTRL_DATA_TYPE_MASK		(0x3f << 8)
#define GASKET_CTRL_DUAL_COMP_ENABLE		BIT(1)
#define GASKET_CTRL_ENABLE			BIT(0)

#define GASKET_HSIZE				0x0004
#define GASKET_VSIZE				0x0008

static void mxc_imx8_gasket_enable(struct mxc_isi_dev *isi,
				   const struct v4l2_mbus_frame_desc *fd,
				   const struct v4l2_mbus_framefmt *fmt,
				   const unsigned int port)
{
	u32 val;

	regmap_write(isi->gasket, GASKET_BASE(port) + GASKET_HSIZE, fmt->width);
	regmap_write(isi->gasket, GASKET_BASE(port) + GASKET_VSIZE, fmt->height);

	val = GASKET_CTRL_DATA_TYPE(fd->entry[0].bus.csi2.dt);
	if (fd->entry[0].bus.csi2.dt == MIPI_CSI2_DT_YUV422_8B)
		val |= GASKET_CTRL_DUAL_COMP_ENABLE;

	val |= GASKET_CTRL_ENABLE;
	regmap_write(isi->gasket, GASKET_BASE(port) + GASKET_CTRL, val);
}

static void mxc_imx8_gasket_disable(struct mxc_isi_dev *isi,
				    const unsigned int port)
{
	regmap_write(isi->gasket, GASKET_BASE(port) + GASKET_CTRL, 0);
}

const struct mxc_gasket_ops mxc_imx8_gasket_ops = {
	.enable = mxc_imx8_gasket_enable,
	.disable = mxc_imx8_gasket_disable,
};

/* -----------------------------------------------------------------------------
 * i.MX93 gasket
 */

#define DISP_MIX_CAMERA_MUX                     0x30
#define DISP_MIX_CAMERA_MUX_DATA_TYPE(x)        (((x) & 0x3f) << 3)
#define DISP_MIX_CAMERA_MUX_GASKET_ENABLE       BIT(16)

static void mxc_imx93_gasket_enable(struct mxc_isi_dev *isi,
				    const struct v4l2_mbus_frame_desc *fd,
				    const struct v4l2_mbus_framefmt *fmt,
				    const unsigned int port)
{
	u32 val;

	val = DISP_MIX_CAMERA_MUX_DATA_TYPE(fd->entry[0].bus.csi2.dt);
	val |= DISP_MIX_CAMERA_MUX_GASKET_ENABLE;
	regmap_write(isi->gasket, DISP_MIX_CAMERA_MUX, val);
}

static void mxc_imx93_gasket_disable(struct mxc_isi_dev *isi,
				     unsigned int port)
{
	regmap_write(isi->gasket, DISP_MIX_CAMERA_MUX, 0);
}

const struct mxc_gasket_ops mxc_imx93_gasket_ops = {
	.enable = mxc_imx93_gasket_enable,
	.disable = mxc_imx93_gasket_disable,
};

/* -----------------------------------------------------------------------------
 * i.MX95 gasket
 */

/*
 * MIPI CSI0 connected to port 2 and MIPI CSI1 connected to port 3 of
 * pixel link crossbar
 */
#define CSI_PIXEL_FORMATER_BASE(n)			(0x0020 + (n - 2) * 0x100)

#define CSI_INTERLACED_LINE_CNT_VC_SET(x)		(0x0000 + (x) * 0x04)
#define CSI_INTERLACED_LINE_CNT_VC_ODD(cnt)		(cnt << 0)
#define CSI_INTERLACED_LINE_CNT_VC_ODD_MASK		GENMASK(13, 0)
#define CSI_INTERLACED_LINE_CNT_VC_EVEN(cnt)		(cnt << 16)
#define CSI_INTERLACED_LINE_CNT_VC_EVEN_MASK		GENMASK(29, 16)

#define CSI_VC_INTERLACED_CTRL				0x20
#define CSI_VC_INTERLACED_CTRL_MODE_VC(x, mode)		((mode) << (x * 0x2))
#define CSI_VC_INTERLACED_CTRL_MODE_MASK_VC(x)		GENMASK((1 + (x * 0x2)), (x * 0x2))

#define CSI_VC_INTERLACED_ERR				0x24
#define CSI_VC_INTERLACED_ERR_VC(x)			BIT(x)

#define CSI_YUV420_FIRST_LINE_EVEN			0x28
#define CSI_YUV420_FIRST_LINE_EVEN_VC(x)		BIT(x)

#define CSI_RAW32_CTRL					0x30
#define CSI_RAW32_CTRL_RAW32_MODE_VC(x)			BIT(x)
#define CSI_RAW32_CTRL_RAW_SWAP_MODE(x)			BIT(((x) + 0x8))
#define CSI_RAW32_CTRL_RAW_SWAP_MODE_MASK(x)		GENMASK(15, 8)

#define CSI_STREAM_FENCING_CTRL				0x34
#define CSI_STREAM_FENCING_CTRL_VC(x)			BIT(x)
#define CSI_STREAM_FENCING_CTRL_MASK			GENMASK(7, 0)
#define CSI_STREAM_FENCING_CTRL_RESET_SM_VC(x)		BIT(((x) + 0x8))
#define CSI_STREAM_FENCING_CTRL_RESET_SM_MASK		GENMASK(15, 8)

#define CSI_STREAM_FENCING_STATUS			0x38
#define CSI_STREAM_FENCING_STATUS_VC(x)			BIT(x)

#define CSI_NP_DATA_TYPE_VC(x)				(0x0040 + (x) * 0x04)
#define CSI_NP_DATA_TYPE_VC_EN(dt)			BIT(dt)
#define CSI_NP_DATA_TYPE_VC_EN_MASK			GENMASK(21, 0)

#define CSI_PIXEL_DATA_CTRL_VC(x)			(0x0060 + (x) * 0x04)
#define CSI_PIXEL_DATA_CTRL_REROUTE			BIT(0)
#define CSI_PIXEL_DATA_CTRL_NEW_VC(x)			(x << 1)
#define CSI_PIXEL_DATA_CTRL_NEW_VC_MASK			GENMASK(3, 1)

#define CSI_ROUTE_PIXEL_DATA_TYPE_VC(x)			(0x0080 + (x) * 0x04)
#define CSI_ROUTE_PIXEL_DATA_TYPE_MASK			GENMASK(23, 0)

#define CSI_NP_DATA_CTRL_VC(x)				(0x00a0 + (x) * 0x04)
#define CSI_NP_DATA_CTRL_REROUTE			BIT(0)
#define CSI_NP_DATA_CTRL_NEW_VC(x)			(x << 1)
#define CSI_NP_DATA_CTRL_NEW_VC_MASK			GENMASK(3, 1)

#define CSI_PIXEL_DATA_TYPE_VC(x)			(0x00c0 + (x) * 0x04)
#define CSI_PIXEL_DATA_TYPE_MASK			GENMASK(23, 0)
#define CSI_PIXEL_DATA_TYPE_INDEX(x)			BIT(x)

#define CSI_PIXEL_DATA_TYPE_ERR_VC(x)			(0x00e0 + (x) * 0x04)
#define CSI_PIXEL_DATA_TYPE_ERR_INDEX(x)		BIT(x)

struct dt_index {
	u8 dtype;
	u8 index;
};

static const struct dt_index imx95_dt_to_index_map[] = {
	{ .dtype = MIPI_CSI2_DT_YUV420_8B,        .index = 0 },
	{ .dtype = MIPI_CSI2_DT_YUV420_8B_LEGACY, .index = 2 },
	{ .dtype = MIPI_CSI2_DT_YUV422_8B,        .index = 6 },
	{ .dtype = MIPI_CSI2_DT_RGB444,		  .index = 8 },
	{ .dtype = MIPI_CSI2_DT_RGB555,           .index = 9 },
	{ .dtype = MIPI_CSI2_DT_RGB565,           .index = 10 },
	{ .dtype = MIPI_CSI2_DT_RGB666,           .index = 11 },
	{ .dtype = MIPI_CSI2_DT_RGB888,           .index = 12 },
	{ .dtype = MIPI_CSI2_DT_RAW6,             .index = 16 },
	{ .dtype = MIPI_CSI2_DT_RAW7,             .index = 17 },
	{ .dtype = MIPI_CSI2_DT_RAW8,             .index = 18 },
	{ .dtype = MIPI_CSI2_DT_RAW10,            .index = 19 },
	{ .dtype = MIPI_CSI2_DT_RAW12,            .index = 20 },
	{ .dtype = MIPI_CSI2_DT_RAW14,            .index = 21 },
};

static u8 get_index_by_dt(u8 data_type)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx95_dt_to_index_map); ++i)
		if (data_type == imx95_dt_to_index_map[i].dtype)
			break;

	if (i == ARRAY_SIZE(imx95_dt_to_index_map)) {
		pr_err("Don't find invalid data type index\n");
		return 0;
	}

	return imx95_dt_to_index_map[i].index;
}

static void mxc_imx95_gasket_enable(struct mxc_isi_dev *isi,
				    const struct v4l2_mbus_frame_desc *fd,
				    const struct v4l2_mbus_framefmt *fmt,
				    const unsigned int port)
{
	const struct v4l2_mbus_frame_desc_entry_csi2 *csi2 =
						&fd->entry[0].bus.csi2;
	u32 reg;

	/* Enable data type for pixel data on the vc */
	reg = CSI_PIXEL_FORMATER_BASE(port) + CSI_PIXEL_DATA_TYPE_VC(csi2->vc);
	regmap_write(isi->gasket, reg, BIT(get_index_by_dt(csi2->dt)));
}

static void mxc_imx95_gasket_disable(struct mxc_isi_dev *isi,
				     unsigned int port)
{
}

const struct mxc_gasket_ops mxc_imx95_gasket_ops = {
	.enable = mxc_imx95_gasket_enable,
	.disable = mxc_imx95_gasket_disable,
};
