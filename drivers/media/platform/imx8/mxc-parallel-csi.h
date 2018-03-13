
/*
 * Copyright 2018 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef MXC_PARALLEL_CSI_H_
#define MXC_PARALLEL_CSI_H_

#include <media/v4l2-device.h>

#define MXC_PARALLEL_CSI_DRIVER_NAME	"mxc-parallel-csi"
#define MXC_PARALLEL_CSI_SUBDEV_NAME	MXC_PARALLEL_CSI_DRIVER_NAME

#define PARALLEL_CSI_OF_NODE_NAME	"pcsi"
#define BIT_U(nr) (1U << (nr))

#define MXC_PARALLEL_CSI_PAD_SOURCE		0
#define MXC_PARALLEL_CSI_PAD_SINK		1
#define MXC_PARALLEL_CSI_PADS_NUM		2

#define CI_PI_BASE_OFFSET       0x0U

/* CI_PI INTERFACE Control */
#define IF_CTRL_REG                     (CI_PI_BASE_OFFSET + 0x00)
#define IF_CTRL_REG_PL_ENABLE           BIT_U(0)
#define IF_CTRL_REG_PL_VALID            BIT_U(1)
#define IF_CTRL_REG_PL_ADDR(x)          (((x) & 0x7U) << 2)
#define IF_CTRL_REG_IF_FORCE(x)         (((x) & 0x7U) << 5)
#define IF_CTRL_REG_DATA_TYPE_SEL       BIT_U(8)
#define IF_CTRL_REG_DATA_TYPE(x)        (((x) & 0x1FU) << 9)

#define DATA_TYPE_OUT_NULL           (0x00)
#define DATA_TYPE_OUT_RGB            (0x04)
#define DATA_TYPE_OUT_YUV444         (0x08)
#define DATA_TYPE_OUT_YYU420_ODD     (0x10)
#define DATA_TYPE_OUT_YYU420_EVEN    (0x12)
#define DATA_TYPE_OUT_YYY_ODD        (0x18)
#define DATA_TYPE_OUT_UYVY_EVEN      (0x1A)
#define DATA_TYPE_OUT_RAW            (0x1C)

#define IF_CTRL_REG_IF_FORCE_HSYNV_OVERRIDE         0x4
#define IF_CTRL_REG_IF_FORCE_VSYNV_OVERRIDE         0x2
#define IF_CTRL_REG_IF_FORCE_DATA_ENABLE_OVERRIDE   0x1

#define IF_CTRL_REG_SET                 (CI_PI_BASE_OFFSET + 0x04)
#define IF_CTRL_REG_CLR                 (CI_PI_BASE_OFFSET + 0x08)
#define IF_CTRL_REG_TOG                 (CI_PI_BASE_OFFSET + 0x0C)

/* CSI INTERFACE CONTROL REG */
#define CSI_CTRL_REG                    (CI_PI_BASE_OFFSET + 0x10)
#define CSI_CTRL_REG_CSI_EN                     BIT_U(0)
#define CSI_CTRL_REG_PIXEL_CLK_POL              BIT_U(1)
#define CSI_CTRL_REG_HSYNC_POL                  BIT_U(2)
#define CSI_CTRL_REG_VSYNC_POL                  BIT_U(3)
#define CSI_CTRL_REG_DE_POL                     BIT_U(4)
#define CSI_CTRL_REG_PIXEL_DATA_POL             BIT_U(5)
#define CSI_CTRL_REG_CCIR_EXT_VSYNC_EN          BIT_U(6)
#define CSI_CTRL_REG_CCIR_EN                    BIT_U(7)
#define CSI_CTRL_REG_CCIR_VIDEO_MODE            BIT_U(8)
#define CSI_CTRL_REG_CCIR_NTSC_EN               BIT_U(9)
#define CSI_CTRL_REG_CCIR_VSYNC_RESET_EN        BIT_U(10)
#define CSI_CTRL_REG_CCIR_ECC_ERR_CORRECT_EN    BIT_U(11)
#define CSI_CTRL_REG_HSYNC_FORCE_EN             BIT_U(12)
#define CSI_CTRL_REG_VSYNC_FORCE_EN             BIT_U(13)
#define CSI_CTRL_REG_GCLK_MODE_EN               BIT_U(14)
#define CSI_CTRL_REG_VALID_SEL                  BIT_U(15)
#define CSI_CTRL_REG_RAW_OUT_SEL                BIT_U(16)
#define CSI_CTRL_REG_HSYNC_OUT_SEL              BIT_U(17)
#define CSI_CTRL_REG_HSYNC_PULSE(x)             (((x) & 0x7U) << 19)
#define CSI_CTRL_REG_UV_SWAP_EN                 BIT_U(22)
#define CSI_CTRL_REG_DATA_TYPE_IN(x)            (((x) & 0xFU) << 23)
#define CSI_CTRL_REG_MASK_VSYNC_COUNTER(x)      (((x) & 0x3U) << 27)
#define CSI_CTRL_REG_SOFTRST                    BIT_U(31)

#define DATA_TYPE_IN_UYVY_BT656_8BITS     0x0
#define DATA_TYPE_IN_UYVY_BT656_10BITS    0x1
#define DATA_TYPE_IN_RGB_8BITS            0x2
#define DATA_TYPE_IN_BGR_8BITS            0x3
#define DATA_TYPE_IN_RGB_24BITS           0x4
#define DATA_TYPE_IN_YVYU_8BITS           0x5
#define DATA_TYPE_IN_YUV_8BITS            0x6
#define DATA_TYPE_IN_YVYU_16BITS          0x7
#define DATA_TYPE_IN_YUV_24BITS           0x8
#define DATA_TYPE_IN_BAYER_8BITS          0x9
#define DATA_TYPE_IN_BAYER_10BITS         0xA
#define DATA_TYPE_IN_BAYER_12BITS         0xB
#define DATA_TYPE_IN_BAYER_16BITS         0xC

#define CSI_CTRL_REG_SET                (CI_PI_BASE_OFFSET + 0x14)
#define CSI_CTRL_REG_CLR                (CI_PI_BASE_OFFSET + 0x18)
#define CSI_CTRL_REG_TOG                (CI_PI_BASE_OFFSET + 0x1C)

/* CSI interface Status */
#define CSI_STATUS                      (CI_PI_BASE_OFFSET + 0x20)
#define CSI_STATUS_FIELD_TOGGLE         BIT_U(0)
#define CSI_STATUS_ECC_ERROR            BIT_U(1)

#define CSI_STATUS_SET                  (CI_PI_BASE_OFFSET + 0x24)
#define CSI_STATUS_CLR                  (CI_PI_BASE_OFFSET + 0x28)
#define CSI_STATUS_TOG                  (CI_PI_BASE_OFFSET + 0x2C)

/* CSI INTERFACE CONTROL REG1 */
#define CSI_CTRL_REG1                   (CI_PI_BASE_OFFSET + 0x30)
#define CSI_CTRL_REG1_PIXEL_WIDTH(v)    (((v) & 0xFFFFU) << 0)
#define CSI_CTRL_REG1_VSYNC_PULSE(v)    (((v) & 0xFFFFU) << 16)

#define CSI_CTRL_REG1_SET               (CI_PI_BASE_OFFSET + 0x34)
#define CSI_CTRL_REG1_CLR               (CI_PI_BASE_OFFSET + 0x38)
#define CSI_CTRL_REG1_TOG               (CI_PI_BASE_OFFSET + 0x3C)

/***********************************************************/
#define LPCG_BASE		(void __iomem *)0x58263000
#define LPCG_LIS_CLK			(0x00)
#define LPCG_CSR_CLK			(0x04)
#define LPCG_GPIO_CLK			(0x08)
#define LPCG_PWM_CLK			(0x0c)
#define LPCG_I2C_CLK			(0x10)
#define LPCG_CSI_INTERFACE_CLK	(0x18)
#define LPCG_CSI_MCLK			(0x1c)
/***********************************************************/

//#define CSI_INTERFACE_CCIR656	1

struct mxc_parallel_csi_dev {
	struct v4l2_device		v4l2_dev;
	struct v4l2_subdev		sd;
	struct v4l2_subdev		*sensor_sd;

	struct media_pad pads[MXC_PARALLEL_CSI_PADS_NUM];
	struct v4l2_mbus_framefmt format;

	void __iomem *csr_regs;
	void __iomem *lpcg_regs;
	struct platform_device *pdev;
	u32 flags;
	int irq;

	struct clk *clk_ipg;
	struct clk *clk_pixel;
	struct clk *clk_sel;
	struct clk *clk_div;
	struct clk *clk_dpll;

	bool clk_enable;

	struct v4l2_async_subdev	asd;
	struct v4l2_async_notifier	subdev_notifier;
	struct v4l2_async_subdev	*async_subdevs[2];

	struct mutex lock;

	u8 running;
	u8 mode;
	u8 uv_swap;
	u8 tvdec;
};
#endif

enum {
	GATE_CLOCK_MODE = 0x01,
	CCIR_MODE = 0x02,
};
