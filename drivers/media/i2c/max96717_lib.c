// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX96717 GMSL2/1 Deserializer Driver
 *
 * Copyright 2024 NXP
 *
 */
#include <linux/i2c.h>
#include <linux/regmap.h>

#include "max96717_regs.h"
#include "max96717.h"

#define MAX96717_DEV_ID			0xB7
#define MAX96717F_DEV_ID		0xC8

static bool max96717_writable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX96717_DEV_REG13 ... MAX96717_DEV_REG14:
	case MAX96717_DEV_REG26:
	case MAX96717_TCTRL_PWR0:
	case MAX96717_TCTRL_CTRL3:
	case MAX96717_TCTRL_INTR3:
	case MAX96717_TCTRL_INTR5:
	case MAX96717_TCTRL_INTR7:
	case MAX96717_GMSL_TX3:
	case MAX96717_SPI_7:
	case MAX96717_MIPI_RX_EXT_8:
	case MAX96717_MIPI_RX_EXT_PHY1_PKT_CNT ... MAX96717_MIPI_RX_EXT_PHY_CLK_CNT:
	case MAX96717_AFE_ADC_DATA_L ... MAX96717_AFE_ADC_DATA1:
	case MAX96717_AFE_ADC_INTR0 ... MAX96717_AFE_ADC_INTR3:
	case MAX96717_MISC_HS_VS_Z:
	case MAX96717_MIPI_RX_EXT3_CTRL1_FS_CNT_L ... MAX96717_MIPI_RX_EXT3_CTRL1_FE_CNT_H:
	case MAX96717_RLMS_7:
	case MAX96717_RLMS_EYEMONVALCNTL ... MAX96717_RLMS_EYEMONVALCNTH:
	case MAX96717_RLMS_AA:
	case MAX96717_EFUSE_SERIAL_NUMBER_0 ... MAX96717_EFUSE_SERIAL_NUMBER_23:
	case MAX96717_FUNC_SAFE_REGCRC_LSB ... MAX96717_FUNC_SAFE_REGCRC_MSB:
	case MAX96717_FUNC_SAFE_I2C_UART_CRC2 ... MAX96717_FUNC_SAFE_I2C_UART_CRC4:
	case MAX96717_FUNC_SAFE_FS_INTR1:
	case MAX96717_FUNC_SAFE_REG_POST0:
	case MAX96717_FUNC_SAFE_T_EST_OUT_B0 ... MAX96717_FUNC_SAFE_ALT_T_EST_OUT_B0:
		return false;

	default:
		return true;
	}
}

static bool max96717_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX96717_TCTRL_PWR0:
	case MAX96717_TCTRL_CTRL0:
	case MAX96717_TCTRL_CTRL3:
	case MAX96717_TCTRL_INTR3:
	case MAX96717_TCTRL_INTR5:
	case MAX96717_TCTRL_INTR7:
	case MAX96717_TCTRL_DEC_ERR_A ... MAX96717_TCTRL_PKT_CNT:
	case MAX96717_GMSL_TX2 ... MAX96717_GMSL_TX3:
	case MAX96717_CFGL_SPI_ARQ2:
	case MAX96717_CFGL_GPIO_ARQ2:
	case MAX96717_CFGL_IIC_ARQ2(0):
	case MAX96717_CFGL_IIC_ARQ2(1):
	case MAX96717_VID_TX_Z_VIDEO_TX2:
	case MAX96717_SPI_7:
	case MAX96717_VTX_Z_VTX1:
	case MAX96717_VTX_Z_VTX29:
	case MAX96717_GPIO_A(0):
	case MAX96717_GPIO_A(1):
	case MAX96717_GPIO_A(2):
	case MAX96717_GPIO_A(3):
	case MAX96717_GPIO_A(4):
	case MAX96717_GPIO_A(5):
	case MAX96717_GPIO_A(6):
	case MAX96717_GPIO_A(7):
	case MAX96717_GPIO_A(8):
	case MAX96717_GPIO_A(9):
	case MAX96717_GPIO_A(10):
	case MAX96717_MIPI_RX_PHY1_HS_ERR ... MAX96717_MIPI_RX_PHY2_HS_ERR:
	case MAX96717_MIPI_RX_20:
	case MAX96717_MIPI_RX_EXT_8:
	case MAX96717_MIPI_RX_EXT_PHY1_PKT_CNT ... MAX96717_MIPI_RX_EXT_PHY_CLK_CNT:
	case MAX96717_REF_VTG_0:
	case MAX96717_AFE_ADC_DATA_L ... MAX96717_AFE_ADC_DATA1:
	case MAX96717_AFE_ADC_INTR0 ... MAX96717_AFE_ADC_INTR3:
	case MAX96717_MISC_HS_VS_Z:
	case MAX96717_MIPI_RX_EXT3_CTRL1_FS_CNT_L ... MAX96717_MIPI_RX_EXT3_CTRL1_FE_CNT_H:
	case MAX96717_RLMS_7:
	case MAX96717_RLMS_EYEMONVALCNTL ... MAX96717_RLMS_EYEMONVALCNTH:
	case MAX96717_RLMS_AA:
	case MAX96717_FUNC_SAFE_REGCRC_LSB ... MAX96717_FUNC_SAFE_REGCRC_MSB:
	case MAX96717_FUNC_SAFE_FS_INTR1:
	case MAX96717_FUNC_SAFE_REG_POST0:
	case MAX96717_FUNC_SAFE_T_EST_OUT_B0 ... MAX96717_FUNC_SAFE_ALT_T_EST_OUT_B0:
		return true;

	default:
		return false;
	}
}

static const struct regmap_config max96717_regmap_cfg = {
	.name = "max96717",
	.reg_bits = 16,
	.val_bits = 8,

	.max_register = MAX96717_FUNC_SAFE_CC_RTTN_ERR,
	.writeable_reg = max96717_writable_register,
	.volatile_reg = max96717_volatile_reg,

	.cache_type = REGCACHE_RBTREE,
};

static int max96717_reset_sensor(struct max96717 *ser, unsigned int gpio)
{
	int ret;

	ret = regmap_write(ser->rmap, MAX96717_GPIO_A(gpio), RES_CFG);
	msleep(35);
	ret |= regmap_write(ser->rmap, MAX96717_GPIO_A(gpio), RES_CFG | GPIO_OUT);
	msleep(35);

	return ret ? -EIO : 0;
}

int max96717_tunnel_mode_en(struct max96717 *ser, bool en)
{
	return regmap_update_bits(ser->rmap, MAX96717_MIPI_RX_EXT_11, TUN_MODE, en ? TUN_MODE : 0);
}
EXPORT_SYMBOL(max96717_tunnel_mode_en);

int max96717_set_lanes_no(struct max96717 *ser, int lanes_no)
{
	return regmap_update_bits(ser->rmap, MAX96717_MIPI_RX_1, CTRL1_NUM_LANES_MASK,
				  (lanes_no - 1) << CTRL1_NUM_LANES_SHIFT);
}
EXPORT_SYMBOL(max96717_set_lanes_no);

/* Map serializer CSI lane to sensor CSI lane */
int max96717_map_csi_lanes(struct max96717 *ser, int *map, int map_size)
{
	u8 phy1_lane_map, phy2_lane_map;
	int ret;

	if (map_size < 4)
		return -EINVAL;

	phy2_lane_map = (map[0] & 0x3) | ((map[1] & 0x3) << 2);
	phy1_lane_map = (map[2] & 0x3) | ((map[3] & 0x3) << 2);

	ret = regmap_write(ser->rmap, MAX96717_MIPI_RX_2, phy1_lane_map << PHY1_LANE_MAP_SHIFT);
	ret |= regmap_write(ser->rmap, MAX96717_MIPI_RX_3, phy2_lane_map << PHY2_LANE_MAP_SHIFT);

	return ret ? -EIO : 0;
}
EXPORT_SYMBOL(max96717_map_csi_lanes);

/* Set lane polarities:
 * pol[0] = clock
 * pol[1:4] = lane 0 to 3
 */
int max96717_set_lanes_polarity(struct max96717 *ser, int *pol, int pol_size)
{
	u8 phy2_pol_map, phy1_pol_map;
	int ret;

	if (pol_size < 5)
		return -EINVAL;

	phy2_pol_map = (pol[1] & 0x1) | ((pol[2] & 0x1) << 1) | ((pol[0] & 0x1) << 2);
	phy1_pol_map = (pol[3] & 0x1) | ((pol[4] & 0x1) << 1);

	ret = regmap_write(ser->rmap, MAX96717_MIPI_RX_4, phy1_pol_map);
	ret |= regmap_write(ser->rmap, MAX96717_MIPI_RX_5, phy2_pol_map);

	return ret ? -EIO : 0;
}
EXPORT_SYMBOL(max96717_set_lanes_polarity);

int max96717_csi_port_en(struct max96717 *ser, bool en)
{
	return regmap_update_bits(ser->rmap, MAX96717_FRONTTOP_0, START_PORTB,
				  en ? START_PORTB : 0);
}
EXPORT_SYMBOL(max96717_csi_port_en);

int max96717_video_pipe_en(struct max96717 *ser, bool en)
{
	return regmap_write(ser->rmap, MAX96717_FRONTTOP_9, en ? START_PORTBZ : 0);
}
EXPORT_SYMBOL(max96717_video_pipe_en);

int max96717_video_pipe_tx_en(struct max96717 *ser, bool en)
{
	/* apparently, we need to set the 2 reserved bits as well */
	return regmap_write(ser->rmap, MAX96717_DEV_REG2, en ? (VID_TX_EN_Z | 0x3) : 0);
}
EXPORT_SYMBOL(max96717_video_pipe_tx_en);

int max96717_data_type_filter(struct max96717 *ser, u8 *dt_array, u8 dt_array_size)
{
	int ret = 0, i;
	int dt_filter_reg_map[] = {
		MAX96717_FRONTTOP_16,
		MAX96717_FRONTTOP_17,
		MAX96717_FRONTTOP_EXT_MEM_DT3_SELZ,
		MAX96717_FRONTTOP_EXT_MEM_DT4_SELZ,
		MAX96717_FRONTTOP_EXT_MEM_DT5_SELZ,
		MAX96717_FRONTTOP_EXT_MEM_DT6_SELZ,
		MAX96717_MIPI_RX_EXT2_A,
		MAX96717_MIPI_RX_EXT2_B,
	};

	if (dt_array_size > ARRAY_SIZE(dt_filter_reg_map))
		return -ERANGE;

	for (i = 0; i < dt_array_size; i++) {
		if (i > 1 && i < 6) {
			ret |= regmap_write(ser->rmap, dt_filter_reg_map[i], dt_array[i]);
			ret |= regmap_update_bits(ser->rmap, MAX96717_FRONTTOP_EXT_17,
						  BIT(i - 2), BIT(i - 2));
			continue;
		}

		ret |= regmap_write(ser->rmap, dt_filter_reg_map[i], BIT(6) | dt_array[i]);
	}

	return ret ? -EIO : 0;
}
EXPORT_SYMBOL(max96717_data_type_filter);

/*
 * Each bit in the filter_map represents whether a virtual channel's packets are processed or not:
 * For example:
 * filter_map = 0x0003 => only VC0 and VC1 packets are processed.
 */
int max96717_vc_filter(struct max96717 *ser, u16 filter_map)
{
	int ret;

	ret = regmap_write(ser->rmap, MAX96717_FRONTTOP_VC_SELZ_H, (filter_map >> 8) & 0xff);
	ret |= regmap_write(ser->rmap, MAX96717_FRONTTOP_VC_SELZ_L, filter_map & 0xff);

	return ret ? -EIO : 0;
}
EXPORT_SYMBOL(max96717_vc_filter);

int max96717_stream_id_set(struct max96717 *ser, int id)
{
	return regmap_write(ser->rmap, MAX96717_CFGV_VIDEO_Z_TX3, id);
}
EXPORT_SYMBOL(max96717_stream_id_set);

static int max96717_fsync_setup(struct max96717 *ser)
{
	int ret;

	/* setting up GPIO0 to receive fsync signals from deserializer with ID = 0x00 */
	ret = regmap_write(ser->rmap, MAX96717_GPIO_B(0),
			   (0x00 << GPIO_TX_ID_SHIFT) | OUT_TYPE_PUSH_PULL |
			   (GPIO_PULL_UP << PULL_UPDN_SEL_SHIFT));
	ret |= regmap_write(ser->rmap, MAX96717_GPIO_A(0), RES_CFG | GPIO_RX_EN);

	return ret ? -EIO : 0;
}

int max96717_gmsl_speed_set(struct max96717 *ser, enum max96717_gmsl_speed speed)
{
	struct regmap *rmap = ser->rmap;
	int ret;
	int reg_val;

	/* Set GMSL link speed to 6Gbps */
	ret = regmap_update_bits(rmap, MAX96717_DEV_REG1, TX_RATE_MASK, speed << TX_RATE_SHIFT);

	/* Reset GMSL link */
	ret |= regmap_write(rmap, MAX96717_TCTRL_CTRL0, RESET_ONESHOT);

	/* According to specs, lock time is 35ms. Give it a little more though, to be safe. */
	msleep(100);

	ret |= regmap_read(rmap, MAX96717_TCTRL_CTRL3, &reg_val);
	if (!(reg_val & LOCKED)) {
		dev_err(ser->dev, "%s: GMSL link not locked, MAX96717_TCTRL_CTRL3 = 0x%02x\n",
			__func__, reg_val);
		return -ENODEV;
	}

	return ret ? -EIO : 0;
}
EXPORT_SYMBOL(max96717_gmsl_speed_set);

int max96717_double_mode_en(struct max96717 *ser)
{
	return regmap_write(ser->rmap, MAX96717_FRONTTOP_10, BPP8DBLZ);
}
EXPORT_SYMBOL(max96717_double_mode_en);

int max96717_soft_bpp_override(struct max96717 *ser, int bpp)
{
	return regmap_write(ser->rmap, MAX96717_FRONTTOP_22, SOFT_BPPZ_EN | (bpp & 0x1f));
}
EXPORT_SYMBOL(max96717_soft_bpp_override);

int max96717_hw_init(struct max96717 *ser, unsigned int reset_pin, unsigned int clock_pin)
{
	struct regmap *rmap = ser->rmap;
	int ret;

	/*
	 * According to specs, we need to enable the 1.1V internal regulator to guarantee proper
	 * device operation.
	 */
	ret = regmap_write(rmap, MAX96717_CMU_CMU2, PFDDIV_VREG_1V1 << PFDDIV_RSHORT_SHIFT);

	/* Disable video transmission */
	ret |= regmap_update_bits(rmap, MAX96717_DEV_REG2, VID_TX_EN_Z, 0);

	/* Disable CSI port */
	ret |= regmap_update_bits(rmap, MAX96717_FRONTTOP_0, START_PORTB, 0);

	/* Activate reference clock to camera sensor on MFP4 */
	ret |= regmap_update_bits(rmap, MAX96717_REF_VTG_1, PCLK_GPIO_MASK | PCLKEN,
				  (4 << PCLK_GPIO_SHIFT) | PCLKEN);

	/* Set PLL clock to 24 MHz */
	ret |= regmap_write(rmap, MAX96717_REF_VTG_0,
			    REFGEN_PREDEF_EN | (1 << REFGEN_PREDEF_FREQ_SHIFT) |
			    REFGEN_PREDEF_FREQ_ALT | REFGEN_EN);

	/* Adjust MFP4 slew rate to faster rise and fall times. */
	ret |= regmap_update_bits(rmap, MAX96717_MISC_PIO_SLEW_1, PIO06_SLEW_MASK,
				  1 << PIO06_SLEW_SHIFT);

	/* Setup the sensor reset pin type. */
	ret |= regmap_update_bits(rmap, MAX96717_GPIO_B(reset_pin),
				  PULL_UPDN_SEL_MASK | OUT_TYPE,
				  (GPIO_PULL_UP << PULL_UPDN_SEL_SHIFT) | OUT_TYPE_PUSH_PULL);

	/* Setup the pull-up resistor to 1Mohm and enable output driver. */
	ret |= regmap_write(rmap, MAX96717_GPIO_A(reset_pin), RES_CFG | GPIO_OUT);

	/* Setup pin for providing sensor clock */
	ret |= regmap_update_bits(rmap, MAX96717_GPIO_B(clock_pin),
				  PULL_UPDN_SEL_MASK | OUT_TYPE,
				  (GPIO_PULL_DOWN << PULL_UPDN_SEL_SHIFT) |
				  OUT_TYPE_PUSH_PULL);

	/* Setup the pull-up resistor to 40kohm and enable output driver. */
	ret |= regmap_update_bits(rmap, MAX96717_GPIO_A(clock_pin),
				  RES_CFG | GPIO_OUT_DIS | GPIO_RX_EN, RES_CFG | GPIO_OUT_DIS);

	ret |= max96717_fsync_setup(ser);

	ret |= max96717_reset_sensor(ser, reset_pin);

	return ret ? -EIO : 0;
}
EXPORT_SYMBOL(max96717_hw_init);

bool max96717_is_dev_id_valid(struct max96717 *ser)
{
	int ret;
	int chip_id;

	ret = regmap_read(ser->rmap, MAX96717_DEV_REG13, &chip_id);
	if (ret) {
		dev_err(ser->dev, "Failed to read device id: %d\n", ret);
		return false;
	}

	if (chip_id != MAX96717_DEV_ID && chip_id != MAX96717F_DEV_ID) {
		dev_err(ser->dev, "Wrong Maxim serializer detected: id 0x%x\n", chip_id);
		return false;
	}

	return true;
}
EXPORT_SYMBOL(max96717_is_dev_id_valid);

struct max96717 *max96717_init(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96717 *ser;

	ser = devm_kzalloc(dev, sizeof(*ser), GFP_KERNEL);
	if (!ser)
		return ERR_PTR(-ENOMEM);

	ser->dev = &client->dev;

	ser->rmap = devm_regmap_init_i2c(client, &max96717_regmap_cfg);
	if (IS_ERR(ser->rmap))
		dev_err(dev, "Failed to allocate register map: %ld\n", PTR_ERR(ser->rmap));

	return ser;
}
EXPORT_SYMBOL(max96717_init);

MODULE_DESCRIPTION("Maxim MAX96717 GMSL2 serializer library");
MODULE_AUTHOR("Laurentiu Palcu");
MODULE_LICENSE("GPL");
