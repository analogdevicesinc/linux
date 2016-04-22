/*
 *  Copyright Altera Corporation (C) 2014-2016. All Rights Reserved
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
 *
 * HW Monitor driver for  Altera Arria10 MAX5 System Resource Chip
 * Adapted from DA9052
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mfd/altera-a10sr.h>
#include <linux/module.h>
#include <linux/of.h>

#define ALTR_A10SR_1V0_BIT_POS          ALTR_A10SR_PG1_1V0_SHIFT
#define ALTR_A10SR_0V95_BIT_POS         ALTR_A10SR_PG1_0V95_SHIFT
#define ALTR_A10SR_0V9_BIT_POS          ALTR_A10SR_PG1_0V9_SHIFT
#define ALTR_A10SR_10V_BIT_POS          ALTR_A10SR_PG1_10V_SHIFT
#define ALTR_A10SR_5V0_BIT_POS          ALTR_A10SR_PG1_5V0_SHIFT
#define ALTR_A10SR_3V3_BIT_POS          ALTR_A10SR_PG1_3V3_SHIFT
#define ALTR_A10SR_2V5_BIT_POS          ALTR_A10SR_PG1_2V5_SHIFT
#define ALTR_A10SR_1V8_BIT_POS          ALTR_A10SR_PG1_1V8_SHIFT
#define ALTR_A10SR_OP_FLAG_BIT_POS      ALTR_A10SR_PG1_OP_FLAG_SHIFT
/* 2nd register needs an offset of 8 to get to 2nd register */
#define ALTR_A10SR_FBC2MP_BIT_POS       (8 + ALTR_A10SR_PG2_FBC2MP_SHIFT)
#define ALTR_A10SR_FAC2MP_BIT_POS       (8 + ALTR_A10SR_PG2_FAC2MP_SHIFT)
#define ALTR_A10SR_FMCBVADJ_BIT_POS     (8 + ALTR_A10SR_PG2_FMCBVADJ_SHIFT)
#define ALTR_A10SR_FMCAVADJ_BIT_POS     (8 + ALTR_A10SR_PG2_FMCAVADJ_SHIFT)
#define ALTR_A10SR_HL_VDDQ_BIT_POS      (8 + ALTR_A10SR_PG2_HL_VDDQ_SHIFT)
#define ALTR_A10SR_HL_VDD_BIT_POS       (8 + ALTR_A10SR_PG2_HL_VDD_SHIFT)
#define ALTR_A10SR_HL_HPS_BIT_POS       (8 + ALTR_A10SR_PG2_HL_HPS_SHIFT)
#define ALTR_A10SR_HPS_BIT_POS          (8 + ALTR_A10SR_PG2_HPS_SHIFT)
/* 3rd register needs an offset of 16 to get to 3rd register */
#define ALTR_A10SR_PCIE_WAKE_BIT_POS    (16 + ALTR_A10SR_PG3_PCIE_WAKE_SHIFT)
#define ALTR_A10SR_PCIE_PR_BIT_POS      (16 + ALTR_A10SR_PG3_PCIE_PR_SHIFT)
#define ALTR_A10SR_FMCB_PR_BIT_POS      (16 + ALTR_A10SR_PG3_FMCB_PR_SHIFT)
#define ALTR_A10SR_FMCA_PR_BIT_POS      (16 + ALTR_A10SR_PG3_FMCA_PR_SHIFT)
#define ALTR_A10SR_FILE_PR_BIT_POS      (16 + ALTR_A10SR_PG3_FILE_PR_SHIFT)
#define ALTR_A10SR_BF_PR_BIT_POS        (16 + ALTR_A10SR_PG3_BF_PR_SHIFT)
#define ALTR_A10SR_10V_FAIL_BIT_POS     (16 + ALTR_A10SR_PG3_10V_FAIL_SHIFT)
#define ALTR_A10SR_FAM2C_BIT_POS        (16 + ALTR_A10SR_PG3_FAM2C_SHIFT)
/* FMCA/B & PCIE Enables need an offset of 24 */
#define ALTR_A10SR_FMCB_AUXEN_POS       (24 + ALTR_A10SR_FMCB_AUXEN_SHIFT)
#define ALTR_A10SR_FMCB_EN_POS          (24 + ALTR_A10SR_FMCB_EN_SHIFT)
#define ALTR_A10SR_FMCA_AUXEN_POS       (24 + ALTR_A10SR_FMCA_AUXEN_SHIFT)
#define ALTR_A10SR_FMCA_EN_POS          (24 + ALTR_A10SR_FMCA_EN_SHIFT)
#define ALTR_A10SR_PCIE_AUXEN_POS       (24 + ALTR_A10SR_PCIE_AUXEN_SHIFT)
#define ALTR_A10SR_PCIE_EN_POS          (24 + ALTR_A10SR_PCIE_EN_SHIFT)
/* HPS Resets need an offset of 32 */
#define ALTR_A10SR_HPS_RST_UART_POS     (32 + ALTR_A10SR_HPS_UARTA_RSTN_SHIFT)
#define ALTR_A10SR_HPS_RST_WARM_POS     (32 + ALTR_A10SR_HPS_WARM_RSTN_SHIFT)
#define ALTR_A10SR_HPS_RST_WARM1_POS    (32 + ALTR_A10SR_HPS_WARM_RST1N_SHIFT)
#define ALTR_A10SR_HPS_RST_COLD_POS     (32 + ALTR_A10SR_HPS_COLD_RSTN_SHIFT)
#define ALTR_A10SR_HPS_RST_NPOR_POS     (32 + ALTR_A10SR_HPS_NPOR_SHIFT)
#define ALTR_A10SR_HPS_RST_NRST_POS     (32 + ALTR_A10SR_HPS_NRST_SHIFT)
#define ALTR_A10SR_HPS_RST_ENET_POS     (32 + ALTR_A10SR_HPS_ENET_RSTN_SHIFT)
#define ALTR_A10SR_HPS_RST_ENETINT_POS  (32 + ALTR_A10SR_HPS_ENET_INTN_SHIFT)
/* Peripheral Resets need an offset of 40 */
#define ALTR_A10SR_PER_RST_USB_POS      (40 + ALTR_A10SR_USB_RST_SHIFT)
#define ALTR_A10SR_PER_RST_BQSPI_POS    (40 + ALTR_A10SR_BQSPI_RST_N_SHIFT)
#define ALTR_A10SR_PER_RST_FILE_POS     (40 + ALTR_A10SR_FILE_RST_N_SHIFT)
#define ALTR_A10SR_PER_RST_PCIE_POS     (40 + ALTR_A10SR_PCIE_PERST_N_SHIFT)
/* HWMON - Read Entire Register */
#define ALTR_A10SR_ENTIRE_REG           (88)
#define ALTR_A10SR_ENTIRE_REG_MASK      (0xFF)
#define ALTR_A10SR_VERSION              (0 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_LED                  (1 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_PB                   (2 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_PBF                  (3 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_PG1                  (4 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_PG2                  (5 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_PG3                  (6 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_FMCAB                (7 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_HPS_RST              (8 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_PER_RST              (9 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_SFPA                 (10 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_SFPB                 (11 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_I2C_MASTER           (12 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_WARM_RST             (13 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_WARM_RST_KEY         (14 + ALTR_A10SR_ENTIRE_REG)
#define ALTR_A10SR_PMBUS                (15 + ALTR_A10SR_ENTIRE_REG)

/**
 * struct altr_a10sr_hwmon - Altera Max5 HWMON device private data structure
 * @device: hwmon class.
 * @regmap: the regmap from the parent device.
 */
struct altr_a10sr_hwmon {
	struct device		*class_device;
	struct regmap		*regmap;
};

static ssize_t altr_a10sr_read_status(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct altr_a10sr_hwmon *hwmon = dev_get_drvdata(dev);
	int val, ret, index = to_sensor_dev_attr(devattr)->index;
	int mask = ALTR_A10SR_REG_BIT_MASK(index);
	unsigned char reg = ALTR_A10SR_PWR_GOOD1_REG +
			    ALTR_A10SR_REG_OFFSET(index);

	/* Check if this is an entire register read */
	if (index >= ALTR_A10SR_ENTIRE_REG) {
		reg = ((index - ALTR_A10SR_ENTIRE_REG) << 1);
		mask = ALTR_A10SR_ENTIRE_REG_MASK;
	}

	ret = regmap_read(hwmon->regmap, reg, &val);
	if (ret < 0)
		return ret;

	if (mask == ALTR_A10SR_ENTIRE_REG_MASK)
		val = val & mask;
	else
		val = !!(val & mask);

	return sprintf(buf, "%d\n", val);
}

static ssize_t altr_a10sr_hwmon_show_name(struct device *dev,
					  struct device_attribute *devattr,
					  char *buf)
{
	return sprintf(buf, "altr_a10sr\n");
}

static ssize_t set_enable(struct device *dev,
			  struct device_attribute *dev_attr,
			  const char *buf, size_t count)
{
	unsigned long val;
	struct altr_a10sr_hwmon *hwmon = dev_get_drvdata(dev);
	int ret, index = to_sensor_dev_attr(dev_attr)->index;
	int mask = ALTR_A10SR_REG_BIT_MASK(index);
	unsigned char reg = (ALTR_A10SR_PWR_GOOD1_REG & WRITE_REG_MASK) +
			    ALTR_A10SR_REG_OFFSET(index);
	int res = kstrtol(buf, 10, &val);

	if (res < 0)
		return res;

	/* Check if this is an entire register write */
	if (index >= ALTR_A10SR_ENTIRE_REG) {
		reg = ((index - ALTR_A10SR_ENTIRE_REG) << 1);
		mask = ALTR_A10SR_ENTIRE_REG_MASK;
	}

	ret = regmap_update_bits(hwmon->regmap, reg, mask, val);
	if (ret < 0)
		return ret;

	return count;
}

/* First Power Good Register Bits */
static SENSOR_DEVICE_ATTR(1v0_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_1V0_BIT_POS);
static SENSOR_DEVICE_ATTR(0v95_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_0V95_BIT_POS);
static SENSOR_DEVICE_ATTR(0v9_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_0V9_BIT_POS);
static SENSOR_DEVICE_ATTR(5v0_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_5V0_BIT_POS);
static SENSOR_DEVICE_ATTR(3v3_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_3V3_BIT_POS);
static SENSOR_DEVICE_ATTR(2v5_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_2V5_BIT_POS);
static SENSOR_DEVICE_ATTR(1v8_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_1V8_BIT_POS);
static SENSOR_DEVICE_ATTR(opflag_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_OP_FLAG_BIT_POS);
/* Second Power Good Register Bits */
static SENSOR_DEVICE_ATTR(fbc2mp_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FBC2MP_BIT_POS);
static SENSOR_DEVICE_ATTR(fac2mp_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FAC2MP_BIT_POS);
static SENSOR_DEVICE_ATTR(fmcbvadj_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FMCBVADJ_BIT_POS);
static SENSOR_DEVICE_ATTR(fmcavadj_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FMCAVADJ_BIT_POS);
static SENSOR_DEVICE_ATTR(hl_vddq_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_HL_VDDQ_BIT_POS);
static SENSOR_DEVICE_ATTR(hl_vdd_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_HL_VDD_BIT_POS);
static SENSOR_DEVICE_ATTR(hlhps_vdd_alarm, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_HL_HPS_BIT_POS);
static SENSOR_DEVICE_ATTR(hps_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_HPS_BIT_POS);
/* Third Power Good Register Bits */
static SENSOR_DEVICE_ATTR(pcie_wake_input, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_PCIE_WAKE_BIT_POS);
static SENSOR_DEVICE_ATTR(pcie_pr_input, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_PCIE_PR_BIT_POS);
static SENSOR_DEVICE_ATTR(fmcb_pr_input, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FMCB_PR_BIT_POS);
static SENSOR_DEVICE_ATTR(fmca_pr_input, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FMCA_PR_BIT_POS);
static SENSOR_DEVICE_ATTR(file_pr_input, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FILE_PR_BIT_POS);
static SENSOR_DEVICE_ATTR(bf_pr_input, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_BF_PR_BIT_POS);
static SENSOR_DEVICE_ATTR(10v_alarm, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_10V_FAIL_BIT_POS);
static SENSOR_DEVICE_ATTR(fam2c_alarm, S_IRUGO, altr_a10sr_read_status, NULL,
			  ALTR_A10SR_FAM2C_BIT_POS);
/* Peripheral Enable bits */
static SENSOR_DEVICE_ATTR(fmcb_aux_en, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_FMCB_AUXEN_POS);
static SENSOR_DEVICE_ATTR(fmcb_en, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_FMCB_EN_POS);
static SENSOR_DEVICE_ATTR(fmca_aux_en, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_FMCA_AUXEN_POS);
static SENSOR_DEVICE_ATTR(fmca_en, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_FMCA_EN_POS);
static SENSOR_DEVICE_ATTR(pcie_aux_en, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_PCIE_AUXEN_POS);
static SENSOR_DEVICE_ATTR(pcie_en, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_PCIE_EN_POS);
/* HPS Reset bits */
static SENSOR_DEVICE_ATTR(hps_uart_rst, S_IRUGO,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_UART_POS);
static SENSOR_DEVICE_ATTR(hps_warm_rst, S_IRUGO,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_WARM_POS);
static SENSOR_DEVICE_ATTR(hps_warm1_rst, S_IRUGO,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_WARM1_POS);
static SENSOR_DEVICE_ATTR(hps_cold_rst, S_IRUGO,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_COLD_POS);
static SENSOR_DEVICE_ATTR(hps_npor, S_IRUGO,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_NPOR_POS);
static SENSOR_DEVICE_ATTR(hps_nrst, S_IRUGO,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_NRST_POS);
static SENSOR_DEVICE_ATTR(hps_enet_rst, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_ENET_POS);
static SENSOR_DEVICE_ATTR(hps_enet_int, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST_ENETINT_POS);
/* Peripheral Reset bits */
static SENSOR_DEVICE_ATTR(usb_reset, S_IRUGO | S_IWUSR, altr_a10sr_read_status,
			  set_enable, ALTR_A10SR_PER_RST_USB_POS);
static SENSOR_DEVICE_ATTR(bqspi_resetn, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_PER_RST_BQSPI_POS);
static SENSOR_DEVICE_ATTR(file_resetn, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_PER_RST_FILE_POS);
static SENSOR_DEVICE_ATTR(pcie_perstn, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_PER_RST_PCIE_POS);
/* Entire Byte Read */
static SENSOR_DEVICE_ATTR(max5_version, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_VERSION);
static SENSOR_DEVICE_ATTR(max5_led, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_LED);
static SENSOR_DEVICE_ATTR(max5_button, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_PB);
static SENSOR_DEVICE_ATTR(max5_button_irq, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable, ALTR_A10SR_PBF);
static SENSOR_DEVICE_ATTR(max5_pg1, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_PG1);
static SENSOR_DEVICE_ATTR(max5_pg2, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_PG2);
static SENSOR_DEVICE_ATTR(max5_pg3, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_PG3);
static SENSOR_DEVICE_ATTR(max5_fmcab, S_IRUGO, altr_a10sr_read_status,
			  NULL, ALTR_A10SR_FMCAB);
static SENSOR_DEVICE_ATTR(max5_hps_resets, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_HPS_RST);
static SENSOR_DEVICE_ATTR(max5_per_resets, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_PER_RST);
static SENSOR_DEVICE_ATTR(max5_sfpa, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable, ALTR_A10SR_SFPA);
static SENSOR_DEVICE_ATTR(max5_sfpb, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable, ALTR_A10SR_SFPB);
static SENSOR_DEVICE_ATTR(max5_i2c_master, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_I2C_MASTER);
static SENSOR_DEVICE_ATTR(max5_pmbus, S_IRUGO | S_IWUSR,
			  altr_a10sr_read_status, set_enable,
			  ALTR_A10SR_PMBUS);

static DEVICE_ATTR(name, S_IRUGO, altr_a10sr_hwmon_show_name, NULL);

static struct attribute *altr_a10sr_attr[] = {
	&dev_attr_name.attr,
	/* First Power Good Register */
	&sensor_dev_attr_opflag_alarm.dev_attr.attr,
	&sensor_dev_attr_1v8_alarm.dev_attr.attr,
	&sensor_dev_attr_2v5_alarm.dev_attr.attr,
	&sensor_dev_attr_1v0_alarm.dev_attr.attr,
	&sensor_dev_attr_3v3_alarm.dev_attr.attr,
	&sensor_dev_attr_5v0_alarm.dev_attr.attr,
	&sensor_dev_attr_0v9_alarm.dev_attr.attr,
	&sensor_dev_attr_0v95_alarm.dev_attr.attr,
	/* Second Power Good Register */
	&sensor_dev_attr_hps_alarm.dev_attr.attr,
	&sensor_dev_attr_hlhps_vdd_alarm.dev_attr.attr,
	&sensor_dev_attr_hl_vdd_alarm.dev_attr.attr,
	&sensor_dev_attr_hl_vddq_alarm.dev_attr.attr,
	&sensor_dev_attr_fmcavadj_alarm.dev_attr.attr,
	&sensor_dev_attr_fmcbvadj_alarm.dev_attr.attr,
	&sensor_dev_attr_fac2mp_alarm.dev_attr.attr,
	&sensor_dev_attr_fbc2mp_alarm.dev_attr.attr,
	/* Third Power Good Register */
	&sensor_dev_attr_pcie_wake_input.dev_attr.attr,
	&sensor_dev_attr_pcie_pr_input.dev_attr.attr,
	&sensor_dev_attr_fmcb_pr_input.dev_attr.attr,
	&sensor_dev_attr_fmca_pr_input.dev_attr.attr,
	&sensor_dev_attr_file_pr_input.dev_attr.attr,
	&sensor_dev_attr_bf_pr_input.dev_attr.attr,
	&sensor_dev_attr_10v_alarm.dev_attr.attr,
	&sensor_dev_attr_fam2c_alarm.dev_attr.attr,
/* Peripheral Enable Register */
	&sensor_dev_attr_fmcb_aux_en.dev_attr.attr,
	&sensor_dev_attr_fmcb_en.dev_attr.attr,
	&sensor_dev_attr_fmca_aux_en.dev_attr.attr,
	&sensor_dev_attr_fmca_en.dev_attr.attr,
	&sensor_dev_attr_pcie_aux_en.dev_attr.attr,
	&sensor_dev_attr_pcie_en.dev_attr.attr,
	/* HPS Reset bits */
	&sensor_dev_attr_hps_uart_rst.dev_attr.attr,
	&sensor_dev_attr_hps_warm_rst.dev_attr.attr,
	&sensor_dev_attr_hps_warm1_rst.dev_attr.attr,
	&sensor_dev_attr_hps_cold_rst.dev_attr.attr,
	&sensor_dev_attr_hps_npor.dev_attr.attr,
	&sensor_dev_attr_hps_nrst.dev_attr.attr,
	&sensor_dev_attr_hps_enet_rst.dev_attr.attr,
	&sensor_dev_attr_hps_enet_int.dev_attr.attr,
	/* Peripheral Reset bits */
	&sensor_dev_attr_usb_reset.dev_attr.attr,
	&sensor_dev_attr_bqspi_resetn.dev_attr.attr,
	&sensor_dev_attr_file_resetn.dev_attr.attr,
	&sensor_dev_attr_pcie_perstn.dev_attr.attr,
	/* Byte Value Register */
	&sensor_dev_attr_max5_version.dev_attr.attr,
	&sensor_dev_attr_max5_led.dev_attr.attr,
	&sensor_dev_attr_max5_button.dev_attr.attr,
	&sensor_dev_attr_max5_button_irq.dev_attr.attr,
	&sensor_dev_attr_max5_pg1.dev_attr.attr,
	&sensor_dev_attr_max5_pg2.dev_attr.attr,
	&sensor_dev_attr_max5_pg3.dev_attr.attr,
	&sensor_dev_attr_max5_fmcab.dev_attr.attr,
	&sensor_dev_attr_max5_hps_resets.dev_attr.attr,
	&sensor_dev_attr_max5_per_resets.dev_attr.attr,
	&sensor_dev_attr_max5_sfpa.dev_attr.attr,
	&sensor_dev_attr_max5_sfpb.dev_attr.attr,
	&sensor_dev_attr_max5_i2c_master.dev_attr.attr,
	&sensor_dev_attr_max5_pmbus.dev_attr.attr,
	NULL
};

static const struct attribute_group altr_a10sr_attr_group = {
	.attrs = altr_a10sr_attr
};

static int altr_a10sr_hwmon_probe(struct platform_device *pdev)
{
	struct altr_a10sr_hwmon *hwmon;
	int ret;
	struct altr_a10sr *a10sr = dev_get_drvdata(pdev->dev.parent);

	hwmon = devm_kzalloc(&pdev->dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->regmap = a10sr->regmap;

	platform_set_drvdata(pdev, hwmon);

	ret = sysfs_create_group(&pdev->dev.kobj, &altr_a10sr_attr_group);
	if (ret)
		goto err_mem;

	hwmon->class_device = hwmon_device_register(&pdev->dev);
	if (IS_ERR(hwmon->class_device)) {
		ret = PTR_ERR(hwmon->class_device);
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	sysfs_remove_group(&pdev->dev.kobj, &altr_a10sr_attr_group);
err_mem:
	return ret;
}

static int altr_a10sr_hwmon_remove(struct platform_device *pdev)
{
	struct altr_a10sr_hwmon *hwmon = platform_get_drvdata(pdev);

	hwmon_device_unregister(hwmon->class_device);
	sysfs_remove_group(&pdev->dev.kobj, &altr_a10sr_attr_group);

	return 0;
}

static const struct of_device_id altr_a10sr_hwmon_of_match[] = {
	{ .compatible = "altr,a10sr-hwmon" },
	{ },
};
MODULE_DEVICE_TABLE(of, altr_a10sr_hwmon_of_match);

static struct platform_driver altr_a10sr_hwmon_driver = {
	.probe = altr_a10sr_hwmon_probe,
	.remove = altr_a10sr_hwmon_remove,
	.driver = {
		.name = "altr_a10sr_hwmon",
		.of_match_table = of_match_ptr(altr_a10sr_hwmon_of_match),
	},
};

module_platform_driver(altr_a10sr_hwmon_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Thor Thayer <tthayer@opensource.altera.com>");
MODULE_DESCRIPTION("HW Monitor driver for Altera Arria10 System Resource Chip");
