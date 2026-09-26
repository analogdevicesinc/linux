// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Debugfs interface for MAX20355/MAX20357 PLC commands.
 * Exposes the PLC command set (UART, reset, seal, data transfer, GPIO)
 * for prototyping and debug. These are not stable ABI.
 */

#include <linux/debugfs.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/completion.h>

#include "max2035x.h"
#include "max2035x_registers.h"
#include "max2035x_plc.h"

#define MAX2035X_RAM_SIZE		128

/* PLC SYS_REQ argument codes */
#define MAX2035X_SYS_REQ_SEAL		0x00
#define MAX2035X_SYS_REQ_SRST		0x01
#define MAX2035X_SYS_REQ_HRST		0x02
#define MAX2035X_SYS_REQ_FGRST		0x03
#define MAX2035X_SYS_REQ_FIFO		0x04
#define MAX2035X_SYS_REQ_FREE		0x05
#define MAX2035X_SYS_REQ_IDLE		0x06

/* UART REQ argument codes */
#define MAX2035X_UART_MANUAL		0x0
#define MAX2035X_UART_MASTER_RX		0x1
#define MAX2035X_UART_MASTER_TX		0x2
#define MAX2035X_UART_LOCAL_LOOPBACK	0x3
#define MAX20357_UART_PLC2_MASTER_RX	0x4
#define MAX20357_UART_PLC2_MASTER_TX	0x8

/* -------------------------------------------------------------------------- */
/* Helper: parse a line from userspace                                        */
/* -------------------------------------------------------------------------- */
static ssize_t dbg_max2035x_parse_buf(const char __user *ubuf, size_t count,
			     char *kbuf, size_t kbuf_sz)
{
	if (count >= kbuf_sz)
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	return 0;
}

/* -------------------------------------------------------------------------- */
/* Internal PLC command helpers (ported from original dead code)               */
/* -------------------------------------------------------------------------- */
static bool dbg_max2035x_check_cmd_ongoing(struct max2035x *chip, int target)
{
	unsigned int reg, val;
	u8 run_bit;

	if (chip->type == MAX20355) {
		reg = (target == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
		run_bit = MAX20355_PLC_CMD_RUN_TRG_BIT;
	} else {
		reg = MAX20357_REG_PLC_CMD;
		run_bit = MAX20357_PLC_CMD_RUN_TRG_BIT;
	}
	if (regmap_read(chip->regmap, reg, &val))
		return false;
	return !(val & run_bit);
}

static int dbg_max2035x_send_syst_req(struct max2035x *chip, int target, u8 arg)
{
	unsigned int reg_arg, reg_cmd;
	int ret;

	if (chip->type == MAX20355) {
		reg_arg = (target == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
	}
	ret = regmap_write(chip->regmap, reg_arg, arg);
	if (ret)
		return ret;
	return regmap_write(chip->regmap, reg_cmd, 0x80 | 0x00);
}

static int dbg_max2035x_send_uart_req(struct max2035x *chip, int target, u8 arg)
{
	unsigned int reg_arg, reg_cmd;
	int ret;

	if (chip->type == MAX20355) {
		reg_arg = (target == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
	}
	ret = regmap_write(chip->regmap, reg_arg, arg);
	if (ret)
		return ret;
	return regmap_write(chip->regmap, reg_cmd, 0x80 | 0x06);
}

static int dbg_max2035x_send_dout_req(struct max2035x *chip, int target, u8 num_bytes)
{
	unsigned int reg_arg, reg_cmd;
	int ret;

	if (num_bytes < 1 || num_bytes > MAX2035X_RAM_SIZE)
		return -EINVAL;
	if (chip->type == MAX20355) {
		reg_arg = (target == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
	}
	ret = regmap_write(chip->regmap, reg_arg, num_bytes - 1);
	if (ret)
		return ret;
	return regmap_write(chip->regmap, reg_cmd, 0x80 | 0x05);
}

static int dbg_max2035x_set_gpio(struct max2035x *chip, int target, u8 arg)
{
	unsigned int reg_arg, reg_cmd;
	int ret;

	if (chip->type == MAX20355) {
		reg_arg = (target == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
	}
	ret = regmap_write(chip->regmap, reg_arg, arg);
	if (ret)
		return ret;
	return regmap_write(chip->regmap, reg_cmd, 0x80 | 0x03);
}

static int dbg_max2035x_write_ram_data(struct max2035x_plc *plc, const u8 *data, size_t len)
{
	if (len > MAX2035X_RAM_SIZE || !plc->ram_regmap)
		return -EINVAL;
	return regmap_bulk_write(plc->ram_regmap, 0x00, data, len);
}

/* -------------------------------------------------------------------------- */
/* UART debugfs files                                                         */
/* -------------------------------------------------------------------------- */

/* echo "channel tmo(0-3)" > uart/enter_auto */
static ssize_t dbg_max2035x_enter_uart_auto_write(struct file *f, const char __user *u,
					  size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[32];
	int ch, tmo, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (sscanf(buf, "%d %d", &ch, &tmo) != 2)
		return -EINVAL;

	if (chip->type == MAX20355) {
		u8 en_bit = (ch == 1) ? MAX20355_UART_CTR_URT_AUTO_EN1_BIT :
					MAX20355_UART_CTR_URT_AUTO_EN2_BIT;
		u8 reg = (ch == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;
		unsigned int mask = MAX20355_UART_CTR_I2C_URT_MOD_BIT |
				    MAX20355_UART_CTR_I2C_URT_SWC_BIT;

		regmap_update_bits(chip->regmap, reg, mask, 0);
		regmap_update_bits(chip->regmap, reg,
				   MAX20355_UART_CTR_TMO_TMR_ENA_BIT,
				   MAX20355_UART_CTR_TMO_TMR_ENA_BIT);
		ret = regmap_write(chip->regmap, MAX20355_REG_UART_CTR0,
				   en_bit | (tmo & 0x03));
		if (!ret)
			ret = dbg_max2035x_send_uart_req(chip, ch, MAX2035X_UART_MASTER_TX);
	} else {
		unsigned int mask = MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
				    MAX20357_UART_CTR1_I2C_URT_SWC_BIT;

		regmap_update_bits(chip->regmap, MAX20357_REG_UART_CTR1, mask, 0);
		regmap_update_bits(chip->regmap, MAX20357_REG_UART_CTR1,
				   MAX20357_UART_CTR1_TMO_TMR_ENA_BIT,
				   MAX20357_UART_CTR1_TMO_TMR_ENA_BIT);
		ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR0,
				   MAX20357_UART_CTR0_URT_AUTO_EN_BIT | (tmo & 0x03));
	}
	return ret ? ret : cnt;
}

/* echo "channel" > uart/enter_tx */
static ssize_t dbg_max2035x_enter_uart_tx_write(struct file *f, const char __user *u,
					size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int ch, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &ch))
		return -EINVAL;

	if (chip->type == MAX20355) {
		u8 reg = (ch == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;
		unsigned int mask = MAX20355_UART_CTR_I2C_URT_MOD_BIT |
				    MAX20355_UART_CTR_I2C_URT_SWC_BIT;

		regmap_update_bits(chip->regmap, reg, mask, 0);
		regmap_update_bits(chip->regmap, reg,
				   MAX20355_UART_CTR_TMO_TMR_ENA_BIT,
				   MAX20355_UART_CTR_TMO_TMR_ENA_BIT);
		ret = dbg_max2035x_send_uart_req(chip, ch, MAX2035X_UART_MASTER_TX);
	} else {
		unsigned int reg_val, mask;
		u8 arg = (chip->channel_id == 1) ? MAX2035X_UART_MASTER_TX :
						    MAX20357_UART_PLC2_MASTER_TX;

		mask = MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
		       MAX20357_UART_CTR1_I2C_URT_SWC_BIT;
		regmap_update_bits(chip->regmap, MAX20357_REG_UART_CTR1, mask, 0);
		regmap_update_bits(chip->regmap, MAX20357_REG_UART_CTR1,
				   MAX20357_UART_CTR1_TMO_TMR_ENA_BIT,
				   MAX20357_UART_CTR1_TMO_TMR_ENA_BIT);
		ret = dbg_max2035x_send_uart_req(chip, chip->channel_id, arg);
		if (ret)
			return ret;

		regmap_update_bits(chip->regmap, MAX20357_REG_PLC_CONFIG4,
				   MAX20357_PLC_CFG4_PLC_FSM_ENA_BIT, 0);
		regmap_read(chip->regmap, MAX20357_REG_UART_CTR1, &reg_val);
		reg_val |= MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
			   MAX20357_UART_CTR1_I2C_URT_ENA_BIT |
			   MAX20357_UART_CTR1_I2C_URT_SWC_BIT |
			   MAX20357_UART_CTR1_I2C_TX_SWC_BIT;
		reg_val &= ~MAX20357_UART_CTR1_I2C_RX_SWC_BIT;
		ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, reg_val);
	}
	return ret ? ret : cnt;
}

/* echo "channel" > uart/enter_rx */
static ssize_t dbg_max2035x_enter_uart_rx_write(struct file *f, const char __user *u,
					size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int ch, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &ch))
		return -EINVAL;

	if (chip->type == MAX20355) {
		u8 reg = (ch == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;
		unsigned int mask = MAX20355_UART_CTR_I2C_URT_MOD_BIT |
				    MAX20355_UART_CTR_I2C_URT_SWC_BIT;

		regmap_update_bits(chip->regmap, reg, mask, 0);
		regmap_update_bits(chip->regmap, reg,
				   MAX20355_UART_CTR_TMO_TMR_ENA_BIT,
				   MAX20355_UART_CTR_TMO_TMR_ENA_BIT);
		ret = dbg_max2035x_send_uart_req(chip, ch, MAX2035X_UART_MASTER_RX);
	} else {
		unsigned int reg_val, mask;
		u8 arg = (chip->channel_id == 1) ? MAX2035X_UART_MASTER_RX :
						    MAX20357_UART_PLC2_MASTER_RX;

		mask = MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
		       MAX20357_UART_CTR1_I2C_URT_SWC_BIT;
		regmap_update_bits(chip->regmap, MAX20357_REG_UART_CTR1, mask, 0);
		regmap_update_bits(chip->regmap, MAX20357_REG_UART_CTR1,
				   MAX20357_UART_CTR1_TMO_TMR_ENA_BIT,
				   MAX20357_UART_CTR1_TMO_TMR_ENA_BIT);
		ret = dbg_max2035x_send_uart_req(chip, chip->channel_id, arg);
		if (ret)
			return ret;

		regmap_read(chip->regmap, MAX20357_REG_UART_CTR1, &reg_val);
		reg_val |= MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
			   MAX20357_UART_CTR1_I2C_URT_ENA_BIT |
			   MAX20357_UART_CTR1_I2C_URT_SWC_BIT |
			   MAX20357_UART_CTR1_I2C_RX_SWC_BIT;
		reg_val &= ~MAX20357_UART_CTR1_I2C_TX_SWC_BIT;
		ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, reg_val);
	}
	return ret ? ret : cnt;
}

/* echo "channel" > uart/exit */
static ssize_t dbg_max2035x_exit_uart_write(struct file *f, const char __user *u,
				    size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int ch, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &ch))
		return -EINVAL;

	if (chip->type == MAX20355) {
		u8 reg = (ch == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;

		ret = regmap_write(chip->regmap, reg, 0x84);
		if (!ret)
			ret = regmap_write(chip->regmap, reg, 0x00);
	} else {
		ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, 0x28);
		if (!ret)
			ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, 0x00);
	}
	return ret ? ret : cnt;
}

/* echo "channel 0|1" > uart/control_mode (0=PLC, 1=I2C) */
static ssize_t dbg_max2035x_set_uart_control_mode_write(struct file *f, const char __user *u,
				       size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int ch, use_i2c, ret;
	unsigned int reg_val, mask;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (sscanf(buf, "%d %d", &ch, &use_i2c) != 2)
		return -EINVAL;

	if (chip->type == MAX20355) {
		u8 reg = (ch == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;

		mask = MAX20355_UART_CTR_I2C_URT_MOD_BIT |
		       MAX20355_UART_CTR_I2C_URT_SWC_BIT;
		ret = regmap_read(chip->regmap, reg, &reg_val);
		if (ret)
			return ret;
		if (use_i2c)
			reg_val |= mask;
		else
			reg_val &= ~mask;
		ret = regmap_write(chip->regmap, reg, reg_val);
	} else {
		mask = MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
		       MAX20357_UART_CTR1_I2C_URT_SWC_BIT;
		ret = regmap_read(chip->regmap, MAX20357_REG_UART_CTR1, &reg_val);
		if (ret)
			return ret;
		if (use_i2c)
			reg_val |= mask;
		else
			reg_val &= ~mask;
		ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, reg_val);
	}
	return ret ? ret : cnt;
}

/* -------------------------------------------------------------------------- */
/* Reset / Seal debugfs files                                                 */
/* -------------------------------------------------------------------------- */

/* echo "target use_plc(0|1)" > reset/soft */
static ssize_t dbg_max2035x_soft_reset_write(struct file *f, const char __user *u,
				     size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, use_plc, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (sscanf(buf, "%d %d", &target, &use_plc) != 2)
		return -EINVAL;

	if (use_plc) {
		if (chip->type == MAX20357)
			target = 0;
		if (!dbg_max2035x_check_cmd_ongoing(chip, target))
			return -EBUSY;
		ret = dbg_max2035x_send_syst_req(chip, target, MAX2035X_SYS_REQ_SRST);
	} else {
		if (chip->type == MAX20355)
			ret = regmap_update_bits(chip->regmap, MAX20355_REG_SYSTEM_REG0,
						 MAX20355_SYSTEM_SOFT_RESET_BIT,
						 MAX20355_SYSTEM_SOFT_RESET_BIT);
		else
			ret = regmap_update_bits(chip->regmap, MAX20357_REG_SYSTEM_REG0,
						 MAX20357_SYSTEM_SOFT_RESET_BIT,
						 MAX20357_SYSTEM_SOFT_RESET_BIT);
	}
	return ret ? ret : cnt;
}

/* echo "target" > reset/hard */
static ssize_t dbg_max2035x_hard_reset_write(struct file *f, const char __user *u,
				     size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &target))
		return -EINVAL;

	if (chip->type == MAX20357)
		ret = regmap_update_bits(chip->regmap, MAX20357_REG_SYSTEM_REG0,
					 MAX20357_SYSTEM_HARD_RESET_BIT,
					 MAX20357_SYSTEM_HARD_RESET_BIT);
	else {
		if (!dbg_max2035x_check_cmd_ongoing(chip, target))
			return -EBUSY;
		ret = dbg_max2035x_send_syst_req(chip, target, MAX2035X_SYS_REQ_HRST);
	}
	return ret ? ret : cnt;
}

/* echo "target" > reset/seal */
static ssize_t dbg_max2035x_request_seal_write(struct file *f, const char __user *u,
			       size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &target))
		return -EINVAL;

	if (chip->type == MAX20357)
		ret = regmap_update_bits(chip->regmap, MAX20357_REG_SYSTEM_REG0,
					 MAX20357_SYSTEM_SEAL_I2C_CMD_BIT,
					 MAX20357_SYSTEM_SEAL_I2C_CMD_BIT);
	else {
		if (!dbg_max2035x_check_cmd_ongoing(chip, target))
			return -EBUSY;
		ret = dbg_max2035x_send_syst_req(chip, target, MAX2035X_SYS_REQ_SEAL);
		if (!ret) {
			u8 ena = (target == 1) ? MAX20355_PLC_CFG2_PL1_CHN_ENA_BIT :
						  MAX20355_PLC_CFG2_PL2_CHN_ENA_BIT;
			regmap_update_bits(chip->regmap, MAX20355_REG_PLC_CONFIG2, ena, 0);
		}
	}
	return ret ? ret : cnt;
}

/* echo "target" > reset/fg */
static ssize_t dbg_max2035x_fuelgauge_reset_write(struct file *f, const char __user *u,
				   size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &target))
		return -EINVAL;

	if (chip->type == MAX20357)
		target = 0;
	if (!dbg_max2035x_check_cmd_ongoing(chip, target))
		return -EBUSY;
	ret = dbg_max2035x_send_syst_req(chip, target, MAX2035X_SYS_REQ_FGRST);
	return ret ? ret : cnt;
}

/* -------------------------------------------------------------------------- */
/* PLC command debugfs files                                                  */
/* -------------------------------------------------------------------------- */

/* echo "target arg_hex" > plc/syst_req */
static ssize_t dbg_max2035x_syst_req_write(struct file *f, const char __user *u,
				   size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[32];
	int target, arg, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (sscanf(buf, "%d 0x%x", &target, &arg) != 2 &&
	    sscanf(buf, "%d %d", &target, &arg) != 2)
		return -EINVAL;

	if (chip->type == MAX20357)
		target = 0;
	ret = dbg_max2035x_send_syst_req(chip, target, (u8)arg);
	return ret ? ret : cnt;
}

/* echo "target" > plc/idle_mode */
static ssize_t dbg_max2035x_request_idle_mode_write(struct file *f, const char __user *u,
				    size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &target))
		return -EINVAL;

	if (chip->type == MAX20357)
		target = 0;
	if (!dbg_max2035x_check_cmd_ongoing(chip, target))
		return -EBUSY;
	ret = dbg_max2035x_send_syst_req(chip, target, MAX2035X_SYS_REQ_IDLE);
	return ret ? ret : cnt;
}

/* echo "target" > plc/resume_idle */
static ssize_t dbg_max2035x_resume_idle_mode_write(struct file *f, const char __user *u,
				      size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, ret;
	unsigned int reg_cfg, reg_val;
	u8 cfg_bit;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &target))
		return -EINVAL;

	if (chip->type == MAX20355) {
		reg_cfg = MAX20355_REG_PLC_CONFIG2;
		cfg_bit = (target == 1) ? MAX20355_PLC_CFG2_PL1_RES_REQ_BIT :
					  MAX20355_PLC_CFG2_PL2_RES_REQ_BIT;
	} else {
		reg_cfg = MAX20357_REG_PLC_CONFIG4;
		cfg_bit = MAX20357_PLC_CFG4_PLC_RES_REQ_BIT;
	}

	ret = regmap_read(chip->regmap, reg_cfg, &reg_val);
	if (ret)
		return ret;
	reg_val |= cfg_bit;
	ret = regmap_write(chip->regmap, reg_cfg, reg_val);
	return ret ? ret : cnt;
}

/* echo > plc/off_mode */
static ssize_t dbg_max2035x_request_off_mode_write(struct file *f, const char __user *u,
				   size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	unsigned int reg;
	u32 bit;

	if (chip->type == MAX20355) {
		reg = MAX20355_REG_SYSTEM_REG0;
		bit = MAX20355_SYSTEM_OFF_CMD_INP_BIT;
	} else {
		reg = MAX20357_REG_SYSTEM_REG0;
		bit = MAX20357_SYSTEM_OFF_CMD_INP_BIT;
	}
	regmap_update_bits(chip->regmap, reg, bit, bit);
	return cnt;
}

/* cat plc/cmd_status */
static int dbg_max2035x_cmd_status_show(struct seq_file *s, void *data)
{
	struct max2035x_plc *plc = s->private;
	struct max2035x *chip = plc->chip;

	if (chip->type == MAX20355) {
		seq_printf(s, "ch1: %s\nch2: %s\n",
			   dbg_max2035x_check_cmd_ongoing(chip, 1) ? "idle" : "busy",
			   dbg_max2035x_check_cmd_ongoing(chip, 2) ? "idle" : "busy");
	} else {
		seq_printf(s, "ch0: %s\n",
			   dbg_max2035x_check_cmd_ongoing(chip, 0) ? "idle" : "busy");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dbg_max2035x_cmd_status);

/* -------------------------------------------------------------------------- */
/* Transfer debugfs files                                                     */
/* -------------------------------------------------------------------------- */

/* echo "target num_bytes" > transfer/dout_req */
static ssize_t dbg_max2035x_dout_req_write(struct file *f, const char __user *u,
				   size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[32];
	int target, nbytes, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (sscanf(buf, "%d %d", &target, &nbytes) != 2)
		return -EINVAL;

	if (chip->type == MAX20357)
		target = 0;
	ret = dbg_max2035x_send_dout_req(chip, target, (u8)nbytes);
	return ret ? ret : cnt;
}

/* echo "target hex_data..." > transfer/mailbox */
static ssize_t dbg_max2035x_transfer_mailbox_write(struct file *f, const char __user *u,
				  size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[512];
	u8 data[MAX2035X_RAM_SIZE];
	int target, n = 0, ret;
	char *ptr;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;

	ret = sscanf(buf, "%d", &target);
	if (ret != 1)
		return -EINVAL;

	ptr = strchr(buf, ' ');
	if (!ptr)
		return -EINVAL;
	ptr++;

	while (*ptr && n < MAX2035X_RAM_SIZE) {
		unsigned int byte;

		while (*ptr == ' ')
			ptr++;
		if (!*ptr)
			break;
		if (sscanf(ptr, "%x", &byte) != 1)
			break;
		data[n++] = (u8)byte;
		while (*ptr && *ptr != ' ')
			ptr++;
	}
	if (n == 0)
		return -EINVAL;

	if (chip->type == MAX20357)
		target = 0;
	if (!dbg_max2035x_check_cmd_ongoing(chip, target))
		return -EBUSY;

	ret = dbg_max2035x_write_ram_data(plc, data, n);
	if (ret)
		return ret;
	ret = dbg_max2035x_send_dout_req(chip, target, (u8)n);
	return ret ? ret : cnt;
}

/* echo "target" > transfer/fifo_lock */
static ssize_t dbg_max2035x_request_fifo_write(struct file *f, const char __user *u,
				    size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &target))
		return -EINVAL;
	if (chip->type == MAX20357)
		target = 0;
	ret = dbg_max2035x_send_syst_req(chip, target, MAX2035X_SYS_REQ_FIFO);
	return ret ? ret : cnt;
}

/* echo "target" > transfer/fifo_unlock */
static ssize_t dbg_max2035x_request_free_write(struct file *f, const char __user *u,
				      size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[16];
	int target, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (kstrtoint(buf, 0, &target))
		return -EINVAL;
	if (chip->type == MAX20357)
		target = 0;
	ret = dbg_max2035x_send_syst_req(chip, target, MAX2035X_SYS_REQ_FREE);
	return ret ? ret : cnt;
}

/* -------------------------------------------------------------------------- */
/* FIFO status (read)                                                         */
/* -------------------------------------------------------------------------- */
static int dbg_max2035x_fifo_status_show(struct seq_file *s, void *data)
{
	struct max2035x_plc *plc = s->private;
	struct max2035x *chip = plc->chip;
	unsigned int val;

	if (chip->type == MAX20355) {
		if (!regmap_read(chip->regmap, MAX20355_REG_PLC_FIFO, &val))
			seq_printf(s, "pl1_master=%d pl1_slave=%d pl2_master=%d pl2_slave=%d (0x%02x)\n",
				   !!(val & MAX20355_PLC_FIFO_PL1_MASTER_BIT),
				   !!(val & MAX20355_PLC_FIFO_PL1_SLAVE_BIT),
				   !!(val & MAX20355_PLC_FIFO_PL2_MASTER_BIT),
				   !!(val & MAX20355_PLC_FIFO_PL2_SLAVE_BIT), val);
	} else {
		if (!regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &val))
			seq_printf(s, "fifo_master=%d fifo_slave=%d (0x%02x)\n",
				   !!(val & MAX20357_PLC_CFG4_FIFO_MASTER_BIT),
				   !!(val & MAX20357_PLC_CFG4_FIFO_SLAVE_BIT), val);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dbg_max2035x_fifo_status);

/* -------------------------------------------------------------------------- */
/* GPIO PLC debugfs files                                                     */
/* -------------------------------------------------------------------------- */

/* echo "target gpio dir(0=out,1=in) val(0|1)" > gpio/set_plc */
static ssize_t dbg_max2035x_set_gpio_plc_write(struct file *f, const char __user *u,
				   size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[32];
	int target, gpio, is_input, is_high, ret;
	u8 reg_gpio, bit_ctr, reg_arg_addr, arg;
	unsigned int val;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (sscanf(buf, "%d %d %d %d", &target, &gpio, &is_input, &is_high) != 4)
		return -EINVAL;
	if (gpio < 1 || gpio > 4)
		return -EINVAL;

	if (!dbg_max2035x_check_cmd_ongoing(chip, target))
		return -EBUSY;

	if (chip->type == MAX20355) {
		reg_gpio = MAX20355_REG_GPIO1;
		bit_ctr = MAX20355_GPIO_PLCCTR_BIT;
		reg_arg_addr = (target == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
	} else {
		reg_gpio = MAX20357_REG_GPIO1;
		bit_ctr = MAX20357_GPIO_PLCCTR_BIT;
		reg_arg_addr = MAX20357_REG_PLC_ARG;
	}

	regmap_update_bits(chip->regmap, reg_gpio + (gpio - 1), bit_ctr, bit_ctr);

	ret = regmap_read(chip->regmap, reg_arg_addr, &val);
	if (ret)
		return ret;
	arg = (u8)val;

	if (is_input) {
		arg |= BIT(gpio + 3);
		arg &= ~BIT(gpio - 1);
	} else {
		arg &= ~BIT(gpio + 3);
		if (is_high)
			arg &= ~BIT(gpio - 1);
		else
			arg |= BIT(gpio - 1);
	}

	ret = dbg_max2035x_set_gpio(chip, target, arg);
	return ret ? ret : cnt;
}

/* echo "target arg_hex" > gpio/set_cmd */
static ssize_t dbg_max2035x_set_gpio_cmd_write(struct file *f, const char __user *u,
				   size_t cnt, loff_t *p)
{
	struct max2035x_plc *plc = f->private_data;
	struct max2035x *chip = plc->chip;
	char buf[32];
	int target, arg, ret;

	if (dbg_max2035x_parse_buf(u, cnt, buf, sizeof(buf)))
		return -EFAULT;
	if (sscanf(buf, "%d 0x%x", &target, &arg) != 2 &&
	    sscanf(buf, "%d %d", &target, &arg) != 2)
		return -EINVAL;

	ret = dbg_max2035x_set_gpio(chip, target, (u8)arg);
	return ret ? ret : cnt;
}

/* -------------------------------------------------------------------------- */
/* Status debugfs files                                                       */
/* -------------------------------------------------------------------------- */

/* cat status/charger_done (MAX20355 only) */
static int dbg_max2035x_read_slave_charger_done_show(struct seq_file *s, void *data)
{
	struct max2035x_plc *plc = s->private;
	struct max2035x *chip = plc->chip;
	unsigned int val;

	if (chip->type != MAX20355) {
		seq_puts(s, "not supported on MAX20357\n");
		return 0;
	}

	if (regmap_read(chip->regmap, MAX20355_REG_FG_RDY_5, &val))
		return -EIO;

	seq_printf(s, "ch1=%s ch2=%s\n",
		   (val & MAX20355_FG_RDY_SLV1_CHG_DNE_BIT) ? "done" : "not_done",
		   (val & MAX20355_FG_RDY_SLV2_CHG_DNE_BIT) ? "done" : "not_done");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dbg_max2035x_read_slave_charger_done);

/* cat status/ram_clear */
static int dbg_max2035x_is_ram_clear_show(struct seq_file *s, void *data)
{
	struct max2035x_plc *plc = s->private;
	struct max2035x *chip = plc->chip;
	unsigned int reg_cfg, val;
	u8 ram_full_bit;

	if (chip->type == MAX20355) {
		reg_cfg = MAX20355_REG_PLC_CONFIG5;
		ram_full_bit = MAX20355_PLC_CFG5_RAM_IS_FULL_BIT;
	} else {
		reg_cfg = MAX20357_REG_PLC_CONFIG4;
		ram_full_bit = MAX20357_PLC_CFG4_RAM_IS_FULL_BIT;
	}

	if (regmap_read(chip->regmap, reg_cfg, &val))
		return -EIO;

	seq_printf(s, "ram_clear=%s (0x%02x)\n",
		   (val & ram_full_bit) ? "no" : "yes", val);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dbg_max2035x_is_ram_clear);

/* -------------------------------------------------------------------------- */
/* File operations boilerplate                                                */
/* -------------------------------------------------------------------------- */
#define DBG_WRITE_FOPS(_name)						\
static int _name##_open(struct inode *inode, struct file *file)		\
{									\
	file->private_data = inode->i_private;				\
	return 0;							\
}									\
static const struct file_operations _name##_fops = {			\
	.owner = THIS_MODULE,						\
	.open = _name##_open,						\
	.write = _name##_write,						\
	.llseek = noop_llseek,						\
}

DBG_WRITE_FOPS(dbg_max2035x_enter_uart_auto);
DBG_WRITE_FOPS(dbg_max2035x_enter_uart_tx);
DBG_WRITE_FOPS(dbg_max2035x_enter_uart_rx);
DBG_WRITE_FOPS(dbg_max2035x_exit_uart);
DBG_WRITE_FOPS(dbg_max2035x_set_uart_control_mode);
DBG_WRITE_FOPS(dbg_max2035x_soft_reset);
DBG_WRITE_FOPS(dbg_max2035x_hard_reset);
DBG_WRITE_FOPS(dbg_max2035x_request_seal);
DBG_WRITE_FOPS(dbg_max2035x_fuelgauge_reset);
DBG_WRITE_FOPS(dbg_max2035x_syst_req);
DBG_WRITE_FOPS(dbg_max2035x_request_idle_mode);
DBG_WRITE_FOPS(dbg_max2035x_resume_idle_mode);
DBG_WRITE_FOPS(dbg_max2035x_request_off_mode);
DBG_WRITE_FOPS(dbg_max2035x_dout_req);
DBG_WRITE_FOPS(dbg_max2035x_transfer_mailbox);
DBG_WRITE_FOPS(dbg_max2035x_request_fifo);
DBG_WRITE_FOPS(dbg_max2035x_request_free);
DBG_WRITE_FOPS(dbg_max2035x_set_gpio_plc);
DBG_WRITE_FOPS(dbg_max2035x_set_gpio_cmd);

/* -------------------------------------------------------------------------- */
/* Init / cleanup                                                             */
/* -------------------------------------------------------------------------- */
void max2035x_debugfs_init(struct max2035x_plc *plc)
{
	struct dentry *root, *d;

	root = debugfs_create_dir(dev_name(plc->dev), NULL);
	plc->debugfs_root = root;

	d = debugfs_create_dir("uart", root);
	debugfs_create_file("enter_auto", 0200, d, plc, &dbg_max2035x_enter_uart_auto_fops);
	debugfs_create_file("enter_tx", 0200, d, plc, &dbg_max2035x_enter_uart_tx_fops);
	debugfs_create_file("enter_rx", 0200, d, plc, &dbg_max2035x_enter_uart_rx_fops);
	debugfs_create_file("exit", 0200, d, plc, &dbg_max2035x_exit_uart_fops);
	debugfs_create_file("control_mode", 0200, d, plc, &dbg_max2035x_set_uart_control_mode_fops);

	d = debugfs_create_dir("reset", root);
	debugfs_create_file("soft", 0200, d, plc, &dbg_max2035x_soft_reset_fops);
	debugfs_create_file("hard", 0200, d, plc, &dbg_max2035x_hard_reset_fops);
	debugfs_create_file("seal", 0200, d, plc, &dbg_max2035x_request_seal_fops);
	debugfs_create_file("fg", 0200, d, plc, &dbg_max2035x_fuelgauge_reset_fops);

	d = debugfs_create_dir("plc", root);
	debugfs_create_file("syst_req", 0200, d, plc, &dbg_max2035x_syst_req_fops);
	debugfs_create_file("idle_mode", 0200, d, plc, &dbg_max2035x_request_idle_mode_fops);
	debugfs_create_file("resume_idle", 0200, d, plc, &dbg_max2035x_resume_idle_mode_fops);
	debugfs_create_file("off_mode", 0200, d, plc, &dbg_max2035x_request_off_mode_fops);
	debugfs_create_file("cmd_status", 0444, d, plc, &dbg_max2035x_cmd_status_fops);

	d = debugfs_create_dir("transfer", root);
	debugfs_create_file("dout_req", 0200, d, plc, &dbg_max2035x_dout_req_fops);
	debugfs_create_file("mailbox", 0200, d, plc, &dbg_max2035x_transfer_mailbox_fops);
	debugfs_create_file("fifo_lock", 0200, d, plc, &dbg_max2035x_request_fifo_fops);
	debugfs_create_file("fifo_unlock", 0200, d, plc, &dbg_max2035x_request_free_fops);

	d = debugfs_create_dir("fifo", root);
	debugfs_create_file("status", 0444, d, plc, &dbg_max2035x_fifo_status_fops);

	d = debugfs_create_dir("gpio", root);
	debugfs_create_file("set_plc", 0200, d, plc, &dbg_max2035x_set_gpio_plc_fops);
	debugfs_create_file("set_cmd", 0200, d, plc, &dbg_max2035x_set_gpio_cmd_fops);

	d = debugfs_create_dir("status", root);
	debugfs_create_file("charger_done", 0444, d, plc, &dbg_max2035x_read_slave_charger_done_fops);
	debugfs_create_file("ram_clear", 0444, d, plc, &dbg_max2035x_is_ram_clear_fops);
}

void max2035x_debugfs_exit(struct max2035x_plc *plc)
{
	debugfs_remove_recursive(plc->debugfs_root);
}
