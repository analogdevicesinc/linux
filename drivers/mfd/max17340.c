// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices MAX17340 MFD Core Driver
 *
 * Copyright (C) 2024-2026 Analog Devices, Inc.
 *
 * The MAX17340 is an integrated single-cell battery management IC combining
 * ModelGauge m5 fuel gauge with charger and protection circuits. It uses two
 * I2C addresses: 0x36 for FG_RAM (runtime data) and 0x0B for FG_NVM
 * (configuration and model data).
 */

#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max17340.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define MAX17340_MBOX_TIMEOUT_MS	5000
#define MAX17340_MBOX_MAX_ARGS		32
#define MAX17340_MBOX_MAX_RESP		32

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
/*
 * Number of response data bytes that follow the result byte (AP_DATAIN0) for
 * the general mailbox commands. The FW writes AP_DATAIN_M1 (0x50) only for
 * the security commands (MAX17340_MBOX_CMD_IS_ECDSA()), so for every other
 * opcode the response length is fixed. Opcodes not listed return only the
 * result byte.
 */
static unsigned int max17340_mbox_resp_data_len(u8 opcode)
{
	switch (opcode) {
	case MAX17340_MBOX_CMD_GET_FG_STATUS:
	case MAX17340_MBOX_CMD_GET_FG_MASK:
		return 8;
	case MAX17340_MBOX_CMD_GET_PAGE_WR_CNT:
	case MAX17340_MBOX_CMD_RISCV_REG_READ:
		return 4;
	case MAX17340_MBOX_CMD_RECALL_LIFE_LOG:
	case MAX17340_MBOX_CMD_BATTVIMON_READ_SAMPLES:
	case MAX17340_MBOX_CMD_BATTVIMON_READ_LATEST:
	case MAX17340_MBOX_CMD_BATTVIMON_READ_OLDEST:
		return 32;
	default:
		return 0;
	}
}

/*
 * Get the response length (result byte + data bytes) of the last command.
 * Security commands report it in AP_DATAIN_M1 (0x50); it is not updated by
 * the general commands, so those use the fixed per-opcode length.
 *
 * Returns 0 on success, -EPROTO if the FW reports an invalid length, or the
 * regmap error if AP_DATAIN_M1 cannot be read.
 */
static int max17340_mbox_resp_len(struct max17340_dev *mfd, u8 opcode,
				  unsigned int *resp_len)
{
	int ret;

	if (!MAX17340_MBOX_CMD_IS_ECDSA(opcode)) {
		*resp_len = 1 + max17340_mbox_resp_data_len(opcode);
		return 0;
	}

	ret = regmap_read(mfd->regmap_func, MAX17340_MBOX_RISCV_RESP_LEN,
			  resp_len);
	if (ret)
		return ret;

	if (*resp_len == 0 || *resp_len > 1 + MAX17340_MBOX_MAX_RESP) {
		dev_err(mfd->dev, "Invalid response length 0x%02x\n", *resp_len);
		return -EPROTO;
	}

	return 0;
}
#endif /* CONFIG_MFD_MAX17340_MAILBOX */

static const struct mfd_cell max17340_cells[] = {
	{ .name = "max17340-battery" },
	{ .name = "max17340-charger" },
	{ .name = "max17340-gpio" },
};

static const struct regmap_config max17340_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.max_register = 0xFF,
};

static const struct regmap_config max17340_func_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int max17340_wait_dnr_clear(struct max17340_dev *mfd)
{
	unsigned int fstat;
	int timeout = 50;
	int ret;

	do {
		ret = regmap_read(mfd->regmap, MAX17340_FSTAT, &fstat);
		if (ret)
			return ret;

		if (!(fstat & MAX17340_FSTAT_DNR))
			return 0;

		usleep_range(10000, 15000);
	} while (--timeout);

	return -ETIMEDOUT;
}

static int max17340_init_device(struct max17340_dev *mfd)
{
	unsigned int status, config;
	int ret;

	ret = max17340_wait_dnr_clear(mfd);
	if (ret)
		return ret;

	ret = regmap_read(mfd->regmap, MAX17340_STATUS, &status);
	if (ret)
		return ret;

	if (status & MAX17340_STATUS_PONR) {
		dev_info(mfd->dev, "POR detected, initializing device\n");
		msleep(MAX17340_MODEL_DELAY_MS);
		regmap_update_bits(mfd->regmap, MAX17340_STATUS,
				   MAX17340_STATUS_PONR, 0);
	}

	ret = regmap_read(mfd->regmap, MAX17340_CONFIG, &config);
	if (ret)
		return ret;

	config |= MAX17340_CONFIG_AEN | MAX17340_CONFIG_TEN | MAX17340_CONFIG_SS;
	return regmap_write(mfd->regmap, MAX17340_CONFIG, config);
}

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
static int max17340_init_mailbox_irq(struct max17340_dev *mfd)
{
	unsigned int mbox_int[5];
	int ret;

	/* Enable AP_INT_01 (CMD_DONE) and AP_INT_02 (ECDSA), mask others
	 * Mask convention: 1 = masked (disabled), 0 = enabled
	 */
	ret = regmap_write(mfd->regmap_func, MAX17340_MBOX_INT_M_0,
			   ~(MAX17340_MBOX_INT0_CMD_DONE | MAX17340_MBOX_INT0_ECDSA) & 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(mfd->regmap_func, MAX17340_MBOX_INT_M_1, 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(mfd->regmap_func, MAX17340_MBOX_INT_M_2, 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(mfd->regmap_func, MAX17340_MBOX_INT_M_3, 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(mfd->regmap_func, MAX17340_MBOX_INT_M_4, 0xFF);
	if (ret)
		return ret;

	/* Read MBOX_INT registers to clear pending interrupts */
	regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_0, &mbox_int[0]);
	regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_1, &mbox_int[1]);
	regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_2, &mbox_int[2]);
	regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_3, &mbox_int[3]);
	regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_4, &mbox_int[4]);

	return 0;
}
#endif

static const char * const max17340_status_names[16] = {
	[1] = "PONR", [3] = "Bst", [5] = "AllowChgB", [7] = "dSOCi",
	[8] = "Vmn", [9] = "Tmn", [10] = "Smn", [11] = "CA",
	[12] = "Vmx", [13] = "Tmx", [14] = "Smx", [15] = "PA",
};

static const char * const max17340_protalrt_names[16] = {
	[1] = "BlockDis/TooColdD", [2] = "ODCP", [3] = "UVP", [4] = "TooHotD",
	[5] = "DieHot", [6] = "TempRegionChange", [7] = "BlockChg",
	[8] = "RiscWDi", [9] = "QOvflw", [10] = "OCCP", [11] = "OVP",
	[12] = "TooColdC", [13] = "Full", [14] = "TooHotC",
	[15] = "ChgWDT/LDet",
};

static void max17340_log_bits(struct max17340_dev *mfd, const char *reg,
			      const char * const *names, unsigned int val,
			      bool warn)
{
	int i;

	for (i = 0; i < 16; i++) {
		if (!(val & BIT(i)) || !names[i])
			continue;
		if (warn)
			dev_warn(mfd->dev, "%s: %s\n", reg, names[i]);
		else
			dev_info(mfd->dev, "%s: %s\n", reg, names[i]);
	}
}

static irqreturn_t max17340_irq_handler(int irq, void *data)
{
	struct max17340_dev *mfd = data;
	unsigned int status, protalrt;
	bool notify_battery = false;

	/* Read FG/Protection status */
	if (regmap_read(mfd->regmap, MAX17340_STATUS, &status))
		return IRQ_HANDLED;

	if (regmap_read(mfd->regmap, MAX17340_PROTALRT, &protalrt))
		return IRQ_HANDLED;

	dev_dbg(mfd->dev, "irq: STATUS=0x%04x PROTALRT=0x%04x\n", status, protalrt);

	/* Report which events caused the interrupt (protection faults are warnings) */
	max17340_log_bits(mfd, "STATUS", max17340_status_names, status, false);
	max17340_log_bits(mfd, "PROTALRT", max17340_protalrt_names, protalrt, true);

	/* FG/Protection event handling */
	if (status & MAX17340_STATUS_ALERT_MASK)
		notify_battery = true;

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
	/* Read all MBOX_INT registers to clear pending interrupts */
	{
		unsigned int mbox_int[5];

		regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_0, &mbox_int[0]);
		regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_1, &mbox_int[1]);
		regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_2, &mbox_int[2]);
		regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_3, &mbox_int[3]);
		regmap_read(mfd->regmap_func, MAX17340_MBOX_INT_4, &mbox_int[4]);

		dev_dbg(mfd->dev, "irq: MBOX_INT=%02x %02x %02x %02x %02x\n",
			mbox_int[0], mbox_int[1], mbox_int[2], mbox_int[3], mbox_int[4]);

		/* Mailbox command completion (AP_INT_01 or AP_INT_02 for ECDSA) */
		if (mbox_int[0] & (MAX17340_MBOX_INT0_CMD_DONE | MAX17340_MBOX_INT0_ECDSA))
			complete(&mfd->mailbox_done);
	}
#endif

	/* Clear FG status (lower byte only, upper byte is read-only alerts) */
	regmap_write(mfd->regmap, MAX17340_STATUS, status & 0x00FF);

	/* Clear protection alert (must be cleared before STATUS.PA) */
	regmap_write(mfd->regmap, MAX17340_PROTALRT, 0);

	/* Notify userspace of battery state change */
	if (notify_battery && mfd->battery)
		power_supply_changed(mfd->battery);
	if (notify_battery && mfd->charger)
		power_supply_changed(mfd->charger);

	return IRQ_HANDLED;
}

static void max17340_unregister_i2c_client(void *data)
{
	i2c_unregister_device(data);
}

#define MAX17340_FW_VERSION_STR_LEN	128

static int max17340_get_fw_version_str(struct max17340_dev *mfd, char *str,
				       size_t size)
{
	unsigned int dev_id, dev_rev, fw_rev, fw_sub;
	int ret;

	ret = regmap_read(mfd->regmap_func, MAX17340_MBOX_DEVICE_ID, &dev_id);
	if (ret)
		return ret;

	ret = regmap_read(mfd->regmap_func, MAX17340_MBOX_DEVICE_REV, &dev_rev);
	if (ret)
		return ret;

	ret = regmap_read(mfd->regmap_func, MAX17340_MBOX_FW_REV, &fw_rev);
	if (ret)
		return ret;

	ret = regmap_read(mfd->regmap_func, MAX17340_MBOX_FW_SUB_REV, &fw_sub);
	if (ret)
		return ret;

	snprintf(str, size,
		 "Device: 0x%02x Rev: 0x%02x FW_REV: 0x%02X FW_SUB: 0x%02X FW:%d.%d.%d (Quality:%d)",
		 dev_id, dev_rev, fw_rev, fw_sub,
		 (fw_rev & 0xF0) >> 4, fw_rev & 0x0F,
		 (fw_sub & 0xF8) >> 3, fw_sub & 0x07);

	return 0;
}

static int max17340_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct i2c_client *nvm_client, *func_client;
	struct max17340_dev *mfd;
	unsigned int devname;
	char fw_str[MAX17340_FW_VERSION_STR_LEN];
	int ret;

	mfd = devm_kzalloc(dev, sizeof(*mfd), GFP_KERNEL);
	if (!mfd)
		return -ENOMEM;

	mfd->dev = dev;
	mfd->irq = client->irq;
	i2c_set_clientdata(client, mfd);

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
	init_completion(&mfd->mailbox_done);
	mutex_init(&mfd->mailbox_lock);
#endif

	/* FG_RAM regmap (0x36, 16-bit) */
	mfd->regmap = devm_regmap_init_i2c(client, &max17340_regmap_cfg);
	if (IS_ERR(mfd->regmap))
		return dev_err_probe(dev, PTR_ERR(mfd->regmap),
				     "Failed to init FG_RAM regmap\n");

	/* FG_NVM client and regmap (0x0B, 16-bit) */
	nvm_client = i2c_new_dummy_device(client->adapter, MAX17340_ADDR_FG_NVM);
	if (IS_ERR(nvm_client))
		return dev_err_probe(dev, PTR_ERR(nvm_client),
				     "Failed to create NVM client\n");

	ret = devm_add_action_or_reset(dev, max17340_unregister_i2c_client,
				       nvm_client);
	if (ret)
		return ret;

	mfd->regmap_nvm = devm_regmap_init_i2c(nvm_client, &max17340_regmap_cfg);
	if (IS_ERR(mfd->regmap_nvm))
		return dev_err_probe(dev, PTR_ERR(mfd->regmap_nvm),
				     "Failed to init FG_NVM regmap\n");

	/* FUNC client and regmap (0x37, 8-bit) */
	func_client = i2c_new_dummy_device(client->adapter, MAX17340_ADDR_FUNC);
	if (IS_ERR(func_client))
		return dev_err_probe(dev, PTR_ERR(func_client),
				     "Failed to create FUNC client\n");

	ret = devm_add_action_or_reset(dev, max17340_unregister_i2c_client,
				       func_client);
	if (ret)
		return ret;

	mfd->regmap_func = devm_regmap_init_i2c(func_client,
						&max17340_func_regmap_cfg);
	if (IS_ERR(mfd->regmap_func))
		return dev_err_probe(dev, PTR_ERR(mfd->regmap_func),
				     "Failed to init FUNC regmap\n");

	ret = regmap_read(mfd->regmap, MAX17340_DEVNAME, &devname);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read device ID\n");

	dev_info(dev, "MAX17340 detected, DevName=0x%04X\n", devname);

	ret = max17340_get_fw_version_str(mfd, fw_str, sizeof(fw_str));
	if (ret)
		dev_warn(dev, "Failed to read FW version: %d\n", ret);
	else
		dev_info(dev, "%s\n", fw_str);

	ret = device_property_read_u32(dev, "adi,rsense-microohm", &mfd->rsense);
	if (ret) {
		unsigned int rsense_raw;

		ret = regmap_read(mfd->regmap_nvm, MAX17340_NVM_RSENSE,
				  &rsense_raw);
		if (!ret && rsense_raw)
			mfd->rsense = rsense_raw * 10;
		else
			mfd->rsense = MAX17340_DEFAULT_RSENSE_UOHM;
	}

	dev_info(dev, "RSense = %u uOhm\n", mfd->rsense);

	ret = max17340_init_device(mfd);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize device\n");

	/* Unlock write protection for runtime register access */
	ret = max17340_unlock_write(mfd);
	if (ret)
		dev_warn(dev, "Failed to unlock write protection: %d\n", ret);

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
	ret = max17340_init_mailbox_irq(mfd);
	if (ret)
		dev_warn(dev, "Failed to initialize mailbox IRQ: %d\n", ret);
#endif

	if (mfd->irq) {
		ret = devm_request_threaded_irq(dev, mfd->irq, NULL,
						max17340_irq_handler,
						IRQF_ONESHOT,
						"max17340-alert", mfd);
		if (ret)
			dev_warn(dev, "Failed to request IRQ: %d\n", ret);
	}

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, max17340_cells,
				    ARRAY_SIZE(max17340_cells), NULL, 0, NULL);
}

/* ========== Sysfs Interface ========== */

static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct max17340_dev *mfd = dev_get_drvdata(dev);
	char str[MAX17340_FW_VERSION_STR_LEN];
	int ret;

	ret = max17340_get_fw_version_str(mfd, str, sizeof(str));
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n", str);
}
static DEVICE_ATTR_RO(fw_version);

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
static ssize_t mailbox_cmd_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct max17340_dev *mfd = dev_get_drvdata(dev);
	u8 opcode, args[MAX17340_MBOX_MAX_ARGS];
	int num_args = 0;
	char *input, *token, *cur;
	unsigned long val;
	int ret;

	input = kstrdup(buf, GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	cur = input;

	/* Parse opcode (first token) */
	token = strsep(&cur, " \t\n");
	if (!token || !*token) {
		ret = -EINVAL;
		goto out;
	}

	ret = kstrtoul(token, 0, &val);
	if (ret || val > 0xFF) {
		ret = -EINVAL;
		goto out;
	}
	opcode = val;

	/* Parse data arguments */
	while ((token = strsep(&cur, " \t\n")) != NULL) {
		if (!*token)
			continue;

		if (num_args >= MAX17340_MBOX_MAX_ARGS) {
			ret = -EINVAL;
			goto out;
		}

		ret = kstrtoul(token, 0, &val);
		if (ret || val > 0xFF) {
			ret = -EINVAL;
			goto out;
		}
		args[num_args++] = val;
	}

	ret = max17340_mailbox_cmd(mfd, opcode, args, num_args, NULL, 0);
	if (ret)
		goto out;

	ret = count;

out:
	kfree(input);
	return ret;
}

static ssize_t mailbox_cmd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max17340_dev *mfd = dev_get_drvdata(dev);
	u8 resp_buf[1 + MAX17340_MBOX_MAX_ARGS];
	unsigned int resp_len;
	u8 resp_opcode;
	u8 *resp_data;
	int ret, i, len = 0;

	/* Do not read the response while a command is in flight */
	guard(mutex)(&mfd->mailbox_lock);

	/* Response length depends on the last issued opcode */
	ret = max17340_mbox_resp_len(mfd, mfd->pending_opcode, &resp_len);
	if (ret == -EPROTO) {
		len += sysfs_emit_at(buf, len, "Invalid response length\n");
		return len;
	}
	if (ret)
		return ret;

	len += sysfs_emit_at(buf, len, "last opcode: 0x%02x\n", mfd->pending_opcode);
	len += sysfs_emit_at(buf, len, "resp_len: %u (result + %u data bytes)\n",
			     resp_len, resp_len - 1);

	/* Read response opcode + data (only resp_len bytes) */
	ret = regmap_bulk_read(mfd->regmap_func, MAX17340_MBOX_RISCV_RESP_OPCODE,
			       resp_buf, resp_len);
	if (ret)
		return ret;

	resp_opcode = resp_buf[0];
	resp_data = &resp_buf[1];

	len += sysfs_emit_at(buf, len, "resp_opcode: 0x%02x", resp_opcode);

	switch (resp_opcode) {
	case MAX17340_RESP_SUCCESS:
		len += sysfs_emit_at(buf, len, " (SUCCESS)\n");
		break;
	case MAX17340_RESP_NOT_IMPLEMENTED:
		len += sysfs_emit_at(buf, len, " (NOT_IMPLEMENTED)\n");
		break;
	case MAX17340_RESP_FAILURE:
		len += sysfs_emit_at(buf, len, " (FAILURE)\n");
		break;
	default:
		len += sysfs_emit_at(buf, len, "\n");
		break;
	}

	/* Display response data (only valid bytes based on resp_len) */
	{
		int data_len = resp_len - 1;  /* exclude opcode */

		len += sysfs_emit_at(buf, len, "resp[0-7]:   ");
		for (i = 0; i < 8; i++) {
			if (i < data_len)
				len += sysfs_emit_at(buf, len, "%02x ", resp_data[i]);
			else
				len += sysfs_emit_at(buf, len, "-- ");
		}
		len += sysfs_emit_at(buf, len, "\n");

		len += sysfs_emit_at(buf, len, "resp[8-15]:  ");
		for (i = 8; i < 16; i++) {
			if (i < data_len)
				len += sysfs_emit_at(buf, len, "%02x ", resp_data[i]);
			else
				len += sysfs_emit_at(buf, len, "-- ");
		}
		len += sysfs_emit_at(buf, len, "\n");

		len += sysfs_emit_at(buf, len, "resp[16-23]: ");
		for (i = 16; i < 24; i++) {
			if (i < data_len)
				len += sysfs_emit_at(buf, len, "%02x ", resp_data[i]);
			else
				len += sysfs_emit_at(buf, len, "-- ");
		}
		len += sysfs_emit_at(buf, len, "\n");

		len += sysfs_emit_at(buf, len, "resp[24-31]: ");
		for (i = 24; i < 32; i++) {
			if (i < data_len)
				len += sysfs_emit_at(buf, len, "%02x ", resp_data[i]);
			else
				len += sysfs_emit_at(buf, len, "-- ");
		}
		len += sysfs_emit_at(buf, len, "\n");
	}

	return len;
}
static DEVICE_ATTR_RW(mailbox_cmd);
#endif /* CONFIG_MFD_MAX17340_MAILBOX */

static struct attribute *max17340_attrs[] = {
	&dev_attr_fw_version.attr,
#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
	&dev_attr_mailbox_cmd.attr,
#endif
	NULL,
};
ATTRIBUTE_GROUPS(max17340);

static const struct of_device_id max17340_of_match[] = {
	{ .compatible = "adi,max17340" },
	{ }
};
MODULE_DEVICE_TABLE(of, max17340_of_match);

static const struct i2c_device_id max17340_id[] = {
	{ "max17340" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17340_id);

static struct i2c_driver max17340_driver = {
	.driver = {
		.name = "max17340",
		.of_match_table = max17340_of_match,
		.dev_groups = max17340_groups,
	},
	.probe = max17340_probe,
	.id_table = max17340_id,
};
module_i2c_driver(max17340_driver);

/**
 * max17340_unlock_write - Unlock write protection
 * @mfd: MAX17340 device
 *
 * Clears the temporary write protection (WRITE_PROT register).
 * Must write twice in a row without accessing other registers.
 * Note: nLOCKCtrl (permanent lock) has priority over WRITE_PROT.
 *
 * Returns 0 on success, negative errno on failure.
 */
int max17340_unlock_write(struct max17340_dev *mfd)
{
	int ret;

	ret = regmap_write(mfd->regmap_func, MAX17340_WRITE_PROT, 0x00);
	if (ret)
		return ret;
	return regmap_write(mfd->regmap_func, MAX17340_WRITE_PROT, 0x00);
}
EXPORT_SYMBOL_GPL(max17340_unlock_write);

/**
 * max17340_lock_write - Lock write protection
 * @mfd: MAX17340 device
 *
 * Sets all write protection bits (WRITE_PROT register).
 * Must write twice in a row without accessing other registers.
 *
 * Returns 0 on success, negative errno on failure.
 */
int max17340_lock_write(struct max17340_dev *mfd)
{
	int ret;

	ret = regmap_write(mfd->regmap_func, MAX17340_WRITE_PROT, 0xFF);
	if (ret)
		return ret;
	return regmap_write(mfd->regmap_func, MAX17340_WRITE_PROT, 0xFF);
}
EXPORT_SYMBOL_GPL(max17340_lock_write);

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
/*
 * Mailbox API
 *
 * The mailbox provides communication between the host and internal RISCV
 * firmware. Commands are sent by writing to AP_REQ_OPCODE and arguments
 * to AP_REQ_DATA. The RISCV triggers an interrupt (AP_INT_0 bit 0) when
 * the response is ready.
 */

/**
 * max17340_mailbox_cmd - Send a mailbox command and wait for response
 * @mfd: MAX17340 device
 * @opcode: Command opcode
 * @args: Command arguments (may be NULL if num_args is 0)
 * @num_args: Number of argument bytes (max 32)
 * @resp: Response buffer (may be NULL if num_resp is 0)
 * @num_resp: Number of response bytes to read (max 32)
 *
 * Returns 0 on success, negative errno on failure.
 */
int max17340_mailbox_cmd(struct max17340_dev *mfd, u8 opcode,
			 const u8 *args, int num_args,
			 u8 *resp, int num_resp)
{
	u8 cmd_buf[1 + MAX17340_MBOX_MAX_ARGS];
	unsigned int resp_opcode;
	int ret, i;

	if (num_args > MAX17340_MBOX_MAX_ARGS ||
	    num_resp > MAX17340_MBOX_MAX_RESP)
		return -EINVAL;

	/*
	 * The mailbox registers, the completion and pending_opcode are shared:
	 * serialize the whole transaction (request, wait and response read).
	 */
	guard(mutex)(&mfd->mailbox_lock);

	/* Build args buffer (padded to 32 bytes) */
	for (i = 0; i < num_args; i++)
		cmd_buf[i] = args[i];
	for (i = num_args; i < MAX17340_MBOX_MAX_ARGS; i++)
		cmd_buf[i] = 0x00;

	/* Step 1: Write parameters to AP_DATAOUT1~32 (0x22~0x41) */
	ret = regmap_bulk_write(mfd->regmap_func, MAX17340_MBOX_AP_REQ_DATA,
				cmd_buf, MAX17340_MBOX_MAX_ARGS);
	if (ret)
		return ret;

	/* Step 2: Write length to AP_DATAOUT_M1 (0x20): 1 + parameter_count */
	ret = regmap_write(mfd->regmap_func, MAX17340_MBOX_AP_REQ_LEN,
			   1 + num_args);
	if (ret)
		return ret;

	/* Reset completion before sending command */
	reinit_completion(&mfd->mailbox_done);

	/* Step 3: Write opcode to AP_DATAOUT0 (0x21) - triggers mbox_irq */
	ret = regmap_write(mfd->regmap_func, MAX17340_MBOX_AP_REQ_OPCODE, opcode);
	if (ret)
		return ret;

	/* Remember the issued opcode: it selects the response length when reading back */
	mfd->pending_opcode = opcode;

	/* Wait for completion via IRQ handler */
	if (MAX17340_MBOX_CMD_NO_INT(opcode)) {
		/* RAM_CODE_EXEC has no completion interrupt, just delay */
		usleep_range(10000, 20000);
	} else {
		unsigned long timeout;

		timeout = wait_for_completion_timeout(&mfd->mailbox_done,
						      msecs_to_jiffies(MAX17340_MBOX_TIMEOUT_MS));
		if (!timeout) {
			/*
			 * An unsupported opcode returns NOT_IMPLEMENTED in
			 * AP_DATAIN0 (0x51) but does not raise an interrupt.
			 */
			if (!regmap_read(mfd->regmap_func,
					 MAX17340_MBOX_RISCV_RESP_OPCODE,
					 &resp_opcode) &&
			    resp_opcode == MAX17340_RESP_NOT_IMPLEMENTED) {
				dev_err(mfd->dev, "Mailbox opcode 0x%02x not implemented\n",
					opcode);
				return -EOPNOTSUPP;
			}
			dev_err(mfd->dev, "Mailbox timeout for opcode 0x%02x\n", opcode);
			return -ETIMEDOUT;
		}
	}

	{
		unsigned int resp_len;
		u8 resp_buf[1 + MAX17340_MBOX_MAX_RESP];

		ret = max17340_mbox_resp_len(mfd, opcode, &resp_len);
		if (ret)
			return ret;

		/* Read response opcode + data (only resp_len bytes) */
		ret = regmap_bulk_read(mfd->regmap_func, MAX17340_MBOX_RISCV_RESP_OPCODE,
				       resp_buf, resp_len);
		if (ret)
			return ret;

		resp_opcode = resp_buf[0];

		if (resp_opcode == MAX17340_RESP_NOT_IMPLEMENTED) {
			dev_err(mfd->dev, "Mailbox opcode 0x%02x not implemented\n", opcode);
			return -EOPNOTSUPP;
		}

		if (resp_opcode == MAX17340_RESP_FAILURE) {
			dev_err(mfd->dev, "Mailbox opcode 0x%02x failed\n", opcode);
			return -EIO;
		}

		if (MAX17340_MBOX_CMD_IS_ECDSA(opcode)) {
			if (resp_opcode != MAX17340_RESP_ECDSA_SUCCESS) {
				dev_err(mfd->dev, "Mailbox ECDSA opcode 0x%02x failed, response 0x%02x\n",
					opcode, resp_opcode);
				return -EPROTO;
			}
		} else {
			if (resp_opcode != MAX17340_RESP_SUCCESS) {
				dev_err(mfd->dev, "Mailbox opcode 0x%02x unknown response 0x%02x\n",
					opcode, resp_opcode);
				return -EPROTO;
			}
		}

		/* Copy response data (only what was actually received) */
		if (num_resp > (int)(resp_len - 1)) {
			dev_warn(mfd->dev, "Requested %d resp bytes but only %u available\n",
				 num_resp, resp_len - 1);
			num_resp = resp_len - 1;
		}
		for (i = 0; i < num_resp; i++)
			resp[i] = resp_buf[1 + i];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max17340_mailbox_cmd);

/**
 * max17340_mailbox_read_fw_version - Read firmware version
 * @mfd: MAX17340 device
 * @major: Firmware major version
 * @minor: Firmware minor version
 *
 * Returns 0 on success, negative errno on failure.
 */
int max17340_mailbox_read_fw_version(struct max17340_dev *mfd,
				     u8 *major, u8 *minor)
{
	unsigned int val;
	int ret;

	ret = regmap_read(mfd->regmap_func, MAX17340_MBOX_FW_REV, &val);
	if (ret)
		return ret;
	*major = val;

	ret = regmap_read(mfd->regmap_func, MAX17340_MBOX_FW_SUB_REV, &val);
	if (ret)
		return ret;
	*minor = val;

	return 0;
}
EXPORT_SYMBOL_GPL(max17340_mailbox_read_fw_version);
#endif /* CONFIG_MFD_MAX17340_MAILBOX */

MODULE_DESCRIPTION("Analog Devices MAX17340 MFD Core Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joan Na <joan.na@analog.com>");
MODULE_VERSION("0.2.0");
