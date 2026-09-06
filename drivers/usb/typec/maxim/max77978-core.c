// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77978 Core Driver
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec/maxim/max77978.h>
#include <linux/usb/typec/maxim/max77978-private.h>
#include <linux/usb/typec/maxim/max77978-bc12.h>
#include <linux/usb/typec/maxim/max77978-usbc.h>
#include <linux/of.h>

#define MAX77978_FIRMWARE_FILE	"max77978-fw.bin"

/* Reset register and value */
#define MAX77978_RESET_VAL	0x7F

#define MAX77978_I2C_BULK_BLOCK_SIZE	(16)
#if defined(CONFIG_MAX77978_DEBUG)
#define MAX77978_I2C_BULK_DEBUG(...)	print_hex_dump(__VA_ARGS__)
#else
#define MAX77978_I2C_BULK_DEBUG(...)
#endif

static const char I2C_BULK_STR[MAX77978_I2C_BULK_MAX][9] = {
	[MAX77978_I2C_BULK_READ] = "BULK_R",
	[MAX77978_I2C_BULK_WRITE] = "BULK_W",
	[MAX77978_I2C_BULK_VERIFY] = "BULK_V",
	[MAX77978_I2C_OPCODE_READ] = "OPCODE_R",
	[MAX77978_I2C_OPCODE_WRITE] = "OPCODE_W",
};

static int max77978_usbc_wait_response(struct max77978_dev *max77978);
static void max77978_reset_ic(struct max77978_dev *max77978);

int max77978_i2c_bulk(const char *caller, u8 type, struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);
	int extra = count % MAX77978_I2C_BULK_BLOCK_SIZE;
	int loop = (count / MAX77978_I2C_BULK_BLOCK_SIZE) + ((extra > 0) ? 1 : 0);
	int i = 0, offset = 0;
	char strdump[64] = "";
	int err = 0;

	if (max77978->suspended == true) {
		msg_info("max77978 in suspend state, just return");
		return -EACCES;
	}

	if (max77978->shutdown) {
		pr_err("%s:%s shutdown. i2c command is skiped\n",
			MFD_DEV_NAME, __func__);
		return 0;
	}

	if (type >= MAX77978_I2C_BULK_MAX)
		return -EINVAL;

	mutex_lock(&max77978->i2c_lock);

	if (reg == OPCODE_WRITE) {
		type = MAX77978_I2C_OPCODE_WRITE;
		extra = (count - 1) % MAX77978_I2C_BULK_BLOCK_SIZE;
		loop =  ((count - 1) / MAX77978_I2C_BULK_BLOCK_SIZE) + ((extra > 0) ? 1 : 0);
		reg++;
	} else if (reg == OPCODE_READ)
		type = MAX77978_I2C_OPCODE_READ;

	for (i = 0, offset = 0; i < loop; i++, offset += MAX77978_I2C_BULK_BLOCK_SIZE) {
		u8 addr = reg + offset;
		int len = (extra && (i == loop-1)) ? extra : MAX77978_I2C_BULK_BLOCK_SIZE;
		u8 *pbuf = &buf[offset];

		/* set string for debug */
		snprintf(strdump, sizeof(strdump), "%s.%s.%02Xh : ", caller, I2C_BULK_STR[type], addr);

		switch (type) {
		case MAX77978_I2C_BULK_READ:
		case MAX77978_I2C_OPCODE_READ:
			err = i2c_smbus_read_i2c_block_data(i2c, addr, len, pbuf);
			if (err == len)
				err = 0;
			if (!err)
				MAX77978_I2C_BULK_DEBUG(KERN_INFO, strdump, DUMP_PREFIX_NONE, 8, 1, (const void *)pbuf, len, false);
			else
				pr_err("%s : failed(%d) to i2c_smbus_read_i2c_block_data\n", __func__, err);
			break;
		case MAX77978_I2C_BULK_WRITE:
		case MAX77978_I2C_BULK_VERIFY:
		case MAX77978_I2C_OPCODE_WRITE:
			// write
			pbuf = &buf[offset + 1];
			MAX77978_I2C_BULK_DEBUG(KERN_INFO, strdump, DUMP_PREFIX_NONE, 8, 1, (const void *)pbuf, len, false);
			err = i2c_smbus_write_i2c_block_data(i2c, addr, len, pbuf);
			if (err) {
				pr_err("%s : failed(%d) to i2c_smbus_write_i2c_block_data\n", __func__, err);
				break;
			} else if (type == MAX77978_I2C_BULK_VERIFY) {
				// verify
				u8 rbuf[MAX77978_I2C_BULK_BLOCK_SIZE] = {0,};

				memset(strdump, 0, sizeof(strdump));
				snprintf(strdump, sizeof(strdump), "%s.%s.%02X.", caller, I2C_BULK_STR[type], addr);
				err = i2c_smbus_read_i2c_block_data(i2c, addr, len, rbuf);
				if (err == len)
					err = 0;
				if (!err) {
					int j;

					for (j = 0; j < len; j++) {
						if (pbuf[j] != rbuf[j]) {
							err = -EFAULT;
							break;
						}
					}
					MAX77978_I2C_BULK_DEBUG(KERN_INFO, strdump, DUMP_PREFIX_NONE, 8, 1, (const void *)pbuf, len, false);
				} else {
					pr_err("%s : failed(%d) to i2c_smbus_read_i2c_block_data\n", __func__, err);
				}
			}
			break;
		default:
			break;
		}
	}
	if (type == MAX77978_I2C_OPCODE_WRITE) {
		snprintf(strdump, sizeof(strdump), "%s.%s.%02Xh : ", caller, I2C_BULK_STR[type], OPCODE_WRITE);
		MAX77978_I2C_BULK_DEBUG(KERN_INFO, strdump, DUMP_PREFIX_NONE, 8, 1, (const void *)buf, 1, false);
		err = i2c_smbus_write_i2c_block_data(i2c, OPCODE_WRITE, 1, &buf[0]);
	}

	mutex_unlock(&max77978->i2c_lock);


	return err;
}
EXPORT_SYMBOL_GPL(max77978_i2c_bulk);

int max77978_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);
	int err, val = 0;

	if (max77978->suspended == true) {
		msg_info("max77978 in suspend state, just return");
		return -EACCES;
	}

	if (max77978->shutdown) {
		pr_err("%s:%s shutdown. i2c command is skiped\n",
			MFD_DEV_NAME, __func__);
		return 0;
	}

	mutex_lock(&max77978->i2c_lock);
	val = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&max77978->i2c_lock);
	if (val < 0) {
		err = val;
		pr_err("%s: sid(0x%02X) reg(0x%x), err(%d)\n", __func__, i2c->addr, reg, err);
	} else {
		err = 0;
		*dest = (u8)(val & 0xff);
	}
	return err;
}
EXPORT_SYMBOL_GPL(max77978_read_reg);

int max77978_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);
	int err;

	if (max77978->suspended == true) {
		msg_info("max77978 in suspend state, just return");
		return -EACCES;
	}

	if (max77978->shutdown) {
		pr_err("%s:%s shutdown. i2c command is skiped\n",
			MFD_DEV_NAME, __func__);
		return 0;
	}

	mutex_lock(&max77978->i2c_lock);
	err = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&max77978->i2c_lock);
	if (err < 0)
		pr_err("%s: reg(0x%x), err(%d)\n", __func__, reg, err);
	return err;
}
EXPORT_SYMBOL_GPL(max77978_write_reg);

int max77978_update_reg(struct i2c_client *i2c, u8 reg, u8 value, u8 mask)
{
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);
	int err, val = 0;
	u8 old_val, new_val;

	if (max77978->suspended == true) {
		msg_info("max77978 in suspend state, just return");
		return -EACCES;
	}

	if (max77978->shutdown) {
		pr_err("%s:%s shutdown. i2c command is skiped\n",
			MFD_DEV_NAME, __func__);
		return 0;
	}

	mutex_lock(&max77978->i2c_lock);
	val = i2c_smbus_read_byte_data(i2c, reg);
	if (val < 0) {
		err = val;
	} else {
		old_val = (u8)(val & 0xff);
		new_val = (value & mask) | (old_val & (~mask));
		err = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&max77978->i2c_lock);

	if (err)
		pr_err("%s: sid(0x%02X) reg(0x%x), err(%d)\n", __func__, i2c->addr, reg, err);

	return err;
}
EXPORT_SYMBOL_GPL(max77978_update_reg);

static int max77978_update_revinfo(struct max77978_dev *max77978)
{
	u8 data[2] = {0, 0};
	int err = max77978_bulk_read(max77978->i2c, REG_UIC_DEVICE_ID, sizeof(data), data);

	if (!err) {
		max77978->device_id = data[0];
		max77978->device_revision = data[1];
	}
	return err;
}

static int max77978_update_fwinfo(struct max77978_dev *max77978)
{
	u8 data[2] = {0, 0};
	int err = max77978_bulk_read(max77978->i2c, REG_UIC_FW_REV, sizeof(data), data);

	if (!err) {
		max77978->fw_revision = data[0];
		max77978->fw_minor_revision = data[1];
	}
	return err;
}

static u8 max77978_read_devrev(struct max77978_dev *max77978)
{
	u8 data = 0xff; /* unknown value */

	if (max77978_read_reg(max77978->i2c, REG_UIC_DEVICE_REV, &data)) {
		pr_err("%s: failed read REG_UIC_DEVICE_REV\n", __func__);
		return MAX77978_DEVREV_UNKNOWN;
	}
	return (data & 0xFF);
}

static int max77978_set_secureboot(struct max77978_dev *max77978)
{
	u8 wdata[OPCODE_MAX_LENGTH] = {0,};
	int err = -ENODEV;

	pr_info("%s : try to set secureboot\n", __func__);
	if (!max77978) {
		pr_err("%s : max77978 is null\n", __func__);
		return -ENODEV;
	}

	wdata[0] = 0xD0;
	err = max77978_opcode_write(max77978->i2c, (int)sizeof(wdata), wdata);
	if (err) {
		pr_err("%s : failed (%d)\n", __func__, err);
	} else {
		msleep(500);
		err = max77978_update_fwinfo(max77978);
		if (err) {
			pr_err("%s : failed to read FW version (%d)\n", __func__, err);
			return -ENODEV;
		} else if (max77978->fw_revision != 0xFF || max77978->fw_minor_revision != 0xFF) {
			pr_err("%s : is not secureboot (%02X.%02X)\n", __func__, max77978->fw_revision, max77978->fw_minor_revision);
			return -EPERM;
		} else {
			pr_info("%s : complete to set secureboot (%02X.%02X)\n", __func__, max77978->fw_revision, max77978->fw_minor_revision);
		}
	}

	return err;
}

static int max77978_enable_recovery(struct max77978_dev *max77978)
{
	const u8 wdata[OPCODE_MAX_LENGTH] = {
		0xDF, 0xDA, 0xA5, 0xAD, 0x78, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00
	};
	int err = -ENODEV;
	int tries = 0;

	pr_info("%s : try to enable recovery\n", __func__);
	do {
		err = max77978_opcode_write(max77978->i2c, (int)sizeof(wdata), (u8 *)wdata);
		if (!err) {
			msleep(500);
			max77978_reset_ic(max77978);
			msleep(100);
			err = max77978_read_reg(max77978->i2c, REG_UIC_FW_REV, &max77978->fw_revision);
			if (!err) {
				err = max77978_read_reg(max77978->i2c, REG_UIC_FW_SUBREV, &max77978->fw_minor_revision);
				if (!err) {
					if ((max77978->fw_revision != 0x3) || (max77978->fw_minor_revision != 0x58))
						err = -ENODEV;
					else
						max77978_reset_ic(max77978);
				}
			}
		}

		if (err)
			tries++;
	} while ((tries < FW_VERIFY_TRY_COUNT) && err);

	if (err) {
		pr_err("%s : failed (%d)\n", __func__, err);
	} else {
		pr_info("%s : complete to enable recovery (%02X.%02X)\n",
				__func__, max77978->fw_revision, max77978->fw_minor_revision);
	}

	return err;
}

static int of_max77978_dt(struct device *dev, struct max77978_platform_data *pdata)
{
	if (!dev->of_node)
		return -EINVAL;

	pdata->wakeup = device_property_read_bool(dev, "wakeup-source");

	return 0;
}

static void max77978_reset_ic(struct max77978_dev *max77978)
{
	pr_info("%s: Reset!!\n", __func__);
	max77978_write_reg(max77978->i2c, REG_RESET, MAX77978_RESET_VAL);
	msleep(800);
}

static void wait_apcmd_resp_work_func(struct work_struct *work)
{
	struct max77978_dev *max77978;
	u8 dummy[2] = {0, 0};

	max77978 = container_of(work, struct max77978_dev, wait_apcmd_resp_work);

	while (max77978->fw_update_state == FW_UPDATE_WAIT_RESP_START) {
		if (!max77978_read_reg(max77978->i2c, REG_UIC_INT, &dummy[0])) {
			/* check SysMsg interrupt */
			if (dummy[0] & BIT_SYSMsgI) {
				if (!max77978_read_reg(max77978->i2c, REG_USBC_STATUS2, &dummy[1])) {
					if (dummy[1] == 0x40) {
						pr_err("%s : SYSERROR_INTER_I2C_FAIL (0x%02X 0x%02X)\n",
						       __func__, dummy[0], dummy[1]);
						break;
					}
				}
			}

			/* check APCmdResponse interrupt */
			if (dummy[0] & BIT_APCmdResI) {
				max77978->fw_update_state = FW_UPDATE_WAIT_RESP_STOP;
				break;
			}
		}
		/* Sleep to avoid busy loop and reduce CPU overhead */
		usleep_range(1000, 1500);
	}

	complete_all(&max77978->wait_apcmd_resp_completion);
}

static int max77978_usbc_wait_response(struct max77978_dev *max77978)
{
	unsigned long time_remaining = 0;

	max77978->fw_update_state = FW_UPDATE_WAIT_RESP_START;

	init_completion(&max77978->wait_apcmd_resp_completion);
	queue_work(max77978->wait_apcmd_resp_work_queue, &max77978->wait_apcmd_resp_work);

	time_remaining = wait_for_completion_timeout(
			&max77978->wait_apcmd_resp_completion,
			msecs_to_jiffies(FW_WAIT_TIMEOUT));

	if (!time_remaining || (max77978->fw_update_state != FW_UPDATE_WAIT_RESP_STOP)) {
		pr_err("%s: failed to get response due to timeout or syserr\n", __func__);
		cancel_work_sync(&max77978->wait_apcmd_resp_work);
		return FW_UPDATE_TIMEOUT_FAIL;
	}

	pr_info("%s: apcmd received\n", __func__);
	return 0;
}

static int max77978_update_action_mtp(struct max77978_dev *max77978,
		const u8 *action_bin, int fw_bin_len)
{
	u8 devrev;
	int err = 0;
	int try_count = 0;
	unsigned long duration = 0;

	int offset = 0;
	int size = 0;
	u8 action_data[28];
	u8 w_data[OPCODE_MAX_LENGTH] = {0, };
	u8 r_data[OPCODE_MAX_LENGTH] = {0, };
	int length = 0;
	int mtp_pos = 0;
	int prev_pos = 0;
	u8 status2 = 0x0;

update_action_mtp_retry:
	offset = 0;
	duration = 0;
	size = 0;
	err = 0;
	mtp_pos = 0;
	prev_pos = 0x0;
	try_count++;

	err = max77978_update_revinfo(max77978);
	if (err)
		return err;
	err = max77978_update_fwinfo(max77978);
	if (err)
		return err;

	devrev = max77978_read_devrev(max77978);
	pr_info("%s: %s FW:%02X.%02X\n", __func__,
			MAX77978_DEVREV_STR(devrev),
			max77978->fw_revision, max77978->fw_minor_revision);

	duration = jiffies;

	if ((max77978->fw_revision != 0xff) && (try_count < 10)) {
		while (offset < fw_bin_len) {
			memset(action_data, 0, sizeof(action_data));
			memset(w_data, 0, sizeof(w_data));
			memset(r_data, 0, sizeof(r_data));
			for (size = 0x0;
				size < 28 && offset < fw_bin_len; size++)
				action_data[size] = action_bin[offset++];
			w_data[0] = OPCODE_ACTION_BLOCK_MTP_UPDATE;
			if (mtp_pos) {
				prev_pos = mtp_pos;
				w_data[1] = mtp_pos++;
			} else {
				w_data[1] = 0xF0;
				mtp_pos++;
			}
			length = size;
			memcpy(&w_data[2], &action_data, length);

			err = max77978_bulk_write(max77978->i2c, OPCODE_WRITE, (int)sizeof(w_data), w_data);
			if (err) {
				pr_err("%s : failed(%d) to max77978_bulk_write at offset 0x%X\n", __func__, err, offset);
				goto update_action_mtp_retry;
			}
			if (max77978_usbc_wait_response(max77978)) {
				pr_err("%s : failed to timeout for waiting response\n", __func__);
				goto update_action_mtp_retry;
			}

			err = max77978_read_reg(max77978->i2c, REG_USBC_STATUS2, &status2);
			if (err) {
				pr_err("%s : failed(%d) to max77978_read_reg(REG_USBC_STATUS2) at offset 0x%X\n", __func__, err, offset);
				goto update_action_mtp_retry;
			}

			err = max77978_bulk_read(max77978->i2c, OPCODE_READ, 3, r_data);
			if (err) {
				pr_err("%s : failed(%d) to max77978_bulk_read at offset 0x%X\n", __func__, err, offset);
				goto update_action_mtp_retry;
			}

			if (r_data[1] == 0xFF) {
				pr_err("%s: MTP Erase Fail\n", __func__);
				break;
			} else if (r_data[1] == prev_pos || r_data[1] == 0xF0) {
				//pr_info("%s: MTP Write Done for No.%d\n", __func__, r_data[1]);
			} else {
				pr_info("%s: REG_USBC_STATUS2(0x%02X) r_data(0x%02X,0x%02X,0x%02X) at offset 0x%X\n", __func__,
					status2, r_data[0], r_data[1], r_data[2], offset);
				goto update_action_mtp_retry;
			}
		}
	} else {
		pr_err("%s: Need to update the firmware! or MTP write fail [%x]", __func__, try_count);
	}

	duration = jiffies - duration;
	pr_info("%s: completed(%d ms)\n", __func__, jiffies_to_msecs(duration));

	return 0;
}

static void max77978_update_custm_info_pass1(struct max77978_dev *max77978)
{
	union max77978_custm_info_pass1 customer_info;
	u8 wdata[OPCODE_MAX_LENGTH] = {0,};
	u8 rdata[OPCODE_MAX_LENGTH] = {0,};
	int err = -ENODEV;

	pr_info("%s: update customer information", __func__);
	customer_info.custm_bits.debugSRC = ENABLED;
	customer_info.custm_bits.debugSNK = ENABLED;
	customer_info.custm_bits.trysnksrc = TRYNONE;
	customer_info.custm_bits.typecstate = TYPEC_PORT_DRP & 0x3;
	customer_info.custm_bits.memoryupdate = MEMORYUPDATEMTP;
	customer_info.custm_bits.moisturedetection = MOISTUREDETECTIONDISABLE;
	customer_info.custm_bits.VID = PD_VID;
	customer_info.custm_bits.PID = PD_PID;
	customer_info.custm_bits.i2cSID1 = I2C_SID0;
	customer_info.custm_bits.i2cSID2 = I2C_SID1;
	customer_info.custm_bits.i2cSID3 = I2C_SID2;
	customer_info.custm_bits.i2cSID4 = I2C_SID3;
	customer_info.custm_bits.fastroleswap = FAST_ROLESWAP;
	customer_info.custm_bits.rpvalue = RPVALUE;

	wdata[0] = OPCODE_SET_CSTM_INFORMATION_W;
	memcpy(&wdata[1], &customer_info, sizeof(customer_info));
	err = max77978_opcode_write(max77978->i2c, (int)sizeof(wdata), wdata);
	if (!err) {
		print_hex_dump(KERN_INFO, "PASS1 customer_info.W : ", DUMP_PREFIX_OFFSET, 8, 1, wdata, sizeof(customer_info)+1, false);
		if (max77978_usbc_wait_response(max77978)) {
			pr_err("%s : failed to timeout for waiting response\n", __func__);
			return;
		}
	} else {
		pr_err("%s : failed(%d) to max77978_opcode_write(wdata)\n", __func__, err);
		return;
	}

	rdata[0] = OPCODE_SET_CSTM_INFORMATION_R;
	err = max77978_opcode_write(max77978->i2c, (int)sizeof(rdata), rdata);
	if (!err) {
		memset(rdata, 0, sizeof(rdata));
		err = max77978_opcode_read(max77978->i2c, (int)sizeof(rdata), rdata);
		if (!err) {
			print_hex_dump(KERN_INFO, "PASS1 customer_info.R : ", DUMP_PREFIX_OFFSET, 8, 1, rdata, sizeof(customer_info)+1, false);
		} else {
			pr_err("%s : failed(%d) to max77978_opcode_read(rdata)\n", __func__, err);
			return;
		}
	} else {
		pr_err("%s : failed(%d) to max77978_opcode_write(rdata)\n", __func__, err);
		return;
	}
}

static void max77978_update_custm_info(struct max77978_dev *max77978)
{
	u8 devrev = max77978_read_devrev(max77978);

	if (devrev == MAX77978_DEVREV_PASS1)
		max77978_update_custm_info_pass1(max77978);
}

static void max77978_update_configuration(struct max77978_dev *max77978, bool enforce_do)
{
	if (enforce_do) {
		const char bin[32] = "Custom_Script.bin"; /* The bin file name which is located on '/lib/firmware/' */
		const struct firmware *fw;

		if (request_firmware(&fw, bin, max77978->dev)) {
			pr_err("%s : failed to load the custom script '%s'\n", __func__, bin);
			return;
		}

		/* disable irqs */
		disable_irq(max77978->irq);
		max77978_write_reg(max77978->i2c, REG_PD_INT_M, 0xFF);
		max77978_write_reg(max77978->i2c, REG_CC_INT_M, 0xFF);
		max77978_write_reg(max77978->i2c, REG_UIC_INT_M, 0xFF);
		max77978_write_reg(max77978->i2c, REG_ACTION_INT_M, 0xFF);
		max77978_update_custm_info(max77978);

		pr_info("%s : '%s' loading OK, size=%zu bytes\n", __func__, bin, fw->size);
		max77978_update_action_mtp(max77978, fw->data, fw->size);
		max77978_reset_ic(max77978);

		/* enable irqs */
		enable_irq(max77978->irq);
	}
}

static int __max77978_usbc_fw_update(struct max77978_dev *max77978, const u8 *fw_bin)
{
	const u8 *pcmd = &fw_bin[0]; // 0:END 1:WRITE 3:READ
	const u8 len = fw_bin[1] - 1; // fw_bin[1] == Length(OPCode + Data)
	const u8 size = 3 + len; /* "3" measn that sizeof(cmd) + sizeof(len) + sizeof(opcode) */
	const u8 *pcode = &fw_bin[2];
	const u8 *pdata = &fw_bin[3];
	u8 rdata[FW_VERIFY_DATA_SIZE] = {0,};

	pr_debug("%s : cmd=0x%02X len=%d opcode=0x%02X\n", __func__, *pcmd, len, *pcode);

	/* (1) if cmd is FW_CMD_END (0x00) then reset and return FW_UPDATE_END */
	if (*pcmd == FW_CMD_END) {
		max77978_reset_ic(max77978);
		max77978->fw_update_state = FW_UPDATE_END;
		return FW_UPDATE_END;
	}

	/* (2) if len is not 0x21 or 0x03 then return FW_UPDATE_MAX_LENGTH_FAIL */
	if (len != 0x21 && len != 0x03) {
		pr_err("%s : length is not supportable (len=0x%02X)\n", __func__, len);
		return FW_UPDATE_MAX_LENGTH_FAIL;
	}

	switch (*pcmd) {
	case FW_CMD_WRITE: /* 0x01 */
		/* (3) if cmd is FW_CMD_WRITE (0x01) then write fw data */
		if (max77978_bulk_write(max77978->i2c, *pcode, len, (u8 *)pdata))
			return FW_UPDATE_CMD_FAIL;
		if (max77978_usbc_wait_response(max77978)) {
			pr_err("%s : failed to timeout for waiting response\n", __func__);
			return FW_UPDATE_TIMEOUT_FAIL;
		}
		return size;

	case FW_CMD_READ: /* 0x03 */
		/* (4) if cmd is FW_CMD_READ (0x03) then read data for verify */
		if (max77978_bulk_read(max77978->i2c, *pcode - 1, len, rdata))
			return FW_UPDATE_VERIFY_FAIL;
		else {
			if ((fw_bin[4] != rdata[1]) || (fw_bin[5] != rdata[0])) {
				pr_err("%s : len=%d fwdata(0x%02X 0x%02X) rdata(0x%02X 0x%02X)\n", __func__,
						len, fw_bin[4], fw_bin[5], rdata[1], rdata[0]);
				return FW_UPDATE_VERIFY_FAIL;
			} else {
				return size;
			}
		}
		break;

	default:
		pr_err("%s: Command error (cmd=0x%02X)\n", __func__, *pcmd);
		break;
	}

	return FW_UPDATE_CMD_FAIL;
}

int max77978_usbc_fw_update(struct max77978_dev *max77978, const u8 *fw_bin, int fw_bin_len, int enforce_do)
{
	struct max77978_fw_header fw_header;
	int offset = 0;
	unsigned long start_jiffies = 0;
	int size = 0;
	int try_count = 0;
	int err = 0;
	u8 try_command = 0;
	u8 devrev = MAX77978_DEVREV_UNKNOWN;

	disable_irq(max77978->irq);
	max77978_write_reg(max77978->i2c, REG_PD_INT_M, 0xFF);
	max77978_write_reg(max77978->i2c, REG_CC_INT_M, 0xFF);
	max77978_write_reg(max77978->i2c, REG_UIC_INT_M, 0xFF);
	max77978_write_reg(max77978->i2c, REG_ACTION_INT_M, 0xFF);

fw_update_retry:
	offset = 0;
	size = 0;
	err = 0;

	if (try_count >= FW_VERIFY_TRY_COUNT)
		goto fw_update_exit;

	/* read devrev and rev info again */
	devrev = max77978_read_devrev(max77978);
	if (devrev == MAX77978_DEVREV_UNKNOWN)
		goto fw_update_exit;

	err |= max77978_update_revinfo(max77978);
	err |= max77978_update_fwinfo(max77978);
	if (err) {
		pr_err("%s: failed(%d) to read revinfo and fwinfo\n", __func__, err);
		if ((!try_count) && (!try_command)) {
			pr_err("%s: exit by i2c failed (try_command=%d try_count=%d)\n", __func__, try_command, try_count);
			goto fw_update_exit;
		} else {
			try_count++;
			goto fw_update_retry;
		}
	}

	start_jiffies = jiffies;

	pr_info("%s: %s now.FW:%02X.%02X BIN:%02X.%02X\n", __func__,
			MAX77978_DEVREV_STR(devrev),
			max77978->fw_revision, max77978->fw_minor_revision,
			fw_bin[4], fw_bin[5]);
	if ((max77978->fw_revision == 0xff) ||
			(max77978->fw_revision < fw_bin[4]) ||
			((max77978->fw_revision == fw_bin[4]) &&
			(max77978->fw_minor_revision < fw_bin[5])) ||
			enforce_do) {
		if (max77978->fw_revision == 0xff) {
			/* in the case of Secure F/W mode,
			 * the kernel driver can't check the plug in or out.
			 * please check the VBUS is valid or invalid using charger device.
			 * the kernel driver can sets the CHGIN current based on
			 * an information of charger chip. */

		}

		/* into secureboot mode */
		err = max77978_set_secureboot(max77978);
		if (err) {
			pr_err("%s: failed(%d) to set secureboot\n", __func__, err);
			/* retry when failed */
			if (try_command < 3) {
				max77978_reset_ic(max77978);
				msleep(100);
				try_command++;
				goto fw_update_retry;
			} else if (try_count < FW_VERIFY_TRY_COUNT) {
				max77978_reset_ic(max77978);
				msleep(100);
				try_count++;
				goto fw_update_retry;
			} else {
				pr_err("%s: exit by secureboot failed (try_command=%d try_count=%d)\n", __func__, try_command, try_count);
				goto fw_update_exit;
			}
		}

		/* start fw update */
		pr_info("%s: Start FW updating (%02X.%02X)\n", __func__, max77978->fw_revision, max77978->fw_minor_revision);

		/* Copy fw header */
		memcpy(&fw_header, fw_bin, FW_HEADER_SIZE);

		for (offset = FW_HEADER_SIZE; (offset < fw_bin_len) && (size != FW_UPDATE_END); ) {
			size = __max77978_usbc_fw_update(max77978, &fw_bin[offset]);
			//pr_info("%s: offset=%d size=0x%02X\n", __func__, offset, size);

			switch (size) {
			case FW_UPDATE_VERIFY_FAIL:
			case FW_UPDATE_TIMEOUT_FAIL:
				if (++try_count < FW_VERIFY_TRY_COUNT) {
					pr_err("%s: retry by fault(0x%02X) at offset(%d) and %d tries\n",
							__func__, size, offset, try_count);
					max77978_reset_ic(max77978);
					msleep(100);
					goto fw_update_retry;
				} else {
					pr_err("%s: exit by fault(0x%02X) at offset(%d) and %d tries\n",
							__func__, size, offset, try_count);
					goto fw_update_exit;
				}
				break;
			case FW_UPDATE_CMD_FAIL:
			case FW_UPDATE_MAX_LENGTH_FAIL:
				pr_err("%s: exit by fault(0x%02X) at offset(%d) and %d tries\n",
						__func__, size, offset, try_count);
				goto fw_update_exit;
			case FW_UPDATE_END: /* 0x00 */
				/* read FW info finally */
				max77978_read_reg(max77978->i2c, REG_UIC_FW_REV, &max77978->fw_revision);
				max77978_read_reg(max77978->i2c, REG_UIC_FW_SUBREV, &max77978->fw_minor_revision);
				pr_info("%s: FW_UPDATE_END %s FW:%02X.%02X BIN:%02X.%02X completed(%d ms) %s\n", __func__,
						MAX77978_DEVREV_STR(devrev),
						max77978->fw_revision, max77978->fw_minor_revision,
						fw_bin[4], fw_bin[5],
						jiffies_to_msecs(jiffies - start_jiffies),
						((max77978->fw_revision == fw_bin[4]) && (max77978->fw_minor_revision == fw_bin[5])) ? "OK" : "FAILED");
				break;
			default:
				offset += size;
				break;
			}
			if (offset == fw_bin_len) {
				max77978_reset_ic(max77978);
				max77978_update_fwinfo(max77978);
				pr_info("%s: %s after.FW:%02X.%02X BIN:%02X.%02X completed(%d ms) %s\n", __func__,
						MAX77978_DEVREV_STR(devrev),
						max77978->fw_revision, max77978->fw_minor_revision,
						fw_bin[4], fw_bin[5],
						jiffies_to_msecs(jiffies - start_jiffies),
						((max77978->fw_revision == fw_bin[4]) && (max77978->fw_minor_revision == fw_bin[5])) ? "OK" : "FAILED");
			}
		}
	} else {
		pr_info("%s: no need to update!\n", __func__);
		goto fw_update_exit;
	}

	max77978_update_configuration(max77978, false);

fw_update_exit:

	if (max77978->fw_revision == 0xFF)
		max77978_enable_recovery(max77978);

	enable_irq(max77978->irq);

	return err;
}
EXPORT_SYMBOL_GPL(max77978_usbc_fw_update);

static int max77978_i2c_probe(struct i2c_client *i2c)
{
	struct max77978_dev *max77978;
	struct max77978_platform_data *pdata = i2c->dev.platform_data;
	const struct firmware *fw;
	int ret = 0;

	pr_info("%s: %s(addr=0x%02X)\n", MFD_DEV_NAME, __func__, i2c->addr);

	max77978 = kzalloc(sizeof(*max77978), GFP_KERNEL);
	if (!max77978)
		return -ENOMEM;

	max77978->boot_complete = 0;
	pr_info("%s : boot_complete=%d\n", __func__, max77978->boot_complete);

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
			sizeof(struct max77978_platform_data),
			GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err_exit;
		}

		ret = of_max77978_dt(&i2c->dev, pdata);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to get device of_node\n");
			goto err_exit;
		}

		i2c->dev.platform_data = pdata;
	} else
		pdata = i2c->dev.platform_data;

	max77978->dev = &i2c->dev;
	max77978->i2c = i2c;
	max77978->irq = i2c->irq;
	if (pdata) {
		max77978->pdata = pdata;

		pdata->irq_base = irq_alloc_descs(-1, 0, MAX77978_IRQ_NR, -1);
		if (pdata->irq_base < 0) {
			pr_err("%s:%s irq_alloc_descs Fail! ret(%d)\n",
				MFD_DEV_NAME,	__func__, pdata->irq_base);
			ret = -EINVAL;
			goto err_exit;
		} else
			max77978->irq_base = pdata->irq_base;

		max77978->wakeup = pdata->wakeup;
	} else {
		ret = -EINVAL;
		goto err_exit;
	}

	max77978->ws = wakeup_source_register(max77978->dev, MFD_DEV_NAME);
	mutex_init(&max77978->i2c_lock);
	init_waitqueue_head(&max77978->suspend_wait);
	i2c_set_clientdata(i2c, max77978);
	if (max77978_update_revinfo(max77978) || max77978_update_fwinfo(max77978)) {
		ret = -ENODEV;
		goto err_mutex;
	}

	init_completion(&max77978->wait_apcmd_resp_completion);
	max77978->wait_apcmd_resp_work_queue = alloc_ordered_workqueue("wait_apcmd_resp_work", 0);
	if (max77978->wait_apcmd_resp_work_queue == NULL) {
		ret = -ENOMEM;
		goto err_mutex;
	}
	INIT_WORK(&max77978->wait_apcmd_resp_work, wait_apcmd_resp_work_func);

	ret = request_firmware(&fw, MAX77978_FIRMWARE_FILE, &max77978->i2c->dev);
	if (ret) {
		dev_err(&max77978->i2c->dev, "Failed to load firmware '%s' (err=%d)\n", MAX77978_FIRMWARE_FILE, ret);
	} else {
		dev_info(&max77978->i2c->dev, "Firmware '%s' loaded as %zu bytes\n", MAX77978_FIRMWARE_FILE, fw->size);
		ret = max77978_usbc_fw_update(max77978, fw->data, fw->size, 0);
		release_firmware(fw);
	}

	max77978_update_configuration(max77978, false);
	ret = max77978_irq_init(max77978);

	if (ret < 0)
		goto err_irq_init;

	disable_irq(max77978->irq);

	ret = max77978_usbc_init(max77978);
	if (ret < 0)
		goto err_usbc;

	device_init_wakeup(max77978->dev, pdata->wakeup);

	pr_info("%s: done (err=%d)\n", __func__, ret);
	return ret;

err_usbc:
	max77978_usbc_deinit(g_usbc_data);
	max77978_irq_exit(max77978);
err_irq_init:
	if (max77978->wait_apcmd_resp_work_queue)
		destroy_workqueue(max77978->wait_apcmd_resp_work_queue);
err_mutex:
	mutex_destroy(&max77978->i2c_lock);
	wakeup_source_unregister(max77978->ws);
	irq_free_descs(max77978->irq_base, MAX77978_IRQ_NR);
err_exit:
	kfree(max77978);
	return ret;
}

static void max77978_i2c_remove(struct i2c_client *i2c)
{
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);

	device_init_wakeup(max77978->dev, 0);
	max77978_usbc_deinit(g_usbc_data);
	max77978_irq_exit(max77978);
	if (max77978->wait_apcmd_resp_work_queue)
		destroy_workqueue(max77978->wait_apcmd_resp_work_queue);
	mutex_destroy(&max77978->i2c_lock);
	wakeup_source_unregister(max77978->ws);
	irq_free_descs(max77978->irq_base, MAX77978_IRQ_NR);
	kfree(max77978);
}

static void max77978_i2c_shutdown(struct i2c_client *i2c)
{
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);

	max77978_usbc_shutdown(g_usbc_data);
	max77978_irq_exit(max77978);
	max77978->shutdown = 1;
}

static const struct i2c_device_id max77978_i2c_id[] = {
	{ MFD_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77978_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id max77978_i2c_dt_ids[] = {
	{ .compatible = "adi,max77978" },
	{ },
};
MODULE_DEVICE_TABLE(of, max77978_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
static int max77978_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);

	pr_info("%s:%s\n", MFD_DEV_NAME, __func__);

	synchronize_irq(max77978->irq);

	max77978->suspended = true;

	return 0;
}

static int max77978_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max77978_dev *max77978 = i2c_get_clientdata(i2c);

	pr_info("%s:%s\n", MFD_DEV_NAME, __func__);

	max77978->suspended = false;
	wake_up_interruptible(&max77978->suspend_wait);

	return 0;
}
#else
#define max77978_suspend	NULL
#define max77978_resume		NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_HIBERNATION
static int max77978_freeze(struct device *dev)
{
	return 0;
}

static int max77978_restore(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops max77978_pm = {
	.suspend = max77978_suspend,
	.resume = max77978_resume,
#ifdef CONFIG_HIBERNATION
	.freeze =  max77978_freeze,
	.thaw = max77978_restore,
	.restore = max77978_restore,
#endif
};

static struct i2c_driver max77978_i2c_driver = {
	.driver		= {
		.name	= MFD_DEV_NAME,
#if defined(CONFIG_PM)
		.pm	= &max77978_pm,
#endif /* CONFIG_PM */
#if defined(CONFIG_OF)
		.of_match_table	= max77978_i2c_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe		= max77978_i2c_probe,
	.remove		= max77978_i2c_remove,
	.shutdown	= max77978_i2c_shutdown,
	.id_table	= max77978_i2c_id,
};

static int __init max77978_i2c_init(void)
{
	return i2c_add_driver(&max77978_i2c_driver);
}

/* init early so consumer devices can complete system boot */
subsys_initcall(max77978_i2c_init);

static void __exit max77978_i2c_exit(void)
{
	i2c_del_driver(&max77978_i2c_driver);
}
module_exit(max77978_i2c_exit);

MODULE_DESCRIPTION("max77978 driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
