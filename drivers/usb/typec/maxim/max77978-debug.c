// SPDX-License-Identifier: GPL-2.0-or-later
/* ******************************************************************************
 * Copyright (C) Analog Device, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 * ******************************************************************************/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif /* CONFIG_COMPAT */

#include <linux/usb/typec/maxim/max77978.h>
#include <linux/usb/typec/maxim/max77978-debug.h>

static struct mxim_debug_registers mxim_debug_reg[] = {
	{
		.reg = MXIM_REG_UIC_HW_REV,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_UIC_DEVICE_ID,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_UIC_FW_REV,
		.val = 0,
		.ignore = 1,
	},
	{
		.reg = MXIM_REG_UIC_FW_SUBREV,
		.val = 0,
		.ignore = 1,
	},
	{
		.reg = MXIM_REG_UIC_INT,
		.val = 0,
		.ignore = 1,
	},
	{
		.reg = MXIM_REG_CC_INT,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_PD_INT,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_ACTION_INT,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_USBC_STATUS1,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_USBC_STATUS2,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_BC_STATUS,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_DP_STATUS,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_CC_STATUS0,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_CC_STATUS1,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_PD_STATUS0,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_PD_STATUS1,
		.val = 0,
		.ignore = 0,
	},

	{
		.reg = MXIM_REG_UIC_INT_M,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_CC_INT_M,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_PD_INT_M,
		.val = 0,
		.ignore = 0,
	},
	{
		.reg = MXIM_REG_ACTION_INT_M,
		.val = 0,
		.ignore = 0,
	},

};

static struct mxim_debug_pdev *mxim_pdev;

inline int mxim_debug_i2c_burst_write(unsigned char command,
		unsigned char length, unsigned char *value)
{
	int ret;

	/* Max length 32 bytes */
	ret = i2c_smbus_write_i2c_block_data(mxim_pdev->client, command, length, value);
	if (ret < 0)
		pr_info("Failed to write as burst mode");

	return ret;
}

inline int mxim_debug_i2c_write(unsigned char command,
		unsigned char value)
{
	if (!mxim_pdev)
		return -ENXIO;

	if (mxim_pdev->client == NULL)
		return -ENXIO;

	return i2c_smbus_write_byte_data(mxim_pdev->client,
			command, value);
}

static int mxim_debug_i2c_burst_read(unsigned char command, unsigned char length, unsigned char *value)
{
	int ret;

	/* Max length 32 bytes */
	ret = i2c_smbus_read_i2c_block_data(mxim_pdev->client, command, length, value);
	if (ret < 0)
		pr_info("Failed to read as burst mode");

	return ret;
}

static int mxim_debug_i2c_read(unsigned char reg, unsigned char *value)
{
	int ret = 0;

	if (!mxim_pdev)
		return -ENXIO;

	if (mxim_pdev->client == NULL)
		return -ENXIO;

	ret = i2c_smbus_read_byte_data(mxim_pdev->client, reg);
	if (ret < 0)
		pr_info("Failed to read [reg:%02x]. ret=%d", reg, ret);

	*value = ret;

	return ret;
}

static ssize_t mxim_debug_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int index;
	int len = 0;

	len += sysfs_emit_at(buf, len, "reg   val\n");
	for (index = 0; index < MXIM_REG_MAX; index++) {
		if (mxim_debug_reg[index].ignore)
			continue;
		mxim_debug_i2c_read(mxim_debug_reg[index].reg, &mxim_debug_reg[index].val);
		len += sysfs_emit_at(buf, len, "0x%02x  0x%02x\n",
				mxim_debug_reg[index].reg,
				mxim_debug_reg[index].val);
	}

	return len;
}

static ssize_t mxim_debug_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int reg = 0;
	unsigned int value = 0;
	unsigned char prev = 0;
	unsigned char curr = 0;
	int ret;

	ret = sscanf(buf, "%x%x", &reg, &value);
	if (!ret) {
		pr_info("Failed to convert user data");
		goto error;
	}

	mxim_debug_i2c_read((unsigned char)reg, &prev);
	mxim_debug_i2c_write((unsigned char)reg, (unsigned char)value);
	mxim_debug_i2c_read((unsigned char)reg, &curr);

error:
	return size;
}

static DEVICE_ATTR(reg, 0644,
		mxim_debug_reg_show, mxim_debug_reg_store);

static ssize_t mxim_debug_opcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char data[OPCODE_READ_MAX_LENGTH] = { 0, };
	int index;
	int len = 0;

	max77978_opcode_read(mxim_pdev->client, OPCODE_MAX_LENGTH, data);
	for (index = 0; index < OPCODE_READ_MAX_LENGTH; index++)
		len += sysfs_emit_at(buf, len, "0x%02x ", data[index]);

	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static ssize_t mxim_debug_opcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char data[OPCODE_WRITE_MAX_LENGTH] = { 0, };
	int ret;

	ret = sscanf(buf,
			"%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
			&data[0], /* AP_DATAOUT0 */
			&data[1], &data[2], &data[3], &data[4],
			&data[5], &data[6], &data[7], &data[8],
			&data[9], &data[10], &data[11], &data[12],
			&data[13], &data[14], &data[15], &data[16],
			&data[17], &data[18], &data[19], &data[20],
			&data[21], &data[22], &data[23], &data[24],
			&data[25], &data[26], &data[27], &data[28],
			&data[29], &data[30], &data[31], &data[32]);
	if (!ret) {
		pr_info("Failed to convert user data");
		goto error;
	}

	max77978_opcode_write(mxim_pdev->client, OPCODE_MAX_LENGTH, data);

error:
	return size;
}

static DEVICE_ATTR(opcode, 0644,
		mxim_debug_opcode_show, mxim_debug_opcode_store);

static struct attribute *mxim_debug_attr[] = {
	&dev_attr_opcode.attr,
	&dev_attr_reg.attr,
	NULL,
};

static struct attribute_group mxim_debug_attr_grp = {
	.attrs = mxim_debug_attr,
};

static int mxim_debug_open(struct inode *inode, struct file *filep)
{
	return 0;
}

static long mxim_debug_ioctl_handler(struct file *file, unsigned int cmd, unsigned int arg, void __user *argp)
{
	unsigned char reg = 0;
	unsigned char val = 0;
	int ret = -EINVAL;

	mutex_lock(&mxim_pdev->lock);

	switch (cmd) {
	case MXIM_DEBUG_OPCODE_WRITE:
		/* copy wdata from user */
		if (copy_from_user(mxim_pdev->opcode_wdata, argp, OPCODE_WRITE_MAX_LENGTH)) {
			pr_info("Failed to copy data buffer from user");
			ret = -EFAULT;
			goto error;
		}

		/* OPCode */
		mxim_debug_i2c_write(OPCODE_WRITE_START_ADDR, mxim_pdev->opcode_wdata[0]);

		/* AP OUT Data */
		ret = mxim_debug_i2c_burst_write(
				(OPCODE_WRITE_START_ADDR + 1),
				(OPCODE_WRITE_MAX_LENGTH - 1),
				&mxim_pdev->opcode_wdata[1]);
		break;
	case MXIM_DEBUG_OPCODE_READ:
		/* OPCode */
		mxim_debug_i2c_read(OPCODE_READ_START_ADDR, &mxim_pdev->opcode_rdata[0]);

		/* AP IN Data */
		ret = mxim_debug_i2c_burst_read(
				(OPCODE_READ_START_ADDR + 1),
				(OPCODE_READ_MAX_LENGTH - 1),
				&mxim_pdev->opcode_rdata[1]);

		/* copy rdata to user */
		if (copy_to_user(argp, mxim_pdev->opcode_rdata, OPCODE_READ_MAX_LENGTH)) {
			pr_info("Failed to copy data buffer to user");
			ret = -EFAULT;
			goto error;
		}
		break;
	case MXIM_DEBUG_REG_WRITE:
		if (copy_from_user(&reg, argp, sizeof(reg))) {
			pr_info("Failed to copy data buffer from user");
			ret = -EFAULT;
			goto error;
		}
		reg = (unsigned char)(reg & 0xFF);
		val = (unsigned char)((reg & 0xFF00) >> 8);
		ret = mxim_debug_i2c_write(reg, val);
		break;
	case MXIM_DEBUG_REG_READ:
		if (copy_from_user(&reg, argp, sizeof(reg))) {
			pr_info("Failed to copy data buffer from user");
			ret = -EFAULT;
			goto error;
		}
		mxim_debug_i2c_read(reg & 0xFF, &val);
		ret = val;
		if (copy_to_user(argp, &val, sizeof(val)))
			goto error;
		break;
	case MXIM_DEBUG_REG_DUMP:
		break;
	default:
		break;
	}
error:
	mutex_unlock(&mxim_pdev->lock);
	return ret;
}

static long mxim_debug_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return mxim_debug_ioctl_handler(file, cmd, arg,
			(void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long mxim_debug_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return mxim_debug_ioctl_handler(file, cmd, arg,
			(void __user *)(unsigned long)arg);
}
#endif

static ssize_t mxim_debug_read(struct file *filep, char __user *buf, size_t count, loff_t *ppos)
{
	size_t to_copy;

	if (!mxim_pdev)
		return -EFAULT;

	to_copy = min(count, (size_t)OPCODE_READ_MAX_LENGTH);
	if (copy_to_user(buf, mxim_pdev->opcode_rdata, to_copy)) {
		pr_info("Failed to copy user buffer");
		return -EFAULT;
	}

	return to_copy;
}

static ssize_t mxim_debug_write(struct file *filep, const char __user *buf, size_t count, loff_t *ppos)
{
	size_t to_copy;

	if (!mxim_pdev)
		return -EFAULT;

	to_copy = min(count, (size_t)OPCODE_READ_MAX_LENGTH);
	if (copy_from_user(mxim_pdev->opcode_wdata, buf, to_copy)) {
		pr_info("Failed to copy user buffer");
		return -EFAULT;
	}

	return to_copy;
}

void mxim_debug_set_i2c_client(struct i2c_client *client)
{
	if (mxim_pdev) {
		mxim_pdev->client = client;
		pr_info("Set i2c_client. %p", mxim_pdev->client);
	} else
		pr_info("Failed to set i2c_client.");
}
EXPORT_SYMBOL_GPL(mxim_debug_set_i2c_client);

static const struct file_operations mxim_debug_fops = {
	.owner			= THIS_MODULE,
	.open			= mxim_debug_open,
	.release		= NULL,
	.unlocked_ioctl = mxim_debug_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= mxim_debug_compat_ioctl,
#endif /* CONFIG_COMPAT */
	.read			= mxim_debug_read,
	.write			= mxim_debug_write,
	.mmap			= NULL,
	.poll			= NULL,
	.fasync			= NULL,
	.llseek			= NULL,
};

static struct miscdevice mxim_debug_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mxim_dev",
	.fops = &mxim_debug_fops,
};

void mxim_debug_exit(void)
{
	if (mxim_pdev) {
		if (mxim_pdev->dev) {
			sysfs_remove_group(&mxim_pdev->dev->kobj, &mxim_debug_attr_grp);
			device_destroy(mxim_pdev->class, 1);
		}
		if (mxim_pdev->class)
			class_destroy(mxim_pdev->class);
		mutex_destroy(&mxim_pdev->lock);
		kfree(mxim_pdev);
		mxim_pdev = NULL;
	}

	misc_deregister(&mxim_debug_miscdev);
}
EXPORT_SYMBOL_GPL(mxim_debug_exit);

int mxim_debug_init(void)
{
	struct mxim_debug_pdev *pdev;
	int ret;

	ret = misc_register(&mxim_debug_miscdev);
	if (ret) {
		pr_err("Failed to register miscdevice\n");
		return ret;
	}

	pdev = kzalloc(sizeof(struct mxim_debug_pdev), GFP_KERNEL);
	if (!pdev) {
		pr_err("Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	mxim_pdev = pdev;

	mutex_init(&pdev->lock);

	pdev->class = class_create("mxim");
	if (IS_ERR(pdev->class)) {
		pr_err("Failed to create class\n");
		ret = PTR_ERR(pdev->class);
		pdev->class = NULL;
		goto err_class;
	}

	pdev->dev = device_create(pdev->class, NULL, 1, NULL, "debug0");
	if (IS_ERR(pdev->dev)) {
		pr_err("Failed to create device\n");
		ret = PTR_ERR(pdev->dev);
		pdev->dev = NULL;
		goto err_device;
	}

	ret = sysfs_create_group(&pdev->dev->kobj, &mxim_debug_attr_grp);
	if (ret) {
		pr_err("Failed to create sysfs group: %d\n", ret);
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	device_destroy(pdev->class, 1);
err_device:
	class_destroy(pdev->class);
err_class:
	mutex_destroy(&pdev->lock);
	kfree(pdev);
	mxim_pdev = NULL;
err_kzalloc:
	misc_deregister(&mxim_debug_miscdev);
	return ret;
}
EXPORT_SYMBOL_GPL(mxim_debug_init);

MODULE_DESCRIPTION("max77978 debug driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
