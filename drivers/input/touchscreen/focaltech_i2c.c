// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "focaltech_core.h"

#define I2C_RETRY_NUMBER                    3

int fts_read(struct fts_ts_data *ts_data, u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	int ret = 0;
	int i = 0;
	struct i2c_msg msg_list[2];
	struct i2c_msg *msg = NULL;
	int msg_num = 0;

	/* must have data when read */
	if (!ts_data || !ts_data->client || !data || !datalen
		|| (datalen > FTS_MAX_BUS_BUF) || (cmdlen > FTS_MAX_BUS_BUF)) {
		dev_err(&ts_data->client->dev, "fts_data/client/cmdlen(%d)/data/datalen(%d) is invalid",
				cmdlen, datalen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	memset(&msg_list[0], 0, sizeof(struct i2c_msg));
	memset(&msg_list[1], 0, sizeof(struct i2c_msg));
	memcpy(ts_data->bus_tx_buf, cmd, cmdlen);
	msg_list[0].addr = ts_data->client->addr;
	msg_list[0].flags = 0;
	msg_list[0].len = cmdlen;
	msg_list[0].buf = ts_data->bus_tx_buf;
	msg_list[1].addr = ts_data->client->addr;
	msg_list[1].flags = I2C_M_RD;
	msg_list[1].len = datalen;
	msg_list[1].buf = ts_data->bus_rx_buf;
	if (cmd && cmdlen) {
		msg = &msg_list[0];
		msg_num = 2;
	} else {
		msg = &msg_list[1];
		msg_num = 1;
	}

	for (i = 0; i < I2C_RETRY_NUMBER; i++) {
		ret = i2c_transfer(ts_data->client->adapter, msg, msg_num);
		if (ret < 0) {
			dev_err(&ts_data->client->dev, "i2c_transfer(read) fail,ret:%d", ret);
		} else {
			memcpy(data, ts_data->bus_rx_buf, datalen);
			break;
		}
	}

	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

int fts_write(struct fts_ts_data *ts_data, u8 *writebuf, u32 writelen)
{
	int ret = 0;
	int i = 0;
	struct i2c_msg msgs;

	if (!ts_data || !ts_data->client || !writebuf || !writelen
		|| (writelen > FTS_MAX_BUS_BUF)) {
		dev_err(&ts_data->client->dev,
				"fts_data/client/data/datalen(%d) is invalid", writelen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	memset(&msgs, 0, sizeof(struct i2c_msg));
	memcpy(ts_data->bus_tx_buf, writebuf, writelen);
	msgs.addr = ts_data->client->addr;
	msgs.flags = 0;
	msgs.len = writelen;
	msgs.buf = ts_data->bus_tx_buf;
	for (i = 0; i < I2C_RETRY_NUMBER; i++) {
		ret = i2c_transfer(ts_data->client->adapter, &msgs, 1);
		if (ret < 0)
			dev_err(&ts_data->client->dev, "i2c_transfer(write) fail,ret:%d", ret);
		else
			break;
	}
	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

int fts_read_reg(struct fts_ts_data *ts_data, u8 addr, u8 *value)
{
	return fts_read(ts_data, &addr, 1, value, 1);
}

int fts_write_reg(struct fts_ts_data *ts_data, u8 addr, u8 value)
{
	u8 buf[2] = { 0 };

	buf[0] = addr;
	buf[1] = value;
	return fts_write(ts_data, buf, sizeof(buf));
}

int fts_read_touchdata_i2c(struct fts_ts_data *ts_data, u8 *buf)
{
	int ret = 0;
	u32 touch_max_size = 0;
	u32 max_touch_num = ts_data->pdata->max_touch_number;
	u8 event = 0xFF;

	ts_data->touch_addr = 0x01;
	ret = fts_read(ts_data, &ts_data->touch_addr, 1, buf, ts_data->touch_size);
	if (ret < 0) {
		dev_err(&ts_data->client->dev, "read touchdata fails,ret:%d", ret);
		return ret;
	}

	event = (buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F;
	if (event == TOUCH_DEFAULT) {
		if (buf[ts_data->touch_size - 1] != 0xFF)
			touch_max_size = max_touch_num * FTS_ONE_TCH_LEN + 2;
	} else if (event == TOUCH_PROTOCOL_v2) {
		touch_max_size = (buf[FTS_TOUCH_E_NUM] & 0x0F) * FTS_ONE_TCH_LEN_V2 + 4;
	} else if (event == TOUCH_EXTRA_MSG) {
		touch_max_size = (buf[FTS_TOUCH_E_NUM] & 0x0F) * FTS_ONE_TCH_LEN +
						4 + ((buf[2] << 8) + buf[3]);
		if (touch_max_size > FTS_MAX_TOUCH_BUF)
			touch_max_size = FTS_MAX_TOUCH_BUF;
	}

	if (touch_max_size > ts_data->touch_size) {
		ts_data->ta_size = touch_max_size;
		ts_data->touch_addr += ts_data->touch_size;
		ret = fts_read(ts_data, &ts_data->touch_addr, 1, buf + ts_data->touch_size,
					touch_max_size - ts_data->touch_size);
		if (ret < 0) {
			dev_err(&ts_data->client->dev, "read touchdata2 fails,ret:%d", ret);
			return ret;
		}
	}

	return 0;
}

static int fts_ts_probe(struct i2c_client *client)
{
	int ret = 0;
	struct fts_ts_data *ts_data = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* malloc memory for global struct variable */
	ts_data = kzalloc(sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data)
		return -ENOMEM;

	ts_data->client = client;
	ts_data->dev = &client->dev;
	ts_data->log_level = 1;
	ts_data->bus_type = BUS_TYPE_I2C;
	ts_data->bus_ver = BUS_VER_DEFAULT;
	i2c_set_clientdata(client, ts_data);

	ret = fts_ts_probe_entry(ts_data);
	if (ret) {
		dev_err(&ts_data->client->dev, "FTS Touch Screen(I2C BUS) driver probe fail");
		i2c_set_clientdata(client, NULL);
		kfree_safe(ts_data);
		return ret;
	}

	dev_info(&ts_data->client->dev, "FTS Touch Screen(I2C BUS) driver prboe successfully");
	return 0;
}

static void fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *ts_data = i2c_get_clientdata(client);

	dev_info(&ts_data->client->dev, "FTS Touch Screen(I2C BUS) driver remove...");
	if (ts_data) {
		fts_ts_remove_entry(ts_data);
		i2c_set_clientdata(client, NULL);
		kfree_safe(ts_data);
	}

}

static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	dev_dbg(&ts_data->client->dev, "FTS enters into pm_suspend");
	ts_data->pm_suspend = true;
	reinit_completion(&ts_data->pm_completion);
	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	dev_dbg(&ts_data->client->dev, "FTS resumes from pm_suspend");
	ts_data->pm_suspend = false;
	complete(&ts_data->pm_completion);
	return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};

static const struct i2c_device_id fts_ts_id[] = {
	{FTS_DRIVER_NAME, 0},
	{},
};
static const struct of_device_id fts_dt_match[] = {
	{.compatible = "focaltech,ft3518u", },
	{},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct i2c_driver fts_ts_i2c_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &fts_dev_pm_ops,
		.of_match_table = of_match_ptr(fts_dt_match),
	},
	.id_table = fts_ts_id,
};

static int __init fts_ts_i2c_init(void)
{
	return i2c_add_driver(&fts_ts_i2c_driver);
}

static void __exit fts_ts_i2c_exit(void)
{
	i2c_del_driver(&fts_ts_i2c_driver);
}

module_init(fts_ts_i2c_init);
module_exit(fts_ts_i2c_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver(I2C)");
MODULE_LICENSE("GPL");
