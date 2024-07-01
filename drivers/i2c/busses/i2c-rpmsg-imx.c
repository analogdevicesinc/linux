/*
 * Copyright 2019 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/* The i2c-rpmsg transfer protocol:
 *
 *   +---------------+-------------------------------+
 *   |  Byte Offset  |            Content            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       0       |           Category            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |     1 ~ 2     |           Version             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       3       |             Type              |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       4       |           Command             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       5       |           Priority            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       6       |           Reserved1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       7       |           Reserved2           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       8       |           Reserved3           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       9       |           Reserved4           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       10      |            BUS ID             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       11      |         Return Value          |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    12 ~ 13    |            Address            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    14 ~ 15    |             Flags             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    16 ~ 17    |           Data Len            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    18 ~ 495   |        478 Bytes Data         |
 *   +---------------+---+---+---+---+---+---+---+---+
 *
 * The definition of Return Value:
 * 0x00 = Success
 * 0x01 = Failed
 * 0x02 = Invalid parameter
 * 0x03 = Invalid message
 * 0x04 = Operate in invalid state
 * 0x05 = Memory allocation failed
 * 0x06 = Timeout when waiting for an event
 * 0x07 = Cannot add to list as node already in another list
 * 0x08 = Cannot remove from list as node not in list
 * 0x09 = Transfer timeout
 * 0x0A = Transfer failed due to peer core not ready
 * 0x0B = Transfer failed due to communication failure
 * 0x0C = Cannot find service for a request/notification
 * 0x0D = Service version cannot support the request/notification
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>

#define I2C_RPMSG_MAX_BUF_SIZE			478
#define I2C_RPMSG_TIMEOUT			500 /* unit: ms */

#define I2C_RPMSG_CATEGORY			0x09
#define I2C_RPMSG_VERSION			0x0001
#define I2C_RPMSG_TYPE_REQUEST			0x00
#define I2C_RPMSG_TYPE_RESPONSE			0x01
#define I2C_RPMSG_COMMAND_READ			0x00
#define I2C_RPMSG_COMMAND_WRITE			0x01
#define I2C_RPMSG_PRIORITY			0x01

#define I2C_RPMSG_M_STOP			0x0200

struct i2c_rpmsg_devtype_data {
	unsigned int max_buf_size;
	bool dynamic_buffer_support;
};

struct i2c_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 bus_id;
	u8 ret_val;
	u16 addr;
	u16 flags;
	u16 len;
	u8 buf[I2C_RPMSG_MAX_BUF_SIZE];
} __packed __aligned(1);

struct i2c_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct i2c_rpmsg_msg *msg;
	const struct i2c_rpmsg_devtype_data *devtype_data;
	struct completion cmd_complete;
	struct mutex lock;
	struct workqueue_struct *wq;
	struct work_struct work;

	u8 bus_id;
	u16 addr;
};

static struct i2c_rpmsg_info i2c_rpmsg;

struct imx_rpmsg_i2c_data {
	struct i2c_adapter adapter;
	const struct i2c_rpmsg_devtype_data *devtype_data;
};

static int i2c_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			   void *priv, u32 src)
{
	struct i2c_rpmsg_msg *msg = (struct i2c_rpmsg_msg *)data;

	if (msg->header.type != I2C_RPMSG_TYPE_RESPONSE)
		return -EINVAL;

	if (msg->bus_id != i2c_rpmsg.bus_id || msg->addr != i2c_rpmsg.addr) {
		dev_err(&rpdev->dev,
		"expected bus_id:%d, addr:%2x, received bus_id:%d, addr:%2x\n",
		i2c_rpmsg.bus_id, i2c_rpmsg.addr, msg->bus_id, msg->addr);
		return -EINVAL;
	}

	if (msg->len > I2C_RPMSG_MAX_BUF_SIZE) {
		dev_err(&rpdev->dev,
		"%s failed: data length greater than %d, len=%d\n",
		__func__, I2C_RPMSG_MAX_BUF_SIZE, msg->len);
		return -EINVAL;
	}

	/* Receive Success */
	i2c_rpmsg.msg = msg;

	complete(&i2c_rpmsg.cmd_complete);

	return 0;
}

static int rpmsg_xfer(struct i2c_rpmsg_msg *rmsg, struct i2c_rpmsg_info *info)
{
	int ret = 0;
	int xfer_len = sizeof(struct i2c_rpmsg_msg) - I2C_RPMSG_MAX_BUF_SIZE;

	if (info->devtype_data->dynamic_buffer_support)
		xfer_len += rmsg->len;
	else
		xfer_len += info->devtype_data->max_buf_size;

	ret = rpmsg_send(info->rpdev->ept, (void *)rmsg, xfer_len);
	if (ret < 0) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(I2C_RPMSG_TIMEOUT));
	if (!ret) {
		dev_err(&info->rpdev->dev, "%s failed: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	if (info->msg->ret_val) {
		dev_dbg(&info->rpdev->dev,
			"%s failed: %d\n", __func__, info->msg->ret_val);
		return -(info->msg->ret_val);
	}

	return 0;
}

static int i2c_rpmsg_read(struct i2c_msg *msg, struct i2c_rpmsg_info *info,
						int bus_id, bool is_last)
{
	int ret;
	struct i2c_rpmsg_msg rmsg;

	if (!info->rpdev)
		return -EINVAL;

	if (msg->len > info->devtype_data->max_buf_size) {
		dev_err(&info->rpdev->dev,
		"%s failed: data length greater than %d, len=%d\n",
		__func__, info->devtype_data->max_buf_size, msg->len);
		return -EINVAL;
	}

	memset(&rmsg, 0, sizeof(struct i2c_rpmsg_msg));
	rmsg.header.cate = I2C_RPMSG_CATEGORY;
	rmsg.header.major = I2C_RPMSG_VERSION;
	rmsg.header.minor = I2C_RPMSG_VERSION >> 8;
	rmsg.header.type = I2C_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd = I2C_RPMSG_COMMAND_READ;
	rmsg.header.reserved[0] = I2C_RPMSG_PRIORITY;
	rmsg.bus_id = bus_id;
	rmsg.ret_val = 0;
	rmsg.addr = msg->addr;
	if (is_last)
		rmsg.flags = msg->flags | I2C_RPMSG_M_STOP;
	else
		rmsg.flags = msg->flags;
	rmsg.len = (msg->len);

	reinit_completion(&info->cmd_complete);

	ret = rpmsg_xfer(&rmsg, info);
	if (ret)
		return ret;

	if (!info->msg ||
	    (info->msg->len != msg->len)) {
		dev_err(&info->rpdev->dev,
					"%s failed: %d\n", __func__, -EPROTO);
		return -EPROTO;
	}

	memcpy(msg->buf, info->msg->buf, info->msg->len);

	return msg->len;
}

static int i2c_rpmsg_write(struct i2c_msg *msg, struct i2c_rpmsg_info *info,
						int bus_id, bool is_last)
{
	int i, ret;
	struct i2c_rpmsg_msg rmsg;

	if (!info || !info->rpdev)
		return -EINVAL;

	if (msg->len > info->devtype_data->max_buf_size) {
		dev_err(&info->rpdev->dev,
		"%s failed: data length greater than %d, len=%d\n",
		__func__, info->devtype_data->max_buf_size, msg->len);
		return -EINVAL;
	}

	memset(&rmsg, 0, sizeof(struct i2c_rpmsg_msg));
	rmsg.header.cate = I2C_RPMSG_CATEGORY;
	rmsg.header.major = I2C_RPMSG_VERSION;
	rmsg.header.minor = I2C_RPMSG_VERSION >> 8;
	rmsg.header.type = I2C_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd = I2C_RPMSG_COMMAND_WRITE;
	rmsg.header.reserved[0] = I2C_RPMSG_PRIORITY;
	rmsg.bus_id = bus_id;
	rmsg.ret_val = 0;
	rmsg.addr = msg->addr;
	if (is_last)
		rmsg.flags = msg->flags | I2C_RPMSG_M_STOP;
	else
		rmsg.flags = msg->flags;
	rmsg.len = msg->len;

	for (i = 0; i < rmsg.len; i++)
		rmsg.buf[i] = msg->buf[i];

	reinit_completion(&info->cmd_complete);

	ret = rpmsg_xfer(&rmsg, info);
	if (ret)
		return ret;

	return ret;
}

static int i2c_rpbus_xfer(struct i2c_adapter *adapter,
			  struct i2c_msg *msgs, int num)
{
	struct imx_rpmsg_i2c_data *rdata =
		container_of(adapter, struct imx_rpmsg_i2c_data, adapter);
	struct i2c_msg *pmsg;
	int i, ret;
	bool is_last = false;

	mutex_lock(&i2c_rpmsg.lock);

	for (i = 0; i < num; i++) {
		if (i == num - 1)
			is_last = true;

		pmsg = &msgs[i];

		i2c_rpmsg.devtype_data = rdata->devtype_data;
		i2c_rpmsg.bus_id = rdata->adapter.nr;
		i2c_rpmsg.addr = pmsg->addr;

		if (pmsg->flags & I2C_M_RD) {
			ret = i2c_rpmsg_read(pmsg, &i2c_rpmsg,
						rdata->adapter.nr, is_last);
			if (ret < 0) {
				mutex_unlock(&i2c_rpmsg.lock);
				return ret;
			}

			pmsg->len = ret;
		} else {
			ret = i2c_rpmsg_write(pmsg, &i2c_rpmsg,
						rdata->adapter.nr, is_last);
			if (ret < 0) {
				mutex_unlock(&i2c_rpmsg.lock);
				return ret;
			}
		}
	}

	mutex_unlock(&i2c_rpmsg.lock);
	return num;
}

static u32 i2c_rpbus_func(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL
		| I2C_FUNC_SMBUS_READ_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_rpbus_algorithm = {
	.master_xfer = i2c_rpbus_xfer,
	.functionality = i2c_rpbus_func,
};

static const struct i2c_adapter_quirks i2c_rpbus_quirks = {
	.max_write_len = I2C_RPMSG_MAX_BUF_SIZE,
	.max_read_len = I2C_RPMSG_MAX_BUF_SIZE,
};

static int i2c_rpbus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rpmsg_i2c_data *rdata;
	struct i2c_adapter *adapter;
	int ret;

	if (!i2c_rpmsg.rpdev)
		return -EPROBE_DEFER;

	rdata = devm_kzalloc(&pdev->dev, sizeof(*rdata), GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	rdata->devtype_data = of_device_get_match_data(&pdev->dev);
	adapter = &rdata->adapter;
	/* setup i2c adapter description */
	adapter->owner = THIS_MODULE;
	adapter->class = I2C_CLASS_HWMON;
	adapter->algo = &i2c_rpbus_algorithm;
	adapter->dev.parent = dev;
	adapter->dev.of_node = np;
	adapter->nr = of_alias_get_id(np, "i2c");
	/*
	 * The driver will send the adapter->nr as BUS ID to the other
	 * side, and the other side will check the BUS ID to see whether
	 * the BUS has been registered. If there is alias id for this
	 * virtual adapter, linux kernel will automatically allocate one
	 * id which might be not the same number used in the other side,
	 * cause i2c slave probe failure under this virtual I2C bus.
	 * So let's add a BUG_ON to catch this issue earlier.
	 */
	BUG_ON(adapter->nr < 0);
	adapter->quirks = &i2c_rpbus_quirks;
	snprintf(rdata->adapter.name, sizeof(rdata->adapter.name), "%s",
							"i2c-rpmsg-adapter");
	platform_set_drvdata(pdev, rdata);

	ret = i2c_add_adapter(&rdata->adapter);
	if (ret < 0) {
		dev_err(dev, "failed to add I2C adapter: %d\n", ret);
		return ret;
	}

	dev_info(dev, "add I2C adapter %s successfully\n", rdata->adapter.name);

	return 0;
}

static void i2c_rpbus_remove(struct platform_device *pdev)
{
	struct imx_rpmsg_i2c_data *rdata = platform_get_drvdata(pdev);

	i2c_del_adapter(&rdata->adapter);
}

static struct i2c_rpmsg_devtype_data i2c_rpmsg_devtype_data = {
	.max_buf_size = 16,
	.dynamic_buffer_support = false,
};

static struct i2c_rpmsg_devtype_data i2c_rpmsg_v2_devtype_data = {
	.max_buf_size = I2C_RPMSG_MAX_BUF_SIZE,
	.dynamic_buffer_support = true,
};

static const struct of_device_id imx_rpmsg_i2c_dt_ids[] = {
	{ .compatible = "fsl,i2c-rpbus", .data = &i2c_rpmsg_devtype_data},
	{ .compatible = "fsl,i2c-rpbus-v2", .data = &i2c_rpmsg_v2_devtype_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_i2c_dt_ids);

static struct platform_driver imx_rpmsg_i2c_driver = {
	.driver = {
		.name	= "imx_rpmsg_i2c",
		.of_match_table = imx_rpmsg_i2c_dt_ids,
	},
	.probe		= i2c_rpbus_probe,
	.remove		= i2c_rpbus_remove
};

static void i2c_rpmsg_work(struct work_struct *work)
{
	platform_driver_register(&imx_rpmsg_i2c_driver);
};

static int i2c_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int ret = 0;

	if (!rpdev) {
		dev_err(&rpdev->dev, "%s failed, rpdev=NULL\n", __func__);
		return -EINVAL;
	}

	i2c_rpmsg.rpdev = rpdev;

	mutex_init(&i2c_rpmsg.lock);
	init_completion(&i2c_rpmsg.cmd_complete);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	i2c_rpmsg.wq = create_singlethread_workqueue("rpmsg-i2c");
	if (!i2c_rpmsg.wq) {
		dev_err(&rpdev->dev, "workqueue create failed\n");
		return -ENOMEM;
	}
	INIT_WORK(&i2c_rpmsg.work, i2c_rpmsg_work);
	queue_work(i2c_rpmsg.wq, &i2c_rpmsg.work);

	return ret;
}

static void i2c_rpmsg_remove(struct rpmsg_device *rpdev)
{
	i2c_rpmsg.rpdev = NULL;

	cancel_work_sync(&i2c_rpmsg.work);
	flush_workqueue(i2c_rpmsg.wq);
	if (i2c_rpmsg.wq)
		destroy_workqueue(i2c_rpmsg.wq);

	platform_driver_unregister(&imx_rpmsg_i2c_driver);
	dev_info(&rpdev->dev, "i2c rpmsg driver is removed\n");
}

static struct rpmsg_device_id i2c_rpmsg_id_table[] = {
	{ .name	= "rpmsg-i2c-channel" },
	{ },
};

static struct rpmsg_driver i2c_rpmsg_driver = {
	.drv.name	= "i2c-rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= i2c_rpmsg_id_table,
	.probe		= i2c_rpmsg_probe,
	.remove		= i2c_rpmsg_remove,
	.callback	= i2c_rpmsg_cb,
};

static int __init imx_rpmsg_i2c_driver_init(void)
{
	int ret;

	ret = register_rpmsg_driver(&i2c_rpmsg_driver);

	return ret;
}
subsys_initcall(imx_rpmsg_i2c_driver_init);

MODULE_AUTHOR("Clark Wang<xiaoning.wang@nxp.com>");
MODULE_DESCRIPTION("Driver for i2c over rpmsg");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-rpbus");
