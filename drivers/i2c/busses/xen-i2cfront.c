/*
 * Copyright 2018 NXP
 *
 * Peng Fan <peng.fan@nxp.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/io.h>

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/grant_table.h>
#include <xen/events.h>
#include <xen/page.h>

#include <xen/interface/io/i2cif.h>

#define GRANT_INVALID_REF	0

struct i2cfront_info {
	spinlock_t lock;
	struct mutex xferlock;
	struct i2c_adapter adapter;
	struct xenbus_device *i2cdev;
	int i2c_ring_ref;
	struct i2cif_front_ring i2c_ring;
	unsigned int evtchn;
	unsigned int irq;
	struct completion completion;
	struct i2cif_request *req;
	struct i2cif_response *res;
};

static void i2cfront_destroy_rings(struct i2cfront_info *info)
{
	if (info->irq)
		unbind_from_irqhandler(info->irq, info);
	info->irq = 0;

	if (info->i2c_ring_ref != GRANT_INVALID_REF) {
		gnttab_end_foreign_access(info->i2c_ring_ref, 0,
					  (unsigned long)info->i2c_ring.sring);
		info->i2c_ring_ref = GRANT_INVALID_REF;
	}
	info->i2c_ring.sring = NULL;
}

static int i2cfront_do_req(struct i2c_adapter *adapter, struct i2c_msg *msg,
			   int num)
{
	struct i2cfront_info *info = i2c_get_adapdata(adapter);
	struct i2cif_request *req;
	struct i2cif_response *res;
	int notify;
	int ret;
	RING_IDX i, rp;
	int more_to_do = 0;
	unsigned long flags;
	int index;

	mutex_lock(&info->xferlock);
	req = RING_GET_REQUEST(&info->i2c_ring, info->i2c_ring.req_prod_pvt);

	for (index = 0; index < num; index++) {
		req->msg[index].addr = msg[index].addr;
		req->msg[index].len = msg[index].len;
		req->msg[index].flags = 0;
		if (msg[index].flags & I2C_M_RD)
			req->msg[index].flags |= I2CIF_M_RD;
		if (msg[index].flags & I2C_M_TEN)
			req->msg[index].flags |= I2CIF_M_TEN;
		if (msg[index].flags & I2C_M_RECV_LEN)
			req->msg[index].flags |= I2CIF_M_RECV_LEN;
		if (msg[index].flags & I2C_M_NO_RD_ACK)
			req->msg[index].flags |= I2CIF_M_NO_RD_ACK;
		if (msg[index].flags & I2C_M_IGNORE_NAK)
			req->msg[index].flags |= I2CIF_M_IGNORE_NAK;
		if (msg[index].flags & I2C_M_REV_DIR_ADDR)
			req->msg[index].flags |= I2CIF_M_REV_DIR_ADDR;
		if (msg[index].flags & I2C_M_NOSTART)
			req->msg[index].flags |= I2CIF_M_NOSTART;
		if (msg[index].flags & I2C_M_STOP)
			req->msg[index].flags |= I2CIF_M_STOP;
	}

	req->num_msg = num;
	req->is_smbus = false;

	if ((num == 2) && !(msg[0].flags & I2C_M_RD) &&
	    (msg[1].flags & I2C_M_RD)) {
		memcpy(req->write_buf, msg[0].buf,
		       min_t(int, msg[0].len, I2CIF_BUF_LEN));
	} else if (num == 1) {
		if (!(msg->flags & I2C_M_RD))
			memcpy(req->write_buf, msg->buf,
			       min_t(int, msg->len, I2CIF_BUF_LEN));
	} else {
		dev_err(&adapter->dev, "%s not supported\n", __func__);
		return -EIO;
	}

	spin_lock(&info->lock);
	info->i2c_ring.req_prod_pvt++;
	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&info->i2c_ring, notify);
	spin_unlock(&info->lock);
	if (notify)
		notify_remote_via_irq(info->irq);

	wait_for_completion(&info->completion);

	spin_lock_irqsave(&info->lock, flags);
	rp = info->i2c_ring.sring->rsp_prod;
	rmb(); /* ensure we see queued responses up to "rp" */

	ret = -EIO;
	for (i = info->i2c_ring.rsp_cons; i != rp; i++) {
		res = RING_GET_RESPONSE(&info->i2c_ring, i);
		if ((num == 2) && !(msg[0].flags & I2C_M_RD) &&
		    (msg[1].flags & I2C_M_RD)) {
			memcpy(msg[1].buf, res->read_buf,
			       min_t(int, msg[1].len, I2CIF_BUF_LEN));
		} else if (num == 1) {
			if (!(msg->flags & I2C_M_RD))
			memcpy(msg->buf, res->read_buf,
			       min_t(int, msg->len, I2CIF_BUF_LEN));
		}

		ret = res->result;
	}

	info->i2c_ring.rsp_cons = i;

	if (i != info->i2c_ring.req_prod_pvt)
		RING_FINAL_CHECK_FOR_RESPONSES(&info->i2c_ring, more_to_do);
	else
		info->i2c_ring.sring->rsp_event = i + 1;

	spin_unlock_irqrestore(&info->lock, flags);

	mutex_unlock(&info->xferlock);

	return ret;
}

int i2cfront_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct i2cfront_info *info = i2c_get_adapdata(adapter);
	int ret, i;

	if (!info || !info->i2cdev) {
		dev_err(&adapter->dev, "Not initialized\n");
		return -EIO;
	}

	if (info->i2cdev->state != XenbusStateConnected) {
		dev_err(&adapter->dev, "Not connected\n");
		return -EIO;
	}

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD) {
			ret = i2cfront_do_req(adapter, &msgs[i], 1);
		} else if ((i + 1 < num) && (msgs[i + 1].flags & I2C_M_RD) &&
			(msgs[i].addr == msgs[i + 1].addr)) {
			ret = i2cfront_do_req(adapter, &msgs[i], 2);
			i++;
		} else {
			ret = i2cfront_do_req(adapter, &msgs[i], 1);
		}

		if (ret < 0)
			goto err;
	}
err:
	return (ret < 0) ? ret : num;
}

static int i2cfront_smbus_xfer(struct i2c_adapter *adapter, u16 addr,
        unsigned short flags, char read_write,
        u8 command, int size, union i2c_smbus_data *data)
{
	struct i2cfront_info *info = i2c_get_adapdata(adapter);
	struct i2cif_response *res;
	struct i2cif_request *req;
	int more_to_do = 0;
	RING_IDX i, rp;
	int notify;
	int ret;

	if (!info || !info->i2cdev) {
		dev_err(&adapter->dev, "Not initialized\n");
		return -EIO;
	}

	if (info->i2cdev->state != XenbusStateConnected) {
		dev_err(&adapter->dev, "Not connected\n");
		return -EIO;
	}

	mutex_lock(&info->xferlock);
	req = RING_GET_REQUEST(&info->i2c_ring, info->i2c_ring.req_prod_pvt);

	req->is_smbus = true;
	req->addr = addr;
	req->flags = flags;
	req->read_write = read_write;
	req->command = command;
	req->protocol = size;
	if (data != NULL)
		memcpy(&req->write_buf, data, sizeof(union i2c_smbus_data));

	spin_lock(&info->lock);
	info->i2c_ring.req_prod_pvt++;
	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&info->i2c_ring, notify);
	spin_unlock(&info->lock);
	if (notify)
		notify_remote_via_irq(info->irq);

	wait_for_completion(&info->completion);

	spin_lock_irqsave(&info->lock, flags);
	rp = info->i2c_ring.sring->rsp_prod;
	rmb(); /* ensure we see queued responses up to "rp" */

	ret = -EIO;
	for (i = info->i2c_ring.rsp_cons; i != rp; i++) {
		res = RING_GET_RESPONSE(&info->i2c_ring, i);

		if (data != NULL && read_write == I2C_SMBUS_READ)
			memcpy(data, &res->read_buf, sizeof(union i2c_smbus_data));

		ret = res->result;
	}

	info->i2c_ring.rsp_cons = i;

	if (i != info->i2c_ring.req_prod_pvt)
		RING_FINAL_CHECK_FOR_RESPONSES(&info->i2c_ring, more_to_do);
	else
		info->i2c_ring.sring->rsp_event = i + 1;

	spin_unlock_irqrestore(&info->lock, flags);

	mutex_unlock(&info->xferlock);

	return ret;
}

static u32 i2cfront_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_WORD_DATA |
			I2C_FUNC_SMBUS_BLOCK_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm i2cfront_algo = {
	.master_xfer = i2cfront_xfer,
	.smbus_xfer = i2cfront_smbus_xfer,
	.functionality = i2cfront_func,
};

static int i2cfront_probe(struct xenbus_device *dev,
			  const struct xenbus_device_id *id)
{
	struct i2cfront_info *info;

	info = kzalloc(sizeof(struct i2cfront_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->i2cdev = dev;
	dev_set_drvdata(&dev->dev, info);
	info->adapter.owner = THIS_MODULE;
	info->adapter.algo = &i2cfront_algo;
	info->adapter.dev.parent = &dev->dev;
	strlcpy(info->adapter.name, dev->nodename, sizeof(info->adapter.name));
	i2c_set_adapdata(&info->adapter, info);
	spin_lock_init(&info->lock);
	mutex_init(&info->xferlock);
	init_completion(&info->completion);

	return 0;
}

static int i2cfront_handle_int(struct i2cfront_info *info)
{
	complete(&info->completion);

	return 0;
}

static irqreturn_t i2cfront_int(int irq, void *dev_id)
{
	struct i2cfront_info *info = dev_id;

	while (i2cfront_handle_int(info))
		cond_resched();

	return IRQ_HANDLED;
}

static int i2cfront_setup_rings(struct xenbus_device *dev,
				struct i2cfront_info *info)
{
	struct i2cif_sring *i2c_sring;
	grant_ref_t gref;
	int err;

	info->i2c_ring_ref = GRANT_INVALID_REF;
	i2c_sring = (struct i2cif_sring *)get_zeroed_page(GFP_NOIO |
							  __GFP_HIGH);
	if (!i2c_sring) {
		xenbus_dev_fatal(dev, -ENOMEM, "allocating i2c sring");
		return -ENOMEM;
	}

	SHARED_RING_INIT(i2c_sring);
	FRONT_RING_INIT(&info->i2c_ring, i2c_sring, PAGE_SIZE);

	err = xenbus_grant_ring(dev, i2c_sring, 1, &gref);
	if (err < 0) {
		free_page((unsigned long)i2c_sring);
		info->i2c_ring.sring = NULL;
		goto fail;
	}
	info->i2c_ring_ref = gref;

	err = xenbus_alloc_evtchn(dev, &info->evtchn);
	if (err) {
		xenbus_dev_fatal(dev, err, "xenbus_alloc_evtchn");
		goto fail;
	}

	err = bind_evtchn_to_irqhandler(info->evtchn, i2cfront_int, 0,
					"xen_i2cif", info);
	if (err <= 0) {
		xenbus_dev_fatal(dev, err, "bind_evtchn_to_irqhandler failed");
		goto fail;
	}

	info->irq = err;

	return 0;

fail:
	i2cfront_destroy_rings(info);
	return err;
}

static int i2cfront_connect(struct xenbus_device *dev)
{
	struct i2cfront_info *info = dev_get_drvdata(&dev->dev);
	struct xenbus_transaction xbt;
	struct device_node *np;
	const char *be_adapter;
	char xenstore_adapter[I2CIF_ADAPTER_NAME_LEN];
	char *message;
	int err;

	err = i2cfront_setup_rings(dev, info);
	if (err) {
		dev_err(&dev->dev, "%s:failure....", __func__);
		return err;
	}
again:
	err = xenbus_transaction_start(&xbt);
	if (err) {
		xenbus_dev_fatal(dev, err, "starting transaction");
		goto destroy_ring;
	}

	err = xenbus_printf(xbt, dev->nodename, "ring-ref", "%u",
			    info->i2c_ring_ref);
	if (err) {
		message = "writing i2c ring-ref";
		goto abort_transaction;
	}

	err = xenbus_printf(xbt, dev->nodename, "event-channel", "%u",
			    info->evtchn);
	if (err) {
		message = "writing event-channel";
		goto abort_transaction;
	}

	err = xenbus_scanf(xbt, dev->nodename,
			   "be-adapter", "%32s", xenstore_adapter);
	if (err != 1) {
		message = "getting be-adapter";
		goto abort_transaction;
	}

	err = xenbus_transaction_end(xbt, 0);
	if (err) {
		if (err == -EAGAIN)
			goto again;
		xenbus_dev_fatal(dev, err, "completing transaction");
		goto destroy_ring;
	}

	for_each_compatible_node(np, NULL, "xen,i2c") {
		err = of_property_read_string(np, "be-adapter", &be_adapter);
		if (err)
			continue;
		if (!strncmp(xenstore_adapter, be_adapter,
		    I2CIF_ADAPTER_NAME_LEN)) {
			info->adapter.dev.of_node = np;
			break;
		}
	}

	err = i2c_add_adapter(&info->adapter);
	if (err)
		return err;

	dev_info(&info->adapter.dev, "XEN I2C adapter registered\n");

	return 0;

abort_transaction:
	xenbus_transaction_end(xbt, 1);
	xenbus_dev_fatal(dev, err, "%s", message);

destroy_ring:
	i2cfront_destroy_rings(info);

	return err;
}

static void i2cfront_disconnect(struct xenbus_device *dev)
{
	pr_info("%s\n", __func__);
	xenbus_frontend_closed(dev);
}

static void i2cfront_backend_changed(struct xenbus_device *dev,
				     enum xenbus_state backend_state)
{
	switch (backend_state) {
	case XenbusStateInitialising:
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateUnknown:
		break;

	case XenbusStateInitWait:
	case XenbusStateInitialised:
	case XenbusStateConnected:
		if (dev->state != XenbusStateInitialising)
			break;
		if (!i2cfront_connect(dev))
			xenbus_switch_state(dev, XenbusStateConnected);
		break;

	case XenbusStateClosed:
		if (dev->state == XenbusStateClosed)
			break;
		/* Missed the backend's Closing state -- fallthrough */
	case XenbusStateClosing:
		i2cfront_disconnect(dev);
		break;

	default:
		xenbus_dev_fatal(dev, -EINVAL, "saw state %d at frontend",
				 backend_state);
		break;
	}
}

static int i2cfront_remove(struct xenbus_device *dev)
{
	struct i2cfront_info *info = dev_get_drvdata(&dev->dev);

	i2c_del_adapter(&info->adapter);
	i2cfront_destroy_rings(info);

	kfree(info);

	dev_info(&dev->dev, "Remove");
	return 0;
}

static const struct xenbus_device_id i2cfront_ids[] = {
	{ "vi2c" },
	{ "" },
};

static struct xenbus_driver i2cfront_driver = {
	.ids = i2cfront_ids,
	.probe = i2cfront_probe,
	.otherend_changed = i2cfront_backend_changed,
	.remove = i2cfront_remove,
};

static int __init i2cfront_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	return xenbus_register_frontend(&i2cfront_driver);
}
subsys_initcall(i2cfront_init);
