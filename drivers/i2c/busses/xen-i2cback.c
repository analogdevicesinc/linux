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
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <xen/xen.h>
#include <xen/events.h>
#include <xen/xenbus.h>
#include <xen/grant_table.h>
#include <xen/page.h>

#include <xen/interface/grant_table.h>
#include <xen/interface/io/i2cif.h>

struct i2cback_info {
	domid_t domid;
	u32 irq;
	u64 handle;
	struct xenbus_device *i2cdev;
	spinlock_t i2c_ring_lock;
	struct i2cif_back_ring i2c_ring;
	int is_connected;
	int ring_error;
	struct i2c_adapter *adapter;
	u32 num_slaves;
	u32 *allowed_slaves;
};

static bool i2cback_access_allowed(struct i2cback_info *info,
				   struct i2cif_request *req)
{
	int i;

	if (req->num_msg == I2CIF_MAX_MSG) {
		if (req->msg[0].addr != req->msg[1].addr)
			return false;
	}

	for (i = 0; i < info->num_slaves; i++) {
		if (req->msg[0].addr == info->allowed_slaves[i])
			return true;
	}

	return false;
}

static bool i2cback_handle_int(struct i2cback_info *info)
{
	struct i2cif_back_ring *i2c_ring = &info->i2c_ring;
	struct i2cif_request req;
	struct i2cif_response *res;
	RING_IDX rc, rp;
	int more_to_do, notify, num_msg, ret;
	struct i2c_msg msg[I2CIF_MAX_MSG];
	char tmp_buf[I2CIF_BUF_LEN];
	unsigned long flags;
	bool allow_access;
	int i;

	rc = i2c_ring->req_cons;
	rp = i2c_ring->sring->req_prod;
	rmb();	/* req_cons is written by frontend. */

	if (RING_REQUEST_PROD_OVERFLOW(i2c_ring, rp)) {
		rc = i2c_ring->rsp_prod_pvt;
		dev_err(&info->i2cdev->dev, "ring overflow\n");
		info->ring_error = 1;
		return 0;
	}

	while (rc != rp) {
		if (RING_REQUEST_CONS_OVERFLOW(i2c_ring, rc)) {
			dev_err(&info->i2cdev->dev, "%s overflow\n", __func__);
			break;
		}

		req = *RING_GET_REQUEST(i2c_ring, rc);
		allow_access = i2cback_access_allowed(info, &req);
		if (allow_access) {
			/* Write/Read sequence */
			if (req.num_msg > I2CIF_MAX_MSG)
				num_msg = I2CIF_MAX_MSG;
			for (i = 0; i < num_msg; i++) {
				msg[i].addr = req.msg[i].addr;
				msg[i].len = req.msg[i].len;
				msg[i].flags = 0;
				if (req.msg[i].flags & I2CIF_M_RD)
					msg[i].flags |= I2C_M_RD;
				if (req.msg[i].flags & I2CIF_M_TEN)
					msg[i].flags |= I2C_M_TEN;
				if (req.msg[i].flags & I2CIF_M_RECV_LEN)
					msg[i].flags |= I2C_M_RECV_LEN;
				if (req.msg[i].flags & I2CIF_M_NO_RD_ACK)
					msg[i].flags |= I2C_M_NO_RD_ACK;
				if (req.msg[i].flags & I2CIF_M_IGNORE_NAK)
					msg[i].flags |= I2C_M_IGNORE_NAK;
				if (req.msg[i].flags & I2CIF_M_REV_DIR_ADDR)
					msg[i].flags |= I2C_M_REV_DIR_ADDR;
				if (req.msg[i].flags & I2CIF_M_NOSTART)
					msg[i].flags |= I2C_M_NOSTART;
				if (req.msg[i].flags & I2CIF_M_STOP)
					msg[i].flags |= I2C_M_STOP;
			}

			if ((num_msg == 2) &&
			    (!(msg[0].flags & I2C_M_RD)) &&
			    (msg[1].flags & I2C_M_RD)) {

				/* overwrite the remote buf with local buf */
				msg[0].buf = tmp_buf;
				msg[1].buf = tmp_buf;

				/* msg[0] write buf */
				memcpy(tmp_buf, req.write_buf, I2CIF_BUF_LEN);
				ret = i2c_transfer(info->adapter, msg,
						   num_msg);
			} else if (req.num_msg == 1) {
				msg[0].buf = tmp_buf;
				if (!(msg[0].flags & I2C_M_RD))
					memcpy(tmp_buf, req.write_buf,
					       I2CIF_BUF_LEN);
				ret = i2c_transfer(info->adapter, msg,
						   req.num_msg);
			} else {
				dev_dbg(&info->i2cdev->dev, "too many msgs\n");

				ret = -EIO;
			}
		}

		spin_lock_irqsave(&info->i2c_ring_lock, flags);
		res = RING_GET_RESPONSE(&info->i2c_ring,
					info->i2c_ring.rsp_prod_pvt);

		if (allow_access) {
			res->result = ret;

			if ((req.num_msg == 2) &&
			    (!(msg[0].flags & I2C_M_RD)) &&
			    (msg[1].flags & I2C_M_RD)) {
				memcpy(res->read_buf, tmp_buf, I2CIF_BUF_LEN);
			} else if (req.num_msg == 1) {
				if (msg[0].flags & I2C_M_RD)
					memcpy(res->read_buf, tmp_buf,
					       I2CIF_BUF_LEN);
			}
		} else
			res->result = -EPERM;

		info->i2c_ring.rsp_prod_pvt++;

		barrier();
		RING_PUSH_RESPONSES_AND_CHECK_NOTIFY(&info->i2c_ring,
						     notify);
		spin_unlock_irqrestore(&info->i2c_ring_lock, flags);

		if (notify)
			notify_remote_via_irq(info->irq);

		i2c_ring->req_cons = ++rc;

		cond_resched();
	}

	RING_FINAL_CHECK_FOR_REQUESTS(i2c_ring, more_to_do);

	return !!more_to_do;
}

static irqreturn_t i2cback_be_int(int irq, void *dev_id)
{
	struct i2cback_info *info = dev_id;

	if (info->ring_error)
		return IRQ_HANDLED;

	while (i2cback_handle_int(info))
		cond_resched();

	return IRQ_HANDLED;
}

static int i2cback_map(struct i2cback_info *info, grant_ref_t *i2c_ring_ref,
		       evtchn_port_t evtchn)
{
	int err;
	void *addr;
	struct i2cif_sring *i2c_sring;

	if (info->irq)
		return 0;

	err = xenbus_map_ring_valloc(info->i2cdev, i2c_ring_ref, 1, &addr);
	if (err)
		return err;

	i2c_sring = addr;

	BACK_RING_INIT(&info->i2c_ring, i2c_sring, PAGE_SIZE);

	err = bind_interdomain_evtchn_to_irq(info->domid, evtchn);
	if (err < 0)
		goto fail_evtchn;
	info->irq = err;

	err = request_threaded_irq(info->irq, NULL, i2cback_be_int,
				   IRQF_ONESHOT, "xen-i2cback", info);
	if (err) {
		dev_err(&info->i2cdev->dev, "bind evtchn to irq failure!\n");
		goto free_irq;
	}

	return 0;
free_irq:
	unbind_from_irqhandler(info->irq, info);
	info->irq = 0;
	info->i2c_ring.sring = NULL;
fail_evtchn:
	xenbus_unmap_ring_vfree(info->i2cdev, i2c_sring);
	return err;
}

static int i2cback_connect_rings(struct i2cback_info *info)
{
	struct xenbus_device *dev = info->i2cdev;
	unsigned int i2c_ring_ref, evtchn;
	int i, err;
	char *buf;
	u32 adapter_id;

	err = xenbus_scanf(XBT_NIL, dev->nodename,
			   "adapter", "%u", &adapter_id);
	if (err != 1) {
		xenbus_dev_fatal(dev, err, "%s reading adapter", dev->nodename);
		return err;
	}

	info->adapter = i2c_get_adapter(adapter_id);
	if (!info->adapter)
		return -ENODEV;

	err = xenbus_scanf(XBT_NIL, dev->nodename,
			   "num-slaves", "%u", &info->num_slaves);
	if (err != 1) {
		xenbus_dev_fatal(dev, err, "%s reading num-slaves",
				 dev->nodename);
		return err;
	}

	info->allowed_slaves = devm_kmalloc(&dev->dev,
					    info->num_slaves * sizeof(u32),
					    GFP_KERNEL);
	if (!info->allowed_slaves)
		return -ENOMEM;

	/* 128 bytes is enough */
	buf = kmalloc(128, GFP_KERNEL);

	for (i = 0; i < info->num_slaves; i++) {
		snprintf(buf, 128, "%s/%d", dev->nodename, i);
		err = xenbus_scanf(XBT_NIL, buf, "addr", "%x",
				   &info->allowed_slaves[i]);
		if (err != 1) {
			kfree(buf);
			return err;
		}
	}

	kfree(buf);

	err = xenbus_gather(XBT_NIL, dev->otherend,
			    "ring-ref", "%u", &i2c_ring_ref,
			    "event-channel", "%u", &evtchn, NULL);
	if (err) {
		xenbus_dev_fatal(dev, err,
				 "reading %s/ring-ref and event-channel",
				 dev->otherend);
		return err;
	}

	dev_info(&info->i2cdev->dev,
		 "xen-pvi2c: ring-ref %u, event-channel %u\n",
		 i2c_ring_ref, evtchn);

	err = i2cback_map(info, &i2c_ring_ref, evtchn);
	if (err)
		xenbus_dev_fatal(dev, err, "mapping ring-ref %u evtchn %u",
			i2c_ring_ref, evtchn);

	return err;
}

static void i2cback_disconnect(struct i2cback_info *info)
{
	if (info->irq) {
		unbind_from_irqhandler(info->irq, info);
		info->irq = 0;
	}

	if (info->i2c_ring.sring) {
		xenbus_unmap_ring_vfree(info->i2cdev, info->i2c_ring.sring);
		info->i2c_ring.sring = NULL;
	}
}

static void i2cback_frontend_changed(struct xenbus_device *dev,
				     enum xenbus_state frontend_state)
{
	struct i2cback_info *info = dev_get_drvdata(&dev->dev);
	int ret;

	switch (frontend_state) {
	case XenbusStateInitialised:
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
		break;

	case XenbusStateInitialising:
		if (dev->state == XenbusStateClosed) {
			dev_info(&dev->dev,
				 "xen-pvi2c: %s: prepare for reconnect\n",
				 dev->nodename);
			xenbus_switch_state(dev, XenbusStateInitWait);
		}
		break;
	case XenbusStateConnected:
		if (dev->state == XenbusStateConnected)
			break;

		xenbus_switch_state(dev, XenbusStateConnected);

		ret = i2cback_connect_rings(info);
		if (ret) {
			xenbus_dev_fatal(dev, ret, "connect ring fail");
		}
		break;
	case XenbusStateClosing:
		i2cback_disconnect(info);
		xenbus_switch_state(dev, XenbusStateClosing);
		break;
	case XenbusStateClosed:
		xenbus_switch_state(dev, XenbusStateClosed);
		if (xenbus_dev_is_online(dev))
			break;
		/* fall through if not online */
	case XenbusStateUnknown:
		device_unregister(&dev->dev);
		break;

	default:
		xenbus_dev_fatal(dev, -EINVAL, "saw state %d at frontend",
				 frontend_state);
		break;
	}
}

static struct i2cback_info *i2cback_alloc(domid_t domid, u64 handle)
{
	struct i2cback_info *info;

	info = kzalloc(sizeof(struct i2cback_info), GFP_KERNEL);
	if (!info)
		return NULL;

	info->domid = domid;
	info->handle = handle;
	spin_lock_init(&info->i2c_ring_lock);
	info->ring_error = 0;

	return info;
}

static int i2cback_probe(struct xenbus_device *dev,
			     const struct xenbus_device_id *id)
{
	struct i2cback_info *info;
	unsigned long handle;
	int err;

	if (kstrtoul(strrchr(dev->otherend, '/') + 1, 0, &handle))
		return -EINVAL;

	info = i2cback_alloc(dev->otherend_id, handle);
	if (!info) {
		xenbus_dev_fatal(dev, -ENOMEM, "Allocating backend interface");
		return -ENOMEM;
	}

	info->i2cdev = dev;
	dev_set_drvdata(&dev->dev, info);

	err = xenbus_switch_state(dev, XenbusStateInitWait);
	if (err)
		return err;

	return 0;
}

static int i2cback_remove(struct xenbus_device *dev)
{
	struct i2cback_info *info = dev_get_drvdata(&dev->dev);

	if (!info)
		return 0;

	i2cback_disconnect(info);

	kfree(info);
	dev_set_drvdata(&dev->dev, NULL);

	dev_info(&dev->dev, "%s\n", __func__);

	return 0;
}

static const struct xenbus_device_id i2cback_ids[] = {
	{ "vi2c" },
	{ "" },
};

static struct xenbus_driver i2cback_driver = {
	.ids			= i2cback_ids,
	.probe			= i2cback_probe,
	.otherend_changed	= i2cback_frontend_changed,
	.remove			= i2cback_remove,
};

static int __init i2cback_init(void)
{
	int err;

	if (!xen_domain())
		return -ENODEV;

	err = xenbus_register_backend(&i2cback_driver);
	if (err)
		return err;

	return 0;
}
module_init(i2cback_init);

static void __exit i2cback_exit(void)
{
	xenbus_unregister_driver(&i2cback_driver);
}
module_exit(i2cback_exit);

MODULE_ALIAS("xen-i2cback:vi2c");
MODULE_AUTHOR("Peng Fan <peng.fan@nxp.com>");
MODULE_DESCRIPTION("Xen I2C backend driver (i2cback)");
MODULE_LICENSE("GPL");
