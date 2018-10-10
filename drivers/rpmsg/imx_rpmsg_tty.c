/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 * Copyright (C) 2017 NXP
 *
 * derived from the omap-rpmsg implementation.
 * Remote processor messaging transport - tty driver
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/virtio.h>

/* this needs to be less then (RPMSG_BUF_SIZE - sizeof(struct rpmsg_hdr)) */
#define RPMSG_MAX_SIZE		256
#define MSG		"hello world!"

/*
 * struct rpmsgtty_port - Wrapper struct for imx rpmsg tty port.
 * @port:		TTY port data
 */
struct rpmsgtty_port {
	struct tty_port		port;
	spinlock_t		rx_lock;
	struct rpmsg_device	*rpdev;
	struct tty_driver	*rpmsgtty_driver;
};

static int rpmsg_tty_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *priv, u32 src)
{
	int space;
	unsigned char *cbuf;
	struct rpmsgtty_port *cport = dev_get_drvdata(&rpdev->dev);

	/* flush the recv-ed none-zero data to tty node */
	if (len == 0)
		return 0;

	dev_dbg(&rpdev->dev, "msg(<- src 0x%x) len %d\n", src, len);

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);

	spin_lock_bh(&cport->rx_lock);
	space = tty_prepare_flip_string(&cport->port, &cbuf, len);
	if (space <= 0) {
		dev_err(&rpdev->dev, "No memory for tty_prepare_flip_string\n");
		spin_unlock_bh(&cport->rx_lock);
		return -ENOMEM;
	}

	memcpy(cbuf, data, len);
	tty_flip_buffer_push(&cport->port);
	spin_unlock_bh(&cport->rx_lock);

	return 0;
}

static struct tty_port_operations  rpmsgtty_port_ops = { };

static int rpmsgtty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct rpmsgtty_port *cport = driver->driver_state;

	return tty_port_install(&cport->port, driver, tty);
}

static int rpmsgtty_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void rpmsgtty_close(struct tty_struct *tty, struct file *filp)
{
	return tty_port_close(tty->port, tty, filp);
}

static int rpmsgtty_write(struct tty_struct *tty, const unsigned char *buf,
			 int total)
{
	int count, ret = 0;
	const unsigned char *tbuf;
	struct rpmsgtty_port *rptty_port = container_of(tty->port,
			struct rpmsgtty_port, port);
	struct rpmsg_device *rpdev = rptty_port->rpdev;

	if (NULL == buf) {
		pr_err("buf shouldn't be null.\n");
		return -ENOMEM;
	}

	count = total;
	tbuf = buf;
	do {
		/* send a message to our remote processor */
		ret = rpmsg_send(rpdev->ept, (void *)tbuf,
			count > RPMSG_MAX_SIZE ? RPMSG_MAX_SIZE : count);
		if (ret) {
			dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
			return ret;
		}

		if (count > RPMSG_MAX_SIZE) {
			count -= RPMSG_MAX_SIZE;
			tbuf += RPMSG_MAX_SIZE;
		} else {
			count = 0;
		}
	} while (count > 0);

	return total;
}

static int rpmsgtty_write_room(struct tty_struct *tty)
{
	/* report the space in the rpmsg buffer */
	return RPMSG_MAX_SIZE;
}

static const struct tty_operations imxrpmsgtty_ops = {
	.install		= rpmsgtty_install,
	.open			= rpmsgtty_open,
	.close			= rpmsgtty_close,
	.write			= rpmsgtty_write,
	.write_room		= rpmsgtty_write_room,
};

static int rpmsg_tty_probe(struct rpmsg_device *rpdev)
{
	int ret;
	struct rpmsgtty_port *cport;
	struct tty_driver *rpmsgtty_driver;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	cport = devm_kzalloc(&rpdev->dev, sizeof(*cport), GFP_KERNEL);
	if (!cport)
		return -ENOMEM;

	rpmsgtty_driver = tty_alloc_driver(1, TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(rpmsgtty_driver)) {
		kfree(cport);
		return PTR_ERR(rpmsgtty_driver);
	}

	rpmsgtty_driver->driver_name = "rpmsg_tty";
	rpmsgtty_driver->name = kasprintf(GFP_KERNEL, "ttyRPMSG%d", rpdev->dst);
	rpmsgtty_driver->major = UNNAMED_MAJOR;
	rpmsgtty_driver->minor_start = 0;
	rpmsgtty_driver->type = TTY_DRIVER_TYPE_CONSOLE;
	rpmsgtty_driver->init_termios = tty_std_termios;

	tty_set_operations(rpmsgtty_driver, &imxrpmsgtty_ops);

	tty_port_init(&cport->port);
	cport->port.ops = &rpmsgtty_port_ops;
	spin_lock_init(&cport->rx_lock);
	cport->port.low_latency = cport->port.flags | ASYNC_LOW_LATENCY;
	cport->rpdev = rpdev;
	dev_set_drvdata(&rpdev->dev, cport);
	rpmsgtty_driver->driver_state = cport;
	cport->rpmsgtty_driver = rpmsgtty_driver;

	ret = tty_register_driver(cport->rpmsgtty_driver);
	if (ret < 0) {
		pr_err("Couldn't install rpmsg tty driver: ret %d\n", ret);
		goto error1;
	} else {
		pr_info("Install rpmsg tty driver!\n");
	}

	/*
	 * send a message to our remote processor, and tell remote
	 * processor about this channel
	 */
	ret = rpmsg_send(rpdev->ept, MSG, strlen(MSG));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		goto error;
	}

	return 0;

error:
	tty_unregister_driver(cport->rpmsgtty_driver);
error1:
	put_tty_driver(cport->rpmsgtty_driver);
	tty_port_destroy(&cport->port);
	cport->rpmsgtty_driver = NULL;
	kfree(cport);

	return ret;
}

static void rpmsg_tty_remove(struct rpmsg_device *rpdev)
{
	struct rpmsgtty_port *cport = dev_get_drvdata(&rpdev->dev);

	dev_info(&rpdev->dev, "rpmsg tty driver is removed\n");

	tty_unregister_driver(cport->rpmsgtty_driver);
	kfree(cport->rpmsgtty_driver->name);
	put_tty_driver(cport->rpmsgtty_driver);
	tty_port_destroy(&cport->port);
	cport->rpmsgtty_driver = NULL;
}

static struct rpmsg_device_id rpmsg_driver_tty_id_table[] = {
	{ .name	= "rpmsg-virtual-tty-channel-1" },
	{ .name	= "rpmsg-virtual-tty-channel" },
	{ .name = "rpmsg-openamp-demo-channel" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_tty_id_table);

static struct rpmsg_driver rpmsg_tty_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_tty_id_table,
	.probe		= rpmsg_tty_probe,
	.callback	= rpmsg_tty_cb,
	.remove		= rpmsg_tty_remove,
};

static int __init init(void)
{
	return register_rpmsg_driver(&rpmsg_tty_driver);
}

static void __exit fini(void)
{
	unregister_rpmsg_driver(&rpmsg_tty_driver);
}
module_init(init);
module_exit(fini);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("iMX virtio remote processor messaging tty driver");
MODULE_LICENSE("GPL v2");
