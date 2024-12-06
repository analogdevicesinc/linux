// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/rpmsg.h>

#define MSG		"hello world!"

static int rpmsg_pingpong_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *priv, u32 src)
{
	int err;
	unsigned int rpmsg_pingpong;

	/* reply */
	rpmsg_pingpong = *(unsigned int *)data;
	pr_info("get %d (src: 0x%x)\n", rpmsg_pingpong, src);

	/* pingpongs should not live forever */
	if (rpmsg_pingpong > 100) {
		dev_info(&rpdev->dev, "goodbye!\n");
		return 0;
	}
	rpmsg_pingpong++;
	err = rpmsg_sendto(rpdev->ept, (void *)(&rpmsg_pingpong), 4, src);

	if (err)
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", err);

	return err;
}

static int rpmsg_pingpong_probe(struct rpmsg_device *rpdev)
{
	int err;
	unsigned int rpmsg_pingpong;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	/*
	 * send a message to our remote processor, and tell remote
	 * processor about this channel
	 */
	err = rpmsg_send(rpdev->ept, MSG, strlen(MSG));
	if (err) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", err);
		return err;
	}

	rpmsg_pingpong = 0;
	err = rpmsg_sendto(rpdev->ept, (void *)(&rpmsg_pingpong),
			   4, rpdev->dst);
	if (err) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", err);
		return err;
	}

	return 0;
}

static void rpmsg_pingpong_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "rpmsg pingpong driver is removed\n");
}

static struct rpmsg_device_id rpmsg_driver_pingpong_id_table[] = {
	{ .name	= "rpmsg-openamp-demo-channel" },
	{ .name	= "rpmsg-openamp-demo-channel-1" },
	{ },
};

static struct rpmsg_driver rpmsg_pingpong_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_pingpong_id_table,
	.probe		= rpmsg_pingpong_probe,
	.callback	= rpmsg_pingpong_cb,
	.remove		= rpmsg_pingpong_remove,
};

static int __init init(void)
{
	return register_rpmsg_driver(&rpmsg_pingpong_driver);
}

static void __exit fini(void)
{
	unregister_rpmsg_driver(&rpmsg_pingpong_driver);
}
module_init(init);
module_exit(fini);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("iMX virtio remote processor messaging pingpong driver");
MODULE_LICENSE("GPL v2");
