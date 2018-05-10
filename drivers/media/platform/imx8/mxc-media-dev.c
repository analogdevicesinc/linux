/*
 * Copyright 2017-2018 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/media-device.h>

#include "mxc-media-dev.h"
#include "mxc-isi-core.h"
#include "mxc-mipi-csi2.h"
#include "mxc-parallel-csi.h"

/*create default links between registered entities  */
static int mxc_md_create_links(struct mxc_md *mxc_md)
{
	struct media_entity *source, *sink;
	struct mxc_isi_dev *mxc_isi;
	struct mxc_sensor_info *sensor;
	struct mxc_mipi_csi2_dev *mipi_csi2;
	struct mxc_parallel_csi_dev *pcsidev;
	int i, j, ret = 0;
	u16  source_pad, sink_pad;
	u32 flags;
	u32 mipi_vc = 0;

	/* Create links between each ISI's subdev and video node */
	flags = MEDIA_LNK_FL_ENABLED;
	for (i = 0; i < MXC_ISI_MAX_DEVS; i++) {
		mxc_isi = mxc_md->mxc_isi[i];
		if (!mxc_isi)
			continue;

		/* Connect ISI source to video device */
		source = &mxc_isi->isi_cap.sd.entity;
		sink = &mxc_isi->isi_cap.vdev.entity;
		sink_pad = 0;

		switch (mxc_isi->interface[OUT_PORT]) {
		case ISI_OUTPUT_INTERFACE_DC0:
			source_pad = MXC_ISI_SD_PAD_SOURCE_DC0;
			break;
		case ISI_OUTPUT_INTERFACE_DC1:
			source_pad = MXC_ISI_SD_PAD_SOURCE_DC1;
			break;
		case ISI_OUTPUT_INTERFACE_MEM:
			source_pad = MXC_ISI_SD_PAD_SOURCE_MEM;
			break;
		default:
			v4l2_err(&mxc_md->v4l2_dev, "Wrong output interface: %x\n",
				mxc_isi->interface[OUT_PORT]);
			return -EINVAL;
		}

		ret = media_create_pad_link(source, source_pad,
					      sink, sink_pad, flags);
		if (ret) {
			v4l2_err(&mxc_md->v4l2_dev, "Failed created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
			break;
		}

		/* Notify capture subdev entity ,ISI cap link setup */
		ret = media_entity_call(source, link_setup, &source->pads[source_pad],
						&sink->pads[sink_pad], flags);
		if (ret) {
			v4l2_err(&mxc_md->v4l2_dev, "failed call link_setup [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
			break;
		}

		v4l2_info(&mxc_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);

		/* Connect MIPI/HDMI/Mem source to ISI sink */
		sink = &mxc_isi->isi_cap.sd.entity;

		switch (mxc_isi->interface[IN_PORT]) {

		case ISI_INPUT_INTERFACE_MIPI0_CSI2:
			if (mxc_md->mipi_csi2[0] == NULL)
				continue;
			source = &mxc_md->mipi_csi2[0]->sd.entity;

			switch (mxc_isi->interface[SUB_IN_PORT]) {
			case ISI_INPUT_SUB_INTERFACE_VC1:
				source_pad = MXC_MIPI_CSI2_VC1_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI0_VC1;
				break;
			case ISI_INPUT_SUB_INTERFACE_VC2:
				source_pad = MXC_MIPI_CSI2_VC2_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI0_VC2;
				break;
			case ISI_INPUT_SUB_INTERFACE_VC3:
				source_pad = MXC_MIPI_CSI2_VC3_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI0_VC3;
				break;
			default:
				source_pad = MXC_MIPI_CSI2_VC0_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI0_VC0;
				break;
			}
			break;

		case ISI_INPUT_INTERFACE_MIPI1_CSI2:
			if (mxc_md->mipi_csi2[1] == NULL)
				continue;
			source = &mxc_md->mipi_csi2[1]->sd.entity;

			switch (mxc_isi->interface[SUB_IN_PORT]) {
			case ISI_INPUT_SUB_INTERFACE_VC1:
				source_pad = MXC_MIPI_CSI2_VC1_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI1_VC1;
				break;
			case ISI_INPUT_SUB_INTERFACE_VC2:
				source_pad = MXC_MIPI_CSI2_VC2_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI1_VC2;
				break;
			case ISI_INPUT_SUB_INTERFACE_VC3:
				source_pad = MXC_MIPI_CSI2_VC3_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI1_VC3;
				break;
			default:
				source_pad = MXC_MIPI_CSI2_VC0_PAD_SOURCE;
				sink_pad = MXC_ISI_SD_PAD_SINK_MIPI1_VC0;
				break;
			}
			break;
		case ISI_INPUT_INTERFACE_PARALLEL_CSI:
			if (mxc_md->pcsidev == NULL)
				continue;
			source = &mxc_md->pcsidev->sd.entity;
			source_pad = MXC_PARALLEL_CSI_PAD_SOURCE;
			sink_pad = MXC_ISI_SD_PAD_SINK_PARALLEL_CSI;
			break;

		case ISI_INPUT_INTERFACE_HDMI:
			if (mxc_md->hdmi_rx == NULL)
				continue;
			source = &mxc_md->hdmi_rx->sd.entity;
			source_pad = MXC_HDMI_RX_PAD_SOURCE;
			sink_pad = MXC_ISI_SD_PAD_SINK_HDMI;
			break;

		case ISI_INPUT_INTERFACE_DC0:
		case ISI_INPUT_INTERFACE_DC1:
		case ISI_INPUT_INTERFACE_MEM:
		default:
			v4l2_err(&mxc_md->v4l2_dev, "Not support input interface: %x\n",
				mxc_isi->interface[IN_PORT]);
			return -EINVAL;
		}
		/* Create link MIPI/HDMI to ISI */
		ret = media_create_pad_link(source, source_pad, sink, sink_pad, flags);
		if (ret) {
			v4l2_err(&mxc_md->v4l2_dev, "created link [%s] %c> [%s] fail\n",
			  source->name, flags ? '=' : '-', sink->name);
			break;
		}

		/* Notify ISI subdev entity */
		ret = media_entity_call(sink, link_setup, &sink->pads[sink_pad],
					&source->pads[source_pad], 0);
		if (ret)
			break;

		/* Notify MIPI/HDMI entity */
		ret = media_entity_call(source, link_setup, &source->pads[source_pad],
					&sink->pads[sink_pad], 0);
		if (ret)
			break;

		v4l2_info(&mxc_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
	}

	/* Connect MIPI Sensor to MIPI CSI2 */
	for (i = 0; i < mxc_md->num_sensors; i++) {
		sensor = &mxc_md->sensor[i];
		if (sensor == NULL || sensor->sd == NULL)
			continue;

		if (mxc_md->parallel_csi && !sensor->mipi_mode) {
			pcsidev = mxc_md->pcsidev;
			if (pcsidev == NULL)
				continue;
			source = &sensor->sd->entity;
			sink = &pcsidev->sd.entity;

			source_pad = 0;
			sink_pad = MXC_PARALLEL_CSI_PAD_SINK;

			ret = media_create_pad_link(source, source_pad, sink, sink_pad,
						  MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
			if (ret)
				return ret;

			/* Notify MIPI subdev entity */
			ret = media_entity_call(sink, link_setup, &sink->pads[sink_pad],
						&source->pads[source_pad], 0);
			if (ret)
				return ret;

			/* Notify MIPI sensor subdev entity */
			ret = media_entity_call(source, link_setup, &source->pads[source_pad],
						&sink->pads[sink_pad], 0);
			if (ret)
				return ret;
			v4l2_info(&mxc_md->v4l2_dev, "created link [%s] => [%s]\n",
						sensor->sd->entity.name, pcsidev->sd.entity.name);
		} else if (mxc_md->mipi_csi2) {
			mipi_csi2 = mxc_md->mipi_csi2[sensor->id];
			if (mipi_csi2 ==  NULL)
				continue;
			source = &sensor->sd->entity;
			sink = &mipi_csi2->sd.entity;

			source_pad = 0;  /* sensor source pad: MIPI_CSI2_SENS_VC0_PAD_SOURCE */
			sink_pad = source_pad;  /* mipi sink pad: MXC_MIPI_CSI2_VC0_PAD_SINK; */

			if (mipi_csi2->vchannel == true)
				mipi_vc = 4;
			else
				mipi_vc = 1;

			for (j = 0; j < mipi_vc; j++) {
				ret = media_create_pad_link(source, source_pad + j, sink, sink_pad + j,
						  MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
				if (ret)
					return ret;

				/* Notify MIPI subdev entity */
				ret = media_entity_call(sink, link_setup, &sink->pads[sink_pad + j],
							&source->pads[source_pad + j], 0);
				if (ret)
					return ret;

				/* Notify MIPI sensor subdev entity */
				ret = media_entity_call(source, link_setup, &source->pads[source_pad + j],
							&sink->pads[sink_pad + j], 0);
				if (ret)
					return ret;
			}
			v4l2_info(&mxc_md->v4l2_dev, "created link [%s] => [%s]\n",
				  sensor->sd->entity.name, mipi_csi2->sd.entity.name);
		}
	}
	dev_info(&mxc_md->pdev->dev, "%s\n", __func__);
	return 0;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_subdev *asd)
{
	struct mxc_md *mxc_md = notifier_to_mxc_md(notifier);
	struct mxc_sensor_info *sensor = NULL;
	int i;

	dev_dbg(&mxc_md->pdev->dev, "%s\n", __func__);
	/* Find platform data for this sensor subdev */
	for (i = 0; i < ARRAY_SIZE(mxc_md->sensor); i++) {
		if (mxc_md->sensor[i].asd.match.fwnode.fwnode ==
				of_fwnode_handle(sd->dev->of_node))
			sensor = &mxc_md->sensor[i];
	}

	if (sensor == NULL)
		return -EINVAL;

	sd->grp_id = GRP_ID_MXC_SENSOR;

	sensor->sd = sd;

	mxc_md->num_sensors++;

	v4l2_info(&mxc_md->v4l2_dev, "Registered sensor subdevice: %s (%d)\n",
		  sd->name, mxc_md->num_sensors);

	return 0;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mxc_md *mxc_md = notifier_to_mxc_md(notifier);
	int ret;

	dev_dbg(&mxc_md->pdev->dev, "%s\n", __func__);
	mutex_lock(&mxc_md->media_dev.graph_mutex);

	ret = mxc_md_create_links(mxc_md);
	if (ret < 0)
		goto unlock;

	ret = v4l2_device_register_subdev_nodes(&mxc_md->v4l2_dev);
unlock:
	mutex_unlock(&mxc_md->media_dev.graph_mutex);
	if (ret < 0) {
		v4l2_err(&mxc_md->v4l2_dev, "%s error exit\n", __func__);
		return ret;
	}

	return media_device_register(&mxc_md->media_dev);
}


/**
 * mxc_sensor_notify - v4l2_device notification from a sensor subdev
 */
void mxc_sensor_notify(struct v4l2_subdev *sd, unsigned int notification,
			void *arg)
{
	return;
}

/* Register mipi sensor / Parallel CSI / HDMI Rx sub-devices */
static int register_sensor_entities(struct mxc_md *mxc_md)
{
	struct device_node *parent = mxc_md->pdev->dev.of_node;
	struct device_node *node, *ep, *rem;
	struct v4l2_fwnode_endpoint endpoint;
	int index = 0;

	mxc_md->num_sensors = 0;

	/* Attach sensors linked to MIPI CSI2 / paralle csi / HDMI Rx */
	for_each_available_child_of_node(parent, node) {
		struct device_node *port;

		if (!of_node_cmp(node->name, "hdmi_rx")) {
			mxc_md->sensor[index].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
			mxc_md->sensor[index].asd.match.fwnode.fwnode = of_fwnode_handle(node);
			mxc_md->async_subdevs[index] = &mxc_md->sensor[index].asd;

			mxc_md->num_sensors++;
			index++;
			continue;
		}

		if (of_node_cmp(node->name, "csi") &&
			of_node_cmp(node->name, "pcsi"))
			continue;

		if (!of_device_is_available(node))
			continue;

		/* csi2 node have only port */
		port = of_get_next_child(node, NULL);
		if (!port)
			continue;

		/* port can have only endpoint */
		ep = of_get_next_child(port, NULL);
		if (!ep)
			return -EINVAL;

		v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &endpoint);
		if (WARN_ON(endpoint.base.port >= MXC_MAX_SENSORS)) {
			v4l2_err(&mxc_md->v4l2_dev, "Failed to get sensor endpoint\n");
			return -EINVAL;
		}

		mxc_md->sensor[index].id = endpoint.base.port;

		if (!of_node_cmp(node->name, "csi"))
			mxc_md->sensor[index].mipi_mode = true;

		/* remote port---sensor node */
		rem = of_graph_get_remote_port_parent(ep);
		of_node_put(ep);
		if (rem == NULL) {
			v4l2_info(&mxc_md->v4l2_dev, "Remote device at %s not found\n",
								ep->full_name);
			continue;
		}

		mxc_md->sensor[index].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		mxc_md->sensor[index].asd.match.fwnode.fwnode = of_fwnode_handle(rem);
		mxc_md->async_subdevs[index] = &mxc_md->sensor[index].asd;

		mxc_md->num_sensors++;

		index++;
	}

	return 0;
}

static int register_isi_entity(struct mxc_md *mxc_md, struct mxc_isi_dev *mxc_isi)
{
	struct v4l2_subdev *sd = &mxc_isi->isi_cap.sd;
	int ret;

	dev_dbg(&mxc_md->pdev->dev, "%s\n", __func__);
	if (WARN_ON(mxc_isi->id >= MXC_ISI_MAX_DEVS || mxc_md->mxc_isi[mxc_isi->id]))
		return -EBUSY;

	sd->grp_id = GRP_ID_MXC_ISI;

	ret = v4l2_device_register_subdev(&mxc_md->v4l2_dev, sd);
	if (!ret)
		mxc_md->mxc_isi[mxc_isi->id] = mxc_isi;
	else
		v4l2_err(&mxc_md->v4l2_dev, "Failed to register ISI.%d (%d)\n",
			 mxc_isi->id, ret);
	return ret;
}

static int register_mipi_csi2_entity(struct mxc_md *mxc_md,
				struct mxc_mipi_csi2_dev *mipi_csi2)
{
	struct v4l2_subdev *sd = &mipi_csi2->sd;
	int ret;

	if (WARN_ON(mipi_csi2->id >= MXC_MIPI_CSI2_MAX_DEVS))
		return -ENOENT;

	sd->grp_id = GRP_ID_MXC_MIPI_CSI2;
	ret = v4l2_device_register_subdev(&mxc_md->v4l2_dev, sd);
	if (!ret)
		mxc_md->mipi_csi2[mipi_csi2->id] = mipi_csi2;
	else
		v4l2_err(&mxc_md->v4l2_dev,
			 "Failed to register MIPI-CSIS.%d (%d)\n", mipi_csi2->id, ret);
	return ret;
}

static int register_parallel_csi_entity(struct mxc_md *mxc_md,
				struct mxc_parallel_csi_dev *pcsidev)
{
	struct v4l2_subdev *sd = &pcsidev->sd;
	int ret;

	sd->grp_id = GRP_ID_MXC_PARALLEL_CSI;
	ret = v4l2_device_register_subdev(&mxc_md->v4l2_dev, sd);
	if (!ret)
		mxc_md->pcsidev = pcsidev;
	else
		v4l2_err(&mxc_md->v4l2_dev,
			 "Failed to register PARALLEL CSI ret=(%d)\n", ret);
	return ret;
}

static int register_hdmi_rx_entity(struct mxc_md *mxc_md,
				struct mxc_hdmi_rx_dev *hdmi_rx)
{
	struct v4l2_subdev *sd = &hdmi_rx->sd;;

	dev_dbg(&mxc_md->pdev->dev, "%s\n", __func__);
	sd->grp_id = GRP_ID_MXC_HDMI_RX;
	mxc_md->hdmi_rx = hdmi_rx;

	return 0;
}

static int mxc_md_register_platform_entity(struct mxc_md *mxc_md,
					    struct platform_device *pdev,
					    int plat_entity)
{
	struct device *dev = &pdev->dev;
	int ret = -EPROBE_DEFER;
	void *drvdata;

	/* Lock to ensure dev->driver won't change. */
	device_lock(dev);

	if (!dev->driver || !try_module_get(dev->driver->owner))
		goto dev_unlock;

	drvdata = dev_get_drvdata(dev);
	/* Some subdev didn't probe successfully id drvdata is NULL */
	if (drvdata) {
		switch (plat_entity) {
		case IDX_ISI:
			ret = register_isi_entity(mxc_md, drvdata);
			break;
		case IDX_MIPI_CSI2:
			ret = register_mipi_csi2_entity(mxc_md, drvdata);
			break;
		case IDX_PARALLEL_CSI:
			ret = register_parallel_csi_entity(mxc_md, drvdata);
			break;
		case IDX_HDMI_RX:
			ret = register_hdmi_rx_entity(mxc_md, drvdata);
			break;
		default:
			ret = -ENODEV;
		}
	}
	module_put(dev->driver->owner);

dev_unlock:
	device_unlock(dev);
	if (ret == -EPROBE_DEFER)
		dev_info(&mxc_md->pdev->dev, "deferring %s device registration\n",
			dev_name(dev));
	else if (ret < 0)
		dev_err(&mxc_md->pdev->dev, "%s device registration failed (%d)\n",
			dev_name(dev), ret);

	return ret;
}

/* Register ISI, MIPI CSI2 and HDMI Rx Media entities */
static int mxc_md_register_platform_entities(struct mxc_md *mxc_md,
					      struct device_node *parent)
{
	struct device_node *node;
	int ret = 0;

	for_each_available_child_of_node(parent, node) {
		struct platform_device *pdev;
		int plat_entity = -1;

		pdev = of_find_device_by_node(node);
		if (!pdev)
			continue;

		/* If driver of any entity isn't ready try all again later. */
		if (!strcmp(node->name, ISI_OF_NODE_NAME))
			plat_entity = IDX_ISI;
		else if (!strcmp(node->name, MIPI_CSI2_OF_NODE_NAME))
			plat_entity = IDX_MIPI_CSI2;
		else if (!strcmp(node->name, PARALLEL_CSI_OF_NODE_NAME))
			plat_entity = IDX_PARALLEL_CSI;
		else if (!strcmp(node->name, MXC_HDMI_RX_NODE_NAME))
			plat_entity = IDX_HDMI_RX;

		if (plat_entity >= 0)
			ret = mxc_md_register_platform_entity(mxc_md, pdev,
							plat_entity);
		put_device(&pdev->dev);
		if (ret < 0)
			break;
	}

	return ret;
}

static void mxc_md_unregister_entities(struct mxc_md *mxc_md)
{
	int i;

	for (i = 0; i < MXC_ISI_MAX_DEVS; i++) {
		struct mxc_isi_dev *dev = mxc_md->mxc_isi[i];
		if (dev == NULL)
			continue;
		v4l2_device_unregister_subdev(&dev->isi_cap.sd);
		mxc_md->mxc_isi[i] = NULL;
	}
	for (i = 0; i < MXC_MIPI_CSI2_MAX_DEVS; i++) {
		if (mxc_md->mipi_csi2[i] == NULL)
			continue;
		v4l2_device_unregister_subdev(&mxc_md->mipi_csi2[i]->sd);
		mxc_md->mipi_csi2[i] = NULL;
	}

	if (mxc_md->pcsidev) {
		v4l2_device_unregister_subdev(&mxc_md->pcsidev->sd);
		mxc_md->pcsidev = NULL;
	}

	v4l2_info(&mxc_md->v4l2_dev, "Unregistered all entities\n");
}

static int mxc_md_link_notify(struct media_link *link, unsigned int flags,
				unsigned int notification)
{
	return 0;
}

static const struct media_device_ops mxc_md_ops = {
	.link_notify = mxc_md_link_notify,
};


static int mxc_md_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct mxc_md *mxc_md;
	int ret;

	mxc_md = devm_kzalloc(dev, sizeof(*mxc_md), GFP_KERNEL);
	if (!mxc_md)
		return -ENOMEM;

	mxc_md->pdev = pdev;
	platform_set_drvdata(pdev, mxc_md);

	mxc_md->parallel_csi = of_property_read_bool(dev->of_node, "parallel_csi");

	/* register media device  */
	strlcpy(mxc_md->media_dev.model, "FSL Capture Media Deivce",
		sizeof(mxc_md->media_dev.model));
	mxc_md->media_dev.ops = &mxc_md_ops;
	mxc_md->media_dev.dev = dev;

	/* register v4l2 device */
	v4l2_dev = &mxc_md->v4l2_dev;
	v4l2_dev->mdev = &mxc_md->media_dev;
	v4l2_dev->notify = mxc_sensor_notify;
	strlcpy(v4l2_dev->name, "mx8-img-md", sizeof(v4l2_dev->name));

	media_device_init(&mxc_md->media_dev);

	ret = v4l2_device_register(dev, &mxc_md->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_device: %d\n", ret);
		goto err_md;
	}

	ret = mxc_md_register_platform_entities(mxc_md, dev->of_node);
	if (ret < 0)
		goto err_v4l2_dev;

	ret = register_sensor_entities(mxc_md);
	if (ret < 0)
		goto err_m_ent;

	if (mxc_md->num_sensors > 0) {
		mxc_md->subdev_notifier.subdevs = mxc_md->async_subdevs;
		mxc_md->subdev_notifier.num_subdevs = mxc_md->num_sensors;
		mxc_md->subdev_notifier.bound = subdev_notifier_bound;
		mxc_md->subdev_notifier.complete = subdev_notifier_complete;
		mxc_md->num_sensors = 0;

		ret = v4l2_async_notifier_register(&mxc_md->v4l2_dev,
						&mxc_md->subdev_notifier);
		if (ret < 0) {
			dev_warn(&mxc_md->pdev->dev, "Sensor register failed\n");
			goto err_m_ent;
		}
	}

	return 0;

err_m_ent:
	mxc_md_unregister_entities(mxc_md);
err_v4l2_dev:
	v4l2_device_unregister(&mxc_md->v4l2_dev);
err_md:
	media_device_cleanup(&mxc_md->media_dev);
	return ret;
}

static int mxc_md_remove(struct platform_device *pdev)
{
	struct mxc_md *mxc_md = platform_get_drvdata(pdev);

	if (!mxc_md)
		return 0;

	v4l2_async_notifier_unregister(&mxc_md->subdev_notifier);

	v4l2_device_unregister(&mxc_md->v4l2_dev);
	mxc_md_unregister_entities(mxc_md);
	media_device_unregister(&mxc_md->media_dev);
	media_device_cleanup(&mxc_md->media_dev);

	return 0;
}

static const struct of_device_id mxc_md_of_match[] = {
	{	.compatible = "fsl,mxc-md",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mxc_md_of_match);


static struct platform_driver mxc_md_driver = {
	.driver = {
		.name = MXC_MD_DRIVER_NAME,
		.of_match_table	= mxc_md_of_match,
	},
	.probe = mxc_md_probe,
	.remove = mxc_md_remove,
};

module_platform_driver(mxc_md_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Media Device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MXC_MD_DRIVER_NAME);
