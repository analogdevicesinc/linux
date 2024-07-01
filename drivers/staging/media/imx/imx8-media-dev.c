// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Media Controller Driver for NXP IMX8QXP/QM SOC
 *
 * Copyright 2019-2021 NXP
 *
 */

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/media-device.h>

#include "imx8-common.h"

#define MXC_MD_DRIVER_NAME	"mxc-md"
#define ISI_OF_NODE_NAME	"isi"
#define MIPI_CSI2_OF_NODE_NAME  "csi"

#define MXC_MAX_SENSORS		3
#define MXC_MIPI_CSI2_MAX_DEVS	2

#define MXC_NAME_LENS		32

/*
 * The subdevices' group IDs.
 */
#define GRP_ID_MXC_SENSOR		BIT(8)
#define GRP_ID_MXC_ISI			BIT(9)
#define GRP_ID_MXC_MIPI_CSI2		BIT(11)
#define GRP_ID_MXC_HDMI_RX		BIT(12)
#define GRP_ID_MXC_MJPEG_DEC		BIT(13)
#define GRP_ID_MXC_MJPEG_ENC		BIT(14)
#define GRP_ID_MXC_PARALLEL_CSI		BIT(15)

enum mxc_subdev_index {
	IDX_SENSOR,
	IDX_ISI,
	IDX_MIPI_CSI2,
	IDX_HDMI_RX,
	IDX_MJPEG_ENC,
	IDX_MJPEG_DEC,
	IDX_PARALLEL_CSI,
	IDX_MAX,
};

struct mxc_isi_info {
	struct v4l2_subdev *sd;
	struct media_entity *entity;
	struct device_node *node;
	u32 interface[MAX_PORTS];

	char vdev_name[MXC_NAME_LENS];
	char sd_name[MXC_NAME_LENS];
	int id;
};

struct mxc_mipi_csi2_info {
	struct v4l2_subdev *sd;
	struct media_entity *entity;
	struct device_node *node;

	char sd_name[MXC_NAME_LENS];
	int id;
	bool vchannel;
};

struct mxc_parallel_csi_info {
	struct v4l2_subdev *sd;
	struct media_entity *entity;
	struct device_node *node;

	char sd_name[MXC_NAME_LENS];
	int id;
};

struct mxc_hdmi_rx_info {
	struct v4l2_subdev *sd;
	struct media_entity *entity;
	struct device_node *node;

	char sd_name[MXC_NAME_LENS];
	int id;
};

struct mxc_sensor_info {
	int				id;
	struct v4l2_subdev		*sd;
	struct fwnode_handle *fwnode;
	bool mipi_mode;
};

struct mxc_md {
	struct mxc_isi_info		mxc_isi[MXC_ISI_MAX_DEVS];
	struct mxc_mipi_csi2_info	mipi_csi2[MXC_MIPI_CSI2_MAX_DEVS];
	struct mxc_parallel_csi_info	pcsidev;
	struct mxc_hdmi_rx_info		hdmi_rx;
	struct mxc_sensor_info		sensor[MXC_MAX_SENSORS];

	int link_status;
	int num_sensors;
	int valid_num_sensors;
	unsigned int nr_isi;
	bool parallel_csi;

	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct platform_device *pdev;

	struct v4l2_async_notifier subdev_notifier;
};

static inline struct mxc_md *notifier_to_mxc_md(struct v4l2_async_notifier *n)
{
	return container_of(n, struct mxc_md, subdev_notifier);
};

static void mxc_md_unregister_entities(struct mxc_md *mxc_md)
{
	struct mxc_parallel_csi_info *pcsidev = &mxc_md->pcsidev;
	struct mxc_hdmi_rx_info *hdmi_rx = &mxc_md->hdmi_rx;
	int i;

	for (i = 0; i < MXC_ISI_MAX_DEVS; i++) {
		struct mxc_isi_info *isi = &mxc_md->mxc_isi[i];

		if (!isi->sd)
			continue;
		v4l2_device_unregister_subdev(isi->sd);
		memset(isi, 0, sizeof(*isi));
	}

	for (i = 0; i < MXC_MIPI_CSI2_MAX_DEVS; i++) {
		struct mxc_mipi_csi2_info *mipi_csi2 = &mxc_md->mipi_csi2[i];
		if (!mipi_csi2->sd)
			continue;
		v4l2_device_unregister_subdev(mipi_csi2->sd);
		memset(mipi_csi2, 0, sizeof(*mipi_csi2));
	}

	if (pcsidev->sd)
		v4l2_device_unregister_subdev(pcsidev->sd);

	if (hdmi_rx->sd)
		v4l2_device_unregister_subdev(hdmi_rx->sd);

	v4l2_info(&mxc_md->v4l2_dev, "Unregistered all entities\n");
}

static struct media_entity *find_entity_by_name(struct mxc_md *mxc_md,
						const char *name)
{
	struct media_entity *ent = NULL;

	if (!mxc_md || !name)
		return NULL;

	media_device_for_each_entity(ent, &mxc_md->media_dev) {
		if (!strcmp(ent->name, name)) {
			dev_dbg(&mxc_md->pdev->dev,
				"%s entity is found\n", ent->name);
			return ent;
		}
	}

	return NULL;
}

static int mxc_md_do_clean(struct mxc_md *mxc_md, struct media_pad *pad)
{
	struct device *dev = &mxc_md->pdev->dev;
	struct media_pad *remote_pad;
	struct v4l2_subdev	*subdev;

	if (!pad->entity->num_links)
		return 0;

	remote_pad = media_pad_remote_pad_first(pad);
	if (remote_pad == NULL) {
		dev_err(dev, "%s get remote pad fail\n", __func__);
		return -ENODEV;
	}

	subdev = media_entity_to_v4l2_subdev(remote_pad->entity);
	if (subdev == NULL) {
		dev_err(dev, "%s media entity to v4l2 subdev fail\n", __func__);
		return -ENODEV;
	}

	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);

	pr_info("clean ISI channel: %s\n", subdev->name);

	return 0;
}

static int mxc_md_clean_channel(struct mxc_md *mxc_md, int index)
{
	struct mxc_sensor_info *sensor = &mxc_md->sensor[index];
	struct mxc_mipi_csi2_info *mipi_csi2;
	struct mxc_parallel_csi_info *pcsidev;
	struct media_pad *local_pad;
	struct media_entity *local_en;
	u32 i, mipi_vc = 0;
	int ret;

	if (mxc_md->mipi_csi2[index].sd) {
		mipi_csi2 = &mxc_md->mipi_csi2[index];

		if (mipi_csi2->vchannel == true)
			mipi_vc = 4;
		else
			mipi_vc = 1;

		local_en = &mipi_csi2->sd->entity;
		if (local_en == NULL)
			return -ENODEV;

		for (i = 0; i < mipi_vc; i++) {
			local_pad = &local_en->pads[MXC_MIPI_CSI2_VC0_PAD_SOURCE + i];
			ret = mxc_md_do_clean(mxc_md, local_pad);
			if (ret < 0)
				return -ENODEV;
		}
	} else if (mxc_md->parallel_csi && !sensor->mipi_mode) {
		pcsidev = &mxc_md->pcsidev;
		if (pcsidev->sd == NULL)
			return -ENODEV;

		local_en = &pcsidev->sd->entity;
		if (local_en == NULL)
			return -ENODEV;

		local_pad = &local_en->pads[MXC_PARALLEL_CSI_PAD_SOURCE];
		ret = mxc_md_do_clean(mxc_md, local_pad);
		if (ret < 0)
			return -ENODEV;
	}

	return 0;
}

static int mxc_md_clean_unlink_channels(struct mxc_md *mxc_md)
{
	struct mxc_sensor_info *sensor;
	int num_subdevs = mxc_md->num_sensors;
	int i, ret;

	for (i = 0; i < num_subdevs; i++) {
		sensor = &mxc_md->sensor[i];
		if (sensor->sd != NULL)
			continue;

		ret = mxc_md_clean_channel(mxc_md, i);
		if (ret < 0) {
			pr_err("%s: clean channel fail(%d)\n", __func__, i);
			return ret;
		}
	}

	return 0;
}

static void mxc_md_unregister_all(struct mxc_md *mxc_md)
{
	struct mxc_isi_info *mxc_isi;
	int i;

	for (i = 0; i < MXC_ISI_MAX_DEVS; i++) {
		mxc_isi = &mxc_md->mxc_isi[i];
		if (!mxc_isi->sd)
			continue;

		v4l2_device_unregister_subdev(mxc_isi->sd);
		media_entity_cleanup(&mxc_isi->sd->entity);

		pr_info("unregister ISI channel: %s\n", mxc_isi->sd->name);
	}
}

static int mxc_md_create_links(struct mxc_md *mxc_md)
{
	struct media_entity *source, *sink;
	struct mxc_isi_info *mxc_isi;
	struct mxc_sensor_info *sensor;
	struct mxc_mipi_csi2_info *mipi_csi2;
	struct mxc_parallel_csi_info *pcsidev;
	struct mxc_hdmi_rx_info *hdmi_rx;
	int num_sensors = mxc_md->num_sensors;
	int i, j, ret = 0;
	u16  source_pad, sink_pad;
	u32 flags;
	u32 mipi_vc = 0;

	/* Create links between each ISI's subdev and video node */
	flags = MEDIA_LNK_FL_ENABLED;
	for (i = 0; i < MXC_ISI_MAX_DEVS; i++) {
		mxc_isi = &mxc_md->mxc_isi[i];
		if (!mxc_isi->sd)
			continue;

		/* Connect ISI source to video device */
		source = find_entity_by_name(mxc_md, mxc_isi->sd_name);
		sink = find_entity_by_name(mxc_md, mxc_isi->vdev_name);
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
			v4l2_err(&mxc_md->v4l2_dev,
				 "Failed created link [%s] %c> [%s]\n",
				 source->name, flags ? '=' : '-', sink->name);
			break;
		}

		/* Notify capture subdev entity ,ISI cap link setup */
		ret = media_entity_call(source, link_setup, &source->pads[source_pad],
					&sink->pads[sink_pad], flags);
		if (ret) {
			v4l2_err(&mxc_md->v4l2_dev,
				 "failed call link_setup [%s] %c> [%s]\n",
				 source->name, flags ? '=' : '-', sink->name);
			break;
		}

		v4l2_info(&mxc_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);

		/* Connect MIPI/HDMI/Mem source to ISI sink */
		sink = find_entity_by_name(mxc_md, mxc_isi->sd_name);

		switch (mxc_isi->interface[IN_PORT]) {
		case ISI_INPUT_INTERFACE_MIPI0_CSI2:
			mipi_csi2 = &mxc_md->mipi_csi2[0];
			if (!mipi_csi2->sd)
				continue;
			source = find_entity_by_name(mxc_md, mipi_csi2->sd_name);

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
			mipi_csi2 = &mxc_md->mipi_csi2[1];
			if (!mipi_csi2->sd)
				continue;
			source = find_entity_by_name(mxc_md, mipi_csi2->sd_name);

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
			pcsidev = &mxc_md->pcsidev;
			if (!pcsidev->sd)
				continue;
			source = find_entity_by_name(mxc_md, pcsidev->sd_name);
			source_pad = MXC_PARALLEL_CSI_PAD_SOURCE;
			sink_pad   = MXC_ISI_SD_PAD_SINK_PARALLEL_CSI;
			break;

		case ISI_INPUT_INTERFACE_HDMI:
			hdmi_rx = &mxc_md->hdmi_rx;
			if (!hdmi_rx->sd)
				continue;
			source = find_entity_by_name(mxc_md, hdmi_rx->sd_name);
			source_pad = MXC_HDMI_RX_PAD_SOURCE;
			sink_pad = MXC_ISI_SD_PAD_SINK_HDMI;
			break;

		case ISI_INPUT_INTERFACE_DC0:
		case ISI_INPUT_INTERFACE_DC1:
		case ISI_INPUT_INTERFACE_MEM:
		default:
			v4l2_err(&mxc_md->v4l2_dev,
				 "Not support input interface: %x\n",
				 mxc_isi->interface[IN_PORT]);
			return -EINVAL;
		}

		/* Create link MIPI/HDMI to ISI */
		ret = media_create_pad_link(source, source_pad, sink, sink_pad, flags);
		if (ret) {
			v4l2_err(&mxc_md->v4l2_dev,
				 "created link [%s] %c> [%s] fail\n",
				 source->name, flags ? '=' : '-', sink->name);
			break;
		}

		/* Notify ISI subdev entity */
		ret = media_entity_call(sink, link_setup,
					&sink->pads[sink_pad],
					&source->pads[source_pad], 0);
		if (ret)
			break;

		/* Notify MIPI/HDMI entity */
		ret = media_entity_call(source, link_setup,
					&source->pads[source_pad],
					&sink->pads[sink_pad], 0);
		if (ret)
			break;

		v4l2_info(&mxc_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
	}

	/* Connect MIPI Sensor to MIPI CSI2 */
	for (i = 0; i < num_sensors; i++) {
		sensor = &mxc_md->sensor[i];
		if (!sensor || !sensor->sd)
			continue;

		if (mxc_md->parallel_csi && !sensor->mipi_mode) {
			pcsidev = &mxc_md->pcsidev;
			if (!pcsidev->sd)
				continue;
			source = &sensor->sd->entity;
			sink = find_entity_by_name(mxc_md, pcsidev->sd_name);

			source_pad = 0;
			sink_pad = MXC_PARALLEL_CSI_PAD_SINK;

			ret = media_create_pad_link(source,
						    source_pad,
						    sink,
						    sink_pad,
						    MEDIA_LNK_FL_IMMUTABLE |
						    MEDIA_LNK_FL_ENABLED);
			if (ret)
				return ret;

			/* Notify MIPI subdev entity */
			ret = media_entity_call(sink, link_setup,
						&sink->pads[sink_pad],
						&source->pads[source_pad], 0);
			if (ret)
				return ret;

			/* Notify MIPI sensor subdev entity */
			ret = media_entity_call(source, link_setup,
						&source->pads[source_pad],
						&sink->pads[sink_pad],
						0);
			if (ret)
				return ret;
			v4l2_info(&mxc_md->v4l2_dev,
				  "created link [%s] => [%s]\n",
				  source->name, sink->name);
		} else if (mxc_md->mipi_csi2[sensor->id].sd) {
			mipi_csi2 = &mxc_md->mipi_csi2[sensor->id];

			source = &sensor->sd->entity;
			sink = find_entity_by_name(mxc_md, mipi_csi2->sd_name);
			source_pad = 0;
			sink_pad = source_pad;

			mipi_vc = (mipi_csi2->vchannel) ? 4 : 1;
			for (j = 0; j < mipi_vc; j++) {
				ret = media_create_pad_link(source,
							    source_pad + j,
							    sink,
							    sink_pad + j,
							    MEDIA_LNK_FL_IMMUTABLE |
							    MEDIA_LNK_FL_ENABLED);
				if (ret)
					return ret;

				/* Notify MIPI subdev entity */
				ret = media_entity_call(sink, link_setup,
							&sink->pads[sink_pad + j],
							&source->pads[source_pad + j],
							0);
				if (ret)
					return ret;

				/* Notify MIPI sensor subdev entity */
				ret = media_entity_call(source, link_setup,
							&source->pads[source_pad + j],
							&sink->pads[sink_pad + j],
							0);
				if (ret)
					return ret;
			}
			v4l2_info(&mxc_md->v4l2_dev,
				  "created link [%s] => [%s]\n",
				  source->name, sink->name);
		}
	}
	dev_info(&mxc_md->pdev->dev, "%s\n", __func__);
	return 0;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_connection *asd)
{
	struct mxc_md *mxc_md = notifier_to_mxc_md(notifier);
	struct mxc_sensor_info *sensor = NULL;
	int i;

	dev_dbg(&mxc_md->pdev->dev, "%s\n", __func__);

	/* Find platform data for this sensor subdev */
	for (i = 0; i < ARRAY_SIZE(mxc_md->sensor); i++) {
		if (mxc_md->sensor[i].fwnode ==
		    of_fwnode_handle(sd->dev->of_node)) {
			sensor = &mxc_md->sensor[i];
		}
	}

	if (!sensor)
		return -EINVAL;

	sd->grp_id = GRP_ID_MXC_SENSOR;
	sensor->sd = sd;
	mxc_md->valid_num_sensors++;

	v4l2_info(&mxc_md->v4l2_dev, "Registered sensor subdevice: %s (%d)\n",
		  sd->name, mxc_md->valid_num_sensors);

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

	mxc_md->link_status = 1;

	ret = v4l2_device_register_subdev_nodes(&mxc_md->v4l2_dev);
unlock:
	mutex_unlock(&mxc_md->media_dev.graph_mutex);
	if (ret < 0) {
		v4l2_err(&mxc_md->v4l2_dev, "%s error exit\n", __func__);
		return ret;
	}

	if (mxc_md->media_dev.devnode)
		return ret;

	return media_device_register(&mxc_md->media_dev);
}

static const struct v4l2_async_notifier_operations sd_async_notifier_ops = {
	.bound = subdev_notifier_bound,
	.complete = subdev_notifier_complete,
};

static void mxc_sensor_notify(struct v4l2_subdev *sd, unsigned int notification,
		       void *arg)
{
}

static int mxc_md_link_notify(struct media_link *link, unsigned int flags,
			      unsigned int notification)
{
	return 0;
}

static const struct media_device_ops mxc_md_ops = {
	.link_notify = mxc_md_link_notify,
};

static struct mxc_isi_info *mxc_md_parse_isi_entity(struct mxc_md *mxc_md,
						    struct device_node *node)
{
	struct device *dev;
	struct mxc_isi_info *mxc_isi;
	struct device_node *child;
	int ret, id = -1;

	if (!mxc_md || !node)
		return NULL;

	dev = &mxc_md->pdev->dev;

	id = of_alias_get_id(node, ISI_OF_NODE_NAME);
	if (id < 0 || id >= MXC_ISI_MAX_DEVS)
		return NULL;

	mxc_isi = &mxc_md->mxc_isi[id];

	child = of_get_child_by_name(node, "cap_device");
	if (!child) {
		dev_err(dev, "Can not get child node for %s.%d\n",
			ISI_OF_NODE_NAME, id);
		return NULL;
	}
	of_node_put(child);

	mxc_isi->id = id;
	mxc_isi->node = child;
	sprintf(mxc_isi->sd_name, "mxc_isi.%d", mxc_isi->id);
	sprintf(mxc_isi->vdev_name, "mxc_isi.%d.capture", mxc_isi->id);

	ret = of_property_read_u32_array(node, "interface",
					 mxc_isi->interface, 3);
	if (ret < 0) {
		dev_err(dev, "%s node has not interface property\n", child->name);
		return NULL;
	}

	return mxc_isi;
}

static struct mxc_mipi_csi2_info *
mxc_md_parse_csi_entity(struct mxc_md *mxc_md,
			struct device_node *node)
{
	struct mxc_mipi_csi2_info *mipi_csi2;
	int id = -1;

	if (!mxc_md || !node)
		return NULL;

	id = of_alias_get_id(node, MIPI_CSI2_OF_NODE_NAME);
	if (id < 0 || id >= MXC_MIPI_CSI2_MAX_DEVS)
		return NULL;

	mipi_csi2 = &mxc_md->mipi_csi2[id];
	if (!mipi_csi2)
		return NULL;

	mipi_csi2->vchannel = of_property_read_bool(node, "virtual-channel");
	mipi_csi2->id = id;
	mipi_csi2->node = node;
	sprintf(mipi_csi2->sd_name, "mxc-mipi-csi2.%d", mipi_csi2->id);

	return mipi_csi2;
}

static struct mxc_parallel_csi_info*
mxc_md_parse_pcsi_entity(struct mxc_md *mxc_md, struct device_node *node)
{
	struct mxc_parallel_csi_info *pcsidev;

	if (!mxc_md || !node)
		return NULL;

	pcsidev = &mxc_md->pcsidev;
	if (!pcsidev)
		return NULL;

	pcsidev->node = node;
	sprintf(pcsidev->sd_name, "mxc-parallel-csi");

	return pcsidev;
}

static struct mxc_hdmi_rx_info*
mxc_md_parse_hdmi_rx_entity(struct mxc_md *mxc_md, struct device_node *node)
{
	struct mxc_hdmi_rx_info *hdmi_rx;

	if (!mxc_md || !node)
		return NULL;

	hdmi_rx = &mxc_md->hdmi_rx;
	if (!hdmi_rx)
		return NULL;

	hdmi_rx->node = node;
	sprintf(hdmi_rx->sd_name, "mxc-hdmi-rx");

	return hdmi_rx;
}

static struct v4l2_subdev *get_subdev_by_node(struct device_node *node)
{
	struct platform_device *pdev;
	struct v4l2_subdev *sd = NULL;
	struct device *dev;
	void *drvdata;

	pdev = of_find_device_by_node(node);
	if (!pdev)
		return NULL;

	dev = &pdev->dev;
	device_lock(&pdev->dev);
	if (!dev->driver || !try_module_get(dev->driver->owner))
		goto dev_unlock;

	drvdata = dev_get_drvdata(dev);
	if (!drvdata)
		goto module_put;

	sd = (struct v4l2_subdev *)drvdata;

module_put:
	module_put(dev->driver->owner);
dev_unlock:
	device_unlock(dev);
	return sd;
}

static int register_isi_entity(struct mxc_md *mxc_md,
			       struct mxc_isi_info *mxc_isi)
{
	struct v4l2_subdev *sd;
	int ret = 0;

	sd = get_subdev_by_node(mxc_isi->node);
	if (!sd) {
		ret = of_device_is_available(mxc_isi->node);
		if (!ret) {
			dev_info(&mxc_md->pdev->dev, "%s device is disabled\n",
				 mxc_isi->node->name);
		} else {
			dev_info(&mxc_md->pdev->dev,
				 "deferring %s registration\n",
				 mxc_isi->node->name);
			ret = -EPROBE_DEFER;
		}
		return ret;
	}

	if (mxc_isi->id >= MXC_ISI_MAX_DEVS)
		return -EBUSY;

	sd->grp_id = GRP_ID_MXC_ISI;

	ret = v4l2_device_register_subdev(&mxc_md->v4l2_dev, sd);
	if (!ret)
		mxc_isi->sd = sd;
	else
		v4l2_err(&mxc_md->v4l2_dev, "Failed to register ISI.%d (%d)\n",
			 mxc_isi->id, ret);
	return ret;
}

static int register_mipi_csi2_entity(struct mxc_md *mxc_md,
				     struct mxc_mipi_csi2_info *mipi_csi2)
{
	struct v4l2_subdev *sd;
	int ret;

	sd = get_subdev_by_node(mipi_csi2->node);
	if (!sd) {
		dev_info(&mxc_md->pdev->dev,
			 "deferring %s device registration\n",
			 mipi_csi2->node->name);
		return -EPROBE_DEFER;
	}

	if (mipi_csi2->id >= MXC_MIPI_CSI2_MAX_DEVS)
		return -EBUSY;

	sd->grp_id = GRP_ID_MXC_MIPI_CSI2;

	ret = v4l2_device_register_subdev(&mxc_md->v4l2_dev, sd);
	if (!ret)
		mipi_csi2->sd = sd;
	else
		v4l2_err(&mxc_md->v4l2_dev, "Failed to register MIPI-CSI.%d (%d)\n",
			 mipi_csi2->id, ret);
	return ret;
}

static int register_parallel_csi_entity(struct mxc_md *mxc_md,
					struct mxc_parallel_csi_info *pcsidev)
{
	struct v4l2_subdev *sd;
	int ret;

	sd = get_subdev_by_node(pcsidev->node);
	if (!sd) {
		dev_info(&mxc_md->pdev->dev,
			 "deferring %s device registration\n",
			 pcsidev->node->name);
		return -EPROBE_DEFER;
	}

	sd->grp_id = GRP_ID_MXC_PARALLEL_CSI;

	ret = v4l2_device_register_subdev(&mxc_md->v4l2_dev, sd);
	if (!ret)
		pcsidev->sd = sd;
	else
		v4l2_err(&mxc_md->v4l2_dev,
			"Failed to register Parallel (%d)\n", ret);
	return ret;
}

static int register_hdmi_rx_entity(struct mxc_md *mxc_md,
			struct mxc_hdmi_rx_info *hdmi_rx)
{
	struct v4l2_subdev *sd;

	sd = get_subdev_by_node(hdmi_rx->node);
	if (!sd) {
		dev_info(&mxc_md->pdev->dev,
			 "deferring %s device registration\n",
			 hdmi_rx->node->name);
		return -EPROBE_DEFER;
	}

	sd->grp_id = GRP_ID_MXC_HDMI_RX;

	hdmi_rx->sd = sd;

	return true;
}

static int mxc_md_register_platform_entity(struct mxc_md *mxc_md,
					   struct device_node *node,
					   int plat_entity)
{
	struct device *dev = &mxc_md->pdev->dev;
	struct mxc_isi_info *isi;
	struct mxc_mipi_csi2_info *mipi_csi2;
	struct mxc_parallel_csi_info *pcsidev;
	struct mxc_hdmi_rx_info *hdmi_rx;
	int ret = -EINVAL;

	switch (plat_entity) {
	case IDX_ISI:
		isi = mxc_md_parse_isi_entity(mxc_md, node);
		if (!isi)
			return -ENODEV;
		ret = register_isi_entity(mxc_md, isi);
		break;
	case IDX_MIPI_CSI2:
		mipi_csi2 = mxc_md_parse_csi_entity(mxc_md, node);
		if (!mipi_csi2)
			return -ENODEV;
		ret = register_mipi_csi2_entity(mxc_md, mipi_csi2);
		break;
	case IDX_PARALLEL_CSI:
		pcsidev = mxc_md_parse_pcsi_entity(mxc_md, node);
		if (!pcsidev)
			return -ENODEV;
		ret = register_parallel_csi_entity(mxc_md, pcsidev);
		break;
	case IDX_HDMI_RX:
		hdmi_rx = mxc_md_parse_hdmi_rx_entity(mxc_md, node);
		if (!hdmi_rx)
			return -ENODEV;
		ret = register_hdmi_rx_entity(mxc_md, hdmi_rx);
		break;
	default:
		dev_err(dev, "Invalid platform entity (%d)", plat_entity);
		return ret;
	}

	return ret;
}

static int mxc_md_register_platform_entities(struct mxc_md *mxc_md,
					     struct device_node *parent)
{
	struct device_node *node;
	int ret = 0;

	for_each_available_child_of_node(parent, node) {
		int plat_entity = -1;

		if (!of_device_is_available(node))
			continue;

		/* If driver of any entity isn't ready try all again later. */
		if (!strcmp(node->name, ISI_OF_NODE_NAME))
			plat_entity = IDX_ISI;
		else if (!strcmp(node->name, MIPI_CSI2_OF_NODE_NAME))
			plat_entity = IDX_MIPI_CSI2;
		else if (!strcmp(node->name, PARALLEL_OF_NODE_NAME))
			plat_entity = IDX_PARALLEL_CSI;
		else if (!strcmp(node->name, HDMI_RX_OF_NODE_NAME))
			plat_entity = IDX_HDMI_RX;

		if (plat_entity >= IDX_SENSOR && plat_entity < IDX_MAX) {
			ret = mxc_md_register_platform_entity(mxc_md, node,
							      plat_entity);
			if (ret < 0)
				break;
		}
	}

	return ret;
}

static int register_sensor_entities(struct mxc_md *mxc_md)
{
	struct device_node *parent = mxc_md->pdev->dev.of_node;
	struct device_node *node, *ep, *rem;
	struct v4l2_fwnode_endpoint endpoint;
	struct i2c_client *client;
	struct v4l2_async_connection *asd;
	int index = 0;
	int ret;

	mxc_md->num_sensors = 0;

	/* Attach sensors linked to MIPI CSI2 / paralle csi / HDMI Rx */
	for_each_available_child_of_node(parent, node) {
		struct device_node *port;

		if (!of_node_cmp(node->name, HDMI_RX_OF_NODE_NAME)) {
			mxc_md->sensor[index].fwnode = of_fwnode_handle(node);
				v4l2_async_nf_add_fwnode(
						&mxc_md->subdev_notifier,
						mxc_md->sensor[index].fwnode,
						struct v4l2_async_connection);
			mxc_md->num_sensors++;
			index++;
			continue;
		}

		if (of_node_cmp(node->name, MIPI_CSI2_OF_NODE_NAME) &&
		    of_node_cmp(node->name, PARALLEL_OF_NODE_NAME))
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

		memset(&endpoint, 0, sizeof(endpoint));
		ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &endpoint);
		if (WARN_ON(endpoint.base.port >= MXC_MAX_SENSORS || ret)) {
			v4l2_err(&mxc_md->v4l2_dev,
				 "Failed to get sensor endpoint\n");
			return -EINVAL;
		}

		mxc_md->sensor[index].id = endpoint.base.port;

		if (!of_node_cmp(node->name, MIPI_CSI2_OF_NODE_NAME))
			mxc_md->sensor[index].mipi_mode = true;

		/* remote port---sensor node */
		rem = of_graph_get_remote_port_parent(ep);
		of_node_put(ep);
		if (!rem) {
			v4l2_info(&mxc_md->v4l2_dev,
				  "Remote device at %s not found\n",
				  ep->full_name);
			continue;
		}

		/*
		 * Need to wait sensor driver probed for the first time
		 */
		client = of_find_i2c_device_by_node(rem);
		if (!client) {
			v4l2_info(&mxc_md->v4l2_dev,
				  "Can't find i2c client device for %s\n",
				  of_node_full_name(rem));
			return -EPROBE_DEFER;
		}

		mxc_md->sensor[index].fwnode = of_fwnode_handle(rem);
		asd = v4l2_async_nf_add_fwnode(
						&mxc_md->subdev_notifier,
						mxc_md->sensor[index].fwnode,
						struct v4l2_async_connection);
		if (IS_ERR(asd)) {
			v4l2_info(&mxc_md->v4l2_dev, "Can't find async subdev\n");
			return PTR_ERR(asd);
		}

		mxc_md->num_sensors++;

		index++;
	}

	return 0;
}

static int mxc_md_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *nd = dev->of_node;
	struct v4l2_device *v4l2_dev;
	struct mxc_md *mxc_md;
	int ret;

	mxc_md = devm_kzalloc(dev, sizeof(*mxc_md), GFP_KERNEL);
	if (!mxc_md)
		return -ENOMEM;

	mxc_md->pdev = pdev;
	platform_set_drvdata(pdev, mxc_md);

	mxc_md->parallel_csi = of_property_read_bool(nd, "parallel_csi");

	/* register media device  */
	strscpy(mxc_md->media_dev.model, "FSL Capture Media Device",
		sizeof(mxc_md->media_dev.model));
	mxc_md->media_dev.ops = &mxc_md_ops;
	mxc_md->media_dev.dev = dev;

	/* register v4l2 device */
	v4l2_dev = &mxc_md->v4l2_dev;
	v4l2_dev->mdev = &mxc_md->media_dev;
	v4l2_dev->notify = mxc_sensor_notify;
	strscpy(v4l2_dev->name, "mx8-img-md", sizeof(v4l2_dev->name));

	media_device_init(&mxc_md->media_dev);

	ret = v4l2_device_register(dev, &mxc_md->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_device (%d)\n", ret);
		goto clean_md;
	}

	v4l2_async_nf_init(&mxc_md->subdev_notifier, &mxc_md->v4l2_dev);
	ret = mxc_md_register_platform_entities(mxc_md, dev->of_node);
	if (ret < 0)
		goto clean_v4l2;

	ret = register_sensor_entities(mxc_md);
	if (ret < 0)
		goto clean_ents;

	if (mxc_md->num_sensors > 0) {
		mxc_md->subdev_notifier.ops = &sd_async_notifier_ops;
		mxc_md->valid_num_sensors = 0;
		mxc_md->link_status = 0;

		ret = v4l2_async_nf_register(&mxc_md->subdev_notifier);
		if (ret < 0) {
			dev_warn(&mxc_md->pdev->dev, "Sensor register failed\n");
			goto clean_ents;
		}

		if (!mxc_md->link_status) {
			if (mxc_md->valid_num_sensors > 0) {
				ret = subdev_notifier_complete(&mxc_md->subdev_notifier);
				if (ret < 0)
					goto err_register_nf;

				mxc_md_clean_unlink_channels(mxc_md);
			} else {
				/* no sensors connected */
				mxc_md_unregister_all(mxc_md);
				v4l2_async_nf_unregister(&mxc_md->subdev_notifier);
				v4l2_async_nf_cleanup(&mxc_md->subdev_notifier);
			}
		}
	}

	return 0;

err_register_nf:
	v4l2_async_nf_unregister(&mxc_md->subdev_notifier);
	v4l2_async_nf_cleanup(&mxc_md->subdev_notifier);
clean_ents:
	mxc_md_unregister_entities(mxc_md);
clean_v4l2:
	v4l2_device_unregister(&mxc_md->v4l2_dev);
clean_md:
	media_device_cleanup(&mxc_md->media_dev);
	return ret;
}

static void mxc_md_remove(struct platform_device *pdev)
{
	struct mxc_md *mxc_md = platform_get_drvdata(pdev);

	if (!mxc_md)
		return;

	v4l2_async_nf_unregister(&mxc_md->subdev_notifier);
	v4l2_async_nf_cleanup(&mxc_md->subdev_notifier);

	v4l2_device_unregister(&mxc_md->v4l2_dev);
	mxc_md_unregister_entities(mxc_md);
	media_device_unregister(&mxc_md->media_dev);
	media_device_cleanup(&mxc_md->media_dev);
}

static const struct of_device_id mxc_md_of_match[] = {
	{ .compatible = "fsl,mxc-md",},
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
