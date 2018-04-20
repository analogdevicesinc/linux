/*
 * Copyright 2017 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef MXC_MEDIA_DEV_H_
#define MXC_MEDIA_DEV_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>

#include "mxc-mipi-csi2.h"
#include "mxc-isi-core.h"
#include "mxc-isi-hw.h"

#define MXC_MD_DRIVER_NAME	"mxc-md"
#define MXC_MAX_MIPI_SENSORS 2

#define MJPEG_DEC_OF_NODE_NAME	"jpegdec"
#define MJPEG_ENC_OF_NODE_NAME	"jpegenc"

/*
 * The subdevices' group IDs.
 */
#define GRP_ID_MXC_SENSOR		(1 << 8)
#define GRP_ID_MXC_ISI			(1 << 9)
#define GRP_ID_MXC_MIPI_CSI2	(1 << 11)
#define GRP_ID_MXC_HDMI_IN		(1 << 12)
#define GRP_ID_MXC_MJPEG_DEC	(1 << 13)
#define GRP_ID_MXC_MJPEG_ENC	(1 << 14)
#define GRP_ID_MXC_PARALLEL_CSI (1 << 15)

enum mxc_subdev_index {
	IDX_SENSOR,
	IDX_ISI,
	IDX_MIPI_CSI2,
	IDX_HDMI_IN,
	IDX_MJPEG_ENC,
	IDX_MJPEG_DEC,
	IDX_PARALLEL_CSI,
	IDX_MAX,
};

struct mxc_sensor_info {
	int				id;
	struct v4l2_subdev		*sd;
	struct v4l2_async_subdev asd;
	bool mipi_mode;
	/* struct mxc_isi_dev *host; */
};

struct mxc_mjpeg_dec{
	struct v4l2_device		*v4l2_dev;
	struct v4l2_subdev		sd;
};

struct mxc_mjpeg_enc{
	struct v4l2_device		*v4l2_dev;
	struct v4l2_subdev		sd;
};

struct mxc_md {
	struct mxc_isi_dev *mxc_isi[MXC_ISI_MAX_DEVS];
	struct mxc_hdmi_in_dev *hdmi_in;
	struct mxc_parallel_csi_dev *pcsidev;
	struct mxc_mipi_csi2_dev *mipi_csi2[MXC_MIPI_CSI2_MAX_DEVS];
	struct mxc_sensor_info sensor[MXC_MAX_MIPI_SENSORS];
	struct mxc_mjpeg_dec  *mjpeg_dec;
	struct mxc_mjpeg_enc  *mjpeg_enc;

	int num_sensors;
	bool parallel_csi;

	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct platform_device *pdev;

	struct v4l2_async_notifier subdev_notifier;
	struct v4l2_async_subdev *async_subdevs[MXC_MAX_MIPI_SENSORS];
};

static inline struct mxc_md *notifier_to_mxc_md(struct v4l2_async_notifier *n)
{
	return container_of(n, struct mxc_md, subdev_notifier);
};

int mxc_isi_initialize_capture_subdev(struct mxc_isi_dev *mxc_isi);
void mxc_isi_unregister_capture_subdev(struct mxc_isi_dev *mxc_isi);
int mxc_isi_register_m2m_device(struct mxc_isi_dev *mxc_isi,
			     struct v4l2_device *v4l2_dev);
void mxc_sensor_notify(struct v4l2_subdev *sd, unsigned int notification,
			void *arg);
void mxc_isi_unregister_m2m_device(struct mxc_isi_dev *mxc_isi);
int mxc_isi_register_driver(void);
void mxc_isi_unregister_driver(void);
#endif
