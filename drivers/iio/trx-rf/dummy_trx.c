// SPDX-License-Identifier: GPL-2.0
/*
 * Dummy TRX Driver - JESD204 FSM Test Driver
 *
 * Modeled after the ADRV903X RF Transceiver driver structure, but with
 * all JESD204 FSM ops being dummy operations that only print the current
 * state. Useful for testing JESD204 FSM link bring-up without real
 * transceiver hardware.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/property.h>

#include <linux/iio/sysfs.h>
#include <linux/iio/iio.h>

#include <linux/jesd204/jesd204.h>

enum dummy_trx_iio_dev_attr {
	DUMMY_TRX_JESD204_FSM_ERROR,
	DUMMY_TRX_JESD204_FSM_PAUSED,
	DUMMY_TRX_JESD204_FSM_STATE,
	DUMMY_TRX_JESD204_FSM_RESUME,
	DUMMY_TRX_JESD204_FSM_CTRL,
};

struct dummy_trx_jesd204_priv {
	struct dummy_trx_phy *phy;
};

struct dummy_trx_phy {
	struct spi_device *spi;
	struct jesd204_dev *jdev;
	/* protect against device accesses */
	struct mutex lock;
	struct iio_dev *indio_dev;
	bool is_initialized;

	/* JESD link parameters (from DT, with defaults) */
	u8 jesd_l;
	u8 jesd_m;
	u8 jesd_s;
	u8 jesd_np;
	u8 jesd_f;
	u16 jesd_k;
	bool jesd204c;
	bool scrambling;
	bool high_density;
	u32 sample_rate_hz;
};

/*
 * JESD204 FSM ops - dummy implementations that only print the current state
 */

static int dummy_trx_jesd204_device_init(struct jesd204_dev *jdev,
					  enum jesd204_state_op_reason reason)
{
	struct dummy_trx_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "DEVICE_INIT: reason=%s\n",
		 jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		priv->phy->is_initialized = false;
		return JESD204_STATE_CHANGE_DONE;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_link_init(struct jesd204_dev *jdev,
					enum jesd204_state_op_reason reason,
					struct jesd204_link *lnk)
{
	struct dummy_trx_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct dummy_trx_phy *phy = priv->phy;

	dev_info(dev, "LINK_INIT: link_id=%u reason=%s\n",
		 lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* Populate link parameters from DT-configured values */
	lnk->num_lanes = phy->jesd_l;
	lnk->num_converters = phy->jesd_m;
	lnk->samples_per_conv_frame = phy->jesd_s;
	lnk->bits_per_sample = phy->jesd_np;
	lnk->converter_resolution = phy->jesd_np;
	lnk->octets_per_frame = phy->jesd_f;
	lnk->frames_per_multiframe = phy->jesd_k;
	lnk->scrambling = phy->scrambling;
	lnk->high_density = phy->high_density;
	lnk->jesd_version = phy->jesd204c ? JESD204_VERSION_C :
					     JESD204_VERSION_B;
	lnk->subclass = JESD204_SUBCLASS_1;
	lnk->jesd_encoder = phy->jesd204c ? JESD204_ENCODER_64B66B :
					     JESD204_ENCODER_8B10B;
	lnk->sample_rate = phy->sample_rate_hz;

	/* TX (deframer) links have link_id 0,1; RX (framer) links have 2,3 */
	lnk->is_transmit = (lnk->link_id <= 1);

	dev_info(dev, "  Link %u: %s L=%u M=%u S=%u NP=%u F=%u K=%u %s %s rate=%llu\n",
		 lnk->link_id, lnk->is_transmit ? "TX" : "RX",
		 lnk->num_lanes, lnk->num_converters,
		 lnk->samples_per_conv_frame, lnk->bits_per_sample,
		 lnk->octets_per_frame, lnk->frames_per_multiframe,
		 jesd204_encoder_str(lnk->jesd_encoder),
		 lnk->jesd_version == JESD204_VERSION_C ? "JESD204C" : "JESD204B",
		 lnk->sample_rate);

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_link_pre_setup(struct jesd204_dev *jdev,
					    enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "LINK_PRE_SETUP: reason=%s\n",
		 jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_link_setup(struct jesd204_dev *jdev,
					enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "LINK_SETUP: reason=%s\n",
		 jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_setup_stage1(struct jesd204_dev *jdev,
					  enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "OPT_SETUP_STAGE1: reason=%s\n",
		 jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_setup_stage2(struct jesd204_dev *jdev,
					  enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "OPT_SETUP_STAGE2: reason=%s\n",
		 jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_clks_enable(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason,
					 struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "CLOCKS_ENABLE: link_id=%u reason=%s\n",
		 lnk->link_id, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_link_enable(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason,
					 struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "LINK_ENABLE: link_id=%u reason=%s\n",
		 lnk->link_id, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_link_running(struct jesd204_dev *jdev,
					  enum jesd204_state_op_reason reason,
					  struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "LINK_RUNNING: link_id=%u reason=%s\n",
		 lnk->link_id, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int dummy_trx_jesd204_post_running_stage(struct jesd204_dev *jdev,
						enum jesd204_state_op_reason reason)
{
	struct dummy_trx_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_info(dev, "OPT_POST_RUNNING_STAGE: reason=%s\n",
		 jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		priv->phy->is_initialized = false;
		return JESD204_STATE_CHANGE_DONE;
	}

	priv->phy->is_initialized = true;
	dev_info(dev, "Dummy TRX successfully initialized via JESD204 FSM\n");

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_dummy_trx_init = {
	.state_ops = {
		[JESD204_OP_DEVICE_INIT] = {
			.per_device = dummy_trx_jesd204_device_init,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_LINK_INIT] = {
			.per_link = dummy_trx_jesd204_link_init,
		},
		[JESD204_OP_LINK_PRE_SETUP] = {
			.per_device = dummy_trx_jesd204_link_pre_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_device = dummy_trx_jesd204_link_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = dummy_trx_jesd204_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE2] = {
			.per_device = dummy_trx_jesd204_setup_stage2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = dummy_trx_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = dummy_trx_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = dummy_trx_jesd204_link_running,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = dummy_trx_jesd204_post_running_stage,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},

	.max_num_links = 4,
	.sizeof_priv = sizeof(struct dummy_trx_jesd204_priv),
};

/*
 * Sysfs attributes for JESD204 FSM control (pause/resume/start/stop)
 */

static ssize_t dummy_trx_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dummy_trx_phy *phy = iio_priv(indio_dev);
	bool enable;
	int ret = 0;

	guard(mutex)(&phy->lock);

	switch ((u32)this_attr->address) {
	case DUMMY_TRX_JESD204_FSM_RESUME:
		if (!phy->jdev)
			return -EOPNOTSUPP;
		ret = jesd204_fsm_resume(phy->jdev, JESD204_LINKS_ALL);
		break;
	case DUMMY_TRX_JESD204_FSM_CTRL:
		if (!phy->jdev)
			return -EOPNOTSUPP;
		ret = kstrtobool(buf, &enable);
		if (ret)
			break;
		if (enable) {
			jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
			jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
			ret = jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
		} else {
			jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
			jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t dummy_trx_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dummy_trx_phy *phy = iio_priv(indio_dev);
	struct jesd204_dev *jdev = phy->jdev;
	struct jesd204_link *links[4];
	int ret = 0, i, err, num_links;
	bool paused;

	guard(mutex)(&phy->lock);

	switch ((u32)this_attr->address) {
	case DUMMY_TRX_JESD204_FSM_ERROR:
		if (!jdev)
			return -EOPNOTSUPP;
		num_links = jesd204_get_active_links_num(jdev);
		if (num_links < 0)
			return num_links;
		ret = jesd204_get_links_data(jdev, links, num_links);
		if (ret)
			break;
		err = 0;
		for (i = 0; i < num_links; i++) {
			if (links[i]->error) {
				err = links[i]->error;
				break;
			}
		}
		ret = sysfs_emit(buf, "%d\n", err);
		break;
	case DUMMY_TRX_JESD204_FSM_PAUSED:
		if (!jdev)
			return -EOPNOTSUPP;
		num_links = jesd204_get_active_links_num(jdev);
		if (num_links < 0)
			return num_links;
		ret = jesd204_get_links_data(jdev, links, num_links);
		if (ret)
			break;
		paused = false;
		for (i = 0; i < num_links; i++) {
			if (jesd204_link_get_paused(links[i])) {
				paused = true;
				break;
			}
		}
		ret = sysfs_emit(buf, "%d\n", paused);
		break;
	case DUMMY_TRX_JESD204_FSM_STATE:
		if (!jdev)
			return -EOPNOTSUPP;
		num_links = jesd204_get_active_links_num(jdev);
		if (num_links < 0)
			return num_links;
		ret = jesd204_get_links_data(jdev, links, num_links);
		if (ret)
			break;
		ret = sysfs_emit(buf, "%s\n",
				 jesd204_link_get_state_str(links[0]));
		break;
	case DUMMY_TRX_JESD204_FSM_CTRL:
		if (!jdev)
			return -EOPNOTSUPP;
		ret = sysfs_emit(buf, "%d\n", phy->is_initialized);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static IIO_DEVICE_ATTR(jesd204_fsm_error, 0444,
		       dummy_trx_show, NULL,
		       DUMMY_TRX_JESD204_FSM_ERROR);

static IIO_DEVICE_ATTR(jesd204_fsm_paused, 0444,
		       dummy_trx_show, NULL,
		       DUMMY_TRX_JESD204_FSM_PAUSED);

static IIO_DEVICE_ATTR(jesd204_fsm_state, 0444,
		       dummy_trx_show, NULL,
		       DUMMY_TRX_JESD204_FSM_STATE);

static IIO_DEVICE_ATTR(jesd204_fsm_resume, 0200,
		       NULL, dummy_trx_store,
		       DUMMY_TRX_JESD204_FSM_RESUME);

static IIO_DEVICE_ATTR(jesd204_fsm_ctrl, 0644,
		       dummy_trx_show, dummy_trx_store,
		       DUMMY_TRX_JESD204_FSM_CTRL);

static struct attribute *dummy_trx_attributes[] = {
	&iio_dev_attr_jesd204_fsm_error.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_paused.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_state.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_resume.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group dummy_trx_attribute_group = {
	.attrs = dummy_trx_attributes,
};

static const struct iio_info dummy_trx_info = {
	.attrs = &dummy_trx_attribute_group,
};

/*
 * Probe / driver registration
 */

static int dummy_trx_probe(struct spi_device *spi)
{
	struct dummy_trx_jesd204_priv *priv;
	struct dummy_trx_phy *phy;
	struct iio_dev *indio_dev;
	struct jesd204_dev *jdev;
	u32 val;
	int ret;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_dummy_trx_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (!indio_dev)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;
	phy->jdev = jdev;

	ret = devm_mutex_init(&spi->dev, &phy->lock);
	if (ret)
		return ret;

	priv = jesd204_dev_priv(jdev);
	if (priv)
		priv->phy = phy;

	/* JESD link parameter defaults (matching typical JESD204C config) */
	phy->jesd_l = 8;
	phy->jesd_m = 8;
	phy->jesd_s = 4;
	phy->jesd_np = 16;
	phy->jesd_f = 1;
	phy->jesd_k = 256;
	phy->jesd204c = true;
	phy->scrambling = true;
	phy->high_density = true;
	phy->sample_rate_hz = 400000000;

	/* Override from DT if present */
	if (!device_property_read_u32(&spi->dev, "adi,jesd-l", &val))
		phy->jesd_l = val;
	if (!device_property_read_u32(&spi->dev, "adi,jesd-m", &val))
		phy->jesd_m = val;
	if (!device_property_read_u32(&spi->dev, "adi,jesd-s", &val))
		phy->jesd_s = val;
	if (!device_property_read_u32(&spi->dev, "adi,jesd-np", &val))
		phy->jesd_np = val;
	if (!device_property_read_u32(&spi->dev, "adi,jesd-f", &val))
		phy->jesd_f = val;
	if (!device_property_read_u32(&spi->dev, "adi,jesd-k", &val))
		phy->jesd_k = val;
	if (!device_property_read_u32(&spi->dev, "adi,jesd-version", &val))
		phy->jesd204c = (val >= 2);
	if (!device_property_read_u32(&spi->dev, "adi,jesd-scrambling", &val))
		phy->scrambling = !!val;
	if (!device_property_read_u32(&spi->dev, "adi,jesd-high-density", &val))
		phy->high_density = !!val;
	if (!device_property_read_u32(&spi->dev, "adi,sample-rate-hz", &val))
		phy->sample_rate_hz = val;

	dev_info(&spi->dev,
		 "Dummy TRX: JESD204%c L=%u M=%u S=%u NP=%u F=%u K=%u rate=%u Hz\n",
		 phy->jesd204c ? 'C' : 'B',
		 phy->jesd_l, phy->jesd_m, phy->jesd_s, phy->jesd_np,
		 phy->jesd_f, phy->jesd_k, phy->sample_rate_hz);

	indio_dev->name = "dummy-trx";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &dummy_trx_info;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		return ret;

	/* FSM is NOT auto-started; use jesd204_fsm_ctrl sysfs to start manually */
	dev_info(&spi->dev, "Dummy TRX ready. Start JESD204 FSM via: echo 1 > jesd204_fsm_ctrl\n");

	return 0;
}

static const struct spi_device_id dummy_trx_id[] = {
	{ "dummy-trx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, dummy_trx_id);

static const struct of_device_id dummy_trx_of_match[] = {
	{ .compatible = "adi,dummy-trx" },
	{ }
};
MODULE_DEVICE_TABLE(of, dummy_trx_of_match);

static struct spi_driver dummy_trx_driver = {
	.driver = {
		.name = "dummy-trx",
		.of_match_table = dummy_trx_of_match,
	},
	.probe = dummy_trx_probe,
	.id_table = dummy_trx_id,
};
module_spi_driver(dummy_trx_driver);

MODULE_AUTHOR("George Mois <george.mois@analog.com>");
MODULE_DESCRIPTION("Dummy TRX Driver - JESD204 FSM Test");
MODULE_LICENSE("GPL");
