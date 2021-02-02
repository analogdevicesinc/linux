// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * JESD204 Generic TOP device implementation
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define JESD204_OF_PREFIX	"adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>

#define JESD204_TOP_DEVICE_MAX_NUM_LINKS	16

struct jesd204_top_device_data {
	struct device *dev;
	struct jesd204_dev *jdev;
	struct jesd204_dev_data	jesd204_dev_data;
	struct mutex lock; /* Protect against concurrent accesses to the device */
	struct jesd204_link links[JESD204_TOP_DEVICE_MAX_NUM_LINKS];
	bool link_init_done;
	u32 d; /* dummy */
};

static int jes204_parse_jesd_link_dt(struct jesd204_top_device_data *tdev,
				     struct device_node *np,
				     struct jesd204_link *link)
{
	struct device *dev = tdev->dev;
	int ret;

	JESD204_LNK_READ_DEVICE_ID(dev, np, link, &tdev->d, 0);
	JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_SAMPLES_PER_CONVERTER_PER_FRAME(dev, np, link,
		&tdev->d, -1);
	JESD204_LNK_READ_HIGH_DENSITY(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_NUM_CONVERTERS(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_CTRL_BITS_PER_SAMPLE(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_NUM_LANES(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_VERSION(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_SUBCLASS(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_SCRAMBLING(dev, np, link, &tdev->d, 1);

	ret = of_property_read_u64(np, JESD204_OF_PREFIX "sample-rate",
		&link->sample_rate);
	if (ret || !link->sample_rate) {
		dev_err(dev, "Missing DT prop '%s' @ line %d ",
			JESD204_OF_PREFIX "sample-rate", __LINE__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, JESD204_OF_PREFIX "sample-rate-div",
		&link->sample_rate_div);
	if (ret)
		link->sample_rate_div = 1;

	return 0;
}

static int jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (lnk->link_id > JESD204_TOP_DEVICE_MAX_NUM_LINKS)
		return -EFAULT;

	if (!tdev->links[lnk->link_id].num_lanes)
		return -EFAULT;

	jesd204_copy_link_params(lnk, &tdev->links[lnk->link_id]);

	lnk->sample_rate = tdev->links[lnk->link_id].sample_rate;
	lnk->sample_rate_div = tdev->links[lnk->link_id].sample_rate_div;

	if (lnk->jesd_version == JESD204_VERSION_C)
		lnk->jesd_encoder = JESD204_ENCODER_64B66B;
	else
		lnk->jesd_encoder = JESD204_ENCODER_8B10B;

	return JESD204_STATE_CHANGE_DONE;
}

static int jesd204_link_post_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (lnk->link_id > JESD204_TOP_DEVICE_MAX_NUM_LINKS)
		return -EFAULT;

	WRITE_ONCE(tdev->link_init_done, reason == JESD204_STATE_OP_REASON_INIT);

	return JESD204_STATE_CHANGE_DONE;
}

static ssize_t jesd204_fsm_error_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);
	struct jesd204_link *links[JESD204_TOP_DEVICE_MAX_NUM_LINKS];
	int i, err, num_links, ret;

	if (!tdev->jdev)
		return -EOPNOTSUPP;

	mutex_lock(&tdev->lock);

	num_links = jesd204_get_active_links_num(tdev->jdev);
	if (num_links < 0) {
		ret = num_links;
		goto out;
	}

	ret = jesd204_get_links_data(tdev->jdev, links, num_links);
	if (ret)
		goto out;

	err = 0;
	for (i = 0; i < num_links; i++) {
		if (links[i]->error) {
			err = links[i]->error;
			break;
		}
	}

	ret = sysfs_emit(buf, "%d\n", err);
out:
	mutex_unlock(&tdev->lock);

	return ret;
}

static ssize_t jesd204_fsm_paused_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);
	struct jesd204_link *links[JESD204_TOP_DEVICE_MAX_NUM_LINKS];
	int i, num_links, ret;
	bool paused;

	if (!tdev->jdev)
		return -EOPNOTSUPP;

	mutex_lock(&tdev->lock);

	num_links = jesd204_get_active_links_num(tdev->jdev);
	if (num_links < 0) {
		ret = num_links;
		goto out;
	}
	ret = jesd204_get_links_data(tdev->jdev, links, num_links);
	if (ret)
		goto out;
	/*
	 * Take the slowest link; if there are N links and one is paused, all are paused.
	 * Not sure if this can happen yet, but best design it like this here.
	 */

	paused = false;
	for (i = 0; i < num_links; i++) {
		if (jesd204_link_get_paused(links[i])) {
			paused = true;
			break;
		}
	}

	ret = sysfs_emit(buf, "%d\n", paused);
out:
	mutex_unlock(&tdev->lock);

	return ret;
}

static ssize_t jesd204_fsm_state_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);
	struct jesd204_link *links[JESD204_TOP_DEVICE_MAX_NUM_LINKS];
	int num_links, ret;

	if (!tdev->jdev)
		return -EOPNOTSUPP;

	mutex_lock(&tdev->lock);

	num_links = jesd204_get_active_links_num(tdev->jdev);
	if (num_links < 0) {
		ret = num_links;
		goto out;
	}

	ret = jesd204_get_links_data(tdev->jdev, links, num_links);
	if (ret)
		goto out;

	ret = sysfs_emit(buf, "%s\n", jesd204_link_get_state_str(links[0]));
out:
	mutex_unlock(&tdev->lock);

	return ret;
}

static ssize_t jesd204_fsm_ctrl_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);

	if (!tdev->jdev)
		return -EOPNOTSUPP;

	return sysfs_emit(buf, "%d\n", READ_ONCE(tdev->link_init_done));
}

static ssize_t jesd204_fsm_ctrl_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);
	int ret;
	bool enable;

	if (!tdev->jdev)
		return -EOPNOTSUPP;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&tdev->lock);
	if (enable) {
		jesd204_fsm_stop(tdev->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_clear_errors(tdev->jdev, JESD204_LINKS_ALL);
		ret = jesd204_fsm_start(tdev->jdev, JESD204_LINKS_ALL);
	} else {
		jesd204_fsm_stop(tdev->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_clear_errors(tdev->jdev, JESD204_LINKS_ALL);
		ret = 0;
	}
	mutex_unlock(&tdev->lock);

	return ret ? ret : len;
}

static ssize_t jesd204_fsm_resume_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);
	int ret;
	bool enable;

	if (!tdev->jdev)
		return -EOPNOTSUPP;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	if (!enable)
		return 0;

	mutex_lock(&tdev->lock);
	ret = jesd204_fsm_resume(tdev->jdev, JESD204_LINKS_ALL);
	mutex_unlock(&tdev->lock);

	return ret ? ret : len;
}

static DEVICE_ATTR_RO(jesd204_fsm_error);
static DEVICE_ATTR_RO(jesd204_fsm_paused);
static DEVICE_ATTR_RO(jesd204_fsm_state);
static DEVICE_ATTR_WO(jesd204_fsm_resume);
static DEVICE_ATTR_RW(jesd204_fsm_ctrl);

static struct attribute *jesd204_fsm_attributes[] = {
	&dev_attr_jesd204_fsm_error.attr,
	&dev_attr_jesd204_fsm_state.attr,
	&dev_attr_jesd204_fsm_paused.attr,
	&dev_attr_jesd204_fsm_resume.attr,
	&dev_attr_jesd204_fsm_ctrl.attr,
	NULL,
};

static const struct attribute_group jesd204_fsm_attributes_group = {
	.attrs = jesd204_fsm_attributes,
};

/* Match table for of_platform binding */
static const struct of_device_id jesd204_top_device_of_match[] = {
	{ .compatible = "adi,jesd204-top-device-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, jesd204_top_device_of_match);

static int jesd204_top_device_probe(struct platform_device *pdev)
{
	struct jesd204_top_device_data *tdev;
	struct jesd204_dev *jdev;
	struct device_node *of_links, *of_link;
	int ret;
	u32 reg;

	if (!pdev->dev.of_node)
		return -ENODEV;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	tdev->dev = &pdev->dev;

	of_links = of_get_child_by_name(pdev->dev.of_node, "adi,jesd-links");
	if (!of_links)
		return -ENODEV;

	for_each_child_of_node(of_links, of_link) {
		ret = of_property_read_u32(of_link, "reg", &reg);
		if (ret || (!ret && reg >= JESD204_TOP_DEVICE_MAX_NUM_LINKS)) {
			dev_err(&pdev->dev, "Missing or invalid DT prop 'reg'\n");
			of_node_put(of_link);
			of_node_put(of_links);
			return -EINVAL;
		}
		ret = jes204_parse_jesd_link_dt(tdev, of_link, &tdev->links[reg]);
		if (!ret)
			tdev->jesd204_dev_data.max_num_links++;
	}

	of_node_put(of_links);

	tdev->jesd204_dev_data.state_ops[JESD204_OP_LINK_INIT].per_link =
		jesd204_link_init;
	tdev->jesd204_dev_data.state_ops[JESD204_OP_OPT_POST_RUNNING_STAGE].per_link =
		jesd204_link_post_running;

	dev_set_drvdata(&pdev->dev, tdev);
	mutex_init(&tdev->lock);

	jdev = devm_jesd204_dev_register(&pdev->dev, &tdev->jesd204_dev_data);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	tdev->jdev = jdev;

	ret = devm_device_add_group(&pdev->dev, &jesd204_fsm_attributes_group);
	if (ret != 0) {
		dev_err(&pdev->dev,
			"Failed to create attribute group: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "JESD204-GENERIC-TOP-DEVICE probed %d links\n",
		tdev->jesd204_dev_data.max_num_links);

	return jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
}

static struct platform_driver jesd204_top_device_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = jesd204_top_device_of_match,
	},
	.probe = jesd204_top_device_probe,
};
module_platform_driver(jesd204_top_device_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Generic jesd204 top-device");
MODULE_LICENSE("Dual BSD/GPL");
