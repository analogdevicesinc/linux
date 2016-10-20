/*
 * MathWorks AXI DMA Driver
 *
 * Copyright 2014-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */


#include "mathworks_ipcore.h"
#include "mw_stream_channel.h"
#include "mw_stream_iio_channel.h"

#include <linux/version.h>

/*
 * @brief mathworks_ipcore_of_match
 */

struct mw_dev_info subdev_dev_info = {
		.stream_mode = MWSTREAM_MODE_SUBDEV,
};

struct mw_dev_info dma_legacy_dev_info = {
		.stream_mode = MWSTREAM_MODE_LEGACY,
};

struct mw_dev_info nostream_legacy_dev_info = {
		.stream_mode = MWSTREAM_MODE_NONE,
};

static const struct of_device_id mathworks_ipcore_of_match[]  = {
	{ .compatible = "mathworks,mwipcore-v3.00", .data = &subdev_dev_info},
	{ .compatible = "mathworks,mwipcore-v2.00", .data = &dma_legacy_dev_info},
    { .compatible = "mathworks,mwipcore-axi4lite-v1.00", .data = &nostream_legacy_dev_info},
    {},
};

static void mathworks_ipcore_get_devname(struct mathworks_ip_info *mw_ip_info,char *devname){
	snprintf(devname,MATHWORKS_IP_DEVNAME_LEN, "%s", mw_ip_info->name);
}

static struct mathworks_ip_ops mathworks_ipcore_ops = {
	.get_devname = mathworks_ipcore_get_devname,
	.get_param = NULL,
	.fops = &mathworks_ip_common_fops,
};

/*
 * @brief mathworks_ipcore_of_probe
 */
static int mathworks_ipcore_of_probe(struct platform_device *op)
{
    int                         status = 0;
    struct device 				*dev  = &op->dev;
    struct mathworks_ipcore_dev		*mwdev;
    const struct of_device_id	*id;
    struct mathworks_ip_ops	*ops;


    mwdev = (struct mathworks_ipcore_dev*)devm_kzalloc(dev, sizeof(struct mathworks_ipcore_dev),GFP_KERNEL);
	if (!mwdev) {
		dev_err(dev, "Failed to allocate memory for device context\n");
		return -ENOMEM;
	}

	id = of_match_node(mathworks_ipcore_of_match, op->dev.of_node);
	if (!id || !id->data)
		return -ENODEV;
	mwdev->info = id->data;

	switch(mwdev->info->stream_mode){
		case MWSTREAM_MODE_LEGACY:
			ops = mw_stream_channel_get_ops();
			break;
		case MWSTREAM_MODE_SUBDEV:
		case MWSTREAM_MODE_NONE:
		default:
			ops = &mathworks_ipcore_ops;
			break;

	}

    mwdev->mw_ip_info = devm_mathworks_ip_of_init(op,THIS_MODULE,ops, true);
    if (IS_ERR(mwdev->mw_ip_info))
    	return PTR_ERR(mwdev->mw_ip_info);

    if (!mwdev->mw_ip_info->mem){
        dev_err(dev, "Failed to get resource for platform device\n");
        return -ENOENT;
    }

    status = of_property_read_u32(dev->of_node, "mathworks,rst-reg", &mwdev->rst_reg);
    if(status) {
    	/* Fall back to 0 if the property does not exist */
    	mwdev->rst_reg = 0;
    }

    mwdev->mw_ip_info->private = mwdev;

	status = devm_mathworks_ip_register(mwdev->mw_ip_info);
	if(status)
	{
		dev_err(dev, "MathWorks IP device registration failed: %d\n", status);
		return status;
	}
   
	switch(mwdev->info->stream_mode){
		case MWSTREAM_MODE_LEGACY:
			status = mw_stream_channels_probe(mwdev);
			if(status)
				return status;
			break;
		case MWSTREAM_MODE_SUBDEV:
			status = mw_stream_iio_channels_probe(mwdev);
			if(status)
				return status;
			break;
		case MWSTREAM_MODE_NONE:
		default:
			break;
	}

    return status;
}

/*
 * @brief mathworks_ipcore_of_remove
 */
static int mathworks_ipcore_of_remove(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver mathworks_ipcore_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = mathworks_ipcore_of_match,
    },
    .probe = mathworks_ipcore_of_probe,
    .remove = mathworks_ipcore_of_remove,
};

module_platform_driver(mathworks_ipcore_driver);

MODULE_DEVICE_TABLE(of, mathworks_ipcore_of_match);
MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks IP Core Driver");

/*DMA PARAMS */
