/*
 * MathWorks AXI DMA Driver
 *
 * Copyright 2014-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */


#include "mathworks_ipcore.h"
#include "mw_stream_channel.h"

#include <linux/version.h>

/*
 * @brief mathworks_ipcore_of_match
 */
static const struct of_device_id mathworks_ipcore_of_match[]  = {
    { .compatible = "mathworks,mwipcore-v2.00",},
    { .compatible = "mathworks,mwipcore-axi4lite-v1.00",},
    {},
};


/*
 * @brief mathworks_ipcore_of_probe
 */
static int mathworks_ipcore_of_probe(struct platform_device *op)
{
    int                         status = 0;
    struct device 				*dev  = &op->dev;
    struct mathworks_ipcore_dev		*mwdev;

    mwdev = (struct mathworks_ipcore_dev*)devm_kzalloc(dev, sizeof(struct mathworks_ipcore_dev),GFP_KERNEL);
	if (!mwdev) {
		dev_err(dev, "Failed to allocate memory for device context\n");
		return -ENOMEM;
	}

    mwdev->mw_ip_info = devm_mathworks_ip_of_init(op,THIS_MODULE,&mwadma_ip_ops, true);
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
   
	status = mw_stream_channels_probe(mwdev);
	if(status)
		return status;

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
