/*
 * Copyright 2013 MathWorks, Inc.
 * 
 *
 */
/* Open firmware includes */
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "mwgeneric.h"

#define ip_to_pdev(x)  (container_of(x->dev, struct platform_device, dev))

#define DRIVER_NAME "mwgeneric_of"

static void mwgeneric_of_i2c_release(void *opaque){
	struct ipcore_info *thisIpcore = opaque;

	sysfs_remove_link(&thisIpcore->char_device->kobj, "i2c_adapter");
	sysfs_remove_link(&thisIpcore->char_device->kobj, "i2c_device");
}

static int mwgeneric_of_i2c_init(struct ipcore_info *thisIpcore){
	struct device_node *nodePointer = thisIpcore->dev->of_node;
	struct device_node *slave_node;
	int status;

	slave_node = of_parse_phandle(nodePointer, "i2c-controller", 0);
	if (slave_node) {
		dev_info(thisIpcore->dev, "%s : creating i2c link\n", nodePointer->name);
		thisIpcore->i2c = of_find_i2c_device_by_node(slave_node);
		if(thisIpcore->i2c == NULL){
			dev_info(thisIpcore->dev, "%s : could not find i2c device\n", nodePointer->name);
		} else {
			dev_info(thisIpcore->dev, "%s : Adding link to %s[%s]\n", nodePointer->name, thisIpcore->i2c->adapter->name, thisIpcore->i2c->name);

			/* add a link to the i2c device */
			status = sysfs_create_link(&thisIpcore->char_device->kobj, &thisIpcore->i2c->dev.kobj, "i2c_device");
			if (status)
				return status;

			/* add a link to the i2c bus */
			status = sysfs_create_link(&thisIpcore->char_device->kobj, &thisIpcore->i2c->adapter->dev.kobj, "i2c_adapter");
			if (status)
				goto out_unlink_device;

		}
		of_node_put(slave_node);

		/* Add the release logic */
		status = devm_add_action(thisIpcore->dev, mwgeneric_of_i2c_release, thisIpcore);
		if(status){
			mwgeneric_of_i2c_release(thisIpcore);
			return status;
		}
	} else {
		thisIpcore->i2c = NULL;
	}

	return 0;

out_unlink_device:
	sysfs_remove_link(&thisIpcore->char_device->kobj, "i2c_device");
	return status;
}

static int	mwgeneric_of_get_param(struct ipcore_info *thisIpcore, void *arg)
{
	struct mwgeneric_param_info pinfo;
	const void *paramData;
	int len;
	
	if( copy_from_user(&pinfo, (struct mwgeneric_param_info *)arg, sizeof(struct mwgeneric_param_info)) ) {
		return -EACCES;
	}
	
	paramData = of_get_property(thisIpcore->dev->of_node,pinfo.name, &len);
	pinfo.size = len;
	
	/* Copy the struct back to user space */
	if( copy_to_user((struct mwgeneric_param_info*)arg, &pinfo, sizeof(struct mwgeneric_param_info)) ) {
		return -EACCES;
	}
	
	/* Copy any data to the user buf */
	if (paramData) {
		if( copy_to_user((void *)pinfo.buf, paramData, pinfo.size) ){
			return -EACCES;
		}	
	} else {
		return -ENODEV;
	}
	
	return 0;
}

static void mwgeneric_of_get_devname(struct ipcore_info *thisIpcore,char *devname){
	const char *of_devname = of_get_property(thisIpcore->dev->of_node,"mwgen,devname", NULL);

	snprintf(devname,MWGENERIC_DEVNAME_LEN, "%s", of_devname);
}

struct mw_generic_ops mw_of_ops = {
	.get_devname = mwgeneric_of_get_devname,
	.get_param = mwgeneric_of_get_param,
};

static const struct of_device_id mwgeneric_of_match[] = {
    { .compatible = "mathworks,mwgeneric-v1.00",},
    {},

};

MODULE_DEVICE_TABLE(of, mwgeneric_of_match);


static int mwgeneric_of_probe(struct platform_device *pdev)
{
    int status = 0;
	struct ipcore_info *thisIpcore;
    struct device_node *nodePointer = pdev->dev.of_node;

    thisIpcore = (struct ipcore_info*)devm_kzalloc(&pdev->dev, sizeof(*thisIpcore), GFP_KERNEL);
    if (!thisIpcore)
    	return -ENOMEM;

    thisIpcore->dev = &pdev->dev;
    thisIpcore->name = nodePointer->name;
    dev_dbg(&pdev->dev,"IPCore name :%s\n", thisIpcore->name);
    thisIpcore->ops = &mw_of_ops;

    /* Check for IRQ first, we may have to defer */
    thisIpcore->irq = platform_get_irq(pdev, 0);
    if (thisIpcore->irq < 0) {
		switch (thisIpcore->irq){
			case -EPROBE_DEFER:
				dev_info(&pdev->dev, "Defering probe for IRQ resources\n");
				return -EPROBE_DEFER;
			case -ENXIO:
				status = 0;
				thisIpcore->irq = 0;
				break;
			default :
				return thisIpcore->irq;
		}
    }
    /* Support only linear IRQ ranges */
    if (thisIpcore->irq){
    	/* capture the number of irqs */
    	thisIpcore->nirq = 1;
		do {
			status = platform_get_irq(pdev, thisIpcore->nirq);
			if (status > 0){
				if (status == thisIpcore->irq + thisIpcore->nirq)
					thisIpcore->nirq++;
				else
					dev_warn(&pdev->dev, "Non-sequential IRQs are not supported\n");
			}
		} while(status > 0);
    }

	thisIpcore->mem = platform_get_resource(pdev, IORESOURCE_MEM,0);
	if(thisIpcore->mem)
	{
		dev_info(&pdev->dev, "Dev memory resource found at %08lX %08lX. \n", (unsigned long)thisIpcore->mem->start, (unsigned long)resource_size(thisIpcore->mem));

		thisIpcore->mem  = devm_request_mem_region(&pdev->dev, thisIpcore->mem->start, resource_size(thisIpcore->mem), pdev->name);

		if (!thisIpcore->mem)
		{
			dev_err(&pdev->dev, "Error while request_mem_region call\n");
			return -ENODEV;
		}
	}

  	status = mwgeneric_of_i2c_init(thisIpcore);
  	if (status){
  		dev_err(&pdev->dev, "Failed to link I2C nodes: %d\n", status);
  		return status;
  	}

    status = devm_mwgeneric_register(thisIpcore);
  	if(status)
  	{
  		dev_err(&pdev->dev, "mwgeneric device registration failed: %d\n", status);
  		return status;
  	}
	
    return 0;
}


static int mwgeneric_of_remove(struct platform_device *pdev)
{
	struct ipcore_info *thisIpcore = dev_get_drvdata(&pdev->dev);

    dev_info(thisIpcore->dev, "free and release memory\n");
	
    return 0;
}



static struct platform_driver mwgeneric_driver = {
    .driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE, 
		.of_match_table = mwgeneric_of_match,
		},
    .probe = mwgeneric_of_probe,
    .remove = mwgeneric_of_remove,
};

module_platform_driver(mwgeneric_driver);


MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks Generic driver");
MODULE_ALIAS(DRIVER_NAME);
