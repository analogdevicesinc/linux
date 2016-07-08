/*
 * MathWorks IP Generic OF Driver
 *
 * Copyright 2013-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <linux/mathworks/mathworks_ip.h>

#define ip_to_pdev(x)  (container_of(x->dev, struct platform_device, dev))

#define DRIVER_NAME "mathworks_generic_of"

static void mwgen_of_unlink_i2c_device(struct mathworks_ip_info *thisIpcore){
	sysfs_remove_link(&thisIpcore->char_device->kobj, "i2c_device");
}

static void mwgen_of_unlink_i2c_adapter(struct mathworks_ip_info *thisIpcore){
	sysfs_remove_link(&thisIpcore->char_device->kobj, "i2c_adapter");
}

static int mathworks_generic_of_i2c_init(struct mathworks_ip_info *thisIpcore){
	struct device_node *nodePointer = thisIpcore->dev->of_node;
	struct device_node *slave_node;
	int status;

	slave_node = of_parse_phandle(nodePointer, "i2c-controller", 0);
	if (slave_node) {
		status = devm_add_action_helper(thisIpcore->dev, (devm_action_fn)of_node_put, slave_node);
		if(status)
			return status;

		dev_info(thisIpcore->dev, "creating i2c link\n");

		thisIpcore->i2c = of_find_i2c_device_by_node(slave_node);
		if(thisIpcore->i2c == NULL){
			dev_err(thisIpcore->dev, "could not find i2c device\n");
			return -ENODEV;
		}
		status = devm_add_action_helper(thisIpcore->dev, (devm_action_fn)put_device, &thisIpcore->i2c->dev);
		if(status)
			return status;

		dev_info(thisIpcore->dev, "Adding link to %s[%s]\n", thisIpcore->i2c->adapter->name, thisIpcore->i2c->name);

		/* add a link to the i2c device */
		status = sysfs_create_link(&thisIpcore->char_device->kobj, &thisIpcore->i2c->dev.kobj, "i2c_device");
		if (status)
			return status;
		status = devm_add_action_helper(thisIpcore->dev, (devm_action_fn)mwgen_of_unlink_i2c_device, thisIpcore);
		if(status)
			return status;

		/* add a link to the i2c bus */
		status = sysfs_create_link(&thisIpcore->char_device->kobj, &thisIpcore->i2c->adapter->dev.kobj, "i2c_adapter");
		if (status)
			return status;
		status = devm_add_action_helper(thisIpcore->dev, (devm_action_fn)mwgen_of_unlink_i2c_adapter, thisIpcore);
		if(status)
			return status;

	} else {
		thisIpcore->i2c = NULL;
	}

	return 0;
}

static int	mathworks_generic_of_get_param(struct mathworks_ip_info *thisIpcore, void *arg)
{
	struct mathworks_ip_param_info pinfo;
	const void *paramData;
	int len;
	
	if( copy_from_user(&pinfo, (struct mathworks_ip_param_info *)arg, sizeof(struct mathworks_ip_param_info)) ) {
		return -EACCES;
	}
	
	paramData = of_get_property(thisIpcore->dev->of_node,pinfo.name, &len);
	pinfo.size = len;
	
	/* Copy the struct back to user space */
	if( copy_to_user((struct mathworks_ip_param_info*)arg, &pinfo, sizeof(struct mathworks_ip_param_info)) ) {
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

static void mathworks_generic_of_get_devname(struct mathworks_ip_info *thisIpcore,char *devname){
	const char *of_devname = of_get_property(thisIpcore->dev->of_node,"mwgen,devname", NULL);

	snprintf(devname,MATHWORKS_IP_DEVNAME_LEN, "%s", of_devname);
}

struct mathworks_ip_ops mw_of_ops = {
	.get_devname = mathworks_generic_of_get_devname,
	.get_param = mathworks_generic_of_get_param,
	.fops = &mathworks_ip_common_fops,
};

static const struct of_device_id mathworks_generic_of_match[] = {
    { .compatible = "mathworks,mwgeneric-v1.00",},
	{ .compatible = "mathworks,mathworks_ip-v1.00",},
    {},

};

MODULE_DEVICE_TABLE(of, mathworks_generic_of_match);


static int mathworks_generic_of_probe(struct platform_device *pdev)
{
    int status = 0;
	struct mathworks_ip_info *thisIpcore;


	thisIpcore = devm_mathworks_ip_of_init(pdev, THIS_MODULE, &mw_of_ops, false);
	if (IS_ERR(thisIpcore))
		return PTR_ERR(thisIpcore);

    status = devm_mathworks_ip_register(thisIpcore);
  	if(status)
  	{
  		dev_err(&pdev->dev, "mwgeneric device registration failed: %d\n", status);
  		return status;
  	}

#if defined(CONFIG_I2C)
	status = mathworks_generic_of_i2c_init(thisIpcore);
  	if (status){
  		dev_err(&pdev->dev, "Failed to link I2C nodes: %d\n", status);
  		return status;
  	}
#endif

    return 0;
}


static int mathworks_generic_of_remove(struct platform_device *pdev)
{
	struct mathworks_ip_info *thisIpcore = dev_get_drvdata(&pdev->dev);

    dev_info(thisIpcore->dev, "free and release memory\n");
	
    return 0;
}



static struct platform_driver mathworks_ip_driver = {
    .driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE, 
		.of_match_table = mathworks_generic_of_match,
		},
    .probe = mathworks_generic_of_probe,
    .remove = mathworks_generic_of_remove,
};

module_platform_driver(mathworks_ip_driver);


MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks Generic OF driver");
MODULE_ALIAS(DRIVER_NAME);
