/*
 * MathWorks IP Generic PCI Driver
 *
 * Copyright 2015-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */
#include <linux/pci.h>
#include <linux/mathworks/mathworks_ip.h>

#define ip_to_pdev(ip)  container_of(ip->dev, struct pci_dev, dev)


#define pci_get_ss_vid(pdev, val) pci_read_config_word(pdev,0x2C,val)
#define pci_get_ss_did(pdev, val) pci_read_config_word(pdev,0x2E,val)

#define DRIVER_NAME "mathworks_generic_pci"

/****************************************
* Bus Specific Functions
****************************************/

void mathworks_generic_pci_get_devname(struct mathworks_ip_info *thisIpcore, char *devname){
	struct pci_dev *pdev = ip_to_pdev(thisIpcore);
	u16 ss_vid, ss_did;

	pci_get_ss_vid(pdev, &ss_vid);
	pci_get_ss_did(pdev, &ss_did);
	
	snprintf(devname,MATHWORKS_IP_DEVNAME_LEN, "mathworks_ip_%04X_%04X_dev", ss_vid, ss_did);
}

struct mathworks_ip_ops mw_pci_ops = {
	.get_devname = mathworks_generic_pci_get_devname,
	.get_param = NULL,
	.fops = &mathworks_ip_common_fops,
};


static int mathworks_generic_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *ident)
{
    int status = 0;
	struct mathworks_ip_info *thisIpcore;

	thisIpcore = (struct mathworks_ip_info*)devm_kzalloc(&pdev->dev, sizeof(*thisIpcore), GFP_KERNEL);
	if (!thisIpcore)
		return -ENOMEM;

    thisIpcore->dev = &pdev->dev;
    thisIpcore->name = DRIVER_NAME;
    thisIpcore->module = THIS_MODULE;
    dev_dbg(&pdev->dev,"IPCore name :%s\n", thisIpcore->name);
    thisIpcore->ops = &mw_pci_ops;

    status = pcim_enable_device(pdev); // Enable managed PCI resources
    if (status)
		return status;

	/* reserve resources */

	status = pci_request_regions(pdev, DRIVER_NAME);
	if (status) {
		dev_err(&pdev->dev, "Unable to request regions\n");
		return status;
	}


	thisIpcore->mem = &pdev->resource[0];
	dev_info(&pdev->dev, "Dev memory resource found at %08X %08X. \n", (unsigned int)thisIpcore->mem->start, (unsigned int)resource_size(thisIpcore->mem));

	// Setup IRQs

    thisIpcore->nirq = pci_msi_vec_count(pdev);
    if (thisIpcore->nirq > 0){
    	thisIpcore->nirq = pci_alloc_irq_vectors(pdev, 1, thisIpcore->nirq, PCI_IRQ_MSI);
		if(thisIpcore->nirq < 1) {
			dev_err(&pdev->dev, "Failed to allocate at least 1 MSI\n");
			return thisIpcore->nirq;
		}
		dev_info(&pdev->dev, "Allocated %d MSIs\n", thisIpcore->nirq);
    	thisIpcore->irq = pdev->irq;
    }

    status = devm_mathworks_ip_register(thisIpcore);
	if(status)
	{
		dev_err(&pdev->dev, "device registration failed: %d\n", status);
		pci_free_irq_vectors(pdev);
		return status;
	}

    pci_set_master(pdev); /* enable DMA */

    return status;
}

static void mathworks_generic_pci_remove(struct pci_dev *pdev)
{
    struct mathworks_ip_info *thisIpcore = dev_get_drvdata(&pdev->dev);
    dev_info(thisIpcore->dev, "%s : free and release memory\n", thisIpcore->name);
    pci_clear_master(pdev);
    pci_free_irq_vectors(pdev);
}

static struct pci_device_id mathworks_generic_pci_ids[ ] = {
 { PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022) },
 { 0, },
};

MODULE_DEVICE_TABLE(pci, mathworks_generic_pci_ids);

static struct pci_driver mathworks_generic_pci_driver = {
 .name = DRIVER_NAME,
 .id_table = mathworks_generic_pci_ids,
 .probe = mathworks_generic_pci_probe,
 .remove = mathworks_generic_pci_remove,
};

module_pci_driver(mathworks_generic_pci_driver);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks Generic PCI driver");
MODULE_ALIAS(DRIVER_NAME);
