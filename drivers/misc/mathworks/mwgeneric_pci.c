#include "mwgeneric.h"
#include <linux/pci.h>

#define ip_to_pdev(ip)  container_of(ip->dev, struct pci_dev, dev)


#define pci_get_ss_vid(pdev, val) pci_read_config_word(pdev,0x2C,val)
#define pci_get_ss_did(pdev, val) pci_read_config_word(pdev,0x2E,val)

#define DRIVER_NAME "mwgeneric_pci"

/****************************************
* Bus Specific Functions
****************************************/

void mwgeneric_pci_get_devname(struct ipcore_info *thisIpcore, char *devname){
	struct pci_dev *pdev = ip_to_pdev(thisIpcore);
	u16 ss_vid, ss_did;

	pci_get_ss_vid(pdev, &ss_vid);
	pci_get_ss_did(pdev, &ss_did);
	
	snprintf(devname,MWGENERIC_DEVNAME_LEN, "mwgeneric_%04X_%04X_dev", ss_vid, ss_did);
}

struct mw_generic_ops mw_pci_ops = {
	.get_devname = mwgeneric_pci_get_devname,
	.get_param = NULL,
};


static int mw_generic_pci_probe(struct pci_dev *pdev, 
				const struct pci_device_id *ident)
{
    int status = 0;
	struct ipcore_info *thisIpcore;

	thisIpcore = (struct ipcore_info*)devm_kzalloc(&pdev->dev, sizeof(*thisIpcore), GFP_KERNEL);
	if (!thisIpcore)
		return -ENOMEM;

    thisIpcore->dev = &pdev->dev;
    thisIpcore->name = DRIVER_NAME;
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
    	thisIpcore->nirq = pci_enable_msi_range(pdev, 1, thisIpcore->nirq);
		if(thisIpcore->nirq < 1) {
			dev_err(&pdev->dev, "Failed to allocate at least 1 MSI\n");
			return thisIpcore->nirq;
		}
		dev_info(&pdev->dev, "Allocated %d MSIs\n", thisIpcore->nirq);
    	thisIpcore->irq = pdev->irq;
    }

    status = devm_mwgeneric_register(thisIpcore);
	if(status)
	{
		dev_err(&pdev->dev, "mwgeneric device registration failed: %d\n", status);
		return status;
	}

    pci_set_master(pdev); /* enable DMA */

    return status;
}

static void mw_generic_pci_remove(struct pci_dev *pdev)
{
    struct ipcore_info *thisIpcore = dev_get_drvdata(&pdev->dev);
    dev_info(thisIpcore->dev, "%s : free and release memory\n", thisIpcore->name);
    pci_clear_master(pdev);
}

static struct pci_device_id mw_generic_ids[ ] = {
 { PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022) },
 { 0, },
};

MODULE_DEVICE_TABLE(pci, mw_generic_ids);

static struct pci_driver mw_generic_pci_driver = {
 .name = DRIVER_NAME,
 .id_table = mw_generic_ids,
 .probe = mw_generic_pci_probe,
 .remove = mw_generic_pci_remove,
};

module_pci_driver(mw_generic_pci_driver);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks Generic driver");
MODULE_ALIAS(DRIVER_NAME);
