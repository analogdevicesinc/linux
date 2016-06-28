#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fpga/fpga-overlay.h>
#include <linux/fpga/pcie-axi-bridge.h>

#define DRIVER_NAME	"xilinx_pcie_axi"
static const char xilinx_pab_compat[] = "xlnx,pcie-axi-bridge";
static char *firmware_file;

struct fpga_overlay_ops xilinx_pcie_axi_overlay_ops = {
	.update_fragment = pab_update_overlay_fragment,
};

static int xilinx_pcie_axi_probe(struct pci_dev *pdev,
				const struct pci_device_id *ident)
{
	const char *fw_file = firmware_file;
	int status = 0;
	const struct firmware *fdt_self;

	if (!fw_file)
		fw_file = (const char *)ident->driver_data;

	status = request_firmware(&fdt_self, fw_file, &pdev->dev);
	if (status)
		return status;

	status = pcie_axi_bridge_register(pdev, xilinx_pab_compat, fdt_self,
		&xilinx_pcie_axi_overlay_ops, NULL);
	release_firmware(fdt_self);

	return status;
}

static void xilinx_pcie_axi_remove(struct pci_dev *pdev)
{
	pcie_axi_bridge_unregister(pdev);
}

static struct pci_device_id xilinx_pcie_axi_ids[ ] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022),
	  .driver_data = (kernel_ulong_t) "xilinx-pcie-axi-bridge.dtbo" },
	{ PCI_DEVICE(PCI_VENDOR_ID_ANALOG_DEVICES, 0x9361),
	  .driver_data = (kernel_ulong_t) "adi-picozed-sdr2-pcie.dtbo"},
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, xilinx_pcie_axi_ids);

static struct pci_driver xilinx_pcie_axi_driver = {
	.name = DRIVER_NAME,
	.id_table = xilinx_pcie_axi_ids,
	.probe = xilinx_pcie_axi_probe,
	.remove = xilinx_pcie_axi_remove,
};

module_pci_driver(xilinx_pcie_axi_driver);

module_param(firmware_file, charp, S_IRUGO);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": PCIe to AXI Bridge");
MODULE_ALIAS(DRIVER_NAME);
