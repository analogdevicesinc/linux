#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/of_fdt.h>
#include <linux/of_fdt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/idr.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/fpga/pcie-axi-bridge.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>

#define DRIVER_NAME "pcie_axi_bridge"

#define PAB_NMSI_MIN 1
#define PAB_NMSI_MAX 32

#define __PAB_RNG_CELLS(na,pna,ns) (na+pna+ns)
#define __PAB_RNG_LENGTH(na,pna,ns) (sizeof(u32) * __PAB_RNG_CELLS(na,pna,ns))

static const char pab_name[] = "pcie-axi-bridge";
static char *firmware_file = "pcie-axi-bridge-root.dtb";

static int pab_class_init = 0;

static LIST_HEAD(pab_list);

struct bus_addr_info {
	int na;
	int ns;
	int pna;
};

static struct device * __pab_get_parent(struct device *child){

	struct device *dev;
	struct of_pcie_axi_bridge *info;

	/* Search up the device hierarchy for a PCIe AXI Bridge */
	list_for_each_entry(info, &pab_list, list){
		for (dev = child; dev; dev = dev->parent){
			if (dev == &info->pdev->dev){
				return &info->pdev->dev;
			}
		}
	}

	return NULL;
}


void* pab_alloc_coherent(struct device *dev, size_t size,
			dma_addr_t *dma_handle, gfp_t gfp,
			struct dma_attrs *attrs){

	void *mem;

	mem = dma_alloc_attrs(__pab_get_parent(dev), size,dma_handle, gfp, attrs);

#ifdef CONFIG_X86
	/* x86 allocates WB memory, which does not work well with the DMA transactions
	 * originating from the AXI bus. Mark the memory as UC to work around this
	 */
	if(mem) {
		set_memory_uc((unsigned long)mem, PAGE_ALIGN(size) >> PAGE_SHIFT);
	}
#endif

	return mem;
}

static void pab_free_coherent(struct device *dev, size_t size,
			  void *virt_addr, dma_addr_t dma_addr,
			  struct dma_attrs *attrs) {

#ifdef CONFIG_X86
	set_memory_wb((unsigned long)virt_addr, PAGE_ALIGN(size) >> PAGE_SHIFT);
#endif
	dma_free_attrs(__pab_get_parent(dev), size, virt_addr, dma_addr,attrs);
}

int pab_mmap(struct device *dev, struct vm_area_struct *vma,
			  void *cpu_addr, dma_addr_t dma_addr, size_t size, struct dma_attrs *attrs){

	return dma_mmap_attrs(__pab_get_parent(dev), vma, cpu_addr, dma_addr,  size, attrs);
}

dma_addr_t pab_map_page(struct device *dev, struct page *page,
		       unsigned long offset, size_t size,
		       enum dma_data_direction dir,
		       struct dma_attrs *attrs){
	struct device *pab_dev = __pab_get_parent(dev);
	struct dma_map_ops *ops = get_dma_ops(pab_dev);

	return ops->map_page(pab_dev, page, offset, size, dir, attrs);
}

void pab_unmap_page(struct device *dev, dma_addr_t dma_handle,
		   size_t size, enum dma_data_direction dir,
		   struct dma_attrs *attrs){
	struct device *pab_dev = __pab_get_parent(dev);
	struct dma_map_ops *ops = get_dma_ops(pab_dev);

	ops->unmap_page(pab_dev, dma_handle, size, dir, attrs);
}

int pab_map_sg(struct device *dev, struct scatterlist *sg,
	      int nents, enum dma_data_direction dir,
	      struct dma_attrs *attrs) {
	struct device *pab_dev = __pab_get_parent(dev);
	struct dma_map_ops *ops = get_dma_ops(pab_dev);

	return ops->map_sg(pab_dev, sg, nents, dir, attrs);
}

void pab_unmap_sg(struct device *dev,
		 struct scatterlist *sg, int nents,
		 enum dma_data_direction dir,
		 struct dma_attrs *attrs){

	struct device *pab_dev = __pab_get_parent(dev);
	struct dma_map_ops *ops = get_dma_ops(pab_dev);

	ops->unmap_sg(pab_dev, sg, nents, dir, attrs);
}

int pab_dma_supported(struct device *dev, u64 mask) {
	struct device *pab_dev = __pab_get_parent(dev);
	struct dma_map_ops *ops = get_dma_ops(pab_dev);

	return ops->dma_supported(pab_dev, mask);
}

static struct dma_map_ops pab_dma_ops = {
	.alloc = pab_alloc_coherent,
	.free = pab_free_coherent,
	.mmap = pab_mmap,
	.map_page = pab_map_page,
	.unmap_page = pab_unmap_page,
	.map_sg = pab_map_sg,
	.unmap_sg = pab_unmap_sg,
	.dma_supported = pab_dma_supported,
};

/**
 *	pab_setup_dev_dma - Setup DMA allocation Parameters
 *
 */
static int pab_setup_dev_dma(struct device *dev) {
	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	dev->dma_mask = &dev->coherent_dma_mask;

	return 0;
}
/**
 *	pab_notifier_call - Callback for registering a platform device
 *
 */
static int pab_notifier_call(struct notifier_block *nb,
			unsigned long action, void *data) {

	struct device *dev = data;
	struct device *pab_dev;

	if (action == BUS_NOTIFY_ADD_DEVICE){
		pab_dev = __pab_get_parent(dev);
		if(pab_dev){
			dev->archdata.dma_ops = &pab_dma_ops;
			return 0;
		}
	}

	return 0;
}

static struct notifier_block pab_notifier = {
	.notifier_call = pab_notifier_call,
};

#ifdef USE_CHAINED_IRQ
static void pab_msi_handler(struct irq_desc *desc)
{

	struct of_pcie_axi_bridge *info = irq_desc_get_handler_data(desc);
	struct irq_data *data = &desc->irq_data;
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	int irq, hw_irq;

	chained_irq_enter(irqchip, desc);

	hw_irq = data->irq - info->irq_base;
	irq = irq_find_mapping(info->irq_domain, hw_irq);
	generic_handle_irq(irq);

	chained_irq_exit(irqchip, desc);

}
#else

irqreturn_t pab_msi_handler(int irq, void *opaque)
{
	struct of_pcie_axi_bridge *info = opaque;
	int hw_irq;

	hw_irq = irq - info->irq_base;
	irq = irq_find_mapping(info->irq_domain, hw_irq);
	generic_handle_irq(irq);
	return IRQ_HANDLED;
}
#endif

static int pab_irq_map(struct irq_domain *domain, unsigned int irq, irq_hw_number_t hwirq){

	struct of_pcie_axi_bridge *info = domain->host_data;

	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_edge_irq);
	irq_set_chip_data(irq, info);



	return 0;
}

static struct irq_domain_ops pab_irq_domain_ops = {
	.xlate = irq_domain_xlate_onetwocell,
	.map = pab_irq_map,
};


/**
 *	pab_irq_domain_remove - Remove MSI / IRQ Domain
 */
static void __pab_irq_domain_cleanup(void *opaque){
	struct of_pcie_axi_bridge *info = opaque;
	int msi_idx;

	for (msi_idx = 0; msi_idx < info->nvec; msi_idx++){
#ifdef USE_CHAINED_IRQ
		irq_set_chained_handler(info->pdev->irq+msi_idx, NULL);
		irq_set_handler_data(info->pdev->irq+msi_idx, NULL);
#endif
		irq_dispose_mapping(
			irq_find_mapping(info->irq_domain, msi_idx));
	}
}

/**
 *	pab_irq_domain_remove - Remove MSI / IRQ Domain
 */
static void __pab_irq_domain_remove(void *opaque){
	struct irq_domain *domain = opaque;
	irq_domain_remove(domain);

}

/**
 *	pab_irq_domain_init - Enable MSI / IRQ domain
 */
static int pab_irq_domain_init(struct of_pcie_axi_bridge *info){
	int msi_idx;
	int status, irq;
	struct pci_dev *pdev = info->pdev;

	if (!devres_open_group(&pdev->dev, pab_irq_domain_init, GFP_KERNEL)){
		return -ENOMEM;
	}

	info->nvec = pci_enable_msi_range(pdev, PAB_NMSI_MIN, PAB_NMSI_MAX);
	if(info->nvec < 0) {
		dev_err(&pdev->dev, "Failed to allocate at least %d MSIs\n", PAB_NMSI_MIN);
		status = info->nvec;
		goto out;
	}
	dev_info(&pdev->dev, "Allocated %d MSIs[%d]\n", info->nvec, pdev->irq);

	info->irq_base = pdev->irq;
	info->irq_domain = irq_domain_add_linear(info->of_self, info->nvec, &pab_irq_domain_ops,
							info);
	if(!info->irq_domain) {
		dev_err(&pdev->dev, "Failed to allocate IRQ domain\n");
		status = -ENODEV;
		goto out;
	}
	status = devm_add_action(&pdev->dev, __pab_irq_domain_remove, info->irq_domain);
	if(status)
		goto out;

	for(msi_idx = 0; msi_idx < info->nvec; msi_idx++){
		irq = irq_create_mapping(info->irq_domain, msi_idx);
#ifdef USE_CHAINED_IRQ
		irq_set_chained_handler_and_data(pdev->irq+msi_idx, pab_msi_handler, info);
#else
		status = devm_request_irq(&pdev->dev, pdev->irq+msi_idx, pab_msi_handler,
					  IRQF_NO_THREAD, info->of_self->name, info);
		if(status)
			goto out;
#endif
	}
	status = devm_add_action(&pdev->dev, __pab_irq_domain_cleanup, info);
	if(status)
		goto out;

	devres_close_group(&pdev->dev, pab_irq_domain_init);
	return 0;

out:
	devres_release_group(&pdev->dev, pab_irq_domain_init);
	return status;

}

/* Helper to write a big number; size is in cells (not bytes) */
static __be32 *__pab_of_write_number(u64 val, __be32 *cell, int size)
{
	int sz = size;
	while(size--){
		cell[size] = cpu_to_be32(val & 0xFFFFFFFF);
		val >>= 32;
	}
	return (cell+sz);
}

static int __pab_of_n_cells(struct device_node *np, const char *type)
{
	const __be32 *ip;

	do {
		ip = of_get_property(np, type, NULL);
		if (ip)
			return be32_to_cpup(ip);
		np = np->parent;
	} while (np->parent);
	/* No #address-cells property for the root node */
	return OF_ROOT_NODE_ADDR_CELLS_DEFAULT;
}

#define __pab_of_n_cells_addr(np) __pab_of_n_cells(np, "#address-cells")
#define __pab_of_n_cells_size(np) __pab_of_n_cells(np, "#size-cells")

static inline size_t __pab_get_range_size(struct bus_addr_info *addr_info){
	return __PAB_RNG_LENGTH(addr_info->na,addr_info->pna, addr_info->ns);
}

/**
 *	__pab_get_addr_info - Extract info about a devicetree address
 *
 */
static void __pab_get_addr_info(struct device_node *np, struct bus_addr_info *addr_info)
{
	struct device_node *parent;

	addr_info->na = __pab_of_n_cells_addr(np);
	addr_info->ns = __pab_of_n_cells_size(np);

	parent = of_get_parent(np);
	if (parent == NULL){
		addr_info->pna = 0;
		return;
	}

	addr_info->pna = __pab_of_n_cells_addr(parent);
	of_node_put(parent);
}

/**
 *	pab_update_overlay_fragment - Common function to update the overlay fragment
 *
 *	Will replace the target node with the of_node of the overlay device
 */
int pab_update_overlay_fragment (struct fpga_overlay_dev *overlay_dev, struct device_node *fragment){
	struct property *prop;
	struct device_node *np = overlay_dev->dev.of_node;

	if (np){
		prop = of_find_property(fragment, "target", NULL);
		if (!prop) {
			prop = of_find_property(fragment, "target-path", NULL);
			if (!prop){
				dev_err(&overlay_dev->dev, "%s:Overlay contains no target property\n",__func__);
				return -ENODEV;
			}
		}

		prop->value = devm_kstrdup(&overlay_dev->dev,np->full_name, GFP_KERNEL);
		if (!prop->value)
			return -ENOMEM;

		prop->name = devm_kstrdup(&overlay_dev->dev,"target-path", GFP_KERNEL);
		if (!prop->name)
			return -ENOMEM;

		prop->length = strlen(np->full_name)+1;
	}
	return 0;

}

EXPORT_SYMBOL_GPL(pab_update_overlay_fragment);

/**
 *	pab_update_name - Update the device_node name of the PCIe Bridge
 *						based on the PCI bus address
 */
static int pab_update_name(struct pci_dev *pdev, struct device_node *np)
{
	char *str;
	size_t len;
	uint8_t bus = pdev->bus->number;
	uint8_t devfn = pdev->devfn;
	struct property *prop;


	len = strlen(pab_name) + strlen("-00.00")+1;
	str = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!str)
		return -ENOMEM;
	snprintf(str, len, "%s-%02X.%02X", pab_name, bus, devfn);
	np->name = str;

	prop = of_find_property(np, "name", NULL);
	if (!prop)
	{
		prop = devm_kzalloc(&pdev->dev, sizeof(struct property), GFP_KERNEL);
		if (!prop)
			return -ENOMEM;
		prop->name = devm_kstrdup(&pdev->dev, "name", GFP_KERNEL);
		if (!prop->name)
			return -ENOMEM;
	}
	prop->value = str;
	prop->length = len;

	len = strlen(np->parent->full_name) + len+1;
	str = devm_kzalloc(&pdev->dev ,len, GFP_KERNEL);
	if (!str)
		return -ENOMEM;
	snprintf(str, len, "%s/%s", np->parent->full_name, np->name);
	np->full_name = str;

	return 0;
}

/**
 *	pab_create_range - Create the ranges property based on the PCI BARs
 */
static int pab_create_range(struct pci_dev *pdev, struct device_node *np)
{
	struct property *prop;
	struct bus_addr_info addr_info;
	__be32 *rng;
	int bar, barcnt;

	prop = of_find_property(np, "ranges", NULL);
	if (!prop)
		return -ENODEV;

	__pab_get_addr_info(np, &addr_info);

	/* Allocate enough memory for all of the standard BARs */
	rng = devm_kzalloc(&pdev->dev, (PCI_STD_RESOURCE_END+1) * __pab_get_range_size(&addr_info), GFP_KERNEL);
	if (!rng)
		return -ENOMEM;

	prop->value = rng;

	/* Load each of the bars into the range table */
	barcnt = 0;
	for (bar = 0; bar < PCI_STD_RESOURCE_END; bar++){
		if (resource_size(&pdev->resource[bar]) > 0 && resource_type(&pdev->resource[bar]) == IORESOURCE_MEM){
			/* Child address */
			rng = __pab_of_write_number(bar, rng, addr_info.na);
			/* Parent Address */
			rng = __pab_of_write_number(pdev->resource[bar].start, rng, addr_info.pna);
			/* Size */
			rng = __pab_of_write_number(resource_size(&pdev->resource[bar]), rng, addr_info.ns);
			barcnt++;
		}
	}

	prop->length = barcnt * __pab_get_range_size(&addr_info);

	return 0;
}

static void __pab_free_overlay(void *opaque)
{
	struct device_node *overlay = opaque;
	kfree(overlay);
}

/**
 *	pab_fdt_load - load a devicetree overlay from a firmware blob
 */
static struct device_node *pab_fdt_load(const struct firmware *fdt){
	void *dt_blob;
	struct device_node *np=NULL;

	if (!fdt->size) {
		return ERR_PTR(-ENODATA);
	}

	/* creating copy */
	dt_blob = kmemdup(fdt->data, fdt->size, GFP_KERNEL);
	if (!dt_blob) {
		np = ERR_PTR(-ENOMEM);
		goto out;
	}

	of_fdt_unflatten_tree(dt_blob, &np);
	if (!np) {
		np = ERR_PTR(-ENODATA);
		goto out;
	}

	of_node_set_flag(np, OF_DETACHED);

out:
	kfree(dt_blob);
	return np;
}

/**
 *	__pab_rmself - Remove the bridge from PCIe to the OF domain
 */
static void __pab_rmself(void *opaque)
{
	struct of_pcie_axi_bridge *info = opaque;
	struct pci_dev *pdev = info->pdev;
	int ret;

	pdev->dev.of_node = NULL;
	info->of_added_self = 0;
	ret = of_overlay_destroy(info->pci_overlay_id);
	if(ret){
		dev_err(&pdev->dev,"Failed to remove overlay\n");
	}
}

static void __of_put_node(void *opaque){
	struct device_node *np = opaque;
	of_node_put(np);
}

/**
 *	pab_addself - add PCIe bridge to the device tree
 */
static int pab_addself(struct of_pcie_axi_bridge *info, const struct firmware *fdt_self){
	struct pci_dev *pdev = info->pdev;
	int ret;
	struct device_node *overlay, *bridge_node;

	if(pdev->dev.of_node) {
		info->of_self = pdev->dev.of_node;
		info->of_added_self = 0;
		return 0;
	}

	if (!devres_open_group(&pdev->dev, pab_addself, GFP_KERNEL)){
		return -ENOMEM;
	}

	/* Load the FDT */
	overlay = pab_fdt_load(fdt_self);
	if (IS_ERR(overlay)){
		ret = PTR_ERR(overlay);
		goto out;
	}
	ret = devm_add_action(&pdev->dev, __pab_free_overlay, overlay);
	if(ret)
		goto out;

	/* Update the bridge node based on the PCI device */
	bridge_node = of_find_compatible_node(overlay, NULL, info->compat);
	ret = devm_add_action(&pdev->dev, __of_put_node, bridge_node);
	if(ret)
		goto out;

	/* Update the name */
	ret = pab_update_name(pdev, bridge_node);
	if (ret) {
		dev_err(&pdev->dev,"%s: Error updating the node name\n", __func__);
		goto out;
	}

	/* Create the range mappings */
	ret = pab_create_range(pdev, bridge_node);
	if (ret) {
		dev_err(&pdev->dev,"%s: Error creating the ranges\n", __func__);
		goto out;
	}

	ret = of_resolve_phandles(overlay);
	if (ret) {
		dev_err(&pdev->dev,"%s: Failed to resolve phandles: %d\n", __func__, ret);
		goto out;
	}


	info->pci_overlay_id = of_overlay_create(overlay);
	if (info->pci_overlay_id < 0){
		dev_err(&pdev->dev,"%s: Creating overlay failed: %d\n", __func__, ret);
		return -ENODEV;
	} else {
		dev_info(&pdev->dev,"PCIe to AXI Bridge added\n");
	}
	ret = devm_add_action(&pdev->dev, __pab_rmself, info);
	if(ret)
		goto out;


	/* Find and store the node*/
	info->of_self = of_find_node_by_name(NULL,bridge_node->name);
	if(!info->of_self){
		ret = -ENODEV;
		dev_err(&pdev->dev, "Could not find node: %s\n", bridge_node->name);
		goto out;
	}
	ret = devm_add_action(&pdev->dev, __of_put_node, info->of_self);
	if(ret)
		goto out;

	info->of_added_self = 1;
	pdev->dev.of_node = info->of_self;

	devres_close_group(&pdev->dev, pab_addself);
	return 0;

	out:
		devres_release_group(&pdev->dev, pab_addself);
		return ret;
}

/**
 *	pab_of_init - initialize devicetree for x86 systems
 */
static int pab_of_init(struct pci_dev *pdev)
{
	/*
	 * __dtb_pcie_axi_bridge_root_begin[] and __dtb_pcie_axi_bridge_root_end[] are magically
	 * created by cmd_dt_S_dtb in scripts/Makefile.lib
	 */
	struct device_node *pab_of_root;
	const struct firmware *fdt_root;
	int rc = 0;

	if (of_root)
		return 0;

	/* Load the FDT */
	rc = request_firmware(&fdt_root, firmware_file, &pdev->dev);
	if (rc)
		return rc;

	pab_of_root = pab_fdt_load(fdt_root);
	release_firmware(fdt_root);

	if (IS_ERR(pab_of_root)){
		return PTR_ERR(pab_of_root);
	}

	rc = of_resolve_phandles(pab_of_root);
	if (rc) {
		dev_err(&pdev->dev,"%s: Failed to resolve phandles (rc=%i)\n", __func__, rc);
		return -EINVAL;
	}


	rc = of_create_root(pab_of_root);
	if (rc == -EEXIST){
		/* Another node created the root */
		rc = 0;
	}

	return rc;

}

/**
 *	of_pcie_axi_bridge - Create the bridge from PCIe to the OF domain
 */
static struct of_pcie_axi_bridge *pab_of_create(struct pci_dev *pdev, const char *compat, const struct firmware *fdt_self)
{
	int ret;
	struct of_pcie_axi_bridge *info;

	ret = pab_of_init(pdev);
	if (ret) {
		dev_err(&pdev->dev,"Failed to create OF Root\n");
		return ERR_PTR(ret);
	}

	info=devm_kzalloc(&pdev->dev, sizeof(struct of_pcie_axi_bridge), GFP_KERNEL);
	if(!info){
		return ERR_PTR(-ENOMEM);
	}
	info->pdev = pdev;
	info->compat = compat;

	ret = pab_addself(info, fdt_self);
	if (ret) {
		dev_err(&pdev->dev,"Failed to update DT\n");
		return ERR_PTR(ret);
	}

	ret = pab_irq_domain_init(info);
	if (ret) {
		dev_err(&pdev->dev,"Failed allocate MSI\n");
		return ERR_PTR(ret);
	}

	dev_info(&pdev->dev,"Created PCIe AXI Bridge Device\n");

	return info;
}

/**
 *	pab_of_remove - Remove the bridge from PCIe to the OF domain
 */
static void pab_of_remove(struct of_pcie_axi_bridge *info)
{
	struct pci_dev *pdev = info->pdev;

	devres_release_group(&pdev->dev, pab_irq_domain_init);

	if(info->of_added_self){
		devres_release_group(&pdev->dev, pab_addself);
	}
}

/**
 *	pcie_axi_bridge_register - Register a PCIe device as an AXI Bridge
 */
int pcie_axi_bridge_register(struct pci_dev *pdev, const char *compat, const struct firmware *fdt_self, struct fpga_overlay_ops *overlay_ops, void *priv)
{
	struct of_pcie_axi_bridge *info;

	int ret;
	
	if(!pab_class_init)
		return -EPROBE_DEFER;

	if (dev_get_drvdata(&pdev->dev) != NULL){
		return -EALREADY;
	}

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	pab_setup_dev_dma(&pdev->dev);

	pci_set_master(pdev); /* enable DMA */

	info = pab_of_create(pdev, compat, fdt_self);
	if (IS_ERR(info))
		return PTR_ERR(info);

	info->priv = priv;

	info->overlay_dev = fpga_overlay_dev_register(&pdev->dev, "pcie axi bridge", overlay_ops, info);
	if (IS_ERR(info->overlay_dev)) {
		return PTR_ERR(info->overlay_dev);
	}

	dev_set_drvdata(&pdev->dev, info);

	list_add_tail(&info->list, &pab_list);

	return 0;
}

EXPORT_SYMBOL_GPL(pcie_axi_bridge_register);

/**
 *	pcie_axi_bridge_unregister - Unregister a PCIe device as an AXI Bridge
 */
void pcie_axi_bridge_unregister(struct pci_dev *pdev)
{
	struct of_pcie_axi_bridge *info = dev_get_drvdata(&pdev->dev);
	
	list_del(&info->list);
	dev_set_drvdata(&pdev->dev, NULL);
	fpga_overlay_dev_unregister(info->overlay_dev);
	pab_of_remove(info);
    pci_clear_master(pdev); /* disable DMA */
}

EXPORT_SYMBOL_GPL(pcie_axi_bridge_unregister);

static int __init pcie_axie_bridge_init(void)
{
	int ret;

	ret = bus_register_notifier(&platform_bus_type, &pab_notifier);
	if (ret) {
		pr_err(DRIVER_NAME ": Failed to register notifier\n");
	}

	pab_class_init = 1;
	return ret;
}

static void __exit pcie_axie_bridge_exit(void)
{
	bus_unregister_notifier(&platform_bus_type, &pab_notifier);
	pab_class_init = 0;
}

module_init(pcie_axie_bridge_init);
module_exit(pcie_axie_bridge_exit);

module_param(firmware_file, charp, S_IRUGO);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": PCIe to AXI Bridge");
MODULE_ALIAS(DRIVER_NAME);

