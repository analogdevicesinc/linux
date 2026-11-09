// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2021 Intel Corporation. All rights reserved. */
#include <linux/units.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/iopoll.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/pci-doe.h>
#include <cxl/pci.h>
#include <linux/aer.h>
#include <linux/string_choices.h>
#include <cxlpci.h>
#include <cxlmem.h>
#include <cxl.h>
#include "core.h"
#include "trace.h"

/**
 * DOC: cxl core pci
 *
 * Compute Express Link protocols are layered on top of PCIe. CXL core provides
 * a set of helpers for CXL interactions which occur via PCIe.
 */

static unsigned short media_ready_timeout = 60;
module_param(media_ready_timeout, ushort, 0644);
MODULE_PARM_DESC(media_ready_timeout, "seconds to wait for media ready");

static int pci_get_port_num(struct pci_dev *pdev)
{
	u32 lnkcap;
	int type;

	type = pci_pcie_type(pdev);
	if (type != PCI_EXP_TYPE_DOWNSTREAM && type != PCI_EXP_TYPE_ROOT_PORT)
		return -EINVAL;

	if (pci_read_config_dword(pdev, pci_pcie_cap(pdev) + PCI_EXP_LNKCAP,
				  &lnkcap))
		return -ENXIO;

	return FIELD_GET(PCI_EXP_LNKCAP_PN, lnkcap);
}

/**
 * devm_cxl_add_dport_by_dev - allocate a dport by dport device
 * @port: cxl_port that hosts the dport
 * @dport_dev: 'struct device' of the dport
 *
 * Returns the allocated dport on success or ERR_PTR() of -errno on error
 */
struct cxl_dport *devm_cxl_add_dport_by_dev(struct cxl_port *port,
					    struct device *dport_dev)
{
	struct cxl_register_map map;
	struct pci_dev *pdev;
	int port_num, rc;

	if (!dev_is_pci(dport_dev))
		return ERR_PTR(-EINVAL);

	pdev = to_pci_dev(dport_dev);
	port_num = pci_get_port_num(pdev);
	if (port_num < 0)
		return ERR_PTR(port_num);

	rc = cxl_find_regblock(pdev, CXL_REGLOC_RBI_COMPONENT, &map);
	if (rc)
		return ERR_PTR(rc);

	device_lock_assert(&port->dev);
	return devm_cxl_add_dport(port, dport_dev, port_num, map.resource);
}
EXPORT_SYMBOL_NS_GPL(devm_cxl_add_dport_by_dev, "CXL");

static int cxl_dvsec_mem_range_valid(struct cxl_dev_state *cxlds, int id)
{
	struct pci_dev *pdev = to_pci_dev(cxlds->dev);
	int d = cxlds->cxl_dvsec;
	bool valid = false;
	int rc, i;
	u32 temp;

	if (id > CXL_DVSEC_RANGE_MAX)
		return -EINVAL;

	/* Check MEM INFO VALID bit first, give up after 1s */
	i = 1;
	do {
		rc = pci_read_config_dword(pdev,
					   d + PCI_DVSEC_CXL_RANGE_SIZE_LOW(id),
					   &temp);
		if (rc)
			return pcibios_err_to_errno(rc);

		valid = FIELD_GET(PCI_DVSEC_CXL_MEM_INFO_VALID, temp);
		if (valid)
			break;
		msleep(1000);
	} while (i--);

	if (!valid) {
		dev_err(&pdev->dev,
			"Timeout awaiting memory range %d valid after 1s.\n",
			id);
		return -ETIMEDOUT;
	}

	return 0;
}

static int cxl_dvsec_mem_range_active(struct cxl_dev_state *cxlds, int id)
{
	struct pci_dev *pdev = to_pci_dev(cxlds->dev);
	int d = cxlds->cxl_dvsec;
	bool active = false;
	int rc, i;
	u32 temp;

	if (id > CXL_DVSEC_RANGE_MAX)
		return -EINVAL;

	/* Check MEM ACTIVE bit, up to 60s timeout by default */
	for (i = media_ready_timeout; i; i--) {
		rc = pci_read_config_dword(
			pdev, d + PCI_DVSEC_CXL_RANGE_SIZE_LOW(id), &temp);
		if (rc)
			return pcibios_err_to_errno(rc);

		active = FIELD_GET(PCI_DVSEC_CXL_MEM_ACTIVE, temp);
		if (active)
			break;
		msleep(1000);
	}

	if (!active) {
		dev_err(&pdev->dev,
			"timeout awaiting memory active after %d seconds\n",
			media_ready_timeout);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Wait up to @media_ready_timeout for the device to report memory
 * active.
 */
int cxl_await_media_ready(struct cxl_dev_state *cxlds)
{
	struct pci_dev *pdev = to_pci_dev(cxlds->dev);
	int d = cxlds->cxl_dvsec;
	int rc, i, hdm_count;
	u64 md_status;
	u16 cap;

	rc = pci_read_config_word(pdev,
				  d + PCI_DVSEC_CXL_CAP, &cap);
	if (rc)
		return pcibios_err_to_errno(rc);

	hdm_count = FIELD_GET(PCI_DVSEC_CXL_HDM_COUNT, cap);
	for (i = 0; i < hdm_count; i++) {
		rc = cxl_dvsec_mem_range_valid(cxlds, i);
		if (rc)
			return rc;
	}

	for (i = 0; i < hdm_count; i++) {
		rc = cxl_dvsec_mem_range_active(cxlds, i);
		if (rc)
			return rc;
	}

	md_status = readq(cxlds->regs.memdev + CXLMDEV_STATUS_OFFSET);
	if (!CXLMDEV_READY(md_status))
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(cxl_await_media_ready, "CXL");

static int cxl_set_mem_enable(struct cxl_dev_state *cxlds, u16 val)
{
	struct pci_dev *pdev = to_pci_dev(cxlds->dev);
	int d = cxlds->cxl_dvsec;
	u16 ctrl;
	int rc;

	rc = pci_read_config_word(pdev, d + PCI_DVSEC_CXL_CTRL, &ctrl);
	if (rc)
		return pcibios_err_to_errno(rc);

	if ((ctrl & PCI_DVSEC_CXL_MEM_ENABLE) == val)
		return 1;
	ctrl &= ~PCI_DVSEC_CXL_MEM_ENABLE;
	ctrl |= val;

	rc = pci_write_config_word(pdev, d + PCI_DVSEC_CXL_CTRL, ctrl);
	if (rc)
		return pcibios_err_to_errno(rc);

	return 0;
}

static void clear_mem_enable(void *cxlds)
{
	cxl_set_mem_enable(cxlds, 0);
}

static int devm_cxl_enable_mem(struct device *host, struct cxl_dev_state *cxlds)
{
	int rc;

	rc = cxl_set_mem_enable(cxlds, PCI_DVSEC_CXL_MEM_ENABLE);
	if (rc < 0)
		return rc;
	if (rc > 0)
		return 0;
	return devm_add_action_or_reset(host, clear_mem_enable, cxlds);
}

/* require dvsec ranges to be covered by a locked platform window */
static int dvsec_range_allowed(struct device *dev, const void *arg)
{
	const struct range *dev_range = arg;
	struct cxl_decoder *cxld;

	if (!is_root_decoder(dev))
		return 0;

	cxld = to_cxl_decoder(dev);

	if (!(cxld->flags & CXL_DECODER_F_RAM))
		return 0;

	return range_contains(&cxld->hpa_range, dev_range);
}

static void disable_hdm(void *_cxlhdm)
{
	u32 global_ctrl;
	struct cxl_hdm *cxlhdm = _cxlhdm;
	void __iomem *hdm = cxlhdm->regs.hdm_decoder;

	global_ctrl = readl(hdm + CXL_HDM_DECODER_CTRL_OFFSET);
	writel(global_ctrl & ~CXL_HDM_DECODER_ENABLE,
	       hdm + CXL_HDM_DECODER_CTRL_OFFSET);
}

static int devm_cxl_enable_hdm(struct device *host, struct cxl_hdm *cxlhdm)
{
	void __iomem *hdm = cxlhdm->regs.hdm_decoder;
	u32 global_ctrl;

	global_ctrl = readl(hdm + CXL_HDM_DECODER_CTRL_OFFSET);
	writel(global_ctrl | CXL_HDM_DECODER_ENABLE,
	       hdm + CXL_HDM_DECODER_CTRL_OFFSET);

	return devm_add_action_or_reset(host, disable_hdm, cxlhdm);
}

int cxl_dvsec_rr_decode(struct cxl_dev_state *cxlds,
			struct cxl_endpoint_dvsec_info *info)
{
	struct pci_dev *pdev = to_pci_dev(cxlds->dev);
	struct device *dev = cxlds->dev;
	int hdm_count, rc, i, ranges = 0;
	int d = cxlds->cxl_dvsec;
	u16 cap, ctrl;

	if (!d) {
		dev_dbg(dev, "No DVSEC Capability\n");
		return -ENXIO;
	}

	rc = pci_read_config_word(pdev, d + PCI_DVSEC_CXL_CAP, &cap);
	if (rc)
		return pcibios_err_to_errno(rc);

	if (!(cap & PCI_DVSEC_CXL_MEM_CAPABLE)) {
		dev_dbg(dev, "Not MEM Capable\n");
		return -ENXIO;
	}

	/*
	 * It is not allowed by spec for MEM.capable to be set and have 0 legacy
	 * HDM decoders (values > 2 are also undefined as of CXL 2.0). As this
	 * driver is for a spec defined class code which must be CXL.mem
	 * capable, there is no point in continuing to enable CXL.mem.
	 */
	hdm_count = FIELD_GET(PCI_DVSEC_CXL_HDM_COUNT, cap);
	if (!hdm_count || hdm_count > 2)
		return -EINVAL;

	/*
	 * The current DVSEC values are moot if the memory capability is
	 * disabled, and they will remain moot after the HDM Decoder
	 * capability is enabled.
	 */
	rc = pci_read_config_word(pdev, d + PCI_DVSEC_CXL_CTRL, &ctrl);
	if (rc)
		return pcibios_err_to_errno(rc);

	info->mem_enabled = FIELD_GET(PCI_DVSEC_CXL_MEM_ENABLE, ctrl);
	if (!info->mem_enabled)
		return 0;

	for (i = 0; i < hdm_count; i++) {
		u64 base, size;
		u32 temp;

		rc = cxl_dvsec_mem_range_valid(cxlds, i);
		if (rc)
			return rc;

		rc = pci_read_config_dword(
			pdev, d + PCI_DVSEC_CXL_RANGE_SIZE_HIGH(i), &temp);
		if (rc)
			return pcibios_err_to_errno(rc);

		size = (u64)temp << 32;

		rc = pci_read_config_dword(
			pdev, d + PCI_DVSEC_CXL_RANGE_SIZE_LOW(i), &temp);
		if (rc)
			return pcibios_err_to_errno(rc);

		size |= temp & PCI_DVSEC_CXL_MEM_SIZE_LOW;
		if (!size) {
			continue;
		}

		rc = pci_read_config_dword(
			pdev, d + PCI_DVSEC_CXL_RANGE_BASE_HIGH(i), &temp);
		if (rc)
			return pcibios_err_to_errno(rc);

		base = (u64)temp << 32;

		rc = pci_read_config_dword(
			pdev, d + PCI_DVSEC_CXL_RANGE_BASE_LOW(i), &temp);
		if (rc)
			return pcibios_err_to_errno(rc);

		base |= temp & PCI_DVSEC_CXL_MEM_BASE_LOW;

		info->dvsec_range[ranges++] = (struct range) {
			.start = base,
			.end = base + size - 1
		};
	}

	info->ranges = ranges;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(cxl_dvsec_rr_decode, "CXL");

/**
 * cxl_hdm_decode_init() - Setup HDM decoding for the endpoint
 * @cxlds: Device state
 * @cxlhdm: Mapped HDM decoder Capability
 * @info: Cached DVSEC range registers info
 *
 * Try to enable the endpoint's HDM Decoder Capability
 */
int cxl_hdm_decode_init(struct cxl_dev_state *cxlds, struct cxl_hdm *cxlhdm,
			struct cxl_endpoint_dvsec_info *info)
{
	void __iomem *hdm = cxlhdm->regs.hdm_decoder;
	struct cxl_port *port = cxlhdm->port;
	struct device *dev = cxlds->dev;
	struct cxl_port *root;
	int i, rc, allowed;
	u32 global_ctrl = 0;

	if (hdm)
		global_ctrl = readl(hdm + CXL_HDM_DECODER_CTRL_OFFSET);

	/*
	 * If the HDM Decoder Capability is already enabled then assume
	 * that some other agent like platform firmware set it up.
	 */
	if (global_ctrl & CXL_HDM_DECODER_ENABLE || (!hdm && info->mem_enabled))
		return devm_cxl_enable_mem(&port->dev, cxlds);

	/*
	 * If the HDM Decoder Capability does not exist and DVSEC was
	 * not setup, the DVSEC based emulation cannot be used.
	 */
	if (!hdm)
		return -ENODEV;

	/* The HDM Decoder Capability exists but is globally disabled. */

	/*
	 * If the DVSEC CXL Range registers are not enabled, just
	 * enable and use the HDM Decoder Capability registers.
	 */
	if (!info->mem_enabled) {
		rc = devm_cxl_enable_hdm(&port->dev, cxlhdm);
		if (rc)
			return rc;

		return devm_cxl_enable_mem(&port->dev, cxlds);
	}

	/*
	 * Per CXL 2.0 Section 8.1.3.8.3 and 8.1.3.8.4 DVSEC CXL Range 1 Base
	 * [High,Low] when HDM operation is enabled the range register values
	 * are ignored by the device, but the spec also recommends matching the
	 * DVSEC Range 1,2 to HDM Decoder Range 0,1. So, non-zero info->ranges
	 * are expected even though Linux does not require or maintain that
	 * match. Check if at least one DVSEC range is enabled and allowed by
	 * the platform. That is, the DVSEC range must be covered by a locked
	 * platform window (CFMWS). Fail otherwise as the endpoint's decoders
	 * cannot be used.
	 */

	root = to_cxl_port(port->dev.parent);
	while (!is_cxl_root(root) && is_cxl_port(root->dev.parent))
		root = to_cxl_port(root->dev.parent);
	if (!is_cxl_root(root)) {
		dev_err(dev, "Failed to acquire root port for HDM enable\n");
		return -ENODEV;
	}

	for (i = 0, allowed = 0; i < info->ranges; i++) {
		struct device *cxld_dev;

		cxld_dev = device_find_child(&root->dev, &info->dvsec_range[i],
					     dvsec_range_allowed);
		if (!cxld_dev) {
			dev_dbg(dev, "DVSEC Range%d denied by platform\n", i);
			continue;
		}
		dev_dbg(dev, "DVSEC Range%d allowed by platform\n", i);
		put_device(cxld_dev);
		allowed++;
	}

	if (!allowed) {
		dev_err(dev, "Range register decodes outside platform defined CXL ranges.\n");
		return -ENXIO;
	}

	return 0;
}
EXPORT_SYMBOL_NS_GPL(cxl_hdm_decode_init, "CXL");

#define CXL_DOE_TABLE_ACCESS_REQ_CODE		0x000000ff
#define   CXL_DOE_TABLE_ACCESS_REQ_CODE_READ	0
#define CXL_DOE_TABLE_ACCESS_TABLE_TYPE		0x0000ff00
#define   CXL_DOE_TABLE_ACCESS_TABLE_TYPE_CDATA	0
#define CXL_DOE_TABLE_ACCESS_ENTRY_HANDLE	0xffff0000
#define CXL_DOE_TABLE_ACCESS_LAST_ENTRY		0xffff
#define CXL_DOE_PROTOCOL_TABLE_ACCESS 2

#define CDAT_DOE_REQ(entry_handle) cpu_to_le32				\
	(FIELD_PREP(CXL_DOE_TABLE_ACCESS_REQ_CODE,			\
		    CXL_DOE_TABLE_ACCESS_REQ_CODE_READ) |		\
	 FIELD_PREP(CXL_DOE_TABLE_ACCESS_TABLE_TYPE,			\
		    CXL_DOE_TABLE_ACCESS_TABLE_TYPE_CDATA) |		\
	 FIELD_PREP(CXL_DOE_TABLE_ACCESS_ENTRY_HANDLE, (entry_handle)))

static int cxl_cdat_get_length(struct device *dev,
			       struct pci_doe_mb *doe_mb,
			       size_t *length)
{
	__le32 request = CDAT_DOE_REQ(0);
	__le32 response[2];
	int rc;

	rc = pci_doe(doe_mb, PCI_VENDOR_ID_CXL,
		     CXL_DOE_PROTOCOL_TABLE_ACCESS,
		     &request, sizeof(request),
		     &response, sizeof(response));
	if (rc < 0) {
		dev_err(dev, "DOE failed: %d", rc);
		return rc;
	}
	if (rc < sizeof(response))
		return -EIO;

	*length = le32_to_cpu(response[1]);
	dev_dbg(dev, "CDAT length %zu\n", *length);

	return 0;
}

static int cxl_cdat_read_table(struct device *dev,
			       struct pci_doe_mb *doe_mb,
			       struct cdat_doe_rsp *rsp, size_t *length)
{
	size_t received, remaining = *length;
	unsigned int entry_handle = 0;
	union cdat_data *data;
	__le32 saved_dw = 0;

	do {
		__le32 request = CDAT_DOE_REQ(entry_handle);
		int rc;

		rc = pci_doe(doe_mb, PCI_VENDOR_ID_CXL,
			     CXL_DOE_PROTOCOL_TABLE_ACCESS,
			     &request, sizeof(request),
			     rsp, sizeof(*rsp) + remaining);
		if (rc < 0) {
			dev_err(dev, "DOE failed: %d", rc);
			return rc;
		}

		if (rc < sizeof(*rsp))
			return -EIO;

		data = (union cdat_data *)rsp->data;
		received = rc - sizeof(*rsp);

		if (entry_handle == 0) {
			if (received != sizeof(data->header))
				return -EIO;
		} else {
			if (received < sizeof(data->entry) ||
			    received != le16_to_cpu(data->entry.length))
				return -EIO;
		}

		/* Get the CXL table access header entry handle */
		entry_handle = FIELD_GET(CXL_DOE_TABLE_ACCESS_ENTRY_HANDLE,
					 le32_to_cpu(rsp->doe_header));

		/*
		 * Table Access Response Header overwrote the last DW of
		 * previous entry, so restore that DW
		 */
		rsp->doe_header = saved_dw;
		remaining -= received;
		rsp = (void *)rsp + received;
		saved_dw = rsp->doe_header;
	} while (entry_handle != CXL_DOE_TABLE_ACCESS_LAST_ENTRY);

	/* Length in CDAT header may exceed concatenation of CDAT entries */
	*length -= remaining;

	return 0;
}

static unsigned char cdat_checksum(void *buf, size_t size)
{
	unsigned char sum, *data = buf;
	size_t i;

	for (sum = 0, i = 0; i < size; i++)
		sum += data[i];
	return sum;
}

/**
 * read_cdat_data - Read the CDAT data on this port
 * @port: Port to read data from
 *
 * This call will sleep waiting for responses from the DOE mailbox.
 */
void read_cdat_data(struct cxl_port *port)
{
	struct device *uport = port->uport_dev;
	struct device *dev = &port->dev;
	struct pci_doe_mb *doe_mb;
	struct pci_dev *pdev = NULL;
	struct cxl_memdev *cxlmd;
	struct cdat_doe_rsp *buf;
	size_t table_length, length;
	int rc;

	if (is_cxl_memdev(uport)) {
		struct device *host;

		cxlmd = to_cxl_memdev(uport);
		host = cxlmd->dev.parent;
		if (dev_is_pci(host))
			pdev = to_pci_dev(host);
	} else if (dev_is_pci(uport)) {
		pdev = to_pci_dev(uport);
	}

	if (!pdev)
		return;

	doe_mb = pci_find_doe_mailbox(pdev, PCI_VENDOR_ID_CXL,
				      CXL_DOE_PROTOCOL_TABLE_ACCESS);
	if (!doe_mb) {
		dev_dbg(dev, "No CDAT mailbox\n");
		return;
	}

	port->cdat_available = true;

	if (cxl_cdat_get_length(dev, doe_mb, &length)) {
		dev_dbg(dev, "No CDAT length\n");
		return;
	}

	/*
	 * The begin of the CDAT buffer needs space for additional 4
	 * bytes for the DOE header. Table data starts afterwards.
	 */
	buf = devm_kzalloc(dev, sizeof(*buf) + length, GFP_KERNEL);
	if (!buf)
		goto err;

	table_length = length;

	rc = cxl_cdat_read_table(dev, doe_mb, buf, &length);
	if (rc)
		goto err;

	if (table_length != length)
		dev_warn(dev, "Malformed CDAT table length (%zu:%zu), discarding trailing data\n",
			table_length, length);

	if (cdat_checksum(buf->data, length))
		goto err;

	port->cdat.table = buf->data;
	port->cdat.length = length;

	return;
err:
	/* Don't leave table data allocated on error */
	devm_kfree(dev, buf);
	dev_err(dev, "Failed to read/validate CDAT.\n");
}
EXPORT_SYMBOL_NS_GPL(read_cdat_data, "CXL");

static int cxl_flit_size(struct pci_dev *pdev)
{
	if (cxl_pci_flit_256(pdev))
		return 256;

	return 68;
}

/**
 * cxl_pci_get_latency - calculate the link latency for the PCIe link
 * @pdev: PCI device
 *
 * return: calculated latency or 0 for no latency
 *
 * CXL Memory Device SW Guide v1.0 2.11.4 Link latency calculation
 * Link latency = LinkPropagationLatency + FlitLatency + RetimerLatency
 * LinkProgationLatency is negligible, so 0 will be used
 * RetimerLatency is assumed to be negligible and 0 will be used
 * FlitLatency = FlitSize / LinkBandwidth
 * FlitSize is defined by spec. CXL rev3.0 4.2.1.
 * 68B flit is used up to 32GT/s. >32GT/s, 256B flit size is used.
 * The FlitLatency is converted to picoseconds.
 */
long cxl_pci_get_latency(struct pci_dev *pdev)
{
	long bw;

	bw = pcie_link_bandwidth_mbps(pdev);
	if (bw <= 0)
		return 0;
	bw /= BITS_PER_BYTE;

	return cxl_flit_size(pdev) * MEGA / bw;
}

static int __cxl_endpoint_decoder_reset_detected(struct device *dev, void *data)
{
	struct cxl_port *port = data;
	struct cxl_decoder *cxld;
	struct cxl_hdm *cxlhdm;
	void __iomem *hdm;
	u32 ctrl;

	if (!is_endpoint_decoder(dev))
		return 0;

	cxld = to_cxl_decoder(dev);
	if ((cxld->flags & CXL_DECODER_F_ENABLE) == 0)
		return 0;

	/*
	 * Decoders emulated from the DVSEC range registers have no commit
	 * callback and no HDM decoder registers to consult.
	 */
	if (!cxld->commit) {
		dev_warn(dev, "DVSEC emulated decode may have been cleared by reset\n");
		return 0;
	}

	cxlhdm = dev_get_drvdata(&port->dev);
	hdm = cxlhdm->regs.hdm_decoder;
	ctrl = readl(hdm + CXL_HDM_DECODER0_CTRL_OFFSET(cxld->id));

	return !FIELD_GET(CXL_HDM_DECODER0_CTRL_COMMITTED, ctrl);
}

bool cxl_endpoint_decoder_reset_detected(struct cxl_port *port)
{
	return device_for_each_child(&port->dev, port,
				     __cxl_endpoint_decoder_reset_detected);
}
EXPORT_SYMBOL_NS_GPL(cxl_endpoint_decoder_reset_detected, "CXL");

static int cxl_rcrb_get_comp_regs(struct pci_dev *pdev,
				  struct cxl_register_map *map,
				  struct cxl_dport *dport)
{
	resource_size_t component_reg_phys;

	*map = (struct cxl_register_map) {
		.host = &pdev->dev,
		.resource = CXL_RESOURCE_NONE,
	};

	component_reg_phys = cxl_rcd_component_reg_phys(&pdev->dev, dport);
	if (component_reg_phys == CXL_RESOURCE_NONE)
		return -ENXIO;

	map->resource = component_reg_phys;
	map->reg_type = CXL_REGLOC_RBI_COMPONENT;
	map->max_size = CXL_COMPONENT_REG_BLOCK_SIZE;

	return 0;
}

int cxl_pci_setup_regs(struct pci_dev *pdev, enum cxl_regloc_type type,
		       struct cxl_register_map *map)
{
	int rc;

	rc = cxl_find_regblock(pdev, type, map);

	/*
	 * If the Register Locator DVSEC does not exist, check if it
	 * is an RCH and try to extract the Component Registers from
	 * an RCRB.
	 */
	if (rc && type == CXL_REGLOC_RBI_COMPONENT && is_cxl_restricted(pdev)) {
		struct cxl_dport *dport;
		struct cxl_port *port __free(put_cxl_port) =
			cxl_pci_find_port(pdev);
		if (!port)
			return -EPROBE_DEFER;

		guard(device)(cxl_port_dport_host(port));

		dport = cxl_pci_find_dport(pdev, port);
		if (!dport)
			return -EPROBE_DEFER;

		rc = cxl_rcrb_get_comp_regs(pdev, map, dport);
		if (rc)
			return rc;

		rc = cxl_dport_map_rcd_linkcap(pdev, dport);
		if (rc)
			return rc;

	} else if (rc) {
		return rc;
	}

	return cxl_setup_regs(map);
}
EXPORT_SYMBOL_NS_GPL(cxl_pci_setup_regs, "CXL");

int cxl_pci_get_bandwidth(struct pci_dev *pdev, struct access_coordinate *c)
{
	int bw;

	bw = pcie_link_bandwidth_mbps(pdev);
	if (bw < 0)
		return bw;
	bw /= BITS_PER_BYTE;

	for (int i = 0; i < ACCESS_COORDINATE_MAX; i++) {
		c[i].read_bandwidth = bw;
		c[i].write_bandwidth = bw;
	}

	return 0;
}

/*
 * Set max timeout such that platforms will optimize GPF flow to avoid
 * the implied worst-case scenario delays. On a sane platform, all
 * devices should always complete GPF within the energy budget of
 * the GPF flow. The kernel does not have enough information to pick
 * anything better than "maximize timeouts and hope it works".
 *
 * A misbehaving device could block forward progress of GPF for all
 * the other devices, exhausting the energy budget of the platform.
 * However, the spec seems to assume that moving on from slow to respond
 * devices is a virtue. It is not possible to know that, in actuality,
 * the slow to respond device is *the* most critical device in the
 * system to wait.
 */
#define GPF_TIMEOUT_BASE_MAX 2
#define GPF_TIMEOUT_SCALE_MAX 7 /* 10 seconds */

u16 cxl_gpf_get_dvsec(struct device *dev)
{
	struct pci_dev *pdev;
	bool is_port = true;
	u16 dvsec;

	if (!dev_is_pci(dev))
		return 0;

	pdev = to_pci_dev(dev);
	if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ENDPOINT ||
	    is_cxl_restricted(pdev))
		is_port = false;

	dvsec = pci_find_dvsec_capability(pdev, PCI_VENDOR_ID_CXL,
			is_port ? PCI_DVSEC_CXL_PORT_GPF : PCI_DVSEC_CXL_DEVICE_GPF);
	if (!dvsec)
		dev_warn(dev, "%s GPF DVSEC not present\n",
			 is_port ? "Port" : "Device");
	return dvsec;
}
EXPORT_SYMBOL_NS_GPL(cxl_gpf_get_dvsec, "CXL");

static int update_gpf_port_dvsec(struct pci_dev *pdev, int dvsec, int phase)
{
	u64 base, scale;
	int rc, offset;
	u16 ctrl;

	switch (phase) {
	case 1:
		offset = PCI_DVSEC_CXL_PORT_GPF_PHASE_1_CONTROL;
		base = PCI_DVSEC_CXL_PORT_GPF_PHASE_1_TMO_BASE;
		scale = PCI_DVSEC_CXL_PORT_GPF_PHASE_1_TMO_SCALE;
		break;
	case 2:
		offset = PCI_DVSEC_CXL_PORT_GPF_PHASE_2_CONTROL;
		base = PCI_DVSEC_CXL_PORT_GPF_PHASE_2_TMO_BASE;
		scale = PCI_DVSEC_CXL_PORT_GPF_PHASE_2_TMO_SCALE;
		break;
	default:
		return -EINVAL;
	}

	rc = pci_read_config_word(pdev, dvsec + offset, &ctrl);
	if (rc)
		return pcibios_err_to_errno(rc);

	if (FIELD_GET(base, ctrl) == GPF_TIMEOUT_BASE_MAX &&
	    FIELD_GET(scale, ctrl) == GPF_TIMEOUT_SCALE_MAX)
		return 0;

	FIELD_MODIFY(base, &ctrl, GPF_TIMEOUT_BASE_MAX);
	FIELD_MODIFY(scale, &ctrl, GPF_TIMEOUT_SCALE_MAX);

	rc = pci_write_config_word(pdev, dvsec + offset, ctrl);
	if (rc) {
		rc = pcibios_err_to_errno(rc);
		pci_warn(pdev, "Port GPF phase %d timeout write failed: %d\n",
			 phase, rc);
		return rc;
	}

	pci_dbg(pdev, "Port GPF phase %d timeout: %d0 secs\n",
		phase, GPF_TIMEOUT_BASE_MAX);

	return 0;
}

int cxl_gpf_port_setup(struct cxl_dport *dport)
{
	if (!dport)
		return -EINVAL;

	if (!dport->gpf_dvsec) {
		struct pci_dev *pdev;
		int dvsec, rc;

		dvsec = cxl_gpf_get_dvsec(dport->dport_dev);
		if (!dvsec)
			return -EINVAL;

		pdev = to_pci_dev(dport->dport_dev);
		rc = update_gpf_port_dvsec(pdev, dvsec, 1);
		if (rc)
			return rc;

		rc = update_gpf_port_dvsec(pdev, dvsec, 2);
		if (rc)
			return rc;

		dport->gpf_dvsec = dvsec;
	}

	return 0;
}

struct cxl_walk_context {
	struct pci_bus *bus;
	struct cxl_port *port;
	int type;
	int error;
	int count;
};

static int count_dports(struct pci_dev *pdev, void *data)
{
	struct cxl_walk_context *ctx = data;
	int type = pci_pcie_type(pdev);

	if (pdev->bus != ctx->bus)
		return 0;
	if (!pci_is_pcie(pdev))
		return 0;
	if (type != ctx->type)
		return 0;

	ctx->count++;
	return 0;
}

int cxl_port_get_possible_dports(struct cxl_port *port)
{
	struct pci_bus *bus = cxl_port_to_pci_bus(port);
	struct cxl_walk_context ctx;
	int type;

	if (!bus) {
		dev_err(&port->dev, "No PCI bus found for port %s\n",
			dev_name(&port->dev));
		return -ENXIO;
	}

	if (pci_is_root_bus(bus))
		type = PCI_EXP_TYPE_ROOT_PORT;
	else
		type = PCI_EXP_TYPE_DOWNSTREAM;

	ctx = (struct cxl_walk_context) {
		.bus = bus,
		.type = type,
	};
	pci_walk_bus(bus, count_dports, &ctx);

	return ctx.count;
}

static void cxl_dport_map_bi(struct cxl_dport *dport)
{
	struct cxl_register_map *map = &dport->reg_map;
	struct device *dev = dport->dport_dev;

	if (!map->component_map.bi_decoder.valid) {
		dev_dbg(dev, "BI Decoder registers not found\n");
		return;
	}

	if (cxl_map_component_regs(map, &dport->regs.component,
				   BIT(CXL_CM_CAP_CAP_ID_BI_DECODER)))
		dev_dbg(dev, "Failed to map BI Decoder capability\n");
}

/**
 * devm_cxl_dport_bi_setup - Map BI Decoder registers on a CXL dport
 * @dport: the cxl_dport that needs to be initialized
 *
 * Must be called while the dport's devres group is open so iomap
 * allocations are released on dport removal.
 */
void devm_cxl_dport_bi_setup(struct cxl_dport *dport)
{
	if (!dev_is_pci(dport->dport_dev))
		return;

	switch (pci_pcie_type(to_pci_dev(dport->dport_dev))) {
	case PCI_EXP_TYPE_ROOT_PORT:
	case PCI_EXP_TYPE_DOWNSTREAM:
		dport->reg_map.host = dport_to_host(dport);
		cxl_dport_map_bi(dport);
		break;
	default:
		break;
	}
}

/*
 * BI requires 256B Flit operation on the link. RP/DSP/endpoint must
 * also have the BI Decoder cap mapped (@bi); for USPs the BI RT cap
 * is optional per CXL 4.0 8.2.4.26, so absent @bi is allowed.
 */
static bool cxl_is_bi_capable(struct pci_dev *pdev, void __iomem *bi)
{
	if (!cxl_pci_flit_256(pdev))
		return false;

	if (pci_pcie_type(pdev) != PCI_EXP_TYPE_UPSTREAM && !bi) {
		dev_dbg(&pdev->dev, "No BI Decoder registers.\n");
		return false;
	}

	return true;
}

/* limit any insane timeouts from hw */
#define CXL_BI_COMMIT_MAXTMO_US (20 * USEC_PER_SEC)

static unsigned long __cxl_bi_get_timeout_us(struct device *dev,
					     unsigned int scale,
					     unsigned int base)
{
	static const unsigned long scale_tbl[] = {
		1, 10, 100, 1000, 10000, 100000, 1000000, 10000000,
	};

	if (scale >= ARRAY_SIZE(scale_tbl) || !base) {
		dev_dbg(dev, "Invalid BI commit timeout: scale=%u base=%u\n",
			scale, base);
		return CXL_BI_COMMIT_MAXTMO_US;
	}

	return scale_tbl[scale] * base;
}

static int __cxl_bi_wait_commit(struct device *dev, void __iomem *status_reg,
				u32 committed_bit, u32 err_bit,
				unsigned int scale, unsigned int base)
{
	unsigned long tmo_us, poll_us;
	ktime_t start;
	u32 status;
	int rc;

	tmo_us = min_t(unsigned long, CXL_BI_COMMIT_MAXTMO_US,
		       __cxl_bi_get_timeout_us(dev, scale, base));
	poll_us = max_t(unsigned long, tmo_us / 10, 1); /* ~10% */
	start = ktime_get();

	rc = readx_poll_timeout(readl, status_reg, status,
				status & (committed_bit | err_bit),
				poll_us, tmo_us);
	if (rc) {
		dev_err(dev, "BI-ID commit timed out (%luus)\n", tmo_us);
		return rc; /* -ETIMEDOUT */
	}

	if (status & err_bit) {
		dev_err(dev, "BI-ID commit rejected by hardware\n");
		return -EIO;
	}

	dev_dbg(dev, "BI-ID commit wait took %lluus\n",
		ktime_to_us(ktime_sub(ktime_get(), start)));
	return 0;
}

/* BI RT only exists on switch upstream ports. */
static int __cxl_bi_commit_rt(struct device *dev, void __iomem *bi)
{
	u32 status, ctrl;
	unsigned int scale, base;

	if (!FIELD_GET(CXL_BI_RT_CAPS_EXPLICIT_COMMIT_REQ,
		       readl(bi + CXL_BI_RT_CAPS_OFFSET)))
		return 0;

	ctrl = readl(bi + CXL_BI_RT_CTRL_OFFSET);
	writel(ctrl & ~CXL_BI_RT_CTRL_BI_COMMIT, bi + CXL_BI_RT_CTRL_OFFSET);
	writel(ctrl | CXL_BI_RT_CTRL_BI_COMMIT, bi + CXL_BI_RT_CTRL_OFFSET);

	status = readl(bi + CXL_BI_RT_STATUS_OFFSET);
	scale = FIELD_GET(CXL_BI_RT_STATUS_BI_COMMIT_TM_SCALE, status);
	base = FIELD_GET(CXL_BI_RT_STATUS_BI_COMMIT_TM_BASE, status);

	return __cxl_bi_wait_commit(dev, bi + CXL_BI_RT_STATUS_OFFSET,
				    CXL_BI_RT_STATUS_BI_COMMITTED,
				    CXL_BI_RT_STATUS_BI_ERR_NOT_COMMITTED,
				    scale, base);
}

static int __cxl_bi_commit_decoder(struct device *dev, void __iomem *bi)
{
	u32 status, ctrl;
	unsigned int scale, base;

	if (!FIELD_GET(CXL_BI_DECODER_CAPS_EXPLICIT_COMMIT_REQ,
		       readl(bi + CXL_BI_DECODER_CAPS_OFFSET)))
		return 0;

	ctrl = readl(bi + CXL_BI_DECODER_CTRL_OFFSET);
	writel(ctrl & ~CXL_BI_DECODER_CTRL_BI_COMMIT,
	       bi + CXL_BI_DECODER_CTRL_OFFSET);
	writel(ctrl | CXL_BI_DECODER_CTRL_BI_COMMIT,
	       bi + CXL_BI_DECODER_CTRL_OFFSET);

	status = readl(bi + CXL_BI_DECODER_STATUS_OFFSET);
	scale = FIELD_GET(CXL_BI_DECODER_STATUS_BI_COMMIT_TM_SCALE, status);
	base = FIELD_GET(CXL_BI_DECODER_STATUS_BI_COMMIT_TM_BASE, status);

	return __cxl_bi_wait_commit(dev, bi + CXL_BI_DECODER_STATUS_OFFSET,
				    CXL_BI_DECODER_STATUS_BI_COMMITTED,
				    CXL_BI_DECODER_STATUS_BI_ERR_NOT_COMMITTED,
				    scale, base);
}

/* Committed, or no explicit commit required */
static bool __cxl_bi_committed(void __iomem *caps_reg, void __iomem *status_reg,
			       u32 req_bit, u32 committed_bit)
{
	if (!(readl(caps_reg) & req_bit))
		return true;

	return readl(status_reg) & committed_bit;
}

static bool cxl_bi_rt_committed(void __iomem *bi)
{
	return __cxl_bi_committed(bi + CXL_BI_RT_CAPS_OFFSET,
				  bi + CXL_BI_RT_STATUS_OFFSET,
				  CXL_BI_RT_CAPS_EXPLICIT_COMMIT_REQ,
				  CXL_BI_RT_STATUS_BI_COMMITTED);
}

static bool cxl_bi_decoder_committed(void __iomem *bi)
{
	return __cxl_bi_committed(bi + CXL_BI_DECODER_CAPS_OFFSET,
				  bi + CXL_BI_DECODER_STATUS_OFFSET,
				  CXL_BI_DECODER_CAPS_EXPLICIT_COMMIT_REQ,
				  CXL_BI_DECODER_STATUS_BI_COMMITTED);
}

static int cxl_bi_commit_dport(struct cxl_dport *dport)
{
	struct cxl_port *port = dport->port;
	int rc;

	lockdep_assert_held(&port->bi_lock);

	/* root ports never require the explicit commit */
	if (pci_pcie_type(to_pci_dev(dport->dport_dev)) !=
	    PCI_EXP_TYPE_DOWNSTREAM)
		return 0;

	rc = __cxl_bi_commit_decoder(dport->dport_dev, dport->regs.bi_decoder);
	if (rc)
		return rc;

	if (port->regs.bi_rt)
		rc = __cxl_bi_commit_rt(&port->dev, port->regs.bi_rt);

	return rc;
}

/*
 * Enable BI-ID changes in the given level of the topology.
 * @direct says the device is connected directly to this dport, which
 * takes BI Enable; every dport above it takes BI Forward.
 */
static int cxl_bi_ctrl_dport_enable(struct cxl_dport *dport, bool direct,
				    bool *adopt)
{
	struct cxl_port *port = dport->port;
	u32 ctrl, value, set, clr;
	void __iomem *bi;
	int rc;

	guard(mutex)(&port->bi_lock);

	bi = dport->regs.bi_decoder;
	if (!bi)
		return -EINVAL;

	ctrl = readl(bi + CXL_BI_DECODER_CTRL_OFFSET);

	/*
	 * First reference through a level already up: firmware brought
	 * it up, or this driver did for a device it let go of while it
	 * still decoded with BI, which a dport re-created since cannot
	 * otherwise know. Either way it is not this driver's to take
	 * down: only a level it brought up from zero is ever cleared.
	 */
	if (!dport->nr_bi &&
	    (ctrl & (CXL_BI_DECODER_CTRL_BI_FW | CXL_BI_DECODER_CTRL_BI_ENABLE)))
		dport->bi_held = true;

	set = direct ? CXL_BI_DECODER_CTRL_BI_ENABLE :
		       CXL_BI_DECODER_CTRL_BI_FW;
	clr = direct ? CXL_BI_DECODER_CTRL_BI_FW :
		       CXL_BI_DECODER_CTRL_BI_ENABLE;

	value = (ctrl | set) & ~clr;

	/*
	 * Adopt this level as firmware left it: nothing below it that
	 * firmware did not account for, nothing this driver has routed
	 * through it, the value this walk would write already in place,
	 * and the decoder (and route table) committed.
	 */
	if (*adopt && !dport->nr_bi && value == ctrl &&
	    cxl_bi_decoder_committed(bi) &&
	    (!port->regs.bi_rt || cxl_bi_rt_committed(port->regs.bi_rt)))
		goto done;
	/* firmware did not bring this level up, so nothing above may */
	*adopt = false;

	if (value != ctrl)
		writel(value, bi + CXL_BI_DECODER_CTRL_OFFSET);

	/* owed per new device below, not per register change */
	rc = cxl_bi_commit_dport(dport);
	if (rc) {
		if (value != ctrl) {
			/* the undo is a BI-ID change owing its own commit */
			writel(ctrl, bi + CXL_BI_DECODER_CTRL_OFFSET);
			cxl_bi_commit_dport(dport);
		}
		return rc;
	}
done:
	dport->nr_bi++;

	return 0;
}

/*
 * Dealloc BI-ID changes in the given level of the topology. Called
 * once per endpoint that enabled this level: on teardown, or to
 * unwind a path that failed partway up. @hold marks the level for a
 * device below that still decodes with BI, so that no teardown
 * reaching zero clears it.
 */
static int cxl_bi_ctrl_dport_disable(struct cxl_dport *dport, bool hold)
{
	struct cxl_port *port = dport->port;
	void __iomem *bi;
	u32 ctrl;

	guard(mutex)(&port->bi_lock);

	bi = dport->regs.bi_decoder;
	if (!bi)
		return -EINVAL;

	if (WARN_ON_ONCE(dport->nr_bi == 0))
		return -EINVAL;

	if (hold)
		dport->bi_held = true;

	/* others below still need this level */
	if (--dport->nr_bi > 0)
		return 0;

	/* found up, or a device below still decodes with BI */
	if (dport->bi_held)
		return 0;

	ctrl = readl(bi + CXL_BI_DECODER_CTRL_OFFSET);
	writel(ctrl & ~(CXL_BI_DECODER_CTRL_BI_FW |
			CXL_BI_DECODER_CTRL_BI_ENABLE),
	       bi + CXL_BI_DECODER_CTRL_OFFSET);

	return cxl_bi_commit_dport(dport);
}

static int __cxl_bi_ctrl_endpoint(struct cxl_dev_state *cxlds, bool enable)
{
	struct cxl_port *endpoint = cxlds->cxlmd->endpoint;
	void __iomem *bi = endpoint->regs.bi_decoder;
	u32 ctrl;

	if (!bi)
		return -EINVAL;

	ctrl = readl(bi + CXL_BI_DECODER_CTRL_OFFSET);

	if (enable) {
		if (FIELD_GET(CXL_BI_DECODER_CTRL_BI_ENABLE, ctrl)) {
			if (!cxlds->bi) /* adopt firmware enabled */
				dev_dbg(cxlds->dev,
					"adopting firmware-enabled BI\n");
			goto done;
		}
	} else {
		if (!FIELD_GET(CXL_BI_DECODER_CTRL_BI_ENABLE, ctrl)) {
			if (!cxlds->bi)
				return 0;
			dev_err(cxlds->dev,
				"BI already disabled in hardware\n");
			return -EBUSY;
		}
	}

	FIELD_MODIFY(CXL_BI_DECODER_CTRL_BI_ENABLE, &ctrl, enable);
	writel(ctrl, bi + CXL_BI_DECODER_CTRL_OFFSET);

	dev_dbg(cxlds->dev, "BI requests %s\n", str_enabled_disabled(enable));
done:
	cxlds->bi = enable;
	return 0;
}

static int cxl_bi_ctrl_endpoint_enable(struct cxl_dev_state *cxlds)
{
	return __cxl_bi_ctrl_endpoint(cxlds, true);
}

static int cxl_bi_ctrl_endpoint_disable(struct cxl_dev_state *cxlds)
{
	return __cxl_bi_ctrl_endpoint(cxlds, false);
}

/*
 * Release the device's BI, then each dport up the path short of @stop,
 * following the deallocate sequence in the implementation notes of
 * CXL 4.0 9.14.2. Setup's unwind and teardown alike: a level the
 * walk adopted was found up and is held, so the unwind leaves its
 * register as it found it.
 *
 * A decoder that still decodes with its BI bit set needs BI Enable,
 * the device-wide permission behind it, so the device keeps it. The
 * references are still released - @nr_bi has to stay true to the
 * endpoints below - and each level is held instead, so a later
 * teardown reaching zero leaves it alone. An unwind never holds: the
 * walk failed before the device's BI was taken up, so no path is left
 * to keep up for it.
 */
static void cxl_bi_release(struct cxl_dev_state *cxlds, struct cxl_dport *stop)
{
	struct cxl_port *endpoint = cxlds->cxlmd->endpoint;
	bool live = cxl_hdm_bi_committed(endpoint);
	bool hold = live && cxlds->bi;
	struct cxl_dport *dport_iter;
	struct cxl_port *port_iter;

	/* only the binding that took up the device's BI releases it */
	if (cxlds->bi && !live)
		cxl_bi_ctrl_endpoint_disable(cxlds);
	cxlds->bi = false;

	/*
	 * Walk the same parent_dport chain that enabled the path. A bus
	 * lookup cannot stand in for it: an ancestor-driven teardown
	 * delists the parent port before this devres action runs.
	 */
	dport_iter = endpoint->parent_dport;
	port_iter = dport_iter->port;
	while (!is_cxl_root(port_iter) && dport_iter != stop) {
		int rc = cxl_bi_ctrl_dport_disable(dport_iter, hold);

		/* best effort */
		if (rc)
			dev_dbg(&port_iter->dev,
				"BI dport disable failed: %d\n", rc);

		dport_iter = port_iter->parent_dport;
		port_iter = dport_iter->port;
	}
}

/*
 * devm teardown on endpoint port destruction. Registered before the
 * decoders, so devres runs it after them: regions are detached and
 * decoders unregistered by the time BI comes down.
 */
static void cxl_bi_dealloc(void *data)
{
	struct cxl_port *endpoint = data;
	struct cxl_memdev *cxlmd = to_cxl_memdev(endpoint->uport_dev);

	cxl_bi_release(cxlmd->cxlds, NULL);
}

/*
 * Enable BI on every dport in the path, then on the device itself,
 * following the allocate sequence in the implementation notes of
 * CXL 4.0 9.14.2. On failure, unwind only the dports that enabled.
 */
static int cxl_bi_enable_path(struct cxl_dev_state *cxlds,
			      struct cxl_port *port, struct cxl_dport *dport)
{
	struct cxl_port *endpoint = cxlds->cxlmd->endpoint;
	struct cxl_dport *dport_iter;
	struct cxl_port *port_iter;
	bool adopt;
	int rc;

	/*
	 * Adoption is a path property, not a per-dport one: only when
	 * firmware enabled the device itself does a dport's committed
	 * state cover this device's BI-ID.
	 */
	adopt = cxl_bi_decoder_enabled(endpoint);

	port_iter = port;
	dport_iter = dport;
	while (!is_cxl_root(port_iter)) {
		rc = cxl_bi_ctrl_dport_enable(dport_iter, dport_iter == dport,
					      &adopt);
		if (rc)
			goto err_rollback;

		dport_iter = port_iter->parent_dport;
		port_iter = dport_iter->port;
	}

	rc = cxl_bi_ctrl_endpoint_enable(cxlds);
	if (rc)
		goto err_rollback;

	return 0;

err_rollback:
	/* the failed level, and any above it, took no reference */
	cxl_bi_release(cxlds, dport_iter);
	return rc;
}

/*
 * An SBR wipes the device's BI Enable; an FLR leaves it alone.
 * The check is against the hardware, not decoder state: BI is
 * enabled at probe, so it can be wiped with no decoder ever
 * committed. A wipe invalidates the software state; BI is never
 * re-enabled here.
 */
void cxl_bi_reset_detected(struct cxl_port *endpoint)
{
	struct cxl_memdev *cxlmd = to_cxl_memdev(endpoint->uport_dev);
	struct cxl_dev_state *cxlds = cxlmd->cxlds;

	/*
	 * The BI Decoder mapping belongs to the endpoint port, and an
	 * unbind of that port alone releases it: hold the port while it
	 * is read, and read nothing once the port driver is gone.
	 */
	guard(device)(&endpoint->dev);
	if (!endpoint->dev.driver)
		return;

	if (!READ_ONCE(cxlds->bi))
		return;

	if (cxl_bi_decoder_enabled(endpoint))
		return;

	dev_warn(cxlds->dev, "BI disabled by reset\n");
	WRITE_ONCE(cxlds->bi, false);
}
EXPORT_SYMBOL_NS_GPL(cxl_bi_reset_detected, "CXL");

/*
 * Verify the device and every component in the path up to the root
 * are BI capable.
 */
void cxl_bi_probe_capable(struct cxl_port *endpoint)
{
	struct cxl_memdev *cxlmd = to_cxl_memdev(endpoint->uport_dev);
	struct cxl_dev_state *cxlds = cxlmd->cxlds;
	struct cxl_dport *dport_iter;
	struct cxl_port *port_iter;

	cxlds->bi_capable = false;

	if (!dev_is_pci(cxlds->dev))
		return;

	/* BI is VH-only */
	if (cxlds->rcd)
		return;

	if (!cxl_is_bi_capable(to_pci_dev(cxlds->dev),
			       endpoint->regs.bi_decoder))
		return;

	dport_iter = endpoint->parent_dport;
	port_iter = dport_iter->port;
	while (!is_cxl_root(port_iter)) {
		/* check rp, dsp */
		if (!cxl_is_bi_capable(to_pci_dev(dport_iter->dport_dev),
				       dport_iter->regs.bi_decoder)) {
			dev_dbg(cxlds->dev, "BI not supported by topology\n");
			return;
		}

		/* check usp */
		if (dev_is_pci(port_iter->uport_dev) &&
		    pci_pcie_type(to_pci_dev(port_iter->uport_dev)) ==
			    PCI_EXP_TYPE_UPSTREAM) {
			if (!cxl_is_bi_capable(to_pci_dev(port_iter->uport_dev),
					       NULL)) {
				dev_dbg(cxlds->dev,
					"BI not supported by USP\n");
				return;
			}
			if (port_iter->reg_map.component_map.bi_rt.valid &&
			    !port_iter->regs.bi_rt) {
				dev_dbg(cxlds->dev,
					"BI RT advertised but unmapped\n");
				return;
			}
		}

		dport_iter = port_iter->parent_dport;
		port_iter = dport_iter->port;
	}

	cxlds->bi_capable = true;
}
EXPORT_SYMBOL_NS_GPL(cxl_bi_probe_capable, "CXL");

int cxl_bi_setup(struct cxl_port *endpoint)
{
	struct cxl_memdev *cxlmd = to_cxl_memdev(endpoint->uport_dev);
	struct cxl_dport *dport = endpoint->parent_dport;
	struct cxl_dev_state *cxlds = cxlmd->cxlds;
	int rc;

	if (!cxlds->bi_capable)
		return 0;

	rc = cxl_bi_enable_path(cxlds, dport->port, dport);
	if (rc)
		return rc;

	return devm_add_action_or_reset(&endpoint->dev, cxl_bi_dealloc,
					endpoint);
}
EXPORT_SYMBOL_NS_GPL(cxl_bi_setup, "CXL");
