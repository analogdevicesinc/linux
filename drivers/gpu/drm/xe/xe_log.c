// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include "xe_log.h"
#include "xe_printk.h"

static void log_emit_cper(struct pci_dev *pdev, int cper_sev, enum xe_sigid sigid,
			  u32 component, u32 location, const void *data, size_t len,
			  struct va_format *vaf)
{
	/* TODO */
}

static bool is_hw_sigid(enum xe_sigid sigid)
{
	return (int)sigid >= INTEL_SIGID_GPU_XE_HARDWARE_START;
}

static bool is_sev_error(int cper_sev)
{
	return cper_sev != CPER_SEV_INFORMATIONAL;
}

static const char *log_hwe_prefix(int cper_sev, enum xe_sigid sigid)
{
	return is_sev_error(cper_sev) && is_hw_sigid(sigid) ? HW_ERR : "";
}

static const char *log_sev_prefix(int cper_sev)
{
	switch (cper_sev) {
	case CPER_SEV_FATAL:
		return "FATAL ";
	case CPER_SEV_RECOVERABLE:
		return "";
	case CPER_SEV_CORRECTED:
		return "CORRECTED ";
	case CPER_SEV_INFORMATIONAL:
		return "";
	default:
		WARN(IS_ENABLED(CONFIG_DRM_XE_DEBUG), "LOG: unknown severity %d\n", cper_sev);
		return "";
	}
}

#define __LOG_DRM_PRINTK_FMT(fmt, args...)	"[drm] " fmt, ##args
#define __LOG_DRM_PRINTK_ERR_FMT(fmt, args...)	__LOG_DRM_PRINTK_FMT("*ERROR* " fmt, args)

static void log_dmesg_vprintk(struct pci_dev *pdev, int cper_sev, struct va_format *vaf)
{
	if (cper_sev == CPER_SEV_INFORMATIONAL)
		pci_info(pdev, __LOG_DRM_PRINTK_FMT("%pV", vaf));
	else
		pci_err(pdev, __LOG_DRM_PRINTK_ERR_FMT("%pV", vaf));
}

static void log_dmesg_printf(struct pci_dev *pdev, int cper_sev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	log_dmesg_vprintk(pdev, cper_sev, &vaf);

	va_end(args);
}

static void log_emit_dmesg(struct pci_dev *pdev, int cper_sev, enum xe_sigid sigid,
			   u32 component, u32 location, const void *data, size_t len,
			   struct va_format *vaf)
{
	const char *hwe_prefix = log_hwe_prefix(cper_sev, sigid);
	const char *sev_prefix = log_sev_prefix(cper_sev);

	/* TODO: add component/location details */

	if (IS_ERR(data))
		log_dmesg_printf(pdev, cper_sev, "SIGID=%u %s(%pe) %s%pV",
				 sigid, sev_prefix, data, hwe_prefix, vaf);
	else if (data && len)
		log_dmesg_printf(pdev, cper_sev, "SIGID=%u %s(%*phN) %s%pV",
				 sigid, sev_prefix, (int)len, data, hwe_prefix, vaf);
	else
		log_dmesg_printf(pdev, cper_sev, "SIGID=%u %s%s%pV",
				 sigid, sev_prefix, hwe_prefix, vaf);
}

/**
 * xe_log_emit() - Emit a structured SIGID log entry
 * @pdev: the &pci_dev device
 * @cper_sev: CPER severity (CPER_SEV_FATAL, CPER_SEV_RECOVERABLE, ...)
 * @sigid: signature identifier, see &enum xe_sigid
 * @component: component identifier
 * @location: location details of the @component
 * @data: pointer to the additional details, or ERR_PTR, or NULL if not applicable
 * @len: length of the @data in bytes, or 0 if not applicable
 * @fmt: printf-style format string
 * @...: format arguments
 *
 * Emits a dmesg line that includes a single stable, machine-matchable token
 * ``SIGID=<n>`` followed by the optional severity token (like ``FATAL``) and,
 * when @data pointer is set, either the error printed with %pe or a packed hex
 * dump of the @data binary blob. The dmesg line will also include printf-style
 * text message.
 *
 * Note that the full dmesg line, with the free text message, is only a debugging
 * aid, not an interface! Only the ``SIGID=<n>`` token is stable there.
 * The durable machine record is the CPER carrying the same SIGID.
 *
 * Note: generation of the CPER record is a planned follow-up.
 *
 * Examples::
 *
 *   <3> xe 0000:03:00.0: [drm] *ERROR* SIGID=104 FATAL (-EPROTO) Invalid GuC reply
 *   <3> xe 0000:03:00.0: [drm] *ERROR* SIGID=106 (-ETIMEDOUT) Engine 'rcs0' hung
 *   <6> xe 0000:03:00.0: [drm] SIGID=103 In survivability mode
 */
void xe_log_emit(struct pci_dev *pdev, int cper_sev, enum xe_sigid sigid,
		 u32 component, u32 location, const void *data, size_t len,
		 const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	log_emit_dmesg(pdev, cper_sev, sigid, component, location, data, len, &vaf);
	log_emit_cper(pdev, cper_sev, sigid, component, location, data, len, &vaf);

	va_end(args);
}
