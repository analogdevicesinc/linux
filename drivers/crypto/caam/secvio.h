/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * CAAM Security Violation Handler
 *
 * Copyright 2012-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 */

#ifndef SECVIO_H
#define SECVIO_H

#include "snvsregs.h"


/*
 * Defines the published interfaces to install/remove application-specified
 * handlers for catching violations
 */

#define MAX_SECVIO_SOURCES 6

/* these are the untranslated causes */
enum secvio_cause {
	SECVIO_CAUSE_SOURCE_0,
	SECVIO_CAUSE_SOURCE_1,
	SECVIO_CAUSE_SOURCE_2,
	SECVIO_CAUSE_SOURCE_3,
	SECVIO_CAUSE_SOURCE_4,
	SECVIO_CAUSE_SOURCE_5
};

/* These are common "recommended" cause definitions for most devices */
#define SECVIO_CAUSE_CAAM_VIOLATION	SECVIO_CAUSE_SOURCE_0
#define SECVIO_CAUSE_JTAG_ALARM		SECVIO_CAUSE_SOURCE_1
#define SECVIO_CAUSE_WATCHDOG		SECVIO_CAUSE_SOURCE_2
#define SECVIO_CAUSE_EXTERNAL_BOOT	SECVIO_CAUSE_SOURCE_4
#define SECVIO_CAUSE_TAMPER_DETECT	SECVIO_CAUSE_SOURCE_5

int snvs_secvio_install_handler(struct device *dev, enum secvio_cause cause,
				void (*handler)(struct device *dev, u32 cause,
						void *ext),
				u8 *cause_description, void *ext);
int snvs_secvio_remove_handler(struct device *dev, enum  secvio_cause cause);

/*
 * Private data definitions for the secvio "driver"
 */

struct secvio_int_src {
	const u8 *intname;	/* Points to a descriptive name for source */
	void *ext;		/* Extended data to pass to the handler */
	void (*handler)(struct device *dev, u32 cause, void *ext);
};

struct snvs_secvio_drv_private {
	struct platform_device *pdev;
	spinlock_t svlock ____cacheline_aligned;
	struct tasklet_struct irqtask[NR_CPUS];
	struct snvs_full __iomem *svregs;	/* both HP and LP domains */
	struct clk *clk;
	int irq;
	u32 irqcause; /* stashed cause of violation interrupt */

	/* Registered handlers for each violation */
	struct secvio_int_src intsrc[MAX_SECVIO_SOURCES];

};

#endif /* SECVIO_H */
