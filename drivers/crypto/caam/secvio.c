// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * SNVS Security Violation Handler
 *
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019, 2023 NXP
 */

#include "compat.h"
#include "secvio.h"
#include "regs.h"
#include "intern.h"
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

/* The driver is matched with node caam_snvs to get regmap
 * It will then retrieve interruption and tamper alarm configuration from
 * node caam-secvio searching for the compat string "fsl,imx6q-caam-secvio"
 */
#define DRIVER_NAME "caam-snvs"

/*
 * These names are associated with each violation handler.
 * The source names were taken from MX6, and are based on recommendations
 * for most common SoCs.
 */
static const u8 *violation_src_name[] = {
	"CAAM Internal Security Violation",
	"JTAG Alarm",
	"Watchdog",
	"(reserved)",
	"External Boot",
	"External Tamper Detect",
};

/* These names help describe security monitor state for the console */
static const u8 *snvs_ssm_state_name[] = {
	"init",
	"hard fail",
	"(undef:2)",
	"soft fail",
	"(undef:4)",
	"(undef:5)",
	"(undef:6)",
	"(undef:7)",
	"transition",
	"check",
	"(undef:10)",
	"non-secure",
	"(undef:12)",
	"trusted",
	"(undef:14)",
	"secure",
};

static DEFINE_STATIC_KEY_TRUE(snvs_little_end);

static inline u32 secvio_read(void __iomem *reg)
{
	if (static_branch_likely(&snvs_little_end))
		return ioread32(reg);
	else
		return ioread32be(reg);
}

static inline void secvio_write(void __iomem *reg, u32 data)
{
	if (static_branch_likely(&snvs_little_end))
		iowrite32(data, reg);
	else
		iowrite32be(data, reg);
}

/* Top-level security violation interrupt */
static irqreturn_t snvs_secvio_interrupt(int irq, void *snvsdev)
{
	struct device *dev = snvsdev;
	struct snvs_secvio_drv_private *svpriv = dev_get_drvdata(dev);

	clk_enable(svpriv->clk);
	/* Check the HP secvio status register */
	svpriv->irqcause = secvio_read(&svpriv->svregs->hp.secvio_status) &
				       HP_SECVIOST_SECVIOMASK;

	if (!svpriv->irqcause) {
		clk_disable(svpriv->clk);
		return IRQ_NONE;
	}

	/* Now ACK cause */
	clrsetbits_32(&svpriv->svregs->hp.secvio_status, 0, svpriv->irqcause);

	/* And run deferred service */
	preempt_disable();
	tasklet_schedule(&svpriv->irqtask[smp_processor_id()]);
	preempt_enable();

	clk_disable(svpriv->clk);

	return IRQ_HANDLED;
}

/* Deferred service handler. Tasklet arg is simply the SNVS dev */
static void snvs_secvio_dispatch(unsigned long indev)
{
	struct device *dev = (struct device *)indev;
	struct snvs_secvio_drv_private *svpriv = dev_get_drvdata(dev);
	unsigned long flags;
	int i;


	/* Look through stored causes, call each handler if exists */
	for (i = 0; i < MAX_SECVIO_SOURCES; i++)
		if (svpriv->irqcause & (1 << i)) {
			spin_lock_irqsave(&svpriv->svlock, flags);
			svpriv->intsrc[i].handler(dev, i,
						  svpriv->intsrc[i].ext);
			spin_unlock_irqrestore(&svpriv->svlock, flags);
		};

	/* Re-enable now-serviced interrupts */
	clrsetbits_32(&svpriv->svregs->hp.secvio_intcfg, 0, svpriv->irqcause);
}

/*
 * Default cause handler, used in lieu of an application-defined handler.
 * All it does at this time is print a console message. It could force a halt.
 */
static void snvs_secvio_default(struct device *dev, u32 cause, void *ext)
{
	struct snvs_secvio_drv_private *svpriv = dev_get_drvdata(dev);

	dev_err(dev, "Unhandled Security Violation Interrupt %d = %s\n",
		cause, svpriv->intsrc[cause].intname);
}

/*
 * Install an application-defined handler for a specified cause
 * Arguments:
 * - dev        points to SNVS-owning device
 * - cause      interrupt source cause
 * - handler    application-defined handler, gets called with dev
 *              source cause, and locally-defined handler argument
 * - cause_description   points to a string to override the default cause
 *                       name, this can be used as an alternate for error
 *                       messages and such. If left NULL, the default
 *                       description string is used.
 * - ext        pointer to any extra data needed by the handler.
 */
int snvs_secvio_install_handler(struct device *dev, enum secvio_cause cause,
				void (*handler)(struct device *dev, u32 cause,
						void *ext),
				u8 *cause_description, void *ext)
{
	unsigned long flags;
	struct snvs_secvio_drv_private *svpriv;

	svpriv = dev_get_drvdata(dev);

	if ((handler == NULL) || (cause > SECVIO_CAUSE_SOURCE_5))
		return -EINVAL;

	spin_lock_irqsave(&svpriv->svlock, flags);
	svpriv->intsrc[cause].handler = handler;
	if (cause_description != NULL)
		svpriv->intsrc[cause].intname = cause_description;
	if (ext != NULL)
		svpriv->intsrc[cause].ext = ext;
	spin_unlock_irqrestore(&svpriv->svlock, flags);

	return 0;
}
EXPORT_SYMBOL(snvs_secvio_install_handler);

/*
 * Remove an application-defined handler for a specified cause (and, by
 * implication, restore the "default".
 * Arguments:
 * - dev	points to SNVS-owning device
 * - cause	interrupt source cause
 */
int snvs_secvio_remove_handler(struct device *dev, enum secvio_cause cause)
{
	unsigned long flags;
	struct snvs_secvio_drv_private *svpriv;

	svpriv = dev_get_drvdata(dev);

	if (cause > SECVIO_CAUSE_SOURCE_5)
		return -EINVAL;

	spin_lock_irqsave(&svpriv->svlock, flags);
	svpriv->intsrc[cause].intname = violation_src_name[cause];
	svpriv->intsrc[cause].handler = snvs_secvio_default;
	svpriv->intsrc[cause].ext = NULL;
	spin_unlock_irqrestore(&svpriv->svlock, flags);
	return 0;
}
EXPORT_SYMBOL(snvs_secvio_remove_handler);

static int snvs_secvio_remove(struct platform_device *pdev)
{
	struct device *svdev;
	struct snvs_secvio_drv_private *svpriv;
	int i;

	svdev = &pdev->dev;
	svpriv = dev_get_drvdata(svdev);

	clk_enable(svpriv->clk);
	/* Set all sources to nonfatal */
	secvio_write(&svpriv->svregs->hp.secvio_intcfg, 0);

	/* Remove tasklets and release interrupt */
	for_each_possible_cpu(i)
		tasklet_kill(&svpriv->irqtask[i]);

	clk_disable_unprepare(svpriv->clk);
	free_irq(svpriv->irq, svdev);
	iounmap(svpriv->svregs);
	kfree(svpriv);

	return 0;
}

static int snvs_secvio_probe(struct platform_device *pdev)
{
	struct device *svdev;
	struct snvs_secvio_drv_private *svpriv;
	struct device_node *np, *npirq;
	struct snvs_full __iomem *snvsregs;
	int i, error;
	u32 hpstate;
	const void *jtd, *wtd, *itd, *etd;
	u32 td_en;
	u32 ipidr, ipid;

	svpriv = kzalloc(sizeof(struct snvs_secvio_drv_private), GFP_KERNEL);
	if (!svpriv)
		return -ENOMEM;

	svdev = &pdev->dev;
	dev_set_drvdata(svdev, svpriv);
	svpriv->pdev = pdev;
	spin_lock_init(&svpriv->svlock);
	np = pdev->dev.of_node;

	npirq = of_find_compatible_node(NULL, NULL, "fsl,imx6q-caam-secvio");
	if (!npirq) {
		dev_err(svdev, "can't find secvio node\n");
		kfree(svpriv);
		return -EINVAL;
	}
	svpriv->irq = irq_of_parse_and_map(npirq, 0);
	if (svpriv->irq <= 0) {
		dev_err(svdev, "can't identify secvio interrupt\n");
		kfree(svpriv);
		return -EINVAL;
	}

	jtd = of_get_property(npirq, "jtag-tamper", NULL);
	wtd = of_get_property(npirq, "watchdog-tamper", NULL);
	itd = of_get_property(npirq, "internal-boot-tamper", NULL);
	etd = of_get_property(npirq, "external-pin-tamper", NULL);
	if (!jtd | !wtd | !itd | !etd ) {
		dev_err(svdev, "can't identify all tamper alarm configuration\n");
		kfree(svpriv);
		return -EINVAL;
	}

	/*
	 * Configure all sources  according to device tree property.
	 * If the property is enabled then the source is ser as
	 * fatal violations except LP section,
	 * source #5 (typically used as an external tamper detect), and
	 * source #3 (typically unused). Whenever the transition to
	 * secure mode has occurred, these will now be "fatal" violations
	 */
	td_en = HP_SECVIO_INTEN_SRC0;
	if (!strcmp(jtd, "enabled"))
		td_en |= HP_SECVIO_INTEN_SRC1;
	if (!strcmp(wtd, "enabled"))
		td_en |= HP_SECVIO_INTEN_SRC2;
	if (!strcmp(itd, "enabled"))
		td_en |= HP_SECVIO_INTEN_SRC4;
	if (!strcmp(etd, "enabled"))
		td_en |= HP_SECVIO_INTEN_SRC5;

	snvsregs = of_iomap(np, 0);
	if (!snvsregs) {
		dev_err(svdev, "register mapping failed\n");
		return -ENOMEM;
	}
	svpriv->svregs = (struct snvs_full __force *)snvsregs;

	svpriv->clk = devm_clk_get_optional(&pdev->dev, "ipg");
	if (IS_ERR(svpriv->clk))
		return PTR_ERR(svpriv->clk);

	clk_prepare_enable(svpriv->clk);

	/*
	 * Reading SNVS version ID register HPVIDR1 to identify the endianness
	 * of the device which contain non-zero constants including 16-bit field
	 * called IP_ID[Bit 31-16] having one of the four values 0x003A, 0x003C,
	 * 0x003E, 0x003F.
	 */
	ipidr = secvio_read(&svpriv->svregs->vid);
	ipid = ipidr >> SNVS_HPVIDR_BLOCK_ID;
	if (ipid == SNVS_ID1 || ipid == SNVS_ID2 || ipid == SNVS_ID3 || ipid == SNVS_ID4) {
		dev_info(svdev, "ipid matched - 0x%x\n", ipid);
	} else {
		/*
		 * Device endianness is not LE.Reading again SNVS version ID
		 * register value to identify the endianness of device is BE.
		 */
		ipid = (ipidr & (u32)0x0000FF00) >> 8;
		if (ipid == SNVS_ID1 || ipid == SNVS_ID2 || ipid == SNVS_ID3 || ipid == SNVS_ID4) {
			dev_info(svdev, "ipid matched - 0x%x\n", ipid);
			static_branch_disable(&snvs_little_end);
		} else {
			dev_err(svdev, "unable to identify secvio endianness\n");
			iounmap(svpriv->svregs);
			kfree(svpriv);
			return -EINVAL;
		}
	}

	/* Write the Secvio Enable Config the SVCR */
	secvio_write(&svpriv->svregs->hp.secvio_ctl, td_en);
	secvio_write(&svpriv->svregs->hp.secvio_intcfg, td_en);

	 /* Device data set up. Now init interrupt source descriptions */
	for (i = 0; i < MAX_SECVIO_SOURCES; i++) {
		svpriv->intsrc[i].intname = violation_src_name[i];
		svpriv->intsrc[i].handler = snvs_secvio_default;
	}
	/* Connect main handler */
	for_each_possible_cpu(i)
		tasklet_init(&svpriv->irqtask[i], snvs_secvio_dispatch,
			     (unsigned long)svdev);

	error = request_irq(svpriv->irq, snvs_secvio_interrupt,
			    IRQF_SHARED, DRIVER_NAME, svdev);
	if (error) {
		dev_err(svdev, "can't connect secvio interrupt\n");
		irq_dispose_mapping(svpriv->irq);
		svpriv->irq = 0;
		iounmap(svpriv->svregs);
		kfree(svpriv);
		return -EINVAL;
	}

	hpstate = (secvio_read(&svpriv->svregs->hp.status) &
			       HP_STATUS_SSM_ST_MASK) >> HP_STATUS_SSM_ST_SHIFT;
	dev_info(svdev, "violation handlers armed - %s state\n",
		 snvs_ssm_state_name[hpstate]);

	clk_disable(svpriv->clk);

	return 0;
}

static struct of_device_id snvs_secvio_match[] = {
	{
		.compatible = "fsl,imx6q-caam-snvs",
	},
	{},
};
MODULE_DEVICE_TABLE(of, snvs_secvio_match);

static struct platform_driver snvs_secvio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = snvs_secvio_match,
	},
	.probe       = snvs_secvio_probe,
	.remove      = snvs_secvio_remove,
};

module_platform_driver(snvs_secvio_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("FSL SNVS Security Violation Handler");
MODULE_AUTHOR("Freescale Semiconductor - MCU");
