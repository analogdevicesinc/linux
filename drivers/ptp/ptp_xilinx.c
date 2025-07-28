// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xilinx 1588 PTP timer syncer driver
 *
 * Copyright (c) 2020 Xilinx, Inc. All rights reserved.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/ptp/ptp_xilinx.h>

/* Register offset definitions */
#define XPTPTIMER_TOD_CONFIG_OFFSET	0x0000
#define XPTPTIMER_TOD_SNAPSHOT_OFFSET	0x0004
#define XPTPTIMER_IER_OFFSET		0x0008
#define XPTPTIMER_ISR_OFFSET		0x000C
#define XPTPTIMER_TOD_SW_SEC_0_OFFSET	0x0010
#define XPTPTIMER_TOD_SW_SEC_1_OFFSET	0x0014
#define XPTPTIMER_TOD_SW_NS_OFFSET	0x0018
#define XPTPTIMER_TOD_SW_LOAD_OFFSET	0x001C
#define XPTPTIMER_TOD_SEC_SYS_OFST_0_OFFSET	0x0028
#define XPTPTIMER_TOD_SEC_SYS_OFST_1_OFFSET	0x002C
#define XPTPTIMER_TOD_NS_SYS_OFST_OFFSET	0x0030
#define TOD_LEGACY_SYS_PERIOD_0		0x0130
#define TOD_LEGACY_SYS_PERIOD_1		0x0134
#define TOD_SYS_PERIOD_0		0x0034
#define TOD_SYS_PERIOD_1		0x0038

#define XPTPTIMER_SYS_SEC_0_OFFSET	0x0100
#define XPTPTIMER_SYS_SEC_1_OFFSET	0x0104
#define XPTPTIMER_SYS_NS_OFFSET		0x0108

#define XPTPTIMER_PORT_TX_PERIOD_0_OFFSET	0x0208
#define XPTPTIMER_PORT_TX_PERIOD_1_OFFSET	0x020C
#define XPTPTIMER_PORT_TX_NS_SNAP_OFFSET	0x0214
#define XPTPTIMER_PORT_TX_SEC_0_SNAP_OFFSET	0x0218
#define XPTPTIMER_PORT_TX_SEC_1_SNAP_OFFSET	0x021C
#define XPTPTIMER_PORT_RX_PERIOD_0_OFFSET	0x0228
#define XPTPTIMER_PORT_RX_PERIOD_1_OFFSET	0x022C
#define XPTPTIMER_PORT_RX_NS_SNAP_OFFSET	0x0234
#define XPTPTIMER_PORT_RX_SEC_0_SNAP_OFFSET	0x0238
#define XPTPTIMER_PORT_RX_SEC_1_SNAP_OFFSET	0x023C
#define PORT0_SEC_OFFSET_0                     0x0250
#define PORT0_SEC_OFFSET_1                     0x0254
#define PORT0_NS_OFFSET_0                      0x0258

#define XPTPTIMER_CFG_MAIN_TOD_EN	BIT(0)
#define XPTPTIMER_CFG_ENABLE_PORT0	BIT(16)
#define XPTPTIMER_CFG_AUTO_REF	BIT(4)

#define XPTPTIMER_MAX_SEC_SIZE		48
#define XPTPTIMER_MAX_SEC_MASK		GENMASK_ULL(XPTPTIMER_MAX_SEC_SIZE - 1, 0)

#define XPTPTIMER_TOD_OFFSET_NEG	BIT_ULL(47)

#define XPTPTIMER_SNAPSHOT_MASK		BIT(0)
#define XPTPTIMER_LOAD_TOD_MASK		BIT(0)
#define XPTPTIMER_LOAD_OFFSET_MASK	BIT(1)
#define XPTPTIMER_ENABLE_SNAPSHOT BIT(5)
#define XPTPTIMER_EXTS_1PPS_INTR_MASK  BIT(16)

/* TODO This should be derived from the system design */
#define XPTPTIMER_CLOCK_PERIOD		4
#define XPTPTIMER_PERIOD_SHIFT		48

#define PPM_FRACTION	16

/* I/O accessors */
static inline u32 xlnx_ptp_ior(struct xlnx_ptp_timer *timer, off_t reg)
{
	return ioread32(timer->baseaddr + reg);
}

static inline void xlnx_ptp_iow(struct xlnx_ptp_timer *timer, off_t reg,
				u32 value)
{
	iowrite32(value, (timer->baseaddr + reg));
}

/*
 * Inline timer helpers
 */
static inline void xlnx_tod_read(struct xlnx_ptp_timer *timer,
				 struct timespec64 *ts)
{
	u32 sech, secl, nsec;

	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SNAPSHOT_OFFSET,
		     XPTPTIMER_SNAPSHOT_MASK);

	if (timer->use_sys_timer_only) {
		nsec = xlnx_ptp_ior(timer, XPTPTIMER_SYS_NS_OFFSET);
		sech = xlnx_ptp_ior(timer, XPTPTIMER_SYS_SEC_1_OFFSET);
		secl = xlnx_ptp_ior(timer, XPTPTIMER_SYS_SEC_0_OFFSET);
	} else {
		/* use TX port here */
		nsec = xlnx_ptp_ior(timer, XPTPTIMER_PORT_TX_NS_SNAP_OFFSET);
		secl = xlnx_ptp_ior(timer, XPTPTIMER_PORT_TX_SEC_0_SNAP_OFFSET);
		sech = xlnx_ptp_ior(timer, XPTPTIMER_PORT_TX_SEC_1_SNAP_OFFSET);
	}

	ts->tv_nsec = nsec;
	ts->tv_sec = (((u64)sech << 32) | secl) & XPTPTIMER_MAX_SEC_MASK;
}

static inline void xlnx_port_offset_write(struct xlnx_ptp_timer *timer,
					  const struct timespec64 *ts)
{
	int i;

	xlnx_ptp_iow(timer, PORT0_SEC_OFFSET_0,
		     lower_32_bits(ts->tv_sec));
	xlnx_ptp_iow(timer, PORT0_SEC_OFFSET_1,
		     upper_32_bits(ts->tv_sec));
	/*
	 * Two writes are done to PORT0_NS_OFFSET_0 due to IP behavior which
	 * uses the same as trigger to accept the whole offset value.
	 */
	for (i = 0; i < 2; i++) {
		xlnx_ptp_iow(timer, PORT0_NS_OFFSET_0,
			     (u32)(ts->tv_nsec));
	}
}

static inline void xlnx_tod_offset_write(struct xlnx_ptp_timer *timer,
					 const struct timespec64 *ts)
{
	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SEC_SYS_OFST_1_OFFSET,
		     upper_32_bits(ts->tv_sec));
	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SEC_SYS_OFST_0_OFFSET,
		     lower_32_bits(ts->tv_sec));
	xlnx_ptp_iow(timer, XPTPTIMER_TOD_NS_SYS_OFST_OFFSET,
		     (u32)(ts->tv_nsec));

	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SW_LOAD_OFFSET,
		     XPTPTIMER_LOAD_OFFSET_MASK);
}

static inline void xlnx_tod_load_write(struct xlnx_ptp_timer *timer,
				       const struct timespec64 *ts)
{
	struct timespec64 offset;

	offset.tv_sec = 0;
	offset.tv_nsec = 0;

	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SW_SEC_1_OFFSET,
		     upper_32_bits(ts->tv_sec));
	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SW_SEC_0_OFFSET,
		     lower_32_bits(ts->tv_sec));
	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SW_NS_OFFSET, ts->tv_nsec);

	/* Make sure offset registers are cleared */
	xlnx_tod_offset_write(timer, &offset);
	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SW_LOAD_OFFSET,
		     XPTPTIMER_LOAD_OFFSET_MASK);

	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SW_LOAD_OFFSET,
		     XPTPTIMER_LOAD_TOD_MASK);
	xlnx_port_offset_write(timer, &offset);
	timer->timeoffset = 0;
}

static inline void xlnx_tod_period_write(struct xlnx_ptp_timer *timer, u64 adj)
{
	u32 adjhigh = upper_32_bits(adj);

	xlnx_ptp_iow(timer, timer->period_0, (u32)(adj));
	xlnx_ptp_iow(timer, timer->period_1, adjhigh);
}

static inline void xlnx_port_period_write(struct xlnx_ptp_timer *timer, u64 adj)
{
	u32 adjhigh = upper_32_bits(adj);

	xlnx_ptp_iow(timer, XPTPTIMER_PORT_TX_PERIOD_0_OFFSET, (u32)(adj));
	xlnx_ptp_iow(timer, XPTPTIMER_PORT_RX_PERIOD_0_OFFSET, (u32)(adj));
	spin_lock(&timer->reg_lock);
	xlnx_ptp_iow(timer, XPTPTIMER_PORT_TX_PERIOD_1_OFFSET, adjhigh);
	xlnx_ptp_iow(timer, XPTPTIMER_PORT_RX_PERIOD_1_OFFSET, adjhigh);

	spin_unlock(&timer->reg_lock);
}

/*
 * PTP clock operations
 */
/**
 * xlnx_ptp_adjfine - Fine adjustment of the frequency on the hardware clock
 * @ptp: ptp clock structure
 * @scaled_ppm: signed scaled parts per million for frequency adjustment.
 * Return: 0 on success
 * TX and RX port periods are reloaded with the adjusted value.
 *
 */
static int xlnx_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct xlnx_ptp_timer *timer = container_of(ptp, struct xlnx_ptp_timer,
					ptp_clock_info);
	bool neg_adj = false;
	u64 adj;

	if (scaled_ppm < 0) {
		neg_adj = true;
		scaled_ppm = -scaled_ppm;
	}

	adj = mul_u64_u32_div(timer->incr, scaled_ppm, USEC_PER_SEC);
	adj >>= PPM_FRACTION; /* remove fractions */
	adj = neg_adj ? (timer->incr - adj) : (timer->incr + adj);

	if (timer->use_sys_timer_only)
		xlnx_tod_period_write(timer, adj);
	else
		xlnx_port_period_write(timer, adj);

	return 0;
}

/**
 * xlnx_ptp_adjtime - Adjust the current time on the hardware clock
 * @ptp: ptp clock structure
 * @delta: signed time in ns to be adjusted.
 * Return: 0 on success
 * System, TX and RX ports are reloaded with the adjusted time.
 *
 */
static int xlnx_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct xlnx_ptp_timer *timer = container_of(ptp, struct xlnx_ptp_timer,
						ptp_clock_info);
	struct timespec64 offset;
	u64 sign = 0;
	s64 cumulative_delta = timer->timeoffset;

	spin_lock(&timer->reg_lock);

	/* Fixed offset between system and port timer */
	if (!timer->use_sys_timer_only)
		delta += timer->static_delay;
	cumulative_delta += delta;
	timer->timeoffset = cumulative_delta;
	if (cumulative_delta < 0) {
		sign = XPTPTIMER_TOD_OFFSET_NEG;
		cumulative_delta = -cumulative_delta;
	}
	offset = ns_to_timespec64(cumulative_delta);
	offset.tv_sec |= sign;

	if (timer->use_sys_timer_only)
		xlnx_tod_offset_write(timer, (const struct timespec64 *)&offset);
	else
		xlnx_port_offset_write(timer, (const struct timespec64 *)&offset);

	spin_unlock(&timer->reg_lock);

	return 0;
}

/**
 * xlnx_ptp_gettime - Get the current time on the hardware clock
 * @ptp: ptp clock structure
 * @ts: timespec64 containing the current TX port timer time.
 * Return: 0 on success
 * Since TX and RX ports are initialized and adjusted simultaneously,
 * they should be the same.
 *
 */
static int xlnx_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct xlnx_ptp_timer *timer = container_of(ptp, struct xlnx_ptp_timer,
						    ptp_clock_info);

	spin_lock(&timer->reg_lock);
	xlnx_tod_read(timer, ts);
	spin_unlock(&timer->reg_lock);

	return 0;
}

/**
 * xlnx_ptp_settime - Set the current time on the hardware clock
 * @ptp: ptp clock structure
 * @ts: timespec64 containing the new time
 * Return: 0 on success
 *
 * The hardware loads the entire new value when a load register is triggered.
 */
static int xlnx_ptp_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	struct xlnx_ptp_timer *timer = container_of(ptp, struct xlnx_ptp_timer,
					ptp_clock_info);

	spin_lock(&timer->reg_lock);
	xlnx_tod_load_write(timer, ts);
	spin_unlock(&timer->reg_lock);

	return 0;
}

static int xlnx_ptp_enable(struct ptp_clock_info *ptp,
			   struct ptp_clock_request *rq, int on)
{
	struct xlnx_ptp_timer *timer = container_of(ptp, struct xlnx_ptp_timer,
			ptp_clock_info);
	u32 data;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		timer->extts_enable = on;

		data = xlnx_ptp_ior(timer, XPTPTIMER_TOD_CONFIG_OFFSET);

		data &= ~(XPTPTIMER_ENABLE_SNAPSHOT);

		if (on)
			data |= XPTPTIMER_ENABLE_SNAPSHOT;

		xlnx_ptp_iow(timer, XPTPTIMER_TOD_CONFIG_OFFSET, data);

		return 0;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static struct ptp_clock_info xlnx_ptp_clock_info = {
	.owner		= THIS_MODULE,
	.name		= "Xilinx Timer",
	.max_adj	= 64000000,	/* Safe max adjutment for clock rate */
	.n_ext_ts	= 1,
	.adjfine	= xlnx_ptp_adjfine,
	.adjtime	= xlnx_ptp_adjtime,
	.gettime64	= xlnx_ptp_gettime,
	.settime64	= xlnx_ptp_settime,
	.enable		= xlnx_ptp_enable,
};

/**
 * xlnx_ptp_timer_isr - Interrupt Service Routine
 * @irq:               IRQ number
 * @priv:              pointer to the timer structure
 * Return:	       IRQ_HANDLED on success. IRQ_NONE on failure
 */
static irqreturn_t xlnx_ptp_timer_isr(int irq, void *priv)
{
	struct xlnx_ptp_timer *timer = priv;
	struct ptp_clock_event event;
	u32 sech, secl, nsec;
	int status;
	u64 sec;

	memset(&event, 0, sizeof(event));
	status = xlnx_ptp_ior(timer, XPTPTIMER_ISR_OFFSET);
	if (status & XPTPTIMER_EXTS_1PPS_INTR_MASK) {
		xlnx_ptp_iow(timer, XPTPTIMER_ISR_OFFSET,
			     XPTPTIMER_EXTS_1PPS_INTR_MASK);
		if (timer->extts_enable) {
			event.type = PTP_CLOCK_EXTTS;
			nsec = xlnx_ptp_ior(timer, XPTPTIMER_SYS_NS_OFFSET);
			sech = xlnx_ptp_ior(timer, XPTPTIMER_SYS_SEC_1_OFFSET);
			secl = xlnx_ptp_ior(timer, XPTPTIMER_SYS_SEC_0_OFFSET);

			sec = (((u64)sech << 32) | secl) & XPTPTIMER_MAX_SEC_MASK;
			event.timestamp = ktime_set(sec, nsec);
			ptp_clock_event(timer->ptp_clock, &event);
		}
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int xlnx_ptp_timer_probe(struct platform_device *pdev)
{
	struct xlnx_ptp_timer *timer;
	struct resource *r_mem;
	int err = 0;
	struct timespec64 ts, tsp;
	u32 nsec, sech, secl;

	if (of_device_is_compatible(pdev->dev.of_node, "xlnx,timer-syncer-1588-1.0")) {
		dev_err(&pdev->dev, "This Beta version is no longer supported. Please upgrade to 2.0 IP version and use xlnx,timer-syncer-1588-2.0 compatible string\n");
		return -EINVAL;
	}

	timer = devm_kzalloc(&pdev->dev, sizeof(*timer), GFP_KERNEL);
	if (!timer)
		return -ENOMEM;

	timer->dev = &pdev->dev;

	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	timer->baseaddr = devm_ioremap_resource(&pdev->dev, r_mem);
	if (IS_ERR(timer->baseaddr)) {
		err = PTR_ERR(timer->baseaddr);
		return err;
	}

	timer->use_sys_timer_only = of_property_read_bool(pdev->dev.of_node,
							  "xlnx,has-timer-syncer");

	timer->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (timer->irq <= 0) {
		dev_warn(&pdev->dev, "could not determine Timer IRQ\n");
	} else {
		/* Enable interrupts */
		err = devm_request_irq(timer->dev, timer->irq,
				       xlnx_ptp_timer_isr,
				       IRQF_SHARED,
				       "xlnx_ptp_timer",
				       timer);
		if (err)
			dev_warn(&pdev->dev, "Failed to request ptp timer irq\n");
	}

	spin_lock_init(&timer->reg_lock);

	timer->ptp_clock_info = xlnx_ptp_clock_info;

	timer->ptp_clock = ptp_clock_register(&timer->ptp_clock_info,
					      &pdev->dev);
	if (IS_ERR(timer->ptp_clock)) {
		err = PTR_ERR(timer->ptp_clock);
		dev_err(&pdev->dev, "Failed to register ptp clock\n");
		return err;
	}

	if (timer->irq > 0)
		/* Enable timer interrupt */
		xlnx_ptp_iow(timer, XPTPTIMER_IER_OFFSET,
			     XPTPTIMER_EXTS_1PPS_INTR_MASK);

	timer->period_0 = TOD_LEGACY_SYS_PERIOD_0;
	timer->period_1 = TOD_LEGACY_SYS_PERIOD_1;

	if (of_device_is_compatible(pdev->dev.of_node, "xlnx,timer-syncer-1588-3.0")) {
		timer->period_0 = TOD_SYS_PERIOD_0;
		timer->period_1 = TOD_SYS_PERIOD_1;
	}

	xlnx_ptp_iow(timer, XPTPTIMER_TOD_CONFIG_OFFSET,
		     XPTPTIMER_CFG_MAIN_TOD_EN | XPTPTIMER_CFG_ENABLE_PORT0);
	/*
	 * TODO: This clock rate should be derived from system design.
	 * This is design specific - for ex. 250MHz port clock rate; period is
	 * 10^9/250 is 4ns. Set port timer PERIOD BEFORE calling settime,
	 * so that the initial LOAD triggers everything together.
	 */
	timer->incr = ((u64)XPTPTIMER_CLOCK_PERIOD << XPTPTIMER_PERIOD_SHIFT);
	xlnx_tod_period_write(timer, timer->incr);
	xlnx_port_period_write(timer, timer->incr);

	/* Initialize current time */
	ts = ns_to_timespec64(ktime_to_ns(ktime_get_real()));
	xlnx_ptp_settime(&timer->ptp_clock_info, &ts);

	/*
	 * A static delay of 7-8 clock cycles is expected between system and
	 * port timer which should be 28-32ns on this system with 250MHz clock.
	 */
	xlnx_ptp_iow(timer, XPTPTIMER_TOD_SNAPSHOT_OFFSET,
		     XPTPTIMER_SNAPSHOT_MASK);
	nsec = xlnx_ptp_ior(timer, XPTPTIMER_SYS_NS_OFFSET);
	sech = xlnx_ptp_ior(timer, XPTPTIMER_SYS_SEC_1_OFFSET);
	secl = xlnx_ptp_ior(timer, XPTPTIMER_SYS_SEC_0_OFFSET);
	ts.tv_nsec = nsec;
	ts.tv_sec = (((u64)sech << 32) | secl) & XPTPTIMER_MAX_SEC_MASK;

	nsec = xlnx_ptp_ior(timer, XPTPTIMER_PORT_TX_NS_SNAP_OFFSET);
	secl = xlnx_ptp_ior(timer, XPTPTIMER_PORT_TX_SEC_0_SNAP_OFFSET);
	sech = xlnx_ptp_ior(timer, XPTPTIMER_PORT_TX_SEC_1_SNAP_OFFSET);
	tsp.tv_nsec = nsec;
	tsp.tv_sec = (((u64)sech << 32) | secl) & XPTPTIMER_MAX_SEC_MASK;

	ts = timespec64_sub(ts, tsp);
	timer->static_delay = timespec64_to_ns(&ts);
	dev_dbg(&pdev->dev, "Static delay %d\n", timer->static_delay);

	platform_set_drvdata(pdev, timer);

	timer->phc_index = ptp_clock_index(timer->ptp_clock);
	dev_info(&pdev->dev, "Xilinx PTP timer driver probed\n");

	return 0;
}

static void xlnx_ptp_timer_remove(struct platform_device *pdev)
{
	struct xlnx_ptp_timer *timer = platform_get_drvdata(pdev);

	if (timer->irq > 0)
		/* Disable timer interrupt */
		xlnx_ptp_iow(timer, XPTPTIMER_IER_OFFSET,
			     (u32)~(XPTPTIMER_EXTS_1PPS_INTR_MASK));
	ptp_clock_unregister(timer->ptp_clock);
}

static const struct of_device_id timer_1588_of_match[] = {
	{ .compatible = "xlnx,timer-syncer-1588-1.0", },
	{ .compatible = "xlnx,timer-syncer-1588-2.0", },
	{ .compatible = "xlnx,timer-syncer-1588-3.0", },
		{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, timer_1588_of_match);

static struct platform_driver xlnx_ptp_timer_driver = {
	.probe		= xlnx_ptp_timer_probe,
	.remove		= xlnx_ptp_timer_remove,
	.driver		= {
		.name = "xlnx_ptp_timer",
		.of_match_table = timer_1588_of_match,
	},
};

module_platform_driver(xlnx_ptp_timer_driver);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("PTP Timer Syncer driver");
MODULE_LICENSE("GPL");
