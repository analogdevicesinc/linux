
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gptimer driver for providing system clock source, clock event source,
 * and generic counters for use in userspace
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 */

#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/counter.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>

/*
 * Shared gptimers registers
 */
#define GPTIMER_RUN								0x04
#define GPTIMER_RUN_SET							0x08
#define GPTIMER_RUN_CLR							0x0C
#define GPTIMER_STOP_CFG						0x10
#define GPTIMER_STOP_CFG_SET					0x14
#define GPTIMER_STOP_CFG_CLR					0x18
#define GPTIMER_DATA_IMSK						0x1C
#define GPTIMER_STAT_IMSK						0x20
#define GPTIMER_TRG_MSK							0x24
#define GPTIMER_TRG_IE							0x28
#define GPTIMER_DATA_ILAT						0x2C
#define GPTIMER_STAT_ILAT						0x30
#define GPTIMER_ERR_TYPE						0x34
#define GPTIMER_BCAST_PER						0x38
#define GPTIMER_BCAST_WID						0x3C
#define GPTIMER_BCAST_DLY						0x40

/**
 * Per-timer registers starting at offset
 */
#define GPTIMER_CFG_OFF							0x00
#define GPTIMER_CNT_OFF							0x04
#define GPTIMER_PER_OFF							0x08
#define GPTIMER_WID_OFF							0x0C
#define GPTIMER_DLY_OFF							0x10

/*
 * Timer Configuration Register Bits
 */
#define TIMER_EMU_RUN       0x8000
#define TIMER_BPER_EN       0x4000
#define TIMER_BWID_EN       0x2000
#define TIMER_BDLY_EN       0x1000
#define TIMER_OUT_DIS       0x0800
#define TIMER_TIN_SEL       0x0400
#define TIMER_CLK_SEL       0x0300
#define TIMER_CLK_SCLK      0x0000
#define TIMER_CLK_ALT_CLK0  0x0100
#define TIMER_CLK_ALT_CLK1  0x0300
#define TIMER_PULSE_HI      0x0080
#define TIMER_SLAVE_TRIG    0x0040
#define TIMER_IRQ_MODE      0x0030
#define TIMER_IRQ_ACT_EDGE  0x0000
#define TIMER_IRQ_DLY       0x0010
#define TIMER_IRQ_WID_DLY   0x0020
#define TIMER_IRQ_PER       0x0030
#define TIMER_MODE          0x000f
#define TIMER_MODE_WDOG_P   0x0008
#define TIMER_MODE_WDOG_W   0x0009
#define TIMER_MODE_PWM_CONT 0x000c
#define TIMER_MODE_PWM      0x000d
#define TIMER_MODE_WDTH     0x000a
#define TIMER_MODE_WDTH_D   0x000b
#define TIMER_MODE_EXT_CLK  0x000e
#define TIMER_MODE_PININT   0x000f

struct sc5xx_gptimer {
	int id;
	int irq;
	void __iomem *io_base;
};

struct gptimer_counter {
	struct counter_device counter;
};

struct clocksource_gptimer {
	struct clocksource cs;
	struct sc5xx_gptimer *timer;
};

struct clockevent_gptimer {
	struct clock_event_device evt;
	struct sc5xx_gptimer *timer;
};

struct sc5xx_gptimer_controller {
	void __iomem *base;
	struct clk *clk;
	struct clocksource_gptimer *cs;
	struct clockevent_gptimer *cevt;
	struct sc5xx_gptimer *timers;
	size_t num_timers;
};

static struct sc5xx_gptimer_controller gptimer_controller = { 0x00 };

static struct clockevent_gptimer *to_clockevent_gptimer(struct
							clock_event_device
							*evt)
{
	return container_of(evt, struct clockevent_gptimer, evt);
}

/**
 * Per gptimer accessors
 */
static void set_gptimer_period(struct sc5xx_gptimer *timer,
			       uint32_t period)
{
	writel(period, timer->io_base + GPTIMER_PER_OFF);
}

static void set_gptimer_pwidth(struct sc5xx_gptimer *timer, uint32_t value)
{
	writel(value, timer->io_base + GPTIMER_WID_OFF);
}

static void set_gptimer_delay(struct sc5xx_gptimer *timer, uint32_t value)
{
	writel(value, timer->io_base + GPTIMER_DLY_OFF);
}

static void set_gptimer_config(struct sc5xx_gptimer *timer,
			       uint16_t config)
{
	writew(config, timer->io_base + GPTIMER_CFG_OFF);
}

static uint32_t get_gptimer_count(struct sc5xx_gptimer *timer)
{
	return readl(timer->io_base + GPTIMER_CNT_OFF);
}

/**
 * Accessors that redirect to the shared registers
 */
static void gptimer_enable(struct sc5xx_gptimer *timer)
{
	writew(1 << timer->id, gptimer_controller.base + GPTIMER_RUN_SET);
}

static void gptimer_disable(struct sc5xx_gptimer *timer)
{
	writew(1 << timer->id,
	       gptimer_controller.base + GPTIMER_STOP_CFG_SET);
	writew(1 << timer->id, gptimer_controller.base + GPTIMER_RUN_CLR);
}

static void gptimer_clear_interrupt(struct sc5xx_gptimer *timer)
{
	writew(1 << timer->id,
	       gptimer_controller.base + GPTIMER_DATA_ILAT);
}

static bool gptimer_is_running(struct sc5xx_gptimer *timer)
{
	u32 stat = readw(gptimer_controller.base + GPTIMER_RUN);
	u32 check = 1 << timer->id;

	return (stat & check) == check;
}

/**
 * Scheduler/clocksource functions
 */
static u64 read_cs_gptimer(struct clocksource *cs)
{
	struct clocksource_gptimer *gp =
	    container_of(cs, struct clocksource_gptimer, cs);

	return (u64) get_gptimer_count(gp->timer);
}

static u64 notrace read_sched_gptimer(void)
{
	return (u64) get_gptimer_count(gptimer_controller.cs->timer);
}

/**
 * Clockevent functions
 */
static int gptimer_set_next_event(unsigned long cycles,
				  struct clock_event_device *evt)
{
	struct clockevent_gptimer *cevt = to_clockevent_gptimer(evt);

	/* it starts counting three SCLK cycles after the TIMENx bit is set */
	set_gptimer_pwidth(cevt->timer, 1);
	set_gptimer_delay(cevt->timer, cycles - 3);

	gptimer_enable(cevt->timer);
	return 0;
}

static int gptimer_set_state_periodic(struct clock_event_device *evt)
{
	struct clockevent_gptimer *cevt = to_clockevent_gptimer(evt);
	unsigned long rate = clk_get_rate(gptimer_controller.clk);

	gptimer_disable(cevt->timer);
	set_gptimer_config(cevt->timer,
			   TIMER_OUT_DIS | TIMER_MODE_PWM_CONT |
			   TIMER_PULSE_HI | TIMER_IRQ_PER);

	set_gptimer_period(cevt->timer, rate / HZ);
	set_gptimer_pwidth(cevt->timer, rate / HZ - 1);

	gptimer_enable(cevt->timer);
	return 0;
}

static int gptimer_set_state_oneshot(struct clock_event_device *evt)
{
	struct clockevent_gptimer *cevt = to_clockevent_gptimer(evt);

	gptimer_disable(cevt->timer);
	set_gptimer_config(cevt->timer, TIMER_OUT_DIS | TIMER_MODE_PWM |
			   TIMER_PULSE_HI | TIMER_IRQ_DLY);

	/* gptimer_set_next_event will configure the period and delay */
	return 0;
}

static int gptimer_set_state_shutdown(struct clock_event_device *evt)
{
	struct clockevent_gptimer *cevt = to_clockevent_gptimer(evt);

	gptimer_disable(cevt->timer);
	return 0;
}

static irqreturn_t cevt_gptimer_handler(int irq, void *dev)
{
	struct clockevent_gptimer *cevt = dev;

	gptimer_clear_interrupt(cevt->timer);
	cevt->evt.event_handler(&cevt->evt);
	return IRQ_HANDLED;
}

static int gptimer_counter_count_read(struct counter_device *counter,
				      struct counter_count *count,
				      u64 * val)
{
	uint32_t id = count->id;
	struct sc5xx_gptimer *timer = &gptimer_controller.timers[id];
	u64 timer_count = get_gptimer_count(timer);

	*val = timer_count;
	return 0;
}

/**
 * Counter implementation
 */
static struct counter_ops gptimer_counter_ops = {
	.count_read = gptimer_counter_count_read,
};

/**
 * Initializes an individual gptimer belonging to the controller
 * @todo resource cleanup in error paths
 */
static int __init sc5xx_gptimer_init(struct device_node *np,
				     struct sc5xx_gptimer *timer)
{
	int irq, id;
	u32 offset;
	int ret;
	struct clk *clk = gptimer_controller.clk;

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("%s: Unable to find irq for gptimer %pOFn\n",
		       __func__, np);
		return -ENODEV;
	}

	ret = of_property_read_s32(np, "reg", &id);
	if (ret) {
		pr_err
		    ("%s: Missing reg property containing timer id for gptimer %pOFn\n",
		     __func__, np);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "adi,offset", &offset);
	if (ret) {
		pr_err("%s: Missing adi,offset for gptimer %pOFn\n",
		       __func__, np);
		return -ENODEV;
	}

	timer->id = id;
	timer->irq = irq;
	timer->io_base = gptimer_controller.base + offset;

	/*
	 * @todo add period or other timing options to dts?
	 */
	if (!gptimer_is_running(timer) ||
	    of_property_read_bool(np, "adi,reset-timer")) {
		gptimer_disable(timer);

		set_gptimer_config(timer,
				   TIMER_OUT_DIS | TIMER_MODE_PWM_CONT |
				   TIMER_PULSE_HI | TIMER_IRQ_PER);
		set_gptimer_period(timer, 0xFFFFFFFF);
		set_gptimer_pwidth(timer, 0xFFFFFFFE);

		gptimer_enable(timer);
	}

	if (of_property_read_bool(np, "adi,is-clocksource")) {
		struct clocksource_gptimer *cs;

		cs = kzalloc(sizeof(*cs), GFP_KERNEL);
		if (!cs)
			return -ENOMEM;

		cs->cs.name = "cs_adi_gptimer";
		cs->cs.rating = 350;
		cs->cs.read = read_cs_gptimer;
		cs->cs.mask = CLOCKSOURCE_MASK(32);
		cs->cs.flags = CLOCK_SOURCE_IS_CONTINUOUS;
		cs->timer = timer;
		gptimer_controller.cs = cs;

		ret = clocksource_register_hz(&cs->cs, clk_get_rate(clk));
		if (ret) {
			pr_err("%s: failed to register clocksource = %d\n",
			       __func__, ret);
			return ret;
		}

		sched_clock_register(read_sched_gptimer, 32,
				     clk_get_rate(clk));

		// optionally, register_current_timer_delay if we also want to
		// use this gptimer for timer-based delays instead of while loops, but
		// this may not be a good idea on slower platforms
	}

	if (of_property_read_bool(np, "adi,is-clockevent")) {
		struct clockevent_gptimer *cevt;
		u16 imsk;

		cevt = kzalloc(sizeof(*cevt), GFP_KERNEL);
		if (!cevt)
			return -ENOMEM;

		cevt->evt = (struct clock_event_device) {
			.name = "cevt_adi_gptimer",
			.features =
			    CLOCK_EVT_FEAT_PERIODIC |
			    CLOCK_EVT_FEAT_ONESHOT,
			.rating = 300,
			.shift = 32,
			.cpumask = cpumask_of(0),
			.set_next_event = gptimer_set_next_event,
			.set_state_periodic = gptimer_set_state_periodic,
			.set_state_oneshot = gptimer_set_state_oneshot,
			.set_state_shutdown = gptimer_set_state_shutdown,
		};

		cevt->timer = timer;
		gptimer_controller.cevt = cevt;

		imsk = readw(gptimer_controller.base + GPTIMER_DATA_IMSK);
		imsk &= ~(1 << timer->id);
		writew(imsk, gptimer_controller.base + GPTIMER_DATA_IMSK);

		ret =
		    request_irq(irq, cevt_gptimer_handler,
				IRQF_TIMER | IRQF_IRQPOLL,
				"sc5xx gptimer clockevent", cevt);
		if (ret) {
			pr_err
			    ("%s: Could not register clockevent handler\n",
			     __func__);
			return ret;
		}

		clockevents_config_and_register(&cevt->evt,
						clk_get_rate(clk), 100,
						-1);
	}

	return 0;
}

/**
 * Map master gptimers module, which finds a clocksource and clockevent child
 * and registers them as such with the system. The other gptimers are registers
 * as counters with the counter framework
 *
 * @todo clear up resource leaks in exit paths
 */
static int __init sc5xx_gptimer_controller_init(struct device_node *np)
{
	struct device_node *timer_np;
	void __iomem *base;
	struct clk *clk;
	int ret;
	int i, n;

	if (gptimer_controller.base) {
		pr_err
		    ("%s: Tried to initialize a second gptimer controller; check your device tree\n",
		     __func__);
		return -EINVAL;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%s: Unable to map gptimer registers\n", __func__);
		return -ENODEV;
	}

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("%s: could not find sclk0_0 = %ld\n", __func__,
		       PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("%s: sclk0_0 clock enable failed = %d\n", __func__,
		       ret);
		return ret;
	}

	gptimer_controller.clk = clk;
	gptimer_controller.base = base;

	n = of_get_child_count(np);
	gptimer_controller.num_timers = n;
	gptimer_controller.timers =
		kcalloc(n, sizeof(*gptimer_controller.timers), GFP_KERNEL);

	if (!gptimer_controller.timers) {
		pr_err("%s: Unable to allocate memory for timers\n",
		       __func__);
		return -ENOMEM;
	}

	i = 0;
	for_each_child_of_node(np, timer_np) {
		ret =
		    sc5xx_gptimer_init(timer_np,
				       &gptimer_controller.timers[i]);
		if (ret) {
			of_node_put(timer_np);
			return ret;
		}

		i += 1;
	}

	return 0;
}

TIMER_OF_DECLARE(sc5xx_gptimers, "adi,sc5xx-gptimers",
		 sc5xx_gptimer_controller_init);

static int gptimer_counter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct counter_count *adi_counts;
	struct sc5xx_gptimer_controller *priv;
	struct counter_device *counter;
	uint32_t i;
	int ret;

	adi_counts =
	    devm_kcalloc(dev, gptimer_controller.num_timers,
			 sizeof(*adi_counts), GFP_KERNEL);
	if (!adi_counts) {
		return -ENOMEM;
	}

	
	counter = devm_counter_alloc(dev, sizeof(*priv));
	if (!counter)
		return -ENOMEM;

	priv = counter_priv(counter);
	

	for (i = 0; i < gptimer_controller.num_timers; ++i) {
		adi_counts[i].name =
		    kasprintf(GFP_KERNEL, "gptimer_counter%d", i);
	}

	priv->clk = gptimer_controller.clk;

	counter->name = dev_name(dev);
	counter->parent = dev;
	counter->ops = &gptimer_counter_ops;
	counter->counts = adi_counts;
	counter->num_counts = gptimer_controller.num_timers;

	/* Register Counter device */
	ret = devm_counter_add(dev, counter);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to add counter\n");
	}
	return ret;

	
}

static const struct of_device_id adsp_gptimer_counter_match[] = {
	{.compatible = "adi,gptimer-counter" },
	{ },
};

MODULE_DEVICE_TABLE(of, adsp_gptimer_counter_match);

static struct platform_driver gptimer_counter_driver = {
	.probe = gptimer_counter_probe,
	.driver = {
		   .name = "adsp-gptimer-counter",
		   .of_match_table = adsp_gptimer_counter_match,
		    },
};

module_platform_driver(gptimer_counter_driver);
