// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC Timer driver
 * Copyright 2023 NXP
 * Copyright (C) 2023 Wei Fang <wei.fang@nxp.com>
 */
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/fsl/netc_global.h>

#define NETC_TMR_CTRL			0x0080
#define  TMR_CTRL_CK_SEL		GENMASK(1, 0)
#define  TMR_CTRL_TE			BIT(2)
#define  TMR_COMP_MODE			BIT(15)
#define  TMR_CTRL_TCLK_PERIOD		GENMASK(25, 16)
#define  TMR_CTRL_FS			BIT(28)
#define  TMR_ALARM1P			BIT(31)

#define NETC_TMR_TEVENT			0x0084
#define  TMR_TEVNET_PPEN(a)		BIT(7 - (a))
#define  TMR_TEVENT_PPEN_ALL		GENMASK(7, 5)
#define  TMR_TEVENT_ALM1EN		BIT(16)
#define  TMR_TEVENT_ALM2EN		BIT(17)
#define  TMR_TEVENT_ETS1_THREN		BIT(20)
#define  TMR_TEVENT_ETS2_THREN		BIT(21)
#define  TMR_TEVENT_ETS1EN		BIT(24)
#define  TMR_TEVENT_ETS2EN		BIT(25)
#define  TMR_TEVENT_ETS1_OVEN		BIT(28)
#define  TMR_TEVENT_ETS2_OVEN		BIT(29)

#define NETC_TMR_TEMASK			0x0088
#define NETC_TMR_STAT			0x0094
#define NETC_TMR_CNT_L			0x0098
#define NETC_TMR_CNT_H			0x009c
#define NETC_TMR_ADD			0x00a0
#define NETC_TMR_ACC			0x00a4
#define NETC_TMR_PRSC			0x00a8
#define NETC_TMR_ECTRL			0x00ac
#define NETC_TMR_OFF_L			0x00b0
#define NETC_TMR_OFF_H			0x00b4

/* a = 0 or 1, a = 0 indicates TMR_ALARM1, a = 1 indicates TMR_ALARM2 */
#define NETC_TMR_ALARM_L(a)		(0x00b8 + (a) * 8)
#define NETC_TMR_ALARM_H(a)		(0x00bc + (a) * 8)

#define NETC_TMR_ALARM_CTRL		0x00cc
#define  ALARM_CTRL_PW(a)		(GENMASK(4, 0) << (a) * 8)
#define  ALARM_CTRL_PG(a)		(BIT(7) << (a) * 8)

/* a = 0, 1, 2. a = 0 indicates TMR_FIPER1, a = 1 indicates TMR_FIPER2,
 * a = 2 indicates TMR_FIPER3.
 */
#define NETC_TMR_FIPER(a)		(0x00d0 + (a) * 4)

#define NETC_TMR_FIPER_CTRL		0x00dc
#define  FIPER_CTRL_PW(a)		(GENMASK(4, 0) << (a) * 8)
#define  FIPER_CTRL_PG(a)		(BIT(6) << (a) * 8)
#define  FIPER_CTRL_DIS(a)		(BIT(7) << (a) * 8)

#define NETC_TMR_ETTS1_L		0x00e0
#define NETC_TMR_ETTS1_H		0x00e4
#define NETC_TMR_ETTS2_L		0x00e8
#define NETC_TMR_ETTS2_H		0x00ec
#define NETC_TMR_PARAM			0x00f8

#define NETC_TMR_REGS_BAR		0
#define NETC_TMR_FIPER_NUM		3
#define NETC_TMR_DEFAULT_PRSC		2
#define NETC_TMR_DEFAULT_ALARM		0xffffffffffffffffULL
#define NETC_TMR_DEFAULT_FIPER		0xffffffff
#define NETC_TMR_PRSC_OCK_MAX		0xfffe

/* 1588 timer reference clock source select */
#define NETC_TMR_CCM_TIMER1		0 /* enet_timer1_clk_root, from CCM */
#define NETC_TMR_SYSTEM_CLK		1 /* enet_clk_root/2, from CCM */
#define NETC_TMR_EXT_OSC		2 /* tmr_1588_clk, from IO pins */

#define NETC_TMR_SYSCLK_RATE		333333333UL

#define NETC_TMR_FIPER_PW		0x1f

#define netc_timer_rd(p, o)		netc_read((p)->base + (o))
#define netc_timer_wr(p, o, v)		netc_write((p)->base + (o), v)

struct netc_timer {
	void __iomem *base;
	struct device *dev;
	struct pci_dev *pci_dev;
	int irq;
	char irq_name[64];
	spinlock_t lock; /* protect regs */

	struct ptp_clock *clock;
	struct ptp_clock_info caps;
	int phc_index;
	struct clk *src_clk;
	u32 clk_select;
	u32 clk_freq;
	u32 period_int;
	/* fractional part of clock period * BIT(32) */
	u32 period_frac;
	/* High 32 bits are the integer part, low 32 bits
	 * are the fractional part
	 */
	u64 base_period;
	u32 oclk_prsc; /* must be an even value */
	u32 fiper[NETC_TMR_FIPER_NUM];
};

#define ptp_to_netc_timer(ptp)		container_of((ptp), struct netc_timer, caps)

static u64 netc_timer_cnt_read(struct netc_timer *priv)
{
	u32 tmr_cnt_l, tmr_cnt_h;
	u64 ns;

	/* The user must read the TMR_CNC_L register first to get
	 * correct 64-bit TMR_CNT_H/L counter values.
	 */
	tmr_cnt_l = netc_timer_rd(priv, NETC_TMR_CNT_L);
	tmr_cnt_h = netc_timer_rd(priv, NETC_TMR_CNT_H);
	ns = (((u64)tmr_cnt_h) << 32) | tmr_cnt_l;

	return ns;
}

static void netc_timer_cnt_write(struct netc_timer *priv, u64 ns)
{
	u32 tmr_cnt_h = upper_32_bits(ns);
	u32 tmr_cnt_l = lower_32_bits(ns);

	/* The user must write to TMR_CNT_L register first. */
	netc_timer_wr(priv, NETC_TMR_CNT_L, tmr_cnt_l);
	netc_timer_wr(priv, NETC_TMR_CNT_H, tmr_cnt_h);
}

static void netc_timer_alarm_write(struct netc_timer *priv,
				   u64 alarm, int index)
{
	u32 alarm_h = upper_32_bits(alarm);
	u32 alarm_l = lower_32_bits(alarm);

	netc_timer_wr(priv, NETC_TMR_ALARM_L(index), alarm_l);
	netc_timer_wr(priv, NETC_TMR_ALARM_H(index), alarm_h);
}

static void netc_timer_set_oclk_prsc(struct netc_timer *priv, u32 oclk_prsc)
{
	if (oclk_prsc < NETC_TMR_PRSC_OCK_MAX) {
		if (oclk_prsc % 2 != 0)
			oclk_prsc++;
	} else {
		oclk_prsc = NETC_TMR_PRSC_OCK_MAX;
	}
	priv->oclk_prsc = oclk_prsc;

	if (oclk_prsc == netc_timer_rd(priv, NETC_TMR_PRSC))
		return;

	netc_timer_wr(priv, NETC_TMR_PRSC, priv->oclk_prsc);
}

static u32 netc_timer_calculate_fiper_pulse_width(struct netc_timer *priv,
						  u32 fiper)
{
	u32 oclk_prsc = NETC_TMR_DEFAULT_PRSC;
	u64 pw;

	/* Set the FIPER pulse width to half FIPER interval by default.
	 * pulse_width = (fiper / 2) / TMR_GCLK_period,
	 * TMR_GCLK_period = NSEC_PER_SEC / TMR_GCLK_freq,
	 * TMR_GCLK_freq = (clk_freq / oclk_prsc) MHz,
	 * so pulse_width = fiper * clk_freq / (2 * NSEC_PER_SEC * oclk_prsc).
	 *
	 * The oclk_prsc value needs to be an even number, so here we use its
	 * default value NETC_TMR_DEFAULT_PRSC to calculate the pw. If pw
	 * exceeds the maximum value, then update the oclk_prsc.
	 */
	pw = (u64)fiper * priv->clk_freq;
	/* 2 * NSEC_PER_SEC * oclk_prsc = 4000000000UL */
	pw = div_u64(pw, 4000000000UL);

	/* The FIPER_PW field only has 5 bits, need to update oclk_prsc */
	if (pw > NETC_TMR_FIPER_PW) {
		oclk_prsc = div_u64(pw, NETC_TMR_FIPER_PW) * oclk_prsc;
		pw = NETC_TMR_FIPER_PW;
	}
	netc_timer_set_oclk_prsc(priv, oclk_prsc);

	return pw;
}

static void netc_timer_adjust_period(struct netc_timer *priv, u64 period)
{
	u32 period_frac = lower_32_bits(period);
	u32 period_int = upper_32_bits(period);
	u32 tmr_ctrl, old_tmr_ctrl;

	guard(spinlock_irqsave)(&priv->lock);
	old_tmr_ctrl = netc_timer_rd(priv, NETC_TMR_CTRL);
	tmr_ctrl = u32_replace_bits(old_tmr_ctrl, period_int,
				    TMR_CTRL_TCLK_PERIOD);
	if (tmr_ctrl != old_tmr_ctrl)
		netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);

	netc_timer_wr(priv, NETC_TMR_ADD, period_frac);
}

static irqreturn_t netc_timer_isr(int irq, void *data)
{
	struct netc_timer *priv = data;
	struct ptp_clock_event event;
	u32 tmr_event, tmr_emask;

	guard(spinlock_irqsave)(&priv->lock);

	tmr_event = netc_timer_rd(priv, NETC_TMR_TEVENT);
	tmr_emask = netc_timer_rd(priv, NETC_TMR_TEMASK);

	/* Clear interrupts status */
	netc_timer_wr(priv, NETC_TMR_TEVENT, tmr_event);

	tmr_event &= tmr_emask;
	if (tmr_event & TMR_TEVENT_PPEN_ALL) {
		event.type = PTP_CLOCK_PPS;
		ptp_clock_event(priv->clock, &event);
	}

	if (tmr_event & TMR_TEVENT_ALM1EN) {
		tmr_emask &= ~TMR_TEVENT_ALM1EN;

		netc_timer_wr(priv, NETC_TMR_TEMASK, tmr_emask);
		netc_timer_alarm_write(priv, NETC_TMR_DEFAULT_ALARM, 0);
	}

	return IRQ_HANDLED;
}

/* ppm: parts per million, ppb: parts per billion */
static int netc_timer_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 new_period;

	if (!scaled_ppm)
		return 0;

	new_period = adjust_by_scaled_ppm(priv->base_period, scaled_ppm);
	netc_timer_adjust_period(priv, new_period);

	return 0;
}

static int netc_timer_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 adj_ns = abs(delta);
	u64 ns;

	guard(spinlock_irqsave)(&priv->lock);

	ns = netc_timer_cnt_read(priv);
	if (delta < 0)
		ns -= adj_ns;
	else
		ns += adj_ns;

	netc_timer_cnt_write(priv, ns);

	return 0;
}

static int netc_timer_gettimex64(struct ptp_clock_info *ptp, struct timespec64 *ts,
				 struct ptp_system_timestamp *sts)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 ns;

	scoped_guard(spinlock_irqsave, &priv->lock) {
		ptp_read_system_prets(sts);
		ns = netc_timer_cnt_read(priv);
		ptp_read_system_postts(sts);
	}

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int netc_timer_settime64(struct ptp_clock_info *ptp,
				const struct timespec64 *ts)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 ns = timespec64_to_ns(ts);

	guard(spinlock_irqsave)(&priv->lock);

	netc_timer_cnt_write(priv, ns);

	return 0;
}

static int netc_timer_enable_pps(struct netc_timer *priv,
				 struct ptp_clock_request *rq, int on)
{
	u32 tmr_emask, fiper, fiper_ctrl, fiper_pw;

	guard(spinlock_irqsave)(&priv->lock);

	tmr_emask = netc_timer_rd(priv, NETC_TMR_TEMASK);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);

	if (on) {
		fiper = div_u64(NSEC_PER_SEC, priv->period_int) - 1;
		fiper = fiper * priv->period_int;
		fiper_pw = netc_timer_calculate_fiper_pulse_width(priv, fiper);
		fiper_ctrl &= ~(FIPER_CTRL_DIS(0) | FIPER_CTRL_PW(0));
		fiper_ctrl |= fiper_pw & FIPER_CTRL_PW(0);
		tmr_emask |= TMR_TEVNET_PPEN(0);
	} else {
		fiper = NETC_TMR_DEFAULT_FIPER;
		tmr_emask &= ~TMR_TEVNET_PPEN(0);
		fiper_ctrl |= FIPER_CTRL_DIS(0);
	}

	netc_timer_wr(priv, NETC_TMR_TEMASK, tmr_emask);
	netc_timer_wr(priv, NETC_TMR_FIPER(0), fiper);
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);

	return 0;
}

static int net_timer_enable_perout(struct netc_timer *priv,
				   struct ptp_clock_request *rq, int on)
{
	u32 tmr_emask, tmr_ctrl, fiper, fiper_ctrl;
	struct timespec64 period, stime;
	u64 alarm, period_ns, cur_time;
	u32 channel, fiper_pw;

	if (rq->perout.flags)
		return -EOPNOTSUPP;

	channel = rq->perout.index;
	if (channel >= NETC_TMR_FIPER_NUM)
		return -EINVAL;

	guard(spinlock_irqsave)(&priv->lock);

	tmr_ctrl = netc_timer_rd(priv, NETC_TMR_CTRL);
	tmr_emask = netc_timer_rd(priv, NETC_TMR_TEMASK);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	if (!on) {
		tmr_emask &= ~(TMR_TEVNET_PPEN(channel) |
			     TMR_TEVENT_ALM1EN);
		tmr_ctrl &= ~TMR_CTRL_FS;
		alarm = NETC_TMR_DEFAULT_ALARM;
		fiper = NETC_TMR_DEFAULT_FIPER;
		fiper_ctrl |= FIPER_CTRL_DIS(channel);
	} else {
		period.tv_sec = rq->perout.period.sec;
		period.tv_nsec = rq->perout.period.nsec;
		period_ns = timespec64_to_ns(&period);
		stime.tv_sec = rq->perout.start.sec;
		stime.tv_nsec = rq->perout.start.nsec;

		tmr_emask |= TMR_TEVNET_PPEN(channel) | TMR_TEVENT_ALM1EN;
		tmr_ctrl |= TMR_CTRL_FS;
		tmr_ctrl &= ~TMR_ALARM1P;

		/* fiper is set to desired FIPER interval in ns - TCLK_PERIOD
		 * and should be an integer multiple of TCLK_PERIOD.
		 */
		fiper = div_u64(period_ns, priv->period_int) - 1;
		fiper = fiper * priv->period_int;
		fiper_pw = netc_timer_calculate_fiper_pulse_width(priv, fiper);
		if (fiper_pw == 0) {
			dev_err(priv->dev, "The setting period is too small!\n");
			return -EINVAL;
		}

		fiper_ctrl &= ~(FIPER_CTRL_DIS(channel) | FIPER_CTRL_PW(channel));
		fiper_ctrl |= (fiper_pw << 8 * channel) & FIPER_CTRL_PW(channel);

		/* alarm must be an integer multiple of TCLK_PERIOD in order
		 * to get correct result. In addtion, In FS mode the alarm
		 * trigger is used as an indication to the fiper start down
		 * counting. Only TMR_ALARM1 supports this mode.
		 */
		alarm = timespec64_to_ns(&stime);
		cur_time = netc_timer_cnt_read(priv);
		if (cur_time >= alarm) {
			dev_err(priv->dev, "Start time must greater than current time!\n");

			return -EINVAL;
		}

		alarm = div_u64(alarm, priv->period_int);
		alarm = alarm * priv->period_int;
	}

	netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);
	netc_timer_wr(priv, NETC_TMR_TEMASK, tmr_emask);
	netc_timer_wr(priv, NETC_TMR_FIPER(channel), fiper);
	netc_timer_alarm_write(priv, alarm, 0);
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);

	return 0;
}

static int netc_timer_enable(struct ptp_clock_info *ptp,
			     struct ptp_clock_request *rq, int on)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		return net_timer_enable_perout(priv, rq, on);
	case PTP_CLK_REQ_PPS:
		return netc_timer_enable_pps(priv, rq, on);
	case PTP_CLK_REQ_EXTTS:
		/* TODO */
	default:
		return -EOPNOTSUPP;
	}
}

static const struct ptp_clock_info netc_timer_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "NETC Timer PTP clock",
	.max_adj	= 500000000,
	.n_alarm	= 2,
	.n_ext_ts	= 2,
	.n_per_out	= 3,
	.n_pins		= 0,
	.pps		= 1,
	.adjfine	= netc_timer_adjfine,
	.adjtime	= netc_timer_adjtime,
	.gettimex64	= netc_timer_gettimex64,
	.settime64	= netc_timer_settime64,
	.enable		= netc_timer_enable,
};

static int netc_timer_get_source_clk(struct netc_timer *priv)
{
	struct device_node *node = priv->dev->of_node;
	struct device *dev = priv->dev;
	const char *clk_name = NULL;
	u64 ns = NSEC_PER_SEC;

	of_property_read_string(node, "clock-names", &clk_name);
	if (clk_name) {
		priv->src_clk = devm_clk_get_optional(dev, clk_name);
		if (IS_ERR_OR_NULL(priv->src_clk)) {
			dev_warn(dev, "Failed to get source clock\n");
			priv->src_clk = NULL;
			goto default_to_system_clk;
		}

		priv->clk_freq = clk_get_rate(priv->src_clk);
		if (!strcmp(clk_name, "netc_clk_root")) {
			/* The system clock should be divided by 2 */
			priv->clk_freq /= 2;
			priv->clk_select = NETC_TMR_SYSTEM_CLK;
		} else if (!strcmp(clk_name, "ccm_timer1_clk")) {
			priv->clk_select = NETC_TMR_CCM_TIMER1;
		} else if (!strcmp(clk_name, "tmr_1588_clk")) {
			priv->clk_select = NETC_TMR_EXT_OSC;
		} else {
			goto default_to_system_clk;
		}

		goto cal_clk_period;
	}

default_to_system_clk:
	priv->clk_select = NETC_TMR_SYSTEM_CLK;
	priv->clk_freq = NETC_TMR_SYSCLK_RATE;

cal_clk_period:
	priv->base_period = div_u64(ns << 32, priv->clk_freq);
	priv->period_int = upper_32_bits(priv->base_period);
	priv->period_frac = lower_32_bits(priv->base_period);

	return 0;
}

int netc_timer_get_phc_index(int domain, unsigned int bus, unsigned int devfn)
{
	struct pci_dev *timer_pdev;
	struct netc_timer *priv;

	timer_pdev = pci_get_domain_bus_and_slot(domain, bus, devfn);
	if (!timer_pdev)
		return -EINVAL;

	priv = pci_get_drvdata(timer_pdev);
	if (!priv)
		return -EINVAL;

	return priv->phc_index;
}
EXPORT_SYMBOL_GPL(netc_timer_get_phc_index);

static int netc_timer_init(struct netc_timer *priv)
{
	u32 tmr_ctrl, alarm_ctrl, fiper_ctrl;
	struct timespec64 now;
	u64 ns;
	int i;

	priv->caps = netc_timer_ptp_caps;
	priv->oclk_prsc = NETC_TMR_DEFAULT_PRSC;

	alarm_ctrl = ALARM_CTRL_PG(0) | ALARM_CTRL_PG(1);

	spin_lock_init(&priv->lock);

	guard(spinlock_irqsave)(&priv->lock);
	/* Software must enable timer first and the clock selected must be
	 * active, otherwise, the registers which are in the timer clock
	 * domain are not accesdible.
	 */
	tmr_ctrl = (priv->clk_select & TMR_CTRL_CK_SEL) | TMR_CTRL_TE;
	netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);

	/* Output FIPER pulse clock (TMR_GCLK) is generated by dividing the
	 * input clock of Timer by priv->oclk_prsc. For example, if input
	 * clock of Timer is 200MHz, and priv->oclk_prsc is 2, then TMR_GCLK
	 * is 100MHz.
	 */
	netc_timer_wr(priv, NETC_TMR_PRSC, priv->oclk_prsc);
	netc_timer_wr(priv, NETC_TMR_ALARM_CTRL, alarm_ctrl);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	for (i = 0; i < NETC_TMR_FIPER_NUM; i++)
		fiper_ctrl |= FIPER_CTRL_DIS(i);
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);

	ktime_get_real_ts64(&now);
	ns = timespec64_to_ns(&now);
	netc_timer_cnt_write(priv, ns);

	/* Allow atomic writes to TCLK_PERIOD and TMR_ADD,  An update
	 * to TCLK_PERIOD doesn't take effect until TMR_ADD is written.
	 */
	tmr_ctrl |= ((priv->period_int << 16) & TMR_CTRL_TCLK_PERIOD) |
		    TMR_COMP_MODE;
	netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);
	netc_timer_wr(priv, NETC_TMR_ADD, priv->period_frac);

	return 0;
}

static void netc_timer_deinit(struct netc_timer *priv)
{
	u32 fiper_ctrl;
	int i;

	guard(spinlock_irqsave)(&priv->lock);

	netc_timer_wr(priv, NETC_TMR_TEMASK, 0);
	netc_timer_alarm_write(priv, NETC_TMR_DEFAULT_ALARM, 0);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	for (i = 0; i < NETC_TMR_FIPER_NUM; i++) {
		netc_timer_wr(priv, NETC_TMR_FIPER(i),
				     NETC_TMR_DEFAULT_FIPER);
		fiper_ctrl |= FIPER_CTRL_DIS(i);
	}
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);
}

static int netc_timer_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct netc_timer *priv;
	int err, len, n;

	err = pci_enable_device_mem(pdev);
	if (err)
		return dev_err_probe(dev, err, "device enable failed\n");

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(dev, "DMA configuration failed: 0x%x\n", err);
		goto disable_dev;
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(dev, "pci_request_regions failed err=%d\n", err);
		goto disable_dev;
	}

	pci_set_master(pdev);
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto release_mem_regions;
	}
	priv->dev = dev;
	priv->pci_dev = pdev;
	priv->phc_index = -1; /* initialize it as an invalid index */

	len = pci_resource_len(pdev, NETC_TMR_REGS_BAR);
	priv->base = ioremap(pci_resource_start(pdev, NETC_TMR_REGS_BAR), len);
	if (!priv->base) {
		err = -ENXIO;
		dev_err(dev, "ioremap() failed\n");
		goto free_priv;
	}

	n = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSIX);
	if (n != 1) {
		err = -EPERM;
		goto unmap_resource;
	}
	priv->irq = pci_irq_vector(pdev, 0);
	snprintf(priv->irq_name, sizeof(priv->irq_name),
		"ptp-netc %s", pci_name(pdev));
	err = request_irq(priv->irq, netc_timer_isr, 0, priv->irq_name, priv);
	if (err) {
		dev_err(dev, "request_irq() failed!\n");
		goto free_irq_vectors;
	}

	netc_timer_get_source_clk(priv);
	err = clk_prepare_enable(priv->src_clk);
	if (err) {
		dev_err(dev, "Enable timer source clock failed!\n");
		goto free_irq;
	}

	err = netc_timer_init(priv);
	if (err) {
		dev_err(dev, "NETC Timer initialization failed\n");
		goto disable_clk;
	}

	priv->clock = ptp_clock_register(&priv->caps, priv->dev);
	if (IS_ERR(priv->clock)) {
		err = PTR_ERR(priv->clock);
		goto deinit_timer;
	}

	priv->phc_index = ptp_clock_index(priv->clock);

	pci_set_drvdata(pdev, priv);

	return 0;

deinit_timer:
	netc_timer_deinit(priv);
disable_clk:
	clk_disable_unprepare(priv->src_clk);
free_irq:
	free_irq(priv->irq, priv);
free_irq_vectors:
	pci_free_irq_vectors(pdev);
unmap_resource:
	iounmap(priv->base);
free_priv:
	kfree(priv);
release_mem_regions:
	pci_release_mem_regions(pdev);
disable_dev:
	pci_disable_device(pdev);

	return err;
}

static void netc_timer_remove(struct pci_dev *pdev)
{
	struct netc_timer *priv = pci_get_drvdata(pdev);

	ptp_clock_unregister(priv->clock);
	netc_timer_deinit(priv);
	clk_disable_unprepare(priv->src_clk);

	disable_irq(priv->irq);
	irq_set_affinity_hint(priv->irq, NULL);
	free_irq(priv->irq, priv);
	pci_free_irq_vectors(pdev);

	iounmap(priv->base);
	kfree(priv);

	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id netc_timer_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_NXP2, PCI_DEVICE_ID_NXP2_NETC_TIMER) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, netc_timer_id_table);

static void ptp_netc_shutdown(struct netc_timer *priv)
{
	struct pci_dev *pdev = priv->pci_dev;

	netc_timer_deinit(priv);
	clk_disable_unprepare(priv->src_clk);
	disable_irq(priv->irq);
	irq_set_affinity_hint(priv->irq, NULL);
	free_irq(priv->irq, priv);
	pci_free_irq_vectors(pdev);

	pci_save_state(pdev);
	pci_disable_device(priv->pci_dev);

	return;
}

static int ptp_netc_powerup(struct netc_timer *priv)
{
	struct pci_dev *pdev = priv->pci_dev;
	int err, n;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}
	pci_restore_state(pdev);

	pci_set_master(pdev);

	n = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSIX);
	if (n != 1) {
		err = -EPERM;
		goto disable_dev;
	}
	priv->irq = pci_irq_vector(pdev, 0);
	err = request_irq(priv->irq, netc_timer_isr, 0, priv->irq_name, priv);
	if (err) {
		dev_err(&pdev->dev, "request_irq() failed!\n");
		goto free_irq_vectors;
	}

	err = clk_prepare_enable(priv->src_clk);
	if (err) {
		dev_err(&pdev->dev, "Enable timer source clock failed!\n");
		goto free_irq;
	}

	err = netc_timer_init(priv);
	if (err) {
		dev_err(&pdev->dev, "NETC Timer initialization failed, err=%d\n", err);
		goto disable_clk;
	}

	return 0;

disable_clk:
	clk_disable_unprepare(priv->src_clk);
free_irq:
	free_irq(priv->irq, priv);
free_irq_vectors:
	pci_free_irq_vectors(pdev);
disable_dev:
	pci_disable_device(pdev);

	return err;
}

static int ptp_netc_suspend_noirq(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct netc_timer *priv;

	priv = pci_get_drvdata(pdev);

	if (netc_ierb_may_wakeonlan())
		return 0;
	ptp_netc_shutdown(priv);

	return 0;
}

static int ptp_netc_resume_noirq(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct netc_timer *priv;
	int err;

	priv = pci_get_drvdata(pdev);

	if (netc_ierb_may_wakeonlan())
		return 0;

	err = ptp_netc_powerup(priv);
	if (err) {
		dev_err(dev, "NETC Timer powerup failed\n");
		return err;
	}

	return err;
}

static const struct dev_pm_ops __maybe_unused ptp_netc_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(ptp_netc_suspend_noirq,
				      ptp_netc_resume_noirq)
};

static struct pci_driver netc_timer_driver = {
	.name = KBUILD_MODNAME,
	.id_table = netc_timer_id_table,
	.probe = netc_timer_probe,
	.remove = netc_timer_remove,
	.driver.pm = &ptp_netc_pm_ops,
};
module_pci_driver(netc_timer_driver);

MODULE_AUTHOR("Wei Fang <wei.fang@nxp.com>");
MODULE_DESCRIPTION("NXP NETC Timer Driver");
MODULE_LICENSE("Dual BSD/GPL");
