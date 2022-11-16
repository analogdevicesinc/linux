// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Performance driver to expose DMC statistics to the kernel
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Author: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>

#define ADI_DMC_EVENT_AXI_IDLE_READ_CYCLES		0x00
#define ADI_DMC_EVENT_AXI_IDLE_WRITE_CYCLES		0x01
#define ADI_DMC_EVENT_AXI_BURST_LENGTH_COUNT	0x02
#define ADI_DMC_EVENT_AXI_BURST_SIZE_COUNT		0x03
#define ADI_DMC_EVENT_IDLE_CYCLE_DQ_COUNT		0x04
#define ADI_DMC_EVENT_IDLE_PAGE_COUNT			0x05
#define ADI_DMC_EVENT_PAGE_HIT_COUNT			0x06
#define ADI_DMC_EVENT_PAGE_MISS_COUNT			0x07
#define ADI_DMC_EVENT_READ_TO_WRITE_TURNAROUND	0x08
#define ADI_DMC_EVENT_WRITE_TO_READ_TURNAROUND	0x09
#define ADI_DMC_EVENT_REFRESH_COUNT				0x0a
#define ADI_DMC_EVENT_READ_BUFFER_HIT_COUNT		0x0b

#define ADI_DMC_PM_CTL						0x140
#define ADI_DMC_PM_IDLE_PAGE_COUNT			0x144
#define ADI_DMC_PM_PAGE_MISS_COUNT			0x148
#define ADI_DMC_PM_PAGE_HIT_COUNT			0x14c
#define ADI_DMC_PM_IDLE_CYCLE_DQ_COUNT		0x150
#define ADI_DMC_PM_READ_TO_WRITE_COUNT		0x154
#define ADI_DMC_PM_WRITE_TO_READ_COUNT		0x158
#define ADI_DMC_PM_REFRESH_COUNT			0x15c
#define ADI_DMC_PM_AXI_IDLE_WRITE			0x164
#define ADI_DMC_PM_AXI_IDLE_READ			0x168
#define ADI_DMC_PM_READ_BUFFER_HIT			0x174
#define ADI_DMC_PM_AXI_BURST_LENGTH			0x178
#define ADI_DMC_PM_AXI_BURST_SIZE			0x17c

#define ADI_DMC_EVENTS 12

#define ADI_DMC_PMC_PMCNT   0x01
#define ADI_DMC_PMU_TIMER_PERIOD (ns_to_ktime(500ul*1000ul*1000ul))

static const u32 dmc_pm_offset[ADI_DMC_EVENTS] = {
	[ADI_DMC_EVENT_AXI_IDLE_READ_CYCLES] = ADI_DMC_PM_AXI_IDLE_READ,
	[ADI_DMC_EVENT_AXI_IDLE_WRITE_CYCLES] = ADI_DMC_PM_AXI_IDLE_WRITE,
	[ADI_DMC_EVENT_AXI_BURST_LENGTH_COUNT] = ADI_DMC_PM_AXI_BURST_LENGTH,
	[ADI_DMC_EVENT_AXI_BURST_SIZE_COUNT] = ADI_DMC_PM_AXI_BURST_SIZE,
	[ADI_DMC_EVENT_IDLE_CYCLE_DQ_COUNT] = ADI_DMC_PM_IDLE_CYCLE_DQ_COUNT,
	[ADI_DMC_EVENT_IDLE_PAGE_COUNT] = ADI_DMC_PM_IDLE_PAGE_COUNT,
	[ADI_DMC_EVENT_PAGE_HIT_COUNT] = ADI_DMC_PM_PAGE_HIT_COUNT,
	[ADI_DMC_EVENT_PAGE_MISS_COUNT] = ADI_DMC_PM_PAGE_MISS_COUNT,
	[ADI_DMC_EVENT_READ_TO_WRITE_TURNAROUND] = ADI_DMC_PM_READ_TO_WRITE_COUNT,
	[ADI_DMC_EVENT_WRITE_TO_READ_TURNAROUND] = ADI_DMC_PM_WRITE_TO_READ_COUNT,
	[ADI_DMC_EVENT_REFRESH_COUNT] = ADI_DMC_PM_REFRESH_COUNT,
	[ADI_DMC_EVENT_READ_BUFFER_HIT_COUNT] = ADI_DMC_PM_READ_BUFFER_HIT,
};

struct adi_dmc_pmu {
	struct pmu pmu;
	struct hrtimer hrtimer;
	void __iomem *base;
	const char *name;
	struct perf_event *events[ADI_DMC_EVENTS];
	int num_events;
};

// Event IDs in string here need to line up with above
PMU_EVENT_ATTR_STRING(axi-idle-read-cycles, adi_dmc_axi_idle_read_cycles, "event=0x00");
PMU_EVENT_ATTR_STRING(axi-idle-write-cycles, adi_dmc_axi_idle_write_cycles, "event=0x01");
PMU_EVENT_ATTR_STRING(axi-burst-length-count, adi_dmc_axi_burst_length_count, "event=0x02");
PMU_EVENT_ATTR_STRING(axi-burst-size-count, adi_dmc_axi_burst_size_count, "event=0x03");
PMU_EVENT_ATTR_STRING(idle-cycle-dq-count, adi_dmc_idle_cycle_dq_count, "event=0x04");
PMU_EVENT_ATTR_STRING(idle-page-count, adi_dmc_idle_page_count, "event=0x05");
PMU_EVENT_ATTR_STRING(page-hit-count, adi_dmc_page_hit_count, "event=0x06");
PMU_EVENT_ATTR_STRING(page-miss-count, adi_dmc_page_miss_count, "event=0x07");
PMU_EVENT_ATTR_STRING(read-to-write-turnaround, adi_dmc_read_write_turnaround, "event=0x08");
PMU_EVENT_ATTR_STRING(write-to-read-turnaround, adi_dmc_write_read_turnaround, "event=0x09");
PMU_EVENT_ATTR_STRING(refresh-count, adi_dmc_refresh_count, "event=0x0a");
PMU_EVENT_ATTR_STRING(read-buffer-hit-count, adi_dmc_read_buffer_hit_count, "event=0x0b");

static struct attribute *adi_pmu_events_attrs[] = {
	&adi_dmc_axi_idle_read_cycles.attr.attr,
	&adi_dmc_axi_idle_write_cycles.attr.attr,
	&adi_dmc_axi_burst_length_count.attr.attr,
	&adi_dmc_axi_burst_size_count.attr.attr,
	&adi_dmc_idle_cycle_dq_count.attr.attr,
	&adi_dmc_idle_page_count.attr.attr,
	&adi_dmc_page_hit_count.attr.attr,
	&adi_dmc_page_miss_count.attr.attr,
	&adi_dmc_read_write_turnaround.attr.attr,
	&adi_dmc_write_read_turnaround.attr.attr,
	&adi_dmc_refresh_count.attr.attr,
	&adi_dmc_read_buffer_hit_count.attr.attr,
	NULL,
};

PMU_FORMAT_ATTR(event, "config:0-63");

static struct attribute_group adi_pmu_events_attr_group = {
	.name = "events",
	.attrs = adi_pmu_events_attrs,
};

static struct attribute *adi_pmu_format_attrs[] = {
	&format_attr_event.attr,
	NULL,
};

static struct attribute_group adi_pmu_format_attr_group = {
	.name = "format",
	.attrs = adi_pmu_format_attrs,
};

static const struct attribute_group *attr_groups[] = {
	&adi_pmu_events_attr_group,
	&adi_pmu_format_attr_group,
	NULL,
};

struct adi_dmc_pmu *to_adi_dmc_pmu(struct pmu *pmu)
{
	return container_of(pmu, struct adi_dmc_pmu, pmu);
}

static void adi_dmc_pmu_start(struct perf_event *event, int flags);

static void adi_dmc_pmu_enable(struct adi_dmc_pmu *adi_dmc)
{
	void __iomem *reg = adi_dmc->base + ADI_DMC_PM_CTL;

	iowrite32(ioread32(reg) | ADI_DMC_PMC_PMCNT, reg);

	hrtimer_start(&adi_dmc->hrtimer, ADI_DMC_PMU_TIMER_PERIOD, HRTIMER_MODE_REL_PINNED);
}

static void adi_dmc_pmu_disable(struct adi_dmc_pmu *adi_dmc)
{
	void __iomem *reg = adi_dmc->base + ADI_DMC_PM_CTL;

	iowrite32(ioread32(reg) & ~ADI_DMC_PMC_PMCNT, reg);

	hrtimer_cancel(&adi_dmc->hrtimer);
}

static int adi_dmc_pmu_event_init(struct perf_event *event)
{
	u64 cfg = event->attr.config;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (is_sampling_event(event) || event->attach_state & PERF_ATTACH_TASK)
		return -EOPNOTSUPP;

	if (cfg < 0 || cfg >= ADI_DMC_EVENTS)
		return -EINVAL;

	event->cpu = 0;
	return 0;
}

static u32 read_counter(struct adi_dmc_pmu *adi_dmc, u64 cfg)
{
	void __iomem *reg = adi_dmc->base;

	if (cfg < ADI_DMC_EVENTS)
		return ioread32(reg + dmc_pm_offset[cfg]);

	return WARN_ONCE(1, "counter for config %llu doesn't exist\n", cfg);
}

static void adi_dmc_event_update(struct perf_event *event)
{
	struct adi_dmc_pmu *adi_dmc = to_adi_dmc_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	u64 delta, prev, new;
	u64 mask = 0xffffffff;

	new = read_counter(adi_dmc, event->attr.config);
	prev = local64_read(&hwc->prev_count);

	delta = (new - prev) & mask;
	local64_set(&hwc->prev_count, new);
	local64_add(delta, &event->count);
}

static int adi_dmc_pmu_add(struct perf_event *event, int flags)
{
	struct adi_dmc_pmu *adi_dmc = to_adi_dmc_pmu(event->pmu);
	int cfg = event->attr.config;

	if (adi_dmc->events[cfg])
		return -EAGAIN;

	adi_dmc->events[cfg] = event;
	adi_dmc->num_events += 1;

	if (adi_dmc->num_events == 1)
		adi_dmc_pmu_enable(adi_dmc);

	if (flags & PERF_EF_START)
		adi_dmc_pmu_start(event, flags);

	adi_dmc_event_update(event);
	return 0;
}

static void adi_dmc_pmu_del(struct perf_event *event, int flags)
{
	struct adi_dmc_pmu *adi_dmc = to_adi_dmc_pmu(event->pmu);
	int cfg = event->attr.config;

	adi_dmc->events[cfg] = NULL;
	adi_dmc->num_events -= 1;

	adi_dmc_event_update(event);

	if (adi_dmc->num_events == 0)
		adi_dmc_pmu_disable(adi_dmc);
}

static void adi_dmc_pmu_stop(struct perf_event *event, int flags)
{
	adi_dmc_event_update(event);
}

static void adi_dmc_pmu_start(struct perf_event *event, int flags)
{
	adi_dmc_event_update(event);
	local64_set(&event->count, 0);
}

static void adi_dmc_pmu_read(struct perf_event *event)
{
	adi_dmc_event_update(event);
}

static enum hrtimer_restart adi_pmu_timer_handler(struct hrtimer *hrtimer)
{
	struct adi_dmc_pmu *adi_dmc = container_of(hrtimer, struct adi_dmc_pmu, hrtimer);
	int i;

	for (i = 0; i < ADI_DMC_EVENTS; ++i) {
		if (adi_dmc->events[i])
			adi_dmc_event_update(adi_dmc->events[i]);
	}

	hrtimer_forward_now(hrtimer, ADI_DMC_PMU_TIMER_PERIOD);
	return HRTIMER_RESTART;
}

// @todo make burst size/length configurable via attributes
static int adi_dmc_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_dmc_pmu *adi_dmc = NULL;
	struct resource *res;
	int ret;

	adi_dmc = devm_kzalloc(dev, sizeof(*adi_dmc), GFP_KERNEL);
	if (!adi_dmc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adi_dmc->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(adi_dmc->base)) {
		dev_err(dev, "Unable to map DMC base address\n");
		return PTR_ERR(adi_dmc->base);
	}

	// Read+write for efficiency, burst size 32, burst length 4, all banks, 1:1 scale
	iowrite32((3 << 16) | (1 << 12) | (3 << 8) | (8 << 4) | (0 << 1),
		adi_dmc->base + ADI_DMC_PM_CTL);

	platform_set_drvdata(pdev, adi_dmc);
	adi_dmc->name = "adi_dmc_perf";

	hrtimer_init(&adi_dmc->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	adi_dmc->hrtimer.function = adi_pmu_timer_handler;

	adi_dmc->pmu = (struct pmu) {
		.task_ctx_nr = perf_invalid_context,
		.event_init = adi_dmc_pmu_event_init,
		.add = adi_dmc_pmu_add,
		.del = adi_dmc_pmu_del,
		.start = adi_dmc_pmu_start,
		.stop = adi_dmc_pmu_stop,
		.read = adi_dmc_pmu_read,
		.attr_groups = attr_groups,
		.capabilities = PERF_PMU_CAP_NO_EXCLUDE,
	};

	ret = perf_pmu_register(&adi_dmc->pmu, adi_dmc->name, -1);
	if (ret)
		return ret;

	return 0;
}

static int adi_dmc_pmu_remove(struct platform_device *pdev)
{
	struct adi_dmc_pmu *adi_dmc = platform_get_drvdata(pdev);

	perf_pmu_unregister(&adi_dmc->pmu);
	return 0;
}

static const struct of_device_id dmc_pmu_dt_ids[] = {
	{ .compatible = "adi,dmc-pmu" },
	{ }
};

static struct platform_driver dmc_pmu_driver = {
	.driver = {
		.name = "adi-dmc-pmu",
		.of_match_table = dmc_pmu_dt_ids
	},
	.probe = adi_dmc_pmu_probe,
	.remove = adi_dmc_pmu_remove,
};
module_platform_driver(dmc_pmu_driver);

MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
MODULE_DESCRIPTION("SC598 DMC Performance Monitor Driver");
MODULE_LICENSE("GPL");
