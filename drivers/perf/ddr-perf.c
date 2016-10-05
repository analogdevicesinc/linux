/*
 * Copyright 2017 NXP
 * Copyright 2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/perf_event.h>
#include <linux/slab.h>


#define COUNTER_CNTL	0x0
#define COUNTER_READ	0x20

#define CNTL_OVER	0x1
#define	CNTL_CLEAR	0x2
#define CNTL_EN		0x4

#define CNTL_CP_SHIFT	16
#define CNTL_CP_MASK	(0xFF << CNTL_CP_SHIFT)

#define CNTL_CSV_SHIFT	24
#define CNTL_CSV_MASK	(0xFF << CNTL_CSV_SHIFT)

#define NUM_COUNTER	4
#define MAX_EVENT	3

#define to_ddr_pmu(p)	container_of(p, struct ddr_pmu, pmu)

static DEFINE_IDA(ddr_ida);

PMU_EVENT_ATTR_STRING(cycles, ddr_perf_cycles, "event=0x00");
PMU_EVENT_ATTR_STRING(selfresh, ddr_perf_selfresh, "event=0x01");
PMU_EVENT_ATTR_STRING(read-access, ddr_perf_read_accesses, "event=0x04");
PMU_EVENT_ATTR_STRING(write-access, ddr_perf_write_accesses, "event=0x05");
PMU_EVENT_ATTR_STRING(read-queue-depth, ddr_perf_read_queue_depth,
		"event=0x08");
PMU_EVENT_ATTR_STRING(write-queue-depth, ddr_perf_write_queue_depth,
		"event=0x09");
PMU_EVENT_ATTR_STRING(lp-read-credit-cnt, ddr_perf_lp_read_credit_cnt,
		"event=0x10");
PMU_EVENT_ATTR_STRING(hp-read-credit-cnt, ddr_perf_hp_read_credit_cnt,
		"event=0x11");
PMU_EVENT_ATTR_STRING(write-credit-cnt, ddr_perf_write_credit_cnt,
		"event=0x12");
PMU_EVENT_ATTR_STRING(read-command, ddr_perf_read_command, "event=0x20");
PMU_EVENT_ATTR_STRING(write-command, ddr_perf_write_command, "event=0x21");
PMU_EVENT_ATTR_STRING(read-modify-write-command,
		ddr_perf_read_modify_write_command, "event=0x22");
PMU_EVENT_ATTR_STRING(hp-read, ddr_perf_hp_read, "event=0x23");
PMU_EVENT_ATTR_STRING(hp-req-nodcredit, ddr_perf_hp_req_nocredit, "event=0x24");
PMU_EVENT_ATTR_STRING(hp-xact-credit, ddr_perf_hp_xact_credit, "event=0x25");
PMU_EVENT_ATTR_STRING(lp-req-nocredit, ddr_perf_lp_req_nocredit, "event=0x26");
PMU_EVENT_ATTR_STRING(lp-xact-credit, ddr_perf_lp_xact_credit, "event=0x27");
PMU_EVENT_ATTR_STRING(wr-xact-credit, ddr_perf_wr_xact_credit, "event=0x29");
PMU_EVENT_ATTR_STRING(read-cycles, ddr_perf_read_cycles, "event=0x2a");
PMU_EVENT_ATTR_STRING(write-cycles, ddr_perf_write_cycles, "event=0x2b");
PMU_EVENT_ATTR_STRING(read-write-transition, ddr_perf_read_write_transition,
		"event=0x30");
PMU_EVENT_ATTR_STRING(precharge, ddr_perf_precharge, "event=0x31");
PMU_EVENT_ATTR_STRING(activate, ddr_perf_activate, "event=0x32");
PMU_EVENT_ATTR_STRING(load-mode, ddr_perf_load_mode, "event=0x33");
PMU_EVENT_ATTR_STRING(mwr, ddr_perf_mwr, "event=0x34");
PMU_EVENT_ATTR_STRING(read, ddr_perf_read, "event=0x35");
PMU_EVENT_ATTR_STRING(read-activate, ddr_perf_read_activate, "event=0x36");
PMU_EVENT_ATTR_STRING(refresh, ddr_perf_refresh, "event=0x37");
PMU_EVENT_ATTR_STRING(write, ddr_perf_write, "event=0x38");
PMU_EVENT_ATTR_STRING(raw-hazard, ddr_perf_raw_hazard, "event=0x39");

struct ddr_pmu {
	struct pmu pmu;
	void __iomem *base;
	cpumask_t cpu;
	struct	hlist_node node;
	struct	device *dev;
	int	e2c_map[NUM_COUNTER];
};

static ssize_t ddr_perf_cpumask_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ddr_pmu *pmu = dev_get_drvdata(dev);

	return cpumap_print_to_pagebuf(true, buf, &pmu->cpu);
}

static struct device_attribute ddr_perf_cpumask_attr =
	__ATTR(cpumask, 0444, ddr_perf_cpumask_show, NULL);

static struct attribute *ddr_perf_cpumask_attrs[] = {
	&ddr_perf_cpumask_attr.attr,
	NULL,
};

static struct attribute_group ddr_perf_cpumask_attr_group = {
	.attrs = ddr_perf_cpumask_attrs,
};

static struct attribute *ddr_perf_events_attrs[] = {
	&ddr_perf_cycles.attr.attr,
	&ddr_perf_selfresh.attr.attr,
	&ddr_perf_read_accesses.attr.attr,
	&ddr_perf_write_accesses.attr.attr,
	&ddr_perf_read_queue_depth.attr.attr,
	&ddr_perf_write_queue_depth.attr.attr,
	&ddr_perf_lp_read_credit_cnt.attr.attr,
	&ddr_perf_hp_read_credit_cnt.attr.attr,
	&ddr_perf_write_credit_cnt.attr.attr,
	&ddr_perf_read_command.attr.attr,
	&ddr_perf_write_command.attr.attr,
	&ddr_perf_read_modify_write_command.attr.attr,
	&ddr_perf_hp_read.attr.attr,
	&ddr_perf_hp_req_nocredit.attr.attr,
	&ddr_perf_hp_xact_credit.attr.attr,
	&ddr_perf_lp_req_nocredit.attr.attr,
	&ddr_perf_lp_xact_credit.attr.attr,
	&ddr_perf_wr_xact_credit.attr.attr,
	&ddr_perf_read_cycles.attr.attr,
	&ddr_perf_write_cycles.attr.attr,
	&ddr_perf_read_write_transition.attr.attr,
	&ddr_perf_precharge.attr.attr,
	&ddr_perf_activate.attr.attr,
	&ddr_perf_load_mode.attr.attr,
	&ddr_perf_mwr.attr.attr,
	&ddr_perf_read.attr.attr,
	&ddr_perf_read_activate.attr.attr,
	&ddr_perf_refresh.attr.attr,
	&ddr_perf_write.attr.attr,
	&ddr_perf_raw_hazard.attr.attr,
	NULL,
};

static struct attribute_group ddr_perf_events_attr_group = {
	.name = "events",
	.attrs = ddr_perf_events_attrs,
};

PMU_FORMAT_ATTR(event, "config:0-63");
static struct attribute *ddr_perf_format_attrs[] = {
	&format_attr_event.attr,
	NULL,
};

static struct attribute_group ddr_perf_format_attr_group = {
	.name = "format",
	.attrs = ddr_perf_format_attrs,
};

static const struct attribute_group *attr_groups[] = {
	&ddr_perf_events_attr_group,
	&ddr_perf_format_attr_group,
	&ddr_perf_cpumask_attr_group,
	NULL,
};

static u32 ddr_perf_alloc_counter(struct ddr_pmu *pmu, int event)
{
	int i;

	if (event == 0)
		return 0; /* always map cycle event to counter 0 */

	for (i = 1; i < NUM_COUNTER; i++) {
		if (pmu->e2c_map[i] == 0) {
			pmu->e2c_map[i] = event;
			return i;
		}
	}

	return -ENOENT;
}

static u32 ddr_perf_free_counter(struct ddr_pmu *pmu, int counter)
{
	if (counter < 0 || counter >= NUM_COUNTER)
		return -ENOENT;

	pmu->e2c_map[counter] = 0;

	return 0;
}

static u32 ddr_perf_read_counter(struct ddr_pmu *pmu, int counter)
{
	return readl(pmu->base + COUNTER_READ + counter * 4);
}

static int ddr_perf_event_init(struct perf_event *event)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (is_sampling_event(event) || event->attach_state & PERF_ATTACH_TASK)
		return -EOPNOTSUPP;

	if (event->cpu < 0) {
		dev_warn(pmu->dev, "Can't provide per-task data!\n");
		return -EOPNOTSUPP;
	}

	if (event->attr.exclude_user        ||
	    event->attr.exclude_kernel      ||
	    event->attr.exclude_hv          ||
	    event->attr.exclude_idle        ||
	    event->attr.exclude_host        ||
	    event->attr.exclude_guest       ||
	    event->attr.sample_period)
		return -EINVAL;

	event->cpu = cpumask_first(&pmu->cpu);
	hwc->idx = -1;

	return 0;
}


static void ddr_perf_event_update(struct perf_event *event)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	u64 delta, prev_raw_count, new_raw_count;
	int counter = hwc->idx;

	do {
		prev_raw_count = local64_read(&hwc->prev_count);
		new_raw_count = ddr_perf_read_counter(pmu, counter);
	} while (local64_cmpxchg(&hwc->prev_count, prev_raw_count,
			new_raw_count) != prev_raw_count);

	delta = (new_raw_count - prev_raw_count) & 0xFFFFFFFF;

	local64_add(delta, &event->count);
}

static void ddr_perf_event_start(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int reg;
	int counter = hwc->idx;

	local64_set(&hwc->prev_count, 0);

	writel(0, pmu->base + counter * 4 + COUNTER_CNTL);
	reg = CNTL_EN | CNTL_CLEAR;
	reg |= (event->attr.config << CNTL_CSV_SHIFT) & CNTL_CSV_MASK;
	writel(reg, pmu->base + counter * 4 + COUNTER_CNTL);
}

static int ddr_perf_event_add(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int counter;
	int cfg = event->attr.config;

	counter = ddr_perf_alloc_counter(pmu, cfg);
	if (counter < 0) {
		dev_warn(pmu->dev, "There are not enough counters\n");
		return -EOPNOTSUPP;
	}

	hwc->idx = counter;

	if (flags & PERF_EF_START)
		ddr_perf_event_start(event, flags);

	local64_set(&hwc->prev_count, ddr_perf_read_counter(pmu, counter));

	return 0;
}

static void ddr_perf_event_stop(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int counter = hwc->idx;

	writel(0, pmu->base + counter * 4 + COUNTER_CNTL);
}

static void ddr_perf_event_del(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int counter = hwc->idx;

	ddr_perf_event_stop(event, PERF_EF_UPDATE);

	ddr_perf_free_counter(pmu, counter);
	hwc->idx = -1;
}

static int ddr_perf_init(struct ddr_pmu *pmu, void __iomem *base,
			 struct device *dev)
{
	*pmu = (struct ddr_pmu) {
		.pmu = (struct pmu) {
			.task_ctx_nr = perf_invalid_context,
			.attr_groups = attr_groups,
			.event_init  = ddr_perf_event_init,
			.add	     = ddr_perf_event_add,
			.del	     = ddr_perf_event_del,
			.start	     = ddr_perf_event_start,
			.stop	     = ddr_perf_event_stop,
			.read	     = ddr_perf_event_update,
		},
		.base = base,
		.dev = dev,
	};

	return ida_simple_get(&ddr_ida, 0, 0, GFP_KERNEL);
}

static int ddr_perf_probe(struct platform_device *pdev)
{
	struct ddr_pmu *pmu;
	void __iomem *base;
	struct resource *iomem;
	char *name;
	int num;
	int ret;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		return ret;
	}

	pmu = kzalloc(sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;

	num = ddr_perf_init(pmu, base, &pdev->dev);
	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "ddr%d", num);

	cpumask_set_cpu(smp_processor_id(), &pmu->cpu);
	ret = perf_pmu_register(&(pmu->pmu), name, -1);
	if (ret)
		goto ddr_perf_err;

	return 0;

ddr_perf_err:
	pr_warn("i.MX8 DDR Perf PMU failed (%d), disabled\n", ret);
	kfree(pmu);
	return ret;
}


static int ddr_perf_remove(struct platform_device *pdev)
{
	struct ddr_pmu *pmu = platform_get_drvdata(pdev);

	perf_pmu_unregister(&pmu->pmu);
	kfree(pmu);

	return 0;
}

static const struct of_device_id imx_ddr_pmu_dt_ids[] = {
	{ .compatible = "fsl,imx8-ddr-pmu", },
	{ /* sentinel */ }
};

static struct platform_driver imx_ddr_pmu_driver = {
	.driver         = {
		.name   = "imx-ddr-pmu",
		.of_match_table = imx_ddr_pmu_dt_ids,
	},
	.probe          = ddr_perf_probe,
	.remove         = ddr_perf_remove,
};

static int __init imx_ddr_pmu_init(void)
{
	return platform_driver_register(&imx_ddr_pmu_driver);
}

module_init(imx_ddr_pmu_init);

