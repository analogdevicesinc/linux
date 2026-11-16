// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Loongson Technology Corporation Limited
 *
 * Author: Yinggang Gu <guyinggang@loongson.cn>
 * Author: Qunqin Zhao <zhaoqunqin@loongson.cn>
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/loongson-se.h>
#include <linux/module.h>
#include <linux/platform_device.h>

struct loongson_se {
	void __iomem *base;
	spinlock_t dev_lock;
	struct completion cmd_completion;

	void *dmam_base;
	u32 dmam_size;

	struct mutex engine_init_lock;
	struct loongson_se_engine engines[SE_ENGINE_MAX];
};

struct loongson_se_controller_cmd {
	u32 command_id;
	u32 info[7];
};

static int loongson_se_poll(struct loongson_se *se, u32 int_bit,
			    struct completion *completion)
{
	u32 status;
	int err;

	spin_lock_irq(&se->dev_lock);

	writel(int_bit, se->base + SE_S2LINT_CL);
	reinit_completion(completion);

	/* Notify the controller that the engine needs to be started */
	writel(int_bit, se->base + SE_L2SINT_SET);

	/* Polling until the controller has forwarded the engine command */
	err = readl_relaxed_poll_timeout_atomic(se->base + SE_L2SINT_STAT, status,
						!(status & int_bit),
						1, LOONGSON_ENGINE_CMD_TIMEOUT_US);

	spin_unlock_irq(&se->dev_lock);

	return err;
}

static int loongson_se_send_controller_cmd(struct loongson_se *se,
					   struct loongson_se_controller_cmd *cmd)
{
	u32 *send_cmd = (u32 *)cmd;
	int err, i;

	for (i = 0; i < SE_SEND_CMD_REG_LEN; i++)
		writel(send_cmd[i], se->base + SE_SEND_CMD_REG + i * 4);

	err = loongson_se_poll(se, SE_INT_CONTROLLER, &se->cmd_completion);
	if (err)
		return err;

	wait_for_completion(&se->cmd_completion);

	return 0;
}

int loongson_se_send_engine_cmd(struct loongson_se_engine *engine)
{
	/*
	 * After engine initialization, the controller already knows
	 * where to obtain engine commands from. Now all we need to
	 * do is notify the controller that the engine needs to be started.
	 */
	int err = loongson_se_poll(engine->se, BIT(engine->id),
				   &engine->completion);

	if (err)
		return err;

	wait_for_completion(&engine->completion);

	return 0;
}
EXPORT_SYMBOL_GPL(loongson_se_send_engine_cmd);

struct loongson_se_engine *loongson_se_init_engine(struct device *dev, int id)
{
	struct loongson_se *se = dev_get_drvdata(dev);
	struct loongson_se_controller_cmd cmd = {};
	struct loongson_se_engine *engine;

	if (id <= 0 || id >= SE_ENGINE_MAX)
		return NULL;

	engine = &se->engines[id];

	engine->se = se;
	engine->id = id;

	/* Divide DMA memory equally among all engines */
	engine->buffer_size = se->dmam_size / SE_ENGINE_MAX;
	engine->buffer_off = (se->dmam_size / SE_ENGINE_MAX) * id;
	engine->data_buffer = se->dmam_base + engine->buffer_off;

	/*
	 * There has no engine0, use its data buffer as command buffer for other
	 * engines. The DMA memory size is obtained from the ACPI table, which
	 * ensures that the data buffer size of engine0 is larger than the
	 * command buffer size of all engines.
	 */
	engine->command = se->dmam_base + id * (2 * SE_ENGINE_CMD_SIZE);
	engine->command_ret = engine->command + SE_ENGINE_CMD_SIZE;

	mutex_lock(&se->engine_init_lock);

	/* Tell the controller where to find engine command */
	cmd.command_id = SE_CMD_SET_ENGINE_CMDBUF;
	cmd.info[0] = id;
	cmd.info[1] = engine->command - se->dmam_base;
	cmd.info[2] = 2 * SE_ENGINE_CMD_SIZE;

	if (loongson_se_send_controller_cmd(se, &cmd))
		engine = NULL;

	mutex_unlock(&se->engine_init_lock);

	return engine;
}
EXPORT_SYMBOL_GPL(loongson_se_init_engine);

static irqreturn_t se_irq_handler(int irq, void *dev_id)
{
	struct loongson_se *se = dev_id;
	u32 int_status;
	int id;

	spin_lock(&se->dev_lock);

	int_status = readl(se->base + SE_S2LINT_STAT);
	if (int_status == 0) {
		spin_unlock(&se->dev_lock);
		return IRQ_NONE;
	}

	/* For controller */
	if (int_status & SE_INT_CONTROLLER) {
		complete(&se->cmd_completion);
		int_status &= ~SE_INT_CONTROLLER;
		writel(SE_INT_CONTROLLER, se->base + SE_S2LINT_CL);
	}

	/* For engines */
	while (int_status) {
		id = __ffs(int_status);
		if (id < SE_ENGINE_MAX)
			complete(&se->engines[id].completion);
		int_status &= ~BIT(id);
		writel(BIT(id), se->base + SE_S2LINT_CL);
	}

	spin_unlock(&se->dev_lock);

	return IRQ_HANDLED;
}

static void loongson_se_stop(void *data)
{
	struct loongson_se *se = data;
	struct loongson_se_controller_cmd cmd = {};

	mutex_lock(&se->engine_init_lock);

	cmd.command_id = SE_CMD_STOP;
	loongson_se_send_controller_cmd(se, &cmd);

	spin_lock_irq(&se->dev_lock);
	writel(0, se->base + SE_S2LINT_EN);
	spin_unlock_irq(&se->dev_lock);

	mutex_unlock(&se->engine_init_lock);
}

static int loongson_se_init(struct loongson_se *se, dma_addr_t addr, u32 size)
{
	struct loongson_se_controller_cmd cmd = {};
	int err;

	cmd.command_id = SE_CMD_START;
	err = loongson_se_send_controller_cmd(se, &cmd);
	if (err) {
		writel(0, se->base + SE_S2LINT_EN);
		return err;
	}

	cmd.command_id = SE_CMD_SET_DMA;
	cmd.info[0] = lower_32_bits(addr);
	cmd.info[1] = upper_32_bits(addr);
	cmd.info[2] = size;

	err = loongson_se_send_controller_cmd(se, &cmd);
	if (err) {
		loongson_se_stop(se);
		return err;
	}

	return 0;
}

static const struct mfd_cell engines[] = {
	{ .name = "loongson-rng" },
	{ .name = "tpm_loongson" },
};

static void loongson_se_shutdown(struct platform_device *pdev)
{
	struct loongson_se *se = platform_get_drvdata(pdev);

	loongson_se_stop(se);
}

static int loongson_se_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_se *se;
	int nr_irq, irq, err, i;
	dma_addr_t paddr;

	se = devm_kzalloc_obj(dev, *se);
	if (!se)
		return -ENOMEM;

	dev_set_drvdata(dev, se);
	init_completion(&se->cmd_completion);
	for (i = 0; i < SE_ENGINE_MAX; i++)
		init_completion(&se->engines[i].completion);
	spin_lock_init(&se->dev_lock);
	mutex_init(&se->engine_init_lock);

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (err)
		return err;

	if (device_property_read_u32(dev, "dmam_size", &se->dmam_size))
		return -ENODEV;

	if ((se->dmam_size / SE_ENGINE_MAX) < (SE_ENGINE_MAX * 2 * SE_ENGINE_CMD_SIZE))
		return -EINVAL;

	se->dmam_base = dmam_alloc_coherent(dev, se->dmam_size, &paddr, GFP_KERNEL);
	if (!se->dmam_base)
		return -ENOMEM;

	se->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(se->base))
		return PTR_ERR(se->base);

	nr_irq = platform_irq_count(pdev);
	if (!nr_irq)
		return -ENODEV;
	if (nr_irq < 0)
		return nr_irq;

	writel(SE_INT_ALL, se->base + SE_S2LINT_CL);

	for (i = 0; i < nr_irq; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;

		err = devm_request_irq(dev, irq, se_irq_handler, IRQF_SHARED, "loongson-se", se);
		if (err)
			return dev_err_probe(dev, err, "Failed to request IRQ: %d\n", irq);
	}

	writel(SE_INT_ALL, se->base + SE_S2LINT_EN);

	err = loongson_se_init(se, paddr, se->dmam_size);
	if (err)
		return err;

	err = devm_add_action_or_reset(dev, loongson_se_stop, se);
	if (err)
		return err;

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, engines,
				    ARRAY_SIZE(engines), NULL, 0, NULL);
}

static const struct acpi_device_id loongson_se_acpi_match[] = {
	{ .id = "LOON0011" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, loongson_se_acpi_match);

static struct platform_driver loongson_se_driver = {
	.probe    = loongson_se_probe,
	.shutdown = loongson_se_shutdown,
	.driver   = {
		.name  = "loongson-se",
		.acpi_match_table = loongson_se_acpi_match,
	},
};
module_platform_driver(loongson_se_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yinggang Gu <guyinggang@loongson.cn>");
MODULE_AUTHOR("Qunqin Zhao <zhaoqunqin@loongson.cn>");
MODULE_DESCRIPTION("Loongson Security Engine chip controller driver");
