// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - platform driver
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include "wave6-vpu.h"
#include "wave6-regdefine.h"
#include "wave6-vpuconfig.h"
#include "wave6.h"
#include "wave6-vpu-ctrl.h"
#include "wave6-vpu-dbg.h"

#define VPU_PLATFORM_DEVICE_NAME "vpu"
#define VPU_CLK_NAME "vcodec"
#define WAVE6_VPU_DEBUGFS_DIR "wave6"

#define WAVE6_IS_ENC BIT(0)
#define WAVE6_IS_DEC BIT(1)

static unsigned int debug;
module_param(debug, uint, 0644);

struct wave6_match_data {
	int flags;
};

static const struct wave6_match_data wave633c_data = {
	.flags = WAVE6_IS_ENC | WAVE6_IS_DEC,
};

unsigned int wave6_vpu_debug(void)
{
	return debug;
}

static irqreturn_t wave6_vpu_irq(int irq, void *dev_id)
{
	struct vpu_device *dev = dev_id;
	u32 irq_status;

	if (wave6_vdi_readl(dev, W6_VPU_VPU_INT_STS)) {
		irq_status = wave6_vdi_readl(dev, W6_VPU_VINT_REASON);

		wave6_vdi_writel(dev, W6_VPU_VINT_REASON_CLR, irq_status);
		wave6_vdi_writel(dev, W6_VPU_VINT_CLEAR, 0x1);

		kfifo_in(&dev->irq_status, &irq_status, sizeof(int));

		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

static irqreturn_t wave6_vpu_irq_thread(int irq, void *dev_id)
{
	struct vpu_device *dev = dev_id;
	struct vpu_instance *inst;
	int irq_status, ret;

	while (kfifo_len(&dev->irq_status)) {
		inst = v4l2_m2m_get_curr_priv(dev->m2m_dev);
		if (inst) {
			inst->ops->finish_process(inst);
		} else {
			ret = kfifo_out(&dev->irq_status, &irq_status, sizeof(int));
			if (!ret)
				break;

			complete(&dev->irq_done);
		}
	}

	return IRQ_HANDLED;
}



static u32 wave6_vpu_read_reg(struct device *dev, u32 addr)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	return wave6_vdi_readl(vpu_dev, addr);
}

static void wave6_vpu_write_reg(struct device *dev, u32 addr, u32 data)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	wave6_vdi_writel(vpu_dev, addr, data);
}

static void wave6_vpu_on_boot(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	u32 product_code;
	u32 version;
	u32 revision;
	u32 hw_version;
	int ret;

	product_code = wave6_vdi_readl(vpu_dev, W6_VPU_RET_PRODUCT_VERSION);
	vpu_dev->product = wave_vpu_get_product_id(vpu_dev);

	wave6_enable_interrupt(vpu_dev);
	ret = wave6_vpu_get_version(vpu_dev, &version, &revision);
	if (ret) {
		dev_err(dev, "wave6_vpu_get_version fail\n");
		return;
	}

	hw_version = wave6_vdi_readl(vpu_dev, W6_RET_CONF_REVISION);

	if (vpu_dev->product_code != product_code ||
	    vpu_dev->fw_version != version ||
	    vpu_dev->fw_revision != revision ||
	    vpu_dev->hw_version != hw_version) {
		vpu_dev->product_code = product_code;
		vpu_dev->fw_version = version;
		vpu_dev->fw_revision = revision;
		vpu_dev->hw_version = hw_version;
		dev_info(dev,
			 "product: 0x%x, fw_version : v%d.%d.%d_g%08x(r%d), hw_version : 0x%x\n",
			 vpu_dev->product_code,
			 (version >> 24) & 0xFF,
			 (version >> 16) & 0xFF,
			 (version >> 0) & 0xFFFF,
			 wave6_vdi_readl(vpu_dev, W6_RET_SHA_ID),
			 revision,
			 vpu_dev->hw_version);
	}

	if (vpu_dev->num_clks)
		vpu_dev->vpu_clk_rate = clk_get_rate(vpu_dev->clks[0].clk);
}

static int wave6_vpu_probe(struct platform_device *pdev)
{
	int ret;
	struct vpu_device *dev;
	const struct wave6_match_data *match_data;
	struct platform_device *pctrl;

	match_data = device_get_match_data(&pdev->dev);
	if (!match_data) {
		dev_err(&pdev->dev, "missing match_data\n");
		return -EINVAL;
	}

	/* physical addresses limited to 32 bits */
	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->dev_lock);
	mutex_init(&dev->hw_lock);
	init_completion(&dev->irq_done);
	dev_set_drvdata(&pdev->dev, dev);
	dev->dev = &pdev->dev;

	dev->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->reg_base))
		return PTR_ERR(dev->reg_base);

	pctrl = of_find_device_by_node(of_parse_phandle(pdev->dev.of_node, "cnm,ctrl", 0));
	if (!pctrl) {
		dev_err(&pdev->dev, "missing vpuctrl\n");
		return -EINVAL;
	}

	dev->ctrl = &pctrl->dev;
	dev->entity.dev = dev->dev;
	dev->entity.read_reg = wave6_vpu_read_reg;
	dev->entity.write_reg = wave6_vpu_write_reg;
	dev->entity.on_boot = wave6_vpu_on_boot;
	if (wave6_vpu_ctrl_get_state(dev->ctrl) < 0) {
		dev_info(&pdev->dev, "vpu ctrl is not ready, defer probe\n");
		return -EPROBE_DEFER;
	}

	ret = devm_clk_bulk_get_all(&pdev->dev, &dev->clks);

	/* continue without clock, assume externally managed */
	if (ret < 0) {
		dev_warn(&pdev->dev, "unable to get clocks: %d\n", ret);
		ret = 0;
	}
	dev->num_clks = ret;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register fail: %d\n", ret);
		return ret;
	}

	if (match_data->flags & WAVE6_IS_DEC) {
		ret = wave6_vpu_dec_register_device(dev);
		if (ret) {
			dev_err(&pdev->dev, "wave6_vpu_dec_register_device fail: %d\n", ret);
			goto err_v4l2_unregister;
		}
	}
	if (match_data->flags & WAVE6_IS_ENC) {
		ret = wave6_vpu_enc_register_device(dev);
		if (ret) {
			dev_err(&pdev->dev, "wave6_vpu_enc_register_device fail: %d\n", ret);
			goto err_dec_unreg;
		}
	}

	ret = wave6_vpu_init_m2m_dev(dev);
	if (ret)
		goto err_enc_unreg;

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		ret = -ENXIO;
		goto err_m2m_dev_release;
	}

	if (kfifo_alloc(&dev->irq_status, 16 * sizeof(int), GFP_KERNEL)) {
		dev_err(&pdev->dev, "failed to allocate fifo\n");
		goto err_m2m_dev_release;
	}

	ret = devm_request_threaded_irq(&pdev->dev, dev->irq, wave6_vpu_irq,
					wave6_vpu_irq_thread, 0, "vpu_irq", dev);
	if (ret) {
		dev_err(&pdev->dev, "fail to register interrupt handler: %d\n", ret);
		goto err_kfifo_free;
	}

	dev->debugfs = debugfs_lookup(WAVE6_VPU_DEBUGFS_DIR, NULL);
	if (!dev->debugfs)
		dev->debugfs = debugfs_create_dir(WAVE6_VPU_DEBUGFS_DIR, NULL);

	pm_runtime_enable(&pdev->dev);

	dev_dbg(&pdev->dev, "Added wave driver with caps %s %s\n",
		match_data->flags & WAVE6_IS_ENC ? "'ENCODE'" : "",
		match_data->flags & WAVE6_IS_DEC ? "'DECODE'" : "");

	return 0;

err_kfifo_free:
	kfifo_free(&dev->irq_status);
err_m2m_dev_release:
	wave6_vpu_release_m2m_dev(dev);
err_enc_unreg:
	if (match_data->flags & WAVE6_IS_ENC)
		wave6_vpu_enc_unregister_device(dev);
err_dec_unreg:
	if (match_data->flags & WAVE6_IS_DEC)
		wave6_vpu_dec_unregister_device(dev);
err_v4l2_unregister:
	v4l2_device_unregister(&dev->v4l2_dev);

	return ret;
}

static int wave6_vpu_remove(struct platform_device *pdev)
{
	struct vpu_device *dev = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	wave6_vpu_release_m2m_dev(dev);
	wave6_vpu_enc_unregister_device(dev);
	wave6_vpu_dec_unregister_device(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	kfifo_free(&dev->irq_status);

	return 0;
}

#ifdef CONFIG_PM
static int wave6_vpu_runtime_suspend(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	if (!vpu_dev)
		return -ENODEV;

	dprintk(dev, "runtime suspend\n");
	wave6_vpu_ctrl_put_sync(vpu_dev->ctrl, &vpu_dev->entity);
	if (vpu_dev->num_clks)
		clk_bulk_disable_unprepare(vpu_dev->num_clks, vpu_dev->clks);

	return 0;
}

static int wave6_vpu_runtime_resume(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	int ret;

	if (!vpu_dev)
		return -ENODEV;

	dprintk(dev, "runtime resume\n");
	if (vpu_dev->num_clks) {
		ret = clk_bulk_prepare_enable(vpu_dev->num_clks, vpu_dev->clks);
		if (ret) {
			dev_err(dev, "failed to enable clocks: %d\n", ret);
			return ret;
		}
	}

	ret = wave6_vpu_ctrl_resume_and_get(vpu_dev->ctrl, &vpu_dev->entity);
	if (ret && vpu_dev->num_clks)
		clk_bulk_disable_unprepare(vpu_dev->num_clks, vpu_dev->clks);

	return ret;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int wave6_vpu_suspend(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	int ret;

	dprintk(dev, "suspend\n");
	v4l2_m2m_suspend(vpu_dev->m2m_dev);

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		v4l2_m2m_resume(vpu_dev->m2m_dev);
	return ret;
}

static int wave6_vpu_resume(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	int ret;

	dprintk(dev, "resume\n");
	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	v4l2_m2m_resume(vpu_dev->m2m_dev);
	return 0;
}
#endif
static const struct dev_pm_ops wave6_vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(wave6_vpu_runtime_suspend, wave6_vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(wave6_vpu_suspend, wave6_vpu_resume)
};

static const struct of_device_id wave6_dt_ids[] = {
	{ .compatible = "fsl,cnm633c-vpu", .data = &wave633c_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wave6_dt_ids);

static struct platform_driver wave6_vpu_driver = {
	.driver = {
		.name = VPU_PLATFORM_DEVICE_NAME,
		.of_match_table = of_match_ptr(wave6_dt_ids),
		.pm = &wave6_vpu_pm_ops,
	},
	.probe = wave6_vpu_probe,
	.remove = wave6_vpu_remove,
	//.suspend = vpu_suspend,
	//.resume = vpu_resume,
};

module_platform_driver(wave6_vpu_driver);
MODULE_DESCRIPTION("chips&media VPU V4L2 driver");
MODULE_LICENSE("Dual BSD/GPL");
