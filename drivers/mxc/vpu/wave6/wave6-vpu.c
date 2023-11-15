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
#include <linux/of_address.h>
#include <linux/genalloc.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include "wave6-vpu.h"
#include "wave6-regdefine.h"
#include "wave6-vpuconfig.h"
#include "wave6.h"

#define VPU_PLATFORM_DEVICE_NAME "vdec"
#define VPU_CLK_NAME "vcodec"

#define WAVE6_IS_ENC BIT(0)
#define WAVE6_IS_DEC BIT(1)

struct wave6_match_data {
	int flags;
	const char *fw_name;
	unsigned int sram_size;
};

static const struct wave6_match_data wave633c_data = {
	.flags = WAVE6_IS_ENC | WAVE6_IS_DEC,
	.fw_name = "wave633c_codec_fw.bin",
	.sram_size = 0x18000,
};

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

static int wave6_vpu_load_firmware(struct device *dev, const char *fw_name)
{
	const struct firmware *fw;
	int ret;
	u32 version;
	u32 revision;
	u32 product_id;

	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "request_firmware fail\n");
		return ret;
	}

	ret = wave6_vpu_init_with_bitcode(dev, (u8 *)fw->data, fw->size);
	if (ret) {
		dev_err(dev, "vpu_init_with_bitcode fail\n");
		goto release_fw;
	}
	release_firmware(fw);

	ret = wave6_vpu_get_version_info(dev, &version, &revision, &product_id);
	if (ret) {
		dev_err(dev, "vpu_get_version_info fail\n");
		goto release_fw;
	}

	dev_err(dev, "enum product_id : %08x\n", product_id);
	dev_err(dev, "fw_version : %08x(r%d)\n", version, revision);

	return 0;

release_fw:
	release_firmware(fw);
	return ret;
}

static int wave6_vpu_probe(struct platform_device *pdev)
{
	int ret;
	struct vpu_device *dev;
	struct device_node *np;
	const struct wave6_match_data *match_data;

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

	dev->vm_reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->vm_reg_base))
		return PTR_ERR(dev->vm_reg_base);
	dev->gb_reg_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dev->gb_reg_base))
		return PTR_ERR(dev->gb_reg_base);

	ret = devm_clk_bulk_get_all(&pdev->dev, &dev->clks);

	/* continue without clock, assume externally managed */
	if (ret < 0) {
		dev_warn(&pdev->dev, "unable to get clocks: %d\n", ret);
		ret = 0;
	}
	dev->num_clks = ret;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_resume_and_get(&pdev->dev);

	ret = clk_bulk_prepare_enable(dev->num_clks, dev->clks);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clocks: %d\n", ret);
		return ret;
	}

	dev->sram_pool = of_gen_pool_get(pdev->dev.of_node, "sram", 0);
	if (!dev->sram_pool) {
		dev_warn(&pdev->dev, "sram node not found\n");
	} else {
		dev->sram_buf.size = match_data->sram_size;
		dev->sram_buf.vaddr = gen_pool_dma_alloc(dev->sram_pool,
							 dev->sram_buf.size,
							 &dev->sram_buf.phys_addr);
		if (!dev->sram_buf.vaddr)
			dev->sram_buf.size = 0;
		else
			dev->sram_buf.dma_addr = dma_map_resource(&pdev->dev, dev->sram_buf.phys_addr, dev->sram_buf.size, DMA_BIDIRECTIONAL, 0);
		dev_info(&pdev->dev, "sram 0x%pad, 0x%pad, size 0x%lx\n",
			 &dev->sram_buf.phys_addr, &dev->sram_buf.dma_addr, dev->sram_buf.size);
	}

	np = of_parse_phandle(pdev->dev.of_node, "boot", 0);
	if (np) {
		struct resource mem;

		ret = of_address_to_resource(np, 0, &mem);
		of_node_put(np);
		if (ret) {
			dev_err(&pdev->dev, "boot resource not available.\n");
			goto err_free_sram;
		}

		dev->common_mem.phys_addr = mem.start;
		dev->common_mem.size = resource_size(&mem);
		if (dev->common_mem.size < SIZE_COMMON) {
			dev_err(&pdev->dev, "boot memory size is small.\n");
			goto err_free_sram;
		}

		dev->common_mem.vaddr = devm_memremap(&pdev->dev,
						      dev->common_mem.phys_addr,
						      dev->common_mem.size,
						      MEMREMAP_WC);
		if (!dev->common_mem.vaddr) {
			dev_err(&pdev->dev, "boot memory mapping fail.\n");
			goto err_free_sram;
		}

		dev->common_mem.dma_addr = dma_map_resource(&pdev->dev,
							    dev->common_mem.phys_addr,
							    dev->common_mem.size,
							    DMA_BIDIRECTIONAL,
							    0);
		if (!dev->common_mem.dma_addr) {
			dev_err(&pdev->dev, "boot memory dma map resource fail.\n");
			goto err_free_sram;
		}

		dev_info(&pdev->dev, "boot phys_addr: %pad, dma_addr: %pad, size: 0x%lx\n",
			 &dev->common_mem.phys_addr,
			 &dev->common_mem.dma_addr,
			 dev->common_mem.size);
	}

	dev->product_code = wave6_vdi_readl(dev, W6_VPU_RET_PRODUCT_VERSION);
	dev->product = wave_vpu_get_product_id(dev);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register fail: %d\n", ret);
		goto err_free_sram;
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

	ret = wave6_vpu_load_firmware(&pdev->dev, match_data->fw_name);
	if (ret) {
		pm_runtime_disable(&pdev->dev);
		dev_err(&pdev->dev, "failed to wave6_vpu_load_firmware: %d\n", ret);
		goto err_kfifo_free;
	}

	dev_dbg(&pdev->dev, "Added wave driver with caps %s %s and product code 0x%x\n",
		match_data->flags & WAVE6_IS_ENC ? "'ENCODE'" : "",
		match_data->flags & WAVE6_IS_DEC ? "'DECODE'" : "",
		dev->product_code);
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
err_free_sram:
	if (dev->sram_pool && dev->sram_buf.vaddr) {
		dma_unmap_resource(&pdev->dev, dev->sram_buf.dma_addr, dev->sram_buf.size, DMA_BIDIRECTIONAL, 0);
		gen_pool_free(dev->sram_pool,
			      (unsigned long)dev->sram_buf.vaddr,
			      dev->sram_buf.size);
	}
	clk_bulk_disable_unprepare(dev->num_clks, dev->clks);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	return ret;
}

static int wave6_vpu_remove(struct platform_device *pdev)
{
	struct vpu_device *dev = dev_get_drvdata(&pdev->dev);

	if (dev->sram_pool && dev->sram_buf.vaddr) {
		dma_unmap_resource(&pdev->dev, dev->sram_buf.dma_addr, dev->sram_buf.size, DMA_BIDIRECTIONAL, 0);
		gen_pool_free(dev->sram_pool,
			      (unsigned long)dev->sram_buf.vaddr,
			      dev->sram_buf.size);
	}
	clk_bulk_disable_unprepare(dev->num_clks, dev->clks);
	wave6_vpu_release_m2m_dev(dev);
	wave6_vpu_enc_unregister_device(dev);
	wave6_vpu_dec_unregister_device(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	kfifo_free(&dev->irq_status);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	if (dev->common_mem.phys_addr) {
		if (dev->common_mem.dma_addr)
			dma_unmap_resource(&pdev->dev,
					   dev->common_mem.dma_addr,
					   dev->common_mem.size,
					   DMA_BIDIRECTIONAL,
					   0);
	} else {
		wave6_vdi_free_dma_memory(dev, (struct vpu_buf *)&dev->common_mem);
	}

	return 0;
}

#ifdef CONFIG_PM
static int wave6_vpu_runtime_suspend(struct device *dev)
{
	return 0;
}

static int wave6_vpu_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int wave6_vpu_suspend(struct device *dev)
{
	return 0;
}

static int wave6_vpu_resume(struct device *dev)
{
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
