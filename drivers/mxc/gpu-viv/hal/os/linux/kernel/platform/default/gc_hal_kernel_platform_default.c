/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2023 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2023 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


/*
 *   dts node example:
 *   gpu_3d: gpu@53100000 {
 *       compatible = "verisilicon,galcore";
 *       reg = <0 0x53100000 0 0x40000>,
 *               <0 0x54100000 0 0x40000>;
 *       reg-names = "core_major", "core_3d1";
 *       interrupts = <GIC_SPI 64 IRQ_TYPE_LEVEL_HIGH>,
 *                   <GIC_SPI 65 IRQ_TYPE_LEVEL_HIGH>;
 *       interrupt-names = "core_major", "core_3d1";
 *       clocks = <&clk IMX_SC_R_GPU_0_PID0 IMX_SC_PM_CLK_PER>,
 *               <&clk IMX_SC_R_GPU_0_PID0 IMX_SC_PM_CLK_MISC>,
 *               <&clk IMX_SC_R_GPU_1_PID0 IMX_SC_PM_CLK_PER>,
 *               <&clk IMX_SC_R_GPU_1_PID0 IMX_SC_PM_CLK_MISC>;
 *       clock-names = "core_major", "core_major_sh", "core_3d1", "core_3d1_sh";
 *       assigned-clocks = <&clk IMX_SC_R_GPU_0_PID0 IMX_SC_PM_CLK_PER>,
 *                       <&clk IMX_SC_R_GPU_0_PID0 IMX_SC_PM_CLK_MISC>,
 *                       <&clk IMX_SC_R_GPU_1_PID0 IMX_SC_PM_CLK_PER>,
 *                       <&clk IMX_SC_R_GPU_1_PID0 IMX_SC_PM_CLK_MISC>;
 *       assigned-clock-rates = <700000000>, <850000000>, <800000000>, <1000000000>;
 *       power-domains = <&pd IMX_SC_R_GPU_0_PID0>, <&pd IMX_SC_R_GPU_1_PID0>;
 *       power-domain-names = "core_major", "core_3d1";
 *       contiguous-base = <0x0>;
 *       contiguous-size = <0x1000000>;
 *       status = "okay";
 *   };
 */

#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_platform.h"
#include "gc_hal_kernel_platform_default.h"
#if gcdSUPPORT_DEVICE_TREE_SOURCE
#   include <linux/pm_runtime.h>
#   include <linux/pm_domain.h>
#   include <linux/clk.h>
#endif

/* Disable MSI for internal FPGA build except PPC */
#if gcdFPGA_BUILD
#    define USE_MSI         0
#else
#    define USE_MSI         1
#endif

#define gcdMIXED_PLATFORM   0

#define gcdDISABLE_NODE_OFFSET 1

gceSTATUS
_AdjustParam(IN gcsPLATFORM *Platform, OUT gcsMODULE_PARAMETERS *Args);

gceSTATUS
_GetGPUPhysical(IN gcsPLATFORM     *Platform,
                IN gctPHYS_ADDR_T   CPUPhysical,
                OUT gctPHYS_ADDR_T *GPUPhysical);

#if gcdENABLE_MP_SWITCH
gceSTATUS
_SwitchCoreCount(IN gcsPLATFORM *Platform, OUT gctUINT32 *Count);
#endif

#if gcdENABLE_VIDEO_MEMORY_MIRROR
gceSTATUS
_dmaCopy(gctPOINTER Object, gcsDMA_TRANS_INFO *Info);
#endif

#if gcdSUPPORT_DEVICE_TREE_SOURCE
static int gpu_parse_dt(struct platform_device *pdev, gcsMODULE_PARAMETERS *params);

gceSTATUS
_set_power(IN gcsPLATFORM *Platform, IN gctUINT32 DevIndex, IN gceCORE GPU, IN gctBOOL Enable);

gceSTATUS
_set_clock(gcsPLATFORM *Platform, gctUINT32 DevIndex, gceCORE GPU, gctBOOL Enable);
#endif

static struct _gcsPLATFORM_OPERATIONS default_ops = {
    .adjustParam        = _AdjustParam,
    .getGPUPhysical     = _GetGPUPhysical,
#if gcdENABLE_MP_SWITCH
    .switchCoreCount    = _SwitchCoreCount,
#endif
#if gcdENABLE_VIDEO_MEMORY_MIRROR
    .dmaCopy            = _dmaCopy,
#endif
#if gcdSUPPORT_DEVICE_TREE_SOURCE
    .setPower           = _set_power,
    .setClock           = _set_clock,
#endif
};

#if gcdSUPPORT_DEVICE_TREE_SOURCE
struct gpu_clk {
    struct clk *core;
    struct clk *shader;
};

struct gpu_power_domain {
    int num_domains;
    int local_core_index[gcvCORE_COUNT];
    struct device **power_dev;
    struct gpu_clk *gclk;
};

static struct _gcsPLATFORM default_platform = {
    .name = __FILE__,
    .ops  = &default_ops,
};

const char *core_names[] = {
    "core_major",
    "core_3d1",
    "core_3d2",
    "core_3d3",
    "core_3d4",
    "core_3d5",
    "core_3d6",
    "core_3d7",
    "core_3d8",
    "core_3d9",
    "core_3d10",
    "core_3d11",
    "core_3d12",
    "core_3d13",
    "core_3d14",
    "core_3d15",
    "core_2d",
    "core_2d1",
    "core_2d2",
    "core_2d3",
    "core_vg",
#if gcdDEC_ENABLE_AHB
    "core_dec",
#endif
};

struct gpu_power_domain gpd;

static inline int get_global_core_index(const char* name)
{
    int i = 0;

    for (i = 0; i < gcvCORE_COUNT; i++) {
        if (!strcmp(name, core_names[i]))
            return i;
    }
    return -1;
}

gceSTATUS
_set_clock(gcsPLATFORM *Platform, gctUINT32 DevIndex, gceCORE GPU, gctBOOL Enable)
{
    int sub_index = gpd.local_core_index[GPU];

    if (Enable) {
        if (gpd.gclk[sub_index].core) {
            clk_prepare(gpd.gclk[sub_index].core);
            clk_enable(gpd.gclk[sub_index].core);
        }
        if (gpd.gclk[sub_index].shader) {
            clk_prepare(gpd.gclk[sub_index].shader);
            clk_enable(gpd.gclk[sub_index].shader);
        }
    } else {
        if (gpd.gclk[sub_index].core) {
            clk_disable(gpd.gclk[sub_index].core);
            clk_unprepare(gpd.gclk[sub_index].core);
        }
        if (gpd.gclk[sub_index].shader) {
            clk_disable(gpd.gclk[sub_index].shader);
            clk_unprepare(gpd.gclk[sub_index].shader);
        }
    }

    return gcvSTATUS_OK;
}

gceSTATUS
_set_power(IN gcsPLATFORM *Platform, IN gctUINT32 DevIndex, IN gceCORE GPU, IN gctBOOL Enable)
{
    int num_domains = gpd.num_domains;

    if (num_domains > 1) {
        int sub_index = gpd.local_core_index[GPU];
        struct device *sub_dev = gpd.power_dev[sub_index];

        if (Enable)
            pm_runtime_get_sync(sub_dev);
        else
            pm_runtime_put(sub_dev);
    }

    if (num_domains == 1) {
        if (Enable)
            pm_runtime_get_sync(&Platform->device->dev);
        else
            pm_runtime_put(&Platform->device->dev);
    }
    return gcvSTATUS_OK;
}

static int gpu_remove_power_domains(struct platform_device *pdev)
{
    int i = 0;

    for (i = 0; i < gpd.num_domains; i++) {
        if (gpd.gclk[i].core) {
            clk_put(gpd.gclk[i].core);
            gpd.gclk[i].core = NULL;
        }

        if (gpd.gclk[i].shader) {
            clk_put(gpd.gclk[i].shader);
            gpd.gclk[i].shader = NULL;
        }

        if (gpd.power_dev) {
            pm_runtime_disable(gpd.power_dev[i]);
            dev_pm_domain_detach(gpd.power_dev[i], true);
        }
    }

    if (gpd.num_domains == 1)
        pm_runtime_disable(&pdev->dev);

    return 0;
}

static int gpu_add_power_domains(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int i, index = 0;
    int num_domains = 0;
    int ret = 0;
    const char **string_names;
    char temp[20];

    memset(&gpd, 0, sizeof(struct gpu_power_domain));
    num_domains = of_count_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells");
    gpd.num_domains = num_domains;

    /* If the num of domains is less than 2, the domain will be attached automatically */
    if (num_domains > 1) {
        gpd.power_dev = devm_kcalloc(dev, num_domains, sizeof(struct device *), GFP_KERNEL);
        if (!gpd.power_dev)
            return -ENOMEM;
    }

    gpd.gclk = devm_kcalloc(dev, num_domains, sizeof(struct gpu_clk), GFP_KERNEL);
    if (!gpd.gclk)
        return -ENOMEM;

    string_names = devm_kcalloc(dev, num_domains, sizeof(*string_names), GFP_KERNEL);
    if (!string_names)
        return -ENOMEM;

    ret = of_property_read_string_array(dev->of_node, "power-domain-names", string_names, num_domains);
    if (ret != num_domains) {
        gcmkPRINT("failed to read power-domain-names\n");
        return -EINVAL;
    }

    for (i = 0; i < num_domains; i++) {
        if (gpd.power_dev) {
            gpd.power_dev[i] = dev_pm_domain_attach_by_id(dev, i);
            if (IS_ERR(gpd.power_dev[i]))
                goto error;
        }

        index = get_global_core_index(string_names[i]);
        gpd.local_core_index[index] = i;
        //pm_runtime_enable(gpd.power_dev[i]);

        gpd.gclk[i].core = clk_get(dev, string_names[i]);
        if (IS_ERR(gpd.gclk[i].core))
            gpd.gclk[i].core = NULL;

        memset(temp, 0, sizeof(temp));
        sprintf(temp, "%s_sh", string_names[i]);
        gpd.gclk[i].shader = clk_get(dev, temp);
        if (IS_ERR(gpd.gclk[i].shader))
            gpd.gclk[i].shader = NULL;
    }

    if (num_domains == 1)
        pm_runtime_enable(&pdev->dev);

    return 0;

error:
    for (i = 0; i < num_domains; i++) {
        if (gpd.power_dev[i])
            dev_pm_domain_detach(gpd.power_dev[i], true);
    }
    return ret;
}

static int gpu_parse_dt(struct platform_device *pdev, gcsMODULE_PARAMETERS *params)
{
    struct device_node *root = pdev->dev.of_node;
    struct resource    *res;
    gctUINT32           i, data;
    const gctUINT32    *value;

    gcmSTATIC_ASSERT(gcvCORE_COUNT == gcmCOUNTOF(core_names),
                     "core_names array does not match core types");

    /* parse the irqs config */
    for (i = 0; i < gcvCORE_COUNT; i++) {
        res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, core_names[i]);
        if (res)
            params->irqs[i] = res->start;
    }

    /* parse the registers config */
    for (i = 0; i < gcvCORE_COUNT; i++) {
        res = platform_get_resource_byname(pdev, IORESOURCE_MEM, core_names[i]);
        if (res) {
            params->registerBases[i] = res->start;
            params->registerSizes[i] = res->end - res->start + 1;
        }
    }

    /* parse the contiguous mem */
    value = of_get_property(root, "contiguous-size", gcvNULL);
    if (value && *value != 0) {
        gctUINT64 addr;

        of_property_read_u64(root, "contiguous-base", &addr);
        params->contiguousSize = *value;
        params->contiguousBase = addr;
    }

    value = of_get_property(root, "contiguous-requested", gcvNULL);
    if (value)
        params->contiguousRequested = *value ? gcvTRUE : gcvFALSE;

    /* parse the external mem */
    value = of_get_property(root, "external-size", gcvNULL);
    if (value && *value != 0) {
        gctUINT64 addr;

        of_property_read_u64(root, "external-base", &addr);
        params->externalSize[0] = *value;
        params->externalBase[0] = addr;
    }

    value = of_get_property(root, "phys-size", gcvNULL);
    if (value && *value != 0) {
        gctUINT64 addr;

        of_property_read_u64(root, "base-address", &addr);
        params->physSize    = *value;
        params->baseAddress = addr;
    }

    value = of_get_property(root, "phys-size", gcvNULL);
    if (value)
        params->bankSize = *value;

    value = of_get_property(root, "recovery", gcvNULL);
    if (value)
        params->recovery = *value;

    value = of_get_property(root, "power-management", gcvNULL);
    if (value)
        params->powerManagement = *value;

    value = of_get_property(root, "enable-mmu", gcvNULL);
    if (value)
        params->enableMmu = *value;

    for (i = 0; i < gcvCORE_3D_MAX; i++) {
        data = 0;
        of_property_read_u32_index(root, "user-cluster-masks", i, &data);
        if (data)
            params->userClusterMasks[i] = data;
    }

    value = of_get_property(root, "stuck-dump", gcvNULL);
    if (value)
        params->stuckDump = *value;

    value = of_get_property(root, "show-args", gcvNULL);
    if (value)
        params->showArgs = *value;

    value = of_get_property(root, "mmu-page-table-pool", gcvNULL);
    if (value)
        params->mmuPageTablePool = *value;

    value = of_get_property(root, "mmu-dynamic-map", gcvNULL);
    if (value)
        params->mmuDynamicMap = *value;

    value = of_get_property(root, "all-map-in-one", gcvNULL);
    if (value)
        params->allMapInOne = *value;

    value = of_get_property(root, "isr-poll-mask", gcvNULL);
    if (value)
        params->isrPoll = *value;

    return 0;
}

static const struct of_device_id gpu_dt_ids[] = {
    { .compatible = "verisilicon,galcore", },

    { /* sentinel */ }
};

#elif USE_LINUX_PCIE

typedef struct _gcsBARINFO {
    gctPHYS_ADDR_T  base;
    gctPOINTER      logical;
    gctUINT64       size;
    gctBOOL         available;
    gctUINT64       reg_max_offset;
    gctUINT32       reg_size;
} gcsBARINFO, *gckBARINFO;

struct _gcsPCIEInfo {
    gcsBARINFO      bar[gcdMAX_PCIE_BAR];
    struct pci_dev *pdev;
    gctPHYS_ADDR_T  sram_base;
    gctPHYS_ADDR_T  sram_gpu_base;
    uint32_t        sram_size;
    int             sram_bar;
    int             sram_offset;
};

struct _gcsPLATFORM_PCIE {
    struct _gcsPLATFORM base;
    struct _gcsPCIEInfo pcie_info[gcdDEVICE_COUNT];
    unsigned int        device_number;
};

struct _gcsPLATFORM_PCIE default_platform = {
    .base = {
        .name = __FILE__,
        .ops  = &default_ops,
    },
};

static gctINT
_QueryBarInfo(struct pci_dev *Pdev,
              gctPHYS_ADDR_T *BarAddr,
              gctUINT64      *BarSize,
              gctUINT         BarNum)
{
    gctUINT   addr, size;
    gctINT    is_64_bit = 0;
    gctUINT64 addr64, size64;

    /* Read the bar address */
    if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, &addr) < 0)
        return -1;

    /* Read the bar size */
    if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, 0xffffffff) < 0)
        return -1;

    if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, &size) < 0)
        return -1;

    /* Write back the bar address */
    if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, addr) < 0)
        return -1;

    /* The bar is not working properly */
    if (size == 0xffffffff)
        return -1;

    if ((size & PCI_BASE_ADDRESS_MEM_TYPE_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64)
        is_64_bit = 1;

    addr64 = addr;
    size64 = size;

    if (is_64_bit) {
        /* Read the bar address */
        if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + (BarNum + 1) * 0x4, &addr) < 0)
            return -1;

        /* Read the bar size */
        if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + (BarNum + 1) * 0x4, 0xffffffff) < 0)
            return -1;

        if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + (BarNum + 1) * 0x4, &size) < 0)
            return -1;

        /* Write back the bar address */
        if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + (BarNum + 1) * 0x4, addr) < 0)
            return -1;

        addr64 |= ((gctUINT64)addr << 32);
        size64 |= ((gctUINT64)size << 32);
    }

    size64 &= ~0xfULL;
    size64 = ~size64;
    size64 += 1;
    addr64 &= ~0xfULL;

    gcmkPRINT("Bar%d addr=0x%08lx size=0x%08lx", BarNum, addr64, size64);

    *BarAddr = addr64;
    *BarSize = size64;

    return is_64_bit;
}

static const struct pci_device_id vivpci_ids[] = {
  {
    .class       = 0x000000,
    .class_mask  = 0x000000,
    .vendor      = 0x10ee,
    .device      = 0x7012,
    .subvendor   = PCI_ANY_ID,
    .subdevice   = PCI_ANY_ID,
    .driver_data = 0
  },
  {
    .class       = 0x000000,
    .class_mask  = 0x000000,
    .vendor      = 0x10ee,
    .device      = 0x7014,
    .subvendor   = PCI_ANY_ID,
    .subdevice   = PCI_ANY_ID,
    .driver_data = 0
  },
  { /* End: all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, vivpci_ids);

static int
gpu_sub_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
    static u64 dma_mask = DMA_BIT_MASK(40);
#    else
    static u64 dma_mask = DMA_40BIT_MASK;
#    endif

    gcmkPRINT("PCIE DRIVER PROBED");
    if (pci_enable_device(pdev))
        pr_err("galcore: pci_enable_device() failed.\n");

    if (pci_set_dma_mask(pdev, dma_mask))
        pr_err("galcore: Failed to set DMA mask.\n");

    pci_set_master(pdev);

    if (pci_request_regions(pdev, "galcore"))
        pr_err("galcore: Failed to get ownership of BAR region.\n");

#if USE_MSI
    if (pci_enable_msi(pdev))
        pr_err("galcore: Failed to enable MSI.\n");
#    endif

#if defined(CONFIG_PPC)
    /* On PPC platform, enable bus master, enable irq. */
    if (pci_write_config_word(pdev, 0x4, 0x0006) < 0)
        pr_err("galcore: Failed to enable bus master on PPC.\n");
#    endif

    default_platform.pcie_info[default_platform.device_number++].pdev = pdev;
    return 0;
}

static void
gpu_sub_remove(struct pci_dev *pdev)
{
    pci_set_drvdata(pdev, NULL);
#if USE_MSI
    pci_disable_msi(pdev);
#    endif
    pci_clear_master(pdev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    return;
}

static struct pci_driver gpu_pci_subdriver = {
    .name     = DEVICE_NAME,
    .id_table = vivpci_ids,
    .probe    = gpu_sub_probe,
    .remove   = gpu_sub_remove
};

#else
static struct _gcsPLATFORM default_platform = {
    .name = __FILE__,
    .ops  = &default_ops,
};
#endif

gceSTATUS
_AdjustParam(IN gcsPLATFORM *Platform, OUT gcsMODULE_PARAMETERS *Args)
{
#if gcdSUPPORT_DEVICE_TREE_SOURCE
    gpu_parse_dt(Platform->device, Args);
    gpu_add_power_domains(Platform->device);
#elif USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform  = (struct _gcsPLATFORM_PCIE *)Platform;
    struct pci_dev           *pdev           = pcie_platform->pcie_info[0].pdev;
    int                       irqline        = pdev->irq;
    unsigned int              i, core = 0;
    unsigned int              core_count     = 0;
    unsigned int              core_2d_count  = 0;
    unsigned int              core_2d        = 0;
    gctPOINTER                ptr            = gcvNULL;
    int                       sram_bar, sram_offset;
    unsigned int              dev_index      = 0;
    unsigned int              pdev_index, bar_index = 0;
    unsigned int              index          = 0;
    unsigned int              hw_dev_index   = 0;
    unsigned long             reg_max_offset = 0;
    unsigned int              reg_size       = 0;
    int                       ret;

    if (Args->irqs[gcvCORE_2D] != -1)
        Args->irqs[gcvCORE_2D] = irqline;
    if (Args->irqs[gcvCORE_MAJOR] != -1)
        Args->irqs[gcvCORE_MAJOR] = irqline;

#if gcdMIXED_PLATFORM
    /* Fill the SOC platform paramters first. */
    {
        hw_dev_index += Args->hwDevCounts[dev_index];
        for (; index < hw_dev_index; index++) {
            core_count += Args->devCoreCounts[index];

            for (; core < core_count; core++) {
                /* Fill the SOC platform parameters here. */
                Args->irqs[core] = -1;
                Args->registerBasesMapped[core] = 0;
                Args->registerSizes[core] = 0;
            }
        }

        dev_index++;
    }
#endif

    for (pdev_index = 0; pdev_index < pcie_platform->device_number; pdev_index++) {
        struct pci_dev *pcie_dev = pcie_platform->pcie_info[pdev_index].pdev;

        memset(&pcie_platform->pcie_info[pdev_index].bar[0], 0, sizeof(gcsBARINFO) * gcdMAX_PCIE_BAR);

        Args->devices[dev_index] = &pcie_dev->dev;

        for (i = 0; i < gcdMAX_PCIE_BAR; i++) {
            ret = _QueryBarInfo(pcie_dev,
                                &pcie_platform->pcie_info[pdev_index].bar[i].base,
                                &pcie_platform->pcie_info[pdev_index].bar[i].size,
                                i);

            if (ret < 0)
                continue;

            pcie_platform->pcie_info[pdev_index].bar[i].available = gcvTRUE;
            i += ret;
        }

        hw_dev_index += Args->hwDevCounts[dev_index];

        for (; index < hw_dev_index; index++) {
            core_count += Args->devCoreCounts[index];

            for (i = 0; i < core_count; i++) {
                if (Args->bars[i] != -1 &&
                    Args->regOffsets[i] > pcie_platform->pcie_info[pdev_index].bar[bar_index].reg_max_offset) {
                    bar_index = Args->bars[i];
                    pcie_platform->pcie_info[pdev_index].bar[bar_index].reg_max_offset = Args->regOffsets[i];
                    pcie_platform->pcie_info[pdev_index].bar[bar_index].reg_size = Args->registerSizes[i];
                }
            }

            for (; core < core_count; core++) {
                if (Args->bars[core] != -1) {
                    bar_index = Args->bars[core];
                    Args->irqs[core] = pcie_dev->irq;

                    gcmkASSERT(pcie_platform->pcie_info[pdev_index].bar[bar_index].available);

                    if (Args->regOffsets[core]) {
                        gcmkASSERT(Args->regOffsets[core] + Args->registerSizes[core] <
                                   pcie_platform->pcie_info[pdev_index].bar[bar_index].size);
                    }

                    ptr =  pcie_platform->pcie_info[pdev_index].bar[bar_index].logical;
                    if (!ptr) {
                        reg_max_offset =
                            pcie_platform->pcie_info[pdev_index].bar[bar_index].reg_max_offset;
                        reg_size = reg_max_offset == 0 ?
                                   Args->registerSizes[core] :
                                   pcie_platform->pcie_info[pdev_index].bar[bar_index].reg_size;
                        ptr = pcie_platform->pcie_info[pdev_index].bar[bar_index].logical =
                            (gctPOINTER)pci_iomap(pcie_dev, bar_index, reg_max_offset + reg_size);
                    }

                    if (ptr) {
                        Args->registerBasesMapped[core] =
                            (gctPOINTER)((gctCHAR *)ptr + Args->regOffsets[core]);
                    }
                }
            }

            core_2d_count += Args->dev2DCoreCounts[index];

            for (; core_2d < core_2d_count; core_2d++) {
                if (Args->bar2Ds[core_2d] != -1) {
                    bar_index = Args->bar2Ds[core_2d];
                    Args->irq2Ds[core_2d] = pcie_dev->irq;

                    if (Args->reg2DOffsets[core_2d]) {
                        gcmkASSERT(Args->reg2DOffsets[core_2d] + Args->register2DSizes[core_2d] <
                                   pcie_platform->pcie_info[pdev_index].bar[bar_index].size);
                    }

                    ptr = pcie_platform->pcie_info[pdev_index].bar[bar_index].logical;
                    if (!ptr) {
                        ptr = pcie_platform->pcie_info[pdev_index].bar[bar_index].logical =
                            (gctPOINTER)pci_iomap(pcie_dev, bar_index, Args->register2DSizes[core_2d]);
                    }

                    if (ptr) {
                        Args->register2DBasesMapped[core_2d] =
                            (gctPOINTER)((gctCHAR *)ptr + Args->reg2DOffsets[core_2d]);
                    }
                }
            }

            if (Args->barVG != -1) {
                bar_index = Args->barVG;
                Args->irqVG = pcie_dev->irq;

                if (Args->regVGOffset) {
                    gcmkASSERT(Args->regVGOffset + Args->registerVGSize <
                               pcie_platform->pcie_info[pdev_index].bar[bar_index].size);
                }

                ptr = pcie_platform->pcie_info[pdev_index].bar[bar_index].logical;
                if (!ptr) {
                    ptr = pcie_platform->pcie_info[pdev_index].bar[bar_index].logical =
                        (gctPOINTER)pci_iomap(pcie_dev, bar_index, Args->registerVGSize);
                }

                if (ptr) {
                    Args->registerVGBaseMapped =
                        (gctPOINTER)((gctCHAR *)ptr + Args->regVGOffset);
                }
            }

            /* All the PCIE devices AXI-SRAM should have same base address. */
            pcie_platform->pcie_info[pdev_index].sram_base     = Args->extSRAMBases[pdev_index];
            pcie_platform->pcie_info[pdev_index].sram_gpu_base = Args->extSRAMBases[pdev_index];

            pcie_platform->pcie_info[pdev_index].sram_size     = Args->extSRAMSizes[pdev_index];

            pcie_platform->pcie_info[pdev_index].sram_bar      = Args->sRAMBars[pdev_index];
            sram_bar                                           = Args->sRAMBars[pdev_index];
            pcie_platform->pcie_info[pdev_index].sram_offset   = Args->sRAMOffsets[pdev_index];
            sram_offset                                        = Args->sRAMOffsets[pdev_index];
        }

        /* Get CPU view SRAM base address from bar address and bar inside offset. */
        if (sram_bar != -1 && sram_offset != -1) {
            gcmkASSERT(pcie_platform->pcie_info[pdev_index].bar[sram_bar].available);
            pcie_platform->pcie_info[pdev_index].sram_base = pcie_platform->pcie_info[pdev_index].bar[sram_bar].base
                                                           + sram_offset;
            Args->extSRAMBases[pdev_index] = pcie_platform->pcie_info[pdev_index].bar[sram_bar].base + sram_offset;
        }

        dev_index++;
    }

    Args->contiguousRequested = gcvTRUE;
#endif
    return gcvSTATUS_OK;
}

gceSTATUS
_GetGPUPhysical(IN gcsPLATFORM     *Platform,
                IN gctPHYS_ADDR_T   CPUPhysical,
                OUT gctPHYS_ADDR_T *GPUPhysical)
{
#if gcdSUPPORT_DEVICE_TREE_SOURCE
#elif USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    unsigned int   pdev_index;
    /* Only support 1 external shared SRAM currently. */
    gctPHYS_ADDR_T sram_base;
    gctPHYS_ADDR_T sram_gpu_base;
    uint32_t       sram_size;

    for (pdev_index = 0; pdev_index < pcie_platform->device_number; pdev_index++) {
        sram_base     = pcie_platform->pcie_info[pdev_index].sram_base;
        sram_gpu_base = pcie_platform->pcie_info[pdev_index].sram_gpu_base;
        sram_size     = pcie_platform->pcie_info[pdev_index].sram_size;

        if (!sram_size && Platform->dev && Platform->dev->extSRAMSizes[0])
            sram_size = Platform->dev->extSRAMSizes[0];

        if (sram_base != gcvINVALID_PHYSICAL_ADDRESS &&
            sram_gpu_base != gcvINVALID_PHYSICAL_ADDRESS &&
            sram_size) {
            if (CPUPhysical >= sram_base && (CPUPhysical < (sram_base + sram_size))) {
                *GPUPhysical = CPUPhysical - sram_base + sram_gpu_base;

                return gcvSTATUS_OK;
            }
        }
    }
#endif

    *GPUPhysical = CPUPhysical;

    return gcvSTATUS_OK;
}

#if gcdENABLE_MP_SWITCH
gceSTATUS
_SwitchCoreCount(IN gcsPLATFORM *Platform, OUT gctUINT32 *Count)
{
    *Count = Platform->coreCount;

    return gcvSTATUS_OK;
}
#endif

#if gcdENABLE_VIDEO_MEMORY_MIRROR
gceSTATUS
_dmaCopy(gctPOINTER Object, gcsDMA_TRANS_INFO *Info)
{
    gceSTATUS      status         = gcvSTATUS_OK;
    gckKERNEL      kernel         = (gckKERNEL)Object;
    gckVIDMEM_NODE dst_node_obj   = (gckVIDMEM_NODE)Info->dst_node;
    gckVIDMEM_NODE src_node_obj   = (gckVIDMEM_NODE)Info->src_node;
    gctPOINTER     src_ptr        = gcvNULL, dst_ptr = gcvNULL;
    gctSIZE_T      size0, size1;
    gctPOINTER     src_mem_handle = gcvNULL, dst_mem_handle = gcvNULL;
    gctBOOL        src_need_unmap = gcvFALSE, dst_need_unmap = gcvFALSE;

    gcsPLATFORM *platform = kernel->os->device->platform;
#if USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)platform;
    struct pci_dev *pdev = pcie_platform->pcie_info[kernel->device->platformIndex].pdev;

    /* Make compiler happy. */
    pdev = pdev;
#else
    /* Make compiler happy. */
    platform = platform;
#endif

#if gcdDISABLE_NODE_OFFSET
    Info->offset = 0;
#endif

    status = gckVIDMEM_NODE_GetSize(kernel, src_node_obj, &size0);
    if (status)
        return status;

    status = gckVIDMEM_NODE_GetSize(kernel, dst_node_obj, &size1);
    if (status)
        return status;

    status = gckVIDMEM_NODE_GetMemoryHandle(kernel, src_node_obj, &src_mem_handle);
    if (status)
        return status;

    status = gckVIDMEM_NODE_GetMemoryHandle(kernel, dst_node_obj, &dst_mem_handle);
    if (status)
        return status;

    status = gckVIDMEM_NODE_GetMapKernel(kernel, src_node_obj, &src_ptr);
    if (status)
        return status;

    if (!src_ptr) {
        gctSIZE_T offset;

        status = gckVIDMEM_NODE_GetOffset(kernel, src_node_obj, &offset);
        if (status)
            return status;

        offset += Info->offset;
        status = gckOS_CreateKernelMapping(kernel->os, src_mem_handle, offset, size0, &src_ptr);

        if (status)
            goto error;

        src_need_unmap = gcvTRUE;
    } else
        src_ptr += Info->offset;

    status = gckVIDMEM_NODE_GetMapKernel(kernel, dst_node_obj, &dst_ptr);
    if (status)
        return status;

    if (!dst_ptr) {
        gctSIZE_T offset;

        status = gckVIDMEM_NODE_GetOffset(kernel, dst_node_obj, &offset);
        if (status)
            return status;

        offset += Info->offset;
        status = gckOS_CreateKernelMapping(kernel->os, dst_mem_handle, offset, size1, &dst_ptr);

        if (status)
            goto error;

        dst_need_unmap = gcvTRUE;
    } else
        dst_ptr += Info->offset;


#if gcdDISABLE_NODE_OFFSET
    gckOS_MemCopy(dst_ptr, src_ptr, gcmMIN(size0, size1));
#else
    gckOS_MemCopy(dst_ptr, src_ptr, gcmMIN(Info->bytes, gcmMIN(size0, size1)));
#endif

error:
    if (src_need_unmap && src_ptr)
        gckOS_DestroyKernelMapping(kernel->os, src_mem_handle, src_ptr);

    if (dst_need_unmap && dst_ptr)
        gckOS_DestroyKernelMapping(kernel->os, dst_mem_handle, dst_ptr);

    return status;
}
#endif

int gckPLATFORM_Init(struct platform_driver *pdrv, struct _gcsPLATFORM **platform)
{
    int ret = 0;
#if !gcdSUPPORT_DEVICE_TREE_SOURCE
    struct platform_device *default_dev = platform_device_alloc(pdrv->driver.name, -1);

    if (!default_dev) {
        pr_err("galcore: platform_device_alloc failed.\n");
        return -ENOMEM;
    }

    /* Add device */
    ret = platform_device_add(default_dev);
    if (ret) {
        pr_err("galcore: platform_device_add failed.\n");
        platform_device_put(default_dev);
        return ret;
    }

#if USE_LINUX_PCIE
    ret = pci_register_driver(&gpu_pci_subdriver);
    if (ret) {
        platform_device_unregister(default_dev);
        return ret;
    }
#    endif
#else
    pdrv->driver.of_match_table = gpu_dt_ids;
#endif

    *platform = (gcsPLATFORM *)&default_platform;
    return ret;
}

int gckPLATFORM_Terminate(struct _gcsPLATFORM *platform)
{
#if !gcdSUPPORT_DEVICE_TREE_SOURCE
#if USE_LINUX_PCIE
    unsigned int              dev_index;
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)platform;

    for (dev_index = 0; dev_index < pcie_platform->device_number; dev_index++) {
        unsigned int i;

        for (i = 0; i < gcdMAX_PCIE_BAR; i++) {
            if (pcie_platform->pcie_info[dev_index].bar[i].logical != 0)
                pci_iounmap(pcie_platform->pcie_info[dev_index].pdev,
                            pcie_platform->pcie_info[dev_index].bar[i].logical);
        }
    }

    pci_unregister_driver(&gpu_pci_subdriver);
#    endif
    if (platform->device) {
        platform_device_unregister(platform->device);
        platform->device = NULL;
    }
#else
    gpu_remove_power_domains(platform->device);
#endif
    return 0;
}
