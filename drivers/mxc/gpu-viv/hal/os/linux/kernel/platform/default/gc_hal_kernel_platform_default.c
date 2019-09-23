/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2019 Vivante Corporation
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
*    Copyright (C) 2014 - 2019 Vivante Corporation
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


#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_platform.h"

/* Disable MSI for internal FPGA build except PPC */
#if gcdFPGA_BUILD && !defined(CONFIG_PPC)
#define USE_MSI     0
#else
#define USE_MSI     1
#endif

gceSTATUS
_AdjustParam(
    IN gcsPLATFORM *Platform,
    OUT gcsMODULE_PARAMETERS *Args
    );

gceSTATUS
_GetGPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T CPUPhysical,
    OUT gctPHYS_ADDR_T *GPUPhysical
    );

static struct _gcsPLATFORM_OPERATIONS default_ops =
{
    .adjustParam   = _AdjustParam,
    .getGPUPhysical = _GetGPUPhysical,
};

#if USE_LINUX_PCIE

#define MAX_PCIE_DEVICE 4
#define MAX_PCIE_BAR    6

typedef struct _gcsBARINFO
{
    gctPHYS_ADDR_T base;
    gctSIZE_T size;
    gctPOINTER logical;
}
gcsBARINFO, *gckBARINFO;

struct _gcsPCIEInfo
{
    gcsBARINFO bar[MAX_PCIE_BAR];
    struct pci_dev *pdev;
    gctPHYS_ADDR_T sram_bases[gcvSRAM_EXT_COUNT];
    gctPHYS_ADDR_T sram_gpu_bases[gcvSRAM_EXT_COUNT];
    uint32_t sram_sizes[gcvSRAM_EXT_COUNT];
    int sram_bars[gcvSRAM_EXT_COUNT];
    int sram_offsets[gcvSRAM_EXT_COUNT];
};

struct _gcsPLATFORM_PCIE
{
    struct _gcsPLATFORM base;
    struct _gcsPCIEInfo pcie_info[MAX_PCIE_DEVICE];
    unsigned int device_number;
};


struct _gcsPLATFORM_PCIE default_platform =
{
    .base =
    {
        .name = __FILE__,
        .ops  = &default_ops,
    },
};

void
_QueryBarInfo(
    struct pci_dev *Pdev,
    gctPHYS_ADDR_T *BarAddr,
    gctSIZE_T *BarSize,
    gctUINT BarNum
    )
{
    gctUINT addr;
    gctUINT size;

    /* Read the bar address */
    if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, &addr) < 0)
    {
        return;
    }

    /* Read the bar size */
    if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, 0xffffffff) < 0)
    {
        return;
    }

    if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, &size) < 0)
    {
        return;
    }

    size &= 0xfffffff0;
    size  = ~size;
    size += 1;

    /* Write back the bar address */
    if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, addr) < 0)
    {
        return;
    }

    gcmkPRINT("Bar%d addr=0x%x size=0x%x", BarNum, addr, size);

    *BarAddr = addr;
    *BarSize = size;
}

#else

static struct _gcsPLATFORM default_platform =
{
    .name = __FILE__,
    .ops  = &default_ops,
};
#endif

gceSTATUS
_AdjustParam(
    IN gcsPLATFORM *Platform,
    OUT gcsMODULE_PARAMETERS *Args
    )
{
#if USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    struct pci_dev *pdev = pcie_platform->pcie_info[0].pdev;
    unsigned char irqline = pdev->irq;
    unsigned int i;

    unsigned int dev_index, core_index = 0;
    int sram_bar, sram_offset;

    if (Args->irqs[gcvCORE_2D] != -1)
    {
        Args->irqs[gcvCORE_2D] = irqline;
    }
    if (Args->irqs[gcvCORE_MAJOR] != -1)
    {
        Args->irqs[gcvCORE_MAJOR] = irqline;
    }

    for (dev_index = 0; dev_index < pcie_platform->device_number; dev_index++)
    {
        struct pci_dev * pcieDev = pcie_platform->pcie_info[dev_index].pdev;

        for (i = 0; i < MAX_PCIE_BAR; i++)
        {
            _QueryBarInfo(
                pcieDev,
                &pcie_platform->pcie_info[dev_index].bar[i].base,
                &pcie_platform->pcie_info[dev_index].bar[i].size,
                i
                );
        }

        for (i = 0; i < gcvCORE_COUNT; i++)
        {
            if (Args->bars[i] != -1)
            {
                Args->irqs[core_index] = pcieDev->irq;

                /* VIV bitfile: Merge last 4 cores to last one bar to support 8 cores. */
                if (Args->bars[i] == 5)
                {
                    Args->registerBasesMapped[4] =
                    pcie_platform->pcie_info[dev_index].bar[i].logical =
                        (gctPOINTER)pci_iomap(pcieDev, Args->bars[i], 0x500000);
                    Args->registerBasesMapped[5] = Args->registerBasesMapped[4] + 0x100000;
                    Args->registerBasesMapped[6] = Args->registerBasesMapped[5] + 0x100000;
                    Args->registerBasesMapped[7] = Args->registerBasesMapped[6] + 0x100000;

                    Args->irqs[5] =
                    Args->irqs[6] =
                    Args->irqs[7] = Args->irqs[4];

                    continue;
                }

                if (Args->regOffsets[i])
                {
                    gcmkASSERT(Args->regOffsets[i] + Args->registerSizes[core_index]
                               < pcie_platform->pcie_info[dev_index].bar[Args->bars[i]].size);
                }

                Args->registerBasesMapped[core_index] =
                pcie_platform->pcie_info[dev_index].bar[i].logical =
                    (gctPOINTER)pci_iomap(pcieDev, Args->bars[i], Args->registerSizes[core_index] + Args->regOffsets[i]) + Args->regOffsets[i];

                core_index++;
            }
        }

        for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
        {
            pcie_platform->pcie_info[dev_index].sram_bases[i] =
            pcie_platform->pcie_info[dev_index].sram_gpu_bases[i] = Args->extSRAMBases[i];

            pcie_platform->pcie_info[dev_index].sram_sizes[i] = Args->extSRAMSizes[i];

            pcie_platform->pcie_info[dev_index].sram_bars[i] = sram_bar = Args->sRAMBars[i];
            pcie_platform->pcie_info[dev_index].sram_offsets[i] = sram_offset = Args->sRAMOffsets[i];

            /* Get CPU view SRAM base address from bar address and bar inside offset. */
            if (sram_bar != -1 && sram_offset != -1)
            {
                pcie_platform->pcie_info[dev_index].sram_bases[i] = Args->extSRAMBases[i]
                                                                  = pcie_platform->pcie_info[dev_index].bar[sram_bar].base
                                                                  + sram_offset;
            }
        }
    }

    Args->contiguousRequested = gcvTRUE;
#endif
    return gcvSTATUS_OK;
}

gceSTATUS
_GetGPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T CPUPhysical,
    OUT gctPHYS_ADDR_T *GPUPhysical
    )
{
#if USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    /* Only support 1 external shared SRAM currently. */
    gctPHYS_ADDR_T sram_base = pcie_platform->pcie_info[0].sram_bases[0];
    gctPHYS_ADDR_T sram_gpu_base = pcie_platform->pcie_info[0].sram_gpu_bases[0];
    uint32_t sram_size = pcie_platform->pcie_info[0].sram_sizes[0];

    /* TODO: We should always set axi sram size by insmod parameters, never from feature database. */
    if (!sram_size && Platform->dev && Platform->dev->extSRAMSizes[0])
    {
        sram_size = Platform->dev->extSRAMSizes[0];
    }

    if (sram_base != gcvINVALID_PHYSICAL_ADDRESS && sram_gpu_base != gcvINVALID_PHYSICAL_ADDRESS && sram_size)
    {
        if ((CPUPhysical >= sram_base) && (CPUPhysical < (sram_base + sram_size)))
        {
            *GPUPhysical = CPUPhysical - sram_base + sram_gpu_base;
        }
        else
        {
            *GPUPhysical = CPUPhysical;
        }
    }
    else
#endif
    {
        *GPUPhysical = CPUPhysical;
    }

    return gcvSTATUS_OK;
}

#if USE_LINUX_PCIE
static const struct pci_device_id vivpci_ids[] = {
  {
    .class = 0x000000,
    .class_mask = 0x000000,
    .vendor = 0x10ee,
    .device = 0x7012,
    .subvendor = PCI_ANY_ID,
    .subdevice = PCI_ANY_ID,
    .driver_data = 0
  }, { /* End: all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, vivpci_ids);


static int gpu_sub_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
    static u64 dma_mask = DMA_BIT_MASK(40);
#else
    static u64 dma_mask = DMA_40BIT_MASK;
#endif

    gcmkPRINT("PCIE DRIVER PROBED");
    if (pci_enable_device(pdev)) {
        printk(KERN_ERR "galcore: pci_enable_device() failed.\n");
    }

    if (pci_set_dma_mask(pdev, dma_mask)) {
        printk(KERN_ERR "galcore: Failed to set DMA mask.\n");
    }

    pci_set_master(pdev);

    if (pci_request_regions(pdev, "galcore")) {
        printk(KERN_ERR "galcore: Failed to get ownership of BAR region.\n");
    }

#if USE_MSI
    if (pci_enable_msi(pdev)) {
        printk(KERN_ERR "galcore: Failed to enable MSI.\n");
    }
#endif
    default_platform.pcie_info[default_platform.device_number++].pdev = pdev;
    return 0;
}

static void gpu_sub_remove(struct pci_dev *pdev)
{
    pci_set_drvdata(pdev, NULL);
#if USE_MSI
    pci_disable_msi(pdev);
#endif
    pci_clear_master(pdev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    return;
}

static struct pci_driver gpu_pci_subdriver = {
    .name = DEVICE_NAME,
    .id_table = vivpci_ids,
    .probe = gpu_sub_probe,
    .remove = gpu_sub_remove
};

#endif

static struct platform_device *default_dev;

int gckPLATFORM_Init(struct platform_driver *pdrv,
            struct _gcsPLATFORM **platform)
{
    int ret;
    default_dev = platform_device_alloc(pdrv->driver.name, -1);

    if (!default_dev) {
        printk(KERN_ERR "galcore: platform_device_alloc failed.\n");
        return -ENOMEM;
    }

    /* Add device */
    ret = platform_device_add(default_dev);
    if (ret) {
        printk(KERN_ERR "galcore: platform_device_add failed.\n");
        goto put_dev;
    }

    *platform = (gcsPLATFORM *)&default_platform;

#if USE_LINUX_PCIE
    ret = pci_register_driver(&gpu_pci_subdriver);
#endif

    return 0;

put_dev:
    platform_device_put(default_dev);

    return ret;
}

int gckPLATFORM_Terminate(struct _gcsPLATFORM *platform)
{
    if (default_dev) {
        platform_device_unregister(default_dev);
        default_dev = NULL;
    }

#if USE_LINUX_PCIE
    {
        unsigned int dev_index;
        struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)platform;
        for (dev_index = 0; dev_index < pcie_platform->device_number; dev_index++)
        {
            unsigned int i;
            for (i = 0; i < MAX_PCIE_BAR; i++)
            {
                if (pcie_platform->pcie_info[dev_index].bar[i].logical != 0)
                {
                    pci_iounmap(pcie_platform->pcie_info[dev_index].pdev, pcie_platform->pcie_info[dev_index].bar[i].logical);
                }
            }
        }

        pci_unregister_driver(&gpu_pci_subdriver);
    }
#endif

    return 0;
}
