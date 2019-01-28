/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2018 Vivante Corporation
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
*    Copyright (C) 2014 - 2018 Vivante Corporation
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


gceSTATUS
_AdjustParam(
    IN gcsPLATFORM *Platform,
    OUT gcsMODULE_PARAMETERS *Args
    )
{
#if USE_LINUX_PCIE
    struct pci_dev *pdev = Platform->device;
    unsigned char   irqline = pdev->irq;

    if ((Args->irqLine2D != -1) && (Args->irqLine2D != irqline))
    {
        Args->irqLine2D = irqline;
    }
    if ((Args->irqLine != -1) && (Args->irqLine != irqline))
    {
        Args->irqLine = irqline;
    }
#endif
    return gcvSTATUS_OK;
}

static struct soc_platform_ops default_ops =
{
    .adjustParam   = _AdjustParam,
};

static struct soc_platform default_platform =
{
    .name = __FILE__,
    .ops  = &default_ops,
};

#if USE_LINUX_PCIE

int soc_platform_init(struct pci_driver *pdrv,
            struct soc_platform **platform)
{
    *platform = &default_platform;
    return 0;
}

int soc_platform_terminate(struct soc_platform *platform)
{
    return 0;
}

#else
static struct platform_device *default_dev;

int soc_platform_init(struct platform_driver *pdrv,
            struct soc_platform **platform)
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

    *platform = &default_platform;
    return 0;

put_dev:
    platform_device_put(default_dev);

    return ret;
}

int soc_platform_terminate(struct soc_platform *platform)
{
    if (default_dev) {
        platform_device_unregister(default_dev);
        default_dev = NULL;
    }

    return 0;
}
#endif
