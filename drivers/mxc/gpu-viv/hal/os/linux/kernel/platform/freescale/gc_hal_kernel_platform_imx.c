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
#include "gc_hal_kernel_device.h"
#include "gc_hal_driver.h"
#include <linux/slab.h>

#if defined(CONFIG_PM_OPP)
#include <linux/pm_opp.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#  include <linux/of_platform.h>
#  include <linux/of_gpio.h>
#  include <linux/of_address.h>
#endif

#if USE_PLATFORM_DRIVER
#   include <linux/platform_device.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)) || defined(IMX8_SCU_CONTROL)
#  define IMX_GPU_SUBSYSTEM   1
#  include <linux/component.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
#  include <mach/viv_gpu.h>
#elif defined (CONFIG_PM)
#  include <linux/pm_runtime.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
#    include <mach/busfreq.h>
#  elif LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 29)
#    include <linux/busfreq-imx6.h>
#    include <linux/reset.h>
#  else
#    include <linux/busfreq-imx.h>
#    include <linux/reset.h>
#  endif
#endif

#include <linux/clk.h>

#if defined(IMX8_SCU_CONTROL)
#  include <soc/imx8/sc/sci.h>
static sc_ipc_t gpu_ipcHandle;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#  include <mach/hardware.h>
#endif

#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_DEVICE_THERMAL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#    include <linux/device_cooling.h>
#    define REG_THERMAL_NOTIFIER(a) register_devfreq_cooling_notifier(a);
#    define UNREG_THERMAL_NOTIFIER(a) unregister_devfreq_cooling_notifier(a);
#  else
extern int register_thermal_notifier(struct notifier_block *nb);
extern int unregister_thermal_notifier(struct notifier_block *nb);
#    define REG_THERMAL_NOTIFIER(a) register_thermal_notifier(a);
#    define UNREG_THERMAL_NOTIFIER(a) unregister_thermal_notifier(a);
#  endif
#endif

#ifndef gcdFSL_CONTIGUOUS_SIZE
#  define gcdFSL_CONTIGUOUS_SIZE (4 << 20)
#endif

static int initgpu3DMinClock = 1;
module_param(initgpu3DMinClock, int, 0644);

struct platform_device *pdevice;

#ifdef CONFIG_GPU_LOW_MEMORY_KILLER
#  include <linux/kernel.h>
#  include <linux/mm.h>
#  include <linux/oom.h>
#  include <linux/sched.h>
#  include <linux/profile.h>

static unsigned long timeout;
struct task_struct *almostfail;

static int
notif_func(struct notifier_block *self, unsigned long val, void *data)
{
    struct task_struct *task = data;

    if (task == almostfail)
        almostfail = NULL;

    return NOTIFY_DONE;
}

static struct notifier_block task_nb = {
    .notifier_call  = notif_func,
};

static int force_shrink_mem(IN gckKERNEL Kernel)
{
    struct task_struct *p = NULL;
    struct task_struct *selected = NULL;
    int cur_size = 0;
    int set_size = 0;
    int oom_val = 0;
    int mem_adj = 0;
    int retVal = -1;

    if (almostfail && time_before_eq(jiffies, timeout))
        return 0;

    rcu_read_lock();

    for_each_process(p) {
        gcuDATABASE_INFO info;
        struct mm_struct *mm;
        struct signal_struct *sig;

        cur_size = 0;

        task_lock(p);
        sig = p->signal;
        mm = p->mm;
        if (!sig || !mm) {
            task_unlock(p);
            continue;
        }
        oom_val = sig->oom_score_adj;
        if (oom_val < 0) {
            task_unlock(p);
            continue;
        }
        task_unlock(p);

        rcu_read_unlock();
        if (gckKERNEL_QueryProcessDB(Kernel, p->pid, gcvFALSE, gcvDB_VIDEO_MEMORY, &info) == gcvSTATUS_OK){
            cur_size += info.counters.bytes / PAGE_SIZE;
        }
        if (gckKERNEL_QueryProcessDB(Kernel, p->pid, gcvFALSE, gcvDB_CONTIGUOUS, &info) == gcvSTATUS_OK){
            cur_size += info.counters.bytes / PAGE_SIZE;
        }
        rcu_read_lock();

        if (cur_size <= 0) continue;

        printk("<gpu> pid %d (%s), adj %d, size %d\n", p->pid, p->comm, oom_val, cur_size);

        if (selected) {
            if ((oom_val < mem_adj) || (oom_val == mem_adj && cur_size <= set_size)) continue;
        }
        set_size = cur_size;
        mem_adj = oom_val;
        selected = p;
    }

    if (selected && mem_adj > 0) {
        printk("<gpu> send sigkill to %d (%s), adj %d, size %d\n",
                 selected->pid, selected->comm, mem_adj, set_size);
        almostfail = selected;
        timeout = jiffies + HZ;
        force_sig(SIGKILL, selected);
        retVal = 0;
    }

    rcu_read_unlock();

    return retVal;
}

extern gckKERNEL
_GetValidKernel(
  gckGALDEVICE Device
  );

static gceSTATUS
_ShrinkMemory(
    IN gcsPLATFORM * Platform
    )
{
    struct platform_device *pdev;
    gckGALDEVICE galDevice;
    gckKERNEL kernel;
    gceSTATUS status = gcvSTATUS_OK;

    pdev = Platform->device;

    galDevice = platform_get_drvdata(pdev);

    kernel = _GetValidKernel(galDevice);

    if (kernel != gcvNULL)
    {
        if (force_shrink_mem(kernel) != 0)
            status = gcvSTATUS_OUT_OF_MEMORY;
    }
    else
    {
        printk("%s: can't find kernel!\n", __FUNCTION__);
    }

    return status;
}
#endif

#if gcdENABLE_FSCALE_VAL_ADJUST && (defined(CONFIG_DEVICE_THERMAL) || defined(CONFIG_DEVICE_THERMAL_MODULE))
static int thermal_hot_pm_notify(struct notifier_block *nb, unsigned long event,
       void *dummy)
{
    static gctUINT orgFscale, minFscale, maxFscale;
    static gctBOOL bAlreadyTooHot = gcvFALSE;
    gckHARDWARE hardware;
    gckGALDEVICE galDevice;

    galDevice = platform_get_drvdata(pdevice);
    if (!galDevice)
    {
        /* GPU is not ready, so it is meaningless to change GPU freq. */
        return NOTIFY_OK;
    }

    if (!galDevice->kernels[gcvCORE_MAJOR])
    {
        return NOTIFY_OK;
    }

    hardware = galDevice->kernels[gcvCORE_MAJOR]->hardware;

    if (!hardware)
    {
        return NOTIFY_OK;
    }

    if (event && !bAlreadyTooHot) {
        gckHARDWARE_GetFscaleValue(hardware,&orgFscale,&minFscale, &maxFscale);
        gckHARDWARE_SetFscaleValue(hardware, minFscale);
        bAlreadyTooHot = gcvTRUE;
        printk("System is too hot. GPU3D will work at %d/64 clock.\n", minFscale);
    } else if (!event && bAlreadyTooHot) {
        gckHARDWARE_SetFscaleValue(hardware, orgFscale);
        printk("Hot alarm is canceled. GPU3D clock will return to %d/64\n", orgFscale);
        bAlreadyTooHot = gcvFALSE;
    }
    return NOTIFY_OK;
}

static struct notifier_block thermal_hot_pm_notifier =
{
    .notifier_call = thermal_hot_pm_notify,
};

static ssize_t gpu3DMinClock_show(struct device_driver *dev, char *buf)
{
    gctUINT currentf,minf,maxf;
    gckGALDEVICE galDevice;

    galDevice = platform_get_drvdata(pdevice);

    minf = 0;
    if (galDevice->kernels[gcvCORE_MAJOR])
    {
         gckHARDWARE_GetFscaleValue(galDevice->kernels[gcvCORE_MAJOR]->hardware,
            &currentf, &minf, &maxf);
    }

    snprintf(buf, PAGE_SIZE, "%d\n", minf);
    return strlen(buf);
}

static ssize_t gpu3DMinClock_store(struct device_driver *dev, const char *buf, size_t count)
{

    gctINT fields;
    gctUINT MinFscaleValue;
    gckGALDEVICE galDevice;

    galDevice = platform_get_drvdata(pdevice);

    if (galDevice->kernels[gcvCORE_MAJOR])
    {
         fields = sscanf(buf, "%d", &MinFscaleValue);

         if (fields < 1)
             return -EINVAL;

         gckHARDWARE_SetMinFscaleValue(galDevice->kernels[gcvCORE_MAJOR]->hardware,MinFscaleValue);
    }

    return count;
}

static DRIVER_ATTR_RW(gpu3DMinClock);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
static const struct of_device_id mxs_gpu_dt_ids[] = {
#ifdef IMX_GPU_SUBSYSTEM
    { .compatible = "fsl,imx8-gpu-ss", },
#endif
    { .compatible = "fsl,imx6q-gpu", }, /*Backward Compatiblity */
    {/* sentinel */}
};
MODULE_DEVICE_TABLE(of, mxs_gpu_dt_ids);
#endif

struct gpu_clk
{
    struct clk *clk_core;
    struct clk *clk_shader;
    struct clk *clk_axi;
    struct clk *clk_ahb;
};

#if defined(CONFIG_PM_OPP)
typedef enum _GOVERN_MODE
{
    OVERDRIVE,
    NOMINAL,
    UNDERDRIVE,
    GOVERN_COUNT
}
GOVERN_MODE;

static const char *govern_modes[] =
{
    "overdrive",
    "nominal",
    "underdrive"
};

struct gpu_govern
{
    unsigned long core_clk_freq[GOVERN_COUNT];
    unsigned long shader_clk_freq[GOVERN_COUNT];
    struct device* dev;
    int num_modes;
    int current_mode;
};
#endif

struct imx_priv
{
    struct gpu_clk imx_gpu_clks[gcdMAX_GPU_COUNT];

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) || LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    /*Power management.*/
    struct regulator *gpu_regulator;
#  endif
#endif
    /*Run time pm*/
    struct device *pmdev[gcdMAX_GPU_COUNT];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    struct reset_control *rstc[gcdMAX_GPU_COUNT];
#endif

#if defined(IMX8_SCU_CONTROL)
    sc_rsrc_t sc_gpu_pid[gcdMAX_GPU_COUNT];
#endif

    int gpu3dCount;

#if defined(CONFIG_PM_OPP)
    struct gpu_govern imx_gpu_govern;
#endif
};

static struct imx_priv imxPriv;

#if defined(CONFIG_PM_OPP)
static ssize_t gpu_govern_show(struct device_driver *dev, char *buf)
{
    struct imx_priv *priv = &imxPriv;
    int i;
    ssize_t len;

    unsigned long core_freq;
    unsigned long shader_freq;

    len = sprintf(buf, "GPU support %d modes\n", priv->imx_gpu_govern.num_modes);

    for (i = 0; i < priv->imx_gpu_govern.num_modes; i++)
    {
        core_freq   = priv->imx_gpu_govern.core_clk_freq[i];
        shader_freq = priv->imx_gpu_govern.shader_clk_freq[i];

        len += sprintf(buf + len,
            "%s:\tcore_clk frequency: %lu\tshader_clk frequency: %lu\n",
            govern_modes[i], core_freq, shader_freq);
    }

    len += sprintf(buf + len, "Currently GPU runs on mode %s\n",
        govern_modes[priv->imx_gpu_govern.current_mode]);

    return len;
}

static ssize_t gpu_govern_store(struct device_driver *dev, const char *buf, size_t count)
{
    unsigned long core_freq = 0;
    unsigned long shader_freq = 0;
    struct imx_priv *priv = &imxPriv;
    int core = gcvCORE_MAJOR;
    int i;

    for (i = 0; i < GOVERN_COUNT; i++)
    {
        if (strstr(buf, govern_modes[i]))
        {
            break;
        }
    }

    if (i == GOVERN_COUNT)
    {
        return count;
    }

    core_freq   = priv->imx_gpu_govern.core_clk_freq[i];
    shader_freq = priv->imx_gpu_govern.shader_clk_freq[i];
    priv->imx_gpu_govern.current_mode = i;

    for (core = gcvCORE_MAJOR; core <= gcvCORE_3D_MAX; core++)
    {
        struct clk* clk_core   = priv->imx_gpu_clks[core].clk_core;
        struct clk* clk_shader = priv->imx_gpu_clks[core].clk_shader;

        if (clk_core != NULL && clk_shader != NULL &&
            core_freq != 0 && shader_freq != 0)
        {
            clk_set_rate(clk_core, core_freq);
            clk_set_rate(clk_shader, shader_freq);
        }
    }

    return count;
}

static DRIVER_ATTR_RW(gpu_govern);


int init_gpu_opp_table(struct device *dev)
{
    const struct property *prop;
    const __be32 *val;
    int nr;
    int ret = 0;
    int i;
    struct imx_priv *priv = &imxPriv;

    priv->imx_gpu_govern.num_modes = 0;
    priv->imx_gpu_govern.current_mode = OVERDRIVE;

    prop = of_find_property(dev->of_node, "operating-points", NULL);
    if (!prop) {
        return 0;
    }

    if (!prop->value) {
	dev_err(dev, "operating-points invalid. Frequency scaling will not work\n");
        return -ENODATA;
    }

    /*
     * Each OPP is a set of tuples consisting of frequency and
     * voltage like <freq-kHz vol-uV>.
     */
    nr = prop->length / sizeof(u32);
    if (nr % 2) {
        dev_err(dev, "%s: Invalid OPP list\n", __func__);
        return -EINVAL;
    }

    val = prop->value;

    for (i = 0; nr > 0 && i < GOVERN_COUNT; nr -= 4)
    {
        unsigned long core_freq, core_volt, shader_freq, shader_volt;

        core_freq = be32_to_cpup(val++) * 1000;
        core_volt = be32_to_cpup(val++);

        if (nr == 2)
        {
            shader_freq = core_freq;
            shader_volt = core_volt;
        }
        else
        {
            shader_freq = be32_to_cpup(val++) * 1000;
            shader_volt = be32_to_cpup(val++);
        }

        /* We only register core_clk frequency */
        if (dev_pm_opp_add(dev, core_freq, core_volt))
        {
            dev_warn(dev, "%s: Failed to add OPP %ld\n",
                 __func__, core_freq);
            continue;
        }

        priv->imx_gpu_govern.core_clk_freq[i]   = core_freq;
        priv->imx_gpu_govern.shader_clk_freq[i] = shader_freq;

        i++;
    }

    priv->imx_gpu_govern.num_modes = i;
    priv->imx_gpu_govern.dev = dev;

    if (priv->imx_gpu_govern.num_modes > 0)
    {
        ret = driver_create_file(dev->driver, &driver_attr_gpu_govern);
        if (ret)
            dev_err(dev, "create gpu_govern attr failed (%d)\n", ret);
    }

    return ret;
}

int remove_gpu_opp_table(void)
{
    struct imx_priv *priv = &imxPriv;
    struct device* dev = priv->imx_gpu_govern.dev;
    int i = 0;

    for (i = 0; i < priv->imx_gpu_govern.num_modes; i++)
    {
        unsigned long core_freq;

        core_freq = priv->imx_gpu_govern.core_clk_freq[i];
        dev_pm_opp_remove(dev, core_freq);
    }

    if (i > 0)
    {
        driver_remove_file(dev->driver, &driver_attr_gpu_govern);
    }

    return 0;
}
#endif

#ifdef IMX_GPU_SUBSYSTEM

static int use_imx_gpu_subsystem;

/* sub device component ops. */
static int mxc_gpu_sub_bind(struct device *dev,
                struct device *master, void *data)
{
    return 0;
}

static void mxc_gpu_sub_unbind(struct device *dev, struct device *master,
    void *data)
{
}

static const struct component_ops mxc_gpu_sub_ops =
{
    .bind   = mxc_gpu_sub_bind,
    .unbind = mxc_gpu_sub_unbind,
};

/* sub device driver. */
static const struct of_device_id mxc_gpu_sub_match[] =
{
    { .compatible = "fsl,imx8-gpu"},
    { /* sentinel */ }
};

static int mxc_gpu_sub_probe(struct platform_device *pdev)
{
    return component_add(&pdev->dev, &mxc_gpu_sub_ops);
}

static int mxc_gpu_sub_remove(struct platform_device *pdev)
{
    component_del(&pdev->dev, &mxc_gpu_sub_ops);
    return 0;
}

struct platform_driver mxc_gpu_sub_driver =
{
    .driver = {
        .name  = "mxc-gpu",
        .owner = THIS_MODULE,
        .of_match_table = mxc_gpu_sub_match,
    },

    .probe  = mxc_gpu_sub_probe,
    .remove = mxc_gpu_sub_remove,
};

static int register_mxc_gpu_sub_driver(void)
{
    return use_imx_gpu_subsystem ? platform_driver_register(&mxc_gpu_sub_driver) : 0;
}

static void unregister_mxc_gpu_sub_driver(void)
{
    if (use_imx_gpu_subsystem) {
        platform_driver_unregister(&mxc_gpu_sub_driver);
    }
}

static int patch_param_imx8_subsystem(struct platform_device *pdev,
                gcsMODULE_PARAMETERS *args)
{
    int i = 0;
    struct resource* res;
    struct device_node *node = pdev->dev.of_node;
    struct device_node *core_node;
    int core = gcvCORE_MAJOR;

    while ((core_node = of_parse_phandle(node, "cores", i++)) != NULL) {
        struct platform_device *pdev_gpu;
        int irqLine = -1;

        if (!of_device_is_available(core_node)) {
            of_node_put(core_node);
            continue;
        }

        pdev_gpu = of_find_device_by_node(core_node);

        if (!pdev_gpu)
            break;

        irqLine = platform_get_irq(pdev_gpu, 0);

        if (irqLine < 0)
            break;

        res = platform_get_resource(pdev_gpu, IORESOURCE_MEM, 0);

        if (!res)
            break;

        args->irqs[core] = irqLine;
        args->registerBases[core] = res->start;
        args->registerSizes[core] = res->end - res->start + 1;

        of_node_put(core_node);
        ++core;
    }

    if (core_node)
        of_node_put(core_node);

    return 0;
}

static inline int get_power_imx8_subsystem(struct device *pdev)
{
    struct imx_priv *priv = &imxPriv;
    struct clk *clk_core = NULL;
    struct clk *clk_shader = NULL;
    struct clk *clk_axi = NULL;

    /* Initialize the clock structure */
    int i = 0;
    struct device_node *node = pdev->of_node;
    struct device_node *core_node;
    int core = gcvCORE_MAJOR;

#if defined(IMX8_SCU_CONTROL)
    sc_err_t sciErr;
    uint32_t mu_id;

    sciErr = sc_ipc_getMuID(&mu_id);

    if (sciErr != SC_ERR_NONE) {
        printk("galcore; cannot obtain mu id\n");
        return -EINVAL;
    }

    sciErr = sc_ipc_open(&gpu_ipcHandle, mu_id);

    if (sciErr != SC_ERR_NONE) {
        printk("galcore: cannot open MU channel to SCU\n");
        return -EINVAL;
    }
#endif

    while ((core_node = of_parse_phandle(node, "cores", i++)) != NULL) {
        struct platform_device *pdev_gpu = NULL;
        clk_shader = NULL;
        clk_core = NULL;
        clk_axi = NULL;

        if (!of_device_is_available(core_node)) {
            of_node_put(core_node);
            continue;
        }

        pdev_gpu = of_find_device_by_node(core_node);

        if (!pdev_gpu)
            break;

        clk_core = clk_get(&pdev_gpu->dev, "core");

        if (IS_ERR(clk_core)) {
            printk("galcore: clk_get clk_core failed\n");
            break;
        }

        clk_axi = clk_get(&pdev_gpu->dev, "bus");

        if (IS_ERR(clk_axi))
            clk_axi = NULL;

        clk_shader = clk_get(&pdev_gpu->dev, "shader");

        if (IS_ERR(clk_shader)) {
            printk("galcore: clk_get clk_3d_shader failed\n");
            continue;
        }

#if defined(CONFIG_ANDROID) && LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
        /* TODO: freescale BSP issue in some platform like imx8dv. */
        clk_prepare(clk_core);
        clk_set_rate(clk_core, 800000000);
        clk_unprepare(clk_core);

        clk_prepare(clk_shader);
        clk_set_rate(clk_shader, 800000000);
        clk_unprepare(clk_shader);
#endif

        priv->imx_gpu_clks[core].clk_shader = clk_shader;
        priv->imx_gpu_clks[core].clk_core   = clk_core;
        priv->imx_gpu_clks[core].clk_axi    = clk_axi;

#if defined(IMX8_SCU_CONTROL)
        if (of_property_read_u32(core_node, "fsl,sc_gpu_pid", &priv->sc_gpu_pid[core])) {
            priv->sc_gpu_pid[core] = 0;
        }
#endif

#ifdef CONFIG_PM
        pm_runtime_get_noresume(&pdev_gpu->dev);
        pm_runtime_set_active(&pdev_gpu->dev);
        pm_runtime_enable(&pdev_gpu->dev);
        pm_runtime_put_sync(&pdev_gpu->dev);
        priv->pmdev[core] = &pdev_gpu->dev;
#endif
        of_node_put(core_node);
        ++core;
    }

    priv->gpu3dCount = core;

    if (core_node)
        of_node_put(core_node);

    return 0;
}

#endif

static int patch_param_imx6(struct platform_device *pdev,
                gcsMODULE_PARAMETERS *args)
{
    struct resource* res;

    res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "irq_3d");

    if (res)
        args->irqLine = res->start;

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iobase_3d");

    if (res) {
        args->registerMemBase = res->start;
        args->registerMemSize = res->end - res->start + 1;
    }

    res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "irq_2d");

    if (res)
        args->irqLine2D = res->start;

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iobase_2d");

    if (res) {
        args->registerMemBase2D = res->start;
        args->registerMemSize2D = res->end - res->start + 1;
    }

    res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "irq_vg");

    if (res)
        args->irqLineVG = res->start;

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iobase_vg");

    if (res) {
        args->registerMemBaseVG = res->start;
        args->registerMemSizeVG = res->end - res->start + 1;
    }

    return 0;
}

static int patch_param(struct platform_device *pdev,
                gcsMODULE_PARAMETERS *args)
{
    struct resource* res;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    struct device_node *dn =pdev->dev.of_node;
    const u32 *prop;
#else
    struct viv_gpu_platform_data *pdata;
#endif

    pdevice = pdev;

#ifdef IMX_GPU_SUBSYSTEM
    if (pdev->dev.of_node && use_imx_gpu_subsystem)
        patch_param_imx8_subsystem(pdev, args);
    else
#endif
        patch_param_imx6(pdev, args);

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phys_baseaddr");

    if (res && !args->baseAddress && !args->physSize) {
        args->baseAddress = res->start;
        args->physSize = res->end - res->start + 1;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "contiguous_mem");

    if (res) {
        if (args->contiguousBase == 0)
            args->contiguousBase = res->start;

        if  (args->contiguousSize == ~0U)
            args->contiguousSize = res->end - res->start + 1;
    }

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    args->contiguousBase = 0;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    prop = of_get_property(dn, "contiguousbase", NULL);

    if (prop)
        args->contiguousBase = *prop;

    of_property_read_u32(dn,"contiguoussize", (u32 *)&contiguousSize);
#else

    pdata = pdev->dev.platform_data;

    if (pdata) {
        args->contiguousBase = pdata->reserved_mem_base;
        args->contiguousSize = pdata->reserved_mem_size;
    }

#endif

    if (args->contiguousSize == ~0U) {
       printk("Warning: No contiguous memory is reserverd for gpu.!\n");
       printk("Warning: Will use default value(%d) for the reserved memory!\n", gcdFSL_CONTIGUOUS_SIZE);

       args->contiguousSize = gcdFSL_CONTIGUOUS_SIZE;
    }

    args->gpu3DMinClock = initgpu3DMinClock;

    if (args->physSize == 0) {
#if defined(IMX8_PHYS_BASE)
        args->baseAddress = IMX8_PHYS_BASE;
#endif

#if defined(IMX8_PHYS_SIZE)
        args->physSize = IMX8_PHYS_SIZE;
#else
        args->physSize = 0x80000000;
#endif
    }

    return 0;
}

int init_priv(void)
{
    memset(&imxPriv, 0, sizeof(imxPriv));

#ifdef CONFIG_GPU_LOW_MEMORY_KILLER
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,1,0)
    task_free_register(&task_nb);
#  else
    task_handoff_register(&task_nb);
#  endif
#endif

    return 0;
}

void
free_priv(void)
{
#ifdef CONFIG_GPU_LOW_MEMORY_KILLER
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,1,0)
     task_free_unregister(&task_nb);
#  else
    task_handoff_unregister(&task_nb);
#  endif
#endif
}

static int set_clock(int gpu, int enable);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
static void imx6sx_optimize_qosc_for_GPU(void)
{
    struct device_node *np;
    void __iomem *src_base;

    np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-qosc");
    if (!np)
        return;

    src_base = of_iomap(np, 0);
    WARN_ON(!src_base);

    set_clock(gcvCORE_MAJOR, 1);

    writel_relaxed(0, src_base); /* Disable clkgate & soft_rst */
    writel_relaxed(0, src_base+0x60); /* Enable all masters */
    writel_relaxed(0, src_base+0x1400); /* Disable clkgate & soft_rst for gpu */
    writel_relaxed(0x0f000222, src_base+0x1400+0xd0); /* Set Write QoS 2 for gpu */
    writel_relaxed(0x0f000822, src_base+0x1400+0xe0); /* Set Read QoS 8 for gpu */

    set_clock(gcvCORE_MAJOR, 0);
    return;
}
#endif

static inline int get_power_imx6(struct device *pdev)
{
    struct imx_priv *priv = &imxPriv;
    struct clk *clk_core = NULL;
    struct clk *clk_shader = NULL;
    struct clk *clk_axi = NULL;
    struct clk *clk_ahb = NULL;

#ifdef CONFIG_RESET_CONTROLLER
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    struct reset_control *rstc;

    rstc = devm_reset_control_get(pdev, "gpu3d");
    priv->rstc[gcvCORE_MAJOR] = IS_ERR(rstc) ? NULL : rstc;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
    rstc = devm_reset_control_get_shared(pdev, "gpu2d");
    priv->rstc[gcvCORE_2D] = IS_ERR(rstc) ? NULL : rstc;
    rstc = devm_reset_control_get_shared(pdev, "gpuvg");
#    else
    rstc = devm_reset_control_get(pdev, "gpu2d");
    priv->rstc[gcvCORE_2D] = IS_ERR(rstc) ? NULL : rstc;
    rstc = devm_reset_control_get(pdev, "gpuvg");
#    endif
    priv->rstc[gcvCORE_VG] = IS_ERR(rstc) ? NULL : rstc;
#  endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
    /* Get gpu regulator */
    priv->gpu_regulator = regulator_get(pdev, "cpu_vddgpu");
#  elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    priv->gpu_regulator = devm_regulator_get(pdev, "pu");
#  endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) || LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    if (IS_ERR(priv->gpu_regulator)) {
       printk("%s: Failed to get gpu regulator\n", __FUNCTION__);
       return -ENXIO;
    }
#  endif
#endif

    clk_core = clk_get(pdev, "gpu3d_clk");

    if (!IS_ERR(clk_core)) {
        clk_axi = clk_get(pdev, "gpu3d_axi_clk");
        clk_shader = clk_get(pdev, "gpu3d_shader_clk");

        if (IS_ERR(clk_shader)) {
            clk_put(clk_core);
            clk_core = NULL;
            clk_shader = NULL;
            printk("galcore: clk_get gpu3d_shader_clk failed, disable 3d!\n");
        }

        clk_ahb = clk_get(pdev, "gpu3d_ahb_clk");
        if (IS_ERR(clk_ahb)) {
            clk_ahb = NULL;
        }
    } else {
        clk_core = NULL;
        printk("galcore: clk_get gpu3d_clk failed, disable 3d!\n");
    }

    priv->imx_gpu_clks[gcvCORE_MAJOR].clk_core = clk_core;
    priv->imx_gpu_clks[gcvCORE_MAJOR].clk_shader = clk_shader;
    priv->imx_gpu_clks[gcvCORE_MAJOR].clk_axi = clk_axi;
    priv->imx_gpu_clks[gcvCORE_MAJOR].clk_ahb = clk_ahb;

    clk_core = clk_get(pdev, "gpu2d_clk");

    if (IS_ERR(clk_core)) {
        clk_core = NULL;
        printk("galcore: clk_get 2d core clock failed, disable 2d/vg!\n");
    } else {
        clk_axi = clk_get(pdev, "gpu2d_axi_clk");
        if (IS_ERR(clk_axi)) {
            clk_axi = NULL;
            printk("galcore: clk_get 2d axi clock failed, disable 2d\n");
        }
        clk_ahb = clk_get(pdev, "gpu2d_ahb_clk");
        if (IS_ERR(clk_ahb)) {
            clk_ahb = NULL;
        }

        priv->imx_gpu_clks[gcvCORE_2D].clk_ahb = clk_ahb;
        priv->imx_gpu_clks[gcvCORE_2D].clk_core = clk_core;
        priv->imx_gpu_clks[gcvCORE_2D].clk_axi = clk_axi;

        clk_axi = clk_get(pdev, "openvg_axi_clk");

        if (IS_ERR(clk_axi)) {
            clk_axi = NULL;
            printk("galcore: clk_get vg clock failed, disable vg!\n");
        }

        priv->imx_gpu_clks[gcvCORE_VG].clk_core = clk_core;
        priv->imx_gpu_clks[gcvCORE_VG].clk_axi = clk_axi;
    }

#ifdef CONFIG_PM
    pm_runtime_enable(pdev);

    priv->pmdev[gcvCORE_MAJOR] = pdev;
    priv->pmdev[gcvCORE_2D]    = pdev;
    priv->pmdev[gcvCORE_VG]    = pdev;
#endif

    return 0;
}

static inline int get_power(struct device *pdev)
{
    int ret;

    /*Initialize the clock structure*/
#ifdef IMX_GPU_SUBSYSTEM
    if (pdev->of_node && use_imx_gpu_subsystem)
        ret = get_power_imx8_subsystem(pdev);
    else
#endif
        ret = get_power_imx6(pdev);

    if (ret)
        return ret;

#if gcdENABLE_FSCALE_VAL_ADJUST && (defined(CONFIG_DEVICE_THERMAL) || defined(CONFIG_DEVICE_THERMAL_MODULE))
    REG_THERMAL_NOTIFIER(&thermal_hot_pm_notifier);

    ret = driver_create_file(pdev->driver, &driver_attr_gpu3DMinClock);

    if (ret)
        dev_err(pdev, "create gpu3DMinClock attr failed (%d)\n", ret);
#endif

#if defined(CONFIG_PM_OPP)
    ret = init_gpu_opp_table(pdev);
    if (ret)
	dev_err(pdev, "OPP init failed!\n");
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    imx6sx_optimize_qosc_for_GPU();
#endif

    return 0;
}

static inline void put_power(void)
{
    int core = 0;
    struct gpu_clk *imx_clk = NULL;
    struct imx_priv *priv = &imxPriv;
    struct device *pmdev_last = NULL;/*legacy gpu device entry for imx6*/
    struct clk *clk_core_last = NULL;/*vg has same core clk as 2d */

    for (core = 0; core < gcdMAX_GPU_COUNT; core++) {
        imx_clk = &priv->imx_gpu_clks[core];

        if (imx_clk->clk_core && imx_clk->clk_core != clk_core_last) {
             clk_put(imx_clk->clk_core);
             clk_core_last = imx_clk->clk_core;
             imx_clk->clk_core = NULL;
        }

        if (imx_clk->clk_shader) {
             clk_put(imx_clk->clk_shader);
             imx_clk->clk_shader = NULL;
        }

        if (imx_clk->clk_axi) {
             clk_put(imx_clk->clk_axi);
             imx_clk->clk_axi = NULL;
        }

        if (imx_clk->clk_ahb) {
             clk_put(imx_clk->clk_ahb);
             imx_clk->clk_ahb = NULL;
        }

#ifdef CONFIG_PM
        if (priv->pmdev[core] && priv->pmdev[core] != pmdev_last){
            pm_runtime_disable(priv->pmdev[core]);
            pmdev_last = priv->pmdev[core];
        }
#endif
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
    if (priv->gpu_regulator) {
       regulator_put(priv->gpu_regulator);
       priv->gpu_regulator = NULL;
    }
#endif

#if gcdENABLE_FSCALE_VAL_ADJUST && (defined(CONFIG_DEVICE_THERMAL) || defined(CONFIG_DEVICE_THERMAL_MODULE))
    UNREG_THERMAL_NOTIFIER(&thermal_hot_pm_notifier);

    driver_remove_file(pdevice->dev.driver, &driver_attr_gpu3DMinClock);
#endif

#if defined(CONFIG_PM_OPP)
    remove_gpu_opp_table();
#endif

#if defined(IMX8_SCU_CONTROL)
    if (gpu_ipcHandle)
        sc_ipc_close(gpu_ipcHandle);
#endif
}

static inline int set_power(int gpu, int enable)
{
#ifdef CONFIG_PM
    struct imx_priv* priv = &imxPriv;
#endif

    if (enable) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) || LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
        if (!IS_ERR(priv->gpu_regulator)) {
            int ret = regulator_enable(priv->gpu_regulator);

            if (ret)
                printk("%s: fail to enable pu regulator %d!\n", __FUNCTION__, ret);
        }
#  else
        imx_gpc_power_up_pu(true);
#  endif
#endif

#ifdef CONFIG_PM
        pm_runtime_get_sync(priv->pmdev[gpu]);
#endif

#if defined(IMX8_SCU_CONTROL)
        if (priv->sc_gpu_pid[gpu]) {
            sc_err_t sciErr = sc_misc_set_control(gpu_ipcHandle, priv->sc_gpu_pid[gpu], SC_C_ID, gpu);
            if (sciErr != SC_ERR_NONE)
                printk("galcore: failed to set gpu id for 3d_%d\n", gpu);

            if (priv->gpu3dCount > 1) {
                sciErr = sc_misc_set_control(gpu_ipcHandle, priv->sc_gpu_pid[gpu], SC_C_SINGLE_MODE, 0);
                if (sciErr != SC_ERR_NONE)
                    printk("galcore: failed to set gpu dual mode for 3d_%d\n", gpu);
            } else {
                 sciErr = sc_misc_set_control(gpu_ipcHandle, priv->sc_gpu_pid[gpu], SC_C_SINGLE_MODE, 1);
                 if (sciErr != SC_ERR_NONE)
                     printk("galcore: failed to set gpu single mode for 3d_%d\n", gpu);
            }
        }
#endif
    } else {
#ifdef CONFIG_PM
        pm_runtime_put_sync(priv->pmdev[gpu]);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) || LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
        if (!IS_ERR(priv->gpu_regulator))
            regulator_disable(priv->gpu_regulator);
#  else
        imx_gpc_power_up_pu(false);
#  endif
#endif
    }

    return 0;
}

int set_clock(int gpu, int enable)
{
    struct imx_priv* priv = &imxPriv;
    struct clk *clk_core = priv->imx_gpu_clks[gpu].clk_core;
    struct clk *clk_shader = priv->imx_gpu_clks[gpu].clk_shader;
    struct clk *clk_axi = priv->imx_gpu_clks[gpu].clk_axi;
    struct clk *clk_ahb = priv->imx_gpu_clks[gpu].clk_ahb;

    if (enable) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
        if (clk_core)
            clk_prepare(clk_core);

        if (clk_shader)
            clk_prepare(clk_shader);

        if (clk_axi)
            clk_prepare(clk_axi);

        if (clk_ahb)
            clk_prepare(clk_ahb);
#endif
        if (clk_core)
            clk_enable(clk_core);

        if (clk_shader)
            clk_enable(clk_shader);

        if (clk_axi)
            clk_enable(clk_axi);

        if (clk_ahb)
            clk_enable(clk_ahb);
    } else {
        if (clk_core)
            clk_disable(clk_core);

        if (clk_shader)
            clk_disable(clk_shader);

        if (clk_axi)
            clk_disable(clk_axi);

        if (clk_ahb)
            clk_disable(clk_ahb);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
        if (clk_core)
            clk_unprepare(clk_core);

        if (clk_shader)
            clk_unprepare(clk_shader);

        if (clk_axi)
            clk_unprepare(clk_axi);

        if (clk_ahb)
            clk_unprepare(clk_ahb);
#endif
    }

    return gcvSTATUS_OK;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
#ifdef CONFIG_PM
static int gpu_runtime_suspend(struct device *dev)
{
    release_bus_freq(BUS_FREQ_HIGH);
    return 0;
}

static int gpu_runtime_resume(struct device *dev)
{
    request_bus_freq(BUS_FREQ_HIGH);
    return 0;
}

static struct dev_pm_ops gpu_pm_ops;
#endif
#endif


static int adjust_platform_driver(struct platform_driver *driver)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    driver->driver.of_match_table = mxs_gpu_dt_ids;
#endif

#ifdef CONFIG_PM
    /* Override PM callbacks to add runtime PM callbacks. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    /* Fill local structure with original value. */
    memcpy(&gpu_pm_ops, driver->driver.pm, sizeof(struct dev_pm_ops));

    /* Add runtime PM callback. */
#ifdef CONFIG_PM
    gpu_pm_ops.runtime_suspend = gpu_runtime_suspend;
    gpu_pm_ops.runtime_resume = gpu_runtime_resume;
    gpu_pm_ops.runtime_idle = NULL;
#endif

    /* Replace callbacks. */
    driver->driver.pm = &gpu_pm_ops;
#endif
#endif

    return 0;
}

#define SRC_SCR_OFFSET 0
#define BP_SRC_SCR_GPU3D_RST 1
#define BP_SRC_SCR_GPU2D_RST 4

static inline int reset_gpu(int gpu)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
    void __iomem *src_base = IO_ADDRESS(SRC_BASE_ADDR);
    uint32_t bit_offset, val;

    if (gpu == gcvCORE_MAJOR)
        bit_offset = BP_SRC_SCR_GPU3D_RST;
    else if ((gpu == gcvCORE_VG) ||(gpu == gcvCORE_2D))
        bit_offset = BP_SRC_SCR_GPU2D_RST;
    else
        return -ENXIO;

    val = __raw_readl(src_base + SRC_SCR_OFFSET);
    val &= ~(1 << (bit_offset));
    val |= (1 << (bit_offset));
    __raw_writel(val, src_base + SRC_SCR_OFFSET);

    while ((__raw_readl(src_base + SRC_SCR_OFFSET) & (1 << (bit_offset))) != 0);

    return -ENODEV;

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    struct imx_priv* priv = &imxPriv;
    struct reset_control *rstc = priv->rstc[gpu];

    if (rstc)
        reset_control_reset(rstc);
#else
    imx_src_reset_gpu((int)gpu);
#endif

    return 0;
}

static gceSTATUS
_AdjustParam(
    gcsPLATFORM * Platform,
    gcsMODULE_PARAMETERS *Args
    )
{
    patch_param(Platform->device, Args);
    return gcvSTATUS_OK;
}

static gceSTATUS
_GetPower(
    gcsPLATFORM * Platform
    )
{
    int ret = get_power(&Platform->device->dev);

    if (ret)
        return gcvSTATUS_GENERIC_IO;

    return gcvSTATUS_OK;
}

static gceSTATUS
_PutPower(
    gcsPLATFORM * Platform
    )
{
    put_power();
    return gcvSTATUS_OK;
}


static gceSTATUS
_SetPower(
    gcsPLATFORM * Platform,
    gceCORE GPU,
    gctBOOL Enable
    )
{
    return set_power((int)GPU, Enable) ? gcvSTATUS_GENERIC_IO
                : gcvSTATUS_OK;
}

static gceSTATUS
_SetClock(
    gcsPLATFORM * Platform,
    gceCORE GPU,
    gctBOOL Enable
    )
{
    set_clock((int)GPU, Enable);
    return gcvSTATUS_OK;
}

static gceSTATUS
_Reset(
    gcsPLATFORM * Platform,
    gceCORE GPU
    )
{
    int ret;

    ret = reset_gpu((int)GPU);

    if (!ret)
        return gcvSTATUS_OK;
    else if (ret == -ENODEV)
        return gcvSTATUS_NOT_SUPPORTED;
    else
        return gcvSTATUS_INVALID_CONFIG;
}

struct soc_platform_ops imx_platform_ops =
{
    .adjustParam  = _AdjustParam,
    .getPower     = _GetPower,
    .putPower     = _PutPower,
    .setPower     = _SetPower,
    .setClock     = _SetClock,
    .reset        = _Reset,
#ifdef CONFIG_GPU_LOW_MEMORY_KILLER
    .shrinkMemory = _ShrinkMemory,
#endif
};

static struct soc_platform imx_platform =
{
    .name = __FILE__,
    .ops  = &imx_platform_ops,
};

int soc_platform_init(struct platform_driver *pdrv,
            struct soc_platform **platform)
{
#ifdef IMX_GPU_SUBSYSTEM
    if (of_find_compatible_node(NULL, NULL, "fsl,imx8-gpu-ss")) {
        use_imx_gpu_subsystem = 1;
    }

    if (of_find_compatible_node(NULL, NULL, "fsl,imx8x-gpu")) {
        printk(KERN_ERR "Incorrect device-tree, please update dtb.");
        return -EINVAL;
    }
#endif

    adjust_platform_driver(pdrv);
    init_priv();

#ifdef IMX_GPU_SUBSYSTEM
    register_mxc_gpu_sub_driver();
#endif

    *platform = &imx_platform;
    return 0;
}

int soc_platform_terminate(struct soc_platform *platform)
{
#ifdef IMX_GPU_SUBSYSTEM
    unregister_mxc_gpu_sub_driver();
#endif

    free_priv();
    return 0;
}

