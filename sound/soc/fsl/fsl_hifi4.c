/*
 * Freescale HIFI 4 driver
 *
 * Copyright (c) 2012-2013 by Tensilica Inc. ALL RIGHTS RESERVED.
 * Copyright 2018 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Copyright (c) 2001 William L. Pitts
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are freely
 * permitted provided that the above copyright notice and this
 * paragraph and the following disclaimer are duplicated in all
 * such forms.
 *
 * This software is provided "AS IS" and without any express or
 * implied warranties, including, without limitation, the implied
 * warranties of merchantability and fitness for a particular
 * purpose.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <uapi/linux/mxc_hifi4.h>
#include <soc/imx8/sc/svc/irq/api.h>
#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/sci.h>
#include "fsl_hifi4.h"


/* ...allocate new client */
static inline struct xf_client *xf_client_alloc(struct fsl_hifi4 *hifi4_priv)
{
	struct xf_client *client;
	u32             id;

	id = hifi4_priv->xf_client_map[0].next;

	/* ...try to allocate a client handle */
	if (id != 0) {
		/* ...allocate client memory */
		client = kmalloc(sizeof(*client), GFP_KERNEL);
		if (!client)
			return ERR_PTR(-ENOMEM);

		/* ...advance the head of free clients */
		hifi4_priv->xf_client_map[0].next =
				hifi4_priv->xf_client_map[id].next;

		/* ...put associate client id with given object */
		hifi4_priv->xf_client_map[id].client = client;

		/* ...mark client is not yet bound to proxy */
		client->proxy = NULL;

		/* ...save global proxy client identifier */
		client->id = id;

		return client;
	}

	/* ...number of clients exceeded */
	return ERR_PTR(-EBUSY);
}

/* ...recycle client object */
static inline void xf_client_free(struct xf_client *client)
{
	int     id = client->id;
	struct fsl_hifi4 *hifi4_priv = (struct fsl_hifi4 *)client->global;

	/* ...put proxy client id into free clients list */
	hifi4_priv->xf_client_map[id].next = hifi4_priv->xf_client_map[0].next;
	hifi4_priv->xf_client_map[0].next = id;

	/* ...destroy client data */
	kfree(client);
}

/* ...lookup client basing on id */
struct xf_client *xf_client_lookup(struct fsl_hifi4 *hifi4_priv, u32 id)
{
	if ((id >= XF_CFG_MAX_IPC_CLIENTS) ||
		(hifi4_priv->xf_client_map[id].next < XF_CFG_MAX_IPC_CLIENTS)
	   )
		return NULL;
	else
		return hifi4_priv->xf_client_map[id].client;
}

/* ...helper function for retrieving the client handle */
static inline struct xf_client *xf_get_client(struct file *file)
{
	struct xf_client *client;
	u32             id;

	client = (struct xf_client *)file->private_data;
	if (!client)
		return ERR_PTR(-EINVAL);

	id = client->id;
	if (id >= XF_CFG_MAX_IPC_CLIENTS)
		return ERR_PTR(-EINVAL);

	return client;
}

static int fsl_dsp_client_register(struct xf_client *client)
{
	struct fsl_hifi4 *hifi4_priv;
	struct device *dev;

	hifi4_priv = (struct fsl_hifi4 *)client->global;
	dev = hifi4_priv->dev;

	/* ...make sure client is not registered yet */
	if (client->proxy != NULL) {
		pr_err("client-%x already registered", client->id);
		return -EBUSY;
	}

	/* ...complete association (no communication with remote proxy here) */
	client->proxy = &hifi4_priv->proxy;

	pr_debug("client-%x registered within proxy", client->id);

	return 0;
}

/* ...unregister client from shared memory interface */
static int fsl_dsp_client_unregister(struct xf_client *client)
{
	struct xf_proxy *proxy = client->proxy;

	/* ...make sure client is registered */
	if (proxy == NULL) {
		pr_err("client-%x is not registered", client->id);
		return -EBUSY;
	}

	/* ...just clean proxy reference */
	client->proxy = NULL;

	pr_debug("client-%x registered within proxy", client->id);

	return 0;
}

static int fsl_dsp_ipc_msg_to_dsp(struct xf_client *client,
							void __user *user)
{
	struct fsl_hifi4 *hifi4_priv = (struct fsl_hifi4 *)client->global;
	struct device *dev = hifi4_priv->dev;
	struct xf_proxy_message msg;
	void *buffer;
	unsigned long ret = 0;

	ret = copy_from_user(&msg, user, sizeof(struct xf_proxy_message));
	if (ret) {
		dev_err(dev, "failed to get message from user space\n");
		return -EFAULT;
	}

	/* ...make sure message pointer is sane */
	buffer = xf_proxy_a2b(&hifi4_priv->proxy, msg.address);
	if (buffer == (void *)-1)
		return -EFAULT;

	/* ...put current proxy client into message session id */
	msg.session_id = XF_MSG_AP_FROM_USER(msg.session_id, client->id);

	xf_cmd_send(&hifi4_priv->proxy,
				msg.session_id,
				msg.opcode,
				buffer,
				msg.length);

	return 0;
}

static int fsl_dsp_ipc_msg_from_dsp(struct xf_client *client,
							void __user *user)
{
	struct fsl_hifi4 *hifi4_priv = (struct fsl_hifi4 *)client->global;
	struct device *dev = hifi4_priv->dev;
	struct xf_message *m;
	struct xf_proxy_message msg;
	unsigned long ret = 0;

	m = xf_cmd_recv(&hifi4_priv->proxy, &client->wait, &client->queue, 0);
	if (IS_ERR(m)) {
		dev_err(dev, "receiving failed: %d", (int)PTR_ERR(m));
		return PTR_ERR(m);
	}

	/* ...check if there is a response available */
	if (m == NULL)
		return -EAGAIN;

	/* ...prepare message parameters (lock is taken) */
	msg.session_id = XF_MSG_AP_TO_USER(m->id);
	msg.opcode = m->opcode;
	msg.length = m->length;
	msg.address = xf_proxy_b2a(&hifi4_priv->proxy, m->buffer);
	msg.ret = m->ret;

	/* ...return the message back to a pool and release lock */
	xf_msg_free(&hifi4_priv->proxy, m);

	ret = copy_to_user(user, &msg, sizeof(struct xf_proxy_message));
	if (ret) {
		dev_err(dev, "failed to response message to user space\n");
		return -EFAULT;
	}

	return 0;
}

static int fsl_dsp_get_shmem_info(struct xf_client *client,
							void __user *user)
{
	struct fsl_hifi4 *hifi4_priv = (struct fsl_hifi4 *)client->global;
	struct device *dev = hifi4_priv->dev;
	struct shmem_info mem_info;
	unsigned long ret = 0;

	mem_info.phys_addr = hifi4_priv->scratch_buf_phys;
	mem_info.size = hifi4_priv->scratch_buf_size;

	ret = copy_to_user(user, &mem_info, sizeof(struct shmem_info));
	if (ret) {
		dev_err(dev, "failed to response message to user space\n");
		return -EFAULT;
	}

	return ret;
}

static struct miscdevice hifi4_miscdev = {
	.name	= "mxc_hifi4",
	.minor	= MISC_DYNAMIC_MINOR,
};

static long fsl_hifi4_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct xf_client *client;
	struct fsl_hifi4 *hifi4_priv;
	struct xf_proxy  *proxy;
	struct device *dev;
	void __user *user;
	long ret = 0;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	hifi4_priv = (struct fsl_hifi4 *)client->global;
	proxy = &hifi4_priv->proxy;
	dev = hifi4_priv->dev;
	user = (void __user *)arg;

	mutex_lock(&hifi4_priv->hifi4_mutex);

	if (!proxy->is_ready) {
		mutex_unlock(&hifi4_priv->hifi4_mutex);
		dev_err(dev, "hifi firmware is not ready\n");
		return -EFAULT;
	}

	switch (cmd) {
	case DSP_CLIENT_REGISTER:
		ret = fsl_dsp_client_register(client);
		break;
	case DSP_CLIENT_UNREGISTER:
		ret = fsl_dsp_client_unregister(client);
		break;
	case DSP_IPC_MSG_SEND:
		ret = fsl_dsp_ipc_msg_to_dsp(client, user);
		break;
	case DSP_IPC_MSG_RECV:
		ret = fsl_dsp_ipc_msg_from_dsp(client, user);
		break;
	case DSP_GET_SHMEM_INFO:
		ret = fsl_dsp_get_shmem_info(client, user);
		break;
	default:
		break;
	}

	mutex_unlock(&hifi4_priv->hifi4_mutex);

	return ret;
}

void resource_release(struct fsl_hifi4 *hifi4_priv)
{
	int i;

	/* ...initialize client association map */
	for (i = 0; i < XF_CFG_MAX_IPC_CLIENTS - 1; i++)
		hifi4_priv->xf_client_map[i].next = i + 1;
	/* ...set list terminator */
	hifi4_priv->xf_client_map[i].next = 0;

	/* ...set pointer to shared memory */
	xf_proxy_init(&hifi4_priv->proxy);
}

static int fsl_hifi4_open(struct inode *inode, struct file *file)
{
	struct fsl_hifi4 *hifi4_priv = dev_get_drvdata(hifi4_miscdev.parent);
	struct device *dev = hifi4_priv->dev;
	struct xf_client *client;
	int ret = 0;

	/* ...basic sanity checks */
	if (!inode || !file)
		return -EINVAL;

	/* ...allocate new proxy client object */
	client = xf_client_alloc(hifi4_priv);
	if (IS_ERR(client))
		return PTR_ERR(client);

	/* ...initialize waiting queue */
	init_waitqueue_head(&client->wait);

	/* ...initialize client pending message queue */
	xf_msg_queue_init(&client->queue);

	/* ...mark user data is not mapped */
	client->vm_start = 0;

	/* ...reset mappings counter */
	atomic_set(&client->vm_use, 0);

	client->global = (void *)hifi4_priv;

	file->private_data = (void *)client;

	pm_runtime_get_sync(dev);

	mutex_lock(&hifi4_priv->hifi4_mutex);
	/* increase reference counter when opening device */
	atomic_long_inc(&hifi4_priv->refcnt);
	mutex_unlock(&hifi4_priv->hifi4_mutex);

	pr_info("client-%x created\n", client->id);

	return ret;
}

static int fsl_hifi4_close(struct inode *inode, struct file *file)
{
	struct fsl_hifi4 *hifi4_priv;
	struct device *dev;
	struct xf_proxy *proxy;
	struct xf_client *client;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	pr_info("client-%x released\n", client->id);

	proxy = client->proxy;
	if (proxy) {
		/* ...release all pending messages */
		xf_msg_free_all(proxy, &client->queue);

		/* ...recycle client id and release memory */
		xf_client_free(client);
	}

	hifi4_priv = (struct fsl_hifi4 *)client->global;
	dev = hifi4_priv->dev;
	pm_runtime_put_sync(dev);

	mutex_lock(&hifi4_priv->hifi4_mutex);
	/* decrease reference counter when closing device */
	atomic_long_dec(&hifi4_priv->refcnt);
	/* If device is free, reinitialize the resource of
	 * hifi4 driver and framework
	 */
	if (atomic_long_read(&hifi4_priv->refcnt) <= 0)
		resource_release(hifi4_priv);

	mutex_unlock(&hifi4_priv->hifi4_mutex);

	return 0;
}

/* ...wait until data is available in the response queue */
static unsigned int fsl_hifi4_poll(struct file *file, poll_table *wait)
{
	struct xf_proxy *proxy;
	struct xf_client *client;
	int mask;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	/* ...get proxy interface */
	proxy = client->proxy;
	if (!proxy)
		return -EPERM;

	/* ...register client waiting queue */
	poll_wait(file, &client->wait, wait);

	/* ...return current queue state */
	mask = (xf_msg_queue_head(&client->queue) ? POLLIN | POLLRDNORM : 0);

	return mask;
}

/*******************************************************************************
 * Low-level mmap interface
 ******************************************************************************/

/* ...add reference to shared buffer */
static void hifi4_mmap_open(struct vm_area_struct *vma)
{
	struct xf_client *client = vma->vm_private_data;

	/* ...probably just increase counter of open references? - tbd */
	atomic_inc(&client->vm_use);

	pr_debug("xf_mmap_open: vma = %p, client = %p", vma, client);
}

/* ...close reference to shared buffer */
static void hifi4_mmap_close(struct vm_area_struct *vma)
{
	struct xf_client *client = vma->vm_private_data;

	pr_debug("xf_mmap_close: vma = %p, b = %p", vma, client);

	/* ...decrement number of mapping */
	atomic_dec_return(&client->vm_use);
}

/* ...memory map operations */
static const struct vm_operations_struct hifi4_mmap_ops = {
	.open   = hifi4_mmap_open,
	.close  = hifi4_mmap_close,
};

/* ...shared memory mapping */
static int fsl_hifi4_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct xf_proxy *proxy;
	struct xf_client *client;
	unsigned long   size;
	unsigned long   pfn;
	int             r;
	struct fsl_hifi4 *hifi4_priv;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	/* ...get proxy interface */
	proxy = client->proxy;
	if (!proxy)
		return -EPERM;

	/* ...check it was not mapped already */
	if (client->vm_start != 0)
		return -EBUSY;

	/* ...check mapping flags (tbd) */
	if ((vma->vm_flags & (VM_READ | VM_WRITE | VM_SHARED))
				!= (VM_READ | VM_WRITE | VM_SHARED))
		return -EPERM;

	/* ...set memory map operations */
	vma->vm_ops = &hifi4_mmap_ops;

	/* ...assign private data */
	client->vm_start = vma->vm_start;

	/* ...set private memory data */
	vma->vm_private_data = client;

	/* ...set page number of shared memory */
	hifi4_priv = (struct fsl_hifi4 *)client->global;
	pfn = hifi4_priv->scratch_buf_phys >> PAGE_SHIFT;
	size = hifi4_priv->scratch_buf_size;

	/* ...remap shared memory to user-space */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	r = remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot);
	if (r != 0) {
		pr_err("mapping failed: %d", r);
		return r;
	}

	/* ...system-specific hook for registering shared memory mapping */
	return 0;
}

void *memset_hifi(void *dest, int c, size_t count)
{
	uint *dl = (uint *)dest;
	void *dl_1, *dl_2;
	size_t align = 4;
	size_t n, n1, n2;

	/* while all data is aligned (common case), copy a word at a time */
	if ((((ulong)dest) & (sizeof(*dl) - 1)) != 0) {
		dl = (unsigned int *)(((ulong)dest + align - 1) &
								(~(align - 1)));
		dl_1 = dest;
		dl_2 = (void *)(((ulong)dest + count) & (~(align - 1)));
		n1 = (ulong)dl - (ulong)dl_1;
		n2 = (ulong)dest + count - (ulong)dl_2;
		n = (count - n1 - n2) / align;

		while (n--) {
			writel_relaxed(0,  dl);
			dl++;
		}
		while (n1--) {
			writeb_relaxed(0, dl_1);
			dl_1++;
		}
		while (n2--) {
			writeb_relaxed(0, dl_2);
			dl_2++;
		}
	} else {
		n = count / align;
		n1 = count - n * align;
		dl_1 = dest + n * align;
		while (n--) {
			writel_relaxed(0,  dl);
			dl++;
		}
		while (n1--) {
			writeb_relaxed(0, dl_1);
			dl_1++;
		}
	}

	return dest;
}

void *memcpy_hifi(void *dest, const void *src, size_t count)
{
	unsigned int *dl = (unsigned int *)dest, *sl = (unsigned int *)src;
	size_t n = round_up(count, 4) / 4;

	if (src == dest)
		return dest;

	/* while all data is aligned (common case), copy a word at a time */
	if ((((ulong)dest | (ulong)src) & (sizeof(*dl) - 1)) != 0)
		pr_info("dest %p src %p not 4 bytes aligned\n", dest, src);

	while (n--) {
		writel_relaxed(*sl,  dl);
		dl++;
		sl++;
	}

	return dest;
}

static void hifi4_load_firmware(const struct firmware *fw, void *context)
{
	struct fsl_hifi4 *hifi4_priv = context;
	struct device *dev = hifi4_priv->dev;
	Elf32_Ehdr *ehdr; /* Elf header structure pointer */
	Elf32_Shdr *shdr; /* Section header structure pointer */
	Elf32_Addr  sh_addr;
	unsigned char *strtab = 0; /* String table pointer */
	unsigned char *image; /* Binary image pointer */
	int i; /* Loop counter */
	unsigned long addr;

	if (!fw) {
		dev_info(dev, "external firmware not found\n");
		return;
	}

	addr = (unsigned long)fw->data;
	ehdr = (Elf32_Ehdr *)addr;

	/* Find the section header string table for output info */
	shdr = (Elf32_Shdr *)(addr + ehdr->e_shoff +
			(ehdr->e_shstrndx * sizeof(Elf32_Shdr)));

	if (shdr->sh_type == SHT_STRTAB)
		strtab = (unsigned char *)(addr + shdr->sh_offset);

	/* Load each appropriate section */
	for (i = 0; i < ehdr->e_shnum; ++i) {
		shdr = (Elf32_Shdr *)(addr + ehdr->e_shoff +
				(i * sizeof(Elf32_Shdr)));

		if (!(shdr->sh_flags & SHF_ALLOC) ||
			shdr->sh_addr == 0 || shdr->sh_size == 0)
			continue;

		if (strtab) {
			dev_dbg(dev, "%sing %s @ 0x%08lx (%ld bytes)\n",
			  (shdr->sh_type == SHT_NOBITS) ? "Clear" : "Load",
				&strtab[shdr->sh_name],
				(unsigned long)shdr->sh_addr,
				(long)shdr->sh_size);
		}

		sh_addr = shdr->sh_addr;

		if (shdr->sh_type == SHT_NOBITS) {
			memset_hifi((void *)(hifi4_priv->sdram_vir_addr +
				(sh_addr - hifi4_priv->sdram_phys_addr)),
				0,
				shdr->sh_size);
		} else {
			image = (unsigned char *)addr + shdr->sh_offset;
			if ((!strcmp(&strtab[shdr->sh_name], ".rodata")) ||
				(!strcmp(&strtab[shdr->sh_name], ".text"))   ||
				(!strcmp(&strtab[shdr->sh_name], ".data"))   ||
				(!strcmp(&strtab[shdr->sh_name], ".bss"))
			) {
				memcpy_hifi((void *)(hifi4_priv->sdram_vir_addr
				  + (sh_addr - hifi4_priv->sdram_phys_addr)),
				  (const void *)image,
				  shdr->sh_size);
			} else {
				memcpy_hifi((void *)(hifi4_priv->regs +
						(sh_addr - hifi4_priv->paddr)),
						(const void *)image,
						shdr->sh_size);
			}
		}
	}

	/* start the core */
	sc_pm_cpu_start(hifi4_priv->hifi_ipcHandle,
					SC_R_DSP, true, hifi4_priv->iram);
}

/* Initialization of the MU code. */
int hifi4_mu_init(struct fsl_hifi4 *hifi4_priv)
{
	struct device *dev = hifi4_priv->dev;
	struct device_node *np;
	unsigned int	hifi_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the hifi
	 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu-hifi");
	if (!np) {
		dev_err(dev, "Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
	hifi4_priv->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!hifi4_priv->mu_base_virtaddr);

	ret = of_property_read_u32_index(np,
				"fsl,hifi_ap_mu_id", 0, &hifi_mu_id);
	if (ret) {
		dev_err(dev, "Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	hifi4_priv->hifi_mu_id = hifi_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(hifi4_priv->dev, irq, fsl_hifi4_mu_isr,
			IRQF_EARLY_RESUME, "hifi4_mu_isr", &hifi4_priv->proxy);
	if (ret) {
		dev_err(dev, "request_irq failed %d, err = %d\n", irq, ret);
		return -EINVAL;
	}

	if (!hifi4_priv->hifi_mu_init) {
		MU_Init(hifi4_priv->mu_base_virtaddr);
		MU_EnableRxFullInt(hifi4_priv->mu_base_virtaddr, 0);
		hifi4_priv->hifi_mu_init = 1;
	}

	return ret;
}

static const struct file_operations hifi4_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= fsl_hifi4_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fsl_hifi4_ioctl,
#endif
	.open		= fsl_hifi4_open,
	.poll		= fsl_hifi4_poll,
	.mmap		= fsl_hifi4_mmap,
	.release	= fsl_hifi4_close,
};

static int fsl_hifi4_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_hifi4 *hifi4_priv;
	const char *fw_name;
	struct resource *res;
	void __iomem *regs;
	uint32_t mu_id;
	sc_err_t sciErr;
	void *buf_virt;
	dma_addr_t buf_phys;
	int size, offset, i;
	int ret;

	hifi4_priv = devm_kzalloc(&pdev->dev, sizeof(*hifi4_priv), GFP_KERNEL);
	if (!hifi4_priv)
		return -ENOMEM;

	hifi4_priv->dev = &pdev->dev;

	/* Get the addresses and IRQ */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	hifi4_priv->paddr = res->start;
	hifi4_priv->regs  = regs;

	hifi4_priv->dram0 = hifi4_priv->paddr + DRAM0_OFFSET;
	hifi4_priv->dram1 = hifi4_priv->paddr + DRAM1_OFFSET;
	hifi4_priv->iram  = hifi4_priv->paddr + IRAM_OFFSET;
	hifi4_priv->sram  = hifi4_priv->paddr + SYSRAM_OFFSET;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Cannot obtain MU ID\n");
		return sciErr;
	}

	sciErr = sc_ipc_open(&hifi4_priv->hifi_ipcHandle, mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Cannot open MU channel to SCU %d, %d\n",
								mu_id, sciErr);
		return sciErr;
	};

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_SEL, 1);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset source select\n");
		return -EIO;
	}

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_AUDIO, 0x80);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset of AUDIO\n");
		return -EIO;
	}

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_PERIPH, 0x5A);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset of PERIPH\n");
		return -EIO;
	}

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_IRQ, 0x51);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset of IRQ\n");
		return -EIO;
	}

	ret = hifi4_mu_init(hifi4_priv);
	if (ret)
		return ret;

	ret = of_property_read_string(np, "fsl,hifi4-firmware", &fw_name);
	hifi4_priv->fw_name = fw_name;

	platform_set_drvdata(pdev, hifi4_priv);
	pm_runtime_enable(&pdev->dev);

	hifi4_miscdev.fops = &hifi4_fops,
	hifi4_miscdev.parent = &pdev->dev,
	ret = misc_register(&hifi4_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register misc device %d\n", ret);
		return ret;
	}

	hifi4_priv->sdram_phys_addr = SDRAM_BASE_ADDR;
	hifi4_priv->sdram_vir_addr = ioremap(hifi4_priv->sdram_phys_addr,
							SDRAM_BASE_SIZE);
	if (!hifi4_priv->sdram_vir_addr) {
		dev_err(&pdev->dev, "failed to remap sdram space for hifi4 firmware\n");
		return -ENXIO;
	}
	memset_io(hifi4_priv->sdram_vir_addr, 0, SDRAM_BASE_SIZE);

	size = MSG_BUF_SIZE + HIFI_CONFIG_SIZE;

	buf_virt = dma_alloc_coherent(&pdev->dev, size, &buf_phys, GFP_KERNEL);
	if (!buf_virt) {
		dev_err(&pdev->dev, "failed alloc memory.\n");
		return -ENOMEM;
	}

	/* msg ring buffer memory */
	hifi4_priv->msg_buf_virt = buf_virt;
	hifi4_priv->msg_buf_phys = buf_phys;
	hifi4_priv->msg_buf_size = MSG_BUF_SIZE;
	offset = MSG_BUF_SIZE;

	/* keep dsp framework's global data when suspend/resume */
	hifi4_priv->hifi_config_virt = buf_virt + offset;
	hifi4_priv->hifi_config_phys = buf_phys + offset;
	hifi4_priv->hifi_config_size = HIFI_CONFIG_SIZE;

	/* scratch memory for dsp framework */
	hifi4_priv->scratch_buf_virt = hifi4_priv->sdram_vir_addr +
						SDRAM_CODEC_LIB_OFFSET;
	hifi4_priv->scratch_buf_phys = hifi4_priv->sdram_phys_addr +
						SDRAM_CODEC_LIB_OFFSET;
	hifi4_priv->scratch_buf_size = SDRAM_BASE_SIZE - SDRAM_CODEC_LIB_OFFSET;

	/* initialize the reference counter for hifi4_priv
	 * structure
	 */
	atomic_long_set(&hifi4_priv->refcnt, 0);

	/* ...initialize client association map */
	for (i = 0; i < XF_CFG_MAX_IPC_CLIENTS - 1; i++)
		hifi4_priv->xf_client_map[i].next = i + 1;
	/* ...set list terminator */
	hifi4_priv->xf_client_map[i].next = 0;

	/* ...set pointer to shared memory */
	xf_proxy_init(&hifi4_priv->proxy);

	/* ...initialize mutex */
	mutex_init(&hifi4_priv->hifi4_mutex);

	return 0;
}

static int fsl_hifi4_remove(struct platform_device *pdev)
{
	struct fsl_hifi4 *hifi4_priv = platform_get_drvdata(pdev);
	int size;

	misc_deregister(&hifi4_miscdev);

	size = MSG_BUF_SIZE + HIFI_CONFIG_SIZE;
	dma_free_coherent(&pdev->dev, size, hifi4_priv->msg_buf_virt,
				hifi4_priv->msg_buf_phys);
	if (hifi4_priv->sdram_vir_addr)
		iounmap(hifi4_priv->sdram_vir_addr);

	return 0;
}

#ifdef CONFIG_PM
static int fsl_hifi4_runtime_resume(struct device *dev)
{
	struct fsl_hifi4 *hifi4_priv = dev_get_drvdata(dev);
	struct xf_proxy *proxy = &hifi4_priv->proxy;
	int ret;

	if (sc_pm_set_resource_power_mode(hifi4_priv->hifi_ipcHandle,
			SC_R_DSP_RAM, SC_PM_PW_MODE_ON) != SC_ERR_NONE) {
		dev_err(dev, "Error power on HIFI RAM\n");
		return -EIO;
	}

	if (!proxy->is_ready) {
		init_completion(&proxy->cmd_complete);

		ret = request_firmware_nowait(THIS_MODULE,
				FW_ACTION_HOTPLUG, hifi4_priv->fw_name,
				dev,
				GFP_KERNEL, hifi4_priv, hifi4_load_firmware);

		if (ret) {
			dev_err(dev, "failed to load firmware\n");
			return ret;
		}

		ret = icm_ack_wait(proxy, 0);
		if (ret) {
			return ret;
		}
		dev_info(dev, "hifi driver registered\n");
	}

	return 0;
}

static int fsl_hifi4_runtime_suspend(struct device *dev)
{
	struct fsl_hifi4 *hifi4_priv = dev_get_drvdata(dev);
	struct xf_proxy *proxy = &hifi4_priv->proxy;

	if (sc_pm_set_resource_power_mode(hifi4_priv->hifi_ipcHandle,
			SC_R_DSP_RAM, SC_PM_PW_MODE_OFF) != SC_ERR_NONE) {
		dev_err(dev, "Error power off HIFI RAM\n");
		return -EIO;
	}
	proxy->is_ready = 0;
	return 0;
}
#endif /* CONFIG_PM */


#ifdef CONFIG_PM_SLEEP
static int fsl_hifi4_suspend(struct device *dev)
{
	int ret = 0;

	ret = pm_runtime_force_suspend(dev);

	return ret;
}

static int fsl_hifi4_resume(struct device *dev)
{
	int ret = 0;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_hifi4_pm = {
	SET_RUNTIME_PM_OPS(fsl_hifi4_runtime_suspend,
					fsl_hifi4_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_hifi4_suspend, fsl_hifi4_resume)
};

static const struct of_device_id fsl_hifi4_ids[] = {
	{ .compatible = "fsl,imx8qxp-hifi4", },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_hifi4_ids);

static struct platform_driver fsl_hifi4_driver = {
	.probe = fsl_hifi4_probe,
	.remove = fsl_hifi4_remove,
	.driver = {
		.name = "fsl-hifi4",
		.of_match_table = fsl_hifi4_ids,
		.pm = &fsl_hifi4_pm,
	},
};
module_platform_driver(fsl_hifi4_driver);

MODULE_DESCRIPTION("Freescale HIFI 4 driver");
MODULE_ALIAS("platform:fsl-hifi4");
MODULE_LICENSE("Dual BSD/GPL");
