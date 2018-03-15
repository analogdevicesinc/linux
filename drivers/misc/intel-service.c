// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2018, Intel Corporation
 */

/*
 *  Intel Stratix10 SoC is composed of a 64 bit quad-core ARM Cortex A53 hard
 *  processor system (HPS) and Secure Device Manager (SDM). SDM is the
 *  hardware which does the FPGA configuration, QSPI, Crypto and warm reset.
 *
 *  When the FPGA is configured from HPS, there needs to be a way for HPS to
 *  notify SDM the location and size of the configuration data. Then SDM will
 *  get the configuration data from that location and perform the FPGA
 *  configuration.
 *
 *  To meet the whole system security needs and support virtual machine
 *  requesting communication with SDM, only the secure world of software (EL3,
 *  Exception Level 3) can interface with SDM. All software entities running
 *  on other exception levels must channel through the EL3 software whenever
 *  it needs service from SDM.
 *
 *  Intel Stratix10 service layer driver is added to provide the service for
 *  FPGA configuration. Running at privileged exception level (EL1, Exception
 *  Level 1), Intel Stratix10 service layer driver interfaces with the service
 *  client at EL1 (Intel Stratix10 FPGA Manager) and manages secure monitor
 *  call (SMC) to communicate with secure monitor software at secure monitor
 *  exception level (EL3).
 */

#include <linux/arm-smccc.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/genalloc.h>
#include <linux/intel-service-client.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "intel-smc.h"

/* SVC_NUM_DATA_IN_FIFO - number of struct intel_svc_data in the FIFO */
#define SVC_NUM_DATA_IN_FIFO			32
/* SVC_NUM_CHANNEL - number of channel supported by service layer driver */
#define SVC_NUM_CHANNEL				2
/*
 * FPGA_CONFIG_DATA_CLAIM_TIMEOUT_MS - claim back the submitted buffer(s)
 * from the secure world for FPGA manager to reuse, or to free the buffer(s)
 * when all bit-stream data had be send.
 */
#define FPGA_CONFIG_DATA_CLAIM_TIMEOUT_MS	200
/*
 * FPGA_CONFIG_STATUS_TIMEOUT_SEC - poll the FPGA configuration status,
 * service layer will return error to FPGA manager when timeout occurs,
 * timeout is set to 30 seconds (30 * 1000) at Intel Stratix10 SoC.
 */
#define FPGA_CONFIG_STATUS_TIMEOUT_SEC		30

typedef void (svc_invoke_fn)(unsigned long, unsigned long, unsigned long,
			     unsigned long, unsigned long, unsigned long,
			     unsigned long, unsigned long,
			     struct arm_smccc_res *);

struct intel_svc_chan;

/**
 * struct intel_svc_sh_memory - service shared memory structure
 * @sync_complete: state for a completion
 * @addr: physical address of shared memory block
 * @size: size of shared memory block
 * @invoke_fn: function to issue secure monitor or hypervisor call
 *
 * This struct is used to save physical address and size of shared memory
 * block. The shared memory blocked is allocated by secure monitor software
 * at secure world.
 *
 * Service layer driver uses the physical address and size to create a memory
 * pool, then allocates data buffer from that memory pool for service client.
 */
struct intel_svc_sh_memory {
	struct completion sync_complete;
	unsigned long addr;
	unsigned long size;
	svc_invoke_fn *invoke_fn;
};

/**
 * struct intel_svc_data_mem - service memory structure
 * @vaddr: virtual address
 * @paddr: physical address
 * @size: size of memory
 * @node: link list head node
 *
 * This struct is used in a list that keeps track of buffers which have
 * been allocated or freed from the memory pool. Service layer driver also
 * uses this struct to transfer physical address to virtual address.
 */
struct intel_svc_data_mem {
	void *vaddr;
	phys_addr_t paddr;
	size_t size;
	struct list_head node;
};

/**
 * struct intel_svc_data - service data structure
 * @chan: service channel
 * @paddr: playload physical address
 * @size: playload size
 * @command: service command requested by client
 *
 * This struct is used in service FIFO for inter-process communication.
 */
struct intel_svc_data {
	struct intel_svc_chan *chan;
	phys_addr_t paddr;
	size_t size;
	u32 command;
};

/**
 * struct intel_svc_controller - service controller
 * @dev: device
 * @chans: array of service channels
 * $num_chans: number of channels in 'chans' array
 * @node: list management
 * @genpool: memory pool pointing to the memory region
 * @task: pointer to the thread task which handles SMC or HVC call
 * @svc_fifo: a queue for storing service message data
 * @complete_status: state for completion
 * @svc_fifo_lock: protect access to service message data queue
 * @invoke_fn: function to issue secure monitor call or hypervisor call
 *
 * This struct is used to create communication channels for service clients, to
 * handle secure monitor or hypervisor call.
 */
struct intel_svc_controller {
	struct device *dev;
	struct intel_svc_chan *chans;
	int num_chans;
	struct list_head node;
	struct gen_pool *genpool;
	struct task_struct *task;
	struct kfifo svc_fifo;
	struct completion complete_status;
	spinlock_t svc_fifo_lock;
	svc_invoke_fn *invoke_fn;
};

/**
 * struct intel_svc_chan - service communication channel
 * @ctrl: pointer to service controller which is the provider of this channel
 * @scl: pointer to service client which owns the channel
 * @name: service client name associated with the channel
 * @lock: protect access to the channel
 *
 * This struct is used by service client to communicate with service layer, each
 * service client has its own channel created by service controller.
 */
struct intel_svc_chan {
	struct intel_svc_controller *ctrl;
	struct intel_svc_client *scl;
	char *name;
	spinlock_t lock;
};

static LIST_HEAD(svc_ctrl);
static LIST_HEAD(svc_data_mem);

/**
 * request_svc_channel_byname() - request a service channel
 * @client: pointer to service client
 * @name: service client name
 *
 * This function is used by service client to request a service channel.
 *
 * Return: a pointer to channel assigned to the client on success,
 * or ERR_PTR() on error.
 */
struct intel_svc_chan *request_svc_channel_byname(
	struct intel_svc_client *client, const char *name)
{
	struct device *dev = client->dev;
	struct intel_svc_controller *controller;
	struct intel_svc_chan *chan;
	unsigned long flag;
	int i;

	chan = ERR_PTR(-EPROBE_DEFER);
	if (list_empty(&svc_ctrl))
		return ERR_PTR(-ENODEV);

	controller = list_first_entry(&svc_ctrl,
				      struct intel_svc_controller, node);
	for (i = 0; i < SVC_NUM_CHANNEL; i++) {
		if (!strcmp(controller->chans[i].name, name)) {
			chan = &controller->chans[i];
			break;
		}
	}

	if (chan->scl || !try_module_get(controller->dev->driver->owner)) {
		dev_dbg(dev, "%s: svc not free\n", __func__);
		return ERR_PTR(-EBUSY);
	}

	spin_lock_irqsave(&chan->lock, flag);
	chan->scl = client;
	spin_unlock_irqrestore(&chan->lock, flag);

	return chan;
}
EXPORT_SYMBOL_GPL(request_svc_channel_byname);

/**
 * free_svc_channel() - free service channel
 * @chan: service channel to be freed
 *
 * This function is used by service client to free a service channel.
 */
void free_svc_channel(struct intel_svc_chan *chan)
{
	unsigned long flag;

	spin_lock_irqsave(&chan->lock, flag);
	chan->scl = NULL;
	module_put(chan->ctrl->dev->driver->owner);
	spin_unlock_irqrestore(&chan->lock, flag);
}
EXPORT_SYMBOL_GPL(free_svc_channel);

/**
 * intel_svc_send() - send a message data to the remote
 * @chan: service channel assigned to the client
 * @msg: message data to be sent, in the format of "struct intel_svc_client_msg"
 *
 * This function is used by service client to send command or data to service
 * layer driver.
 *
 * Return: non-negative value for successful submission to the data queue
 * created by service layer driver, or negative value on error.
 */
int intel_svc_send(struct intel_svc_chan *chan, void *msg)
{
	struct intel_svc_client_msg *p_msg = (struct intel_svc_client_msg *)msg;
	struct intel_svc_data_mem *p_mem;
	struct intel_svc_data *p_data;
	int ret = 0;

	p_data = kmalloc(sizeof(*p_data), GFP_KERNEL);
	if (!p_data)
		return -ENOMEM;

	pr_debug("%s: sent P-va=%p, P-com=%x, P-size=%u\n", __func__,
		 p_msg->payload, p_msg->command,
		 (unsigned int)p_msg->payload_length);

	list_for_each_entry(p_mem, &svc_data_mem, node) {
		if (p_mem->vaddr == p_msg->payload) {
			p_data->paddr = p_mem->paddr;
			break;
		}
	}

	p_data->command = p_msg->command;
	p_data->size = p_msg->payload_length;
	p_data->chan = chan;
	pr_debug("%s: put to FIFO pa=0x%016x, cmd=%x, size=%u\n", __func__,
	       (unsigned int)p_data->paddr, p_data->command,
	       (unsigned int)p_data->size);
	ret = kfifo_in_spinlocked(&chan->ctrl->svc_fifo, p_data,
				  sizeof(*p_data),
				  &chan->ctrl->svc_fifo_lock);

	kfree(p_data);

	if (!ret)
		return -ENOBUFS;

	return ret;
}
EXPORT_SYMBOL_GPL(intel_svc_send);

/**
 * intel_svc_allocate_memory() - allocate memory
 * @chan: service channel assigned to the client
 * @size: memory size requested by a specific service client
 *
 * Service layer allocates the requested number of bytes buffer from the
 * memory pool, service client uses this function to get allocated buffers.
 *
 * Return: address of allocated memory on success, or ERR_PTR() on error.
 */
void *intel_svc_allocate_memory(struct intel_svc_chan *chan, size_t size)
{
	struct intel_svc_data_mem *pmem;
	unsigned long va;
	phys_addr_t pa;
	struct gen_pool *genpool = chan->ctrl->genpool;
	size_t s = roundup(size, 1 << genpool->min_alloc_order);

	pmem = devm_kzalloc(chan->ctrl->dev, sizeof(*pmem), GFP_KERNEL);
	if (!pmem)
		return ERR_PTR(-ENOMEM);

	va = gen_pool_alloc(genpool, s);
	if (!va)
		return ERR_PTR(-ENOMEM);

	memset((void *)va, 0, s);
	pa = gen_pool_virt_to_phys(genpool, va);

	pmem->vaddr = (void *)va;
	pmem->paddr = pa;
	pmem->size = s;
	list_add_tail(&pmem->node, &svc_data_mem);
	pr_debug("%s: va=%p, pa=0x%016x\n", __func__,
		 pmem->vaddr, (unsigned int)pmem->paddr);

	return (void *)va;
}
EXPORT_SYMBOL_GPL(intel_svc_allocate_memory);

/**
 * intel_svc_free_memory() - free allocated memory
 * @chan: service channel assigned to the client
 * @kaddr: memory to be freed
 *
 * This function is used by service client to free allocated buffers.
 */
void intel_svc_free_memory(struct intel_svc_chan *chan, void *kaddr)
{
	struct intel_svc_data_mem *pmem;
	size_t size = 0;

	list_for_each_entry(pmem, &svc_data_mem, node)
		if (pmem->vaddr == kaddr) {
			size = pmem->size;
			break;
		}

	gen_pool_free(chan->ctrl->genpool, (unsigned long)kaddr, size);
	pmem->vaddr = NULL;
	list_del(&pmem->node);
}
EXPORT_SYMBOL_GPL(intel_svc_free_memory);

/**
 * svc_pa_to_va() - translate physical address to virtual address
 * @addr: to be translated physical address
 *
 * Return: valid virtual address or NULL if the provided physical
 * address doesn't exist.
 */
static void *svc_pa_to_va(unsigned long addr)
{
	struct intel_svc_data_mem *pmem;

	pr_debug("claim back P-addr=0x%016x\n", (unsigned int)addr);
	list_for_each_entry(pmem, &svc_data_mem, node) {
		if (pmem->paddr == addr)
			return pmem->vaddr;
	}

	/* physical address is not found */
	return NULL;
}

/**
 * svc_thread_cmd_data_claim() - claim back buffer from the secure world
 * @addr: pointer to service layer controller
 * @p_data: pointer to service data structure
 * @c_data: pointer to callback data structure to service client
 *
 * Claim back the submitted buffers from the secure world and pass buffer
 * back to service client (FPGA manager, etc) for reuse.
 */
static void svc_thread_cmd_data_claim(struct intel_svc_controller *ctrl,
				      struct intel_svc_data *p_data,
				      struct intel_svc_c_data *c_data)
{
	struct arm_smccc_res res;
	unsigned long timeout;

	reinit_completion(&ctrl->complete_status);
	timeout = msecs_to_jiffies(FPGA_CONFIG_DATA_CLAIM_TIMEOUT_MS);

	pr_debug("%s: claim back the submitted buffer\n", __func__);
	do {
		ctrl->invoke_fn(INTEL_SIP_SMC_FPGA_CONFIG_COMPLETED_WRITE,
				0, 0, 0, 0, 0, 0, 0, &res);

		if (res.a0 == INTEL_SIP_SMC_STATUS_OK) {
			if (!res.a1) {
				complete(&ctrl->complete_status);
				break;
			}
			c_data->status = BIT(SVC_STATUS_RECONFIG_BUFFER_DONE);
			c_data->kaddr1 = svc_pa_to_va(res.a1);
			c_data->kaddr2 = (res.a2) ? svc_pa_to_va(res.a2) : NULL;
			c_data->kaddr3 = (res.a3) ? svc_pa_to_va(res.a3) : NULL;
			p_data->chan->scl->receive_cb(p_data->chan->scl,
						      c_data);
		} else {
			pr_debug("%s: secure world busy, polling again\n",
				 __func__);
		}
	} while (res.a0 == INTEL_SIP_SMC_STATUS_OK ||
		 res.a0 == INTEL_SIP_SMC_FPGA_CONFIG_STATUS_BUSY ||
		 wait_for_completion_timeout(&ctrl->complete_status, timeout));
}

/**
 * svc_thread_cmd_config_status() - check configuration status
 * @ctrl: pointer to service layer controller
 * @p_data: pointer to service data structure
 * @c_data: pointer to callback data structure to service client
 *
 * Check whether the secure firmware at secure world has finished the FPGA
 * configuration, and then inform FPGA manager the configuration status.
 */
static void svc_thread_cmd_config_status(struct intel_svc_controller *ctrl,
					   struct intel_svc_data *p_data,
					   struct intel_svc_c_data *c_data)
{
	struct arm_smccc_res res;
	int count_in_sec;

	c_data->kaddr1 = NULL;
	c_data->kaddr2 = NULL;
	c_data->kaddr3 = NULL;
	c_data->status = BIT(SVC_STATUS_RECONFIG_ERROR);

	pr_debug("%s: polling config status\n", __func__);

	count_in_sec = FPGA_CONFIG_STATUS_TIMEOUT_SEC;
	while (count_in_sec) {
		ctrl->invoke_fn(INTEL_SIP_SMC_FPGA_CONFIG_ISDONE,
				0, 0, 0, 0, 0, 0, 0, &res);
		if ((res.a0 == INTEL_SIP_SMC_STATUS_OK) ||
		    (res.a0 == INTEL_SIP_SMC_FPGA_CONFIG_STATUS_ERROR))
			break;

		/*
		 * configuration is still in progress, wait one second then
		 * poll again
		 */
		msleep(1000);
		count_in_sec--;
	};

	if (res.a0 == INTEL_SIP_SMC_STATUS_OK && count_in_sec)
		c_data->status = BIT(SVC_STATUS_RECONFIG_COMPLETED);

	p_data->chan->scl->receive_cb(p_data->chan->scl, c_data);
}

/**
 * svc_thread_recv_status_ok() - handle the successful status
 * @p_data: pointer to service data structure
 * @c_data: pointer to callback data structure to service client
 * @res: result from SMC or HVC call
 *
 * Send back the correspond status to the service client (FPGA manager etc).
 */
static void svc_thread_recv_status_ok(struct intel_svc_data *p_data,
				      struct intel_svc_c_data *c_data,
				      struct arm_smccc_res res)
{
	c_data->kaddr1 = NULL;
	c_data->kaddr2 = NULL;
	c_data->kaddr3 = NULL;

	switch (p_data->command) {
	case COMMAND_RECONFIG:
		c_data->status = BIT(SVC_STATUS_RECONFIG_REQUEST_OK);
		break;
	case COMMAND_RECONFIG_DATA_SUBMIT:
		c_data->status = BIT(SVC_STATUS_RECONFIG_BUFFER_SUBMITTED);
		break;
	case COMMAND_NOOP:
		c_data->status = BIT(SVC_STATUS_RECONFIG_BUFFER_SUBMITTED);
		c_data->kaddr1 = svc_pa_to_va(res.a1);
		break;
	case COMMAND_RECONFIG_STATUS:
		c_data->status = BIT(SVC_STATUS_RECONFIG_COMPLETED);
		break;
	default:
		break;
	}

	pr_debug("%s: call receive_cb\n", __func__);
	p_data->chan->scl->receive_cb(p_data->chan->scl, c_data);
}

/**
 * svc_normal_to_secure_thread() - the function to run in the kthread
 * @data: data pointer for kthread function
 *
 * Service layer driver creates intel_svc_smc_hvc_call kthread on CPU
 * node 0, its function intel_svc_secure_call_thread is used to handle
 * SMC or HVC calls between kernel driver and secure monitor software.
 *
 * Return: 0
 */
static int svc_normal_to_secure_thread(void *data)
{
	struct intel_svc_controller *ctrl = (struct intel_svc_controller *)data;
	struct intel_svc_data *pdata;
	struct intel_svc_c_data *cdata;
	struct arm_smccc_res res;
	unsigned long a0, a1, a2;
	int ret_fifo = 0;

	pdata =  kmalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	/* default set, to remove build warning */
	a0 = INTEL_SIP_SMC_FPGA_CONFIG_LOOPBACK;
	a1 = 0;
	a2 = 0;

	while (!kthread_should_stop()) {
		ret_fifo = kfifo_out_spinlocked(&ctrl->svc_fifo,
						pdata, sizeof(*pdata),
						&ctrl->svc_fifo_lock);

		if (!ret_fifo)
			continue;

		pr_debug("get from FIFO pa=0x%016x, command=%u, size=%u\n",
			 (unsigned int)pdata->paddr, pdata->command,
			 (unsigned int)pdata->size);

		switch (pdata->command) {
		case COMMAND_RECONFIG_DATA_CLAIM:
			svc_thread_cmd_data_claim(ctrl, pdata, cdata);
			continue;
		case COMMAND_RECONFIG:
			a0 = INTEL_SIP_SMC_FPGA_CONFIG_START;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_RECONFIG_DATA_SUBMIT:
			a0 = INTEL_SIP_SMC_FPGA_CONFIG_WRITE;
			a1 = (unsigned long)pdata->paddr;
			a2 = (unsigned long)pdata->size;
			break;
		case COMMAND_RECONFIG_STATUS:
			a0 = INTEL_SIP_SMC_FPGA_CONFIG_ISDONE;
			a1 = 0;
			a2 = 0;
			break;
		default:
			/* it shouldn't happen */
			break;
		}
		pr_debug("%s: before SMC call -- a0=0x%016x a1=0x%016x",
			 __func__, (unsigned int)a0, (unsigned int)a1);
		pr_debug(" a2=0x%016x\n", (unsigned int)a2);

		ctrl->invoke_fn(a0, a1, a2, 0, 0, 0, 0, 0, &res);

		pr_debug("%s: after SMC call -- res.a0=0x%016x",
			 __func__, (unsigned int)res.a0);
		pr_debug(" res.a1=0x%016x, res.a2=0x%016x",
			 (unsigned int)res.a1, (unsigned int)res.a2);
		pr_debug(" res.a3=0x%016x\n", (unsigned int)res.a3);

		switch (res.a0) {
		case INTEL_SIP_SMC_STATUS_OK:
			svc_thread_recv_status_ok(pdata, cdata, res);
			break;
		case INTEL_SIP_SMC_FPGA_CONFIG_STATUS_BUSY:
			switch (pdata->command) {
			case COMMAND_RECONFIG_DATA_SUBMIT:
				svc_thread_cmd_data_claim(ctrl,
							  pdata, cdata);
				break;
			case COMMAND_RECONFIG_STATUS:
				svc_thread_cmd_config_status(ctrl,
							     pdata, cdata);
				break;
			default:
				break;
			}
			break;
		case INTEL_SIP_SMC_FPGA_CONFIG_STATUS_REJECTED:
			pr_debug("%s: STATUS_REJECTED\n", __func__);
			break;
		case INTEL_SIP_SMC_FPGA_CONFIG_STATUS_ERROR:
			pr_err("%s: STATUS_ERROR\n", __func__);
			cdata->status = BIT(SVC_STATUS_RECONFIG_ERROR);
			cdata->kaddr1 = NULL;
			cdata->kaddr2 = NULL;
			cdata->kaddr3 = NULL;
			pdata->chan->scl->receive_cb(pdata->chan->scl, cdata);
			break;
		default:
			break;
		}
	};

	kfree(cdata);
	kfree(pdata);

	return 0;
}

/**
 * svc_normal_to_secure_shm_thread() - the function to run in the kthread
 * @data: data pointer for kthread function
 *
 * Service layer driver creates intel_svc_smc_hvc_shm kthread on CPU
 * node 0, its function intel_svc_secure_shm_thread is used to query the
 * physical address of memory block reserved by secure monitor software at
 * secure world.
 *
 * Return: 0
 */
static int svc_normal_to_secure_shm_thread(void *data)
{
	struct intel_svc_sh_memory *sh_mem = (struct intel_svc_sh_memory *)data;
	struct arm_smccc_res res;

	/* SMC or HVC call to get shared memory info from secure world */
	sh_mem->invoke_fn(INTEL_SIP_SMC_FPGA_CONFIG_GET_MEM,
			  0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == INTEL_SIP_SMC_STATUS_OK) {
		sh_mem->addr = res.a1;
		sh_mem->size = res.a2;
	} else {
		pr_err("%s: after SMC call -- res.a0=0x%016x",  __func__,
		       (unsigned int)res.a0);
		sh_mem->addr = 0;
		sh_mem->size = 0;
	}

	complete(&sh_mem->sync_complete);
	do_exit(0);
}

/**
 * svc_get_sh_memory_param() - get memory block reserved by secure monitor SW
 * @pdev: pointer to service layer device
 * @param: pointer to service shared memory structure
 *
 * Return: zero for successfully getting the physical address of memory block
 * reserved by secure monitor software, or negative value on error.
 */
static int svc_get_sh_memory_param(struct platform_device *pdev,
				    struct intel_svc_sh_memory *param)
{
	struct device *dev = &pdev->dev;
	struct task_struct *sh_memory_task;

	init_completion(&param->sync_complete);

	/* smc/hvc call happens on cpu 0 bound kthread */
	sh_memory_task = kthread_create_on_cpu(svc_normal_to_secure_shm_thread,
					       (void *)param,
						0, "svc_smc_hvc_shm_thread");
	if (IS_ERR(sh_memory_task))
		dev_err(dev, "fail to create intel_svc_smc_shm_thread\n");
	wake_up_process(sh_memory_task);

	if (!wait_for_completion_timeout(&param->sync_complete, 10 * HZ)) {
		dev_err(dev,
			"timeout to get sh-memory paras from secure world\n");
		return -ETIMEDOUT;
	}

	if (!param->addr || !param->size) {
		dev_err(dev,
			"fails to get shared memory info from secure world\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "SM software provides paddr: 0x%016x, size: 0x%08x\n",
		(unsigned int)param->addr,
		(unsigned int)param->size);

	return 0;
}

/**
 * svc_create_memory_pool() - create a memory pool from reserved memory block
 * @pdev: pointer to service layer device
 * @param: pointer to service shared memory structure
 *
 * Return: pool allocated from reserved memory block or ERR_PTR() on error.
 */
static struct gen_pool *
svc_create_memory_pool(struct platform_device *pdev,
		       struct intel_svc_sh_memory *param)
{
	struct device *dev = &pdev->dev;
	struct gen_pool *genpool;
	unsigned long vaddr;
	phys_addr_t paddr;
	size_t size;
	phys_addr_t begin;
	phys_addr_t end;
	void *va;
	size_t page_mask = PAGE_SIZE - 1;
	int min_alloc_order = 3;
	int ret;

	begin = roundup(param->addr, PAGE_SIZE);
	end = rounddown(param->addr + param->size, PAGE_SIZE);
	paddr = begin;
	size = end - begin;
	va = memremap(paddr, size, MEMREMAP_WC);
	if (!va) {
		dev_err(dev, "fail to remap shared memory\n");
		return ERR_PTR(-EINVAL);
	}
	vaddr = (unsigned long)va;
	dev_dbg(dev,
		"reserved memory vaddr: %p, paddr: 0x%16x size: 0x%8x\n",
		va, (unsigned int)paddr, (unsigned int)size);
	if ((vaddr & page_mask) || (paddr & page_mask) ||
	    (size & page_mask)) {
		dev_err(dev, "page is not aligned\n");
		return ERR_PTR(-EINVAL);
	}
	genpool = gen_pool_create(min_alloc_order, -1);
	if (!genpool) {
		dev_err(dev, "fail to create genpool\n");
		return ERR_PTR(-ENOMEM);
	}
	gen_pool_set_algo(genpool, gen_pool_best_fit, NULL);
	ret = gen_pool_add_virt(genpool, vaddr, paddr, size, -1);
	if (ret) {
		dev_err(dev, "fail to add memory chunk to the pool\n");
		gen_pool_destroy(genpool);
		return ERR_PTR(ret);
	}

	return genpool;
}

/**
 * svc_smccc_smc() - secure monitor call between normal and secure world
 * @a0-a7: arguments passed in registers 0 to 7
 * @res: result values from register 0 to 3
 */
static void svc_smccc_smc(unsigned long a0, unsigned long a1,
			  unsigned long a2, unsigned long a3,
			  unsigned long a4, unsigned long a5,
			  unsigned long a6, unsigned long a7,
			  struct arm_smccc_res *res)
{
	arm_smccc_smc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

/**
 * svc_smccc_hvc() - hypervisor call between normal and secure world
 * @a0-a7: arguments passed in registers 0 to 7
 * @res: result values from register 0 to 3
 */
static void svc_smccc_hvc(unsigned long a0, unsigned long a1,
			  unsigned long a2, unsigned long a3,
			  unsigned long a4, unsigned long a5,
			  unsigned long a6, unsigned long a7,
			  struct arm_smccc_res *res)
{
	arm_smccc_hvc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

/**
 * get_invoke_func() - invoke SMC or HVC call
 * @dev: pointer to device
 *
 * Return: function pointer to svc_smccc_smc or svc_smccc_hvc.
 */
static svc_invoke_fn *get_invoke_func(struct device *dev)
{
	const char *method;

	if (of_property_read_string(dev->of_node, "method", &method)) {
		dev_warn(dev, "missing \"method\" property\n");
		return ERR_PTR(-ENXIO);
	}

	if (!strcmp(method, "smc"))
		return svc_smccc_smc;
	if (!strcmp(method, "hvc"))
		return svc_smccc_hvc;

	dev_warn(dev, "invalid \"method\" property: %s\n", method);

	return ERR_PTR(-EINVAL);
}

static const struct of_device_id intel_svc_drv_match[] = {
	{.compatible = "intel,stratix10-svc"},
	{},
};

static int intel_svc_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct intel_svc_controller *controller;
	struct intel_svc_chan *chans;
	struct gen_pool *genpool;
	struct intel_svc_sh_memory *sh_memory;
	struct task_struct *task;
	svc_invoke_fn *invoke_fn;
	size_t fifo_size;
	int ret;

	/* get SMC or HVC function */
	invoke_fn = get_invoke_func(dev);
	if (IS_ERR(invoke_fn))
		return -EINVAL;

	sh_memory = devm_kzalloc(dev, sizeof(*sh_memory), GFP_KERNEL);
	if (!sh_memory)
		return -ENOMEM;

	sh_memory->invoke_fn = invoke_fn;
	ret = svc_get_sh_memory_param(pdev, sh_memory);
	if (ret)
		return ret;

	genpool = svc_create_memory_pool(pdev, sh_memory);
	if (!genpool)
		return -ENOMEM;

	/* allocate service controller and supporting channel */
	controller = devm_kzalloc(dev, sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;

	chans = devm_kmalloc_array(dev, SVC_NUM_CHANNEL,
				   sizeof(*chans), GFP_KERNEL | __GFP_ZERO);
	if (!chans)
		return -ENOMEM;

	/* smc or hvc call happens on cpu 0 bound kthread */
	task = kthread_create_on_cpu(svc_normal_to_secure_thread,
				     (void *)controller, 0,
				     "svc_smc_hvc_thread");
	if (IS_ERR(task)) {
		dev_err(dev, "fails to create svc_smc_hvc_thread\n");
		return -EINVAL;
	}

	controller->dev = dev;
	controller->num_chans = SVC_NUM_CHANNEL;
	controller->chans = chans;
	controller->genpool = genpool;
	controller->task = task;
	controller->invoke_fn = invoke_fn;
	init_completion(&controller->complete_status);

	fifo_size = sizeof(struct intel_svc_data) * SVC_NUM_DATA_IN_FIFO;
	ret = kfifo_alloc(&controller->svc_fifo, fifo_size, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "fails to allocate FIFO\n");
		return ret;
	}
	spin_lock_init(&controller->svc_fifo_lock);

	chans[0].scl = NULL;
	chans[0].ctrl = controller;
	chans[0].name = "fpga";
	spin_lock_init(&chans[0].lock);

	chans[1].scl = NULL;
	chans[1].ctrl = controller;
	chans[1].name = "dummy";
	spin_lock_init(&chans[1].lock);

	wake_up_process(controller->task);

	list_add_tail(&controller->node, &svc_ctrl);
	platform_set_drvdata(pdev, controller);

	pr_info("Intel Service Layer Driver Initialized\n");

	return ret;
}

static int intel_svc_drv_remove(struct platform_device *pdev)
{
	struct intel_svc_controller *ctrl = platform_get_drvdata(pdev);

	kfifo_free(&ctrl->svc_fifo);
	kthread_stop(ctrl->task);
	if (ctrl->genpool)
		gen_pool_destroy(ctrl->genpool);
	list_del(&ctrl->node);

	return 0;
}

static struct platform_driver intel_svc_driver = {
	.probe = intel_svc_drv_probe,
	.remove = intel_svc_drv_remove,
	.driver = {
		.name = "intel-svc",
		.of_match_table = intel_svc_drv_match,
	},
};

static int __init intel_svc_init(void)
{
	struct device_node *fw_np;
	struct device_node *np;
	int ret;

	fw_np = of_find_node_by_name(NULL, "firmware");
	if (!fw_np)
		return -ENODEV;

	np = of_find_matching_node(fw_np, intel_svc_drv_match);
	if (!np) {
		of_node_put(fw_np);
		return -ENODEV;
	}

	of_node_put(np);
	ret = of_platform_populate(fw_np, intel_svc_drv_match, NULL, NULL);
	of_node_put(fw_np);
	if (ret)
		return ret;

	return platform_driver_register(&intel_svc_driver);
}

static void __exit intel_svc_exit(void)
{
	return platform_driver_unregister(&intel_svc_driver);
}

subsys_initcall(intel_svc_init);
module_exit(intel_svc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Stratix10 Service Layer Driver");
MODULE_AUTHOR("Richard Gong <richard.gong@intel.com>");
MODULE_ALIAS("platform:intel-svc");
