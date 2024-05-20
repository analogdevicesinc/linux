// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2018, Intel Corporation
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/genalloc.h>
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
#include <linux/firmware/intel/stratix10-smc.h>
#include <linux/firmware/intel/stratix10-svc-client.h>
#include <linux/types.h>
#include <linux/iommu.h>
#include <linux/iova.h>

/**
 * SVC_NUM_DATA_IN_FIFO - number of struct stratix10_svc_data in the FIFO
 *
 * SVC_NUM_CHANNEL - number of channel supported by service layer driver
 *
 * FPGA_CONFIG_DATA_CLAIM_TIMEOUT_MS - claim back the submitted buffer(s)
 * from the secure world for FPGA manager to reuse, or to free the buffer(s)
 * when all bit-stream data had be send.
 *
 * FPGA_CONFIG_POLL_INTERVAL_MS_FAST - interval for polling the service status
 * at secure world for fast response commands. Interval is set to 20ms.
 *
 * FPGA_CONFIG_POLL_INTERVAL_MS_SLOW - interval for polling the service status
 * at secure world for slow response commands. Interval is set to 500ms.
 *
 * FPGA_CONFIG_POLL_COUNT_FAST - number of count for polling service status for
 * fast response commands. Count is set to 50 (50*20ms=1sec)
 *
 * FPGA_CONFIG_POLL_COUNT_SLOW - number of count for polling service status for
 * slow response commands. Count is set to 58 (58*500ms=29sec)
 */
#define SVC_NUM_DATA_IN_FIFO			8
#define SVC_NUM_CHANNEL					4
#define FPGA_CONFIG_DATA_CLAIM_TIMEOUT_MS	2000
#define FPGA_CONFIG_STATUS_TIMEOUT_SEC		30
#define FPGA_CONFIG_POLL_INTERVAL_MS_FAST	20
#define FPGA_CONFIG_POLL_INTERVAL_MS_SLOW	500
#define FPGA_CONFIG_POLL_COUNT_FAST		50
#define FPGA_CONFIG_POLL_COUNT_SLOW		58
#define AGILEX5_SDM_DMA_ADDR_OFFSET		0x80000000
#define BYTE_TO_WORD_SIZE				4
#define IOMMU_LIMIT_ADDR			0x20000000
#define IOMMU_STARTING_ADDR			0x0

/* stratix10 service layer clients */
#define STRATIX10_RSU				"stratix10-rsu"

typedef void (svc_invoke_fn)(unsigned long, unsigned long, unsigned long,
			     unsigned long, unsigned long, unsigned long,
			     unsigned long, unsigned long,
			     struct arm_smccc_res *);
struct stratix10_svc_chan;

/**
 * struct stratix10_svc - svc private data
 * @stratix10_svc_rsu: pointer to stratix10 RSU device
 */
struct stratix10_svc {
	struct platform_device *stratix10_svc_rsu;
	struct platform_device *intel_svc_fcs;
};

/**
 * struct stratix10_svc_sh_memory - service shared memory structure
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
struct stratix10_svc_sh_memory {
	struct completion sync_complete;
	unsigned long addr;
	unsigned long size;
	svc_invoke_fn *invoke_fn;
};

/**
 * struct stratix10_svc_data_mem - service memory structure
 * @vaddr: virtual address
 * @paddr: physical address
 * @size: size of memory
 * @node: link list head node
 *
 * This struct is used in a list that keeps track of buffers which have
 * been allocated or freed from the memory pool. Service layer driver also
 * uses this struct to transfer physical address to virtual address.
 */
struct stratix10_svc_data_mem {
	void *vaddr;
	phys_addr_t paddr;
	size_t size;
	struct list_head node;
};

/**
 * struct stratix10_svc_data - service data structure
 * @chan: service channel
 * @paddr: physical address of to be processed payload
 * @size: to be processed playload size
 * @paddr_output: physical address of processed payload
 * @size_output: processed payload size
 * @command: service command requested by client
 * @flag: configuration type (full or partial)
 * @arg: args to be passed via registers and not physically mapped buffers
 *
 * This struct is used in service FIFO for inter-process communication.
 */
struct stratix10_svc_data {
	struct stratix10_svc_chan *chan;
	phys_addr_t paddr;
	size_t size;
	phys_addr_t paddr_output;
	size_t size_output;
	u32 command;
	u32 flag;
	u64 arg[6];
};

/**
 * struct stratix10_svc_controller - service controller
 * @dev: device
 * @chans: array of service channels
 * @num_chans: number of channels in 'chans' array
 * @num_active_client: number of active service client
 * @node: list management
 * @genpool: memory pool pointing to the memory region
 * @task: pointer to the thread task which handles SMC or HVC call
 * @complete_status: state for completion
 * @invoke_fn: function to issue secure monitor call or hypervisor call
 * @sdm_lock: only allows a single command single response to SDM
 * @domain: pointer to allocated iommu domain
 * @is_smmu_enabled: flag to indicate whether is smmu_enabled for device
 * @carveout: iova_domain used to allocate iova addr that is accessible by SDM
 * @svc: manages the list of client svc drivers
 *
 * This struct is used to create communication channels for service clients, to
 * handle secure monitor or hypervisor call.
 */
struct stratix10_svc_controller {
	struct device *dev;
	struct stratix10_svc_chan *chans;
	int num_chans;
	int num_active_client;
	struct list_head node;
	struct gen_pool *genpool;
	struct completion complete_status;
	svc_invoke_fn *invoke_fn;
	struct mutex *sdm_lock;
	struct iommu_domain *domain;
	bool is_smmu_enabled;
	dma_addr_t sdm_dma_addr_offset;
	struct {
		struct iova_domain domain;
		unsigned long shift;
		unsigned long limit;
	} carveout;
	struct stratix10_svc *svc;
};

/**
 * struct stratix10_svc_chan - service communication channel
 * @ctrl: pointer to service controller which is the provider of this channel
 * @scl: pointer to service client which owns the channel
 * @name: service client name associated with the channel
 * @lock: protect access to the channel
 *
 * This struct is used by service client to communicate with service layer, each
 * service client has its own channel created by service controller.
 */
struct stratix10_svc_chan {
	struct stratix10_svc_controller *ctrl;
	struct stratix10_svc_client *scl;
	char *name;
	struct task_struct *task;
	/* Separate fifo for every channel */
	struct kfifo svc_fifo;
	spinlock_t svc_fifo_lock;
	spinlock_t lock;
};

static LIST_HEAD(svc_ctrl);
static LIST_HEAD(svc_data_mem);

/**
 * svc_pa_to_va() - translate physical address to virtual address
 * @addr: to be translated physical address
 *
 * Return: valid virtual address or NULL if the provided physical
 * address doesn't exist.
 */
static void *svc_pa_to_va(unsigned long addr)
{
	struct stratix10_svc_data_mem *pmem;

	pr_debug("claim back P-addr=0x%016x\n", (unsigned int)addr);
	list_for_each_entry(pmem, &svc_data_mem, node)
		if (pmem->paddr == addr)
			return pmem->vaddr;

	/* physical address is not found */
	return NULL;
}

/**
 * svc_thread_cmd_data_claim() - claim back buffer from the secure world
 * @ctrl: pointer to service layer controller
 * @p_data: pointer to service data structure
 * @cb_data: pointer to callback data structure to service client
 *
 * Claim back the submitted buffers from the secure world and pass buffer
 * back to service client (FPGA manager, etc) for reuse.
 */
static void svc_thread_cmd_data_claim(struct stratix10_svc_controller *ctrl,
				      struct stratix10_svc_data *p_data,
				      struct stratix10_svc_cb_data *cb_data)
{
	struct arm_smccc_res res;
	unsigned long timeout;
	void *buf_claim_addr[4] = {NULL};
	int buf_claim_count = 0;

	reinit_completion(&ctrl->complete_status);
	timeout = msecs_to_jiffies(FPGA_CONFIG_DATA_CLAIM_TIMEOUT_MS);

	pr_debug("%s: claim back the submitted buffer\n", __func__);
	do {
		ctrl->invoke_fn(INTEL_SIP_SMC_FPGA_CONFIG_COMPLETED_WRITE,
				0, 0, 0, 0, 0, 0, 0, &res);

		if (res.a0 == INTEL_SIP_SMC_STATUS_OK) {
			if (!res.a1) {
				/* Transaction of 4 blocks are now done */
				complete(&ctrl->complete_status);
				cb_data->status = BIT(SVC_STATUS_BUFFER_DONE);
				cb_data->kaddr1 = buf_claim_addr[0];
				cb_data->kaddr2 = buf_claim_addr[1];
				cb_data->kaddr3 = buf_claim_addr[2];
				cb_data->kaddr4 = buf_claim_addr[3];
				p_data->chan->scl->receive_cb(p_data->chan->scl,
				cb_data);
				break;
			}
			if (buf_claim_count < 4) {
				buf_claim_addr[buf_claim_count]
				= svc_pa_to_va(res.a1);
				buf_claim_count++;
			}
			if ((res.a2) && (buf_claim_count < 4)) {
				buf_claim_addr[buf_claim_count]
				= svc_pa_to_va(res.a2);
				buf_claim_count++;
			}
			if ((res.a3) && (buf_claim_count < 4)) {
				buf_claim_addr[buf_claim_count]
				= svc_pa_to_va(res.a3);
				buf_claim_count++;
			}
		}
	} while (res.a0 == INTEL_SIP_SMC_STATUS_OK ||
		 res.a0 == INTEL_SIP_SMC_STATUS_BUSY ||
		 wait_for_completion_timeout(&ctrl->complete_status, timeout));
}

/**
 * svc_cmd_poll_status() - poll for status
 * @p_data: pointer to service data structure
 * @ctrl: pointer to service layer controller
 * @res: pointer to store response
 * @poll_count: pointer to poll count value
 * @poll_interval_in_ms: interval value in miliseconds
 *
 * Check whether the service at secure world has completed, and then inform the
 * response.
 */
static void svc_cmd_poll_status(struct stratix10_svc_data *p_data,
				struct stratix10_svc_controller *ctrl,
				struct arm_smccc_res *res,
				int *poll_count, int poll_interval_in_ms)
{
	unsigned long a0, a1, a2;

	a0 = INTEL_SIP_SMC_FPGA_CONFIG_ISDONE;
	a1 = (unsigned long)p_data->paddr;
	a2 = (unsigned long)p_data->size;

	if (p_data->command == COMMAND_POLL_SERVICE_STATUS)
		a0 = INTEL_SIP_SMC_SERVICE_COMPLETED;

	while (*poll_count) {
		ctrl->invoke_fn(a0, a1, a2, 0, 0, 0, 0, 0, res);
		if ((res->a0 == INTEL_SIP_SMC_STATUS_OK) ||
		    (res->a0 == INTEL_SIP_SMC_STATUS_ERROR) ||
		    (res->a0 == INTEL_SIP_SMC_STATUS_REJECTED))
			break;

		/*
		 * request is still in progress, go to sleep then
		 * poll again
		 */
		msleep(poll_interval_in_ms);
		(*poll_count)--;
	}
}

/**
 * svc_thread_cmd_config_status() - check configuration status
 * @ctrl: pointer to service layer controller
 * @p_data: pointer to service data structure
 * @cb_data: pointer to callback data structure to service client
 *
 * Check whether the secure firmware at secure world has finished the FPGA
 * configuration, and then inform FPGA manager the configuration status.
 */
static void svc_thread_cmd_config_status(struct stratix10_svc_controller *ctrl,
					 struct stratix10_svc_data *p_data,
					 struct stratix10_svc_cb_data *cb_data)
{
	struct arm_smccc_res res;
	int poll_count;

	cb_data->kaddr1 = NULL;
	cb_data->kaddr2 = NULL;
	cb_data->kaddr3 = NULL;
	cb_data->status = BIT(SVC_STATUS_ERROR);

	pr_debug("%s: polling config status\n", __func__);

	poll_count = FPGA_CONFIG_POLL_COUNT_FAST;
	svc_cmd_poll_status(p_data, ctrl, &res, &poll_count,
			    FPGA_CONFIG_POLL_INTERVAL_MS_FAST);
	/* Inceased poll interval if response is still not ready */
	if (!poll_count) {
		poll_count = FPGA_CONFIG_POLL_COUNT_SLOW;
		svc_cmd_poll_status(p_data, ctrl, &res, &poll_count,
				    FPGA_CONFIG_POLL_INTERVAL_MS_SLOW);
	}

	if (!poll_count) {
		pr_err("%s: poll status timeout\n", __func__);
		cb_data->status = BIT(SVC_STATUS_BUSY);
	} else if (res.a0 == INTEL_SIP_SMC_STATUS_OK) {
		cb_data->status = BIT(SVC_STATUS_COMPLETED);
		cb_data->kaddr1 = (res.a1) ? &res.a1 : NULL;
		cb_data->kaddr2 = (res.a2) ?
				  svc_pa_to_va(res.a2) : NULL;
		cb_data->kaddr3 = (res.a3) ? &res.a3 : NULL;
	} else {
		pr_err("%s: poll status error\n", __func__);
		cb_data->kaddr1 = &res.a1;
		cb_data->kaddr2 = (res.a2) ?
				  svc_pa_to_va(res.a2) : NULL;
		cb_data->kaddr3 = (res.a3) ? &res.a3 : NULL;
		cb_data->status = BIT(SVC_STATUS_ERROR);
	}

	p_data->chan->scl->receive_cb(p_data->chan->scl, cb_data);
}

/**
 * svc_thread_recv_status_ok() - handle the successful status
 * @p_data: pointer to service data structure
 * @cb_data: pointer to callback data structure to service client
 * @res: result from SMC or HVC call
 *
 * Send back the correspond status to the service clients.
 */
static void svc_thread_recv_status_ok(struct stratix10_svc_data *p_data,
				      struct stratix10_svc_cb_data *cb_data,
				      struct arm_smccc_res res)
{
	cb_data->kaddr1 = NULL;
	cb_data->kaddr2 = NULL;
	cb_data->kaddr3 = NULL;

	switch (p_data->command) {
	case COMMAND_RECONFIG:
	case COMMAND_RSU_UPDATE:
	case COMMAND_RSU_NOTIFY:
	case COMMAND_FCS_REQUEST_SERVICE:
	case COMMAND_FCS_SEND_CERTIFICATE:
	case COMMAND_FCS_DATA_ENCRYPTION:
	case COMMAND_FCS_DATA_DECRYPTION:
	case COMMAND_FCS_GET_PROVISION_DATA:
	case COMMAND_FCS_PSGSIGMA_TEARDOWN:
	case COMMAND_FCS_COUNTER_SET_PREAUTHORIZED:
	case COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD:
	case COMMAND_FCS_CRYPTO_CLOSE_SESSION:
	case COMMAND_FCS_CRYPTO_IMPORT_KEY:
	case COMMAND_FCS_CRYPTO_REMOVE_KEY:
	case COMMAND_FCS_CRYPTO_AES_CRYPT_INIT:
	case COMMAND_FCS_CRYPTO_GET_DIGEST_INIT:
	case COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT:
	case COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT:
	case COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT:
	case COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT:
	case COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT:
		cb_data->status = BIT(SVC_STATUS_OK);
		break;
	case COMMAND_RECONFIG_DATA_SUBMIT:
		cb_data->status = BIT(SVC_STATUS_BUFFER_SUBMITTED);
		break;
	case COMMAND_RECONFIG_STATUS:
		cb_data->status = BIT(SVC_STATUS_COMPLETED);
		break;
	case COMMAND_RSU_RETRY:
	case COMMAND_RSU_MAX_RETRY:
	case COMMAND_RSU_DCMF_STATUS:
	case COMMAND_FIRMWARE_VERSION:
	case COMMAND_HWMON_READTEMP:
	case COMMAND_HWMON_READVOLT:
	case COMMAND_READ_SECURE_REG:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr1 = &res.a1;
		break;
	case COMMAND_SMC_SVC_VERSION:
	case COMMAND_WRITE_TO_SECURE_REG:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr1 = &res.a1;
		cb_data->kaddr2 = &res.a2;
		break;
	case COMMAND_RSU_DCMF_VERSION:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr1 = &res.a1;
		cb_data->kaddr2 = &res.a2;
		break;
	case COMMAND_FCS_RANDOM_NUMBER_GEN:
	case COMMAND_POLL_SERVICE_STATUS:
	case COMMAND_POLL_SERVICE_STATUS_ASYNC:
	case COMMAND_FCS_GET_ROM_PATCH_SHA384:
	case COMMAND_FCS_SDOS_DATA_EXT:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr1 = &res.a1;
		cb_data->kaddr2 = svc_pa_to_va(res.a2);
		cb_data->kaddr3 = &res.a3;
		break;
	case COMMAND_FCS_GET_CHIP_ID:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr2 = &res.a2;
		cb_data->kaddr3 = &res.a3;
		break;
	case COMMAND_FCS_ATTESTATION_SUBKEY:
	case COMMAND_FCS_ATTESTATION_MEASUREMENTS:
	case COMMAND_FCS_ATTESTATION_CERTIFICATE:
	case COMMAND_FCS_CRYPTO_EXPORT_KEY:
	case COMMAND_FCS_CRYPTO_GET_KEY_INFO:
	case COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE:
	case COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE:
	case COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE_SMMU:
	case COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE_SMMU:
	case COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE:
	case COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE:
	case COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE_SMMU:
	case COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE_SMMU:
	case COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE:
	case COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE:
	case COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE_SMMU:
	case COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE_SMMU:
	case COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE_SMMU:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE_SMMU:
	case COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE_SMMU:
	case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE_SMMU:
	case COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE:
	case COMMAND_FCS_CRYPTO_ECDH_REQUEST_FINALIZE:
	case COMMAND_FCS_RANDOM_NUMBER_GEN_EXT:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr2 = svc_pa_to_va(res.a2);
		cb_data->kaddr3 = &res.a3;
		break;
	case COMMAND_FCS_CRYPTO_OPEN_SESSION:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr2 = &res.a2;
		break;
	case COMMAND_MBOX_SEND_CMD:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr1 = &res.a1;
		/* SDM return size in u8. Convert size to u32 word */
		res.a2 = res.a2 * BYTE_TO_WORD_SIZE;
		cb_data->kaddr2 = &res.a2;
		break;
	case COMMAND_RSU_GET_DEVICE_INFO:
		cb_data->status = BIT(SVC_STATUS_OK);
		cb_data->kaddr1 = &res;
		cb_data->kaddr2 = NULL;
		cb_data->kaddr3 = NULL;
		break;
	default:
		pr_warn("it shouldn't happen\n");
		break;
	}

	pr_debug("%s: call receive_cb\n", __func__);
	if (p_data->chan->scl->receive_cb)
		p_data->chan->scl->receive_cb(p_data->chan->scl, cb_data);
}

/**
 * svc_normal_to_secure_thread() - the function to run in the kthread
 * @data: data pointer for kthread function
 *
 * Service layer driver creates stratix10_svc_smc_hvc_call kthread on CPU
 * node 0, its function stratix10_svc_secure_call_thread is used to handle
 * SMC or HVC calls between kernel driver and secure monitor software.
 *
 * Return: 0 for success or -ENOMEM on error.
 */
static int svc_normal_to_secure_thread(void *data)
{
	struct stratix10_svc_chan *chan =  (struct stratix10_svc_chan *)data;
	struct stratix10_svc_controller	*ctrl = chan->ctrl;
	struct stratix10_svc_data *pdata = NULL;
	struct stratix10_svc_cb_data *cbdata = NULL;
	struct arm_smccc_res res;
	unsigned long a0, a1, a2, a3, a4, a5, a6, a7;
	int ret_fifo = 0;
	bool sdm_lock_owned = false;

	pdata =  kmalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	cbdata = kmalloc(sizeof(*cbdata), GFP_KERNEL);
	if (!cbdata) {
		kfree(pdata);
		return -ENOMEM;
	}

	/* default set, to remove build warning */
	a0 = INTEL_SIP_SMC_FPGA_CONFIG_LOOPBACK;
	a1 = 0;
	a2 = 0;
	a3 = 0;
	a4 = 0;
	a5 = 0;
	a6 = 0;
	a7 = 0;

	pr_debug("%s: %s: Thread is running!\n", __func__, chan->name);

	while (!kthread_should_stop()) {

		ret_fifo = kfifo_out_spinlocked(&chan->svc_fifo,
					pdata, sizeof(*pdata),
					&chan->svc_fifo_lock);

		if (!ret_fifo)
			continue;

		pr_debug("get from FIFO pa=0x%016x, command=%u, size=%u\n",
			 (unsigned int)pdata->paddr, pdata->command,
			 (unsigned int)pdata->size);

		/* SDM can only processs one command at a time */
		if (sdm_lock_owned == false) {
			/* Must not do mutex re-lock */
			pr_debug("%s: %s: Thread is waiting for mutex!\n",
			__func__, chan->name);
			mutex_lock(ctrl->sdm_lock);
		}

		sdm_lock_owned = true;

		switch (pdata->command) {
		case COMMAND_RECONFIG_DATA_CLAIM:
			svc_thread_cmd_data_claim(ctrl, pdata, cbdata);
			continue;
		case COMMAND_RECONFIG:
			a0 = INTEL_SIP_SMC_FPGA_CONFIG_START;
			pr_debug("conf_type=%u\n", (unsigned int)pdata->flag);
			a1 = pdata->flag;
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
		case COMMAND_RSU_STATUS:
			a0 = INTEL_SIP_SMC_RSU_STATUS;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_RSU_UPDATE:
			a0 = INTEL_SIP_SMC_RSU_UPDATE;
			a1 = pdata->arg[0];
			a2 = 0;
			break;
		case COMMAND_RSU_NOTIFY:
			a0 = INTEL_SIP_SMC_RSU_NOTIFY;
			a1 = pdata->arg[0];
			a2 = 0;
			break;
		case COMMAND_RSU_RETRY:
			a0 = INTEL_SIP_SMC_RSU_RETRY_COUNTER;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_RSU_MAX_RETRY:
			a0 = INTEL_SIP_SMC_RSU_MAX_RETRY;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_RSU_DCMF_VERSION:
			a0 = INTEL_SIP_SMC_RSU_DCMF_VERSION;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_FIRMWARE_VERSION:
			a0 = INTEL_SIP_SMC_FIRMWARE_VERSION;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_RSU_DCMF_STATUS:
			a0 = INTEL_SIP_SMC_RSU_DCMF_STATUS;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_RSU_GET_DEVICE_INFO:
			a0 = INTEL_SIP_SMC_RSU_GET_DEVICE_INFO;
			a1 = 0;
			a2 = 0;
			break;

		/* for FCS */
		case COMMAND_FCS_DATA_ENCRYPTION:
			a0 = INTEL_SIP_SMC_FCS_CRYPTION;
			a1 = 1;
			a2 = (unsigned long)pdata->paddr;
			a3 = (unsigned long)pdata->size;
			a4 = (unsigned long)pdata->paddr_output;
			a5 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_DATA_DECRYPTION:
			a0 = INTEL_SIP_SMC_FCS_CRYPTION;
			a1 = 0;
			a2 = (unsigned long)pdata->paddr;
			a3 = (unsigned long)pdata->size;
			a4 = (unsigned long)pdata->paddr_output;
			a5 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_RANDOM_NUMBER_GEN:
			a0 = INTEL_SIP_SMC_FCS_RANDOM_NUMBER;
			a1 = (unsigned long)pdata->paddr;
			a2 = 0;
			break;
		case COMMAND_FCS_REQUEST_SERVICE:
			a0 = INTEL_SIP_SMC_FCS_SERVICE_REQUEST;
			a1 = (unsigned long)pdata->paddr;
			a2 = (unsigned long)pdata->size;
			break;
		case COMMAND_FCS_SEND_CERTIFICATE:
			a0 = INTEL_SIP_SMC_FCS_SEND_CERTIFICATE;
			a1 = (unsigned long)pdata->paddr;
			a2 = (unsigned long)pdata->size;
			break;
		case COMMAND_FCS_COUNTER_SET_PREAUTHORIZED:
			a0 = INTEL_SIP_SMC_FCS_COUNTER_SET_PREAUTHORIZED;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			break;
		case COMMAND_FCS_GET_PROVISION_DATA:
			a0 = INTEL_SIP_SMC_FCS_GET_PROVISION_DATA;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_FCS_PSGSIGMA_TEARDOWN:
			a0 = INTEL_SIP_SMC_FCS_PSGSIGMA_TEARDOWN;
			a1 = pdata->arg[0];
			a2 = 0;
			break;
		case COMMAND_FCS_GET_CHIP_ID:
			a0 = INTEL_SIP_SMC_FCS_CHIP_ID;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_FCS_ATTESTATION_SUBKEY:
			a0 = INTEL_SIP_SMC_FCS_ATTESTATION_SUBKEY;
			a1 = (unsigned long)pdata->paddr;
			a2 = (unsigned long)pdata->size;
			a3 = (unsigned long)pdata->paddr_output;
			a4 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_ATTESTATION_MEASUREMENTS:
			a0 = INTEL_SIP_SMC_FCS_ATTESTATION_MEASUREMENTS;
			a1 = (unsigned long)pdata->paddr;
			a2 = (unsigned long)pdata->size;
			a3 = (unsigned long)pdata->paddr_output;
			a4 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_ATTESTATION_CERTIFICATE:
			a0 = INTEL_SIP_SMC_FCS_GET_ATTESTATION_CERTIFICATE;
			a1 = pdata->arg[0];
			a2 = (unsigned long)pdata->paddr_output;
			a3 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD:
			a0 = INTEL_SIP_SMC_FCS_CREATE_CERTIFICATE_ON_RELOAD;
			a1 = pdata->arg[0];
			a2 = 0;
			break;
		/* for crypto service */
		case COMMAND_FCS_CRYPTO_OPEN_SESSION:
			a0 = INTEL_SIP_SMC_FCS_OPEN_CRYPTO_SERVICE_SESSION;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_FCS_CRYPTO_CLOSE_SESSION:
			a0 = INTEL_SIP_SMC_FCS_CLOSE_CRYPTO_SERVICE_SESSION;
			a1 = pdata->arg[0];
			a2 = 0;
			break;

		/* for service key management */
		case COMMAND_FCS_CRYPTO_IMPORT_KEY:
			a0 = INTEL_SIP_SMC_FCS_IMPORT_CRYPTO_SERVICE_KEY;
			a1 = (unsigned long)pdata->paddr;
			a2 = (unsigned long)pdata->size;
			break;
		case COMMAND_FCS_CRYPTO_EXPORT_KEY:
			a0 = INTEL_SIP_SMC_FCS_EXPORT_CRYPTO_SERVICE_KEY;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr_output;
			a4 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_REMOVE_KEY:
			a0 = INTEL_SIP_SMC_FCS_REMOVE_CRYPTO_SERVICE_KEY;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			break;
		case COMMAND_FCS_CRYPTO_GET_KEY_INFO:
			a0 = INTEL_SIP_SMC_FCS_GET_CRYPTO_SERVICE_KEY_INFO;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr_output;
			a4 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_AES_CRYPT_INIT:
			a0 = INTEL_SIP_SMC_FCS_AES_CRYPTO_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = (unsigned long)pdata->paddr;
			a5 = (unsigned long)pdata->size;
			break;
		case COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE:
			a0 = INTEL_SIP_SMC_FCS_AES_CRYPTO_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_AES_CRYPTO_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_AES_CRYPTO_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_AES_CRYPTO_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_GET_DIGEST_INIT:
			a0 = INTEL_SIP_SMC_FCS_GET_DIGEST_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE:
			a0 = INTEL_SIP_SMC_FCS_GET_DIGEST_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_GET_DIGEST_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_GET_DIGEST_SMMU_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_GET_DIGEST_SMMU_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT:
			a0 = INTEL_SIP_SMC_FCS_MAC_VERIFY_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE:
			a0 = INTEL_SIP_SMC_FCS_MAC_VERIFY_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_MAC_VERIFY_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_MAC_VERIFY_SMMU_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_MAC_VERIFY_SMMU_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_HASH_SIGNING_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_HASH_SIGNING_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNING_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNING_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNING_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNING_SMMU_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNING_SMMU_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_HASH_SIGNATURE_VERIFY_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_HASH_SIGNATURE_VERIFY_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNATURE_VERIFY_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNATURE_VERIFY_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNATURE_VERIFY_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNATURE_VERIFY_SMMU_UPDATE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE_SMMU:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_SHA2_DATA_SIGNATURE_VERIFY_SMMU_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			a7 = pdata->arg[2];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_GET_PUBLIC_KEY_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_ECDSA_GET_PUBLIC_KEY_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr_output;
			a4 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT:
			a0 = INTEL_SIP_SMC_FCS_ECDH_INIT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = pdata->arg[3];
			a5 = pdata->arg[4];
			break;
		case COMMAND_FCS_CRYPTO_ECDH_REQUEST_FINALIZE:
			a0 = INTEL_SIP_SMC_FCS_ECDH_FINALIZE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = (unsigned long)pdata->paddr;
			a4 = (unsigned long)pdata->size;
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output;
			break;
		case COMMAND_FCS_RANDOM_NUMBER_GEN_EXT:
			a0 = INTEL_SIP_SMC_FCS_RANDOM_NUMBER_EXT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			break;
		case COMMAND_FCS_SDOS_DATA_EXT:
			a0 = INTEL_SIP_SMC_FCS_CRYPTION_EXT;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			a3 = pdata->arg[2];
			a4 = (unsigned long)pdata->paddr;
			a5 = (unsigned long)pdata->size;
			a6 = (unsigned long)pdata->paddr_output;
			a7 = (unsigned long)pdata->size_output;
			break;
		/* for HWMON */
		case COMMAND_HWMON_READTEMP:
			a0 = INTEL_SIP_SMC_HWMON_READTEMP;
			a1 = pdata->arg[0];
			a2 = 0;
			break;
		case COMMAND_HWMON_READVOLT:
			a0 = INTEL_SIP_SMC_HWMON_READVOLT;
			a1 = pdata->arg[0];
			a2 = 0;
			break;
		/* for polling */
		case COMMAND_POLL_SERVICE_STATUS:
		case COMMAND_POLL_SERVICE_STATUS_ASYNC:
			a0 = INTEL_SIP_SMC_SERVICE_COMPLETED;
			a1 = (unsigned long)pdata->paddr;
			a2 = (unsigned long)pdata->size;
			a3 = pdata->arg[0];
			break;
		case COMMAND_SMC_SVC_VERSION:
			a0 = INTEL_SIP_SMC_SVC_VERSION;
			a1 = 0;
			a2 = 0;
			break;
		case COMMAND_FCS_GET_ROM_PATCH_SHA384:
			a0 = INTEL_SIP_SMC_FCS_GET_ROM_PATCH_SHA384;
			a1 = (unsigned long)pdata->paddr;
			a2 = 0;
			break;
		case COMMAND_MBOX_SEND_CMD:
			a0 = INTEL_SIP_SMC_MBOX_SEND_CMD;
			a1 = pdata->arg[0];
			a2 = (unsigned long)pdata->paddr;
			a3 = (unsigned long)pdata->size / BYTE_TO_WORD_SIZE;
			a4 = pdata->arg[1];
			a5 = (unsigned long)pdata->paddr_output;
			a6 = (unsigned long)pdata->size_output / BYTE_TO_WORD_SIZE;
			break;
		case COMMAND_WRITE_TO_SECURE_REG:
			a0 = INTEL_SIP_SMC_REG_WRITE;
			a1 = pdata->arg[0];
			a2 = pdata->arg[1];
			break;
		case COMMAND_READ_SECURE_REG:
			a0 = INTEL_SIP_SMC_REG_READ;
			a1 = pdata->arg[0];
			break;
		default:
			pr_warn("it shouldn't happen\n");
			break;
		}
		pr_debug("%s: %s: before SMC call -- a0=0x%016x a1=0x%016x",
			 __func__, chan->name,
			 (unsigned int)a0,
			 (unsigned int)a1);
		pr_debug(" a2=0x%016x\n", (unsigned int)a2);
		pr_debug(" a3=0x%016x\n", (unsigned int)a3);
		pr_debug(" a4=0x%016x\n", (unsigned int)a4);
		pr_debug(" a5=0x%016x\n", (unsigned int)a5);
		ctrl->invoke_fn(a0, a1, a2, a3, a4, a5, a6, a7, &res);

		pr_debug("%s: %s: after SMC call -- res.a0=0x%016x",
			 __func__, chan->name, (unsigned int)res.a0);
		pr_debug(" res.a1=0x%016x, res.a2=0x%016x",
			 (unsigned int)res.a1, (unsigned int)res.a2);
		pr_debug(" res.a3=0x%016x\n", (unsigned int)res.a3);

		if (pdata->command == COMMAND_RSU_STATUS) {
			if (res.a0 == INTEL_SIP_SMC_RSU_ERROR)
				cbdata->status = BIT(SVC_STATUS_ERROR);
			else
				cbdata->status = BIT(SVC_STATUS_OK);

			cbdata->kaddr1 = &res;
			cbdata->kaddr2 = NULL;
			cbdata->kaddr3 = NULL;
			pdata->chan->scl->receive_cb(pdata->chan->scl, cbdata);
			mutex_unlock(ctrl->sdm_lock);
			sdm_lock_owned = false;
			continue;
		}

		switch (res.a0) {
		case INTEL_SIP_SMC_STATUS_OK:
			svc_thread_recv_status_ok(pdata, cbdata, res);
			break;
		case INTEL_SIP_SMC_STATUS_BUSY:
			switch (pdata->command) {
			case COMMAND_RECONFIG_DATA_SUBMIT:
				svc_thread_cmd_data_claim(ctrl,
							  pdata, cbdata);
				break;
			case COMMAND_RECONFIG_STATUS:
			case COMMAND_POLL_SERVICE_STATUS:
				svc_thread_cmd_config_status(ctrl,
							     pdata, cbdata);
				break;
			case COMMAND_POLL_SERVICE_STATUS_ASYNC:
				cbdata->status = BIT(SVC_STATUS_BUSY);
				cbdata->kaddr1 = NULL;
				cbdata->kaddr2 = NULL;
				cbdata->kaddr3 = NULL;
				pdata->chan->scl->receive_cb(pdata->chan->scl,
							     cbdata);
				break;
			default:
				pr_warn("it shouldn't happen\n");
				break;
			}
			break;
		case INTEL_SIP_SMC_STATUS_REJECTED:
			pr_debug("%s: STATUS_REJECTED\n", __func__);
			/* for FCS */
			switch (pdata->command) {
			case COMMAND_FCS_REQUEST_SERVICE:
			case COMMAND_FCS_SEND_CERTIFICATE:
			case COMMAND_FCS_GET_PROVISION_DATA:
			case COMMAND_FCS_DATA_ENCRYPTION:
			case COMMAND_FCS_DATA_DECRYPTION:
			case COMMAND_FCS_RANDOM_NUMBER_GEN:
			case COMMAND_FCS_PSGSIGMA_TEARDOWN:
			case COMMAND_FCS_GET_CHIP_ID:
			case COMMAND_FCS_ATTESTATION_SUBKEY:
			case COMMAND_FCS_ATTESTATION_MEASUREMENTS:
			case COMMAND_FCS_COUNTER_SET_PREAUTHORIZED:
			case COMMAND_FCS_ATTESTATION_CERTIFICATE:
			case COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD:
			case COMMAND_FCS_GET_ROM_PATCH_SHA384:
			case COMMAND_FCS_CRYPTO_OPEN_SESSION:
			case COMMAND_FCS_CRYPTO_CLOSE_SESSION:
			case COMMAND_FCS_CRYPTO_IMPORT_KEY:
			case COMMAND_FCS_CRYPTO_EXPORT_KEY:
			case COMMAND_FCS_CRYPTO_REMOVE_KEY:
			case COMMAND_FCS_CRYPTO_GET_KEY_INFO:
			case COMMAND_FCS_CRYPTO_AES_CRYPT_INIT:
			case COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE:
			case COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE:
			case COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE_SMMU:
			case COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE_SMMU:
			case COMMAND_FCS_CRYPTO_GET_DIGEST_INIT:
			case COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE:
			case COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE:
			case COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE_SMMU:
			case COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE_SMMU:
			case COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT:
			case COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE:
			case COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE:
			case COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE_SMMU:
			case COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE_SMMU:
			case COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT:
			case COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE_SMMU:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE_SMMU:
			case COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT:
			case COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE_SMMU:
			case COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE_SMMU:
			case COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT:
			case COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE:
			case COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT:
			case COMMAND_FCS_CRYPTO_ECDH_REQUEST_FINALIZE:
			case COMMAND_FCS_RANDOM_NUMBER_GEN_EXT:
			case COMMAND_FCS_SDOS_DATA_EXT:
			case COMMAND_MBOX_SEND_CMD:
				cbdata->status = BIT(SVC_STATUS_INVALID_PARAM);
				cbdata->kaddr1 = NULL;
				cbdata->kaddr2 = NULL;
				cbdata->kaddr3 = NULL;
				pdata->chan->scl->receive_cb(pdata->chan->scl,
							     cbdata);
				break;
			}
			break;
		case INTEL_SIP_SMC_STATUS_ERROR:
		case INTEL_SIP_SMC_RSU_ERROR:
			pr_err("%s: STATUS_ERROR\n", __func__);
			cbdata->status = BIT(SVC_STATUS_ERROR);
			cbdata->kaddr1 = &res.a1;
			cbdata->kaddr2 = (res.a2) ?
				svc_pa_to_va(res.a2) : NULL;
			cbdata->kaddr3 = (res.a3) ? &res.a3 : NULL;
			pdata->chan->scl->receive_cb(pdata->chan->scl, cbdata);
			break;
		case INTEL_SIP_SMC_STATUS_NO_RESPONSE:
			switch (pdata->command) {
			case COMMAND_POLL_SERVICE_STATUS_ASYNC:
				cbdata->status = BIT(SVC_STATUS_NO_RESPONSE);
				cbdata->kaddr1 = NULL;
				cbdata->kaddr2 = NULL;
				cbdata->kaddr3 = NULL;
				pdata->chan->scl->receive_cb(pdata->chan->scl,
							     cbdata);
				break;
			default:
				pr_warn("it shouldn't receive no response\n");
				break;
			}
			break;
		default:
			pr_warn("Secure firmware doesn't support...\n");

			cbdata->status = BIT(SVC_STATUS_NO_SUPPORT);
			cbdata->kaddr1 = NULL;
			cbdata->kaddr2 = NULL;
			cbdata->kaddr3 = NULL;
			if (pdata->chan->scl->receive_cb)
				pdata->chan->scl->receive_cb(pdata->chan->scl, cbdata);
			break;

		}
	}
	pr_debug("%s: %s: Exit thread\n", __func__, chan->name);
	if (sdm_lock_owned == true)
		mutex_unlock(ctrl->sdm_lock);
	kfree(cbdata);
	kfree(pdata);

	return 0;
}

/**
 * svc_normal_to_secure_shm_thread() - the function to run in the kthread
 * @data: data pointer for kthread function
 *
 * Service layer driver creates stratix10_svc_smc_hvc_shm kthread on CPU
 * node 0, its function stratix10_svc_secure_shm_thread is used to query the
 * physical address of memory block reserved by secure monitor software at
 * secure world.
 *
 * svc_normal_to_secure_shm_thread() terminates directly since it is a
 * standlone thread for which no one will call kthread_stop() or return when
 * 'kthread_should_stop()' is true.
 */
static int svc_normal_to_secure_shm_thread(void *data)
{
	struct stratix10_svc_sh_memory
			*sh_mem = (struct stratix10_svc_sh_memory *)data;
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
	return 0;
}

/**
 * svc_get_sh_memory() - get memory block reserved by secure monitor SW
 * @pdev: pointer to service layer device
 * @sh_memory: pointer to service shared memory structure
 *
 * Return: zero for successfully getting the physical address of memory block
 * reserved by secure monitor software, or negative value on error.
 */
static int svc_get_sh_memory(struct platform_device *pdev,
				    struct stratix10_svc_sh_memory *sh_memory)
{
	struct device *dev = &pdev->dev;
	struct task_struct *sh_memory_task;
	unsigned int cpu = 0;

	init_completion(&sh_memory->sync_complete);

	/* smc or hvc call happens on cpu 0 bound kthread */
	sh_memory_task = kthread_create_on_node(svc_normal_to_secure_shm_thread,
					       (void *)sh_memory,
						cpu_to_node(cpu),
						"svc_smc_hvc_shm_thread");
	if (IS_ERR(sh_memory_task)) {
		dev_err(dev, "fail to create stratix10_svc_smc_shm_thread\n");
		return -EINVAL;
	}

	wake_up_process(sh_memory_task);

	if (!wait_for_completion_timeout(&sh_memory->sync_complete, 10 * HZ)) {
		dev_err(dev,
			"timeout to get sh-memory paras from secure world\n");
		return -ETIMEDOUT;
	}

	if (!sh_memory->addr || !sh_memory->size) {
		dev_err(dev,
			"failed to get shared memory info from secure world\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "SM software provides paddr: 0x%016x, size: 0x%08x\n",
		(unsigned int)sh_memory->addr,
		(unsigned int)sh_memory->size);

	return 0;
}

/**
 * svc_create_memory_pool() - create a memory pool from reserved memory block
 * @pdev: pointer to service layer device
 * @sh_memory: pointer to service shared memory structure
 *
 * Return: pool allocated from reserved memory block or ERR_PTR() on error.
 */
static struct gen_pool *
svc_create_memory_pool(struct platform_device *pdev,
		       struct stratix10_svc_sh_memory *sh_memory)
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

	begin = roundup(sh_memory->addr, PAGE_SIZE);
	end = rounddown(sh_memory->addr + sh_memory->size, PAGE_SIZE);
	paddr = begin;
	size = end - begin;
	va = devm_memremap(dev, paddr, size, MEMREMAP_WC);
	if (IS_ERR(va)) {
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
 * @a0: argument passed in registers 0
 * @a1: argument passed in registers 1
 * @a2: argument passed in registers 2
 * @a3: argument passed in registers 3
 * @a4: argument passed in registers 4
 * @a5: argument passed in registers 5
 * @a6: argument passed in registers 6
 * @a7: argument passed in registers 7
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
 * @a0: argument passed in registers 0
 * @a1: argument passed in registers 1
 * @a2: argument passed in registers 2
 * @a3: argument passed in registers 3
 * @a4: argument passed in registers 4
 * @a5: argument passed in registers 5
 * @a6: argument passed in registers 6
 * @a7: argument passed in registers 7
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

/**
 * stratix10_svc_request_channel_byname() - request a service channel
 * @client: pointer to service client
 * @name: service client name
 *
 * This function is used by service client to request a service channel.
 *
 * Return: a pointer to channel assigned to the client on success,
 * or ERR_PTR() on error.
 */
struct stratix10_svc_chan *stratix10_svc_request_channel_byname(
	struct stratix10_svc_client *client, const char *name)
{
	struct device *dev = client->dev;
	struct stratix10_svc_controller *controller;
	struct stratix10_svc_chan *chan = NULL;
	unsigned long flag;
	int i;

	/* if probe was called after client's, or error on probe */
	if (list_empty(&svc_ctrl))
		return ERR_PTR(-EPROBE_DEFER);

	controller = list_first_entry(&svc_ctrl,
				      struct stratix10_svc_controller, node);
	for (i = 0; i < SVC_NUM_CHANNEL; i++) {
		if (!strcmp(controller->chans[i].name, name)) {
			chan = &controller->chans[i];
			break;
		}
	}

	/* if there was no channel match */
	if (i == SVC_NUM_CHANNEL) {
		dev_err(dev, "%s: channel not allocated\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	if (chan->scl || !try_module_get(controller->dev->driver->owner)) {
		dev_dbg(dev, "%s: svc not free\n", __func__);
		return ERR_PTR(-EBUSY);
	}

	spin_lock_irqsave(&chan->lock, flag);
	chan->scl = client;
	chan->ctrl->num_active_client++;
	spin_unlock_irqrestore(&chan->lock, flag);

	return chan;
}
EXPORT_SYMBOL_GPL(stratix10_svc_request_channel_byname);

/**
 * stratix10_svc_free_channel() - free service channel
 * @chan: service channel to be freed
 *
 * This function is used by service client to free a service channel.
 */
void stratix10_svc_free_channel(struct stratix10_svc_chan *chan)
{
	unsigned long flag;

	spin_lock_irqsave(&chan->lock, flag);
	chan->scl = NULL;
	chan->ctrl->num_active_client--;
	module_put(chan->ctrl->dev->driver->owner);
	spin_unlock_irqrestore(&chan->lock, flag);
}
EXPORT_SYMBOL_GPL(stratix10_svc_free_channel);

/**
 * stratix10_svc_send() - send a message data to the remote
 * @chan: service channel assigned to the client
 * @msg: message data to be sent, in the format of
 * "struct stratix10_svc_client_msg"
 *
 * This function is used by service client to add a message to the service
 * layer driver's queue for being sent to the secure world.
 *
 * Return: 0 for success, -ENOMEM or -ENOBUFS on error.
 */
int stratix10_svc_send(struct stratix10_svc_chan *chan, void *msg)
{
	struct stratix10_svc_client_msg
		*p_msg = (struct stratix10_svc_client_msg *)msg;
	struct stratix10_svc_data_mem *p_mem;
	struct stratix10_svc_data *p_data;
	int ret = 0;
	unsigned int cpu = 0;
	phys_addr_t *src_addr;
	phys_addr_t *dst_addr;

	p_data = kzalloc(sizeof(*p_data), GFP_KERNEL);
	if (!p_data)
		return -ENOMEM;

	/* first client will create kernel thread */
	if (!chan->task) {
		chan->task =
			kthread_create_on_node(svc_normal_to_secure_thread,
					      (void *)chan,
					      cpu_to_node(cpu),
					      "svc_smc_hvc_thread");
			if (IS_ERR(chan->task)) {
				dev_err(chan->ctrl->dev,
					"failed to create svc_smc_hvc_thread\n");
				kfree(p_data);
				return -EINVAL;
			}
		kthread_bind(chan->task, cpu);
		wake_up_process(chan->task);
	}

	pr_debug("%s: %s: sent P-va=%p, P-com=%x, P-size=%u\n", __func__,
		 chan->name, p_msg->payload, p_msg->command,
		 (unsigned int)p_msg->payload_length);

	if (list_empty(&svc_data_mem)) {
		if (p_msg->command == COMMAND_RECONFIG) {
			struct stratix10_svc_command_config_type *ct =
				(struct stratix10_svc_command_config_type *)
				p_msg->payload;
			p_data->flag = ct->flags;
		}
	} else {
		if (p_msg->command == COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE_SMMU ||
				p_msg->command == COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE_SMMU){
			src_addr = (phys_addr_t *)p_msg->payload;
			p_data->paddr = *src_addr;
			p_data->size = p_msg->payload_length;
			dst_addr = (phys_addr_t *)p_msg->payload_output;
			p_data->paddr_output = *dst_addr;
			p_data->size_output = p_msg->payload_length_output;
		} else if (
			p_msg->command ==
				COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE_SMMU ||
			p_msg->command ==
				COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE_SMMU ||
			p_msg->command ==
				COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE_SMMU ||
			p_msg->command ==
				COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE_SMMU ||
			p_msg->command ==
				COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE_SMMU ||
			p_msg->command ==
				COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE_SMMU ||
			p_msg->command ==
				COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE_SMMU ||
			p_msg->command ==
				COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE_SMMU) {
			src_addr = (phys_addr_t *)p_msg->payload;
			p_data->paddr = *src_addr;
			p_data->size = p_msg->payload_length;
			list_for_each_entry(p_mem, &svc_data_mem, node)
				if (p_mem->vaddr == p_msg->payload_output) {
					p_data->paddr_output = p_mem->paddr;
					p_data->size_output = p_msg->payload_length_output;
					break;
				}
		} else {
			list_for_each_entry(p_mem, &svc_data_mem, node)
				if (p_mem->vaddr == p_msg->payload) {
					p_data->paddr = p_mem->paddr;
					p_data->size = p_msg->payload_length;
					if(p_msg->command == COMMAND_RECONFIG_DATA_SUBMIT && chan->ctrl->is_smmu_enabled)
						p_data->paddr += chan->ctrl->sdm_dma_addr_offset;
				}
			if (p_msg->payload_output) {
				list_for_each_entry(p_mem, &svc_data_mem, node)
					if (p_mem->vaddr == p_msg->payload_output) {
						p_data->paddr_output =
							(p_msg->command == COMMAND_MBOX_SEND_CMD
							&& chan->ctrl->is_smmu_enabled) ?
							virt_to_phys(p_mem->vaddr) : p_mem->paddr;
						p_data->size_output =
							p_msg->payload_length_output;
						break;
					}
			}
		}
	}

	p_data->command = p_msg->command;
	p_data->arg[0] = p_msg->arg[0];
	p_data->arg[1] = p_msg->arg[1];
	p_data->arg[2] = p_msg->arg[2];
	p_data->arg[3] = p_msg->arg[3];
	p_data->arg[4] = p_msg->arg[4];
	p_data->arg[5] = p_msg->arg[5];
	p_data->chan = chan;
	pr_debug("%s: %s: put to FIFO pa=0x%016x, cmd=%x, size=%u\n",
			__func__,
			chan->name,
			(unsigned int)p_data->paddr,
			p_data->command,
			(unsigned int)p_data->size);

	ret = kfifo_in_spinlocked(&chan->svc_fifo, p_data,
					sizeof(*p_data),
					&chan->svc_fifo_lock);

	kfree(p_data);

	if (!ret)
		return -ENOBUFS;

	return 0;
}
EXPORT_SYMBOL_GPL(stratix10_svc_send);

/**
 * stratix10_svc_done() - complete service request transactions
 * @chan: service channel assigned to the client
 *
 * This function should be called when client has finished its request
 * or there is an error in the request process. It allows the service layer
 * to stop the running thread to have maximize savings in kernel resources.
 */
void stratix10_svc_done(struct stratix10_svc_chan *chan)
{
	/* stop thread when thread is running */
	if (chan->task) {

		if (!IS_ERR(chan->task)) {
			struct task_struct *task_to_stop = chan->task;

			chan->task = NULL;
			pr_debug("%s: %s: svc_smc_hvc_shm_thread is stopping\n",
					__func__, chan->name);
			kthread_stop(task_to_stop);
		}

		chan->task = NULL;
	}
	pr_debug("%s: %s: svc_smc_hvc_shm_thread has stopped\n",
					__func__, chan->name);
}
EXPORT_SYMBOL_GPL(stratix10_svc_done);

/**
 * stratix10_svc_allocate_memory() - allocate memory
 * @chan: service channel assigned to the client
 * @size: memory size requested by a specific service client
 *
 * Service layer allocates the requested number of bytes buffer from the
 * memory pool, service client uses this function to get allocated buffers.
 *
 * Return: address of allocated memory on success, or ERR_PTR() on error.
 */
void *stratix10_svc_allocate_memory(struct stratix10_svc_chan *chan,
				    size_t size)
{
	struct stratix10_svc_data_mem *pmem;
	unsigned long va_gen_pool;
	phys_addr_t pa;
	struct gen_pool *genpool = chan->ctrl->genpool;
	size_t s;
	void *va;
	int ret;
	struct iova *alloc;
	dma_addr_t dma_addr;

	pmem = devm_kzalloc(chan->ctrl->dev, sizeof(*pmem), GFP_KERNEL);
	if (!pmem)
		return ERR_PTR(-ENOMEM);

	if (chan->ctrl->is_smmu_enabled == true) {
		s = PAGE_ALIGN(size);
		va = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO | __GFP_DMA, get_order(s));
		if (!va) {
			pr_debug("%s get_free_pages_failes\n", __func__);
			return ERR_PTR(-ENOMEM);
		}

		alloc = alloc_iova(&chan->ctrl->carveout.domain,
					s >> chan->ctrl->carveout.shift,
					chan->ctrl->carveout.limit >> chan->ctrl->carveout.shift,
					true);

		dma_addr = iova_dma_addr(&chan->ctrl->carveout.domain, alloc);

		ret = iommu_map(chan->ctrl->domain, dma_addr, virt_to_phys(va),
				s, IOMMU_READ | IOMMU_WRITE | IOMMU_MMIO | IOMMU_CACHE,
				GFP_KERNEL);
		if (ret < 0) {
			pr_debug("%s IOMMU map failed\n", __func__);
			free_iova(&chan->ctrl->carveout.domain,
						iova_pfn(&chan->ctrl->carveout.domain,
									dma_addr));
			free_pages((unsigned long)va, get_order(size));
			return ERR_PTR(-ENOMEM);
		}

		pmem->paddr = dma_addr;
	} else {
		s = roundup(size, 1 << genpool->min_alloc_order);

		va_gen_pool = gen_pool_alloc(genpool, s);
		if (!va_gen_pool)
			return ERR_PTR(-ENOMEM);

		va = (void *)va_gen_pool;

		memset(va, 0, s);
		pa = gen_pool_virt_to_phys(genpool, va_gen_pool);

		pmem->paddr = pa;
	}

	pmem->vaddr = va;
	pmem->size = s;
	list_add_tail(&pmem->node, &svc_data_mem);
	pr_debug("%s: %s: va=%p, pa=0x%016x\n", __func__,
		chan->name, pmem->vaddr, (unsigned int)pmem->paddr);

	return (void *)va;
}
EXPORT_SYMBOL_GPL(stratix10_svc_allocate_memory);

/**
 * stratix10_svc_free_memory() - free allocated memory
 * @chan: service channel assigned to the client
 * @kaddr: memory to be freed
 *
 * This function is used by service client to free allocated buffers.
 */
void stratix10_svc_free_memory(struct stratix10_svc_chan *chan, void *kaddr)
{
	struct stratix10_svc_data_mem *pmem;

	list_for_each_entry(pmem, &svc_data_mem, node)
		if (pmem->vaddr == kaddr) {
			if (chan->ctrl->is_smmu_enabled) {
				iommu_unmap(chan->ctrl->domain, pmem->paddr, pmem->size);
				free_iova(&chan->ctrl->carveout.domain,
							iova_pfn(&chan->ctrl->carveout.domain,
										pmem->paddr));
				free_pages((unsigned long)pmem->vaddr, get_order(pmem->size));
			} else {
				gen_pool_free(chan->ctrl->genpool,
					(unsigned long)kaddr, pmem->size);
				pmem->vaddr = NULL;
			}
			list_del(&pmem->node);
			return;
		}

	list_del(&svc_data_mem);
}
EXPORT_SYMBOL_GPL(stratix10_svc_free_memory);

static const struct of_device_id stratix10_svc_drv_match[] = {
	{.compatible = "intel,stratix10-svc"},
	{.compatible = "intel,agilex-svc"},
	{.compatible = "intel,agilex5-svc"},
	{},
};

static DEFINE_MUTEX(mailbox_lock);

static int stratix10_svc_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stratix10_svc_controller *controller;
	struct stratix10_svc_chan *chans;
	struct gen_pool *genpool;
	struct stratix10_svc_sh_memory *sh_memory;
	struct stratix10_svc *svc;
	struct device_node *node = pdev->dev.of_node;

	svc_invoke_fn *invoke_fn;
	size_t fifo_size;
	int ret;
	unsigned long order;

	/* get SMC or HVC function */
	invoke_fn = get_invoke_func(dev);
	if (IS_ERR(invoke_fn))
		return -EINVAL;

	sh_memory = devm_kzalloc(dev, sizeof(*sh_memory), GFP_KERNEL);
	if (!sh_memory)
		return -ENOMEM;

	sh_memory->invoke_fn = invoke_fn;
	ret = svc_get_sh_memory(pdev, sh_memory);
	if (ret)
		return ret;

	genpool = svc_create_memory_pool(pdev, sh_memory);
	if (IS_ERR(genpool))
		return PTR_ERR(genpool);

	/* allocate service controller and supporting channel */
	controller = devm_kzalloc(dev, sizeof(*controller), GFP_KERNEL);
	if (!controller) {
		ret = -ENOMEM;
		goto err_destroy_pool;
	}

	chans = devm_kmalloc_array(dev, SVC_NUM_CHANNEL,
				   sizeof(*chans), GFP_KERNEL | __GFP_ZERO);
	if (!chans) {
		ret = -ENOMEM;
		goto err_destroy_pool;
	}

	controller->dev = dev;
	controller->num_chans = SVC_NUM_CHANNEL;
	controller->num_active_client = 0;
	controller->chans = chans;
	controller->genpool = genpool;
	controller->invoke_fn = invoke_fn;
	controller->sdm_dma_addr_offset = 0x0;
	init_completion(&controller->complete_status);

	if (of_device_is_compatible(node, "intel,agilex5-svc")) {
		if (iommu_present(&platform_bus_type) &&
			device_property_read_bool(dev, "altr,smmu_enable_quirk")) {
			controller->is_smmu_enabled = true;
			controller->sdm_dma_addr_offset = AGILEX5_SDM_DMA_ADDR_OFFSET;
			pr_debug("Intel Service Layer Driver: IOMMU Present\n");
			controller->domain = iommu_get_dma_domain(dev);

			if (!controller->domain) {
				pr_debug("Intel Service Layer Driver: Error IOMMU domain\n");
				ret = -ENODEV;
				goto err_destroy_pool;
			} else {
				ret = iova_cache_get();
				if (ret < 0) {
					pr_debug("Intel Service Layer Driver: IOVA cache failed\n");
					iommu_domain_free(controller->domain);
					ret = -ENODEV;
					goto err_destroy_pool;
				}
				ret = iommu_attach_device(controller->domain, dev);
				if (ret) {
					pr_debug("Intel Service Layer Driver: Error IOMMU attach failed\n");
					iova_cache_put();
					iommu_domain_free(controller->domain);
					ret = -ENODEV;
					goto err_destroy_pool;
				}
			}

			order = __ffs(controller->domain->pgsize_bitmap);
			init_iova_domain(&controller->carveout.domain, 1UL << order,
				 IOMMU_STARTING_ADDR);

			controller->carveout.shift = iova_shift(&controller->carveout.domain);
			controller->carveout.limit = IOMMU_LIMIT_ADDR;
		} else {
			pr_debug("Intel Service Layer Driver: IOMMU Not Present\n");
			ret = -ENODEV;
			goto err_destroy_pool;
		}
	}

	/* This mutex is used to block threads from utilizing
	 * SDM to prevent out of order command tx
	 */
	controller->sdm_lock = &mailbox_lock;

	fifo_size = sizeof(struct stratix10_svc_data) * SVC_NUM_DATA_IN_FIFO;

	chans[0].scl = NULL;
	chans[0].ctrl = controller;
	chans[0].name = SVC_CLIENT_FPGA;
	spin_lock_init(&chans[0].lock);
	ret = kfifo_alloc(&chans[0].svc_fifo, fifo_size, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "failed to allocate FIFO 0\n");
		return ret;
	}
	spin_lock_init(&chans[0].svc_fifo_lock);

	chans[1].scl = NULL;
	chans[1].ctrl = controller;
	chans[1].name = SVC_CLIENT_RSU;
	spin_lock_init(&chans[1].lock);
	ret = kfifo_alloc(&chans[1].svc_fifo, fifo_size, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "failed to allocate FIFO 1\n");
		return ret;
	}
	spin_lock_init(&chans[1].svc_fifo_lock);

	chans[2].scl = NULL;
	chans[2].ctrl = controller;
	chans[2].name = SVC_CLIENT_FCS;
	spin_lock_init(&chans[2].lock);
	ret = kfifo_alloc(&chans[2].svc_fifo, fifo_size, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "failed to allocate FIFO 2\n");
		return ret;
	}
	spin_lock_init(&chans[2].svc_fifo_lock);

	chans[3].scl = NULL;
	chans[3].ctrl = controller;
	chans[3].name = SVC_CLIENT_HWMON;
	spin_lock_init(&chans[3].lock);
	ret = kfifo_alloc(&chans[3].svc_fifo, fifo_size, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "failed to allocate FIFO 3\n");
		return ret;
	}
	spin_lock_init(&chans[3].svc_fifo_lock);

	chans[3].scl = NULL;
	chans[3].ctrl = controller;
	chans[3].name = SVC_CLIENT_HWMON;
	spin_lock_init(&chans[3].lock);

	list_add_tail(&controller->node, &svc_ctrl);
	platform_set_drvdata(pdev, controller);

	/* add svc client device(s) */
	svc = devm_kzalloc(dev, sizeof(*svc), GFP_KERNEL);
	if (!svc) {
		ret = -ENOMEM;
		return ret;
	}

	controller->svc = svc;

	svc->stratix10_svc_rsu = platform_device_alloc(STRATIX10_RSU, 0);
	if (!svc->stratix10_svc_rsu) {
		dev_err(dev, "failed to allocate %s device\n", STRATIX10_RSU);
		return -ENOMEM;
	}

	ret = platform_device_add(svc->stratix10_svc_rsu);
	if (ret)
		goto err_put_device;

	pr_info("Intel Service Layer Driver Initialized\n");

	return 0;

err_put_device:
	platform_device_put(svc->stratix10_svc_rsu);
err_destroy_pool:
	gen_pool_destroy(genpool);

	return ret;
}

static int stratix10_svc_drv_remove(struct platform_device *pdev)
{
	int i;
	struct stratix10_svc_controller *ctrl = platform_get_drvdata(pdev);
	struct stratix10_svc *svc = ctrl->svc;

	if (ctrl->domain) {
		put_iova_domain(&ctrl->carveout.domain);
		iova_cache_put();
		iommu_detach_device(ctrl->domain, &pdev->dev);
		iommu_domain_free(ctrl->domain);
	}

	platform_device_unregister(svc->intel_svc_fcs);
	platform_device_unregister(svc->stratix10_svc_rsu);

	for (i = 0; i < SVC_NUM_CHANNEL; i++) {
		if (ctrl->chans[i].task) {
			kthread_stop(ctrl->chans[i].task);
			ctrl->chans[i].task = NULL;
		}
		kfifo_free(&ctrl->chans[i].svc_fifo);
	}

	if (ctrl->genpool)
		gen_pool_destroy(ctrl->genpool);
	list_del(&ctrl->node);

	return 0;
}

static struct platform_driver stratix10_svc_driver = {
	.probe = stratix10_svc_drv_probe,
	.remove = stratix10_svc_drv_remove,
	.driver = {
		.name = "stratix10-svc",
		.of_match_table = stratix10_svc_drv_match,
	},
};

static int __init stratix10_svc_init(void)
{
	struct device_node *fw_np;
	struct device_node *np;
	int ret;

	fw_np = of_find_node_by_name(NULL, "firmware");
	if (!fw_np)
		return -ENODEV;

	np = of_find_matching_node(fw_np, stratix10_svc_drv_match);
	if (!np)
		return -ENODEV;

	of_node_put(np);
	ret = of_platform_populate(fw_np, stratix10_svc_drv_match, NULL, NULL);
	if (ret)
		return ret;

	return platform_driver_register(&stratix10_svc_driver);
}

static void __exit stratix10_svc_exit(void)
{
	return platform_driver_unregister(&stratix10_svc_driver);
}

subsys_initcall(stratix10_svc_init);
module_exit(stratix10_svc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Stratix10 Service Layer Driver");
MODULE_AUTHOR("Richard Gong <richard.gong@intel.com>");
MODULE_ALIAS("platform:stratix10-svc");
