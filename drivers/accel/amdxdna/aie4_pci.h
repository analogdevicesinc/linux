/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#ifndef _AIE4_PCI_H_
#define _AIE4_PCI_H_

#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "aie.h"
#include "aie4_msg_priv.h"
#include "amdxdna_mailbox.h"

/* Default context switch hysteresis timeout in microseconds. */
#define AIE4_CTX_HYSTERESIS_US	1000

struct cert_comp {
	struct amdxdna_dev_hdl          *ndev;
	u32                             msix_idx;
	int                             irq;
	struct kref                     kref;
	wait_queue_head_t               waitq;
};

/*
 * aie4 kernel-submission job states (stored in amdxdna_sched_job priv.aie4.state).
 * Anonymous enum - the aie4_job_state identifier is already a field-access macro.
 */
enum {
	AIE4_JOB_STATE_INIT,
	AIE4_JOB_STATE_PENDING,
	AIE4_JOB_STATE_SUBMITTING,
	AIE4_JOB_STATE_SUBMITTED,
	AIE4_JOB_STATE_DONE,
};

struct amdxdna_hwctx_priv {
	struct amdxdna_hwctx		*hwctx;
	struct amdxdna_gem_obj          *umq_bo;
	u64                             *umq_read_index;
	u64                             *umq_write_index;

	struct cert_comp                *cert_comp;
	u32                             hw_ctx_id;
	bool                            has_error;

	/* Direct and indirect packet storage aliasing umq_bo. */
	u64                             write_index;
	struct host_queue_packet        *umq_pkts;
	struct host_indirect_packet_data *umq_indirect_pkts;
	u64                             umq_indirect_pkts_dev_addr;
	/* Transport doorbell target set by aie4_doorbell_setup(). */
	void                    __iomem *doorbell_addr;

	struct mutex                    io_lock; /* serialize submit, protect job lists */
	struct list_head                pending_job_list;
	/* Head of pending_job_list, read locklessly by wait conditions. */
	struct amdxdna_sched_job        *pending_head;
	struct list_head                running_job_list;
	wait_queue_head_t               job_list_wq;
	struct work_struct              job_work;
	struct workqueue_struct         *job_work_q;
};

struct amdxdna_dev_priv {
	const char              *npufw_path;
	const char              *certfw_path;
	u32			mbox_bar;
	u32			mbox_rbuf_bar;
	u64			mbox_info_off;
	u32			doorbell_off;

	struct aie_bar_off_pair	psp_regs_off[PSP_MAX_REGS];
	struct aie_bar_off_pair	smu_regs_off[SMU_MAX_REGS];

	const struct dpm_clk_freq	*dpm_clk_tbl;
	const struct aie_hw_ops		*hw_ops;
};

struct amdxdna_dev_hdl {
	struct aie_device		aie;
	const struct amdxdna_dev_priv	*priv;
	void			__iomem *mbox_base;
	void			__iomem *rbuf_base;
	void			__iomem *doorbell_base;

	struct mailbox			*mbox;
	u32				partition_id;
	u32				total_col;
	u32				max_aieclk_level;
	u32				max_npuhclk_level;
	u32				num_vfs;

	struct dpm_clk_freq		dpm_clk_tbl[AIE4_MAX_DPM_LEVEL_COUNT];

	struct xarray                   cert_comp_xa; /* device level indexed by msix id */
	struct mutex                    cert_comp_lock; /* protects cert_comp operations*/

	void				*work_buf;
	dma_addr_t			work_buf_addr;
	u32				work_buf_size;

	void				*fw_log_buf;
	dma_addr_t			fw_log_buf_addr;
	u32				fw_log_buf_size;

	u8				pw_mode;

	/* Context switch hysteresis timeout in microseconds. */
	u32				ctx_switch_hysteresis_us;

	struct amdxdna_drm_query_firmware_version cert_version;
};

enum aie4_fw_feature {
	AIE4_HSA_COMMAND = 5,
	AIE4_FEATURE_MAX
};

/* aie4_message.c */
int aie4_query_aie_metadata(struct amdxdna_dev_hdl *ndev,
			    struct amdxdna_drm_query_aie_metadata *metadata);
int aie4_query_aie_version(struct amdxdna_dev_hdl *ndev,
			   struct amdxdna_drm_query_aie_version *version);
int aie4_suspend_fw(struct amdxdna_dev_hdl *ndev);
int aie4_attach_work_buffer(struct amdxdna_dev_hdl *ndev);
int aie4_start_fw_log(struct amdxdna_dev_hdl *ndev, u32 level);
int aie4_query_npu_firmware_version(struct amdxdna_dev_hdl *ndev,
				    struct amdxdna_drm_query_firmware_version *fw_version);
int aie4_query_cert_firmware_version(struct amdxdna_dev_hdl *ndev,
				     struct amdxdna_drm_query_firmware_version *cert_version);
int aie4_init_dpm_freq_table(struct amdxdna_dev_hdl *ndev);
int aie4_query_dpm_level(struct amdxdna_dev_hdl *ndev,
			 u32 *aieclk_dpm_level, u32 *npuhclk_dpm_level);
int aie4_msg_set_power_mode(struct amdxdna_dev_hdl *ndev, u8 power_mode);
int aie4_set_runtime_cfg(struct amdxdna_dev_hdl *ndev, u32 type,
			 const void *data, size_t size);
int aie4_set_ctx_hysteresis(struct amdxdna_dev_hdl *ndev, u32 timeout_us);
u32 aie4_msg_pasid(struct amdxdna_client *client);

/* aie4_ctx.c */
enum aie4_hwctx_flags {
	AIE4_HWCTX_NORMAL = 0,
	AIE4_HWCTX_GRACEFUL,
	AIE4_HWCTX_DISCONNECT, /* sets has_error, do not destroy context */
	AIE4_HWCTX_ERROR, /* sets has_error, destroy context */
};

int aie4_hwctx_init(struct amdxdna_hwctx *hwctx);
void aie4_hwctx_stop(struct amdxdna_hwctx *hwctx);
void aie4_hwctx_fini(struct amdxdna_hwctx *hwctx);
int aie4_cmd_wait(struct amdxdna_hwctx *hwctx, u64 seq, u32 timeout);
int aie4_cmd_submit(struct amdxdna_hwctx *hwctx, struct amdxdna_sched_job *job, u64 *seq);
int aie4_hwctx_create(struct amdxdna_hwctx *hwctx);
void aie4_hwctx_destroy(struct amdxdna_hwctx *hwctx, enum aie4_hwctx_flags);
int aie4_hwctx_wait_for_running(struct amdxdna_hwctx *hwctx);
void aie4_hwctx_resume_jobs(struct amdxdna_hwctx *hwctx);

/* aie4_pci.c */
void aie4_hwctx_suspend_all(struct amdxdna_dev_hdl *ndev, int clean_jobs);
int aie4_hwctx_resume_all(struct amdxdna_dev_hdl *ndev);
int aie4_restore_power_mode(struct amdxdna_dev_hdl *ndev);

/*
 * Transport hooks implemented by PCI backend. Doorbell hooks are wired during
 * kernel queue creation and command submission in later patches in the series.
 */
struct aie4_msg_create_hw_context_resp;
int aie4_doorbell_setup(struct amdxdna_hwctx *hwctx,
			const struct aie4_msg_create_hw_context_resp *resp);
void aie4_doorbell_ring(struct amdxdna_hwctx *hwctx);
int aie4_request_notification(struct cert_comp *comp);
void aie4_free_notification(struct cert_comp *comp);

/* aie4_sriov.c */
#if IS_ENABLED(CONFIG_PCI_IOV)
int aie4_create_vfs(struct amdxdna_dev_hdl *ndev, int num_vfs);
int aie4_sriov_configure(struct amdxdna_dev *xdna, int num_vfs);
int aie4_sriov_stop(struct amdxdna_dev_hdl *ndev);
#else
static inline int aie4_create_vfs(struct amdxdna_dev_hdl *ndev, int num_vfs)
{
	return 0;
}

#define aie4_sriov_configure NULL

static inline int aie4_sriov_stop(struct amdxdna_dev_hdl *ndev)
{
	return 0;
}
#endif

extern const struct amdxdna_dev_ops aie4_pf_ops;
extern const struct amdxdna_dev_ops aie4_vf_ops;
extern const struct amdxdna_dev_ops aie4_classic_ops;

#endif /* _AIE4_PCI_H_ */
