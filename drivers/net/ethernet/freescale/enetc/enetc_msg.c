// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2017-2019 NXP */

#include "enetc_pf_common.h"

#define ENETC_PF_MSG_SUCCESS	FIELD_PREP(ENETC_PF_MSG_CLASS_ID, \
					   ENETC_MSG_CLASS_ID_CMD_SUCCESS)
#define ENETC_PF_MSG_NOTSUPP	FIELD_PREP(ENETC_PF_MSG_CLASS_ID, \
					   ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT)
#define ENETC_PF_MSG_PERM_DENY	FIELD_PREP(ENETC_PF_MSG_CLASS_ID, \
					   ENETC_MSG_CLASS_ID_PERMISSION_DENY)

static void enetc_msg_disable_mr_int(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 psiier;

	psiier = enetc_rd(hw, ENETC_PSIIER) & ~ENETC_PSIMR_MASK(pf->num_vfs);

	/* disable MR int source(s) */
	enetc_wr(hw, ENETC_PSIIER, psiier);
}

static void enetc_msg_enable_mr_int(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 psiier;

	psiier = enetc_rd(hw, ENETC_PSIIER) | ENETC_PSIMR_MASK(pf->num_vfs);

	enetc_wr(hw, ENETC_PSIIER, psiier);
}

static irqreturn_t enetc_msg_psi_msix(int irq, void *data)
{
	struct enetc_si *si = (struct enetc_si *)data;
	struct enetc_pf *pf = enetc_si_priv(si);

	enetc_msg_disable_mr_int(pf);
	schedule_work(&si->msg_task);

	return IRQ_HANDLED;
}

/* Messaging */
static bool enetc_msg_check_crc16(void *msg_addr, u32 msg_size)
{
	u32 data_size = msg_size - 2;
	u8 *data_buf = msg_addr + 2;
	u16 verify_val;

	verify_val = crc_itu_t(ENETC_CRC_INIT, data_buf, data_size);
	verify_val = crc_itu_t(verify_val, msg_addr, 2);
	if (verify_val)
		return false;

	return true;
}

static u16 enetc_msg_set_vf_primary_mac_addr(struct enetc_pf *pf, int vf_id,
					     void *vf_msg)
{
	struct enetc_vf_state *vf_state = &pf->vf_state[vf_id];
	struct enetc_msg_mac_exact_filter *msg = vf_msg;
	struct device *dev = &pf->si->pdev->dev;
	u16 pf_msg = ENETC_PF_MSG_SUCCESS;
	char *addr = msg->mac[0].addr;

	mutex_lock(&vf_state->lock);

	/* Untrusted VFs cannot set their MAC addresses by the mailbox
	 * messages.
	 */
	if (!(vf_state->flags & ENETC_VF_FLAG_TRUSTED)) {
		pf_msg = ENETC_PF_MSG_PERM_DENY;
		goto vf_state_unlock;
	}

	if (!is_valid_ether_addr(addr)) {
		dev_err_ratelimited(dev, "VF%d attempted to set invalid MAC\n",
				    vf_id);
		pf_msg = FIELD_PREP(ENETC_PF_MSG_CLASS_ID,
				    ENETC_MSG_CLASS_ID_MAC_FILTER) |
			 FIELD_PREP(ENETC_PF_MSG_CLASS_CODE,
				    ENETC_MF_CLASS_CODE_INVALID_MAC);
		goto vf_state_unlock;
	}

	/* PF has higher privileges. If PF has already modified the MAC
	 * address for VF through .ndo_set_vf_mac() interface, VF is not
	 * allowed to set its MAC address via mailbox messages, even if
	 * it is trusted.
	 */
	if (vf_state->flags & ENETC_VF_FLAG_PF_SET_MAC) {
		dev_err_ratelimited(dev,
				    "VF%d attempted to override PF set MAC\n",
				    vf_id);
		pf_msg = FIELD_PREP(ENETC_PF_MSG_CLASS_ID,
				    ENETC_MSG_CLASS_ID_CMD_NOT_PERMITTED);
		goto vf_state_unlock;
	}

	enetc_set_si_hw_addr(pf, vf_id + 1, addr);

vf_state_unlock:
	mutex_unlock(&vf_state->lock);

	return pf_msg;
}

static u16 enetc_msg_handle_mac_filter(struct enetc_pf *pf, int vf_id,
				       void *vf_msg)
{
	struct enetc_msg_header *msg_hdr = vf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_SET_PRIMARY_MAC:
		return enetc_msg_set_vf_primary_mac_addr(pf, vf_id, vf_msg);
	default:
		return ENETC_PF_MSG_NOTSUPP;
	}
}

static u16 enetc_msg_handle_ip_revision(struct enetc_pf *pf, void *vf_msg)
{
	struct enetc_msg_header *msg_hdr = vf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_GET_IP_MN:
		return (FIELD_PREP(ENETC_PF_MSG_CLASS_ID,
				   ENETC_MSG_CLASS_ID_IP_REVISION) |
			FIELD_PREP(ENETC_PF_MSG_CLASS_CODE_U8,
				   pf->si->revision));
	default:
		return ENETC_PF_MSG_NOTSUPP;
	}
}

static void enetc_pf_reply_msg(struct enetc_hw *hw, int vf_id, u16 pf_msg)
{
	/* w1c to clear the corresponding VF MR bit */
	enetc_wr(hw, ENETC_PSIIDR, ENETC_PSIMR_BIT(vf_id));
	enetc_wr(hw, ENETC_PSIMSGRR, ENETC_SIMSGSR_SET_MC(pf_msg) |
		 ENETC_PSIMR_BIT(vf_id));
}

static u16 enetc_build_link_status_msg(struct enetc_ndev_priv *priv,
				       bool link_up)
{
	u8 status = 0;

	if (link_up) {
		if (test_bit(ENETC_RXBDR_CM, &priv->flags))
			status |= ENETC_CLASS_CODE_TX_PAUSE_EN;
	} else {
		status |= ENETC_CLASS_CODE_LINK_DOWN;
	}

	return FIELD_PREP(ENETC_PF_MSG_CLASS_ID,
			  ENETC_MSG_CLASS_ID_LINK_STATUS) |
	       FIELD_PREP(ENETC_PF_MSG_CLASS_CODE_U8, status);
}

static void enetc_msg_get_link_status(struct enetc_pf *pf, int vf_id)
{
	struct enetc_ndev_priv *priv = netdev_priv(pf->si->ndev);
	struct enetc_si *si = pf->si;
	u16 pf_msg;

	spin_lock(&si->gen_lock);
	pf_msg = enetc_build_link_status_msg(priv, pf->link_up);
	enetc_pf_reply_msg(&si->hw, vf_id, pf_msg);
	spin_unlock(&si->gen_lock);
}

static void enetc_msg_register_link_status_notifier(struct enetc_pf *pf,
						    int vf_id)
{
	struct enetc_si *si = pf->si;

	spin_lock(&si->gen_lock);
	enetc_pf_reply_msg(&si->hw, vf_id, ENETC_PF_MSG_SUCCESS);

	/* SR-IOV is being disabled if pf->sriov_enabled is false, so no
	 * need to set link_status_ms_mask and notify the link status.
	 */
	if (!pf->sriov_enabled) {
		spin_unlock(&si->gen_lock);
		return;
	}

	pf->link_status_ms_mask |= PSIMSGSR_MS(vf_id);
	spin_unlock(&si->gen_lock);

	/* Notify VF the current link status */
	queue_work(si->workqueue, &pf->link_status_task);
}

static void enetc_msg_unregister_link_status_notifier(struct enetc_pf *pf,
						      int vf_id)
{
	spin_lock(&pf->si->gen_lock);
	pf->link_status_ms_mask &= ~PSIMSGSR_MS(vf_id);
	enetc_pf_reply_msg(&pf->si->hw, vf_id, ENETC_PF_MSG_SUCCESS);
	spin_unlock(&pf->si->gen_lock);
}

static u16 enetc_msg_handle_link_status(struct enetc_pf *pf, int vf_id,
					void *vf_msg)
{
	struct enetc_msg_header *msg_hdr = vf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_GET_CURRENT_LINK_STATUS:
		/* Currently, this message is intended only for
		 * DPDK-owned VFs.
		 */
		enetc_msg_get_link_status(pf, vf_id);
		break;
	case ENETC_MSG_REGISTER_LINK_CHANGE_NOTIFIER:
		enetc_msg_register_link_status_notifier(pf, vf_id);
		break;
	case ENETC_MSG_UNREGISTER_LINK_CHANGE_NOTIFIER:
		enetc_msg_unregister_link_status_notifier(pf, vf_id);
		break;
	default:
		return ENETC_PF_MSG_NOTSUPP;
	}

	return 0;
}

/* If *pf_msg is set to 0, it means that PF has responded to VF in
 * enetc_msg_handle_rxmsg() through enetc_pf_reply_msg(), which also
 * clears the corresponding VF MR bit in PSIIDR.
 */
static void enetc_msg_handle_rxmsg(struct enetc_pf *pf, int vf_id,
				   u16 *pf_msg)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_header *msg_hdr = msg_swbd->vaddr;
	u32 msg_size = ENETC_MSG_SIZE(msg_hdr->len);
	struct device *dev = &pf->si->pdev->dev;
	u8 *msg;

	if (msg_size > ENETC_DEFAULT_MSG_SIZE) {
		dev_err_ratelimited(dev,
				    "Invalid message size: %u\n", msg_size);
		*pf_msg = FIELD_PREP(ENETC_PF_MSG_CLASS_ID,
				     ENETC_MSG_CLASS_ID_INVALID_MSG_LEN);
		return;
	}

	/* To prevent malicious VF from tampering with the original data by
	 * sending new messages after passing the check, the DMA buffer data
	 * is copied to the msg buffer before validation.
	 */
	msg = kzalloc_objs(*msg, msg_size);
	if (!msg) {
		dev_err_ratelimited(dev,
				    "Failed to allocate message buffer\n");
		*pf_msg = FIELD_PREP(ENETC_PF_MSG_CLASS_ID,
				     ENETC_MSG_CLASS_ID_CMD_FAIL);
		return;
	}

	memcpy(msg, msg_swbd->vaddr, msg_size);
	if (!enetc_msg_check_crc16(msg, msg_size)) {
		dev_err_ratelimited(dev, "VSI to PSI Message CRC16 error\n");
		*pf_msg = FIELD_PREP(ENETC_PF_MSG_CLASS_ID,
				     ENETC_MSG_CLASS_ID_CRC_ERROR);

		goto free_msg;
	}

	/* Default to not supported */
	*pf_msg = ENETC_PF_MSG_NOTSUPP;
	msg_hdr = (struct enetc_msg_header *)msg;

	/* Currently, asynchronous actions are not supported */
	if (FIELD_GET(ENETC_VF_MSG_COOKIE, msg_hdr->cookie)) {
		dev_err_ratelimited(dev,
				    "Cookie field is not supported yet\n");
		goto free_msg;
	}

	/* Currently only support protocol version 0 */
	if (msg_hdr->proto_ver) {
		dev_err_ratelimited(dev, "Unsupported protocol version %u\n",
				    msg_hdr->proto_ver);
		goto free_msg;
	}

	/* The new messages are currently only supported on ENETC v4. If v1
	 * requires them, the current restriction can be lifted.
	 */
	if (is_enetc_rev1(pf->si) &&
	    !(msg_hdr->class_id == ENETC_MSG_CLASS_ID_MAC_FILTER &&
	      msg_hdr->cmd_id == ENETC_MSG_SET_PRIMARY_MAC)) {
		dev_err_ratelimited(dev, "Unsupported message for ENETC v1\n");

		goto free_msg;
	}

	switch (msg_hdr->class_id) {
	case ENETC_MSG_CLASS_ID_MAC_FILTER:
		*pf_msg = enetc_msg_handle_mac_filter(pf, vf_id, msg);
		break;
	case ENETC_MSG_CLASS_ID_IP_REVISION:
		*pf_msg = enetc_msg_handle_ip_revision(pf, msg);
		break;
	case ENETC_MSG_CLASS_ID_LINK_STATUS:
		*pf_msg = enetc_msg_handle_link_status(pf, vf_id, msg);
		break;
	default:
		dev_err_ratelimited(dev,
				    "Unsupported message class ID: 0x%x\n",
				    msg_hdr->class_id);
	}

free_msg:
	kfree(msg);
}

static void enetc_msg_task(struct work_struct *work)
{
	struct enetc_si *si = container_of(work, struct enetc_si, msg_task);
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	u32 mr_status, mr_mask;
	int i;

	mr_mask = ENETC_PSIMR_MASK(pf->num_vfs);
	mr_status = (enetc_rd(hw, ENETC_PSIMSGRR) & mr_mask) |
		    (enetc_rd(hw, ENETC_PSIIDR) & mr_mask);
	if (!mr_status)
		goto out;

	for (i = 0; i < pf->num_vfs; i++) {
		u16 msg_code;

		if (!(ENETC_PSIMR_BIT(i) & mr_status))
			continue;

		enetc_msg_handle_rxmsg(pf, i, &msg_code);

		/* If msg_code is 0, it means that PF has responded to VF
		 * in enetc_msg_handle_rxmsg() through enetc_pf_reply_msg(),
		 * which also clears the corresponding VF MR bit in PSIIDR.
		 */
		if (!msg_code)
			continue;

		enetc_pf_reply_msg(hw, i, msg_code);
	}

out:
	enetc_msg_enable_mr_int(pf);
}

/* Init */
static int enetc_msg_alloc_mbx(struct enetc_si *si, int idx)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct device *dev = &si->pdev->dev;
	struct enetc_hw *hw = &si->hw;
	struct enetc_msg_swbd *msg;
	u32 val;

	msg = &pf->rxmsg[idx];
	/* allocate and set receive buffer */
	msg->size = ENETC_DEFAULT_MSG_SIZE;

	msg->vaddr = dma_alloc_coherent(dev, msg->size, &msg->dma,
					GFP_KERNEL);
	if (!msg->vaddr) {
		dev_err(dev, "msg: fail to alloc dma buffer of size: %d\n",
			msg->size);
		return -ENOMEM;
	}

	/* set multiple of 32 bytes */
	val = lower_32_bits(msg->dma);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR0(idx), val);
	val = upper_32_bits(msg->dma);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR1(idx), val);

	return 0;
}

static void enetc_msg_free_mbx(struct enetc_si *si, int idx)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	struct enetc_msg_swbd *msg;

	enetc_wr(hw, ENETC_PSIVMSGRCVAR0(idx), 0);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR1(idx), 0);

	msg = &pf->rxmsg[idx];
	dma_free_coherent(&si->pdev->dev, msg->size, msg->vaddr, msg->dma);
	memset(msg, 0, sizeof(*msg));
}

static int enetc_msg_psi_init(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;
	int vector, i, err;

	for (i = 0; i < pf->num_vfs; i++) {
		err = enetc_msg_alloc_mbx(si, i);
		if (err)
			goto free_mbx;
	}

	/* initialize PSI mailbox */
	INIT_WORK(&si->msg_task, enetc_msg_task);

	/* register message passing interrupt handler */
	snprintf(si->msg_int_name, sizeof(si->msg_int_name), "%s-vfmsg",
		 si->ndev->name);
	vector = pci_irq_vector(si->pdev, ENETC_SI_INT_IDX);
	err = request_irq(vector, enetc_msg_psi_msix, 0, si->msg_int_name, si);
	if (err) {
		dev_err(&si->pdev->dev,
			"PSI messaging: request_irq() failed!\n");
		goto free_mbx;
	}

	/* set one IRQ entry for PSI message receive notification (SI int) */
	enetc_wr(&si->hw, ENETC_SIMSIVR, ENETC_SI_INT_IDX);

	/* enable MR interrupts */
	enetc_msg_enable_mr_int(pf);

	return 0;

free_mbx:
	for (i--; i >= 0; i--)
		enetc_msg_free_mbx(si, i);

	return err;
}

static void enetc_msg_clear_vf_config(struct enetc_pf *pf, int vf_id)
{
	struct enetc_vf_state *vf_state = &pf->vf_state[vf_id];
	struct enetc_si *si = pf->si;

	/* For ENETC v1, we only support setting the VF's MAC address via
	 * VSI-to-PSI messages, so there is no configuration to clear.
	 */
	if (is_enetc_rev1(si))
		return;

	spin_lock(&si->gen_lock);
	vf_state->msg_fail_cnt = 0;
	spin_unlock(&si->gen_lock);
}

static void enetc_msg_psi_free(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;
	int i;

	/* disable MR interrupts */
	enetc_msg_disable_mr_int(pf);

	/* de-register message passing interrupt handler */
	free_irq(pci_irq_vector(si->pdev, ENETC_SI_INT_IDX), si);

	cancel_work_sync(&si->msg_task);

	/* MR interrupts may be re-enabled by workqueue */
	enetc_msg_disable_mr_int(pf);

	for (i = 0; i < pf->num_vfs; i++) {
		enetc_msg_free_mbx(si, i);
		enetc_msg_clear_vf_config(pf, i);
	}
}

int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf = enetc_si_priv(si);
	int err;

	if (!num_vfs) {
		spin_lock(&si->gen_lock);
		pf->sriov_enabled = false;
		pf->link_status_ms_mask = 0;
		spin_unlock(&si->gen_lock);

		pci_disable_sriov(pdev);
		enetc_msg_psi_free(pf);
		pf->num_vfs = 0;
	} else {
		pf->num_vfs = num_vfs;

		err = enetc_msg_psi_init(pf);
		if (err) {
			dev_err(&pdev->dev, "enetc_msg_psi_init (%d)\n", err);
			goto err_msg_psi;
		}

		/* As PCI SR-IOV is not enabled at the moment, there is no
		 * concurrent access to sriov_enabled. So no need to use
		 * gen_lock to protect sriov_enabled.
		 */
		pf->sriov_enabled = true;
		err = pci_enable_sriov(pdev, num_vfs);
		if (err) {
			dev_err(&pdev->dev, "pci_enable_sriov err %d\n", err);
			goto err_en_sriov;
		}
	}

	return num_vfs;

err_en_sriov:
	/* If pci_enable_sriov() fails after partially creating VFs, a VF
	 * driver that successfully bound to one of the created VFs could
	 * have sent a registration message, setting its bit in
	 * link_status_ms_mask.
	 */
	spin_lock(&si->gen_lock);
	pf->sriov_enabled = false;
	pf->link_status_ms_mask = 0;
	spin_unlock(&si->gen_lock);
	enetc_msg_psi_free(pf);
err_msg_psi:
	pf->num_vfs = 0;

	return err;
}
EXPORT_SYMBOL_GPL(enetc_sriov_configure);

void enetc_pf_send_link_status_msg(struct enetc_pf *pf)
{
	struct enetc_ndev_priv *priv = netdev_priv(pf->si->ndev);
	u16 pf_msg, ms_mask, new_ms_msk, ms_status;
	struct enetc_si *si = pf->si;
	int retry_num = 0;

retry:
	spin_lock(&si->gen_lock);
	ms_mask = pf->link_status_ms_mask;
	/* VFs have unregistered link status notification, return directly  */
	if (!ms_mask)
		goto unlock;

	/* The MS bit is set, indicating that the corresponding VF has not
	 * read the last message, PF cannot send new message to the VF. To
	 * avoid sending messages to such a VF, the bit corresponding to VF
	 * is cleared from ms_mask. Because the MS bit can only be written
	 * as 1, writing a 0 has no effect. Writing a 1 when the bit is
	 * already set is undefined.
	 */
	ms_status = enetc_rd(&si->hw, ENETC_PSIMSGSR) & 0xfffe;
	if ((ms_mask & ms_status) && retry_num++ < 200) {
		spin_unlock(&si->gen_lock);
		/* Wait VFs to handle the last message */
		usleep_range(1000, 1020);
		goto retry;
	}

	/* None of the relevant VFs have processed the previous message, and
	 * the PF has tried 200 times. This situation indicates that VF has
	 * malfunctioned.
	 */
	new_ms_msk = ms_mask & (~ms_status);
	if (!new_ms_msk) {
		dev_err_ratelimited(&si->pdev->dev,
				    "All registered VFs (MS: 0x%x) are busy\n",
				    ms_mask);
		goto ms_status_check;
	}

	if (new_ms_msk != ms_mask)
		dev_warn_ratelimited(&si->pdev->dev,
				     "Failed to notify link status to VFs (MS: 0x%x)\n",
				     ms_mask ^ new_ms_msk);

	pf_msg = enetc_build_link_status_msg(priv, pf->link_up);
	enetc_wr(&si->hw, ENETC_PSIMSGSR,
		 FIELD_PREP(PSIMSGSR_MC, pf_msg) | new_ms_msk);

ms_status_check:
	/* If the PF fails to send messages to the corresponding VF for 10
	 * consecutive times, clear that VF's bit in link_status_ms_mask.
	 */
	for (int i = 0; i < pf->num_vfs; i++) {
		struct enetc_vf_state *vf_state = &pf->vf_state[i];

		if (!(PSIMSGSR_MS(i) & ms_mask))
			continue;

		if (!(PSIMSGSR_MS(i) & ms_status)) {
			vf_state->msg_fail_cnt = 0;
			continue;
		}

		if (vf_state->msg_fail_cnt++ < 10)
			continue;

		vf_state->msg_fail_cnt = 0;
		pf->link_status_ms_mask &= ~PSIMSGSR_MS(i);
		dev_warn_ratelimited(&si->pdev->dev,
				     "Clear VF%d's link status MS bit\n", i);
	}

unlock:
	spin_unlock(&si->gen_lock);
}
EXPORT_SYMBOL_GPL(enetc_pf_send_link_status_msg);

static void enetc_pf_notify_vf_link_status(struct enetc_pf *pf,
					   bool link_up)
{
	struct enetc_si *si = pf->si;

	/* Currently we do not add link status message support for ENETC v1 */
	if (!pf->total_vfs || is_enetc_rev1(si))
		return;

	spin_lock(&si->gen_lock);
	pf->link_up = link_up;
	if (!pf->link_status_ms_mask) {
		spin_unlock(&si->gen_lock);
		return;
	}
	spin_unlock(&si->gen_lock);

	queue_work(si->workqueue, &pf->link_status_task);
}

void enetc_pf_notify_vf_link_up(struct enetc_pf *pf)
{
	enetc_pf_notify_vf_link_status(pf, true);
}
EXPORT_SYMBOL_GPL(enetc_pf_notify_vf_link_up);

void enetc_pf_notify_vf_link_down(struct enetc_pf *pf)
{
	enetc_pf_notify_vf_link_status(pf, false);
}
EXPORT_SYMBOL_GPL(enetc_pf_notify_vf_link_down);
