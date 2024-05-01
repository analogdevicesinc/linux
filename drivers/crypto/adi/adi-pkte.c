// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Packet engine driver for Analog Devices Incorporated
 *
 * Currently tested on SC598 processor
 *
 * Copyright (c) 2023 - Timesys Corporation
 *   Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 */

#include <linux/cacheflush.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <crypto/engine.h>
#include <crypto/scatterwalk.h>
#include <linux/dma-mapping.h>

/*Hashing*/
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/internal/hash.h>

/*Symmetric Crytography*/
#include <crypto/aes.h>
#include <crypto/internal/des.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>

#include "adi-pkte.h"
#include "adi-pkte-hash.h"
#include "adi-pkte-skcipher.h"

DECLARE_WAIT_QUEUE_HEAD(wq_processing);
int processing;

DECLARE_WAIT_QUEUE_HEAD(wq_ready);
bool ready;

struct adi_drv adi = {
	.dev_list = LIST_HEAD_INIT(adi.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(adi.lock),
};


struct adi_dev *adi_find_dev(struct adi_ctx *ctx)
{
	struct adi_dev *pkte_dev = NULL, *tmp;

	spin_lock_bh(&adi.lock);
	if (!ctx->pkte_dev) {
		list_for_each_entry(tmp, &adi.dev_list, list) {
			pkte_dev = tmp;
			break;
		}
		ctx->pkte_dev = pkte_dev;
	} else {
		pkte_dev = ctx->pkte_dev;
	}

	spin_unlock_bh(&adi.lock);
	return pkte_dev;
}

u32 adi_read(struct adi_dev *pkte_dev, u32 offset)
{
	return readl(pkte_dev->io_base + offset);
}

void adi_write(struct adi_dev *pkte_dev, u32 offset, u32 value)
{
	writel(value, pkte_dev->io_base + offset);
}

u32 adi_physical_address(struct adi_dev *pkte_dev, void *var_addr)
{
	u32 var_addr_32 = *((u32 *) var_addr);
	u32 pkte_device_32 = *((u32 *) pkte_dev->pkte_device);

#ifdef PKTE_USE_SRAM
	return PKTE_SRAM_ADDRESS + var_addr_32 - pkte_device_32;
#else
	return pkte_dev->dma_handle + var_addr_32 - pkte_device_32;
#endif
}

static void adi_reset_pkte(struct adi_dev *pkte_dev)
{
	u32 pkte_cfg;

	//Reset the packet engine
	pkte_cfg = adi_read(pkte_dev, CFG_OFFSET);
	adi_write(pkte_dev, CFG_OFFSET, pkte_cfg | 0x03);
	adi_write(pkte_dev, CFG_OFFSET, pkte_cfg & ~0x03);
}

void adi_reset_state(struct adi_dev *hdev)
{
	hdev->src_count_set = 0;
	hdev->src_bytes_available = 0;
	hdev->ring_pos_produce = 0;
	hdev->ring_pos_consume = 0;
	hdev->flags &= ~(PKTE_FLAGS_STARTED |
			 PKTE_FLAGS_COMPLETE |
			 PKTE_FLAGS_FINAL |
			 PKTE_FLAGS_FINUP |
			 PKTE_FLAGS_HMAC | PKTE_FLAGS_HMAC_KEY_PREPARED);
}

int adi_wait_for_bit(struct adi_dev *pkte_dev, u32 reg_offset, u32 bitmask)
{
	u32 start_time, elapsed_time;
	u32 pkte_status;

	start_time = ktime_get_seconds();
	pkte_status = adi_read(pkte_dev, reg_offset) & bitmask;
	while (!pkte_status) {
		elapsed_time = ktime_get_seconds() - start_time;
		if (elapsed_time == PKTE_OP_TIMEOUT)
			return -EIO;

		pkte_status = adi_read(pkte_dev, reg_offset) & bitmask;
		/* Be nice to others and don't block */
		cond_resched();
	}

	return 0;
}

static irqreturn_t adi_irq_handler(int irq, void *dev_id)
{
	struct adi_dev *pkte_dev = dev_id;
	u32 pkte_imsk_stat, pkte_imsk_disable;
	struct ADI_PKTE_LIST *pkte_list;

	pkte_list = &pkte_dev->pkte_device->pPkteList;
	pkte_imsk_stat = adi_read(pkte_dev, IMSK_STAT_OFFSET);
	dev_dbg(pkte_dev->dev, "%s %x\n", __func__, pkte_imsk_stat);
	if (pkte_imsk_stat & BITM_PKTE_IMSK_EN_RINGERR) {
		dev_dbg(pkte_dev->dev, "%s: RINGERR, check error codes %x",
			__func__, adi_read(pkte_dev, RING_STAT_OFFSET));
		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_RINGERR);
	}

	if (pkte_imsk_stat & BITM_PKTE_IMSK_EN_PROCERR) {
		dev_dbg(pkte_dev->dev, "%s: PROCERR, check error codes %x",
			__func__, adi_read(pkte_dev, CTL_STAT_OFFSET));
		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_PROCERR);
	}

	if (pkte_imsk_stat & BITM_PKTE_IMSK_EN_RDRTHRSH) {
		if (pkte_dev->flags & PKTE_TCM_MODE)
			adi_write(pkte_dev, RDSC_DECR_OFFSET, 0x1);

		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
		pkte_imsk_disable = adi_read(pkte_dev, IMSK_DIS_OFFSET);

		adi_write(pkte_dev, IMSK_DIS_OFFSET,
			  pkte_imsk_disable | BITM_PKTE_IMSK_EN_RDRTHRSH);

		if (pkte_list->pCommand.final_hash_condition == final_hash) {
			ready = 1;
			pkte_dev->flags |= PKTE_FLAGS_COMPLETE;
			wake_up_interruptible(&wq_ready);
		}

		if (pkte_dev->flags & PKTE_TCM_MODE) {
			processing = 0;
			wake_up_interruptible(&wq_processing);
		}
	}

	if (pkte_imsk_stat & BITM_PKTE_IMSK_EN_OPDN) {
		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_OPDN);
		pkte_imsk_disable = adi_read(pkte_dev, IMSK_DIS_OFFSET);
		adi_write(pkte_dev, IMSK_DIS_OFFSET,
			  pkte_imsk_disable | BITM_PKTE_IMSK_EN_OPDN |
			  BITM_PKTE_IMSK_EN_IBUFTHRSH);
		adi_write(pkte_dev, RDSC_DECR_OFFSET, 0x1);

		if (pkte_list->pCommand.final_hash_condition == final_hash) {
			ready = 1;
			pkte_dev->flags |= PKTE_FLAGS_COMPLETE;
			wake_up_interruptible(&wq_ready);
		}

		processing = 0;
		wake_up_interruptible(&wq_processing);
	}

	if (pkte_imsk_stat & BITM_PKTE_IMSK_EN_IBUFTHRSH) {
		if (pkte_dev->src_bytes_available) {
#ifdef CONFIG_CRYPTO_DEV_ADI_HASH
			adi_write_packet(pkte_dev, &pkte_list->pSource[0]);
#endif
			adi_write(pkte_dev, INT_CLR_OFFSET,
				  BITM_PKTE_IMSK_EN_IBUFTHRSH);
		} else {
			adi_write(pkte_dev, INT_EN_OFFSET, 0);
		}

	}

	return IRQ_HANDLED;
}

static void adi_init_state(struct adi_dev *pkte_dev)
{
	struct STATE *state;

	state = &pkte_dev->pkte_device->pPkteDescriptor.State;
	adi_write(pkte_dev, STATE_IV_OFFSET(0), state->STATE_IV0);
	adi_write(pkte_dev, STATE_IV_OFFSET(1), state->STATE_IV1);
	adi_write(pkte_dev, STATE_IV_OFFSET(2), state->STATE_IV2);
	adi_write(pkte_dev, STATE_IV_OFFSET(3), state->STATE_IV3);
	adi_write(pkte_dev, STATE_BYTE_CNT_OFFSET(0), state->STATE_BYTE_CNT0);
	adi_write(pkte_dev, STATE_BYTE_CNT_OFFSET(1), state->STATE_BYTE_CNT1);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(0), state->STATE_IDIGEST0);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(1), state->STATE_IDIGEST1);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(2), state->STATE_IDIGEST2);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(3), state->STATE_IDIGEST3);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(4), state->STATE_IDIGEST4);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(5), state->STATE_IDIGEST5);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(6), state->STATE_IDIGEST6);
	adi_write(pkte_dev, STATE_IDIGEST_OFFSET(7), state->STATE_IDIGEST7);
}

void adi_init_spe(struct adi_dev *pkte_dev)
{
	u8 pos;
	struct PE_CDR *pkte_command_desc;
	struct ADI_PKTE_DESCRIPTOR *pkte_desc;

	pkte_desc = &pkte_dev->pkte_device->pPkteDescriptor;

	if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE)
		pos = pkte_dev->ring_pos_consume;
	else
		pos = 0;

	pkte_command_desc = &pkte_desc->CmdDescriptor[pos];

	adi_write(pkte_dev, CTL_STAT_OFFSET, pkte_command_desc->PE_CTRL_STAT);
	adi_write(pkte_dev, SRC_ADDR_OFFSET, pkte_command_desc->PE_SOURCE_ADDR);
	adi_write(pkte_dev, DEST_ADDR_OFFSET, pkte_command_desc->PE_DEST_ADDR);
	adi_write(pkte_dev, SA_ADDR_OFFSET, pkte_command_desc->PE_SA_ADDR);
	adi_write(pkte_dev, STATE_ADDR_OFFSET, pkte_command_desc->PE_STATE_ADDR);
	adi_write(pkte_dev, USERID_OFFSET, pkte_command_desc->PE_USER_ID);
}

static void adi_init_sa(struct adi_dev *pkte_dev)
{
	struct ADI_PKTE_DESCRIPTOR *pkte_desc;
	struct SA *sa_rec;

	pkte_desc = &pkte_dev->pkte_device->pPkteDescriptor;
	sa_rec = &pkte_desc->SARecord[pkte_dev->ring_pos_consume];
	adi_write(pkte_dev, SA_CMD_OFFSET(0), sa_rec->SA_Para.SA_CMD0);
	adi_write(pkte_dev, SA_CMD_OFFSET(1), sa_rec->SA_Para.SA_CMD1);
	adi_write(pkte_dev, SA_KEY_OFFSET(0), sa_rec->SA_Key.SA_KEY0);
	adi_write(pkte_dev, SA_KEY_OFFSET(1), sa_rec->SA_Key.SA_KEY1);
	adi_write(pkte_dev, SA_KEY_OFFSET(2), sa_rec->SA_Key.SA_KEY2);
	adi_write(pkte_dev, SA_KEY_OFFSET(3), sa_rec->SA_Key.SA_KEY3);
	adi_write(pkte_dev, SA_KEY_OFFSET(4), sa_rec->SA_Key.SA_KEY4);
	adi_write(pkte_dev, SA_KEY_OFFSET(5), sa_rec->SA_Key.SA_KEY5);
	adi_write(pkte_dev, SA_KEY_OFFSET(6), sa_rec->SA_Key.SA_KEY6);
	adi_write(pkte_dev, SA_KEY_OFFSET(7), sa_rec->SA_Key.SA_KEY7);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(0),
		  sa_rec->SA_Idigest.SA_IDIGEST0);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(1),
		  sa_rec->SA_Idigest.SA_IDIGEST1);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(2),
		  sa_rec->SA_Idigest.SA_IDIGEST2);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(3),
		  sa_rec->SA_Idigest.SA_IDIGEST3);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(4),
		  sa_rec->SA_Idigest.SA_IDIGEST4);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(5),
		  sa_rec->SA_Idigest.SA_IDIGEST5);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(6),
		  sa_rec->SA_Idigest.SA_IDIGEST6);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(7),
		  sa_rec->SA_Idigest.SA_IDIGEST7);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(0),
		  sa_rec->SA_Odigest.SA_ODIGEST0);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(1),
		  sa_rec->SA_Odigest.SA_ODIGEST1);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(2),
		  sa_rec->SA_Odigest.SA_ODIGEST2);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(3),
		  sa_rec->SA_Odigest.SA_ODIGEST3);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(4),
		  sa_rec->SA_Odigest.SA_ODIGEST4);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(5),
		  sa_rec->SA_Odigest.SA_ODIGEST5);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(6),
		  sa_rec->SA_Odigest.SA_ODIGEST6);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(7),
		  sa_rec->SA_Odigest.SA_ODIGEST7);
}

void adi_init_ring(struct adi_dev *pkte_dev)
{
	struct ADI_PKTE_DEVICE *pkte;
	struct ADI_PKTE_DESCRIPTOR *pkte_desc;
	u32 addr;

	pkte = pkte_dev->pkte_device;
	pkte_desc = &pkte->pPkteDescriptor;
	addr =
	    adi_physical_address(pkte_dev,
				 (void *)&pkte_desc->CmdDescriptor[0]);
	adi_write(pkte_dev, CDRBASE_ADDR_OFFSET, addr);
	addr =
	    adi_physical_address(pkte_dev,
				 (void *)&pkte_desc->ResultDescriptor[0]);

	adi_write(pkte_dev, RDRBASE_ADDR_OFFSET, addr);
	addr = (PKTE_RING_BUFFERS - 1) << BITP_PKTE_RING_CFG_RINGSZ;
	adi_write(pkte_dev, RING_CFG_OFFSET, addr);
	addr =
	    0 << BITP_PKTE_RING_THRESH_CDRTHRSH | 0 <<
	    BITP_PKTE_RING_THRESH_RDRTHRSH;
	adi_write(pkte_dev, RING_THRESH_OFFSET, addr);
}

void adi_source_data(struct adi_dev *pkte_dev, u32 size)
{
	u8 pos;
	struct PE_CDR *pkte_command_desc;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;

	if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE)
		pos = pkte_dev->ring_pos_consume;
	else
		pos = 0;

	pkte_command_desc = &pkte->pPkteDescriptor.CmdDescriptor[pos];
	if ((!pkte_dev->src_count_set) ||
	    (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE))) {
		pkte_dev->src_count_set = 1;
		pkte->pPkteList.nSrcSize = size;
		pkte_command_desc->PE_LENGTH =
		    BITM_PKTE_LEN_HSTRDY | pkte->pPkteList.nSrcSize;

		if (!(pkte_dev->flags & PKTE_AUTONOMOUS_MODE))
			adi_write(pkte_dev, LEN_OFFSET, pkte_command_desc->PE_LENGTH);
		dev_dbg(pkte_dev->dev, "%s: LEN set to %x\n", __func__,
			pkte_command_desc->PE_LENGTH);
	}
}

void adi_configure_cdr(struct adi_dev *pkte_dev)
{
	struct ADI_PKTE_DEVICE *pkte;
	struct ADI_PKTE_DESCRIPTOR *pkte_desc;
	struct PE_CDR *pkte_command_desc;
	struct SA *sa_rec;
	u8 pos;
	void *var_addr;

	pkte = pkte_dev->pkte_device;
	pkte_desc = &pkte->pPkteDescriptor;
	sa_rec = &pkte_desc->SARecord[pkte_dev->ring_pos_consume];
	if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE)
		pos = pkte_dev->ring_pos_consume;
	else
		pos = 0;

	pkte_command_desc = &pkte_desc->CmdDescriptor[pos];
	pkte_command_desc->PE_CTRL_STAT =
	    0x1 | (pkte->pPkteList.pCommand.final_hash_condition << 4);

	var_addr = (void *)&pkte->source[pkte_dev->ring_pos_consume][0];
	pkte_command_desc->PE_SOURCE_ADDR =
		adi_physical_address(pkte_dev, var_addr);

	pkte_command_desc->PE_DEST_ADDR =
	    adi_physical_address(pkte_dev, (void *)&pkte->destination[0]);

	pkte_command_desc->PE_SA_ADDR =
	    adi_physical_address(pkte_dev, (void *)&sa_rec->SA_Para.SA_CMD0);

	pkte_command_desc->PE_STATE_ADDR =
	    adi_physical_address(pkte_dev, (void *)&pkte_desc->State.STATE_IV0);

	pkte_command_desc->PE_USER_ID = pkte->pPkteList.nUserID;
}

void adi_config_sa_para(struct adi_dev *pkte_dev)
{
	struct ADI_PKTE_DEVICE *pkte;
	struct ADI_PKTE_COMMAND *pkte_cmd;
	struct SA *sa_rec;

	pkte = pkte_dev->pkte_device;
	pkte_cmd = &pkte->pPkteList.pCommand;
	sa_rec = &pkte->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume];
	sa_rec->SA_Para.SA_CMD0 =
	    (pkte_cmd->opcode << BITP_PKTE_SA_CMD0_OPCD) |
	    (pkte_cmd->direction << BITP_PKTE_SA_CMD0_DIR) |
	    (0x0 << BITP_PKTE_SA_CMD0_OPGRP) |
	    (0x3 << BITP_PKTE_SA_CMD0_PADTYPE) |
	    (pkte_cmd->cipher << BITP_PKTE_SA_CMD0_CIPHER) |
	    (pkte_cmd->hash << BITP_PKTE_SA_CMD0_HASH) |
	    0x1 << BITP_PKTE_SA_CMD0_SCPAD |
	    (pkte_cmd->digest_length << BITP_PKTE_SA_CMD0_DIGESTLEN) |
	    (0x2 << BITP_PKTE_SA_CMD0_IVSRC) |
	    (pkte_cmd->hash_source << BITP_PKTE_SA_CMD0_HASHSRC) |
	    0x0 << BITP_PKTE_SA_CMD0_SVIV | 0x1 << BITP_PKTE_SA_CMD0_SVHASH;

	sa_rec->SA_Para.SA_CMD1 =
	    0x1 << BITP_PKTE_SA_CMD1_CPYDGST |
	    (pkte_cmd->hash_mode << BITP_PKTE_SA_CMD1_HMAC) |
	    (pkte_cmd->cipher_mode << BITP_PKTE_SA_CMD1_CIPHERMD) |
	    (pkte_cmd->aes_key_length << BITP_PKTE_SA_CMD1_AESKEYLEN) |
	    (pkte_cmd->aes_des_key << BITP_PKTE_SA_CMD1_AESDECKEY);
}

void adi_config_sa_key(struct adi_dev *pkte_dev, const u32 Key[])
{
	struct SA *sa_rec;
	struct ADI_PKTE_DESCRIPTOR *pkte_desc;

	pkte_desc = &pkte_dev->pkte_device->pPkteDescriptor;
	sa_rec = &pkte_desc->SARecord[pkte_dev->ring_pos_consume];
	sa_rec->SA_Key.SA_KEY0 = pkte_dev->Key[0];
	sa_rec->SA_Key.SA_KEY1 = pkte_dev->Key[1];
	sa_rec->SA_Key.SA_KEY2 = pkte_dev->Key[2];
	sa_rec->SA_Key.SA_KEY3 = pkte_dev->Key[3];
	sa_rec->SA_Key.SA_KEY4 = pkte_dev->Key[4];
	sa_rec->SA_Key.SA_KEY5 = pkte_dev->Key[5];
	sa_rec->SA_Key.SA_KEY6 = pkte_dev->Key[6];
	sa_rec->SA_Key.SA_KEY7 = pkte_dev->Key[7];
}

static void adi_config_idigest(struct adi_dev *pkte_dev, const u32 Idigest[])
{
	struct SA *sa_rec;
	struct ADI_PKTE_DESCRIPTOR *pkte_desc;

	pkte_desc = &pkte_dev->pkte_device->pPkteDescriptor;
	sa_rec = &pkte_desc->SARecord[pkte_dev->ring_pos_consume];
	sa_rec->SA_Idigest.SA_IDIGEST0 = pkte_dev->Idigest[0];
	sa_rec->SA_Idigest.SA_IDIGEST1 = pkte_dev->Idigest[1];
	sa_rec->SA_Idigest.SA_IDIGEST2 = pkte_dev->Idigest[2];
	sa_rec->SA_Idigest.SA_IDIGEST3 = pkte_dev->Idigest[3];
	sa_rec->SA_Idigest.SA_IDIGEST4 = pkte_dev->Idigest[4];
	sa_rec->SA_Idigest.SA_IDIGEST5 = pkte_dev->Idigest[5];
	sa_rec->SA_Idigest.SA_IDIGEST6 = pkte_dev->Idigest[6];
	sa_rec->SA_Idigest.SA_IDIGEST7 = pkte_dev->Idigest[7];
}

static void adi_config_odigest(struct adi_dev *pkte_dev, const u32 Odigest[])
{
	struct SA *sa_rec;
	struct ADI_PKTE_DESCRIPTOR *pkte_desc;

	pkte_desc = &pkte_dev->pkte_device->pPkteDescriptor;
	sa_rec = &pkte_desc->SARecord[pkte_dev->ring_pos_consume];
	sa_rec->SA_Odigest.SA_ODIGEST0 = pkte_dev->Odigest[0];
	sa_rec->SA_Odigest.SA_ODIGEST1 = pkte_dev->Odigest[1];
	sa_rec->SA_Odigest.SA_ODIGEST2 = pkte_dev->Odigest[2];
	sa_rec->SA_Odigest.SA_ODIGEST3 = pkte_dev->Odigest[3];
	sa_rec->SA_Odigest.SA_ODIGEST4 = pkte_dev->Odigest[4];
	sa_rec->SA_Odigest.SA_ODIGEST5 = pkte_dev->Odigest[5];
	sa_rec->SA_Odigest.SA_ODIGEST6 = pkte_dev->Odigest[6];
	sa_rec->SA_Odigest.SA_ODIGEST7 = pkte_dev->Odigest[7];
}

void adi_config_state(struct adi_dev *pkte_dev, u32 IV[])
{
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
#ifdef DEBUG_PKTE
	dev_dbg(pkte_dev->dev, "%s IV: %x %x %x %x\n", __func__, IV[0], IV[1],
		IV[2], IV[3]);
#endif
	pkte->pPkteDescriptor.State.STATE_IV0 = pkte_dev->IV[0];
	pkte->pPkteDescriptor.State.STATE_IV1 = pkte_dev->IV[1];
	pkte->pPkteDescriptor.State.STATE_IV2 = pkte_dev->IV[2];
	pkte->pPkteDescriptor.State.STATE_IV3 = pkte_dev->IV[3];
	pkte->pPkteDescriptor.State.STATE_BYTE_CNT0 = 0x0;
	pkte->pPkteDescriptor.State.STATE_BYTE_CNT1 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST0 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST1 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST2 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST3 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST4 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST5 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST6 = 0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST7 = 0x0;
}

void adi_start_engine(struct adi_dev *pkte_dev)
{
	u32 i, pkte_cfg;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	//Set up configuration structures
	adi_configure_cdr(pkte_dev);
	adi_config_sa_para(pkte_dev);
	adi_config_sa_key(pkte_dev, pkte_dev->Key);
	adi_config_idigest(pkte_dev, pkte_dev->Idigest);
	adi_config_odigest(pkte_dev, pkte_dev->Odigest);
	adi_config_state(pkte_dev, pkte_dev->IV);
	//Copy command descriptor into all command descriptor rings
	if (pkte_dev->flags & (PKTE_AUTONOMOUS_MODE | PKTE_TCM_MODE)) {
		for (i = 1; i < PKTE_RING_BUFFERS; i++) {
			memcpy(&pkte->pPkteDescriptor.SARecord[i],
			       &pkte->pPkteDescriptor.SARecord[0],
			       sizeof(struct SA));

			memcpy(&pkte->pPkteDescriptor.CmdDescriptor[i],
			       &pkte->pPkteDescriptor.CmdDescriptor[0],
			       sizeof(struct PE_CDR));
		}
	}

	if (pkte_dev->flags & PKTE_HOST_MODE)
		adi_write(pkte_dev, SA_RDY_OFFSET, 1);

	//Reset and release the packet engine
	adi_reset_pkte(pkte_dev);
	pkte_cfg = adi_read(pkte_dev, CFG_OFFSET);
	if (pkte_dev->flags & PKTE_HOST_MODE) {
		dev_dbg(pkte_dev->dev, "%s PKTE_HOST_MODE selected\n",
			__func__);
		pkte_cfg |= BITM_PKTE_HOST_MODE << 8;
		adi_write(pkte_dev, CLK_CTL_OFFSET, 1);
	} else if (pkte_dev->flags & PKTE_TCM_MODE) {
		dev_dbg(pkte_dev->dev, "%s PKTE_TCM_MODE selected\n", __func__);
		pkte_cfg |= BITM_PKTE_TCM_MODE << 8;
	} else if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE) {
		dev_dbg(pkte_dev->dev, "%s PKTE_AUTONOMOUS_MODE selected\n",
			__func__);
		adi_init_ring(pkte_dev);
		pkte_cfg |= 1 << 10;
		pkte_cfg |= BITM_PKTE_AUTONOMOUS_MODE << 8;
	}

	adi_write(pkte_dev, CFG_OFFSET, pkte_cfg);
	adi_init_sa(pkte_dev);
	adi_init_state(pkte_dev);
	adi_init_spe(pkte_dev);
}

int adi_hw_init(struct adi_dev *pkte_dev)
{
	pkte_dev->src_count_set = 0;
	pkte_dev->src_bytes_available = 0;
	ready = 0;
	processing = 0;
	//Continue with previous operation
	if (!(pkte_dev->flags & PKTE_FLAGS_STARTED))
		adi_start_engine(pkte_dev);

	return 0;
}

static int adi_unregister_algs(struct adi_dev *pkte_dev)
{
	struct adi_algs_info *adi_algs;
	struct ahash_engine_alg *alg;
	u32 i, j;

	if (!pkte_dev)
		return 0;

	adi_algs = pkte_dev->pdata->algs_info;
#ifdef CONFIG_CRYPTO_DEV_ADI_HASH
	for (i = 0; i < pkte_dev->pdata->algs_info_size; i++)
		for (j = 0; j < pkte_dev->pdata->algs_info[i].size; j++) {
			alg = adi_algs[i].algs_list;
			crypto_engine_unregister_ahash(&alg[j]);
		}

#endif

#ifdef CONFIG_CRYPTO_DEV_ADI_SKCIPHER
	crypto_unregister_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));
#endif
	return 0;
}

static int adi_register_algs(struct adi_dev *pkte_dev)
{
	struct adi_algs_info *adi_algs;
	struct ahash_engine_alg *alg;
	unsigned int i, j;
	int err;

	adi_algs = pkte_dev->pdata->algs_info;
#ifdef CONFIG_CRYPTO_DEV_ADI_HASH
	for (i = 0; i < pkte_dev->pdata->algs_info_size; i++) {
		for (j = 0; j < pkte_dev->pdata->algs_info[i].size; j++) {
			alg = adi_algs[i].algs_list;
			err = crypto_engine_register_ahash(&alg[j]);
			if (err) {
				dev_err(pkte_dev->dev,
					"Could not register hash alg\n");
				if (!i && !j)
					return err;

				goto err_algs;
			}
		}
	}
#endif

#ifdef CONFIG_CRYPTO_DEV_ADI_SKCIPHER
	err = crypto_register_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));
	if (err) {
		dev_err(pkte_dev->dev, "Could not register skcipher alg\n");
		goto err_algs;
	}
#endif

	return 0;

err_algs:

	adi_unregister_algs(pkte_dev);
	return err;
}

static const struct adi_pdata adi_pdata_adi = {
#ifdef CONFIG_CRYPTO_DEV_ADI_HASH
	.algs_info = adi_algs_info_adi,
	.algs_info_size = ARRAY_SIZE(adi_algs_info_adi),
#endif
};

static const struct of_device_id adi_of_match[] = {
	{
	 .compatible = "adi,pkte",
	 .data = &adi_pdata_adi,
	  },
	{ },
};

MODULE_DEVICE_TABLE(of, adi_of_match);

static int adi_get_of_match(struct adi_dev *pkte_dev, struct device *dev)
{
	pkte_dev->pdata = of_device_get_match_data(dev);
	if (!pkte_dev->pdata) {
		dev_dbg(dev, "no compatible OF match\n");
		return -EINVAL;
	}

	return 0;
}

static int adi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct adi_dev *pkte_dev;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret, irq;
	u32 pkte_int_cfg;
	const char *mode;

	pkte_dev = devm_kzalloc(dev, sizeof(*pkte_dev), GFP_KERNEL);
	if (!pkte_dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pkte_dev->io_base = devm_ioremap_resource(dev, res);

	if (IS_ERR(pkte_dev->io_base))
		return PTR_ERR(pkte_dev->io_base);

	pkte_dev->phys_base = res->start;
	ret = adi_get_of_match(pkte_dev, dev);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	pkte_dev->dev = dev;
	ret = of_property_read_string(np, "mode", &mode);
	if (ret < 0) {
		if (ret != -EINVAL)
			dev_err(pkte_dev->dev, "%s: could not parse mode\n",
				of_node_full_name(np));
		return -EINVAL;
	}

	if (strcmp(mode, "tcm") == 0) {
		pkte_dev->flags |= PKTE_TCM_MODE;
		dev_info(pkte_dev->dev, "Selected TCM mode\n");
	} else if (strcmp(mode, "dhm") == 0) {
		pkte_dev->flags |= PKTE_HOST_MODE;
		dev_info(pkte_dev->dev, "Selected DHM mode\n");
	} else {
		pkte_dev->flags |= PKTE_AUTONOMOUS_MODE;
		dev_info(pkte_dev->dev, "Selected ARM mode\n");
	}

#ifdef PKTE_USE_SRAM
	pkte_dev->pkte_device =
	    ioremap(PKTE_SRAM_ADDRESS, sizeof(struct ADI_PKTE_DEVICE));
	dev_dbg(pkte_dev->dev, "sram/ioremap %lx @ %p\n",
		sizeof(struct ADI_PKTE_DEVICE), pkte_dev->pkte_device);
#else
	pkte_dev->pkte_device = dma_alloc_coherent(pkte_dev->dev,
						   sizeof(struct
							  ADI_PKTE_DEVICE),
						   &pkte_dev->dma_handle,
						   GFP_KERNEL);
	dev_dbg(pkte_dev->dev, "dma_alloc_coherent %lx @ %p\n",
		sizeof(struct ADI_PKTE_DEVICE), pkte_dev->pkte_device);
#endif

	//Disable interrupts until we're using the PKTE
	pkte_int_cfg = adi_read(pkte_dev, INT_CFG_OFFSET);
	/* PKTE0 Interrupt Level Sensitive */
	adi_write(pkte_dev, INT_CFG_OFFSET,
		  pkte_int_cfg & ~BITM_PKTE_INT_CFG_TYPE);
	/* PKTE0 Interrupt Clear Register */
	adi_write(pkte_dev, INT_CLR_OFFSET, 0x00071E03);
	/* Disable All Interrupts */
	adi_write(pkte_dev, IMSK_DIS_OFFSET,
		  BITM_PKTE_IMSK_EN_IFERR |
		  BITM_PKTE_IMSK_EN_RDRTHRSH |
		  BITM_PKTE_IMSK_EN_RINGERR |
		  BITM_PKTE_IMSK_EN_PROCERR |
		  BITM_PKTE_IMSK_EN_HLT |
		  BITM_PKTE_IMSK_EN_CDRTHRSH |
		  BITM_PKTE_IMSK_EN_OPDN |
		  BITM_PKTE_IMSK_EN_IBUFTHRSH | BITM_PKTE_IMSK_EN_OBUFTHRSH);

	ret = devm_request_threaded_irq(dev, irq, NULL,
					adi_irq_handler,
					IRQF_SHARED, dev_name(dev), pkte_dev);
	if (ret) {
		dev_err(dev, "Cannot get IRQ\n");
		return ret;
	}

	platform_set_drvdata(pdev, pkte_dev);

	spin_lock(&adi.lock);
	list_add_tail(&pkte_dev->list, &adi.dev_list);
	spin_unlock(&adi.lock);

	/* Initialize crypto engine */
	pkte_dev->engine = crypto_engine_alloc_init(dev, 1);
	if (!pkte_dev->engine) {
		dev_err(dev, "Cannot Initialize crypto engine(%d)\n", ret);
		ret = -ENOMEM;
		goto err_engine;
	}

	/* start crypto engine */
	ret = crypto_engine_start(pkte_dev->engine);
	if (ret) {
		dev_err(dev, "Cannot start crypto engine(%d)\n", ret);
		goto err_engine_start;
	}

	/* Register algos */
	ret = adi_register_algs(pkte_dev);
	if (ret) {
		dev_err(dev, "Cannot register algorithms crypto engine(%d)\n",
			ret);
		goto err_algs;
	}

	memset(&pkte_dev->Key, 0, sizeof(u32) * 8);
	memset(&pkte_dev->Idigest, 0, sizeof(u32) * 8);
	memset(&pkte_dev->Odigest, 0, sizeof(u32) * 8);
	memset(&pkte_dev->IV, 0, sizeof(u32) * 4);
	return 0;

err_algs:
err_engine_start:
	crypto_engine_exit(pkte_dev->engine);
err_engine:
	spin_lock(&adi.lock);
	list_del(&pkte_dev->list);
	spin_unlock(&adi.lock);

	return ret;
}

static int adi_remove(struct platform_device *pdev)
{
	struct adi_dev *pkte_dev;

	pkte_dev = platform_get_drvdata(pdev);
	if (!pkte_dev)
		return -ENODEV;

	adi_unregister_algs(pkte_dev);

	crypto_engine_exit(pkte_dev->engine);

	spin_lock(&adi.lock);
	list_del(&pkte_dev->list);
	spin_unlock(&adi.lock);

#ifndef PKTE_USE_SRAM
	dma_free_coherent(pkte_dev->dev, sizeof(struct ADI_PKTE_DEVICE),
			  pkte_dev->pkte_device, pkte_dev->dma_handle);
#endif

	return 0;
}

static struct platform_driver adi_driver = {
	.probe = adi_probe,
	.remove = adi_remove,
	.driver = {
		   .name = "adi-pkte",
		   .of_match_table = adi_of_match,
		}
};

module_platform_driver(adi_driver);

MODULE_DESCRIPTION("ADI SHA1/224/256 & MD5 HW Acceleration");
MODULE_AUTHOR("Nathan Barrett-Morrison <nathan.morrison@timesys.com>");
MODULE_LICENSE("GPL v2");
