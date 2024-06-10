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

#include <asm/cacheflush.h>

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

u32 Key[8] = {0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0};
u32 IV[4] = {0x0, 0x0, 0x0, 0x0};
u32 IDigest[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
u32 ODigest[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

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

u32 adi_physical_address(struct adi_dev *pkte_dev, void *variableAddress)
{

	u32 variableAddress_32 = *((u32 *)variableAddress);
	u32 pkte_device_32 = *((u32 *)pkte_dev->pkte_device);

#ifdef PKTE_USE_SRAM
	return PKTE_SRAM_ADDRESS + variableAddress_32 - pkte_device_32;
#else
	return pkte_dev->dma_handle + variableAddress_32 - pkte_device_32;
#endif
}

static void adi_reset_pkte(struct adi_dev *pkte_dev)
{
	u32 temp;

	//Reset the packet engine
	temp = adi_read(pkte_dev, CFG_OFFSET);
	adi_write(pkte_dev, CFG_OFFSET, temp | 0x03);
	adi_write(pkte_dev, CFG_OFFSET, temp & ~0x03);
}

void adi_reset_state(struct adi_dev *hdev)
{
	hdev->src_count_set = 0;
	hdev->src_bytes_available = 0;
	hdev->ring_pos_produce = 0;
	hdev->ring_pos_consume = 0;
	hdev->flags &= ~(PKTE_FLAGS_STARTED|
			 PKTE_FLAGS_COMPLETE|
			 PKTE_FLAGS_FINAL|
			 PKTE_FLAGS_FINUP|
			 PKTE_FLAGS_HMAC|
			 PKTE_FLAGS_HMAC_KEY_PREPARED);
}

static irqreturn_t adi_irq_handler(int irq, void *dev_id)
{
	struct adi_dev *pkte_dev = dev_id;
	u32 temp = adi_read(pkte_dev, IMSK_STAT_OFFSET);
	u32 temp2;

	dev_dbg(pkte_dev->dev, "%s %x\n", __func__, temp);

	if (temp & BITM_PKTE_IMSK_EN_RINGERR) {
		dev_dbg(pkte_dev->dev, "%s: RINGERR, check error codes %x", __func__,
			adi_read(pkte_dev, RING_STAT_OFFSET));
		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_RINGERR);
	}

	if (temp & BITM_PKTE_IMSK_EN_PROCERR) {
		dev_dbg(pkte_dev->dev, "%s: PROCERR, check error codes %x", __func__,
			adi_read(pkte_dev, CTL_STAT_OFFSET));
		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_PROCERR);
	}

	if (temp & BITM_PKTE_IMSK_EN_RDRTHRSH) {
		if (pkte_dev->flags & PKTE_TCM_MODE)
			adi_write(pkte_dev, RDSC_DECR_OFFSET, 0x1);

		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
		temp2 = adi_read(pkte_dev, IMSK_DIS_OFFSET);

		adi_write(pkte_dev, IMSK_DIS_OFFSET, temp2 | BITM_PKTE_IMSK_EN_RDRTHRSH);


		if (pkte_dev->pkte_device->pPkteList.pCommand.final_hash_condition == final_hash) {
			ready = 1;
			pkte_dev->flags |= PKTE_FLAGS_COMPLETE;
			wake_up_interruptible(&wq_ready);
		}

		if (pkte_dev->flags & PKTE_TCM_MODE) {
			processing = 0;
			wake_up_interruptible(&wq_processing);
		}
	}

	if (temp & BITM_PKTE_IMSK_EN_OPDN) {
		adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_OPDN);
		temp2 = adi_read(pkte_dev, IMSK_DIS_OFFSET);
		adi_write(pkte_dev, IMSK_DIS_OFFSET,
			  temp2 | BITM_PKTE_IMSK_EN_OPDN | BITM_PKTE_IMSK_EN_IBUFTHRSH);
		adi_write(pkte_dev, RDSC_DECR_OFFSET, 0x1);

		if (pkte_dev->pkte_device->pPkteList.pCommand.final_hash_condition == final_hash) {
			ready = 1;
			pkte_dev->flags |= PKTE_FLAGS_COMPLETE;
			wake_up_interruptible(&wq_ready);
		}
		processing = 0;
		wake_up_interruptible(&wq_processing);
	}

	if (temp & BITM_PKTE_IMSK_EN_IBUFTHRSH) {
		if (pkte_dev->src_bytes_available) {
#ifdef CONFIG_CRYPTO_DEV_ADI_HASH
			adi_write_packet(pkte_dev, &pkte_dev->pkte_device->pPkteList.pSource[0]);
#endif
			adi_write(pkte_dev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_IBUFTHRSH);
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
	struct PE_CDR *commandDesc;

	pos = pkte_dev->flags & PKTE_AUTONOMOUS_MODE ? pkte_dev->ring_pos_consume : 0;
	commandDesc = &pkte_dev->pkte_device->pPkteDescriptor.CmdDescriptor[pos];

	adi_write(pkte_dev, CTL_STAT_OFFSET, commandDesc->PE_CTRL_STAT);
	adi_write(pkte_dev, SRC_ADDR_OFFSET, commandDesc->PE_SOURCE_ADDR);
	adi_write(pkte_dev, DEST_ADDR_OFFSET, commandDesc->PE_DEST_ADDR);
	adi_write(pkte_dev, SA_ADDR_OFFSET, commandDesc->PE_SA_ADDR);
	adi_write(pkte_dev, STATE_ADDR_OFFSET, commandDesc->PE_STATE_ADDR);
	adi_write(pkte_dev, USERID_OFFSET, commandDesc->PE_USER_ID);
}

static void adi_init_sa(struct adi_dev *pkte_dev)
{
	struct SA *SARec;

	SARec = &pkte_dev->pkte_device->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume];

	adi_write(pkte_dev, SA_CMD_OFFSET(0), SARec->SA_Para.SA_CMD0);
	adi_write(pkte_dev, SA_CMD_OFFSET(1), SARec->SA_Para.SA_CMD1);
	adi_write(pkte_dev, SA_KEY_OFFSET(0), SARec->SA_Key.SA_KEY0);
	adi_write(pkte_dev, SA_KEY_OFFSET(1), SARec->SA_Key.SA_KEY1);
	adi_write(pkte_dev, SA_KEY_OFFSET(2), SARec->SA_Key.SA_KEY2);
	adi_write(pkte_dev, SA_KEY_OFFSET(3), SARec->SA_Key.SA_KEY3);
	adi_write(pkte_dev, SA_KEY_OFFSET(4), SARec->SA_Key.SA_KEY4);
	adi_write(pkte_dev, SA_KEY_OFFSET(5), SARec->SA_Key.SA_KEY5);
	adi_write(pkte_dev, SA_KEY_OFFSET(6), SARec->SA_Key.SA_KEY6);
	adi_write(pkte_dev, SA_KEY_OFFSET(7), SARec->SA_Key.SA_KEY7);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(0), SARec->SA_Idigest.SA_IDIGEST0);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(1), SARec->SA_Idigest.SA_IDIGEST1);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(2), SARec->SA_Idigest.SA_IDIGEST2);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(3), SARec->SA_Idigest.SA_IDIGEST3);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(4), SARec->SA_Idigest.SA_IDIGEST4);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(5), SARec->SA_Idigest.SA_IDIGEST5);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(6), SARec->SA_Idigest.SA_IDIGEST6);
	adi_write(pkte_dev, SA_IDIGEST_OFFSET(7), SARec->SA_Idigest.SA_IDIGEST7);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(0), SARec->SA_Odigest.SA_ODIGEST0);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(1), SARec->SA_Odigest.SA_ODIGEST1);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(2), SARec->SA_Odigest.SA_ODIGEST2);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(3), SARec->SA_Odigest.SA_ODIGEST3);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(4), SARec->SA_Odigest.SA_ODIGEST4);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(5), SARec->SA_Odigest.SA_ODIGEST5);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(6), SARec->SA_Odigest.SA_ODIGEST6);
	adi_write(pkte_dev, SA_ODIGEST_OFFSET(7), SARec->SA_Odigest.SA_ODIGEST7);
}

void adi_init_ring(struct adi_dev *pkte_dev)
{
	u32 temp;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;

	temp = adi_physical_address(pkte_dev, (void *)&pkte->pPkteDescriptor.CmdDescriptor[0]);
	adi_write(pkte_dev, CDRBASE_ADDR_OFFSET, temp);
	temp = adi_physical_address(pkte_dev, (void *)&pkte->pPkteDescriptor.ResultDescriptor[0]);
	adi_write(pkte_dev, RDRBASE_ADDR_OFFSET, temp);
	temp = (PKTE_RING_BUFFERS-1)<<BITP_PKTE_RING_CFG_RINGSZ;
	adi_write(pkte_dev, RING_CFG_OFFSET, temp);
	temp = 0<<BITP_PKTE_RING_THRESH_CDRTHRSH|0<<BITP_PKTE_RING_THRESH_RDRTHRSH;
	adi_write(pkte_dev, RING_THRESH_OFFSET, temp);
}

void adi_source_data(struct adi_dev *pkte_dev, u32 size)
{
	u8 pos;
	struct PE_CDR *commandDesc;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	pos = pkte_dev->flags & PKTE_AUTONOMOUS_MODE ? pkte_dev->ring_pos_consume : 0;
	commandDesc = &pkte->pPkteDescriptor.CmdDescriptor[pos];

	if ((!pkte_dev->src_count_set) ||
	    (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE))) {
		pkte_dev->src_count_set = 1;
		pkte->pPkteList.nSrcSize = size;
		commandDesc->PE_LENGTH = (u32)BITM_PKTE_LEN_HSTRDY|pkte->pPkteList.nSrcSize;
		if (!(pkte_dev->flags & PKTE_AUTONOMOUS_MODE))
			adi_write(pkte_dev, LEN_OFFSET, commandDesc->PE_LENGTH);
		dev_dbg(pkte_dev->dev, "%s: LEN set to %x\n", __func__, commandDesc->PE_LENGTH);
	}
}

void adi_configure_cdr(struct adi_dev *pkte_dev)
{
	u8 pos;
	struct PE_CDR *commandDesc;
	struct SA *SARec;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	SARec = &pkte->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume];
	pos = pkte_dev->flags & PKTE_AUTONOMOUS_MODE ? pkte_dev->ring_pos_consume : 0;
	commandDesc = &pkte->pPkteDescriptor.CmdDescriptor[pos];

	commandDesc->PE_CTRL_STAT = (u32)0x1 | (pkte->pPkteList.pCommand.final_hash_condition<<4);
	commandDesc->PE_SOURCE_ADDR = adi_physical_address(pkte_dev,
		(void *)&pkte->source[pkte_dev->ring_pos_consume][0]);
	commandDesc->PE_DEST_ADDR = adi_physical_address(pkte_dev, (void *)&pkte->destination[0]);
	commandDesc->PE_SA_ADDR = adi_physical_address(pkte_dev, (void *)&SARec->SA_Para.SA_CMD0);
	commandDesc->PE_STATE_ADDR = adi_physical_address(pkte_dev,
		(void *)&pkte->pPkteDescriptor.State.STATE_IV0);
	commandDesc->PE_USER_ID = pkte->pPkteList.nUserID;
}

void adi_config_sa_para(struct adi_dev *pkte_dev)
{
	struct SA *SARec;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;

	SARec = &pkte->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume];

	SARec->SA_Para.SA_CMD0 =
		(pkte->pPkteList.pCommand.opcode<<BITP_PKTE_SA_CMD0_OPCD)|
		(pkte->pPkteList.pCommand.direction<<BITP_PKTE_SA_CMD0_DIR)|
		((u32)0x0 << BITP_PKTE_SA_CMD0_OPGRP)|
		((u32)0x3 << BITP_PKTE_SA_CMD0_PADTYPE)|
		(pkte->pPkteList.pCommand.cipher<<BITP_PKTE_SA_CMD0_CIPHER)|
		(pkte->pPkteList.pCommand.hash << BITP_PKTE_SA_CMD0_HASH)|
		(u32)0x1<<BITP_PKTE_SA_CMD0_SCPAD|
		(pkte->pPkteList.pCommand.digest_length<<BITP_PKTE_SA_CMD0_DIGESTLEN)|
		((u32)0x2 << BITP_PKTE_SA_CMD0_IVSRC)|
		(pkte->pPkteList.pCommand.hash_source << BITP_PKTE_SA_CMD0_HASHSRC)|
		(u32)0x0<<BITP_PKTE_SA_CMD0_SVIV|
		(u32)0x1<<BITP_PKTE_SA_CMD0_SVHASH;

	SARec->SA_Para.SA_CMD1 =
		(u32)0x1<<BITP_PKTE_SA_CMD1_CPYDGST|
		(pkte->pPkteList.pCommand.hash_mode<<BITP_PKTE_SA_CMD1_HMAC)|
		(pkte->pPkteList.pCommand.cipher_mode<<BITP_PKTE_SA_CMD1_CIPHERMD)|
		(pkte->pPkteList.pCommand.aes_key_length<<BITP_PKTE_SA_CMD1_AESKEYLEN)|
		(pkte->pPkteList.pCommand.aes_des_key<<BITP_PKTE_SA_CMD1_AESDECKEY);
}

void adi_config_sa_key(struct adi_dev *pkte_dev, u32 Key[])
{
	struct SA *SARec;

	SARec = &pkte_dev->pkte_device->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume];

	SARec->SA_Key.SA_KEY0 = Key[0];
	SARec->SA_Key.SA_KEY1 = Key[1];
	SARec->SA_Key.SA_KEY2 = Key[2];
	SARec->SA_Key.SA_KEY3 = Key[3];
	SARec->SA_Key.SA_KEY4 = Key[4];
	SARec->SA_Key.SA_KEY5 = Key[5];
	SARec->SA_Key.SA_KEY6 = Key[6];
	SARec->SA_Key.SA_KEY7 = Key[7];
}

static void adi_config_idigest(struct adi_dev *pkte_dev, u32 Idigest[])
{
	struct SA *SARec;

	SARec = &pkte_dev->pkte_device->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume];

	SARec->SA_Idigest.SA_IDIGEST0 = Idigest[0];
	SARec->SA_Idigest.SA_IDIGEST1 = Idigest[1];
	SARec->SA_Idigest.SA_IDIGEST2 = Idigest[2];
	SARec->SA_Idigest.SA_IDIGEST3 = Idigest[3];
	SARec->SA_Idigest.SA_IDIGEST4 = Idigest[4];
	SARec->SA_Idigest.SA_IDIGEST5 = Idigest[5];
	SARec->SA_Idigest.SA_IDIGEST6 = Idigest[6];
	SARec->SA_Idigest.SA_IDIGEST7 = Idigest[7];
}

static void adi_config_odigest(struct adi_dev *pkte_dev, u32 Odigest[])
{
	struct SA *SARec;

	SARec = &pkte_dev->pkte_device->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume];

	SARec->SA_Odigest.SA_ODIGEST0 = Odigest[0];
	SARec->SA_Odigest.SA_ODIGEST1 = Odigest[1];
	SARec->SA_Odigest.SA_ODIGEST2 = Odigest[2];
	SARec->SA_Odigest.SA_ODIGEST3 = Odigest[3];
	SARec->SA_Odigest.SA_ODIGEST4 = Odigest[4];
	SARec->SA_Odigest.SA_ODIGEST5 = Odigest[5];
	SARec->SA_Odigest.SA_ODIGEST6 = Odigest[6];
	SARec->SA_Odigest.SA_ODIGEST7 = Odigest[7];
}

void adi_config_state(struct adi_dev *pkte_dev, u32 IV[])
{
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;

#ifdef DEBUG_PKTE
	dev_dbg(pkte_dev->dev, "%s IV: %x %x %x %x\n", __func__, IV[0], IV[1], IV[2], IV[3]);
#endif

	pkte->pPkteDescriptor.State.STATE_IV0 = IV[0];
	pkte->pPkteDescriptor.State.STATE_IV1 = IV[1];
	pkte->pPkteDescriptor.State.STATE_IV2 = IV[2];
	pkte->pPkteDescriptor.State.STATE_IV3 = IV[3];
	pkte->pPkteDescriptor.State.STATE_BYTE_CNT0 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_BYTE_CNT1 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST0 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST1 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST2 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST3 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST4 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST5 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST6 = (u32)0x0;
	pkte->pPkteDescriptor.State.STATE_IDIGEST7 = (u32)0x0;
}

void adi_start_engine(struct adi_dev *pkte_dev)
{
	u32 temp;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;

	//Set up configuration structures
	adi_configure_cdr(pkte_dev);
	adi_config_sa_para(pkte_dev);
	adi_config_sa_key(pkte_dev, Key);
	adi_config_idigest(pkte_dev, IDigest);
	adi_config_odigest(pkte_dev, ODigest);
	adi_config_state(pkte_dev, IV);

	//Copy command descriptor into all command descriptor rings
	if (pkte_dev->flags & (PKTE_AUTONOMOUS_MODE | PKTE_TCM_MODE)) {
		for (temp = 1; temp < PKTE_RING_BUFFERS; temp++) {
			memcpy(&pkte->pPkteDescriptor.SARecord[temp],
				&pkte->pPkteDescriptor.SARecord[0],
				sizeof(struct SA));
			memcpy(&pkte->pPkteDescriptor.CmdDescriptor[temp],
				&pkte->pPkteDescriptor.CmdDescriptor[0],
				sizeof(struct PE_CDR));
		}
	}

	if (pkte_dev->flags & PKTE_HOST_MODE)
		adi_write(pkte_dev, SA_RDY_OFFSET, 1);

	//Reset and release the packet engine
	adi_reset_pkte(pkte_dev);

	temp = adi_read(pkte_dev, CFG_OFFSET);
	if (pkte_dev->flags & PKTE_HOST_MODE) {
		dev_dbg(pkte_dev->dev, "%s PKTE_HOST_MODE selected\n", __func__);
		temp |= BITM_PKTE_HOST_MODE<<8;
		adi_write(pkte_dev, CLK_CTL_OFFSET, 1);
	} else if (pkte_dev->flags & PKTE_TCM_MODE) {
		dev_dbg(pkte_dev->dev, "%s PKTE_TCM_MODE selected\n", __func__);
		temp |= BITM_PKTE_TCM_MODE<<8;
	} else if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE) {
		dev_dbg(pkte_dev->dev, "%s PKTE_AUTONOMOUS_MODE selected\n", __func__);
		adi_init_ring(pkte_dev);
		temp |= (u32)1<<10;
		temp |= BITM_PKTE_AUTONOMOUS_MODE<<8;
	}
	adi_write(pkte_dev, CFG_OFFSET, temp);

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
	unsigned int i, j;


	if(!pkte_dev)
		return 0;

#ifdef CONFIG_CRYPTO_DEV_ADI_HASH
	for (i = 0; i < pkte_dev->pdata->algs_info_size; i++)
		for (j = 0; j < pkte_dev->pdata->algs_info[i].size; j++)
			crypto_engine_unregister_ahash(
				&pkte_dev->pdata->algs_info[i].algs_list[j]);

#endif

#ifdef CONFIG_CRYPTO_DEV_ADI_SKCIPHER
	crypto_unregister_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));
#endif
	return 0;
}

static int adi_register_algs(struct adi_dev *pkte_dev)
{
	unsigned int i, j;
	int err;

#ifdef CONFIG_CRYPTO_DEV_ADI_HASH
	for (i = 0; i < pkte_dev->pdata->algs_info_size; i++) {
		for (j = 0; j < pkte_dev->pdata->algs_info[i].size; j++) {
			err = crypto_engine_register_ahash(
				&pkte_dev->pdata->algs_info[i].algs_list[j]);
			if (err) {
				dev_err(pkte_dev->dev, "Could not register hash alg\n");
				if ((i == 0) && (j==0))
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
	.algs_info	= adi_algs_info_adi,
	.algs_info_size	= ARRAY_SIZE(adi_algs_info_adi),
#endif
};

static const struct of_device_id adi_of_match[] = {
	{
		.compatible = "adi,pkte",
		.data = &adi_pdata_adi,
	},
	{},
};

MODULE_DEVICE_TABLE(of, adi_of_match);

static int adi_get_of_match(struct adi_dev *pkte_dev,
				   struct device *dev)
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
	u32 temp;
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
		pkte_dev->pkte_device = ioremap(PKTE_SRAM_ADDRESS, sizeof(struct ADI_PKTE_DEVICE));
		dev_dbg(pkte_dev->dev, "sram/ioremap %lx @ %p\n",
			sizeof(struct ADI_PKTE_DEVICE), pkte_dev->pkte_device);
	#else
		pkte_dev->pkte_device = dma_alloc_coherent(pkte_dev->dev,
					sizeof(struct ADI_PKTE_DEVICE),
					&pkte_dev->dma_handle, GFP_KERNEL);
		dev_dbg(pkte_dev->dev, "dma_alloc_coherent %lx @ %p\n",
			sizeof(struct ADI_PKTE_DEVICE), pkte_dev->pkte_device);
	#endif

	//Disable interrupts until we're using the PKTE
	temp = adi_read(pkte_dev, INT_CFG_OFFSET);

	 /* PKTE0 Interrupt Level Sensitive */
	adi_write(pkte_dev, INT_CFG_OFFSET, temp & ~BITM_PKTE_INT_CFG_TYPE);

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
		  BITM_PKTE_IMSK_EN_IBUFTHRSH |
		  BITM_PKTE_IMSK_EN_OBUFTHRSH);

	ret = devm_request_irq(dev, irq, adi_irq_handler,
					IRQF_SHARED,
					dev_name(dev), pkte_dev);
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
	.probe		= adi_probe,
	.remove		= adi_remove,
	.driver		= {
		.name	= "adi-pkte",
		.of_match_table	= adi_of_match,
	}
};

module_platform_driver(adi_driver);

MODULE_DESCRIPTION("ADI SHA1/224/256 & MD5 HW Acceleration");
MODULE_AUTHOR("Nathan Barrett-Morrison <nathan.morrison@timesys.com>");
MODULE_LICENSE("GPL v2");

