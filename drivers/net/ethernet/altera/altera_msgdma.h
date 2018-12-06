/* SPDX-License-Identifier: GPL-2.0-only */
/* Altera TSE SGDMA and MSGDMA Linux driver
 * Copyright (C) 2014 Altera Corporation. All rights reserved
 */

#ifndef __ALTERA_MSGDMA_H__
#define __ALTERA_MSGDMA_H__

void msgdma_reset(struct altera_tse_private *priv);
void msgdma_enable_txirq(struct altera_tse_private *priv);
void msgdma_enable_rxirq(struct altera_tse_private *priv);
void msgdma_disable_rxirq(struct altera_tse_private *priv);
void msgdma_disable_txirq(struct altera_tse_private *priv);
void msgdma_clear_rxirq(struct altera_tse_private *priv);
void msgdma_clear_txirq(struct altera_tse_private *priv);
u32 msgdma_tx_completions(struct altera_tse_private *priv);
void msgdma_add_rx_desc(struct altera_tse_private *priv,
			struct tse_buffer *buffer);
int msgdma_tx_buffer(struct altera_tse_private *priv,
		     struct tse_buffer *buffer);
u32 msgdma_rx_status(struct altera_tse_private *priv);
int msgdma_initialize(struct altera_tse_private *priv);
void msgdma_uninitialize(struct altera_tse_private *priv);
void msgdma_start_rxdma(struct altera_tse_private *priv);

#endif /*  __ALTERA_MSGDMA_H__ */
