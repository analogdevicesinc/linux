/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices AXI HSCI interface
 *
 * Copyright 2023 Analog Devices Inc.
 *
 * Wiki: https://wiki.analog.com/resources/fpga/docs/hsci
 */

#ifndef ADI_AXI_HSCI_H_
#define ADI_AXI_HSCI_H_

struct axi_hsci_state;

#ifdef CONFIG_ADI_AXI_HSCI

int axi_hsci_manual_linkup(struct axi_hsci_state *st, uint8_t enable,
			   uint16_t link_up_signal_bits);

int axi_hsci_auto_linkup(struct axi_hsci_state *st, uint8_t enable,
			 u8 hscim_mosi_clk_inv, uint8_t hscim_miso_clk_inv);

int axi_hsci_readm(struct axi_hsci_state *st, const uint8_t tx_data[],
		   u8 rx_data[], uint32_t num_tx_rx_bytes,
		   u8 addr_len,
		   u8 data_len,
		   uint32_t stream_len);

int axi_hsci_writem(struct axi_hsci_state *st, const uint8_t tx_data[],
		    u32 num_tx_rx_bytes,
		    u8 addr_len,
		    u8 data_len,
		    uint32_t stream_len);

struct axi_hsci_state *devm_axi_hsci_get_optional(struct device *source);

void axi_hsci_silent(struct axi_hsci_state *st, bool enable);

int axi_hsci_alink_tbl_get(struct axi_hsci_state *st, u16 *hscim_alink_table);

#else

static inline void axi_hsci_silent(struct axi_hsci_state *st, bool enable)
{
}

static inline int axi_hsci_alink_tbl_get(struct axi_hsci_state *st,
					 u16 *hscim_alink_table)
{
	return -ENODEV;
}

static inline int axi_hsci_manual_linkup(struct axi_hsci_state *st,
					 u8 enable,
					 uint16_t link_up_signal_bits)
{
	return -ENODEV;
}

static inline int axi_hsci_auto_linkup(struct axi_hsci_state *st,
				       u8 enable,
				       u8 hscim_mosi_clk_inv, uint8_t hscim_miso_clk_inv)
{
	return -ENODEV;
}

static inline int axi_hsci_readm(struct axi_hsci_state *st,
				 const u8 tx_data[],
				 u8 rx_data[], uint32_t num_tx_rx_bytes,
				 u8 addr_len,
				 u8 data_len,
				 uint32_t stream_len)
{
	return -ENODEV;
}

static inline int axi_hsci_writem(struct axi_hsci_state *st,
				  const u8 tx_data[],
				  u32 num_tx_rx_bytes,
				  u8 addr_len,
				  u8 data_len,
				  uint32_t stream_len)
{
	return -ENODEV;
}

static inline struct
axi_hsci_state *devm_axi_hsci_get_optional(struct device *source)
{
	return NULL;
}

#endif /* CONFIG_ADI_AXI_HSCI */

#endif /* ADI_AXI_HSCI_H_ */
