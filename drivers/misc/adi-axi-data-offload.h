/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices Data Offload Engine Driver
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * Wiki: https://wiki.analog.com/resources/fpga/docs/data_offload
 */

#ifndef ADI_AXI_DATA_OFFLOAD_H_
#define ADI_AXI_DATA_OFFLOAD_H_

struct axi_data_offload_state;

int axi_data_offload_ctrl_bypass(struct axi_data_offload_state *st, bool en);
int axi_data_offload_ctrl_oneshot(struct axi_data_offload_state *st, bool en);

struct axi_data_offload_state *devm_axi_data_offload_get_optional(struct device *source);

#endif /* ADI_AXI_DATA_OFFLOAD_H_ */
