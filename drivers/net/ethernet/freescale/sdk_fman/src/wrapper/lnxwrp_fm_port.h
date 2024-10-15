// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/* Copyright 2024 NXP
 */
#ifndef __LNXWRP_FM_PORT_H__
#define __LNXWRP_FM_PORT_H__

struct device_node *GetFmPortAdvArgsDevTreeNode(struct device_node *fm_node,
						e_FmPortType portType,
						uint8_t portId);

#endif
