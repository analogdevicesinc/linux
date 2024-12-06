// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2023 NXP
 */

#ifndef _MAXIM_SERDES_H_
#define _MAXIM_SERDES_H_

int maxim_serdes_chain_register_remote(struct device *dev, int link, void (*cb)(void *),
				       void *data);
int maxim_serdes_chain_register_local(struct device *dev, int remote_links_used,
				      void (*cb)(void *), void *data);

#endif