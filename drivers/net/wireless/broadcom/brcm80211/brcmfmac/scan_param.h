/* SPDX-License-Identifier: ISC */
/*
 * Copyright (c) 2023 Daniel Berlin
 */

#ifndef _BRCMF_SCAN_PARAM_H
#define _BRCMF_SCAN_PARAM_H

struct brcmf_pub;

int brcmf_scan_param_setup_for_version(struct brcmf_pub *drvr, u32 version);

#endif /* _BRCMF_SCAN_PARAM_H */
