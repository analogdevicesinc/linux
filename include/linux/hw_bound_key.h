/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2022 NXP
 * Author: Pankaj Gupta <pankaj.gupta@nxp.com>
 */

#ifndef _HW_BOUND_KEY_H
#define _HW_BOUND_KEY_H

#include "types.h"

struct hw_bound_key_info {
	/* Key types specific to the hw. [Implementation Defined]
	 */
	uint8_t flags;
	uint8_t reserved;
	/* Plain key size.
	 */
	uint16_t key_sz;
};

#define set_hbk_info(hbk_info, hw_flags, key_len) do {\
	hbk_info->flags = hw_flags;\
	hbk_info->key_sz = key_len;\
} while (0)

#endif /* _HW_BOUND_KEY_H */
