/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2024 Intel Corporation
 */

#ifndef _XE_SHRINKER_H_
#define _XE_SHRINKER_H_

struct xe_shrinker;
struct xe_device;

void xe_shrinker_mod_pages(struct xe_shrinker *shrinker, long shrinkable, long purgeable,
			   long tts);

int xe_shrinker_create(struct xe_device *xe);

int xe_shrinker_non_purgeable_seeks(struct xe_shrinker *shrinker);
int xe_shrinker_set_non_purgeable_seeks(struct xe_shrinker *shrinker,
					unsigned int seeks);

#endif
