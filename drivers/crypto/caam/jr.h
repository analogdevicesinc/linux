/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CAAM public-level include definitions for the JobR backend
 *
 * Copyright 2008-2011 Freescale Semiconductor, Inc.
 * Copyright 2020 NXP
 */

#ifndef JR_H
#define JR_H

#include <linux/completion.h>

 /**
  * struct jr_job_result - Job Ring result structure, used for requests
  *                        that need to run and wait for their completion
  *
  * @error               : The result returned after request was executed
  * @completion          : Structure used to maintain state for a "completion"
  */
struct jr_job_result {
	int error;
	struct completion completion;
};

/* Prototypes for backend-level services exposed to APIs */
int caam_jr_driver_probed(void);
struct device *caam_jr_alloc(void);
struct device *caam_jridx_alloc(int idx);
void caam_jr_free(struct device *rdev);
int caam_jr_enqueue(struct device *dev, u32 *desc,
		    void (*cbk)(struct device *dev, u32 *desc, u32 status,
				void *areq),
		    void *areq);
int caam_jr_run_and_wait_for_completion(struct device *dev, u32 *desc,
					void (*cbk)(struct device *dev,
						    u32 *desc, u32 status,
						    void *areq));

#endif /* JR_H */
