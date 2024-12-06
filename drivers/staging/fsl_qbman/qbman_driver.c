/* Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/time.h>
#include "qman_private.h"
#include "bman_private.h"
__init int qman_init_early(void);
__init int bman_init_early(void);

static __init int qbman_init(void)
{
	struct device_node *dn;
	u32 is_portal_available;

	bman_init();
	qman_init();

	is_portal_available = 0;
	for_each_compatible_node(dn, NULL, "fsl,qman-portal") {
		if (!of_device_is_available(dn))
			continue;
		else
			is_portal_available = 1;
	}

	if (!qman_have_ccsr() && is_portal_available) {
		struct qman_fq fq = {
				.fqid = 1
		};
		struct qm_mcr_queryfq_np np;
		int err, retry = CONFIG_FSL_QMAN_INIT_TIMEOUT;
		struct timespec64 nowts, diffts, startts;

		ktime_get_coarse_real_ts64(&startts);

		/* Loop while querying given fqid succeeds or time out */
		while (1) {
			err = qman_query_fq_np(&fq, &np);
			if (!err) {
				/* success, control-plane has configured QMan */
				break;
			} else if (err != -ERANGE) {
				pr_err("QMan: I/O error, continuing anyway\n");
				break;
			}
			ktime_get_coarse_real_ts64(&nowts);
			diffts = timespec64_sub(nowts, startts);
			if (diffts.tv_sec > 0) {
				if (!retry--) {
					pr_err("QMan: time out, control-plane"
								" dead?\n");
					break;
				}
				pr_warn("QMan: polling for the control-plane"
							" (%d)\n", retry);
			}
		}
	}
	bman_resource_init();
	qman_resource_init();
	return 0;
}
subsys_initcall(qbman_init);
