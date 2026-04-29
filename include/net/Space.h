/* SPDX-License-Identifier: GPL-2.0 */
/* A unified ethernet device probe.  This is the easiest way to have every
 * ethernet adaptor have the name "eth[0123...]".
 */

struct net_device *ne_probe(int unit);
