/* Copyright 2008-2013 Freescale Semiconductor, Inc.
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

#ifndef __DPAA_ETH_COMMON_H
#define __DPAA_ETH_COMMON_H

#include <linux/etherdevice.h> /* struct net_device */
#include <linux/fsl_bman.h> /* struct bm_buffer */
#include <linux/of_platform.h> /* struct platform_device */
#include <linux/net_tstamp.h>	/* struct hwtstamp_config */

#include "dpaa_eth.h"
#include "lnxwrp_fsl_fman.h"

#define dpaa_eth_init_port(type, port, param, errq_id, defq_id, buf_layout,\
			   frag_enabled) \
{ \
	param.errq = errq_id; \
	param.defq = defq_id; \
	param.priv_data_size = buf_layout->priv_data_size; \
	param.parse_results = buf_layout->parse_results; \
	param.hash_results = buf_layout->hash_results; \
	param.frag_enable = frag_enabled; \
	param.time_stamp = buf_layout->time_stamp; \
	param.manip_extra_space = buf_layout->manip_extra_space; \
	param.data_align = buf_layout->data_align; \
	fm_set_##type##_port_params(port, &param); \
}

/* The SGT needs to be 256 bytes long. Even if the table has only one entry,
 * the FMan will read 256 bytes from its start.
 */
#define DPA_SGT_SIZE 256
#define DPA_SGT_MAX_ENTRIES 16 /* maximum number of entries in SG Table */

#define DPA_BUFF_RELEASE_MAX 8 /* maximum number of buffers released at once */

#define DPA_RX_PCD_HI_PRIO_FQ_INIT_FAIL(dpa_fq, _errno) \
	(((dpa_fq)->fq_type == FQ_TYPE_RX_PCD_HI_PRIO) && \
	  (_errno == -EIO))
/* return codes for the dpaa-eth hooks */
enum dpaa_eth_hook_result {
	/* fd/skb was retained by the hook.
	 *
	 * On the Rx path, this means the Ethernet driver will _not_
	 * deliver the skb to the stack. Instead, the hook implementation
	 * is expected to properly dispose of the skb.
	 *
	 * On the Tx path, the Ethernet driver's dpa_tx() function will
	 * immediately return NETDEV_TX_OK. The hook implementation is expected
	 * to free the skb. *DO*NOT* release it to BMan, or enqueue it to FMan,
	 * unless you know exactly what you're doing!
	 *
	 * On the confirmation/error paths, the Ethernet driver will _not_
	 * perform any fd cleanup, nor update the interface statistics.
	 */
	DPAA_ETH_STOLEN,
	/* fd/skb was returned to the Ethernet driver for regular processing.
	 * The hook is not allowed to, for instance, reallocate the skb (as if
	 * by linearizing, copying, cloning or reallocating the headroom).
	 */
	DPAA_ETH_CONTINUE
};

typedef enum dpaa_eth_hook_result (*dpaa_eth_ingress_hook_t)(
		struct sk_buff *skb, struct net_device *net_dev, u32 fqid);
typedef enum dpaa_eth_hook_result (*dpaa_eth_egress_hook_t)(
		struct sk_buff *skb, struct net_device *net_dev);
typedef enum dpaa_eth_hook_result (*dpaa_eth_confirm_hook_t)(
		struct net_device *net_dev, const struct qm_fd *fd, u32 fqid);

/* used in napi related functions */
extern u16 qman_portal_max;

/* from dpa_ethtool.c */
extern const struct ethtool_ops dpa_ethtool_ops;

#ifdef CONFIG_FSL_DPAA_HOOKS
/* Various hooks used for unit-testing and/or fastpath optimizations.
 * Currently only one set of such hooks is supported.
 */
struct dpaa_eth_hooks_s {
	/* Invoked on the Tx private path, immediately after receiving the skb
	 * from the stack.
	 */
	dpaa_eth_egress_hook_t	tx;

	/* Invoked on the Rx private path, right before passing the skb
	 * up the stack. At that point, the packet's protocol id has already
	 * been set. The skb's data pointer is now at the L3 header, and
	 * skb->mac_header points to the L2 header. skb->len has been adjusted
	 * to be the length of L3+payload (i.e., the length of the
	 * original frame minus the L2 header len).
	 * For more details on what the skb looks like, see eth_type_trans().
	 */
	dpaa_eth_ingress_hook_t	rx_default;

	/* Driver hook for the Rx error private path. */
	dpaa_eth_confirm_hook_t rx_error;
	/* Driver hook for the Tx confirmation private path. */
	dpaa_eth_confirm_hook_t tx_confirm;
	/* Driver hook for the Tx error private path. */
	dpaa_eth_confirm_hook_t tx_error;
};

void fsl_dpaa_eth_set_hooks(struct dpaa_eth_hooks_s *hooks);

extern struct dpaa_eth_hooks_s dpaa_eth_hooks;
#endif

int dpa_netdev_init(struct net_device *net_dev,
		    const uint8_t *mac_addr,
		    uint16_t tx_timeout);
int __cold dpa_start(struct net_device *net_dev);
int __cold dpa_stop(struct net_device *net_dev);
void __cold dpa_timeout(struct net_device *net_dev);
void __cold
dpa_get_stats64(struct net_device *net_dev,
		struct rtnl_link_stats64 *stats);
int dpa_ndo_init(struct net_device *net_dev);
int dpa_set_features(struct net_device *dev, netdev_features_t features);
netdev_features_t dpa_fix_features(struct net_device *dev,
		netdev_features_t features);
#ifdef CONFIG_FSL_DPAA_TS
u64 dpa_get_timestamp_ns(const struct dpa_priv_s *priv,
			enum port_type rx_tx, const void *data);
/* Updates the skb shared hw timestamp from the hardware timestamp */
int dpa_get_ts(const struct dpa_priv_s *priv, enum port_type rx_tx,
	struct skb_shared_hwtstamps *shhwtstamps, const void *data);
#endif /* CONFIG_FSL_DPAA_TS */
int dpa_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
int __cold dpa_remove(struct platform_device *of_dev);
struct mac_device * __cold __must_check
__attribute__((nonnull)) dpa_mac_probe(struct platform_device *_of_dev);
int dpa_set_mac_address(struct net_device *net_dev, void *addr);
void dpa_set_rx_mode(struct net_device *net_dev);
void dpa_set_buffers_layout(struct mac_device *mac_dev,
		struct dpa_buffer_layout_s *layout);
int __attribute__((nonnull))
dpa_bp_alloc(struct dpa_bp *dpa_bp, struct device *dev);
void __cold __attribute__((nonnull))
dpa_bp_free(struct dpa_priv_s *priv);
struct dpa_bp *dpa_bpid2pool(int bpid);
void dpa_bpid2pool_map(int bpid, struct dpa_bp *dpa_bp);
bool dpa_bpid2pool_use(int bpid);
void dpa_bp_drain(struct dpa_bp *bp);
#ifdef CONFIG_FMAN_PFC
u16 dpa_select_queue(struct net_device *net_dev, struct sk_buff *skb,
		     struct net_device *sb_dev,
		     select_queue_fallback_t fallback);
#endif
struct dpa_fq *dpa_fq_alloc(struct device *dev,
			    u32 fq_start,
			    u32 fq_count,
			    struct list_head *list,
			    enum dpa_fq_type fq_type);
int dpa_fq_probe_mac(struct device *dev, struct list_head *list,
		     struct fm_port_fqs *port_fqs,
		     bool tx_conf_fqs_per_core,
		     enum port_type ptype);
int dpa_get_channel(void);
void dpa_release_channel(void);
void dpaa_eth_add_channel(u16 channel);
int dpaa_eth_cgr_init(struct dpa_priv_s *priv);
void dpa_fq_setup(struct dpa_priv_s *priv, const struct dpa_fq_cbs_t *fq_cbs,
		struct fm_port *tx_port);
int dpa_fq_init(struct dpa_fq *dpa_fq, bool td_enable);
int dpa_fqs_init(struct device *dev, struct list_head *list, bool td_enable);
int __cold __attribute__((nonnull))
dpa_fq_free(struct device *dev, struct list_head *list);
void dpaa_eth_init_ports(struct mac_device *mac_dev,
		struct dpa_bp *bp, size_t count,
		struct fm_port_fqs *port_fqs,
		struct dpa_buffer_layout_s *buf_layout,
		struct device *dev);
void dpa_release_sgt(struct qm_sg_entry *sgt);
void __attribute__((nonnull))
dpa_fd_release(const struct net_device *net_dev, const struct qm_fd *fd);
void count_ern(struct dpa_percpu_priv_s *percpu_priv,
		      const struct qm_mr_entry *msg);
int dpa_enable_tx_csum(struct dpa_priv_s *priv,
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results);
#ifdef CONFIG_FSL_DPAA_CEETM
void dpa_enable_ceetm(struct net_device *dev);
void dpa_disable_ceetm(struct net_device *dev);
#endif
struct proxy_device {
		struct mac_device *mac_dev;
};

/* mac device control functions exposed by proxy interface*/
int dpa_proxy_start(struct net_device *net_dev);
int dpa_proxy_stop(struct proxy_device *proxy_dev, struct net_device *net_dev);
int dpa_proxy_set_mac_address(struct proxy_device *proxy_dev,
			  struct net_device *net_dev);
int dpa_proxy_set_rx_mode(struct proxy_device *proxy_dev,
		      struct net_device *net_dev);

#endif /* __DPAA_ETH_COMMON_H */
