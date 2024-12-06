// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2024 NXP */

#include <linux/netdevice.h>
#include "dpaa2-eth.h"

static int dpaa2_mdo_dev_open(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	int err;

	if (!priv->sec.secy)
		return -ENOENT;

	err = dpni_secy_set_state(priv->mc_io, 0, priv->mc_token, priv->secy_id, true);
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_set_state(secy_id %d) failed with %d\n",
			   priv->secy_id, err);
		return err;
	}

	return 0;
}

static int dpaa2_mdo_dev_stop(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	int err;

	if (!priv->sec.secy)
		return -ENOENT;

	err = dpni_secy_set_state(priv->mc_io, 0, priv->mc_token, priv->secy_id, false);
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_set_state(secy_id %d) failed with %d\n",
			   priv->secy_id, err);
		return err;
	}

	return 0;
}

static bool dpaa2_macsec_secy_features_validate(struct macsec_context *ctx)
{
	struct net_device *net_dev = ctx->netdev;
	struct macsec_secy *secy = ctx->secy;

	if (secy->icv_len != MACSEC_DEFAULT_ICV_LEN) {
		netdev_err(net_dev, "MACSec offload is supported only when icv_len is %d\n",
			   MACSEC_DEFAULT_ICV_LEN);
		return false;
	}

	if (secy->xpn) {
		netdev_err(net_dev, "MACSec offload is supported only without XPN\n");
		return -EOPNOTSUPP;
	}

	return true;
}

static int dpaa2_mdo_add_secy(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	struct macsec_secy *secy = ctx->secy;
	struct macsec_secy_cfg cfg = { 0 };
	u8 secy_id;
	bool up;
	int err;

	if (priv->sec.secy) {
		netdev_err(net_dev, "MACSec offload supports only one SecY instance\n");
		return -EOPNOTSUPP;
	}

	if (!dpaa2_macsec_secy_features_validate(ctx))
		return -EINVAL;

	up = netif_running(net_dev);
	if (up)
		dev_close(net_dev);

	cfg.tx_sci = sci_to_cpu(secy->sci);

	/* Validate frames */
	switch (secy->validate_frames) {
	case MACSEC_VALIDATE_DISABLED:
		cfg.validation_mode = MACSEC_SECY_VALIDATION_DISABLE;
		break;
	case MACSEC_VALIDATE_CHECK:
		cfg.validation_mode = MACSEC_SECY_VALIDATION_CHECK;
		break;
	case MACSEC_VALIDATE_STRICT:
		cfg.validation_mode = MACSEC_SECY_VALIDATION_STRICT;
		break;
	default:
		return -EOPNOTSUPP;
	}

	/* Configure the cipher suite */
	if (secy->key_len == 32)
		cfg.cs.cipher_suite = MACSEC_CIPHER_SUITE_GCM_AES_256;
	else if (secy->key_len == 16)
		cfg.cs.cipher_suite = MACSEC_CIPHER_SUITE_GCM_AES_128;
	cfg.cs.co_offset = 0;
	cfg.cs.confidentiality = ctx->secy->tx_sc.encrypt;

	cfg.is_ptp = 0;
	cfg.max_rx_sc = 1;

	err = dpni_add_secy(priv->mc_io, 0, priv->mc_token, &cfg, &secy_id);
	if (err) {
		netdev_err(net_dev, "dpni_add_secy() failed with %d\n", err);
		return err;
	}

	err = dpni_secy_set_tx_protection(priv->mc_io, 0, priv->mc_token,
					  secy_id, secy->protect_frames);
	if (err) {
		netdev_err(net_dev, "dpni_secy_set_protect_tx_protection() failed with %d\n", err);
		goto err_remove_secy;
	}
	netdev_info(net_dev, "Configured protect_frames %s\n", secy->protect_frames ? "on" : "off");

	err = dpni_secy_set_replay_protection(priv->mc_io, 0, priv->mc_token, secy_id,
					      secy->replay_protect, secy->replay_window);
	if (err) {
		netdev_err(net_dev, "dpni_secy_set_replay_protection() failed with %d\n", err);
		goto err_remove_secy;
	}
	netdev_info(net_dev, "Configured replay protection %s, window %u\n",
		    secy->replay_protect ? "on" : "off", secy->replay_window);

	if (up) {
		err = dev_open(net_dev, NULL);
		if (err)
			return err;
	}

	priv->secy_id = secy_id;
	priv->sec.secy = secy;

	return 0;

err_remove_secy:
	dpni_remove_secy(priv->mc_io, 0, priv->mc_token, secy_id);

	if (up)
		dev_open(net_dev, NULL);
	return err;
}

static int dpaa2_mdo_del_secy(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	bool up;
	int err;

	up = netif_running(net_dev);
	if (up)
		dev_close(net_dev);

	err = dpni_remove_secy(priv->mc_io, 0, priv->mc_token, priv->secy_id);
	if (err) {
		netdev_err(net_dev, "dpni_remove_secy() failed with %d\n", err);
		goto err_open_dev;
	}

	priv->sec.secy = NULL;
	priv->secy_id = 0;

	if (up) {
		err = dev_open(net_dev, NULL);
		if (err)
			return err;
	}

	return 0;

err_open_dev:
	if (up)
		dev_open(net_dev, NULL);

	return err;
}

static int dpaa2_mdo_add_txsa(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct macsec_tx_sa_cfg cfg = { 0 };
	int err, i;

	cfg.next_pn = ctx->sa.tx_sa->next_pn_halves.lower;
	for (i = 0; i < ctx->secy->key_len; i++)
		cfg.key[i] = ctx->sa.key[i];
	cfg.an = ctx->sa.assoc_num;

	err = dpni_secy_add_tx_sa(priv->mc_io, 0, priv->mc_token, priv->secy_id, &cfg);
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_add_tx_sa() failed with %d\n", err);
		return err;
	}

	if (ctx->secy->tx_sc.encoding_sa == ctx->sa.assoc_num) {
		err = dpni_secy_set_active_tx_sa(priv->mc_io, 0, priv->mc_token,
						 priv->secy_id, ctx->sa.assoc_num);
		if (err) {
			netdev_err(priv->net_dev,
				   "dpni_secy_set_active_tx_sa(secy_id %d, assoc_num %d) failed with %d\n",
				   priv->secy_id, ctx->sa.assoc_num, err);
			goto err_remove_tx_sa;
		}
	}

	return 0;

err_remove_tx_sa:
	dpni_secy_remove_tx_sa(priv->mc_io, 0, priv->mc_token, priv->secy_id, ctx->sa.assoc_num);

	return err;
}

static int dpaa2_mdo_upd_txsa(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	int err;

	if (ctx->sa.update_pn)
		return -EOPNOTSUPP;

	if (ctx->secy->tx_sc.encoding_sa == ctx->sa.assoc_num) {
		err = dpni_secy_set_active_tx_sa(priv->mc_io, 0, priv->mc_token,
						 priv->secy_id, ctx->sa.assoc_num);
		if (err) {
			netdev_err(priv->net_dev,
				   "dpni_secy_set_active_tx_sa(secy_id %d, assoc_num %d) failed with %d\n",
				   priv->secy_id, ctx->sa.assoc_num, err);
			return err;
		}
	}

	return 0;
}

static int dpaa2_mdo_del_txsa(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	int err;

	err = dpni_secy_remove_tx_sa(priv->mc_io, 0, priv->mc_token,
				     priv->secy_id, ctx->sa.assoc_num);
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_remove_tx_sa() failed with %d\n", err);
		return err;
	}

	return 0;
}

static int dpaa2_mdo_add_rxsc(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	int err;

	if (priv->md_dst) {
		netdev_err(priv->net_dev, "MACSec offload supports only one RX SC\n");
		return -EOPNOTSUPP;
	}

	/* This metadata will be used to direct packets towards the macsec
	 * software interface based on the Rx SC SCI value. Since we limit the
	 * number of Rx SCs to one, we will use the same metadata for all
	 * MACSec packets.
	 */
	priv->md_dst = metadata_dst_alloc(0, METADATA_MACSEC, GFP_KERNEL);
	if (!priv->md_dst)
		return -ENOMEM;
	priv->md_dst->u.macsec_info.sci = ctx->rx_sc->sci;

	err = dpni_secy_add_rx_sc(priv->mc_io, 0, priv->mc_token, priv->secy_id,
				  sci_to_cpu(ctx->rx_sc->sci));
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_add_rx_sc() failed with %d\n", err);
		goto err_free_md_dst;
	}

	err = dpni_secy_set_rx_sc_state(priv->mc_io, 0, priv->mc_token,
					priv->secy_id, sci_to_cpu(ctx->rx_sc->sci),
					ctx->rx_sc->active);
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_set_rx_sc_state() failed with %d\n", err);
		goto err_remove_rxsc;
	}

	return 0;

err_remove_rxsc:
	dpni_secy_remove_rx_sc(priv->mc_io, 0, priv->mc_token, priv->secy_id,
			       sci_to_cpu(ctx->rx_sc->sci));

err_free_md_dst:
	metadata_dst_free(priv->md_dst);

	return err;
}

static int dpaa2_mdo_upd_rxsc(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	int err;

	err = dpni_secy_set_rx_sc_state(priv->mc_io, 0, priv->mc_token,
					priv->secy_id, sci_to_cpu(ctx->rx_sc->sci),
					ctx->rx_sc->active);
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_set_rx_sc_state() failed with %d\n", err);
		return err;
	}
	netdev_err(net_dev, "Changed RX SC active state to %s\n",
		   ctx->rx_sc->active ? "on" : "off");

	return 0;
}

static int dpaa2_mdo_del_rxsc(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	int err;

	err = dpni_secy_remove_rx_sc(priv->mc_io, 0, priv->mc_token,
				     priv->secy_id, sci_to_cpu(ctx->rx_sc->sci));
	if (err) {
		netdev_err(priv->net_dev, "dpni_secy_remove_rx_sc() failed with %d\n", err);
		return err;
	}
	netdev_err(net_dev, "Removed RX SC with SCI 0x%llx\n", sci_to_cpu(ctx->rx_sc->sci));

	return 0;
}

static int dpaa2_mdo_add_rxsa(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	struct macsec_rx_sa_cfg cfg = { 0 };
	int err, i;

	cfg.lowest_pn = ctx->sa.tx_sa->next_pn_halves.lower;
	for (i = 0; i < ctx->secy->key_len; i++)
		cfg.key[i] = ctx->sa.key[i];
	cfg.an = ctx->sa.assoc_num;

	err = dpni_secy_add_rx_sa(priv->mc_io, 0, priv->mc_token, priv->secy_id,
				  sci_to_cpu(ctx->sa.rx_sa->sc->sci), &cfg);
	if (err) {
		netdev_err(net_dev, "dpni_secy_add_rx_sa() failed with %d\n", err);
		return err;
	}

	err = dpni_secy_set_rx_sa_next_pn(priv->mc_io, 0, priv->mc_token, priv->secy_id,
					  sci_to_cpu(ctx->sa.rx_sa->sc->sci),
					  ctx->sa.assoc_num,
					  ctx->sa.rx_sa->next_pn_halves.lower);
	if (err) {
		netdev_err(net_dev, "dpni_secy_set_rx_sa_next_pn() failed with %d\n", err);
		goto err_remove_rx_sa;
	}

	err = dpni_secy_set_rx_sa_state(priv->mc_io, 0, priv->mc_token, priv->secy_id,
					sci_to_cpu(ctx->sa.rx_sa->sc->sci), ctx->sa.assoc_num,
					ctx->sa.rx_sa->active);
	if (err) {
		netdev_err(net_dev, "dpni_secy_set_rx_sa_state() failed with %d\n", err);
		goto err_remove_rx_sa;
	}

	netdev_err(net_dev, "Added RX SA %d for SCI 0x%llx, active %s, next_pn %d\n",
		   ctx->sa.assoc_num, sci_to_cpu(ctx->sa.rx_sa->sc->sci),
		   ctx->sa.rx_sa->active ? "on" : "off",
		   ctx->sa.rx_sa->next_pn_halves.lower);

	return 0;

err_remove_rx_sa:
	dpni_secy_remove_rx_sa(priv->mc_io, 0, priv->mc_token, priv->secy_id,
			       sci_to_cpu(ctx->sa.rx_sa->sc->sci), ctx->sa.assoc_num);

	return err;
}

static int dpaa2_mdo_upd_rxsa(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	int err;

	if (ctx->sa.update_pn)
		return -EOPNOTSUPP;

	err = dpni_secy_set_rx_sa_state(priv->mc_io, 0, priv->mc_token, priv->secy_id,
					sci_to_cpu(ctx->sa.rx_sa->sc->sci), ctx->sa.assoc_num,
					ctx->sa.rx_sa->active);
	if (err) {
		netdev_err(net_dev, "dpni_secy_set_rx_sa_state() failed with %d\n", err);
		return err;
	}

	netdev_err(net_dev, "Changed RX SA %d for with SCI 0x%llx, active %s\n",
		   ctx->sa.assoc_num, sci_to_cpu(ctx->sa.rx_sa->sc->sci),
		   ctx->sa.rx_sa->active ? "on" : "off");

	return 0;
}

static int dpaa2_mdo_del_rxsa(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	int err;

	err = dpni_secy_remove_rx_sa(priv->mc_io, 0, priv->mc_token, priv->secy_id,
				     sci_to_cpu(ctx->sa.rx_sa->sc->sci), ctx->sa.assoc_num);
	if (err) {
		netdev_err(net_dev, "dpni_secy_remove_rx_sa() failed with %d\n", err);
		return err;
	}

	netdev_err(net_dev, "Removed RX SA %d for SCI 0x%llx\n",
		   ctx->sa.assoc_num, sci_to_cpu(ctx->sa.rx_sa->sc->sci));

	return 0;
}

static int dpaa2_mdo_get_dev_stats(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	union macsec_global_stats global_stats;
	union macsec_secy_stats stats;
	int err;

	/* Get and display statistics from page #0 */
	memset(&stats, 0, sizeof(stats));
	err = dpni_secy_get_stats(priv->mc_io, 0, priv->mc_token, priv->secy_id, 0, &stats);
	if (err) {
		netdev_err(net_dev, "dpni_secy_get_stats() failed with %d\n", err);
		return err;
	}

	netdev_info(net_dev, "Per SecY statistics:\n");
	netdev_info(net_dev, "\tcnt_ing_bytes : %llu\n", stats.raw.counter[0]);
	netdev_info(net_dev, "\tcnt_ing_ucast_frames : %llu\n", stats.raw.counter[1]);
	netdev_info(net_dev, "\tcnt_ing_mcast_frames : %llu\n", stats.raw.counter[2]);
	netdev_info(net_dev, "\tcnt_ing_bcast_frames : %llu\n", stats.raw.counter[3]);
	netdev_info(net_dev, "\tcnt_egr_bytes : %llu\n", stats.raw.counter[4]);
	netdev_info(net_dev, "\tcnt_egr_ucast_frames : %llu\n", stats.raw.counter[5]);
	netdev_info(net_dev, "\tcnt_egr_mcast_frames : %llu\n", stats.raw.counter[6]);

	/* Get and display statistics from page #1 */
	memset(&stats, 0, sizeof(stats));
	err = dpni_secy_get_stats(priv->mc_io, 0, priv->mc_token, priv->secy_id, 1, &stats);
	if (err) {
		netdev_err(net_dev, "dpni_secy_get_stats() failed with %d\n", err);
		return err;
	}

	netdev_info(net_dev, "\tcnt_egr_bcast_frames: %llu\n", stats.raw.counter[0]);

	/* Get global MACSec statistics (not per SecY) */
	memset(&global_stats, 0, sizeof(global_stats));
	err = dpni_get_macsec_stats(priv->mc_io, 0, priv->mc_token, &global_stats);
	if (err) {
		netdev_err(net_dev, "dpni_get_macsec_stats() failed with %d\n", err);
		return err;
	}

	netdev_info(net_dev, "MACSec global statistics:\n");
	netdev_info(net_dev, "\tin_without_tag_frames: %u\n",
		    global_stats.page_0.in_without_tag_frames);
	netdev_info(net_dev, "\tin_kay_frames: %u\n",
		    global_stats.page_0.in_kay_frames);
	netdev_info(net_dev, "\tin_bag_tag_frames: %u\n",
		    global_stats.page_0.in_bag_tag_frames);
	netdev_info(net_dev, "\tin_sci_not_found_frames: %u\n",
		    global_stats.page_0.in_sci_not_found_frames);
	netdev_info(net_dev, "\tin_unsupported_ec_frames: %u\n",
		    global_stats.page_0.in_unsupported_ec_frames);
	netdev_info(net_dev, "\tin_too_long_frames: %u\n",
		    global_stats.page_0.in_too_long_frames);
	netdev_info(net_dev, "\tout_discarded_frames: %u\n",
		    global_stats.page_0.out_discarded_frames);

	return 0;
}

static int dpaa2_mdo_get_tx_sc_stats(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	union macsec_secy_tx_sc_stats stats;
	int err;

	memset(&stats, 0, sizeof(stats));
	err = dpni_secy_get_tx_sc_stats(priv->mc_io, 0, priv->mc_token, priv->secy_id, &stats);
	if (err) {
		netdev_err(net_dev, "dpni_secy_get_tx_sc_stats() failed with %d\n", err);
		return err;
	}

	ctx->stats.tx_sc_stats->OutPktsProtected = stats.page_0.protected_frames;
	ctx->stats.tx_sc_stats->OutPktsEncrypted = stats.page_0.encrypted_frames;
	ctx->stats.tx_sc_stats->OutOctetsProtected = stats.page_0.protected_bytes;
	ctx->stats.tx_sc_stats->OutOctetsEncrypted = stats.page_0.encrypted_bytes;

	return 0;
}

static int dpaa2_mdo_get_tx_sa_stats(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	union macsec_secy_tx_sa_stats stats;
	int err;

	memset(&stats, 0, sizeof(stats));
	err = dpni_secy_get_tx_sa_stats(priv->mc_io, 0, priv->mc_token, priv->secy_id,
					ctx->sa.assoc_num, &stats);
	if (err) {
		netdev_err(net_dev, "dpni_secy_get_tx_sa_stats() failed with %d\n", err);
		return err;
	}

	ctx->stats.tx_sa_stats->OutPktsProtected = stats.page_0.protected_frames;
	ctx->stats.tx_sa_stats->OutPktsEncrypted = stats.page_0.encrypted_frames;

	return 0;
}

static int dpaa2_mdo_get_rx_sc_stats(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	union macsec_secy_rx_sc_stats stats;
	int err;

	memset(&stats, 0, sizeof(stats));
	err = dpni_secy_get_rx_sc_stats(priv->mc_io, 0, priv->mc_token, priv->secy_id,
					sci_to_cpu(ctx->rx_sc->sci), 0, &stats);
	if (err) {
		netdev_err(net_dev, "dpni_secy_get_rx_sc_stats() failed with %d\n", err);
		return err;
	}

	ctx->stats.rx_sc_stats->InPktsUnusedSA = stats.page_0.unused_frames;
	ctx->stats.rx_sc_stats->InPktsNotUsingSA = stats.page_0.not_using_sa_frames;
	ctx->stats.rx_sc_stats->InPktsInvalid = stats.page_0.invalid_frames;
	ctx->stats.rx_sc_stats->InPktsNotValid = stats.page_0.not_valid_frames;
	ctx->stats.rx_sc_stats->InPktsLate = stats.page_0.late_frames;
	ctx->stats.rx_sc_stats->InPktsDelayed = stats.page_0.delayed_frames;
	ctx->stats.rx_sc_stats->InPktsUnchecked = stats.page_0.unchecked_frames;

	memset(&stats, 0, sizeof(stats));
	err = dpni_secy_get_rx_sc_stats(priv->mc_io, 0, priv->mc_token, priv->secy_id,
					sci_to_cpu(ctx->rx_sc->sci), 1, &stats);
	if (err) {
		netdev_err(net_dev, "dpni_secy_get_rx_sc_stats() failed with %d\n", err);
		return err;
	}

	ctx->stats.rx_sc_stats->InPktsOK = stats.page_1.ok_frames;
	ctx->stats.rx_sc_stats->InOctetsValidated = stats.page_1.validated_bytes;
	ctx->stats.rx_sc_stats->InOctetsDecrypted = stats.page_1.decrypted_bytes;

	return 0;
}

static int dpaa2_mdo_get_rx_sa_stats(struct macsec_context *ctx)
{
	struct dpaa2_eth_priv *priv = macsec_netdev_priv(ctx->netdev);
	struct net_device *net_dev = priv->net_dev;
	union macsec_secy_rx_sa_stats stats;
	int err;

	memset(&stats, 0, sizeof(stats));
	err = dpni_secy_get_rx_sa_stats(priv->mc_io, 0, priv->mc_token, priv->secy_id,
					sci_to_cpu(ctx->rx_sc->sci), ctx->sa.assoc_num, &stats);
	if (err) {
		netdev_err(net_dev, "dpni_secy_get_rx_sa_stats() failed with %d\n", err);
		return err;
	}

	ctx->stats.rx_sa_stats->InPktsUnusedSA = stats.page_0.unused_sa_frames;
	ctx->stats.rx_sa_stats->InPktsNotUsingSA = stats.page_0.not_using_sa_frames;
	ctx->stats.rx_sa_stats->InPktsInvalid = stats.page_0.invalid_frames;
	ctx->stats.rx_sa_stats->InPktsNotValid = stats.page_0.not_valid_frames;
	ctx->stats.rx_sa_stats->InPktsOK = stats.page_0.ok_frames;

	return 0;
}

static const struct macsec_ops dpaa2_eth_macsec_ops = {
	.mdo_dev_open = dpaa2_mdo_dev_open,
	.mdo_dev_stop = dpaa2_mdo_dev_stop,
	.mdo_add_secy = dpaa2_mdo_add_secy,
	.mdo_del_secy = dpaa2_mdo_del_secy,
	.mdo_add_rxsc = dpaa2_mdo_add_rxsc,
	.mdo_upd_rxsc = dpaa2_mdo_upd_rxsc,
	.mdo_del_rxsc = dpaa2_mdo_del_rxsc,
	.mdo_add_rxsa = dpaa2_mdo_add_rxsa,
	.mdo_upd_rxsa = dpaa2_mdo_upd_rxsa,
	.mdo_del_rxsa = dpaa2_mdo_del_rxsa,
	.mdo_add_txsa = dpaa2_mdo_add_txsa,
	.mdo_upd_txsa = dpaa2_mdo_upd_txsa,
	.mdo_del_txsa = dpaa2_mdo_del_txsa,
	.mdo_get_dev_stats = dpaa2_mdo_get_dev_stats,
	.mdo_get_tx_sc_stats = dpaa2_mdo_get_tx_sc_stats,
	.mdo_get_tx_sa_stats = dpaa2_mdo_get_tx_sa_stats,
	.mdo_get_rx_sc_stats = dpaa2_mdo_get_rx_sc_stats,
	.mdo_get_rx_sa_stats = dpaa2_mdo_get_rx_sa_stats,
	.rx_uses_md_dst = true,
};

int dpaa2_eth_macsec_init(struct dpaa2_eth_priv *priv)
{
	struct net_device *net_dev = priv->net_dev;
	int capable, err;

	if (!(priv->features & DPAA2_ETH_FEATURE_MACSEC))
		return 0;

	err = dpni_is_macsec_capable(priv->mc_io, 0, priv->mc_token, &capable);
	if (err) {
		netdev_err(priv->net_dev, "dpni_is_macsec_capable() failed, not MACSec offload\n");
		return 0;
	}

	if (!capable)
		return 0;

	priv->sec.secy = NULL;
	priv->secy_id = 0;
	priv->md_dst = NULL;

	net_dev->features |= NETIF_F_HW_MACSEC;
	net_dev->macsec_ops = &dpaa2_eth_macsec_ops;
	netif_keep_dst(net_dev);
	return 0;
}

void dpaa2_eth_macsec_deinit(struct dpaa2_eth_priv *priv)
{
	struct net_device *net_dev = priv->net_dev;

	net_dev->features &= !NETIF_F_HW_MACSEC;
	net_dev->macsec_ops = NULL;

	if (priv->md_dst)
		metadata_dst_free(priv->md_dst);

	priv->sec.secy = NULL;
	priv->secy_id = 0;
	priv->md_dst = NULL;
}
