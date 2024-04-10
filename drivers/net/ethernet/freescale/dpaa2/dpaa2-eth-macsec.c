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

static const struct macsec_ops dpaa2_eth_macsec_ops = {
	.mdo_dev_open = dpaa2_mdo_dev_open,
	.mdo_dev_stop = dpaa2_mdo_dev_stop,
	.mdo_add_secy = dpaa2_mdo_add_secy,
	.mdo_del_secy = dpaa2_mdo_del_secy,
	.mdo_add_txsa = dpaa2_mdo_add_txsa,
	.mdo_upd_txsa = dpaa2_mdo_upd_txsa,
	.mdo_del_txsa = dpaa2_mdo_del_txsa,
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
}
