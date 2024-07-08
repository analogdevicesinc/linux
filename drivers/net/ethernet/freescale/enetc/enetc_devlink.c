// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2024 NXP */
#include "enetc_pf.h"

#define ENETC_RINGS_STR_LEN		16
#define ENETC_RINGS_STR_BUFF_LEN	(ENETC_RINGS_STR_LEN + 1)
#define ENETC_SI_MAX_RINGS_NUM		32

static int enetc_devlink_reload_down(struct devlink *devlink, bool netns_change,
				     enum devlink_reload_action action,
				     enum devlink_reload_limit limit,
				     struct netlink_ext_ack *extack)
{
	struct enetc_devlink_priv *devl_priv = devlink_priv(devlink);
	struct enetc_pf *pf = devl_priv->pf;

	switch (action) {
	case DEVLINK_RELOAD_ACTION_DRIVER_REINIT:
		if (pf->num_vfs) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Remove all VFs before doing reinit\n");
			return -EOPNOTSUPP;
		}

		if (devl_priv->pf_unload)
			return devl_priv->pf_unload(pf);
		else
			return -EOPNOTSUPP;
	default:
		return -EOPNOTSUPP;
	}
}

static int enetc_devlink_parse_rings_param(union devlink_param_value *val,
					   u32 *si_num_rings, u32 *ring_cnt)
{
	char str_buff[ENETC_RINGS_STR_BUFF_LEN];
	int str_len, num_step;
	int i, j, k, err;
	u64 param_val;
	u32 ring_num;
	int pos, len;

	str_len = strnlen(val->vstr, sizeof(val->vstr));
	num_step = str_len / ENETC_RINGS_STR_LEN;
	if (str_len % ENETC_RINGS_STR_LEN)
		num_step += 1;

	for (i = num_step - 1, k = 0; i >= 0; i--) {
		pos = i ? str_len - ENETC_RINGS_STR_LEN : 0;
		len = i ? ENETC_RINGS_STR_BUFF_LEN : str_len + 1;
		strscpy(str_buff, &val->vstr[pos], len);

		err = kstrtoull(str_buff, 16, &param_val);
		if (err)
			return err;

		for (j = 0; j < 8 && k < ENETC_MAX_VF_NUM; j++) {
			ring_num = param_val & 0xff;
			si_num_rings[k++] = ring_num;
			*ring_cnt += ring_num;
			param_val >>= 8;
		}

		str_len -= ENETC_RINGS_STR_LEN;
	}

	return 0;
}

static void enetc_devlink_get_rings_param(struct devlink *devlink)
{
	struct enetc_devlink_priv *devl_priv = devlink_priv(devlink);
	struct enetc_pf *pf = devl_priv->pf;
	u32 rings_cap = pf->caps.num_rx_bdr;
	union devlink_param_value val;
	u32 ring_num, ring_cnt = 0;
	int vf_base_id = 1;
	int str_len, err;

	err = devl_param_driverinit_value_get(devlink,
					      ENETC_DEVLINK_PARAM_ID_RING_NUM_ASSIGN,
					      &val);
	if (err)
		goto clear_si_num_rings;

	str_len = strnlen(val.vstr, sizeof(val.vstr));
	if (!str_len)
		goto clear_si_num_rings;

	err = enetc_devlink_parse_rings_param(&val, &devl_priv->si_num_rings[vf_base_id],
					      &ring_cnt);
	if (err)
		goto clear_si_num_rings;

	/* Calculate the number of rings of PF */
	ring_num = rings_cap - ring_cnt;
	if (ring_num > ENETC_SI_MAX_RINGS_NUM)
		ring_num = ENETC_SI_MAX_RINGS_NUM;

	devl_priv->si_num_rings[0] = ring_num;

	return;

clear_si_num_rings:
	memset(devl_priv->si_num_rings, 0, sizeof(devl_priv->si_num_rings));
}

static void enetc_devlink_get_params(struct devlink *devlink)
{
	devl_assert_locked(devlink);

	enetc_devlink_get_rings_param(devlink);
}

static int enetc_devlink_reload_up(struct devlink *devlink,
				   enum devlink_reload_action action,
				   enum devlink_reload_limit limit,
				   u32 *actions_performed,
				   struct netlink_ext_ack *extack)
{
	struct enetc_devlink_priv *devl_priv = devlink_priv(devlink);
	struct enetc_pf *pf = devl_priv->pf;

	switch (action) {
	case DEVLINK_RELOAD_ACTION_DRIVER_REINIT:
		*actions_performed = BIT(DEVLINK_RELOAD_ACTION_DRIVER_REINIT);
		enetc_devlink_get_params(devlink);

		if (devl_priv->pf_load)
			return devl_priv->pf_load(pf);
		else
			return -EOPNOTSUPP;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct devlink_ops enetc_devlink_ops = {
	.reload_actions = BIT(DEVLINK_RELOAD_ACTION_DRIVER_REINIT),
	.reload_down = enetc_devlink_reload_down,
	.reload_up = enetc_devlink_reload_up,
};

static void enetc_devlink_free(void *devlink)
{
	devlink_free((struct devlink *)devlink);
}

int enetc_devlink_alloc(struct enetc_pf *pf)
{
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_devlink_priv *devl_priv;
	struct devlink *devlink;
	int err;

	devlink = devlink_alloc(&enetc_devlink_ops,
				sizeof(struct enetc_devlink_priv), dev);
	if (!devlink)
		return -ENOMEM;

	/* Add an action to teardown the devlink when unwinding the driver */
	err = devm_add_action_or_reset(dev, enetc_devlink_free, devlink);
	if (err)
		return err;

	devl_priv = devlink_priv(devlink);
	devl_priv->pf = pf;
	pf->devl_priv = devl_priv;

	return 0;
}

void enetc_devlink_init_params(struct devlink *devlink)
{
	struct enetc_devlink_priv *devl_priv = devlink_priv(devlink);
	union devlink_param_value val;

	devl_lock(devlink);
	memset(&val, 0, sizeof(val));
	memset(devl_priv->si_num_rings, 0, sizeof(devl_priv->si_num_rings));
	devl_param_driverinit_value_set(devlink,
					ENETC_DEVLINK_PARAM_ID_RING_NUM_ASSIGN,
					val);
	devl_unlock(devlink);
}

static int enetc_devlink_ring_num_validate(struct devlink *devlink, u32 id,
					   union devlink_param_value val,
					   struct netlink_ext_ack *extack)
{
	struct enetc_devlink_priv *devl_priv = devlink_priv(devlink);
	u32 vf_num_rings[ENETC_MAX_VF_NUM] = {0};
	struct enetc_pf *pf = devl_priv->pf;
	u32 rings_cap = pf->caps.num_rx_bdr;
	u32 num_vf = pf->caps.num_vsi;
	u32 ring_cnt = 0;
	int i, err;

	/* Parse string parameter */
	err = enetc_devlink_parse_rings_param(&val, vf_num_rings, &ring_cnt);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Parameter format is error");
		return -EINVAL;
	}

	for (i = 0; i < ENETC_MAX_VF_NUM; i++) {
		if (i < num_vf) {
			if (!vf_num_rings[i]) {
				NL_SET_ERR_MSG_MOD(extack, "Rings number is at least 1");
				return -EINVAL;
			} else if (vf_num_rings[i] > ENETC_SI_MAX_RINGS_NUM) {
				NL_SET_ERR_MSG_FMT_MOD(extack, "Maximum number is %d",
						       ENETC_SI_MAX_RINGS_NUM);
				return -EINVAL;
			}
		} else {
			if (vf_num_rings[i]) {
				NL_SET_ERR_MSG_MOD(extack,
						   "Cannot allocate rings to inexistent VF");
				return -EINVAL;
			}
		}
	}

	if (rings_cap < ring_cnt) {
		NL_SET_ERR_MSG_MOD(extack, "Rings number exceeds hardware capability");
		return -EINVAL;
	}

	if (rings_cap == ring_cnt) {
		NL_SET_ERR_MSG_MOD(extack, "No rings left for PF");
		return -EINVAL;
	}

	return 0;
}

static const struct devlink_param enetc_devlink_params[] = {
	DEVLINK_PARAM_DRIVER(ENETC_DEVLINK_PARAM_ID_RING_NUM_ASSIGN,
			     "si_num_rings", DEVLINK_PARAM_TYPE_STRING,
			     BIT(DEVLINK_PARAM_CMODE_DRIVERINIT),
			     NULL, NULL, enetc_devlink_ring_num_validate),
};

int enetc_devlink_params_register(struct devlink *devlink)
{
	return devlink_params_register(devlink, enetc_devlink_params,
				       ARRAY_SIZE(enetc_devlink_params));
}

void enetc_devlink_params_unregister(struct devlink *devlink)
{
	devlink_params_unregister(devlink, enetc_devlink_params,
				  ARRAY_SIZE(enetc_devlink_params));
}
