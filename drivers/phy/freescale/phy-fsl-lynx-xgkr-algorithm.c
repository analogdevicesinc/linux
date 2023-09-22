// SPDX-License-Identifier: GPL-2.0
/* Copyright 2018-2023 NXP
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>

#include "phy-fsl-lynx-xgkr-algorithm.h"

#define BIN_SNAPSHOT_NUM			10
#define TIMEOUT_RX_HAPPY			3
#define TIMEOUT_LONG				3
#define TIMEOUT_M1				3

/* Max/Min coefficient values (according to algorithm designer) */
#define PRE_COE_MAX				0x0
#define PRE_COE_MIN				0x8
#define POST_COE_MAX				0x0
#define POST_COE_MIN				0x10
#define ZERO_COE_MIN				0x1A
#define ZERO_COE_MAX				0x30

#define BIN_SNP_AV_THR_LOW			-150
#define BIN_SNP_AV_THR_HIGH			150

/* OSESTAT middle range */
#define OSESTAT_MIDRANGE_LOW			0x10
#define OSESTAT_MIDRANGE_HIGH			0x2F

enum lynx_xgkr_update_requester {
	UPDATE_REQUESTER_NONE,
	UPDATE_REQUESTER_INIT,
	UPDATE_REQUESTER_PRESET,
	UPDATE_REQUESTER_MOVE_BACK_TO_PREV,
	UPDATE_REQUESTER_BIN_LONG,
	UPDATE_REQUESTER_BIN_M1,
};

enum lynx_xgkr_bad_state_reason {
	BAD_STATE_RX_NOT_HAPPY,
	BAD_STATE_BIN_LONG,
	BAD_STATE_BIN_M1,
};

static const char * const bad_state_reason_strings[] = {
	[BAD_STATE_RX_NOT_HAPPY] = "RX not happy",
	[BAD_STATE_BIN_LONG] = "BinLong module failed",
	[BAD_STATE_BIN_M1] = "BinM1 module failed",
};

enum coef_field {
	COEF_FIELD_COP1,
	COEF_FIELD_COZ,
	COEF_FIELD_COM1,
	COEF_FIELD_MAX,
};

enum lynx_bin_state {
	BIN_INVALID,
	BIN_EARLY,
	BIN_TOGGLE,
	BIN_LATE,
};

struct lynx_xgkr_remote_tx_status {
	enum lynx_xgkr_update_requester last_update_requester;
	bool bin_m1_stop;
	bool bin_long_stop;
	enum lynx_bin_state prev_bin_m1_state;
	enum lynx_bin_state prev_bin_long_state;
	struct c72_coef_update prev_update;
	struct c72_coef_status prev_status;
	struct c72_coef_update prev_bin_m1_update;
	struct c72_coef_status prev_bin_m1_status;
	struct c72_coef_update prev_bin_long_update;
	struct c72_coef_status prev_bin_long_status;
	enum coef_status last_updated_status_cop1;
	enum coef_status last_updated_status_coz;
	int num_steps;
};

struct lynx_xgkr_local_tx_status {
	struct lynx_xgkr_tx_eq tuned_tx_eq;
	int num_steps;
};

struct lynx_xgkr_algorithm {
	struct phy *phy;
	const struct lynx_xgkr_algorithm_ops *ops;
	struct lynx_xgkr_remote_tx_status rts;
	struct lynx_xgkr_local_tx_status lts;
	struct lynx_xgkr_tx_eq default_tx_eq;
};

static int lynx_xgkr_remote_tx_request(struct lynx_xgkr_remote_tx_status *rts,
				       enum lynx_xgkr_update_requester requester)
{
	rts->num_steps++;
	rts->last_update_requester = requester;
	return 0;
}

static void lynx_xgkr_move_back_to_prev(struct lynx_xgkr_algorithm *algorithm,
					struct c72_coef_update *update)
{
	struct lynx_xgkr_remote_tx_status *rts = &algorithm->rts;
	const struct c72_coef_update *prev = &rts->prev_update;

	/* Move back to previous C-, C0, C+ */
	update->com1 = coef_update_opposite(prev->com1);
	update->coz = coef_update_opposite(prev->coz);
	update->cop1 = coef_update_opposite(prev->cop1);
}

static int
lynx_xgkr_process_bad_state(struct lynx_xgkr_algorithm *algorithm,
			    struct c72_phy_configure_remote_tx *remote_tx,
			    enum lynx_xgkr_bad_state_reason reason)
{
	struct lynx_xgkr_remote_tx_status *rts = &algorithm->rts;
	struct c72_coef_update *update = &remote_tx->update;
	struct phy *phy = algorithm->phy;

	if (rts->last_update_requester == UPDATE_REQUESTER_INIT) {
		dev_warn(&phy->dev,
			 "Bad state detected (%s) while LP is still at INIT, trying PRESET\n",
			 bad_state_reason_strings[reason]);
		update->preset = true;
		return lynx_xgkr_remote_tx_request(rts, UPDATE_REQUESTER_PRESET);
	}

	if (rts->last_update_requester != UPDATE_REQUESTER_PRESET) {
		dev_warn(&phy->dev,
			 "Bad state detected (%s), moving to previous coefficients\n",
			 bad_state_reason_strings[reason]);
		/* Move back to previous C-, C0, C+ and HOLD */
		lynx_xgkr_move_back_to_prev(algorithm, update);
		return lynx_xgkr_remote_tx_request(rts, UPDATE_REQUESTER_MOVE_BACK_TO_PREV);
	}

	dev_warn(&phy->dev,
		 "Bad state (%s) persists after requesting PRESET; LT failure\n",
		 bad_state_reason_strings[reason]);

	return -EINVAL;
}

static bool
lynx_xgkr_collect_bin_snapshots(struct lynx_xgkr_algorithm *algorithm,
				enum lynx_bin_type bin_type, s16 *bin_snapshot)
{
	const struct lynx_xgkr_algorithm_ops *ops = algorithm->ops;
	struct phy *phy = algorithm->phy;
	int i, err;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		err = ops->snapshot_rx_eq_bin(phy, bin_type, &bin_snapshot[i]);
		if (err)
			return false;
	}

	return true;
}

static bool lynx_xgkr_collect_gains(struct lynx_xgkr_algorithm *algorithm,
				    u8 *gaink2_snapshot, u8 *gaink3_snapshot,
				    u8 *osestat_snapshot)
{
	const struct lynx_xgkr_algorithm_ops *ops = algorithm->ops;
	struct phy *phy = algorithm->phy;
	int i, err;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		err = ops->snapshot_rx_eq_gains(phy, &gaink2_snapshot[i],
						&gaink3_snapshot[i],
						&osestat_snapshot[i]);
		if (err)
			return false;
	}

	return true;
}

static enum lynx_bin_state lynx_get_bin_snapshots_state(s16 *bin_snapshots)
{
	s32 snapshot_avg, snapshot_sum = 0;
	int i;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++)
		snapshot_sum += bin_snapshots[i];

	snapshot_avg = (s16)(snapshot_sum / BIN_SNAPSHOT_NUM);

	if (snapshot_avg >= -256 && snapshot_avg < BIN_SNP_AV_THR_LOW)
		return BIN_EARLY;
	else if (snapshot_avg >= BIN_SNP_AV_THR_LOW &&
		 snapshot_avg < BIN_SNP_AV_THR_HIGH)
		return BIN_TOGGLE;
	else if (snapshot_avg >= BIN_SNP_AV_THR_HIGH &&
		 snapshot_avg <= 255)
		return BIN_LATE;

	return BIN_INVALID;
}

/* Checking Bins/Gains after LP has updated its TX */
static bool lynx_xgkr_is_rx_happy(struct lynx_xgkr_algorithm *algorithm)
{
	/* Bin snapshots */
	enum lynx_bin_state bin1_snapshot_state;
	enum lynx_bin_state bin2_snapshot_state;
	enum lynx_bin_state bin3_snapshot_state;
	s16 bin_offset_snapshot[BIN_SNAPSHOT_NUM];
	s16 bin1_snapshot[BIN_SNAPSHOT_NUM];
	s16 bin2_snapshot[BIN_SNAPSHOT_NUM];
	s16 bin3_snapshot[BIN_SNAPSHOT_NUM];
	/* Gain snapshots */
	u8 osestat_snapshot[BIN_SNAPSHOT_NUM];
	u8 gaink2_snapshot[BIN_SNAPSHOT_NUM];
	u8 gaink3_snapshot[BIN_SNAPSHOT_NUM];
	struct phy *phy = algorithm->phy;
	bool rx_happy = false;
	u8 min_snp, max_snp;
	s16 snapshot;
	int i;

	/* collect Bin snapshots */
	if (!lynx_xgkr_collect_bin_snapshots(algorithm, BIN_1, bin1_snapshot) ||
	    !lynx_xgkr_collect_bin_snapshots(algorithm, BIN_2, bin2_snapshot) ||
	    !lynx_xgkr_collect_bin_snapshots(algorithm, BIN_3, bin3_snapshot) ||
	    !lynx_xgkr_collect_bin_snapshots(algorithm, BIN_OFFSET,
					     bin_offset_snapshot)) {
		dev_err(&phy->dev,
			"Failed to collect bin snapshots for is_rx_happy module\n");
		return false;
	}

	/* collect Gains */
	if (!lynx_xgkr_collect_gains(algorithm, gaink2_snapshot, gaink3_snapshot,
				     osestat_snapshot)) {
		dev_err(&phy->dev,
			"Failed to collect gains for is_rx_happy module\n");
		return false;
	}

	/* Offset Bin must NOT be 10 of the same value
	 * 10G Lynx (T or LS): when LNmTCSR1.CDR_SEL = 0x3, LNmTCSR1.EQ_SNPBIN_DATA= Offset Bin
	 * 28G Lynx (LX): when LNmTCSR1.CDR_SEL = 0x4, LNmRECR4.EQ_SNPBIN_DATA= Offset Bin
	 */
	snapshot = bin_offset_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (snapshot != bin_offset_snapshot[i]) {
			rx_happy = true;
			break;
		}
	}

	if (!rx_happy) {
		dev_info(&phy->dev,
			 "RX not happy: offset Bin must not be 10 of the same value\n");
		return false;
	}

	/* Offset status must dither (+/-2) around MidRange value
	 *
	 * 10G Lynx (T or LS): LNmRECR1.OSETSTAT = Offset Status
	 * MaxNeg = 0x0, MaxPos = 0x3F, MidRange: 0x10 - 0x2F
	 * 28G Lynx (LX): LNmRECR4.OSETSTAT = Offset Status
	 * MaxNeg = 0x0, MaxPos = 0x3F, MidRange: 0x10 - 0x2F
	 *
	 * What we want to see is that the Offset has settled to a value
	 * somewhere between 0x10 and 0x2F and that the series of snapshot
	 * values are +/-2 of the settled value.
	 */
	rx_happy = true;
	min_snp = osestat_snapshot[0];
	max_snp = osestat_snapshot[0];

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (osestat_snapshot[i] < OSESTAT_MIDRANGE_LOW ||
		    osestat_snapshot[i] > OSESTAT_MIDRANGE_HIGH) {
			rx_happy = false;
			break;
		}
		if (osestat_snapshot[i] < min_snp)
			min_snp = osestat_snapshot[i];
		if (osestat_snapshot[i] > max_snp)
			max_snp = osestat_snapshot[i];
	}
	if (max_snp - min_snp > 4)
		rx_happy = false;
	if (!rx_happy) {
		dev_info(&phy->dev,
			 "RX not happy: offset status must dither (+/-2) around mid range value\n");
		return false;
	}

	/* The RX is happy if:
	 * Bin1, Bin2, and Bin3 are toggling as defined on slide 0
	 * Proceed to BinLong/BinM1 modules
	 */
	bin1_snapshot_state = lynx_get_bin_snapshots_state(bin1_snapshot);
	bin2_snapshot_state = lynx_get_bin_snapshots_state(bin2_snapshot);
	bin3_snapshot_state = lynx_get_bin_snapshots_state(bin3_snapshot);

	rx_happy = (bin1_snapshot_state == BIN_TOGGLE &&
		    bin2_snapshot_state == BIN_TOGGLE &&
		    bin3_snapshot_state == BIN_TOGGLE);
	if (rx_happy)
		return true;

	dev_info(&phy->dev, "RX not happy: No happy condition met\n");

	return false;
}

/* If the latest state was at MIN/MAX some time in the past and we still want
 * to INC/DEC, then do we are done with this module.
 */
static bool lynx_xgkr_process_bin_m1(struct lynx_xgkr_algorithm *algorithm,
				     struct c72_coef_update *update)
{
	struct lynx_xgkr_remote_tx_status *rts = &algorithm->rts;
	s16 bin_m1_snapshot[BIN_SNAPSHOT_NUM];
	enum coef_status prev_status_com1;
	enum lynx_bin_state bin_m1_state;
	enum coef_update prev_req_com1;

	prev_req_com1 = rts->prev_bin_m1_update.com1;
	prev_status_com1 = rts->prev_bin_m1_status.com1;

	if (!lynx_xgkr_collect_bin_snapshots(algorithm, BIN_M1,
					     bin_m1_snapshot))
		return false;

	bin_m1_state = lynx_get_bin_snapshots_state(bin_m1_snapshot);
	if (WARN_ON(bin_m1_state == BIN_INVALID))
		return false;

	if (bin_m1_state == BIN_TOGGLE) {
		/* Toggle path */
		if (rts->prev_bin_m1_state == bin_m1_state) {
			/* Hold C- */
			update->com1 = COEF_UPD_HOLD;
		} else {
			update->com1 = COEF_UPD_HOLD;
			/* according to: v1.0 */
			/* If previous step moved C- repeat C- move */
			if (prev_req_com1 == COEF_UPD_INC ||
			    prev_req_com1 == COEF_UPD_DEC)
				update->com1 = prev_req_com1;
		}
	} else {
		if (rts->prev_bin_m1_state == BIN_TOGGLE) {
			/* according to: v1.0 */
			/* If previous step moved C- go back on C- */
			update->com1 = coef_update_opposite(prev_req_com1);
		} else {
			if (rts->prev_bin_m1_state == bin_m1_state) {
				if (bin_m1_state == BIN_LATE) {
					/* Late path, request Decrement c(-1) */
					update->com1 = COEF_UPD_DEC;
				} else {
					/* Early path, request Increment c(-1) */
					update->com1 = COEF_UPD_INC;
				}
			} else {
				/* according to: v1.0 */
				if (bin_m1_state == BIN_LATE) {
					/* request Decrement c(-1) */
					update->com1 = COEF_UPD_DEC;
				} else {
					/* Hold C(-1) */
					update->com1 = COEF_UPD_HOLD;
				}
			}
		}
	}

	coef_update_clamp(&update->com1, prev_status_com1);

	rts->prev_bin_m1_state = bin_m1_state;

	return true;
}

static bool lynx_xgkr_process_bin_long(struct lynx_xgkr_algorithm *algorithm,
				       struct c72_coef_update *update)
{
	struct lynx_xgkr_remote_tx_status *rts = &algorithm->rts;
	enum coef_status prev_status_cop1, prev_status_coz;
	enum coef_update prev_req_cop1, prev_req_coz;
	s16 bin_long_snapshot[BIN_SNAPSHOT_NUM];
	enum lynx_bin_state bin_long_state;

	prev_req_cop1 = rts->prev_bin_long_update.cop1;
	prev_req_coz = rts->prev_bin_long_update.coz;
	prev_status_cop1 = rts->prev_bin_long_status.cop1;
	prev_status_coz = rts->prev_bin_long_status.coz;

	if (prev_status_cop1 != COEF_STAT_NOT_UPDATED)
		rts->last_updated_status_cop1 = prev_status_cop1;
	if (prev_status_coz != COEF_STAT_NOT_UPDATED)
		rts->last_updated_status_coz = prev_status_coz;

	if (!lynx_xgkr_collect_bin_snapshots(algorithm, BIN_LONG,
					     bin_long_snapshot))
		return false;

	bin_long_state = lynx_get_bin_snapshots_state(bin_long_snapshot);
	if (WARN_ON(bin_long_state == BIN_INVALID))
		return false;

	if (bin_long_state == BIN_TOGGLE) {
		/* Toggle path */
		if (rts->prev_bin_long_state != bin_long_state) {
			/* If previous step moved C+/C0 repeat C+/C0 move */
			if (prev_req_cop1 == COEF_UPD_INC || prev_req_cop1 == COEF_UPD_DEC ||
			    prev_req_coz == COEF_UPD_INC || prev_req_coz == COEF_UPD_DEC) {
				update->cop1 = prev_req_cop1;
				update->coz = prev_req_coz;
			}
		}
	} else {
		if (rts->prev_bin_long_state == BIN_TOGGLE) {
			/* If previous step moved C+/C0 go back on C+/C0 */
			update->cop1 = coef_update_opposite(prev_req_cop1);
			update->coz = coef_update_opposite(prev_req_coz);
		} else {
			if (rts->prev_bin_long_state == bin_long_state) {
				if (bin_long_state == BIN_LATE) {
					/* Late path (make edge earlier) */
					if (prev_status_cop1 == COEF_STAT_MIN) {
						if (prev_status_coz == COEF_STAT_MIN) {
							/* Hold C(0) */
							update->coz = COEF_UPD_HOLD;
						} else {
							/* request Decrement c(0) */
							update->coz = COEF_UPD_DEC;
						}
					} else {
						/* request Decrement c(+1) */
						update->cop1 = COEF_UPD_DEC;
					}
				} else {
					/* Early path (make edge later) */
					if (prev_status_cop1 == COEF_STAT_MAX) {
						if (prev_status_coz == COEF_STAT_MAX) {
							/* Hold C(+1), C(0) */
							update->cop1 = COEF_UPD_HOLD;
							update->coz = COEF_UPD_HOLD;
						} else {
							/* request Increment C(0) and Decrement c(+1) */
							update->coz = COEF_UPD_INC;
							update->cop1 = COEF_UPD_DEC;
						}
					} else {
						/* request Increment c(+1) */
						update->cop1 = COEF_UPD_INC;
					}
				}
			} else {
				/* If previous step moved C+ go back on C+ */
				update->cop1 = coef_update_opposite(prev_req_cop1);

				if (bin_long_state == BIN_LATE) {
					/* request Decrement C(0) */
					update->coz = COEF_UPD_DEC;
				} else {
					/* request Increment C(0) */
					update->coz = COEF_UPD_INC;
				}
			}
		}
	}

	coef_update_clamp(&update->coz, rts->last_updated_status_coz);
	coef_update_clamp(&update->cop1, rts->last_updated_status_cop1);

	rts->prev_bin_long_state = bin_long_state;

	return true;
}

/* Gets called once link partner acks our coefficient request */
static void lynx_xgkr_remote_tx_cb(void *priv, int err,
				   struct c72_coef_update update,
				   struct c72_coef_status status)
{
	struct lynx_xgkr_algorithm *algorithm = priv;
	struct lynx_xgkr_remote_tx_status *rts = &algorithm->rts;
	struct phy *phy = algorithm->phy;

	/* Communication timeout - not much we can do */
	if (err)
		return;

	mutex_lock(&phy->mutex);

	/* Different portions of the algorithm ("requesters of updates") act on
	 * different cursors/coefficients, and so, we need to cache the updates
	 * and replies separately so that they aren't mixed up with HOLD
	 * updates or NOT_UPDATED statuses for those same coefficients from
	 * other modules.
	 */
	switch (rts->last_update_requester) {
	case UPDATE_REQUESTER_BIN_M1:
		rts->prev_bin_m1_update = update;
		rts->prev_bin_m1_status = status;
		break;
	case UPDATE_REQUESTER_BIN_LONG:
		rts->prev_bin_long_update = update;
		rts->prev_bin_long_status = status;
		break;
	default:
		break;
	}

	rts->prev_update = update;
	rts->prev_status = status;

	mutex_unlock(&phy->mutex);
}

static int
lynx_xgkr_train_remote_tx(struct lynx_xgkr_algorithm *algorithm,
			  struct c72_phy_configure_remote_tx *remote_tx)
{
	struct lynx_xgkr_remote_tx_status *rts = &algorithm->rts;
	struct c72_coef_update *update = &remote_tx->update;
	bool is_rx_happy;
	int i;

	remote_tx->cb = lynx_xgkr_remote_tx_cb;
	remote_tx->cb_priv = algorithm;

	if (rts->last_update_requester == UPDATE_REQUESTER_MOVE_BACK_TO_PREV) {
		remote_tx->rx_ready = true;
		return 0;
	}

	/* IEEE802.3-2008, 72.6.10.2.3.2 we send initialize to the other side
	 * to ensure default settings for the LP. Naturally, we should do this
	 * only once.
	 */
	if (rts->last_update_requester == UPDATE_REQUESTER_NONE) {
		update->init = true;
		return lynx_xgkr_remote_tx_request(rts, UPDATE_REQUESTER_INIT);
	}

	for (i = 0; i < TIMEOUT_RX_HAPPY; i++) {
		is_rx_happy = lynx_xgkr_is_rx_happy(algorithm);
		if (is_rx_happy)
			break;
	}

	if (!is_rx_happy)
		return lynx_xgkr_process_bad_state(algorithm, remote_tx,
						   BAD_STATE_RX_NOT_HAPPY);

	/* Move to BinLong/BinM1 modules */

	/* The order of bin modules is that we try to finish BinLong before we
	 * do BinM1 (as used by the old algorithm). This controls the movement
	 * of the link partner's post1q and zero cursors.
	 */
	if (!rts->bin_long_stop) {
		for (i = 0; i < TIMEOUT_LONG; i++) {
			if (!lynx_xgkr_process_bin_long(algorithm, update))
				return lynx_xgkr_process_bad_state(algorithm,
								   remote_tx,
								   BAD_STATE_BIN_LONG);

			/* We have a request coming from BinLong, so send it */
			if (!coef_update_is_all_hold(update))
				return lynx_xgkr_remote_tx_request(rts,
								   UPDATE_REQUESTER_BIN_LONG);
		}

		rts->bin_long_stop = true;
	}

	/* Decide on movement of preq, ask for movement */
	if (!rts->bin_m1_stop) {
		for (i = 0; i < TIMEOUT_M1; i++) {
			if (!lynx_xgkr_process_bin_m1(algorithm, update))
				return lynx_xgkr_process_bad_state(algorithm,
								   remote_tx,
								   BAD_STATE_BIN_M1);

			/* We have a request coming from BinM1, so send it */
			if (!coef_update_is_all_hold(update))
				return lynx_xgkr_remote_tx_request(rts,
								   UPDATE_REQUESTER_BIN_M1);
		}

		rts->bin_m1_stop = true;
	}

	/* All C are in Hold and both Bins are stopped,
	 * so the training is done
	 */
	if (rts->bin_m1_stop && rts->bin_long_stop)
		remote_tx->rx_ready = true;

	return 0;
}

static void lynx_tune_tx_eq(struct lynx_xgkr_algorithm *algorithm,
			    const struct lynx_xgkr_tx_eq *tx_eq)
{
	const struct lynx_xgkr_algorithm_ops *ops = algorithm->ops;
	struct lynx_xgkr_local_tx_status *lts = &algorithm->lts;
	struct phy *phy = algorithm->phy;

	ops->tune_tx_eq(phy, tx_eq);

	lts->tuned_tx_eq = *tx_eq;
}

static int lynx_xgkr_init(struct lynx_xgkr_algorithm *algorithm,
			  struct c72_coef_status *status)
{
	lynx_tune_tx_eq(algorithm, &algorithm->default_tx_eq);

	status->com1 = COEF_STAT_UPDATED;
	status->coz = COEF_STAT_UPDATED;
	status->cop1 = COEF_STAT_UPDATED;

	return 0;
}

static int lynx_xgkr_preset(struct lynx_xgkr_algorithm *algorithm,
			    struct c72_coef_status *status)
{
	struct lynx_xgkr_tx_eq new_tx_eq = {
		.ratio_preq = PRE_COE_MAX,
		.ratio_post1q = POST_COE_MAX,
		.adapt_eq = ZERO_COE_MAX,
		.amp_reduction = algorithm->default_tx_eq.amp_reduction,
	};

	/* Preset as defined by: IEEE 802.3, sub-clause 72.6.10.2.3.1
	 * These are all MAX values from the IEEE802.3 perspective.
	 */
	lynx_tune_tx_eq(algorithm, &new_tx_eq);

	status->com1 = COEF_STAT_MAX;
	status->coz = COEF_STAT_MAX;
	status->cop1 = COEF_STAT_MAX;

	return 0;
}

static void lynx_xgkr_read_default_tx_eq(struct lynx_xgkr_algorithm *algorithm)
{
	const struct lynx_xgkr_algorithm_ops *ops = algorithm->ops;
	struct lynx_xgkr_local_tx_status *lts = &algorithm->lts;
	struct phy *phy = algorithm->phy;

	ops->read_tx_eq(phy, &algorithm->default_tx_eq);
	lts->tuned_tx_eq = algorithm->default_tx_eq;

	dev_dbg(&phy->dev,
		"Default Tx equalization: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x, AMP_RED = 0x%x\n",
		algorithm->default_tx_eq.ratio_preq,
		algorithm->default_tx_eq.ratio_post1q,
		algorithm->default_tx_eq.adapt_eq,
		algorithm->default_tx_eq.amp_reduction);
}

/* Coefficient values have hardware restrictions:
 *
 * Section 5.3.1 10GBaseKR Transmit Adaptive Equalization Control additional
 * restrictions set down by the 802.3 specification Clause 72, specifically
 * 72.7.1.11 Transmitter output waveform requirements.
 *
 * Maintaining the following relationships limit the transmit equalization to
 * reasonable levels compliant with the 10GBaseKR specification.
 *
 * These restrictions are:
 *
 * 1. 6'd26 <= ratio_preq[3:0] + adpt_eq[5:0] + ratio_post1q[4:0] <= 6'd48
 * 2. 4'b0000 <= ratio_preq[3:0] <= 4'b1000
 * 3. 5'b0_0000 <= ratio_post1q[4:0] <= 5'b1_0000
 * 4. 6'b01_1010 <= adpt_eq[5:0] <= 6'b11_0000
 * 5. ratio_post1q[4:0] >= ratio_preq[3:0]
 * 6. (adpt_eq[5:0] + ratio_preq[3:0] + ratio_post1q[4:0]) /
 *    (adpt_eq[5:0] - ratio_preq[3:0] - ratio_post1q[4:0]) < 4.25
 */
static bool lynx_check_tx_hw_restrictions(struct phy *phy, s32 ratio_post1q,
					  s32 adapt_eq, s32 ratio_preq)
{
	struct device *dev = &phy->dev;

	/* Basic HW restrictions first.
	 * 2. 4'b0000 <= ratio_preq[3:0] <= 4'b1000
	 */
	if (ratio_preq > PRE_COE_MIN) {
		dev_dbg(dev, "RATIO_PREQ(%d) would exceed PRE_COE_MIN(%d)\n",
			ratio_preq, PRE_COE_MIN);
		return false;
	}

	if (ratio_preq < PRE_COE_MAX) {
		dev_dbg(dev, "RATIO_PREQ(%d) would go below PRE_COE_MAX(%d)\n",
			ratio_preq, PRE_COE_MAX);
		return false;
	}

	/* 3. 5'b0_0000 <= ratio_post1q[4:0] <= 5'b1_0000 */
	if (ratio_post1q > POST_COE_MIN) {
		dev_dbg(dev, "RATIO_POST1Q(%d) would exceed POST_COE_MIN(%d)\n",
			ratio_post1q, POST_COE_MIN);
		return false;
	}

	if (ratio_post1q < POST_COE_MAX) {
		dev_dbg(dev, "RATIO_POST1Q(%d) would go below POST_COE_MAX(%d)\n",
			ratio_post1q, POST_COE_MAX);
		return false;
	}

	/* 4. 6'b01_1010 <= adpt_eq[5:0] <= 6'b11_0000 */
	if (adapt_eq < ZERO_COE_MIN) {
		dev_dbg(dev, "ADAPT_EQ(%d) would go below ZERO_COE_MIN(%d)\n",
			adapt_eq, ZERO_COE_MIN);
		return false;
	}

	if (adapt_eq > ZERO_COE_MAX) {
		dev_dbg(dev, "ADAPT_EQ(%d) would exceed ZERO_COE_MAX(%d)\n",
			adapt_eq, ZERO_COE_MAX);
		return false;
	}

	/* 5. ratio_post1q[4:0] >= ratio_preq[3:0] */
	if (ratio_post1q < ratio_preq) {
		dev_dbg(dev, "RATIO_POST1Q(%d) would exceed RATIO_PREQ(%d)\n",
			ratio_post1q, ratio_preq);
		return false;
	}

	/* Additional HW restrictions.
	 * 1. 6'd26 <= ratio_preq[3:0] + adpt_eq[5:0] + ratio_post1q[4:0] <= 6'd48
	 */
	if ((ratio_preq + ratio_post1q + adapt_eq) < 26) {
		dev_dbg(dev, "RATIO_PREQ(%d) + RATIO_POST1Q(%d) + ADAPT_EQ(%d) would go below 26\n",
			ratio_preq, ratio_post1q, adapt_eq);
		return false;
	}

	if ((ratio_preq + ratio_post1q + adapt_eq) > 48) {
		dev_dbg(dev, "RATIO_PREQ(%d) + RATIO_POST1Q(%d) + ADAPT_EQ(%d) would exceed 48\n",
			ratio_preq, ratio_post1q, adapt_eq);
		return false;
	}

	/* 6. (adpt_eq[5:0] + ratio_preq[3:0] + ratio_post1q[4:0] ) /
	 *    (adpt_eq[5:0] - ratio_preq[3:0] - ratio_post1q[4:0] ) < 4.25 = 17/4
	 */
	if (((ratio_post1q + adapt_eq + ratio_preq) * 4) >=
	    ((adapt_eq - ratio_post1q - ratio_preq) * 17)) {
		dev_dbg(dev, "Ratio between RATIO_PREQ(%d), RATIO_POST1Q(%d) and ADAPT_EQ(%d) exceeds 17/4\n",
			ratio_preq, ratio_post1q, adapt_eq);
		return false;
	}

	return true;
}

static enum coef_status
lynx_xgkr_inc_dec_one(struct lynx_xgkr_algorithm *algorithm,
		      enum coef_field field, enum coef_update request)
{
	struct lynx_xgkr_local_tx_status *lts = &algorithm->lts;
	s32 ld_coe[COEF_FIELD_MAX], step[COEF_FIELD_MAX];
	struct lynx_xgkr_tx_eq new_tx_eq = {};
	struct phy *phy = algorithm->phy;
	bool passes;

	ld_coe[COEF_FIELD_COP1] = lts->tuned_tx_eq.ratio_post1q;
	ld_coe[COEF_FIELD_COZ] = lts->tuned_tx_eq.adapt_eq;
	ld_coe[COEF_FIELD_COM1] = lts->tuned_tx_eq.ratio_preq;

	step[COEF_FIELD_COP1] = -1;
	step[COEF_FIELD_COZ] = +1;
	step[COEF_FIELD_COM1] = -1;

	/* IEEE 802.3 72.6.10.2.5 Coefficient update process
	 * Upon execution of a received increment or decrement request,
	 * the status is reported as updated, maximum, or minimum.
	 */
	switch (request) {
	case COEF_UPD_INC:
		ld_coe[field] += step[field];
		break;
	case COEF_UPD_DEC:
		ld_coe[field] -= step[field];
		break;
	case COEF_UPD_HOLD:
		return COEF_STAT_NOT_UPDATED;
	default:
		WARN_ON(1);
		return COEF_STAT_NOT_UPDATED;
	}

	passes = lynx_check_tx_hw_restrictions(phy, ld_coe[COEF_FIELD_COP1],
					       ld_coe[COEF_FIELD_COZ],
					       ld_coe[COEF_FIELD_COM1]);
	if (!passes) {
		if (request == COEF_UPD_DEC)
			return COEF_STAT_MIN;

		/* implicitly COEF_UPD_INC */
		return COEF_STAT_MAX;
	}

	/* accept new tx_eq */
	new_tx_eq.ratio_preq = ld_coe[COEF_FIELD_COM1];
	new_tx_eq.ratio_post1q = ld_coe[COEF_FIELD_COP1];
	new_tx_eq.adapt_eq = ld_coe[COEF_FIELD_COZ];
	new_tx_eq.amp_reduction = algorithm->default_tx_eq.amp_reduction;
	lynx_tune_tx_eq(algorithm, &new_tx_eq);

	return COEF_STAT_UPDATED;
}

static int lynx_xgkr_inc_dec_coef(struct lynx_xgkr_algorithm *algorithm,
				  const struct c72_coef_update *update,
				  struct c72_coef_status *status)
{
	status->cop1 = lynx_xgkr_inc_dec_one(algorithm, COEF_FIELD_COP1,
					     update->cop1);
	status->coz = lynx_xgkr_inc_dec_one(algorithm, COEF_FIELD_COZ,
					    update->coz);
	status->com1 = lynx_xgkr_inc_dec_one(algorithm, COEF_FIELD_COM1,
					     update->com1);

	return 0;
}

static int lynx_xgkr_train_local_tx(struct lynx_xgkr_algorithm *algorithm,
				    struct c72_phy_configure_local_tx *local_tx)
{
	const struct c72_coef_update *update = &local_tx->update;
	struct lynx_xgkr_local_tx_status *lts = &algorithm->lts;
	struct c72_coef_status *status = &local_tx->status;

	lts->num_steps++;

	if (update->preset)
		return lynx_xgkr_preset(algorithm, status);
	if (update->init)
		return lynx_xgkr_init(algorithm, status);

	return lynx_xgkr_inc_dec_coef(algorithm, update, status);
}

static int lynx_xgkr_lt_done(struct lynx_xgkr_algorithm *algorithm)
{
	struct lynx_xgkr_remote_tx_status *rts = &algorithm->rts;
	struct lynx_xgkr_local_tx_status *lts = &algorithm->lts;
	struct phy *phy = algorithm->phy;

	dev_info(&phy->dev, "Link trained after %d local and %d remote steps, TX equalization: RATIO_PREQ = %d, RATIO_PST1Q = %d, ADPT_EQ = %d\n",
		 lts->num_steps, rts->num_steps, lts->tuned_tx_eq.ratio_preq,
		 lts->tuned_tx_eq.ratio_post1q, lts->tuned_tx_eq.adapt_eq);

	memset(&algorithm->rts, 0, sizeof(algorithm->rts));

	return 0;
}

int lynx_xgkr_algorithm_configure(struct lynx_xgkr_algorithm *algorithm,
				  struct phy_configure_opts_ethernet *opts)
{
	switch (opts->type) {
	case C72_LOCAL_TX:
		return lynx_xgkr_train_local_tx(algorithm, &opts->local_tx);
	case C72_REMOTE_TX:
		return lynx_xgkr_train_remote_tx(algorithm, &opts->remote_tx);
	case C72_LT_DONE:
		return lynx_xgkr_lt_done(algorithm);
	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL_GPL(lynx_xgkr_algorithm_configure);

struct lynx_xgkr_algorithm *
lynx_xgkr_algorithm_create(struct phy *phy,
			   const struct lynx_xgkr_algorithm_ops *ops)
{
	struct lynx_xgkr_algorithm *algorithm;

	algorithm = devm_kzalloc(&phy->dev, sizeof(*algorithm), GFP_KERNEL);
	if (!algorithm)
		return NULL;

	algorithm->phy = phy;
	algorithm->ops = ops;
	lynx_xgkr_read_default_tx_eq(algorithm);

	return algorithm;
}
EXPORT_SYMBOL_GPL(lynx_xgkr_algorithm_create);

void lynx_xgkr_algorithm_destroy(struct lynx_xgkr_algorithm *algorithm)
{
	struct phy *phy = algorithm->phy;

	devm_kfree(&phy->dev, algorithm);
}
EXPORT_SYMBOL_GPL(lynx_xgkr_algorithm_destroy);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Lynx SerDes link training algorithm for copper backplanes");
