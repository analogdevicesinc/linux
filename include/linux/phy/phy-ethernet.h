/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 NXP
 */

#ifndef __PHY_ETHERNET_H_
#define __PHY_ETHERNET_H_

struct phy;

/**
 * enum coef_status - Status report field of one TX equalization tap
 *		      (coefficient), according to IEEE 802.3-2018 clause
 *		      72.6.10.2.4.5 Coefficient (k) status
 *
 * @COEF_STAT_NOT_UPDATED: The default state for a given tap is not_updated.
 *			   An increment or decrement request will only be acted
 *			   upon when the state of the tap is not_updated.
 * @COEF_STAT_UPDATED: The tap transitions to this state after it has
 *		       successfully responded to an update request different
 *		       from HOLD.
 * @COEF_STAT_MIN: The tap transitions to this state when it has received a DEC
 *		   request, but it has already reached its minimum limit and it
 *		   cannot fulfill the request.
 * @COEF_STAT_MAX: The tap transitions to this state when it has received an
 *		   INC request, but it has already reached its maximum limit
 *		   and it cannot fulfill the request.
 *
 * After any non-HOLD update request which results in an update of the
 * coefficient status, the update request must return to the HOLD state.
 * Then, the status field returns to NOT_UPDATED and the process can repeat.
 *
 * Example of one remote TX link training step from the perspective of the
 * local device (LD), which is a local TX link training step from the
 * perspective of the link partner (LP):
 *
 * Step                    | coef_update of LD    | coef_status of LP
 * ------------------------+----------------------+-----------------------
 * #1                      | INIT: false          | C(-1): NOT_UPDATED
 * (steady state, no       | PRESET: false        | C(0): NOT_UPDATED
 *  request pending)       | C(-1): HOLD          | C(+1): NOT_UPDATED
 *                         | C(0): HOLD           |
 *                         | C(+1): HOLD          |
 * ------------------------+----------------------+-----------------------
 * #2                      | INIT: true           | C(-1): NOT_UPDATED
 * (init request issued,   | (other fields        | C(0): NOT_UPDATED
 *  pending LP ack)        |  unchanged)          | C(+1): NOT_UPDATED
 * ------------------------+----------------------+-----------------------
 * #3                      | INIT: true           | C(-1): UPDATED
 * (LP responds to         |                      | C(0): UPDATED
 *  init request)          |                      | C(+1): UPDATED
 * ------------------------+----------------------+-----------------------
 * #4                      | INIT: false          | C(-1): UPDATED
 * (LD returns to the      |                      | C(0): UPDATED
 *  "no request" state)    |                      | C(+1): UPDATED
 * ------------------------+----------------------+-----------------------
 * #5                      | INIT: false          | C(-1): NOT_UPDATED
 * (same as #1, LP         |                      | C(0): NOT_UPDATED
 *  returns to idle state) |                      | C(+1): NOT_UPDATED
 * ------------------------+----------------------+-----------------------
 */
enum coef_status {
	COEF_STAT_NOT_UPDATED = 0,
	COEF_STAT_UPDATED = 1,
	COEF_STAT_MIN = 2,
	COEF_STAT_MAX = 3,
};

/**
 * enum coef_update - Requested update field of one TX equalization tap
 *		      (coefficient), according to IEEE 802.3-2018 clause
 *		      72.6.10.2.3.3 Coefficient (k) update
 *
 * @COEF_UPD_HOLD: The default state for a given tap is hold, which corresponds
 *		   to no change in the coefficient.
 * @COEF_UPD_INC: Request the link partner to increase the equalization of the
 *		  given tap by one step, which implies a change to the waveform
 *		  voltages within the limits defined in IEEE 802.3-2018 Table
 *		  72-7 "Transmitter output waveform requirements related to
 *		  coefficient update"
 * @COEF_UPD_DEC: See @COEF_UPD_DEC
 *
 * Coefficient increment/decrement shall not be sent in combination with
 * initialize or preset.
 */
enum coef_update {
	COEF_UPD_HOLD = 0,
	COEF_UPD_INC = 1,
	COEF_UPD_DEC = 2,
};

/**
 * struct c72_coef_update - C72 coefficient request
 *
 * @com1:	corresponds to "Coefficient (-1) update" ("minus 1")
 * @coz:	corresponds to "Coefficient (0) update" ("zero")
 * @cop1:	corresponds to "Coefficient (+1) update" ("plus 1")
 * @preset:	when set, preset coefficients are requested
 * @init:	when set, coefficient initialization is requested
 *
 * This is the input structure for a local TX link training step (or output for
 * remote TX link training), and is based on the definitions from IEEE
 * 802.3-2018 clause 72.6.10.2.3 "Coefficient update field". It carries
 * correction information from the local receiver to the link partner transmit
 * equalizer. The structure consists of preset controls, initialization
 * controls, and coefficient updates for three transmit equalizer taps.
 */
struct c72_coef_update {
	enum coef_update com1;
	enum coef_update coz;
	enum coef_update cop1;
	bool preset;
	bool init;
};

/**
 * struct c72_coef_status - response to C72 coefficient request
 *
 * @com1:	corresponds to "Coefficient (-1) status" ("minus 1")
 * @coz:	corresponds to "Coefficient (0) status" ("zero")
 * @cop1:	corresponds to "Coefficient (+1) status" ("plus 1")
 *
 * This is the output structure for a local TX link training step, and is based
 * on the definitions from IEEE 802.3-2018 clause 72.6.10.2.4 "Status report
 * field". The "Receiver ready" (bit 15 as defined by IEEE) of the status
 * report is deliberately not part of this structure, since it is logically
 * part of the remote TX link training procedure, and is decoupled from the
 * local TX link training.
 */
struct c72_coef_status {
	enum coef_status com1;
	enum coef_status coz;
	enum coef_status cop1;
};

/**
 * enum ethernet_phy_configure_type - Configuration types for an Ethernet phy
 *
 * @C72_LOCAL_TX: Execute a C72 link training step for the local transmitter.
 * @C72_REMOTE_TX: Execute a C72 link training step for the remote transmitter.
 * @C72_LT_DONE: Finalize C72 link training.
 *
 * The @C72_LOCAL_TX, @C72_REMOTE_TX and @C72_LT_DONE types apply to Ethernet
 * phys supporting media types with the IEEE 802.3 clause 72: 10GBase-KR,
 * 40GBase-KR4 etc.
 */
enum ethernet_phy_configure_type {
	C72_LOCAL_TX,
	C72_REMOTE_TX,
	C72_LT_DONE,
};

/**
 * struct c72_phy_configure_local_tx - configuration set for C72 local TX
 *				       link training
 * @update:	input structure containing a C72 coefficient update request
 *		from the link partner.
 * @status:	output structure containing the response of the local PHY to
 *		the given update request
 *
 * Adjust the TX equalization of the local PHY in response to a C72 coefficient
 * update request from the link partner. Used when @ethernet_phy_configure_type
 * is set to @C72_LOCAL_TX.
 */
struct c72_phy_configure_local_tx {
	struct c72_coef_update update;
	struct c72_coef_status status;
};

/**
 * struct c72_phy_configure_remote_tx - configuration set for C72 remote TX
 *					link training
 *
 * @rx_ready:	output boolean set by phy when it does not need any transmitter
 *		adjustments from the link partner
 * @update:	output structure containing a request to the link partner
 *		transmitter, based on information from the local receiver
 * @cb:		optional callback to see how the link partner reacted to the
 *		update request (which is echoed back unmodified). The
 *		coefficient status is only valid if there was no error
 *		during its propagation.
 * @cb_priv:	private structure for the callback @cb.
 *
 * Query the phy RX quality in order to compute a C72 coefficient update
 * request to the link partner to improve that. The phy consumer is responsible
 * for taking the computed request and transmitting it to the link partner, and
 * then calling the optional phy callback before making any other query.
 *
 * Used when @ethernet_phy_configure_type is set to @C72_REMOTE_TX.
 *
 * WARNING: the phy implementation may be stateful, i.e. the number of previous
 * requests and their received status may modify the phy's state and it might
 * influence the next computed request.
 */
struct c72_phy_configure_remote_tx {
	bool rx_ready;
	struct c72_coef_update update;
	void (*cb)(void *cb_priv, int err, struct c72_coef_update update,
		   struct c72_coef_status status);
	void *cb_priv;
};

/**
 * struct phy_configure_opts_ethernet - Ethernet PHY configuration set
 *
 * @type:	type of Ethernet phy configuration structure
 * @local_tx:	configuration state for one C72 link training step for the
 *		local transmitter, valid when @type is C72_LOCAL_TX
 * @remote_tx:	configuration state for one C72 link training step for the
 *		remote transmitter, valid when @type is C72_REMOTE_TX
 *
 * This structure is used to represent the configuration state of an Ethernet
 * PHY (of various media types).
 */
struct phy_configure_opts_ethernet {
	enum ethernet_phy_configure_type type;
	union {
		struct c72_phy_configure_local_tx local_tx;
		struct c72_phy_configure_remote_tx remote_tx;
	};
};

/**
 * coef_update_opposite - return the opposite of one C72 coefficient update
 *			  request
 *
 * @update:	original coefficient update
 *
 * Helper to transform the update request of one equalization tap into a
 * request of the same tap in the opposite direction. May be used by C72
 * phy remote TX link training algorithms.
 */
static inline enum coef_update coef_update_opposite(enum coef_update update)
{
	switch (update) {
	case COEF_UPD_INC:
		return COEF_UPD_DEC;
	case COEF_UPD_DEC:
		return COEF_UPD_INC;
	default:
		return COEF_UPD_HOLD;
	}
}

/**
 * coef_update_clamp - clamp one C72 coefficient update request
 *
 * @update:	pointer to coefficient update
 * @status:	response from link partner to a previous update request to the
 *		same tap, based on which we are clamping this one
 *
 * Helper which may be used by C72 phy remote TX link training algorithms to
 * clamp coefficient updates for a tap. When the link partner responded with
 * MAX or MIN to a previous update request for the same tap, future requests
 * are likely to result in the same response, so just transform them into HOLD,
 * which represents the lack of an update request.
 */
static inline void coef_update_clamp(enum coef_update *update,
				     enum coef_status status)
{
	if (*update == COEF_UPD_INC && status == COEF_STAT_MAX)
		*update = COEF_UPD_HOLD;
	if (*update == COEF_UPD_DEC && status == COEF_STAT_MIN)
		*update = COEF_UPD_HOLD;
}

/* Other helpers */
static inline bool coef_update_is_all_hold(const struct c72_coef_update *update)
{
	return update->coz == COEF_UPD_HOLD &&
	       update->com1 == COEF_UPD_HOLD &&
	       update->cop1 == COEF_UPD_HOLD;
}

#define C72_COEF_UPDATE_BUFSIZ 64
#define C72_COEF_STATUS_BUFSIZ 64

static inline const char *coef_update_to_string(enum coef_update coef)
{
	switch (coef) {
	case COEF_UPD_HOLD:
		return "HOLD";
	case COEF_UPD_INC:
		return "INC";
	case COEF_UPD_DEC:
		return "DEC";
	default:
		return "unknown";
	}
}

static inline const char *coef_status_to_string(enum coef_status coef)
{
	switch (coef) {
	case COEF_STAT_NOT_UPDATED:
		return "NOT_UPDATED";
	case COEF_STAT_UPDATED:
		return "UPDATED";
	case COEF_STAT_MIN:
		return "MIN";
	case COEF_STAT_MAX:
		return "MAX";
	default:
		return "unknown";
	}
}

static inline void c72_coef_update_print(const struct c72_coef_update *update,
					 char buf[C72_COEF_UPDATE_BUFSIZ])
{
	sprintf(buf, "INIT %d, PRESET %d, C(-1) %s, C(0) %s, C(+1) %s",
		update->init, update->preset,
		coef_update_to_string(update->com1),
		coef_update_to_string(update->coz),
		coef_update_to_string(update->cop1));
}

static inline void c72_coef_status_print(const struct c72_coef_status *status,
					 char buf[C72_COEF_STATUS_BUFSIZ])
{
	sprintf(buf, "C(-1) %s, C(0) %s, C(+1) %s",
		coef_status_to_string(status->com1),
		coef_status_to_string(status->coz),
		coef_status_to_string(status->cop1));
}

#endif /* __PHY_ETHERNET_H_ */
