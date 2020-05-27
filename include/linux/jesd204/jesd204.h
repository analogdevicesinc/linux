/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * The JESD204 framework
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */
#ifndef _JESD204_H_
#define _JESD204_H_

struct jesd204_dev;

enum jesd204_subclass {
	JESD204_SUBCLASS_0,
	JESD204_SUBCLASS_1,
	JESD204_SUBCLASS_2,
};

enum jesd204_version {
	JESD204_VERSION_A,
	JESD204_VERSION_B,
	JESD204_VERSION_C,
};

/* JESD204C Supported encoding scheme */
enum jesd204_enc { /* FIXME: unify with link layer defines */
	JESD204_ENC_8B10B,
	JESD204_ENC_64B66B,
	JESD204_ENC_64B80B,
};

enum jesd204_sysref_mode {
	JESD204_SYSREF_DISABLED,
	JESD204_SYSREF_CONTINUOUS,
	JESD204_SYSREF_ONESHOT,
};

enum jesd204_state_change_result {
	JESD204_STATE_CHANGE_ERROR = -1,
	JESD204_STATE_CHANGE_DEFER = 0,
	JESD204_STATE_CHANGE_DONE,
};

#define JESD204_LINKS_ALL		((unsigned int)(-1))

/** struct jesd204_sysref - JESD204 configuration for SYSREF
 * @mode			SYSREF mode (see @jesd204_sysref_mode)
 * @capture_falling_edge	true if it should capture falling edge
 * @valid_falling_edge		true if falling edge should be valid
 */
struct jesd204_sysref {
	enum jesd204_sysref_mode	mode;
	u8				capture_falling_edge;
	u8				valid_falling_edge;
};

/**
 * struct jesd204_link - JESD204 link configuration settings
 * @link_id			JESD204 link ID provided via DT configuration
 * @is_transmit			true if this link is transmit (digital to analog)
 * @sample_rate			sample rate for the link
 * @num_lanes			number of JESD204 lanes (L)
 * @num_converters		number of converters per link (M)
 * @octets_per_frame		number of octets per frame (F)
 * @frames_per_multiframe	number of frames per frame (K)
 * @num_of_multiblocks_in_emb	number of multiblocks in extended multiblock (E) (JESD204C)
 * @bits_per_sample		number of bits per sample (N')
 * @converter_resolution	converter resolution (N)
 * @jesd_version		JESD204 version (A, B or C) (JESDV)
 * @jesd_encoder		JESD204C encoder (8B10B, 64B66B, 64B80B)
 * @subclass			JESD204 subclass (0,1 or 2) (SUBCLASSV)
 * @did				device ID (DID)
 * @bid				bank ID (BID)
 * @scrambling			true if scrambling enabled (SCR)
 * @high_density		true if high-density format is used (HD)
 * @ctrl_words_per_frame_clk	number of control words per frame clock
 *				period (CF)
 * @ctrl_bits_per_sample	number of control bits per sample (CS)
 * @samples_per_conv_frame	number of samples per converter per frame
 *				cycle (S)
 * @lane_ids			array of lane IDs (LID); note that this is an
 *				array the size of @num_lanes
 * @sysref			JESD204 sysref config, see @jesd204_sysref
 * @dac_adj_resolution_steps	number of adjustment resolution steps to adjust
 *				DAC LMFC (ADJCNT) - Subclass 2 only
 * @dac_adj_direction		direction to adjust DAC LMFC (ADJDIR)
 *				Subclass 2 only
 * @dac_phase_adj		true to do phase adjustment request to DAC
 *				Subclass 2 only
 */
struct jesd204_link {
	u32 link_id;

	u64 sample_rate;

	bool is_transmit;

	u8 num_lanes;
	u8 num_converters;
	u8 octets_per_frame;
	u8 frames_per_multiframe;
	u8 num_of_multiblocks_in_emb; /* E */

	u8 bits_per_sample;

	u8 converter_resolution;
	u8 jesd_version;
	u8 jesd_encoder;
	u8 subclass;

	u8 did;
	u8 bid;

	u8 scrambling;
	u8 high_density;

	u8 ctrl_words_per_frame_clk;
	u8 ctrl_bits_per_sample;
	u8 samples_per_conv_frame;

	u8 *lane_ids;

	struct jesd204_sysref sysref;

	/* Subclass 2 only */
	u8 dac_adj_resolution_steps;
	u8 dac_adj_direction;
	u8 dac_phase_adj;
};

typedef int (*jesd204_dev_cb)(struct jesd204_dev *jdev,
			      unsigned int link_idx);

typedef int (*jesd204_link_cb)(struct jesd204_dev *jdev,
			       unsigned int link_idx,
			       struct jesd204_link *lnk);

/**
 * struct jesd204_state_ops - JESD204 device per-state ops
 * @pre_transition_ops	ops to be called (once) before the @per_link are called for each link
 * @per_link		ops called for **each** JESD204 link individually during a transition
 * @post_transition_ops	ops to be called (once) after the @per_link are called for each link
 */
struct jesd204_state_ops {
	jesd204_dev_cb		pre_transition;
	jesd204_link_cb		per_link;
	jesd204_dev_cb		post_transition;
};

enum jesd204_dev_op {
	JESD204_OP_LINK_INIT,
	JESD204_OP_LINK_UNINIT,
	JESD204_OP_LINK_SUPPORTED,
	JESD204_OP_CLOCKS_ENABLE,
	JESD204_OP_CLOCKS_DISABLE,
	JESD204_OP_LINK_SETUP,
	JESD204_OP_LINK_ENABLE,
	JESD204_OP_LINK_RUNNING,
	JESD204_OP_LINK_DISABLE,
	__JESD204_MAX_OPS,
};

/**
 * struct jesd204_dev_data - JESD204 device initialization data
 * @state_ops		ops for each state transition of type @struct jesd204_state_ops
 * @sizeof_priv		amount of data to allocate for private information
 * @links		JESD204 initial link configuration
 * @num_links		number of JESD204 links
 */
struct jesd204_dev_data {
	struct jesd204_state_ops		state_ops[__JESD204_MAX_OPS];
	size_t					sizeof_priv;
	const struct jesd204_link		*links;
	unsigned int				num_links;
};

#if IS_ENABLED(CONFIG_JESD204)

struct jesd204_dev *jesd204_dev_register(struct device *dev,
					 const struct jesd204_dev_data *init);
struct jesd204_dev *devm_jesd204_dev_register(struct device *dev,
					      const struct jesd204_dev_data *i);

void jesd204_dev_unregister(struct jesd204_dev *jdev);
void devm_jesd204_unregister(struct device *dev, struct jesd204_dev *jdev);

struct device *jesd204_dev_to_device(struct jesd204_dev *jdev);
struct jesd204_dev *jesd204_dev_from_device(struct device *dev);

void *jesd204_dev_priv(struct jesd204_dev *jdev);

int jesd204_link_get_lmfc_lemc_rate(struct jesd204_link *lnk,
				    unsigned long *rate_hz);
int jesd204_link_get_rate_khz(struct jesd204_link *lnk,
				 unsigned long *lane_rate_khz);
int jesd204_link_get_rate(struct jesd204_link *lnk, u64 *lane_rate_hz);
int jesd204_link_get_device_clock(struct jesd204_link *lnk,
				  unsigned long *device_clock);

#else /* !IS_ENABLED(CONFIG_JESD204) */

static inline struct jesd204_dev *jesd204_dev_register(
		struct device *dev, const struct jesd204_dev_data *init)
{
	return NULL;
}

static inline void jesd204_dev_unregister(struct jesd204_dev *jdev) {}

static inline struct jesd204_dev *devm_jesd204_dev_register(
		struct device *dev, const struct jesd204_dev_data *init)
{
	return NULL;
}

static inline void devm_jesd204_unregister(struct device *dev,
	       struct jesd204_dev *jdev) {}

static inline struct device *jesd204_dev_to_device(struct jesd204_dev *jdev)
{
	return NULL;
}

static inline struct jesd204_dev *jesd204_dev_from_device(struct device *dev)
{
	return NULL;
}

static inline void *jesd204_dev_priv(struct jesd204_dev *jdev)
{
	return NULL;
}

static inline int jesd204_link_get_lmfc_lemc_rate(struct jesd204_link *lnk,
						  unsigned long *rate_hz)
{
	return -ENOTSUPP;
}

static inline int jesd204_link_get_rate_khz(struct jesd204_link *lnk,
					    unsigned long *lane_rate_khz);
{
	return -ENOTSUPP;
}

static inline int jesd204_link_get_rate(struct jesd204_link *lnk,
					u64 *lane_rate_hz)
{
	return -ENOTSUPP;
}

static inline int jesd204_link_get_device_clock(struct jesd204_link *lnk,
						unsigned long *device_clock)
{
	return -ENOTSUPP;
}

#endif /* IS_ENABLED(CONFIG_JESD204) */

#endif
