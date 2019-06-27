/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * The JESD204 framework
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */
#ifndef _JESD204_H_
#define _JESD204_H_

struct jesd204_dev;

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

typedef int (*jesd204_cb)(struct jesd204_dev *jdev);

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
 * @sample_rate			sample rate for the link
 * @num_lanes			number of JESD204 lanes (L)
 * @num_converters		number of converters per link (M)
 * @octets_per_frame		number of octets per frame (F)
 * @frames_per_multiframe	number of frames per frame (K)
 * @bits_per_sample		number of bits per sample (N')
 * @converter_resolution	converter resolution (N)
 * @jesd_version		JESD204 version (A, B or C) (JESDV)
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
	u64 sample_rate;

	u8 num_lanes;
	u8 num_converters;
	u8 octets_per_frame;
	u8 frames_per_multiframe;

	u8 bits_per_sample;

	u8 converter_resolution;
	u8 jesd_version;
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

typedef int (*jesd204_link_cb)(struct jesd204_dev *jdev,
			       unsigned int link_id,
			       struct jesd204_link *lnk);

enum jesd204_dev_op {
	JESD204_OP_LINK_INIT,
	JESD204_OP_LINK_SUPPORTED,
	JESD204_OP_CLOCKS_ENABLE,
	JESD204_OP_LINK_SETUP,
	JESD204_OP_LINK_ENABLE,
	JESD204_OP_LINK_RUNNING,
	__JESD204_MAX_OPS,
};

/**
 * struct jesd204_dev_data - JESD204 device initialization data
 * @link_ops		JESD204 operations this device passes to the framework
 *			for JESD204 link management
 * @links		JESD204 initial link configuration
 * @num_links		number of JESD204 links
 */
struct jesd204_dev_data {
	const jesd204_link_cb			link_ops[__JESD204_MAX_OPS];
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

#endif /* IS_ENABLED(CONFIG_JESD204) */

#endif
