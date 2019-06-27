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
	bool				capture_falling_edge;
	bool				valid_falling_edge;
};

/**
 * struct jesd204_link - JESD204 link configuration settings
 * @sample_rate			sample rate for the link
 * @num_lanes			number of JESD204 lanes (L)
 * @num_converters		number of converters per link (M)
 * @frames_per_multiframe	number of frames per frame (F)
 * @bits_per_sample		number of bits per sample (N')
 * @converter_resolution	converter resolution (N)
 * @subclass			JESD204 subclass (0,1 or 2)
 * @did				device ID
 * @bid				bank ID
 * @scrambling			true if scrambling enabled
 * @lane_ids			array of lane IDs
 * @sysref			JESD204 sysref config, see @jesd204_sysref
 */
struct jesd204_link {
	u64 sample_rate;

	u8 num_lanes;
	u8 num_converters;
	u8 octets_per_frame;
	u8 frames_per_multiframe;

	u8 bits_per_sample;

	u8 converter_resolution;
	u8 subclass;

	u8 did;
	u8 bid;

	bool scrambling;

	u8 *lane_ids;

	struct jesd204_sysref sysref;
};

/**
 * struct jesd204_dev_data - JESD204 device initialization data
 * @links		JESD204 initial link configuration
 * @num_links		number of JESD204 links
 */
struct jesd204_dev_data {
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
