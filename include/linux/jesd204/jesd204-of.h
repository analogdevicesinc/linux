/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * The JESD204 framework OF helpers
 *
 * Copyright (c) 2020 Analog Devices Inc.
 */
#ifndef _JESD204_OF_H_
#define _JESD204_OF_H_

#ifndef JESD204_OF_PREFIX
#define JESD204_OF_PREFIX	"jesd204-"
#endif

/**
 * Note: usually macros should not define control follows, but other methods
 * can cause insanity.
 */

#define _JESD204_LNK_READ_PROP(dev, np, sprop, link, field, alt, dflt)	\
{									\
	int ret;							\
	u32 tmp;							\
									\
	ret = of_property_read_u32(np, JESD204_OF_PREFIX sprop, &tmp);	\
	if (ret) {							\
		if (dflt < 0) {						\
			dev_err(dev, "Missing DT prop '%s' @ line %d ",	\
				JESD204_OF_PREFIX sprop, __LINE__);	\
			return -EINVAL;					\
		}							\
		tmp = dflt;						\
	}								\
	(link)->field = tmp;						\
	if (alt)							\
		*(alt) = tmp;						\
}

#define JESD204_LNK_READ_DEVICE_ID(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "device-id", link, device_id, alt, dflt)

#define JESD204_LNK_READ_BANK_ID(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "bank-id", link, bank_id, alt, dflt)

#define JESD204_LNK_READ_NUM_LANES(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "lanes-per-device", link, num_lanes, alt, dflt)

#define JESD204_LNK_READ_NUM_CONVERTERS(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "converters-per-device", link, num_converters, alt, dflt)

#define JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "octets-per-frame", link, octets_per_frame, alt, dflt)

#define JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "frames-per-multiframe", link, frames_per_multiframe, alt, dflt)

#define JESD204_LNK_READ_NUM_OF_MULTIBLOCKS_IN_EMB(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "num-multiblocks-in-emb", link, num_of_multiblocks_in_emb, alt, dflt)

#define JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "bits-per-sample", link, bits_per_sample, alt, dflt)

#define JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "converter-resolution", link, converter_resolution, alt, dflt)

#define JESD204_LNK_READ_SCRAMBLING(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "scrambling", link, scrambling, alt, dflt)

#define JESD204_LNK_READ_ENCODING(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "encoding", link, jesd_encoder, alt, dflt)

#define JESD204_LNK_READ_HIGH_DENSITY(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "high-density", link, high_density, alt, dflt)

#define JESD204_LNK_READ_VERSION(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "version", link, jesd_version, alt, dflt)

#define JESD204_LNK_READ_SUBCLASS(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "subclass", link, subclass, alt, dflt)

#define JESD204_LNK_READ_CTRL_WORDS_PER_FRAME_CLK(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "control-words-per-frame-clk", link, ctrl_words_per_frame_clk, alt, dflt)

#define JESD204_LNK_READ_CTRL_BITS_PER_SAMPLE(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "control-bits-per-sample", link, ctrl_bits_per_sample, alt, dflt)

#define JESD204_LNK_READ_SAMPLES_PER_CONVERTER_PER_FRAME(dev, np, link, alt, dflt)		\
	_JESD204_LNK_READ_PROP(dev, np, "samples-per-converter-per-frame", link, samples_per_conv_frame, alt, dflt)

#define JESD204_LNK_READ_DAC_ADJ_RESOLUTION_STEPS(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "dac-adj-resolution-steps", link, dac_adj_resolution_steps, alt, dflt)

#define JESD204_LNK_READ_DAC_ADJ_DIRECTION(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "dac-adj-direction", link, dac_adj_direction, alt, dflt)

#define JESD204_LNK_READ_DAC_ADJ_PHASE_ADJ(dev, np, link, alt, dflt)	\
	_JESD204_LNK_READ_PROP(dev, np, "dac-phase-adjust", link, dac_phase_adj, alt, dflt)

#endif /* _JESD204_OF_H_ */
