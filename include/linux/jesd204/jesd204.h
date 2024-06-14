/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * The JESD204 framework
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */
#ifndef _JESD204_H_
#define _JESD204_H_

#include <linux/kconfig.h>
#include <linux/kern_levels.h>
#include <dt-bindings/jesd204/device-states.h>

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
enum jesd204_encoder {
	JESD204_ENCODER_UNKNOWN,
	JESD204_ENCODER_8B10B,
	JESD204_ENCODER_64B66B,
	JESD204_ENCODER_64B80B,

	JESD204_ENCODER_MAX
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

#define JESD204_LMFC_OFFSET_UNINITIALIZED	((u16)-1)

/** struct jesd204_sysref - JESD204 parameters for SYSREF
 * @mode:			SYSREF mode (see @jesd204_sysref_mode)
 * @capture_falling_edge:	true if it should capture falling edge
 * @valid_falling_edge:		true if falling edge should be valid
 * @lmfc_offset:		offset for LMFC
 */
struct jesd204_sysref {
	enum jesd204_sysref_mode	mode;
	u8				capture_falling_edge;
	u8				valid_falling_edge;
	u16				lmfc_offset;
};

/**
 * struct jesd204_link - JESD204 link configuration settings
 * @link_id:			JESD204 link ID provided via DT configuration
 * @error:			error code for this JESD204 link
 * @is_transmit:		true if this link is transmit (digital to analog)
 * @sample_rate:		sample rate for the link
 * @sample_rate_div:		optional sample rate divider for the link
 * 					final rate = sample_rate / sample_rate_div
 * @num_lanes:			number of JESD204 lanes (L)
 * @num_converters:		number of converters per link (M)
 * @octets_per_frame:		number of octets per frame (F)
 * @frames_per_multiframe:	number of frames per frame (K)
 * @num_of_multiblocks_in_emb:	number of multiblocks in extended multiblock (E) (JESD204C)
 * @bits_per_sample:		number of bits per sample (N')
 * @converter_resolution:	converter resolution (N)
 * @jesd_version:		JESD204 version (A, B or C) (JESDV)
 * @jesd_encoder:		JESD204C encoder (8B10B, 64B66B, 64B80B)
 * @subclass:			JESD204 subclass (0,1 or 2) (SUBCLASSV)
 * @device_id:			device ID (DID)
 * @bank_id:			bank ID (BID)
 * @scrambling:			true if scrambling enabled (SCR)
 * @high_density:		true if high-density format is used (HD)
 * @ctrl_words_per_frame_clk:	number of control words per frame clock
 *				period (CF)
 * @ctrl_bits_per_sample:	number of control bits per sample (CS)
 * @samples_per_conv_frame:	number of samples per converter per frame
 *				cycle (S)
 * @lane_ids:			array of lane IDs (LID); note that this is an
 *				array the size of @num_lanes
 * @sysref:			JESD204 sysref config, see @jesd204_sysref
 * @dac_adj_resolution_steps:	number of adjustment resolution steps to adjust
 *				DAC LMFC (ADJCNT) - Subclass 2 only
 * @dac_adj_direction:		direction to adjust DAC LMFC (ADJDIR)
 *				Subclass 2 only
 * @dac_phase_adj:		true to do phase adjustment request to DAC
 *				Subclass 2 only
 */
struct jesd204_link {
	u32 link_id;
	int error;

	u64 sample_rate;
	u32 sample_rate_div;

	bool is_transmit;

	u8 num_lanes;
	u8 num_converters;
	u8 octets_per_frame;
	u16 frames_per_multiframe;
	u8 num_of_multiblocks_in_emb; /* E */

	u8 bits_per_sample;

	u8 converter_resolution;
	u8 jesd_version;
	u8 jesd_encoder;
	u8 subclass;

	u8 device_id;
	u8 bank_id;

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

enum jesd204_state_op_reason {
	JESD204_STATE_OP_REASON_INIT,
	JESD204_STATE_OP_REASON_UNINIT,
};

static inline const char *jesd204_encoder_str(enum jesd204_encoder enc)
{
	switch (enc) {
	case JESD204_ENCODER_8B10B:
		return "8b10b";
	case JESD204_ENCODER_64B66B:
		return "64b66b";
	case JESD204_ENCODER_64B80B:
		return "64b80b";
	default:
		return "unknown";
	}
}

static inline const char *jesd204_state_op_reason_str(enum jesd204_state_op_reason reason)
{
	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		return "initialization";
	case JESD204_STATE_OP_REASON_UNINIT:
		return "uninitialization";
	default:
		return "unknown";
	}
}

typedef int (*jesd204_sysref_cb)(struct jesd204_dev *jdev);

typedef int (*jesd204_dev_cb)(struct jesd204_dev *jdev,
			      enum jesd204_state_op_reason reason);

typedef int (*jesd204_link_cb)(struct jesd204_dev *jdev,
			       enum jesd204_state_op_reason,
			       struct jesd204_link *lnk);

typedef void (*jesd204_fsm_finished_cb)(struct jesd204_dev *jdev,
					const struct jesd204_link * const *links,
					unsigned int num_links);

enum jesd204_state_op_mode {
	JESD204_STATE_OP_MODE_PER_LINK,
	JESD204_STATE_OP_MODE_PER_DEVICE,
};

/**
 * struct jesd204_state_op - JESD204 device per-state op
 * @mode:		mode for this state op, depending on this @per_device or @per_link is called
 * @per_device:		op called for each JESD204 **device** during a transition
 * @per_link		op called for each JESD204 **link** individually during a transition
 * // FIXME: maybe pass 'struct jesd204_sysref' for post_state_sysref, to make this configurable? we'll see later
 * // FIXME: for now, the device should also be a top-level device, in case of multi-chip setups
 * @post_state_sysref:	true if a SYSREF should be issued after the state change
 */
struct jesd204_state_op {
	enum jesd204_state_op_mode	mode;
	jesd204_dev_cb			per_device;
	jesd204_link_cb			per_link;
	bool				post_state_sysref;
};

enum jesd204_dev_op {
	JESD204_OP_DEVICE_INIT,
	JESD204_OP_LINK_INIT,
	JESD204_OP_LINK_SUPPORTED,
	JESD204_OP_LINK_PRE_SETUP,
	JESD204_OP_CLK_SYNC_STAGE1,
	JESD204_OP_CLK_SYNC_STAGE2,
	JESD204_OP_CLK_SYNC_STAGE3,
	JESD204_OP_LINK_SETUP,
	JESD204_OP_OPT_SETUP_STAGE1,
	JESD204_OP_OPT_SETUP_STAGE2,
	JESD204_OP_OPT_SETUP_STAGE3,
	JESD204_OP_OPT_SETUP_STAGE4,
	JESD204_OP_OPT_SETUP_STAGE5,
	JESD204_OP_CLOCKS_ENABLE,
	JESD204_OP_LINK_ENABLE,
	JESD204_OP_LINK_RUNNING,
	JESD204_OP_OPT_POST_RUNNING_STAGE,

	__JESD204_MAX_OPS,
};

/**
 * struct jesd204_dev_data - JESD204 device initialization data
 * @sysref_cb:		SYSREF callback, if this device/driver supports it
 * @fsm_finished_cb:	callback for when the FSM finishes a series of transitions
 * @sizeof_priv:	amount of data to allocate for private information
 * @links:		JESD204 initial link configuration
 * @max_num_links:	maximum number of JESD204 links this device can support
 * @num_retries:	number of retries in case of error (only for top-level device)
 * @state_ops:		ops for each state transition of type @struct jesd204_state_op
 */
struct jesd204_dev_data {
	jesd204_sysref_cb			sysref_cb;
	jesd204_fsm_finished_cb			fsm_finished_cb;
	size_t					sizeof_priv;
	const struct jesd204_link		*links;
	unsigned int				max_num_links;
	unsigned int				num_retries;
	struct jesd204_state_op			state_ops[__JESD204_MAX_OPS];
};

#if IS_ENABLED(CONFIG_JESD204)

struct jesd204_dev *devm_jesd204_dev_register(struct device *dev,
					      const struct jesd204_dev_data *i);

int jesd204_get_active_links_num(struct jesd204_dev *jdev);

int jesd204_get_links_data(struct jesd204_dev *jdev,
			   struct jesd204_link ** const links,
			   const unsigned int num_links);

int jesd204_fsm_start(struct jesd204_dev *jdev, unsigned int link_idx);
void jesd204_fsm_stop(struct jesd204_dev *jdev, unsigned int link_idx);

int jesd204_fsm_resume(struct jesd204_dev *jdev, unsigned int link_idx);

void jesd204_fsm_clear_errors(struct jesd204_dev *jdev, unsigned int link_idx);

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

int jesd204_sysref_async(struct jesd204_dev *jdev);

int jesd204_sysref_async_force(struct jesd204_dev *jdev);

bool jesd204_dev_is_top(struct jesd204_dev *jdev);

bool jesd204_link_get_paused(const struct jesd204_link *lnk);

const char *jesd204_link_get_state_str(const struct jesd204_link *lnk);

void jesd204_copy_link_params(struct jesd204_link *dst,
			      const struct jesd204_link *src);

#else /* !IS_ENABLED(CONFIG_JESD204) */

static inline struct jesd204_dev *devm_jesd204_dev_register(
		struct device *dev, const struct jesd204_dev_data *init)
{
	return NULL;
}

static inline int jesd204_get_active_links_num(struct jesd204_dev *jdev)
{
	return -ENOTSUPP;
}

static inline int jesd204_get_links_data(struct jesd204_dev *jdev,
					 struct jesd204_link * const *links,
					 const unsigned int num_links)
{
	return -ENOTSUPP;
}

static inline int jesd204_fsm_start(struct jesd204_dev *jdev,
				    unsigned int link_idx)
{
	return 0;
}

static inline void jesd204_fsm_stop(struct jesd204_dev *jdev,
				    unsigned int link_idx) {}

static inline int jesd204_fsm_resume(struct jesd204_dev *jdev,
				     unsigned int link_idx)
{
	return 0;
}

static inline void jesd204_fsm_clear_errors(struct jesd204_dev *jdev,
					    unsigned int link_idx) {}

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
					    unsigned long *lane_rate_khz)
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

static inline int jesd204_sysref_async(struct jesd204_dev *jdev)
{
	return -ENOTSUPP;
}

static inline int jesd204_sysref_async_force(struct jesd204_dev *jdev)
{
	return -ENOTSUPP;
}

static inline bool jesd204_dev_is_top(struct jesd204_dev *jdev)
{
	return false;
}

static inline bool jesd204_link_get_paused(const struct jesd204_link *lnk)
{
	return false;
}

static inline const char *jesd204_link_get_state_str(const struct jesd204_link *lnk)
{
	return NULL;
}

static inline void jesd204_copy_link_params(struct jesd204_link *dst,
					    const struct jesd204_link *src) {}

#endif /* #ifdef CONFIG_JESD204 */

#if defined(CONFIG_PRINTK) && IS_ENABLED(CONFIG_JESD204)

__printf(3, 4)
void jesd204_printk(const char *level, const struct jesd204_dev *jdev,
		    const char *fmt, ...);
#else

static inline __printf(3, 4)
void jesd204_printk(const char *level, const struct jesd204_dev *jdev,
		    const char *fmt, ...)
{}

#endif /* #ifdef CONFIG_PRINTK */

#define jesd204_emerg(dev, fmt, ...)					\
	jesd204_printk(KERN_EMERG, dev, fmt, ##__VA_ARGS__)
#define jesd204_crit(dev, fmt, ...)					\
	jesd204_printk(KERN_CRIT, dev, fmt, ##__VA_ARGS__)
#define jesd204_alert(dev, fmt, ...)					\
	jesd204_printk(KERN_ALERT, dev, fmt, ##__VA_ARGS__)
#define jesd204_err(dev, fmt, ...)					\
	jesd204_printk(KERN_ERR, dev, fmt, ##__VA_ARGS__)
#define jesd204_warn(dev, fmt, ...)					\
	jesd204_printk(KERN_WARNING, dev, fmt, ##__VA_ARGS__)
#define jesd204_notice(dev, fmt, ...)					\
	jesd204_printk(KERN_NOTICE, dev, fmt, ##__VA_ARGS__)
#define jesd204_info(dev, fmt, ...)					\
	jesd204_printk(KERN_INFO, dev, fmt, ##__VA_ARGS__)

#endif
