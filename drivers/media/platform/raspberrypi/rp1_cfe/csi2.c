// SPDX-License-Identifier: GPL-2.0-only
/*
 * RP1 CSI-2 Driver
 *
 * Copyright (C) 2021 - Raspberry Pi Ltd.
 *
 */

#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>

#include <media/videobuf2-dma-contig.h>

#include "csi2.h"
#include "cfe.h"

static bool csi2_track_errors;
module_param_named(track_csi2_errors, csi2_track_errors, bool, 0);
MODULE_PARM_DESC(track_csi2_errors, "track csi-2 errors");

#define csi2_dbg_verbose(fmt, arg...)                             \
	do {                                                      \
		if (cfe_debug_verbose)                            \
			dev_dbg(csi2->v4l2_dev->dev, fmt, ##arg); \
	} while (0)
#define csi2_dbg(fmt, arg...) dev_dbg(csi2->v4l2_dev->dev, fmt, ##arg)
#define csi2_info(fmt, arg...) dev_info(csi2->v4l2_dev->dev, fmt, ##arg)
#define csi2_err(fmt, arg...) dev_err(csi2->v4l2_dev->dev, fmt, ##arg)

/* CSI2-DMA registers */
#define CSI2_STATUS		0x000
#define CSI2_QOS		0x004
#define CSI2_DISCARDS_OVERFLOW	0x008
#define CSI2_DISCARDS_INACTIVE	0x00c
#define CSI2_DISCARDS_UNMATCHED	0x010
#define CSI2_DISCARDS_LEN_LIMIT	0x014

#define CSI2_DISCARDS_AMOUNT_SHIFT	0
#define CSI2_DISCARDS_AMOUNT_MASK	GENMASK(23, 0)
#define CSI2_DISCARDS_DT_SHIFT		24
#define CSI2_DISCARDS_DT_MASK		GENMASK(29, 24)
#define CSI2_DISCARDS_VC_SHIFT		30
#define CSI2_DISCARDS_VC_MASK		GENMASK(31, 30)

#define CSI2_LLEV_PANICS	0x018
#define CSI2_ULEV_PANICS	0x01c
#define CSI2_IRQ_MASK		0x020
#define CSI2_IRQ_MASK_IRQ_OVERFLOW		BIT(0)
#define CSI2_IRQ_MASK_IRQ_DISCARD_OVERFLOW	BIT(1)
#define CSI2_IRQ_MASK_IRQ_DISCARD_LENGTH_LIMIT	BIT(2)
#define CSI2_IRQ_MASK_IRQ_DISCARD_UNMATCHED	BIT(3)
#define CSI2_IRQ_MASK_IRQ_DISCARD_INACTIVE	BIT(4)
#define CSI2_IRQ_MASK_IRQ_ALL                                              \
	(CSI2_IRQ_MASK_IRQ_OVERFLOW | CSI2_IRQ_MASK_IRQ_DISCARD_OVERFLOW | \
	 CSI2_IRQ_MASK_IRQ_DISCARD_LENGTH_LIMIT |                          \
	 CSI2_IRQ_MASK_IRQ_DISCARD_UNMATCHED |                             \
	 CSI2_IRQ_MASK_IRQ_DISCARD_INACTIVE)

#define CSI2_CTRL		0x024
#define CSI2_CH_CTRL(x)		((x) * 0x40 + 0x28)
#define CSI2_CH_ADDR0(x)	((x) * 0x40 + 0x2c)
#define CSI2_CH_ADDR1(x)	((x) * 0x40 + 0x3c)
#define CSI2_CH_STRIDE(x)	((x) * 0x40 + 0x30)
#define CSI2_CH_LENGTH(x)	((x) * 0x40 + 0x34)
#define CSI2_CH_DEBUG(x)	((x) * 0x40 + 0x38)
#define CSI2_CH_FRAME_SIZE(x)	((x) * 0x40 + 0x40)
#define CSI2_CH_COMP_CTRL(x)	((x) * 0x40 + 0x44)
#define CSI2_CH_FE_FRAME_ID(x)	((x) * 0x40 + 0x48)

/* CSI2_STATUS */
#define IRQ_FS(x)		(BIT(0) << (x))
#define IRQ_FE(x)		(BIT(4) << (x))
#define IRQ_FE_ACK(x)		(BIT(8) << (x))
#define IRQ_LE(x)		(BIT(12) << (x))
#define IRQ_LE_ACK(x)		(BIT(16) << (x))
#define IRQ_CH_MASK(x)		(IRQ_FS(x) | IRQ_FE(x) | IRQ_FE_ACK(x) | IRQ_LE(x) | IRQ_LE_ACK(x))
#define IRQ_OVERFLOW		BIT(20)
#define IRQ_DISCARD_OVERFLOW	BIT(21)
#define IRQ_DISCARD_LEN_LIMIT	BIT(22)
#define IRQ_DISCARD_UNMATCHED	BIT(23)
#define IRQ_DISCARD_INACTIVE	BIT(24)

/* CSI2_CTRL */
#define EOP_IS_EOL		BIT(0)

/* CSI2_CH_CTRL */
#define DMA_EN			BIT(0)
#define FORCE			BIT(3)
#define AUTO_ARM		BIT(4)
#define IRQ_EN_FS		BIT(13)
#define IRQ_EN_FE		BIT(14)
#define IRQ_EN_FE_ACK		BIT(15)
#define IRQ_EN_LE		BIT(16)
#define IRQ_EN_LE_ACK		BIT(17)
#define FLUSH_FE		BIT(28)
#define PACK_LINE		BIT(29)
#define PACK_BYTES		BIT(30)
#define CH_MODE_MASK		GENMASK(2, 1)
#define VC_MASK			GENMASK(6, 5)
#define DT_MASK			GENMASK(12, 7)
#define LC_MASK			GENMASK(27, 18)

/* CHx_COMPRESSION_CONTROL */
#define COMP_OFFSET_MASK	GENMASK(15, 0)
#define COMP_SHIFT_MASK		GENMASK(19, 16)
#define COMP_MODE_MASK		GENMASK(25, 24)

static inline u32 csi2_reg_read(struct csi2_device *csi2, u32 offset)
{
	return readl(csi2->base + offset);
}

static inline void csi2_reg_write(struct csi2_device *csi2, u32 offset, u32 val)
{
	writel(val, csi2->base + offset);
	csi2_dbg_verbose("csi2: write 0x%04x -> 0x%03x\n", val, offset);
}

static inline void set_field(u32 *valp, u32 field, u32 mask)
{
	u32 val = *valp;

	val &= ~mask;
	val |= (field << __ffs(mask)) & mask;
	*valp = val;
}

static int csi2_regs_show(struct seq_file *s, void *data)
{
	struct csi2_device *csi2 = s->private;
	unsigned int i;
	int ret;

	ret = pm_runtime_resume_and_get(csi2->v4l2_dev->dev);
	if (ret)
		return ret;

#define DUMP(reg) seq_printf(s, #reg " \t0x%08x\n", csi2_reg_read(csi2, reg))
#define DUMP_CH(idx, reg) seq_printf(s, #reg "(%u) \t0x%08x\n", idx, csi2_reg_read(csi2, reg(idx)))

	DUMP(CSI2_STATUS);
	DUMP(CSI2_DISCARDS_OVERFLOW);
	DUMP(CSI2_DISCARDS_INACTIVE);
	DUMP(CSI2_DISCARDS_UNMATCHED);
	DUMP(CSI2_DISCARDS_LEN_LIMIT);
	DUMP(CSI2_LLEV_PANICS);
	DUMP(CSI2_ULEV_PANICS);
	DUMP(CSI2_IRQ_MASK);
	DUMP(CSI2_CTRL);

	for (i = 0; i < CSI2_NUM_CHANNELS; ++i) {
		DUMP_CH(i, CSI2_CH_CTRL);
		DUMP_CH(i, CSI2_CH_ADDR0);
		DUMP_CH(i, CSI2_CH_ADDR1);
		DUMP_CH(i, CSI2_CH_STRIDE);
		DUMP_CH(i, CSI2_CH_LENGTH);
		DUMP_CH(i, CSI2_CH_DEBUG);
		DUMP_CH(i, CSI2_CH_FRAME_SIZE);
		DUMP_CH(i, CSI2_CH_COMP_CTRL);
		DUMP_CH(i, CSI2_CH_FE_FRAME_ID);
	}

#undef DUMP
#undef DUMP_CH

	pm_runtime_put(csi2->v4l2_dev->dev);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(csi2_regs);

static int csi2_errors_show(struct seq_file *s, void *data)
{
	struct csi2_device *csi2 = s->private;
	unsigned long flags;
	u32 discards_table[DISCARDS_TABLE_NUM_VCS][DISCARDS_TABLE_NUM_ENTRIES];
	u32 discards_dt_table[DISCARDS_TABLE_NUM_ENTRIES];
	u32 overflows;

	spin_lock_irqsave(&csi2->errors_lock, flags);

	memcpy(discards_table, csi2->discards_table, sizeof(discards_table));
	memcpy(discards_dt_table, csi2->discards_dt_table,
	       sizeof(discards_dt_table));
	overflows = csi2->overflows;

	csi2->overflows = 0;
	memset(csi2->discards_table, 0, sizeof(discards_table));
	memset(csi2->discards_dt_table, 0, sizeof(discards_dt_table));

	spin_unlock_irqrestore(&csi2->errors_lock, flags);

	seq_printf(s, "Overflows %u\n", overflows);
	seq_puts(s, "Discards:\n");
	seq_puts(s, "VC            OVLF        LEN  UNMATCHED   INACTIVE\n");

	for (unsigned int vc = 0; vc < DISCARDS_TABLE_NUM_VCS; ++vc) {
		seq_printf(s, "%u       %10u %10u %10u %10u\n", vc,
			   discards_table[vc][DISCARDS_TABLE_OVERFLOW],
			   discards_table[vc][DISCARDS_TABLE_LENGTH_LIMIT],
			   discards_table[vc][DISCARDS_TABLE_UNMATCHED],
			   discards_table[vc][DISCARDS_TABLE_INACTIVE]);
	}

	seq_printf(s, "Last DT %10u %10u %10u %10u\n",
		   discards_dt_table[DISCARDS_TABLE_OVERFLOW],
		   discards_dt_table[DISCARDS_TABLE_LENGTH_LIMIT],
		   discards_dt_table[DISCARDS_TABLE_UNMATCHED],
		   discards_dt_table[DISCARDS_TABLE_INACTIVE]);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(csi2_errors);

static void csi2_isr_handle_errors(struct csi2_device *csi2, u32 status)
{
	spin_lock(&csi2->errors_lock);

	if (status & IRQ_OVERFLOW)
		csi2->overflows++;

	for (unsigned int i = 0; i < DISCARDS_TABLE_NUM_ENTRIES; ++i) {
		static const u32 discard_bits[] = {
			IRQ_DISCARD_OVERFLOW,
			IRQ_DISCARD_LEN_LIMIT,
			IRQ_DISCARD_UNMATCHED,
			IRQ_DISCARD_INACTIVE,
		};
		static const u8 discard_regs[] = {
			CSI2_DISCARDS_OVERFLOW,
			CSI2_DISCARDS_LEN_LIMIT,
			CSI2_DISCARDS_UNMATCHED,
			CSI2_DISCARDS_INACTIVE,
		};
		u32 amount;
		u8 dt, vc;
		u32 v;

		if (!(status & discard_bits[i]))
			continue;

		v = csi2_reg_read(csi2, discard_regs[i]);
		csi2_reg_write(csi2, discard_regs[i], 0);

		amount = (v & CSI2_DISCARDS_AMOUNT_MASK) >>
			 CSI2_DISCARDS_AMOUNT_SHIFT;
		dt = (v & CSI2_DISCARDS_DT_MASK) >> CSI2_DISCARDS_DT_SHIFT;
		vc = (v & CSI2_DISCARDS_VC_MASK) >> CSI2_DISCARDS_VC_SHIFT;

		csi2->discards_table[vc][i] += amount;
		csi2->discards_dt_table[i] = dt;
	}

	spin_unlock(&csi2->errors_lock);
}

void csi2_isr(struct csi2_device *csi2, bool *sof, bool *eof)
{
	unsigned int i;
	u32 status;

	status = csi2_reg_read(csi2, CSI2_STATUS);
	csi2_dbg_verbose("ISR: STA: 0x%x\n", status);

	/* Write value back to clear the interrupts */
	csi2_reg_write(csi2, CSI2_STATUS, status);

	for (i = 0; i < CSI2_NUM_CHANNELS; i++) {
		u32 dbg;

		if ((status & IRQ_CH_MASK(i)) == 0)
			continue;

		dbg = csi2_reg_read(csi2, CSI2_CH_DEBUG(i));

		csi2_dbg_verbose("ISR: [%u], %s%s%s%s%s frame: %u line: %u\n",
				 i, (status & IRQ_FS(i)) ? "FS " : "",
				 (status & IRQ_FE(i)) ? "FE " : "",
				 (status & IRQ_FE_ACK(i)) ? "FE_ACK " : "",
				 (status & IRQ_LE(i)) ? "LE " : "",
				 (status & IRQ_LE_ACK(i)) ? "LE_ACK " : "",
				 dbg >> 16,
				 csi2->num_lines[i] ?
					 ((dbg & 0xffff) % csi2->num_lines[i]) :
					 0);

		sof[i] = !!(status & IRQ_FS(i));
		eof[i] = !!(status & IRQ_FE_ACK(i));
	}

	if (csi2_track_errors)
		csi2_isr_handle_errors(csi2, status);
}

void csi2_set_buffer(struct csi2_device *csi2, unsigned int channel,
		     dma_addr_t dmaaddr, unsigned int stride, unsigned int size)
{
	u64 addr = dmaaddr;
	/*
	 * ADDRESS0 must be written last as it triggers the double buffering
	 * mechanism for all buffer registers within the hardware.
	 */
	addr >>= 4;
	csi2_reg_write(csi2, CSI2_CH_LENGTH(channel), size >> 4);
	csi2_reg_write(csi2, CSI2_CH_STRIDE(channel), stride >> 4);
	csi2_reg_write(csi2, CSI2_CH_ADDR1(channel), addr >> 32);
	csi2_reg_write(csi2, CSI2_CH_ADDR0(channel), addr & 0xffffffff);
}

void csi2_set_compression(struct csi2_device *csi2, unsigned int channel,
			  enum csi2_compression_mode mode, unsigned int shift,
			  unsigned int offset)
{
	u32 compression = 0;

	set_field(&compression, COMP_OFFSET_MASK, offset);
	set_field(&compression, COMP_SHIFT_MASK, shift);
	set_field(&compression, COMP_MODE_MASK, mode);
	csi2_reg_write(csi2, CSI2_CH_COMP_CTRL(channel), compression);
}

static int csi2_get_vc_dt_fallback(struct csi2_device *csi2,
				   unsigned int channel, u8 *vc, u8 *dt)
{
	struct v4l2_subdev *sd = &csi2->sd;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	const struct cfe_fmt *cfe_fmt;

	state = v4l2_subdev_get_locked_active_state(sd);

	/* Without Streams API, the channel number matches the sink pad */
	fmt = v4l2_subdev_state_get_format(state, channel);
	if (!fmt)
		return -EINVAL;

	cfe_fmt = find_format_by_code(fmt->code);
	if (!cfe_fmt)
		return -EINVAL;

	*vc = 0;
	*dt = cfe_fmt->csi_dt;

	return 0;
}

static int csi2_get_vc_dt(struct csi2_device *csi2, unsigned int channel,
			  u8 *vc, u8 *dt)
{
	struct v4l2_mbus_frame_desc remote_desc;
	const struct media_pad *remote_pad;
	struct v4l2_subdev *source_sd;
	int ret;

	/* Without Streams API, the channel number matches the sink pad */
	remote_pad = media_pad_remote_pad_first(&csi2->pad[channel]);
	if (!remote_pad)
		return -EPIPE;

	source_sd = media_entity_to_v4l2_subdev(remote_pad->entity);

	ret = v4l2_subdev_call(source_sd, pad, get_frame_desc,
			       remote_pad->index, &remote_desc);
	if (ret == -ENOIOCTLCMD) {
		csi2_dbg("source does not support get_frame_desc, use fallback\n");
		return csi2_get_vc_dt_fallback(csi2, channel, vc, dt);
	} else if (ret) {
		csi2_err("Failed to get frame descriptor\n");
		return ret;
	}

	if (remote_desc.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2) {
		csi2_err("Frame descriptor does not describe CSI-2 link");
		return -EINVAL;
	}

	if (remote_desc.num_entries != 1) {
		csi2_err("Frame descriptor does not have a single entry");
		return -EINVAL;
	}

	*vc = remote_desc.entry[0].bus.csi2.vc;
	*dt = remote_desc.entry[0].bus.csi2.dt;

	return 0;
}

void csi2_start_channel(struct csi2_device *csi2, unsigned int channel,
			enum csi2_mode mode, bool auto_arm,
			bool pack_bytes, unsigned int width,
			unsigned int height)
{
	u32 ctrl;
	int ret;
	u8 vc, dt;

	ret = csi2_get_vc_dt(csi2, channel, &vc, &dt);
	if (ret)
		return;

	csi2_dbg("%s [%u]\n", __func__, channel);

	csi2_reg_write(csi2, CSI2_CH_CTRL(channel), 0);
	csi2_reg_write(csi2, CSI2_CH_DEBUG(channel), 0);
	csi2_reg_write(csi2, CSI2_STATUS, IRQ_CH_MASK(channel));

	/* Enable channel and FS/FE interrupts. */
	ctrl = DMA_EN | IRQ_EN_FS | IRQ_EN_FE_ACK | PACK_LINE;
	/* PACK_BYTES ensures no striding for embedded data. */
	if (pack_bytes)
		ctrl |= PACK_BYTES;

	if (auto_arm)
		ctrl |= AUTO_ARM;

	if (width && height) {
		set_field(&ctrl, mode, CH_MODE_MASK);
		csi2_reg_write(csi2, CSI2_CH_FRAME_SIZE(channel),
			       (height << 16) | width);
	} else {
		set_field(&ctrl, 0x0, CH_MODE_MASK);
		csi2_reg_write(csi2, CSI2_CH_FRAME_SIZE(channel), 0);
	}

	set_field(&ctrl, vc, VC_MASK);
	set_field(&ctrl, dt, DT_MASK);
	csi2_reg_write(csi2, CSI2_CH_CTRL(channel), ctrl);
	csi2->num_lines[channel] = height;
}

void csi2_stop_channel(struct csi2_device *csi2, unsigned int channel)
{
	csi2_dbg("%s [%u]\n", __func__, channel);

	/* Channel disable.  Use FORCE to allow stopping mid-frame. */
	csi2_reg_write(csi2, CSI2_CH_CTRL(channel), FORCE);
	/* Latch the above change by writing to the ADDR0 register. */
	csi2_reg_write(csi2, CSI2_CH_ADDR0(channel), 0);
	/* Write this again, the HW needs it! */
	csi2_reg_write(csi2, CSI2_CH_ADDR0(channel), 0);
}

void csi2_open_rx(struct csi2_device *csi2)
{
	csi2_reg_write(csi2, CSI2_IRQ_MASK,
		       csi2_track_errors ? CSI2_IRQ_MASK_IRQ_ALL : 0);

	dphy_start(&csi2->dphy);

	csi2_reg_write(csi2, CSI2_CTRL,
		       csi2->multipacket_line ? 0 : EOP_IS_EOL);
}

void csi2_close_rx(struct csi2_device *csi2)
{
	dphy_stop(&csi2->dphy);

	csi2_reg_write(csi2, CSI2_IRQ_MASK, 0);
}

static int csi2_init_cfg(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;

	for (unsigned int i = 0; i < CSI2_NUM_CHANNELS; ++i) {
		const struct v4l2_mbus_framefmt *def_fmt;

		/* CSI2_CH1_EMBEDDED */
		if (i == 1)
			def_fmt = &cfe_default_meta_format;
		else
			def_fmt = &cfe_default_format;

		fmt = v4l2_subdev_state_get_format(state, i);
		*fmt = *def_fmt;

		fmt = v4l2_subdev_state_get_format(state, i + CSI2_NUM_CHANNELS);
		*fmt = *def_fmt;
	}

	return 0;
}

static int csi2_pad_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *format)
{
	if (format->pad < CSI2_NUM_CHANNELS) {
		/*
		 * Store the sink pad format and propagate it to the source pad.
		 */

		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_state_get_format(state, format->pad);
		if (!fmt)
			return -EINVAL;

		*fmt = format->format;

		fmt = v4l2_subdev_state_get_format(state,
			format->pad + CSI2_NUM_CHANNELS);
		if (!fmt)
			return -EINVAL;

		format->format.field = V4L2_FIELD_NONE;

		*fmt = format->format;
	} else {
		/*
		 * Only allow changing the source pad mbus code.
		 */

		struct v4l2_mbus_framefmt *sink_fmt, *source_fmt;
		u32 sink_code;
		u32 code;

		sink_fmt = v4l2_subdev_state_get_format(state,
			format->pad - CSI2_NUM_CHANNELS);
		if (!sink_fmt)
			return -EINVAL;

		source_fmt = v4l2_subdev_state_get_format(state, format->pad);
		if (!source_fmt)
			return -EINVAL;

		sink_code = sink_fmt->code;
		code = format->format.code;

		/*
		 * If the source code from the user does not match the code in
		 * the sink pad, check that the source code matches either the
		 * 16-bit version or the compressed version of the sink code.
		 */

		if (code != sink_code &&
		    (code == cfe_find_16bit_code(sink_code) ||
		     code == cfe_find_compressed_code(sink_code)))
			source_fmt->code = code;

		format->format.code = source_fmt->code;
	}

	return 0;
}

static const struct v4l2_subdev_internal_ops csi2_internal_ops = {
	.init_state = csi2_init_cfg,
};

static const struct v4l2_subdev_pad_ops csi2_subdev_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = csi2_pad_set_fmt,
	.link_validate = v4l2_subdev_link_validate_default,
};

static const struct media_entity_operations csi2_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_ops csi2_subdev_ops = {
	.pad = &csi2_subdev_pad_ops,
};

int csi2_init(struct csi2_device *csi2, struct dentry *debugfs)
{
	unsigned int i, ret;

	spin_lock_init(&csi2->errors_lock);

	csi2->dphy.dev = csi2->v4l2_dev->dev;
	dphy_probe(&csi2->dphy);

	debugfs_create_file("csi2_regs", 0444, debugfs, csi2, &csi2_regs_fops);

	if (csi2_track_errors)
		debugfs_create_file("csi2_errors", 0444, debugfs, csi2,
				    &csi2_errors_fops);

	for (i = 0; i < CSI2_NUM_CHANNELS * 2; i++)
		csi2->pad[i].flags = i < CSI2_NUM_CHANNELS ?
				     MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&csi2->sd.entity, ARRAY_SIZE(csi2->pad),
				     csi2->pad);
	if (ret)
		return ret;

	/* Initialize subdev */
	v4l2_subdev_init(&csi2->sd, &csi2_subdev_ops);
	csi2->sd.internal_ops = &csi2_internal_ops;
	csi2->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	csi2->sd.entity.ops = &csi2_entity_ops;
	csi2->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2->sd.owner = THIS_MODULE;
	snprintf(csi2->sd.name, sizeof(csi2->sd.name), "csi2");

	ret = v4l2_subdev_init_finalize(&csi2->sd);
	if (ret)
		goto err_entity_cleanup;

	ret = v4l2_device_register_subdev(csi2->v4l2_dev, &csi2->sd);
	if (ret) {
		csi2_err("Failed register csi2 subdev (%d)\n", ret);
		goto err_subdev_cleanup;
	}

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&csi2->sd);
err_entity_cleanup:
	media_entity_cleanup(&csi2->sd.entity);

	return ret;
}

void csi2_uninit(struct csi2_device *csi2)
{
	v4l2_device_unregister_subdev(&csi2->sd);
	v4l2_subdev_cleanup(&csi2->sd);
	media_entity_cleanup(&csi2->sd.entity);
}
