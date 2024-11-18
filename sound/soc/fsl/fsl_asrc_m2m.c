/*
 * Freescale ASRC Memory to Memory (M2M) driver
 *
 * Copyright (C) 2014-2016 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#define FSL_ASRC_INPUTFIFO_WML	0x4
#define FSL_ASRC_OUTPUTFIFO_WML	0x2

#define DIR_STR(dir) dir == IN ? "in" : "out"

struct fsl_asrc_m2m {
	struct fsl_asrc_pair *pair;
	struct completion complete[2];
	struct dma_block dma_block[2];
	unsigned int pair_hold;
	unsigned int asrc_active;
	unsigned int sg_nodes[2];
	struct scatterlist sg[2][4];

	snd_pcm_format_t word_format[2];
	unsigned int rate[2];
	unsigned int last_period_size;
	u32 watermark[2];
	spinlock_t lock;
};

static void fsl_asrc_get_status(struct fsl_asrc_pair *pair,
				struct asrc_status_flags *flags)
{
	struct fsl_asrc *asrc = pair->asrc;
	unsigned long lock_flags;

	spin_lock_irqsave(&asrc->lock, lock_flags);

	flags->overload_error = pair->error;

	spin_unlock_irqrestore(&asrc->lock, lock_flags);
}

#define ASRC_xPUT_DMA_CALLBACK(dir) \
	((dir == IN) ? fsl_asrc_input_dma_callback : fsl_asrc_output_dma_callback)

static void fsl_asrc_input_dma_callback(void *data)
{
	struct fsl_asrc_pair *pair = (struct fsl_asrc_pair *)data;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;

	complete(&m2m->complete[IN]);
}

static void fsl_asrc_output_dma_callback(void *data)
{
	struct fsl_asrc_pair *pair = (struct fsl_asrc_pair *)data;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;

	complete(&m2m->complete[OUT]);
}

static unsigned int fsl_asrc_get_output_FIFO_size(struct fsl_asrc_pair *pair)
{
	struct fsl_asrc *asrc = pair->asrc;
	enum asrc_pair_index index = pair->index;
	u32 val;

	regmap_read(asrc->regmap, REG_ASRFST(index), &val);

	val &= ASRFSTi_OUTPUT_FIFO_MASK;

	return val >> ASRFSTi_OUTPUT_FIFO_SHIFT;
}

static void fsl_asrc_read_last_FIFO(struct fsl_asrc_pair *pair)
{
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc *asrc = pair->asrc;
	enum asrc_pair_index index = pair->index;
	struct dma_block *output = &m2m->dma_block[OUT];
	u32 i, reg, size, t_size = 0, width;
	u32 *reg32 = NULL;
	u16 *reg16 = NULL;
	u8  *reg24 = NULL;

	width = snd_pcm_format_physical_width(m2m->word_format[OUT]);

	if (width == 32)
		reg32 = output->dma_vaddr + output->length;
	else if (width == 16)
		reg16 = output->dma_vaddr + output->length;
	else
		reg24 = output->dma_vaddr + output->length;

retry:
	size = fsl_asrc_get_output_FIFO_size(pair);

	for (i = 0; i < size * pair->channels; i++) {
		regmap_read(asrc->regmap, REG_ASRDO(index), &reg);
		if (reg32) {
			*(reg32) = reg;
			reg32++;
		} else if (reg16) {
			*(reg16) = (u16)reg;
			reg16++;
		} else {
			*reg24++ = (u8)reg;
			*reg24++ = (u8)(reg >> 8);
			*reg24++ = (u8)(reg >> 16);
		}
	}
	t_size += size;

	if (size)
		goto retry;

	if (t_size > m2m->last_period_size)
		t_size = m2m->last_period_size;

	if (reg32)
		output->length += t_size * pair->channels * 4;
	else if (reg16)
		output->length += t_size * pair->channels * 2;
	else
		output->length += t_size * pair->channels * 3;
}

static int fsl_allocate_dma_buf(struct fsl_asrc_pair *pair)
{
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc *asrc = pair->asrc;
	struct dma_block *input = &m2m->dma_block[IN];
	struct dma_block *output = &m2m->dma_block[OUT];
	enum asrc_pair_index index = pair->index;

	input->dma_vaddr = kzalloc(input->length, GFP_KERNEL);
	if (!input->dma_vaddr) {
		pair_err("failed to allocate input DMA buffer\n");
		return -ENOMEM;
	}

	output->dma_vaddr = kzalloc(output->length, GFP_KERNEL);
	if (!output->dma_vaddr) {
		pair_err("failed to allocate output DMA buffer\n");
		goto exit;
	}

	return 0;

exit:
	kfree(input->dma_vaddr);
	input->dma_vaddr = NULL;
	return -ENOMEM;
}

static int fsl_asrc_dmaconfig(struct fsl_asrc_pair *pair, struct dma_chan *chan,
			      u32 dma_addr, void *buf_addr, u32 buf_len,
			      bool dir, snd_pcm_format_t word_format)
{
	struct dma_async_tx_descriptor *desc = pair->desc[dir];
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_priv *asrc_priv = asrc->private;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	unsigned int sg_nent = m2m->sg_nodes[dir];
	enum asrc_pair_index index = pair->index;
	struct scatterlist *sg = m2m->sg[dir];
	struct dma_slave_config slave_config;
	enum dma_slave_buswidth buswidth;
	enum dma_data_direction dma_dir;
	int ret, i;

	switch (snd_pcm_format_physical_width(word_format)) {
	case 8:
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case 16:
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 24:
		buswidth = DMA_SLAVE_BUSWIDTH_3_BYTES;
		break;
	case 32:
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		pair_err("invalid word width\n");
		return -EINVAL;
	}

	memset(&slave_config, 0, sizeof(slave_config));
	if (dir == IN) {
		slave_config.direction = DMA_MEM_TO_DEV;
		dma_dir = DMA_TO_DEVICE;
		slave_config.dst_addr = dma_addr;
		slave_config.dst_addr_width = buswidth;
		if (!asrc_priv->soc->use_edma)
			slave_config.dst_maxburst =
				m2m->watermark[IN] * pair->channels;
		else
			slave_config.dst_maxburst = 1;
	} else {
		slave_config.direction = DMA_DEV_TO_MEM;
		dma_dir = DMA_FROM_DEVICE;
		slave_config.src_addr = dma_addr;
		slave_config.src_addr_width = buswidth;
		if (!asrc_priv->soc->use_edma)
			slave_config.src_maxburst =
				m2m->watermark[OUT] * pair->channels;
		else
			slave_config.src_maxburst = 1;
	}

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret) {
		pair_err("failed to config dmaengine for %sput task: %d\n",
				DIR_STR(dir), ret);
		return -EINVAL;
	}

	sg_init_table(sg, sg_nent);
	switch (sg_nent) {
	case 1:
		sg_init_one(sg, buf_addr, buf_len);
		break;
	case 2:
	case 3:
	case 4:
		for (i = 0; i < (sg_nent - 1); i++)
			sg_set_buf(&sg[i], buf_addr + i * ASRC_MAX_BUFFER_SIZE,
					ASRC_MAX_BUFFER_SIZE);

		sg_set_buf(&sg[i], buf_addr + i * ASRC_MAX_BUFFER_SIZE,
				buf_len - ASRC_MAX_BUFFER_SIZE * i);
		break;
	default:
		pair_err("invalid input DMA nodes number: %d\n", sg_nent);
		return -EINVAL;
	}

	ret = dma_map_sg(&asrc->pdev->dev, sg, sg_nent, dma_dir);
	if (ret != sg_nent) {
		pair_err("failed to map DMA sg for %sput task\n", DIR_STR(dir));
		return -EINVAL;
	}

	desc = dmaengine_prep_slave_sg(chan, sg, sg_nent,
			slave_config.direction, DMA_PREP_INTERRUPT);
	if (!desc) {
		pair_err("failed to prepare dmaengine for %sput task\n",
				DIR_STR(dir));
		return -EINVAL;
	}

	pair->desc[dir] = desc;
	pair->desc[dir]->callback = ASRC_xPUT_DMA_CALLBACK(dir);

	desc->callback = ASRC_xPUT_DMA_CALLBACK(dir);
	desc->callback_param = pair;

	return 0;
}

static int fsl_asrc_prepare_io_buffer(struct fsl_asrc_pair *pair,
				      struct asrc_convert_buffer *pbuf, bool dir)
{
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_priv *asrc_priv = asrc->private;
	unsigned int *dma_len = &m2m->dma_block[dir].length;
	void *dma_vaddr = m2m->dma_block[dir].dma_vaddr;
	struct dma_chan *dma_chan = pair->dma_chan[dir];
	unsigned int buf_len, wm = m2m->watermark[dir];
	unsigned int *sg_nodes = &m2m->sg_nodes[dir];
	unsigned int last_period_size = m2m->last_period_size;
	enum asrc_pair_index index = pair->index;
	u32 word_size, fifo_addr;
	void __user *buf_vaddr;

	/* Clean the DMA buffer */
	memset(dma_vaddr, 0, ASRC_DMA_BUFFER_SIZE);

	if (dir == IN) {
		buf_vaddr = (void __user *)pbuf->input_buffer_vaddr;
		buf_len = pbuf->input_buffer_length;
	} else {
		buf_vaddr = (void __user *)pbuf->output_buffer_vaddr;
		buf_len = pbuf->output_buffer_length;
	}

	word_size = snd_pcm_format_physical_width(m2m->word_format[dir]) / 8;

	if (buf_len < word_size * pair->channels * wm ||
	    buf_len > ASRC_DMA_BUFFER_SIZE ||
	    (dir == OUT && buf_len < word_size * pair->channels * last_period_size)) {
		pair_err("%sput buffer size is error: [%d]\n",
				DIR_STR(dir), buf_len);
		return -EINVAL;
	}

	/* Copy origin data into input buffer */
	if (dir == IN && copy_from_user(dma_vaddr, buf_vaddr, buf_len))
		return -EFAULT;

	*dma_len = buf_len;
	if (dir == OUT) {
		*dma_len -= last_period_size * word_size * pair->channels;
		*dma_len = *dma_len / (word_size * pair->channels) *
				(word_size * pair->channels);
		if (asrc_priv->soc->use_edma)
			*dma_len = *dma_len / (word_size * pair->channels * m2m->watermark[OUT])
					* (word_size * pair->channels * m2m->watermark[OUT]);
	}

	*sg_nodes = *dma_len / ASRC_MAX_BUFFER_SIZE;
	if (*dma_len % ASRC_MAX_BUFFER_SIZE)
		*sg_nodes += 1;

	fifo_addr = asrc->paddr + REG_ASRDx(dir, index);

	return fsl_asrc_dmaconfig(pair, dma_chan, fifo_addr, dma_vaddr,
				  *dma_len, dir, m2m->word_format[dir]);
}

static int fsl_asrc_prepare_buffer(struct fsl_asrc_pair *pair,
				   struct asrc_convert_buffer *pbuf)
{
	struct fsl_asrc *asrc = pair->asrc;
	enum asrc_pair_index index = pair->index;
	int ret;

	ret = fsl_asrc_prepare_io_buffer(pair, pbuf, IN);
	if (ret) {
		pair_err("failed to prepare input buffer: %d\n", ret);
		return ret;
	}

	ret = fsl_asrc_prepare_io_buffer(pair, pbuf, OUT);
	if (ret) {
		pair_err("failed to prepare output buffer: %d\n", ret);
		return ret;
	}

	return 0;
}

static int fsl_asrc_process_buffer_pre(struct completion *complete,
				       enum asrc_pair_index index, bool dir)
{
	if (!wait_for_completion_interruptible_timeout(complete, 10 * HZ)) {
		pr_err("%sput DMA task timeout\n", DIR_STR(dir));
		return -ETIME;
	} else if (signal_pending(current)) {
		pr_err("%sput task forcibly aborted\n", DIR_STR(dir));
		return -ERESTARTSYS;
	}

	return 0;
}

#define mxc_asrc_dma_umap(dev, m2m) \
	do { \
		dma_unmap_sg(dev, m2m->sg[IN], m2m->sg_nodes[IN], \
			     DMA_TO_DEVICE); \
		dma_unmap_sg(dev, m2m->sg[OUT], m2m->sg_nodes[OUT], \
			     DMA_FROM_DEVICE); \
	} while (0)

static int fsl_asrc_process_buffer(struct fsl_asrc_pair *pair,
				   struct asrc_convert_buffer *pbuf)
{
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	enum asrc_pair_index index = pair->index;
	unsigned long lock_flags;
	int ret;

	/* Check input task first */
	ret = fsl_asrc_process_buffer_pre(&m2m->complete[IN], index, IN);
	if (ret) {
		mxc_asrc_dma_umap(&asrc->pdev->dev, m2m);
		return ret;
	}

	/* ...then output task*/
	ret = fsl_asrc_process_buffer_pre(&m2m->complete[OUT], index, OUT);
	if (ret) {
		mxc_asrc_dma_umap(&asrc->pdev->dev, m2m);
		return ret;
	}

	mxc_asrc_dma_umap(&asrc->pdev->dev, m2m);

	/* Fetch the remaining data */
	spin_lock_irqsave(&m2m->lock, lock_flags);
	if (!m2m->pair_hold) {
		spin_unlock_irqrestore(&m2m->lock, lock_flags);
		return -EFAULT;
	}
	spin_unlock_irqrestore(&m2m->lock, lock_flags);

	fsl_asrc_read_last_FIFO(pair);

	/* Update final lengths after getting last FIFO */
	pbuf->input_buffer_length = m2m->dma_block[IN].length;
	pbuf->output_buffer_length = m2m->dma_block[OUT].length;

	if (copy_to_user((void __user *)pbuf->output_buffer_vaddr,
			 m2m->dma_block[OUT].dma_vaddr,
			 m2m->dma_block[OUT].length))
		return -EFAULT;

	return 0;
}

#ifdef ASRC_POLLING_WITHOUT_DMA
/* THIS FUNCTION ONLY EXISTS FOR DEBUGGING AND ONLY SUPPORTS TWO CHANNELS */
static void fsl_asrc_polling_debug(struct fsl_asrc_pair *pair)
{
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	enum asrc_pair_index index = pair->index;
	u32 *in24 = m2m->dma_block[IN].dma_vaddr;
	u32 dma_len = m2m->dma_block[IN].length / (pair->channels * 4);
	u32 *reg24 = m2m->dma_block[OUT].dma_vaddr;
	u32 size, i, j, t_size, reg;

	t_size = 0;

	for (i = 0; i < dma_len; ) {
		for (j = 0; j < 2; j++) {
			regmap_write(asrc->regmap, REG_ASRDx(index), *in24);
			in24++;
			regmap_write(asrc->regmap, REG_ASRDx(index), *in24);
			in24++;
			i++;
		}
		udelay(50);
		udelay(50 * m2m->rate[OUT] / m2m->rate[IN]);

		size = fsl_asrc_get_output_FIFO_size(index);
		for (j = 0; j < size; j++) {
			regmap_read(asrc->regmap, REG_ASRDO(index), &reg);
			*(reg24) = reg;
			reg24++;
			regmap_read(asrc->regmap, REG_ASRDO(index), &reg);
			*(reg24) = reg;
			reg24++;
		}
		t_size += size;
	}

	mdelay(1);
	size = fsl_asrc_get_output_FIFO_size(index);
	for (j = 0; j < size; j++) {
		regmap_read(asrc->regmap, REG_ASRDO(index), &reg);
		*(reg24) = reg;
		reg24++;
		regmap_read(asrc->regmap, REG_ASRDO(index), &reg);
		*(reg24) = reg;
		reg24++;
	}
	t_size += size;

	m2m->dma_block[OUT].length = t_size * pair->channels * 4;

	complete(&m2m->complete[OUT]);
	complete(&m2m->complete[IN]);
}
#else
static void fsl_asrc_submit_dma(struct fsl_asrc_pair *pair)
{
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	enum asrc_pair_index index = pair->index;
	u32 size = fsl_asrc_get_output_FIFO_size(pair);
	int i;

	/* Read all data in OUTPUT FIFO */
	while (size) {
		u32 val;
		for (i = 0; i < size * pair->channels; i++)
			regmap_read(asrc->regmap, REG_ASRDO(index), &val);
		/* Fetch the data every 100us */
		udelay(100);

		size = fsl_asrc_get_output_FIFO_size(pair);
	}

	/* Submit DMA request */
	dmaengine_submit(pair->desc[IN]);
	dma_async_issue_pending(pair->desc[IN]->chan);

	dmaengine_submit(pair->desc[OUT]);
	dma_async_issue_pending(pair->desc[OUT]->chan);

	/*
	 * Clear DMA request during the stall state of ASRC:
	 * During STALL state, the remaining in input fifo would never be
	 * smaller than the input threshold while the output fifo would not
	 * be bigger than output one. Thus the DMA request would be cleared.
	 */
	fsl_asrc_set_watermarks(pair, ASRC_FIFO_THRESHOLD_MIN,
				ASRC_FIFO_THRESHOLD_MAX);

	/* Update the real input threshold to raise DMA request */
	fsl_asrc_set_watermarks(pair, m2m->watermark[IN], m2m->watermark[OUT]);
}
#endif /* ASRC_POLLING_WITHOUT_DMA */

static long fsl_asrc_ioctl_req_pair(struct fsl_asrc_pair *pair,
				    void __user *user)
{
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_priv *asrc_priv = asrc->private;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct device *dev = &asrc->pdev->dev;
	struct asrc_req req;
	unsigned long lock_flags;
	long ret;

	ret = copy_from_user(&req, user, sizeof(req));
	if (ret) {
		dev_err(dev, "failed to get req from user space: %ld\n", ret);
		return ret;
	}

	ret = fsl_asrc_request_pair(req.chn_num, pair);
	if (ret) {
		dev_err(dev, "failed to request pair: %ld\n", ret);
		return ret;
	}

	spin_lock_irqsave(&m2m->lock, lock_flags);
	m2m->pair_hold = 1;
	spin_unlock_irqrestore(&m2m->lock, lock_flags);
	pair->channels = req.chn_num;

	req.index = pair->index;
	req.supported_in_format = FSL_ASRC_FORMATS | SNDRV_PCM_FMTBIT_S8;
	req.supported_out_format = FSL_ASRC_FORMATS;
	if (asrc_priv->soc->use_edma) {
		req.supported_in_format &= ~SNDRV_PCM_FMTBIT_S24_3LE;
		req.supported_out_format &= ~SNDRV_PCM_FMTBIT_S24_3LE;
	}

	ret = copy_to_user(user, &req, sizeof(req));
	if (ret) {
		dev_err(dev, "failed to send req to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long fsl_asrc_ioctl_config_pair(struct fsl_asrc_pair *pair,
				       void __user *user)
{
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc_pair_priv *pair_priv = pair->private;
	struct device *dev = &asrc->pdev->dev;
	struct asrc_config config;
	enum asrc_pair_index index;
	long ret;

	ret = copy_from_user(&config, user, sizeof(config));
	if (ret) {
		dev_err(dev, "failed to get config from user space: %ld\n", ret);
		return ret;
	}

	index = config.pair;

	pair_priv->config = &config;
	ret = fsl_asrc_config_pair(pair, true);
	if (ret) {
		pair_err("failed to config pair: %ld\n", ret);
		return ret;
	}

	m2m->watermark[IN] = FSL_ASRC_INPUTFIFO_WML;
	m2m->watermark[OUT] = FSL_ASRC_OUTPUTFIFO_WML;

	fsl_asrc_set_watermarks(pair, m2m->watermark[IN], m2m->watermark[OUT]);

	m2m->dma_block[IN].length = ASRC_DMA_BUFFER_SIZE;
	m2m->dma_block[OUT].length = ASRC_DMA_BUFFER_SIZE;

	m2m->word_format[IN] = config.input_format;
	m2m->word_format[OUT] = config.output_format;

	m2m->rate[IN] = config.input_sample_rate;
	m2m->rate[OUT] = config.output_sample_rate;

	m2m->last_period_size = ASRC_OUTPUT_LAST_SAMPLE;

	ret = fsl_allocate_dma_buf(pair);
	if (ret) {
		pair_err("failed to allocate DMA buffer: %ld\n", ret);
		return ret;
	}

	/* Request DMA channel for both input and output */
	pair->dma_chan[IN] = fsl_asrc_get_dma_channel(pair, IN);
	if (pair->dma_chan[IN] == NULL) {
		pair_err("failed to request input task DMA channel\n");
		return  -EBUSY;
	}

	pair->dma_chan[OUT] = fsl_asrc_get_dma_channel(pair, OUT);
	if (pair->dma_chan[OUT] == NULL) {
		pair_err("failed to request output task DMA channel\n");
		return  -EBUSY;
	}

	ret = copy_to_user(user, &config, sizeof(config));
	if (ret) {
		pair_err("failed to send config to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long fsl_asrc_ioctl_release_pair(struct fsl_asrc_pair *pair,
					void __user *user)
{
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc *asrc = pair->asrc;
	enum asrc_pair_index index;
	unsigned long lock_flags;
	long ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		pair_err("failed to get index from user space: %ld\n", ret);
		return ret;
	}

	/* index might be not valid due to some application failure. */
	if (index < 0)
		return -EINVAL;

	m2m->asrc_active = 0;

	spin_lock_irqsave(&m2m->lock, lock_flags);
	m2m->pair_hold = 0;
	spin_unlock_irqrestore(&m2m->lock, lock_flags);

	if (pair->dma_chan[IN])
		dma_release_channel(pair->dma_chan[IN]);
	if (pair->dma_chan[OUT])
		dma_release_channel(pair->dma_chan[OUT]);
	kfree(m2m->dma_block[IN].dma_vaddr);
	kfree(m2m->dma_block[OUT].dma_vaddr);
	fsl_asrc_release_pair(pair);

	return 0;
}

static long fsl_asrc_calc_last_period_size(struct fsl_asrc_pair *pair,
					struct asrc_convert_buffer *pbuf)
{
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_priv *asrc_priv = asrc->private;
	unsigned int out_length;
	unsigned int in_width, out_width;
	unsigned int channels = pair->channels;
	unsigned int in_samples, out_samples;
	unsigned int last_period_size;
	unsigned int remain;

	in_width = snd_pcm_format_physical_width(m2m->word_format[IN]) / 8;
	out_width = snd_pcm_format_physical_width(m2m->word_format[OUT]) / 8;

	in_samples = pbuf->input_buffer_length / (in_width * channels);

	out_samples = (m2m->rate[OUT] * in_samples / m2m->rate[IN]);

	out_length = out_samples * out_width * channels;

	last_period_size = pbuf->output_buffer_length / (out_width * channels)
					- out_samples;

	m2m->last_period_size = last_period_size + 1 + ASRC_OUTPUT_LAST_SAMPLE;

	if (asrc_priv->soc->use_edma) {
		remain = pbuf->output_buffer_length % (out_width * channels * m2m->watermark[OUT]);
		if (remain)
			m2m->last_period_size += remain / (out_width * channels);
	}

	return 0;
}

static long fsl_asrc_ioctl_convert(struct fsl_asrc_pair *pair,
				   void __user *user)
{
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc *asrc = pair->asrc;
	enum asrc_pair_index index = pair->index;
	struct asrc_convert_buffer buf;
	long ret;

	ret = copy_from_user(&buf, user, sizeof(buf));
	if (ret) {
		pair_err("failed to get buf from user space: %ld\n", ret);
		return ret;
	}

	fsl_asrc_calc_last_period_size(pair, &buf);

	ret = fsl_asrc_prepare_buffer(pair, &buf);
	if (ret) {
		pair_err("failed to prepare buffer: %ld\n", ret);
		return ret;
	}

	reinit_completion(&m2m->complete[IN]);
	reinit_completion(&m2m->complete[OUT]);

#ifdef ASRC_POLLING_WITHOUT_DMA
	fsl_asrc_polling_debug(pair);
#else
	fsl_asrc_submit_dma(pair);
#endif

	ret = fsl_asrc_process_buffer(pair, &buf);
	if (ret) {
		if (ret != -ERESTARTSYS)
			pair_err("failed to process buffer: %ld\n", ret);
		return ret;
	}

	ret = copy_to_user(user, &buf, sizeof(buf));
	if (ret) {
		pair_err("failed to send buf to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long fsl_asrc_ioctl_start_conv(struct fsl_asrc_pair *pair,
				      void __user *user)
{
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	enum asrc_pair_index index;
	long ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		pair_err("failed to get index from user space: %ld\n", ret);
		return ret;
	}

	m2m->asrc_active = 1;
	fsl_asrc_start_pair(pair);

	return 0;
}

static long fsl_asrc_ioctl_stop_conv(struct fsl_asrc_pair *pair,
				     void __user *user)
{
	struct fsl_asrc *asrc = pair->asrc;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	enum asrc_pair_index index;
	long ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		pair_err("failed to get index from user space: %ld\n", ret);
		return ret;
	}

	dmaengine_terminate_all(pair->dma_chan[IN]);
	dmaengine_terminate_all(pair->dma_chan[OUT]);

	fsl_asrc_stop_pair(pair);
	m2m->asrc_active = 0;

	return 0;
}

static long fsl_asrc_ioctl_status(struct fsl_asrc_pair *pair, void __user *user)
{
	struct fsl_asrc *asrc = pair->asrc;
	enum asrc_pair_index index = pair->index;
	struct asrc_status_flags flags;
	long ret;

	ret = copy_from_user(&flags, user, sizeof(flags));
	if (ret) {
		pair_err("failed to get flags from user space: %ld\n", ret);
		return ret;
	}

	fsl_asrc_get_status(pair, &flags);

	ret = copy_to_user(user, &flags, sizeof(flags));
	if (ret) {
		pair_err("failed to send flags to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long fsl_asrc_ioctl_flush(struct fsl_asrc_pair *pair, void __user *user)
{
	struct fsl_asrc *asrc = pair->asrc;
	enum asrc_pair_index index = pair->index;

	/* Release DMA and request again */
	dma_release_channel(pair->dma_chan[IN]);
	dma_release_channel(pair->dma_chan[OUT]);

	pair->dma_chan[IN] = fsl_asrc_get_dma_channel(pair, IN);
	if (pair->dma_chan[IN] == NULL) {
		pair_err("failed to request input task DMA channel\n");
		return -EBUSY;
	}

	pair->dma_chan[OUT] = fsl_asrc_get_dma_channel(pair, OUT);
	if (pair->dma_chan[OUT] == NULL) {
		pair_err("failed to request output task DMA channel\n");
		return -EBUSY;
	}

	return 0;
}

static long fsl_asrc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct fsl_asrc_pair *pair = file->private_data;
	struct fsl_asrc *asrc = pair->asrc;
	void __user *user = (void __user *)arg;
	long ret = 0;

	switch (cmd) {
	case ASRC_REQ_PAIR:
		ret = fsl_asrc_ioctl_req_pair(pair, user);
		break;
	case ASRC_CONFIG_PAIR:
		ret = fsl_asrc_ioctl_config_pair(pair, user);
		break;
	case ASRC_RELEASE_PAIR:
		ret = fsl_asrc_ioctl_release_pair(pair, user);
		break;
	case ASRC_CONVERT:
		ret = fsl_asrc_ioctl_convert(pair, user);
		break;
	case ASRC_START_CONV:
		ret = fsl_asrc_ioctl_start_conv(pair, user);
		break;
	case ASRC_STOP_CONV:
		ret = fsl_asrc_ioctl_stop_conv(pair, user);
		break;
	case ASRC_STATUS:
		ret = fsl_asrc_ioctl_status(pair, user);
		break;
	case ASRC_FLUSH:
		ret = fsl_asrc_ioctl_flush(pair, user);
		break;
	default:
		dev_err(&asrc->pdev->dev, "invalid ioctl cmd!\n");
		break;
	}

	return ret;
}

static int fsl_asrc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *asrc_miscdev = file->private_data;
	struct fsl_asrc *asrc = dev_get_drvdata(asrc_miscdev->parent);
	struct device *dev = &asrc->pdev->dev;
	struct fsl_asrc_pair *pair;
	struct fsl_asrc_m2m *m2m;
	int ret;

	ret = signal_pending(current);
	if (ret) {
		dev_err(dev, "current process has a signal pending\n");
		return ret;
	}

	pair = kzalloc(sizeof(struct fsl_asrc_pair) + asrc->pair_priv_size, GFP_KERNEL);
	if (!pair) {
		dev_err(dev, "failed to allocate pair\n");
		return -ENOMEM;
	}

	pair->private = (void *)pair + sizeof(struct fsl_asrc_pair);

	m2m = kzalloc(sizeof(struct fsl_asrc_m2m), GFP_KERNEL);
	if (!m2m) {
		dev_err(dev, "failed to allocate m2m resource\n");
		ret = -ENOMEM;
		goto out;
	}

	pair->private_m2m = m2m;
	pair->asrc = asrc;

	spin_lock_init(&m2m->lock);
	init_completion(&m2m->complete[IN]);
	init_completion(&m2m->complete[OUT]);

	file->private_data = pair;

	pm_runtime_get_sync(dev);

	return 0;
out:
	kfree(pair);

	return ret;
}

static int fsl_asrc_close(struct inode *inode, struct file *file)
{
	struct fsl_asrc_pair *pair = file->private_data;
	struct fsl_asrc_m2m *m2m = pair->private_m2m;
	struct fsl_asrc *asrc = pair->asrc;
	struct device *dev = &asrc->pdev->dev;
	unsigned long lock_flags;

	if (m2m->asrc_active) {
		m2m->asrc_active = 0;

		dmaengine_terminate_all(pair->dma_chan[IN]);
		dmaengine_terminate_all(pair->dma_chan[OUT]);

		fsl_asrc_stop_pair(pair);
		fsl_asrc_input_dma_callback((void *)pair);
		fsl_asrc_output_dma_callback((void *)pair);
	}

	spin_lock_irqsave(&m2m->lock, lock_flags);
	if (m2m->pair_hold) {
		m2m->pair_hold = 0;
		spin_unlock_irqrestore(&m2m->lock, lock_flags);

		if (pair->dma_chan[IN])
			dma_release_channel(pair->dma_chan[IN]);
		if (pair->dma_chan[OUT])
			dma_release_channel(pair->dma_chan[OUT]);

		kfree(m2m->dma_block[IN].dma_vaddr);
		kfree(m2m->dma_block[OUT].dma_vaddr);

		fsl_asrc_release_pair(pair);
	} else
		spin_unlock_irqrestore(&m2m->lock, lock_flags);

	spin_lock_irqsave(&asrc->lock, lock_flags);
	kfree(m2m);
	kfree(pair);
	spin_unlock_irqrestore(&asrc->lock, lock_flags);
	file->private_data = NULL;

	pm_runtime_put_sync(dev);

	return 0;
}

static const struct file_operations asrc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= fsl_asrc_ioctl,
	.open		= fsl_asrc_open,
	.release	= fsl_asrc_close,
};

static int fsl_asrc_m2m_init(struct fsl_asrc *asrc)
{
	struct device *dev = &asrc->pdev->dev;
	struct fsl_asrc_priv *asrc_priv = asrc->private;
	int ret;

	asrc->asrc_miscdev.fops = &asrc_fops;
	asrc->asrc_miscdev.parent = dev;
	asrc->asrc_miscdev.name = asrc_priv->name;
	asrc->asrc_miscdev.minor = MISC_DYNAMIC_MINOR;
	ret = misc_register(&asrc->asrc_miscdev);
	if (ret) {
		dev_err(dev, "failed to register char device %d\n", ret);
		return ret;
	}

	return 0;
}

static int fsl_asrc_m2m_remove(struct platform_device *pdev)
{
	struct fsl_asrc *asrc = dev_get_drvdata(&pdev->dev);

	misc_deregister(&asrc->asrc_miscdev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void fsl_asrc_m2m_suspend(struct fsl_asrc *asrc)
{
	struct fsl_asrc_pair *pair;
	struct fsl_asrc_m2m *m2m;
	unsigned long lock_flags;
	int i;

	for (i = 0; i < ASRC_PAIR_MAX_NUM; i++) {
		spin_lock_irqsave(&asrc->lock, lock_flags);
		pair = asrc->pair[i];
		if (!pair || !pair->private_m2m) {
			spin_unlock_irqrestore(&asrc->lock, lock_flags);
			continue;
		}
		m2m = pair->private_m2m;

		if (!completion_done(&m2m->complete[IN])) {
			if (pair->dma_chan[IN])
				dmaengine_terminate_all(pair->dma_chan[IN]);
			fsl_asrc_input_dma_callback((void *)pair);
		}
		if (!completion_done(&m2m->complete[OUT])) {
			if (pair->dma_chan[OUT])
				dmaengine_terminate_all(pair->dma_chan[OUT]);
			fsl_asrc_output_dma_callback((void *)pair);
		}

		spin_unlock_irqrestore(&asrc->lock, lock_flags);
	}
}

static void fsl_asrc_m2m_resume(struct fsl_asrc *asrc)
{
	struct fsl_asrc_pair *pair;
	struct fsl_asrc_m2m *m2m;
	unsigned long lock_flags;
	enum asrc_pair_index index;
	int i, j;

	for (i = 0; i < ASRC_PAIR_MAX_NUM; i++) {
		spin_lock_irqsave(&asrc->lock, lock_flags);
		pair = asrc->pair[i];
		if (!pair || !pair->private_m2m) {
			spin_unlock_irqrestore(&asrc->lock, lock_flags);
			continue;
		}
		m2m = pair->private_m2m;
		index = pair->index;

		for (j = 0; j < pair->channels * 4; j++)
			regmap_write(asrc->regmap, REG_ASRDI(index), 0);

		spin_unlock_irqrestore(&asrc->lock, lock_flags);
	}
}
#endif
