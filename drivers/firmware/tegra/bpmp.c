// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/acpi.h>
#include <linux/clk/tegra.h>
#include <linux/genalloc.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/semaphore.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>

#include <soc/tegra/bpmp.h>
#include <soc/tegra/bpmp-abi.h>
#include <soc/tegra/ivc.h>

#include "bpmp-private.h"

#define MSG_ACK		BIT(0)
#define MSG_RING	BIT(1)
#define TAG_SZ		32
#define TEGRA_BPMP_ACPI_BMRQ_DATA_SZ	3960U

struct tegra_bpmp_acpi_message {
	u64 status;
	u8 *data;
};

static inline const struct tegra_bpmp_ops *
channel_to_ops(struct tegra_bpmp_channel *channel)
{
	struct tegra_bpmp *bpmp = channel->bpmp;

	return bpmp->soc->ops;
}

struct tegra_bpmp *tegra_bpmp_get_with_id(struct device *dev, unsigned int *id)
{
	struct platform_device *pdev;
	struct of_phandle_args args;
	struct tegra_bpmp *bpmp;
	int err;

	err = __of_parse_phandle_with_args(dev->of_node, "nvidia,bpmp", NULL,
					   1, 0, &args);
	if (err < 0)
		return ERR_PTR(err);

	pdev = of_find_device_by_node(args.np);
	if (!pdev) {
		bpmp = ERR_PTR(-ENODEV);
		goto put;
	}

	bpmp = platform_get_drvdata(pdev);
	if (!bpmp) {
		bpmp = ERR_PTR(-EPROBE_DEFER);
		put_device(&pdev->dev);
		goto put;
	}

	if (id)
		*id = args.args[0];

put:
	of_node_put(args.np);
	return bpmp;
}
EXPORT_SYMBOL_GPL(tegra_bpmp_get_with_id);

struct tegra_bpmp *tegra_bpmp_get(struct device *dev)
{
	struct platform_device *pdev;
	struct tegra_bpmp *bpmp;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "nvidia,bpmp", 0);
	if (!np)
		return ERR_PTR(-ENOENT);

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		bpmp = ERR_PTR(-ENODEV);
		goto put;
	}

	bpmp = platform_get_drvdata(pdev);
	if (!bpmp) {
		bpmp = ERR_PTR(-EPROBE_DEFER);
		put_device(&pdev->dev);
		goto put;
	}

put:
	of_node_put(np);
	return bpmp;
}
EXPORT_SYMBOL_GPL(tegra_bpmp_get);

void tegra_bpmp_put(struct tegra_bpmp *bpmp)
{
	if (bpmp)
		put_device(bpmp->dev);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_put);

static int
tegra_bpmp_channel_get_thread_index(struct tegra_bpmp_channel *channel)
{
	struct tegra_bpmp *bpmp = channel->bpmp;
	unsigned int count;
	int index;

	count = bpmp->soc->channels.thread.count;

	index = channel - channel->bpmp->threaded_channels;
	if (index < 0 || index >= count)
		return -EINVAL;

	return index;
}

static bool tegra_bpmp_message_valid(const struct tegra_bpmp_message *msg)
{
	return (msg->tx.size <= MSG_DATA_MIN_SZ) &&
	       (msg->rx.size <= MSG_DATA_MIN_SZ) &&
	       (msg->tx.size == 0 || msg->tx.data) &&
	       (msg->rx.size == 0 || msg->rx.data);
}

static bool tegra_bpmp_is_response_ready(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->is_response_ready(channel);
}

static bool tegra_bpmp_is_request_ready(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->is_request_ready(channel);
}

static int tegra_bpmp_wait_response(struct tegra_bpmp_channel *channel)
{
	unsigned long timeout = channel->bpmp->soc->channels.cpu_tx.timeout;
	ktime_t end;

	end = ktime_add_us(ktime_get(), timeout);

	do {
		if (tegra_bpmp_is_response_ready(channel))
			return 0;
	} while (ktime_before(ktime_get(), end));

	return -ETIMEDOUT;
}

static int tegra_bpmp_ack_response(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->ack_response(channel);
}

static int tegra_bpmp_ack_request(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->ack_request(channel);
}

static bool
tegra_bpmp_is_request_channel_free(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->is_request_channel_free(channel);
}

static bool
tegra_bpmp_is_response_channel_free(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->is_response_channel_free(channel);
}

static int
tegra_bpmp_wait_request_channel_free(struct tegra_bpmp_channel *channel)
{
	unsigned long timeout = channel->bpmp->soc->channels.cpu_tx.timeout;
	ktime_t start, now;

	start = ns_to_ktime(local_clock());

	do {
		if (tegra_bpmp_is_request_channel_free(channel))
			return 0;

		now = ns_to_ktime(local_clock());
	} while (ktime_us_delta(now, start) < timeout);

	return -ETIMEDOUT;
}

static int tegra_bpmp_post_request(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->post_request(channel);
}

static int tegra_bpmp_post_response(struct tegra_bpmp_channel *channel)
{
	const struct tegra_bpmp_ops *ops = channel_to_ops(channel);

	return ops->post_response(channel);
}

static int tegra_bpmp_ring_doorbell(struct tegra_bpmp *bpmp)
{
	return bpmp->soc->ops->ring_doorbell(bpmp);
}

static ssize_t __tegra_bpmp_channel_read(struct tegra_bpmp_channel *channel,
					 void *data, size_t size, int *ret)
{
	int err;

	if (data && size > 0)
		tegra_bpmp_mb_read(data, &channel->ib, size);

	err = tegra_bpmp_ack_response(channel);
	if (err < 0)
		return err;

	*ret = tegra_bpmp_mb_read_field(&channel->ib, code);

	return 0;
}

static ssize_t tegra_bpmp_channel_read(struct tegra_bpmp_channel *channel,
				       void *data, size_t size, int *ret)
{
	struct tegra_bpmp *bpmp = channel->bpmp;
	unsigned long flags;
	ssize_t err;
	int index;

	index = tegra_bpmp_channel_get_thread_index(channel);
	if (index < 0) {
		err = index;
		goto unlock;
	}

	spin_lock_irqsave(&bpmp->lock, flags);
	err = __tegra_bpmp_channel_read(channel, data, size, ret);
	clear_bit(index, bpmp->threaded.allocated);
	spin_unlock_irqrestore(&bpmp->lock, flags);

unlock:
	up(&bpmp->threaded.lock);

	return err;
}

static ssize_t __tegra_bpmp_channel_write(struct tegra_bpmp_channel *channel,
					  unsigned int mrq, unsigned long flags,
					  const void *data, size_t size)
{
	tegra_bpmp_mb_write_field(&channel->ob, code, mrq);
	tegra_bpmp_mb_write_field(&channel->ob, flags, flags);

	if (data && size > 0)
		tegra_bpmp_mb_write(&channel->ob, data, size);

	return tegra_bpmp_post_request(channel);
}

static struct tegra_bpmp_channel *
tegra_bpmp_write_threaded(struct tegra_bpmp *bpmp, unsigned int mrq,
			  const void *data, size_t size)
{
	unsigned long timeout = bpmp->soc->channels.thread.timeout;
	unsigned int count = bpmp->soc->channels.thread.count;
	struct tegra_bpmp_channel *channel;
	unsigned long flags;
	unsigned int index;
	int err;

	err = down_timeout(&bpmp->threaded.lock, usecs_to_jiffies(timeout));
	if (err < 0)
		return ERR_PTR(err);

	spin_lock_irqsave(&bpmp->lock, flags);

	index = find_first_zero_bit(bpmp->threaded.allocated, count);
	if (index == count) {
		err = -EBUSY;
		goto unlock;
	}

	channel = &bpmp->threaded_channels[index];

	if (!tegra_bpmp_is_request_channel_free(channel)) {
		err = -EBUSY;
		goto unlock;
	}

	set_bit(index, bpmp->threaded.allocated);

	err = __tegra_bpmp_channel_write(channel, mrq, MSG_ACK | MSG_RING,
					 data, size);
	if (err < 0)
		goto clear_allocated;

	set_bit(index, bpmp->threaded.busy);

	spin_unlock_irqrestore(&bpmp->lock, flags);
	return channel;

clear_allocated:
	clear_bit(index, bpmp->threaded.allocated);
unlock:
	spin_unlock_irqrestore(&bpmp->lock, flags);
	up(&bpmp->threaded.lock);

	return ERR_PTR(err);
}

static ssize_t tegra_bpmp_channel_write(struct tegra_bpmp_channel *channel,
					unsigned int mrq, unsigned long flags,
					const void *data, size_t size)
{
	int err;

	err = tegra_bpmp_wait_request_channel_free(channel);
	if (err < 0)
		return err;

	return __tegra_bpmp_channel_write(channel, mrq, flags, data, size);
}

static int __maybe_unused tegra_bpmp_resume(struct device *dev);

#ifdef CONFIG_ACPI
static int tegra_bpmp_transfer_acpi(struct tegra_bpmp *bpmp,
				    struct tegra_bpmp_message *msg)
{
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_buffer response = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_buffer format = { sizeof("NB"), "NB" };
	struct acpi_object_list param_list;
	union acpi_object params[2];
	struct tegra_bpmp_acpi_message *reply;
	acpi_status status;
	size_t data_len;
	int err = 0;

	params[0].type = ACPI_TYPE_INTEGER;
	params[0].integer.value = msg->mrq;

	params[1].type = ACPI_TYPE_BUFFER;
	params[1].buffer.length = msg->tx.size;
	params[1].buffer.pointer = (u8 *)msg->tx.data;

	param_list.count = 2;
	param_list.pointer = params;

	status = acpi_evaluate_object(ACPI_HANDLE(bpmp->dev), "BMRQ",
				      &param_list, &output);
	if (ACPI_FAILURE(status)) {
		acpi_evaluation_failure_warn(ACPI_HANDLE(bpmp->dev), "BMRQ",
					     status);
		return -ENODEV;
	}

	status = acpi_extract_package(output.pointer, &format, &response);
	if (ACPI_FAILURE(status)) {
		dev_err(bpmp->dev, "BMRQ: invalid response package: %s\n",
			acpi_format_exception(status));
		err = -ENODATA;
		goto out;
	}

	if (response.length < sizeof(*reply)) {
		dev_err(bpmp->dev, "BMRQ: response too short\n");
		err = -ENODATA;
		goto out;
	}

	reply = response.pointer;
	data_len = response.length - sizeof(*reply);
	if (data_len > TEGRA_BPMP_ACPI_BMRQ_DATA_SZ) {
		dev_err(bpmp->dev, "BMRQ: reply buffer too large (%zu)\n",
			data_len);
		err = -EINVAL;
		goto out;
	}

	msg->rx.ret = (int)reply->status;

	if (msg->rx.data && msg->rx.size) {
		if (data_len < msg->rx.size) {
			dev_err(bpmp->dev, "BMRQ: response data too short\n");
			err = -ENODATA;
			goto out;
		}

		memcpy(msg->rx.data, reply->data, msg->rx.size);
	}

out:
	kfree(response.pointer);
	kfree(output.pointer);

	return err;
}
#else
static int tegra_bpmp_transfer_acpi(struct tegra_bpmp *bpmp,
				    struct tegra_bpmp_message *msg)
{
	return -EOPNOTSUPP;
}
#endif

int tegra_bpmp_transfer_atomic(struct tegra_bpmp *bpmp,
			       struct tegra_bpmp_message *msg)
{
	struct tegra_bpmp_channel *channel;
	int err;

	if (WARN_ON(ACPI_HANDLE(bpmp->dev)))
		return -EOPNOTSUPP;

	if (WARN_ON(!irqs_disabled()))
		return -EPERM;

	if (!tegra_bpmp_message_valid(msg))
		return -EINVAL;

	if (bpmp->suspended) {
		/* Reset BPMP IPC channels during resume based on flags passed */
		if (msg->flags & TEGRA_BPMP_MESSAGE_RESET)
			tegra_bpmp_resume(bpmp->dev);
		else
			return -EAGAIN;
	}

	channel = bpmp->tx_channel;

	spin_lock(&bpmp->atomic_tx_lock);

	err = tegra_bpmp_channel_write(channel, msg->mrq, MSG_ACK,
				       msg->tx.data, msg->tx.size);
	if (err < 0) {
		spin_unlock(&bpmp->atomic_tx_lock);
		return err;
	}

	spin_unlock(&bpmp->atomic_tx_lock);

	err = tegra_bpmp_ring_doorbell(bpmp);
	if (err < 0)
		return err;

	err = tegra_bpmp_wait_response(channel);
	if (err < 0)
		return err;

	return __tegra_bpmp_channel_read(channel, msg->rx.data, msg->rx.size,
					 &msg->rx.ret);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_transfer_atomic);

static int tegra_bpmp_transfer_channel(struct tegra_bpmp *bpmp,
				       struct tegra_bpmp_message *msg)
{
	struct tegra_bpmp_channel *channel;
	unsigned long timeout;
	int err;

	if (bpmp->suspended) {
		/* Reset BPMP IPC channels during resume based on flags passed */
		if (msg->flags & TEGRA_BPMP_MESSAGE_RESET)
			tegra_bpmp_resume(bpmp->dev);
		else
			return -EAGAIN;
	}

	channel = tegra_bpmp_write_threaded(bpmp, msg->mrq, msg->tx.data,
					    msg->tx.size);
	if (IS_ERR(channel))
		return PTR_ERR(channel);

	err = tegra_bpmp_ring_doorbell(bpmp);
	if (err < 0)
		return err;

	timeout = usecs_to_jiffies(bpmp->soc->channels.thread.timeout);

	err = wait_for_completion_timeout(&channel->completion, timeout);
	if (err == 0)
		return -ETIMEDOUT;

	return tegra_bpmp_channel_read(channel, msg->rx.data, msg->rx.size,
				       &msg->rx.ret);
}

int tegra_bpmp_transfer(struct tegra_bpmp *bpmp,
			struct tegra_bpmp_message *msg)
{
	if (WARN_ON(irqs_disabled()))
		return -EPERM;

	if (!tegra_bpmp_message_valid(msg))
		return -EINVAL;

	if (ACPI_HANDLE(bpmp->dev))
		return tegra_bpmp_transfer_acpi(bpmp, msg);

	return tegra_bpmp_transfer_channel(bpmp, msg);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_transfer);

static struct tegra_bpmp_mrq *tegra_bpmp_find_mrq(struct tegra_bpmp *bpmp,
						  unsigned int mrq)
{
	struct tegra_bpmp_mrq *entry;

	list_for_each_entry(entry, &bpmp->mrqs, list)
		if (entry->mrq == mrq)
			return entry;

	return NULL;
}

void tegra_bpmp_mrq_return(struct tegra_bpmp_channel *channel, int code,
			   const void *data, size_t size)
{
	unsigned long flags = tegra_bpmp_mb_read_field(&channel->ib, flags);
	struct tegra_bpmp *bpmp = channel->bpmp;
	int err;

	if (WARN_ON(size > MSG_DATA_MIN_SZ))
		return;

	err = tegra_bpmp_ack_request(channel);
	if (WARN_ON(err < 0))
		return;

	if ((flags & MSG_ACK) == 0)
		return;

	if (WARN_ON(!tegra_bpmp_is_response_channel_free(channel)))
		return;

	tegra_bpmp_mb_write_field(&channel->ob, code, code);

	if (data && size > 0)
		tegra_bpmp_mb_write(&channel->ob, data, size);

	err = tegra_bpmp_post_response(channel);
	if (WARN_ON(err < 0))
		return;

	if (flags & MSG_RING) {
		err = tegra_bpmp_ring_doorbell(bpmp);
		if (WARN_ON(err < 0))
			return;
	}
}
EXPORT_SYMBOL_GPL(tegra_bpmp_mrq_return);

static void tegra_bpmp_handle_mrq(struct tegra_bpmp *bpmp,
				  unsigned int mrq,
				  struct tegra_bpmp_channel *channel)
{
	struct tegra_bpmp_mrq *entry;
	u32 zero = 0;

	spin_lock(&bpmp->lock);

	entry = tegra_bpmp_find_mrq(bpmp, mrq);
	if (!entry) {
		spin_unlock(&bpmp->lock);
		tegra_bpmp_mrq_return(channel, -EINVAL, &zero, sizeof(zero));
		return;
	}

	entry->handler(mrq, channel, entry->data);

	spin_unlock(&bpmp->lock);
}

int tegra_bpmp_request_mrq(struct tegra_bpmp *bpmp, unsigned int mrq,
			   tegra_bpmp_mrq_handler_t handler, void *data)
{
	struct tegra_bpmp_mrq *entry;
	unsigned long flags;

	if (!handler)
		return -EINVAL;

	entry = devm_kzalloc(bpmp->dev, sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	spin_lock_irqsave(&bpmp->lock, flags);

	entry->mrq = mrq;
	entry->handler = handler;
	entry->data = data;
	list_add(&entry->list, &bpmp->mrqs);

	spin_unlock_irqrestore(&bpmp->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_bpmp_request_mrq);

void tegra_bpmp_free_mrq(struct tegra_bpmp *bpmp, unsigned int mrq, void *data)
{
	struct tegra_bpmp_mrq *entry;
	unsigned long flags;

	spin_lock_irqsave(&bpmp->lock, flags);

	entry = tegra_bpmp_find_mrq(bpmp, mrq);
	if (!entry)
		goto unlock;

	list_del(&entry->list);
	devm_kfree(bpmp->dev, entry);

unlock:
	spin_unlock_irqrestore(&bpmp->lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_free_mrq);

bool tegra_bpmp_mrq_is_supported(struct tegra_bpmp *bpmp, unsigned int mrq)
{
	struct mrq_query_abi_request req = { .mrq = mrq };
	struct mrq_query_abi_response resp;
	struct tegra_bpmp_message msg = {
		.mrq = MRQ_QUERY_ABI,
		.tx = {
			.data = &req,
			.size = sizeof(req),
		},
		.rx = {
			.data = &resp,
			.size = sizeof(resp),
		},
	};
	int err;

	err = tegra_bpmp_transfer(bpmp, &msg);
	if (err || msg.rx.ret)
		return false;

	return resp.status == 0;
}
EXPORT_SYMBOL_GPL(tegra_bpmp_mrq_is_supported);

static void tegra_bpmp_mrq_handle_ping(unsigned int mrq,
				       struct tegra_bpmp_channel *channel,
				       void *data)
{
	struct mrq_ping_request request;
	struct mrq_ping_response response;

	tegra_bpmp_mb_read(&request, &channel->ib, sizeof(request));

	memset(&response, 0, sizeof(response));
	response.reply = request.challenge << 1;

	tegra_bpmp_mrq_return(channel, 0, &response, sizeof(response));
}

static int tegra_bpmp_ping(struct tegra_bpmp *bpmp)
{
	struct mrq_ping_response response;
	struct mrq_ping_request request;
	struct tegra_bpmp_message msg;
	unsigned long flags;
	ktime_t start, end;
	int err;

	memset(&request, 0, sizeof(request));
	request.challenge = 1;

	memset(&response, 0, sizeof(response));

	memset(&msg, 0, sizeof(msg));
	msg.mrq = MRQ_PING;
	msg.tx.data = &request;
	msg.tx.size = sizeof(request);
	msg.rx.data = &response;
	msg.rx.size = sizeof(response);

	start = ktime_get();

	if (ACPI_HANDLE(bpmp->dev)) {
		err = tegra_bpmp_transfer(bpmp, &msg);
	} else {
		local_irq_save(flags);
		err = tegra_bpmp_transfer_atomic(bpmp, &msg);
		local_irq_restore(flags);
	}

	end = ktime_get();

	if (!err)
		dev_dbg(bpmp->dev,
			"ping ok: challenge: %u, response: %u, time: %lld\n",
			request.challenge, response.reply,
			ktime_to_us(ktime_sub(end, start)));

	return err;
}

/* deprecated version of tag query */
static int tegra_bpmp_get_firmware_tag_old(struct tegra_bpmp *bpmp, char *tag,
					   size_t size)
{
	struct mrq_query_tag_request request;
	struct tegra_bpmp_message msg;
	unsigned long flags;
	dma_addr_t phys;
	void *virt;
	int err;

	if (ACPI_HANDLE(bpmp->dev))
		return -EOPNOTSUPP;

	if (size != TAG_SZ)
		return -EINVAL;

	virt = dma_alloc_coherent(bpmp->dev, TAG_SZ, &phys,
				  GFP_KERNEL | GFP_DMA32);
	if (!virt)
		return -ENOMEM;

	memset(&request, 0, sizeof(request));
	request.addr = phys;

	memset(&msg, 0, sizeof(msg));
	msg.mrq = MRQ_QUERY_TAG;
	msg.tx.data = &request;
	msg.tx.size = sizeof(request);

	local_irq_save(flags);
	err = tegra_bpmp_transfer_atomic(bpmp, &msg);
	local_irq_restore(flags);

	if (err == 0)
		memcpy(tag, virt, TAG_SZ);

	dma_free_coherent(bpmp->dev, TAG_SZ, virt, phys);

	return err;
}

static int tegra_bpmp_get_firmware_tag(struct tegra_bpmp *bpmp, char *tag,
				       size_t size)
{
	if (tegra_bpmp_mrq_is_supported(bpmp, MRQ_QUERY_FW_TAG)) {
		struct mrq_query_fw_tag_response resp;
		struct tegra_bpmp_message msg = {
			.mrq = MRQ_QUERY_FW_TAG,
			.rx = {
				.data = &resp,
				.size = sizeof(resp),
			},
		};
		int err;

		if (size != sizeof(resp.tag))
			return -EINVAL;

		err = tegra_bpmp_transfer(bpmp, &msg);

		if (err)
			return err;
		if (msg.rx.ret < 0)
			return -EINVAL;

		memcpy(tag, resp.tag, sizeof(resp.tag));
		return 0;
	}

	return tegra_bpmp_get_firmware_tag_old(bpmp, tag, size);
}

static void tegra_bpmp_channel_signal(struct tegra_bpmp_channel *channel)
{
	unsigned long flags = tegra_bpmp_mb_read_field(&channel->ob, flags);

	if ((flags & MSG_RING) == 0)
		return;

	complete(&channel->completion);
}

void tegra_bpmp_handle_rx(struct tegra_bpmp *bpmp)
{
	struct tegra_bpmp_channel *channel;
	unsigned int i, count;
	unsigned long *busy;

	channel = bpmp->rx_channel;
	count = bpmp->soc->channels.thread.count;
	busy = bpmp->threaded.busy;

	if (tegra_bpmp_is_request_ready(channel)) {
		unsigned int mrq = tegra_bpmp_mb_read_field(&channel->ib, code);

		tegra_bpmp_handle_mrq(bpmp, mrq, channel);
	}

	spin_lock(&bpmp->lock);

	for_each_set_bit(i, busy, count) {
		struct tegra_bpmp_channel *channel;

		channel = &bpmp->threaded_channels[i];

		if (tegra_bpmp_is_response_ready(channel)) {
			tegra_bpmp_channel_signal(channel);
			clear_bit(i, busy);
		}
	}

	spin_unlock(&bpmp->lock);
}


/*
 * MBWT controls are not exposed through ordinary host MMIO and are accessed
 * through BPMP firmware requests. These helpers provide the request wrappers
 * used by bandwidth control callers.
 */
bool tegra_bpmp_mbwt_cmd_is_supported(struct tegra_bpmp *bpmp,
				      unsigned int cmd_code)
{
	struct mrq_sochub_mbwt_request request = {
		.cmd = CMD_SOCHUB_MBWT_QUERY_ABI,
		.query_abi.cmd_code = cmd_code,
	};
	struct tegra_bpmp_message msg = {
		.mrq = MRQ_SOCHUB_MBWT,
		.tx = {
			.data = &request,
			.size = sizeof(request),
		},
	};
	int err;

	err = tegra_bpmp_transfer(bpmp, &msg);
	if (err || msg.rx.ret)
		return false;

	return true;
}

int tegra_bpmp_mbwt_get(struct tegra_bpmp *bpmp, unsigned int instance,
			unsigned int vc_type, unsigned int *bandwidth)
{
	struct mrq_sochub_mbwt_request request = {
		.cmd = CMD_SOCHUB_MBWT_GET_BW,
		.get_bw = {
			.instance = instance,
			.vc_type = vc_type,
		},
	};
	struct mrq_sochub_mbwt_response response = {};
	struct tegra_bpmp_message msg = {
		.mrq = MRQ_SOCHUB_MBWT,
		.tx = {
			.data = &request,
			.size = sizeof(request),
		},
		.rx = {
			.data = &response,
			.size = sizeof(response),
		},
	};
	int err;

	if (!bandwidth)
		return -EINVAL;

	err = tegra_bpmp_transfer(bpmp, &msg);
	if (err) {
		dev_err(bpmp->dev, "MBWT get bandwidth transfer failed: %d\n",
			err);
		return err;
	}

	if (msg.rx.ret) {
		dev_err(bpmp->dev, "MBWT get bandwidth failed: BPMP error %d\n",
			msg.rx.ret);
		return -EIO;
	}

	*bandwidth = response.get_bw.bw;

	return 0;
}

int tegra_bpmp_mbwt_set(struct tegra_bpmp *bpmp, unsigned int instance,
			unsigned int vc_type, unsigned int bandwidth)
{
	struct mrq_sochub_mbwt_request request = {
		.cmd = CMD_SOCHUB_MBWT_SET_BW,
		.set_bw = {
			.instance = instance,
			.vc_type = vc_type,
			.bw = bandwidth,
		},
	};
	struct tegra_bpmp_message msg = {
		.mrq = MRQ_SOCHUB_MBWT,
		.tx = {
			.data = &request,
			.size = sizeof(request),
		},
	};
	int err;

	err = tegra_bpmp_transfer(bpmp, &msg);
	if (err) {
		dev_err(bpmp->dev, "MBWT set bandwidth transfer failed: %d\n",
			err);
		return err;
	}

	if (msg.rx.ret) {
		dev_err(bpmp->dev, "MBWT set bandwidth failed: BPMP error %d\n",
			msg.rx.ret);
		return -EIO;
	}

	return 0;
}

int tegra_bpmp_init_channels(struct tegra_bpmp *bpmp)
{
	size_t size;

	INIT_LIST_HEAD(&bpmp->mrqs);
	spin_lock_init(&bpmp->lock);

	bpmp->threaded.count = bpmp->soc->channels.thread.count;
	sema_init(&bpmp->threaded.lock, bpmp->threaded.count);

	size = BITS_TO_LONGS(bpmp->threaded.count) * sizeof(long);

	bpmp->threaded.allocated = devm_kzalloc(bpmp->dev, size, GFP_KERNEL);
	if (!bpmp->threaded.allocated)
		return -ENOMEM;

	bpmp->threaded.busy = devm_kzalloc(bpmp->dev, size, GFP_KERNEL);
	if (!bpmp->threaded.busy)
		return -ENOMEM;

	spin_lock_init(&bpmp->atomic_tx_lock);
	bpmp->tx_channel = devm_kzalloc(bpmp->dev, sizeof(*bpmp->tx_channel),
					GFP_KERNEL);
	if (!bpmp->tx_channel)
		return -ENOMEM;

	bpmp->rx_channel = devm_kzalloc(bpmp->dev, sizeof(*bpmp->rx_channel),
					GFP_KERNEL);
	if (!bpmp->rx_channel)
		return -ENOMEM;

	bpmp->threaded_channels = devm_kcalloc(bpmp->dev, bpmp->threaded.count,
					       sizeof(*bpmp->threaded_channels),
					       GFP_KERNEL);
	if (!bpmp->threaded_channels)
		return -ENOMEM;

	return 0;
}

int tegra_bpmp_init_ping(struct tegra_bpmp *bpmp)
{
	int err;

	err = tegra_bpmp_request_mrq(bpmp, MRQ_PING,
				     tegra_bpmp_mrq_handle_ping, bpmp);
	if (err < 0)
		return err;

	err = tegra_bpmp_ping(bpmp);
	if (err < 0) {
		dev_err(bpmp->dev, "failed to ping BPMP: %d\n", err);
		goto free_mrq;
	}

	return 0;

free_mrq:
	tegra_bpmp_free_mrq(bpmp, MRQ_PING, bpmp);

	return err;
}

void tegra_bpmp_deinit_ping(struct tegra_bpmp *bpmp)
{
	tegra_bpmp_free_mrq(bpmp, MRQ_PING, bpmp);
}

int tegra_bpmp_init_dt_providers(struct tegra_bpmp *bpmp)
{
	struct device *dev = bpmp->dev;
	int err;

	err = of_platform_default_populate(dev->of_node, NULL, dev);
	if (err < 0)
		return err;

	if (of_property_present(dev->of_node, "#clock-cells")) {
		err = tegra_bpmp_init_clocks(bpmp);
		if (err < 0)
			return err;
	}

	if (of_property_present(dev->of_node, "#reset-cells")) {
		err = tegra_bpmp_init_resets(bpmp);
		if (err < 0)
			return err;
	}

	if (of_property_present(dev->of_node, "#power-domain-cells")) {
		err = tegra_bpmp_init_powergates(bpmp);
		if (err < 0)
			return err;
	}

	return 0;
}

static int tegra_bpmp_probe(struct platform_device *pdev)
{
	struct tegra_bpmp *bpmp;
	char tag[TAG_SZ];
	int err;

	bpmp = devm_kzalloc(&pdev->dev, sizeof(*bpmp), GFP_KERNEL);
	if (!bpmp)
		return -ENOMEM;

	bpmp->soc = device_get_match_data(&pdev->dev);
	bpmp->dev = &pdev->dev;

	platform_set_drvdata(pdev, bpmp);

	err = bpmp->soc->ops->init(bpmp);
	if (err < 0)
		return err;

	err = tegra_bpmp_get_firmware_tag(bpmp, tag, sizeof(tag));
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get firmware tag: %d\n", err);
		goto deinit;
	}

	dev_info(&pdev->dev, "firmware: %.*s\n", (int)sizeof(tag), tag);

	if (bpmp->soc->ops->init_providers) {
		err = bpmp->soc->ops->init_providers(bpmp);
		if (err < 0)
			goto deinit;
	}

	err = tegra_bpmp_init_sysfs(bpmp);
	if (err < 0)
		dev_err(&pdev->dev, "failed to initialize sysfs: %d\n", err);

	err = tegra_bpmp_init_debugfs(bpmp);
	if (err < 0)
		dev_err(&pdev->dev, "debugfs initialization failed: %d\n", err);

	return 0;

deinit:
	if (bpmp->soc->ops->deinit)
		bpmp->soc->ops->deinit(bpmp);

	return err;
}

static int __maybe_unused tegra_bpmp_suspend(struct device *dev)
{
	struct tegra_bpmp *bpmp = dev_get_drvdata(dev);

	bpmp->suspended = true;

	return 0;
}

static int __maybe_unused tegra_bpmp_resume(struct device *dev)
{
	struct tegra_bpmp *bpmp = dev_get_drvdata(dev);

	bpmp->suspended = false;

	if (bpmp->soc->ops && bpmp->soc->ops->resume)
		return bpmp->soc->ops->resume(bpmp);
	else
		return 0;
}

static const struct dev_pm_ops tegra_bpmp_pm_ops = {
	.suspend_noirq = tegra_bpmp_suspend,
	.resume_noirq = tegra_bpmp_resume,
};

#if IS_ENABLED(CONFIG_ARCH_TEGRA_186_SOC) || \
    IS_ENABLED(CONFIG_ARCH_TEGRA_194_SOC) || \
    IS_ENABLED(CONFIG_ARCH_TEGRA_234_SOC) || \
    IS_ENABLED(CONFIG_ARCH_TEGRA_264_SOC)
static const struct tegra_bpmp_soc tegra186_soc = {
	.channels = {
		.cpu_tx = {
			.offset = 3,
			.timeout = 60 * USEC_PER_SEC,
		},
		.thread = {
			.offset = 0,
			.count = 3,
			.timeout = 600 * USEC_PER_SEC,
		},
		.cpu_rx = {
			.offset = 13,
			.timeout = 0,
		},
	},
	.ops = &tegra186_bpmp_ops,
	.num_resets = 193,
};
#endif

#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
static const struct tegra_bpmp_soc tegra210_soc = {
	.channels = {
		.cpu_tx = {
			.offset = 0,
			.count = 1,
			.timeout = 60 * USEC_PER_SEC,
		},
		.thread = {
			.offset = 4,
			.count = 1,
			.timeout = 600 * USEC_PER_SEC,
		},
		.cpu_rx = {
			.offset = 8,
			.count = 1,
			.timeout = 0,
		},
	},
	.ops = &tegra210_bpmp_ops,
};
#endif

static const struct of_device_id tegra_bpmp_match[] = {
#if IS_ENABLED(CONFIG_ARCH_TEGRA_186_SOC) || \
    IS_ENABLED(CONFIG_ARCH_TEGRA_194_SOC) || \
    IS_ENABLED(CONFIG_ARCH_TEGRA_234_SOC) || \
    IS_ENABLED(CONFIG_ARCH_TEGRA_264_SOC)
	{ .compatible = "nvidia,tegra186-bpmp", .data = &tegra186_soc },
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
	{ .compatible = "nvidia,tegra210-bpmp", .data = &tegra210_soc },
#endif
	{ }
};

#ifdef CONFIG_ACPI
#define TEGRA_BPMP_MBWT_VC_PCIE_READ	0
#define TEGRA_BPMP_MBWT_VC_PCIE_WRITE	1
#define TEGRA_BPMP_MBWT_VC_NVCLINK	2

static const struct tegra_bpmp_mbwt_vc tegra_bpmp_acpi_mbwt_vcs[] = {
	{ .name = "pcie_read", .type = TEGRA_BPMP_MBWT_VC_PCIE_READ },
	{ .name = "pcie_write", .type = TEGRA_BPMP_MBWT_VC_PCIE_WRITE },
	{ .name = "nvclink", .type = TEGRA_BPMP_MBWT_VC_NVCLINK },
};

static const struct tegra_bpmp_mbwt_group tegra_bpmp_acpi_mbwt_groups[] = {
	{
		.name = "pcie0",
		.id = 0,
		.vcs = tegra_bpmp_acpi_mbwt_vcs,
		.num_vcs = ARRAY_SIZE(tegra_bpmp_acpi_mbwt_vcs)
	}, {
		.name = "pcie1",
		.id = 1,
		.vcs = tegra_bpmp_acpi_mbwt_vcs,
		.num_vcs = ARRAY_SIZE(tegra_bpmp_acpi_mbwt_vcs)
	}, {
		.name = "pcie2",
		.id = 2,
		.vcs = tegra_bpmp_acpi_mbwt_vcs,
		.num_vcs = ARRAY_SIZE(tegra_bpmp_acpi_mbwt_vcs)
	}, {
		.name = "pcie3",
		.id = 3,
		.vcs = tegra_bpmp_acpi_mbwt_vcs,
		.num_vcs = ARRAY_SIZE(tegra_bpmp_acpi_mbwt_vcs)
	}, {
		.name = "pcie4",
		.id = 4,
		.vcs = tegra_bpmp_acpi_mbwt_vcs,
		.num_vcs = ARRAY_SIZE(tegra_bpmp_acpi_mbwt_vcs)
	}, {
		.name = "pcie5",
		.id = 5,
		.vcs = tegra_bpmp_acpi_mbwt_vcs,
		.num_vcs = ARRAY_SIZE(tegra_bpmp_acpi_mbwt_vcs)
	},
};

static const struct tegra_bpmp_mbwt_soc tegra_bpmp_acpi_mbwt_soc = {
	.groups = tegra_bpmp_acpi_mbwt_groups,
	.num_groups = ARRAY_SIZE(tegra_bpmp_acpi_mbwt_groups),
};

static int tegra_bpmp_acpi_init(struct tegra_bpmp *bpmp)
{
	return tegra_bpmp_ping(bpmp);
}

static const struct tegra_bpmp_ops tegra_bpmp_acpi_ops = {
	.init = tegra_bpmp_acpi_init,
};

static const struct tegra_bpmp_soc tegra_bpmp_acpi_soc = {
	.ops = &tegra_bpmp_acpi_ops,
	.mbwt = &tegra_bpmp_acpi_mbwt_soc,
};

static const struct acpi_device_id tegra_bpmp_acpi_match[] = {
	{
		.id = "NVDA3001",
		.driver_data = (kernel_ulong_t)&tegra_bpmp_acpi_soc,
	},
	{ }
};
#endif

static struct platform_driver tegra_bpmp_driver = {
	.driver = {
		.name = "tegra-bpmp",
		.of_match_table = tegra_bpmp_match,
#ifdef CONFIG_ACPI
		.acpi_match_table = tegra_bpmp_acpi_match,
#endif
		.pm = &tegra_bpmp_pm_ops,
		.suppress_bind_attrs = true,
	},
	.probe = tegra_bpmp_probe,
};
builtin_platform_driver(tegra_bpmp_driver);
