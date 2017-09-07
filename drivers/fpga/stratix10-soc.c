/*
 * FPGA Manager Driver for Intel Stratix10 SoC
 *
 *  Copyright (C) 2018 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/completion.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/intel-service-client.h>
#include <linux/module.h>
#include <linux/of.h>

/*
 * FPGA programming requires a higher level of privilege (EL3), per the SoC
 * design.
 */
#define NUM_SVC_BUFS	4
#define SVC_BUF_SIZE	SZ_512K

/* Indicates buffer is in use if set */
#define SVC_BUF_LOCK	0

/**
 * struct s10_svc_buf
 * @buf: virtual address of buf provided by service layer
 * @lock: locked if buffer is in use
 */
struct s10_svc_buf {
	char *buf;
	unsigned long lock;
};

struct s10_priv {
	struct intel_svc_chan *chan;
	struct intel_svc_client client;
	struct completion status_return_completion;
	struct s10_svc_buf svc_bufs[NUM_SVC_BUFS];
	unsigned long status;
};

static int s10_svc_send_msg(struct s10_priv *priv,
			    enum intel_svc_command_code command,
			    void *payload, u32 payload_length)
{
	struct intel_svc_chan *chan = priv->chan;
	struct intel_svc_client_msg msg;
	int ret;

	pr_debug("%s cmd=%d payload=%p legnth=%d\n",
		 __func__, command, payload, payload_length);

	msg.command = command;
	msg.payload = payload;
	msg.payload_length = payload_length;

	ret = intel_svc_send(chan, &msg);
	pr_debug("intel_svc_send returned status %d\n", ret);

	return ret;
}

/**
 * s10_free_buffers
 * Free buffers allocated from the service layer's pool that are not in use.
 * @mgr: fpga manager struct
 * Free all buffers that are not in use.
 * Return true when all buffers are freed.
 */
static bool s10_free_buffers(struct fpga_manager *mgr)
{
	struct s10_priv *priv = mgr->priv;
	uint num_free = 0;
	uint i;

	for (i = 0; i < NUM_SVC_BUFS; i++) {
		if (!priv->svc_bufs[i].buf) {
			num_free++;
			continue;
		}

		if (!test_and_set_bit_lock(SVC_BUF_LOCK,
					   &priv->svc_bufs[i].lock)) {
			intel_svc_free_memory(priv->chan,
					      priv->svc_bufs[i].buf);
			priv->svc_bufs[i].buf = NULL;
			num_free++;
		}
	}

	return num_free == NUM_SVC_BUFS;
}

/**
 * s10_free_buffer_count
 * Count how many buffers are not in use.
 * @mgr: fpga manager struct
 * Return # of buffers that are not in use.
 */
static uint s10_free_buffer_count(struct fpga_manager *mgr)
{
	struct s10_priv *priv = mgr->priv;
	uint num_free = 0;
	uint i;

	for (i = 0; i < NUM_SVC_BUFS; i++)
		if (!priv->svc_bufs[i].buf)
			num_free++;

	return num_free;
}

/**
 * s10_unlock_bufs
 * Given the returned buffer address, match that address to our buffer struct
 * and unlock that buffer.  This marks it as available to be refilled and sent
 * (or freed).
 * @priv: private data
 * @kaddr: kernel address of buffer that was returned from service layer
 */
static void s10_unlock_bufs(struct s10_priv *priv, void *kaddr)
{
	uint i;

	if (!kaddr)
		return;

	for (i = 0; i < NUM_SVC_BUFS; i++)
		if (priv->svc_bufs[i].buf == kaddr) {
			clear_bit_unlock(SVC_BUF_LOCK,
					 &priv->svc_bufs[i].lock);
			return;
		}

	WARN(1, "Unknown buffer returned from service layer %p\n", kaddr);
}

/**
 * s10_receive_callback
 * Callback for service layer to use to provide client (this driver) messages
 * received through the mailbox.
 * @client: service layer client struct
 * @data: message
 */
static void s10_receive_callback(struct intel_svc_client *client,
				 struct intel_svc_c_data *data)
{
	struct s10_priv *priv = client->priv;
	u32 status;
	int i;

	WARN_ONCE(!data, "%s: intel_svc_rc_data = NULL", __func__);

	status = data->status;

	/*
	 * Here we set status bits as we receive them.  Elsewhere, we always use
	 * test_and_clear_bit() to check status in priv->status
	 */
	for (i = 0; i <= SVC_STATUS_RECONFIG_ERROR; i++)
		if (status & (1 << i))
			set_bit(i, &priv->status);

	if (status & BIT(SVC_STATUS_RECONFIG_BUFFER_DONE)) {
		s10_unlock_bufs(priv, data->kaddr1);
		s10_unlock_bufs(priv, data->kaddr2);
		s10_unlock_bufs(priv, data->kaddr3);
	}

	complete(&priv->status_return_completion);
}

/**
 * s10_ops_write_init
 * Prepare for FPGA reconfiguration by requesting partial reconfig and
 * allocating buffers from the service layer.
 * @mgr: fpga manager
 * @info: fpga image info
 * @buf: fpga image buffer
 * @count: size of buf in bytes
 */
static int s10_ops_write_init(struct fpga_manager *mgr,
			      struct fpga_image_info *info,
			      const char *buf, size_t count)
{
	struct s10_priv *priv = mgr->priv;
	struct device *dev = priv->client.dev;
	unsigned long timeout;
	struct intel_command_reconfig_payload payload;
	char *kbuf;
	uint i;
	int ret;

	if (info->flags & FPGA_MGR_PARTIAL_RECONFIG) {
		dev_info(dev, "Requesting partial reconfiguration.\n");
		payload.flags |= BIT(COMMAND_RECONFIG_FLAG_PARTIAL);
	} else {
		dev_info(dev, "Requesting full reconfiguration.\n");
	}

	reinit_completion(&priv->status_return_completion);
	ret = s10_svc_send_msg(priv, COMMAND_RECONFIG,
			       &payload, sizeof(payload));
	if (ret < 0)
		return ret;

	timeout = msecs_to_jiffies(SVC_RECONFIG_REQUEST_TIMEOUT_MS);
	ret = wait_for_completion_interruptible_timeout(
		&priv->status_return_completion, timeout);
	if (!ret) {
		dev_err(dev, "timeout waiting for RECONFIG_REQUEST\n");
		return -ETIMEDOUT;
	}
	if (ret < 0) {
		dev_err(dev, "error (%d) waiting for RECONFIG_REQUEST\n", ret);
		return ret;
	}

	if (!test_and_clear_bit(SVC_STATUS_RECONFIG_REQUEST_OK,
				&priv->status))
		return -ETIMEDOUT;

	/* Allocate buffers from the service layer's pool. */
	for (i = 0; i < NUM_SVC_BUFS; i++) {
		kbuf = intel_svc_allocate_memory(priv->chan, SVC_BUF_SIZE);
		if (!kbuf) {
			s10_free_buffers(mgr);
			return -ENOMEM;
		}

		priv->svc_bufs[i].buf = kbuf;
		priv->svc_bufs[i].lock = 0;
	}

	return 0;
}

/**
 * s10_send_buf
 * Send a buffer to the service layer queue
 * @mgr: fpga manager struct
 * @buf_num: index of buffer in svc_bufs array
 * @buf: fpga image buffer
 * @count: size of buf in bytes
 * Returns # of bytes transferred or -errno, never 0
 */
static int s10_send_buf(struct fpga_manager *mgr, uint buf_num,
			const char *buf, size_t count)

{
	struct s10_priv *priv = mgr->priv;
	struct device *dev = priv->client.dev;
	void *svc_buf;
	size_t xfer_sz;
	int ret;

	xfer_sz = count < SVC_BUF_SIZE ? count : SVC_BUF_SIZE;

	svc_buf = priv->svc_bufs[buf_num].buf;
	memcpy(svc_buf, buf, xfer_sz);
	ret = s10_svc_send_msg(priv, COMMAND_RECONFIG_DATA_SUBMIT,
			       svc_buf, xfer_sz);
	if (ret < 0) {
		dev_err(dev,
			"Error while sending data to service layer (%d)", ret);
		return ret;
	}

	return xfer_sz;
}

/**
 * s10_ops_write
 * Send a FPGA image to privileged layers to write to the FPGA.  When done
 * sending, free all service layer buffers we allocated in write_init.
 * @mgr: fpga manager
 * @buf: fpga image buffer
 * @count: size of buf in bytes
 * Returns 0 for success or negative errno.
 */
static int s10_ops_write(struct fpga_manager *mgr, const char *buf,
			 size_t count)
{
	struct s10_priv *priv = mgr->priv;
	struct device *dev = priv->client.dev;
	unsigned long timeout;
	size_t sent = 0;
	int ret = 0;
	uint i;

	timeout = msecs_to_jiffies(SVC_RECONFIG_BUFFER_TIMEOUT_MS);

	/* Buffer loop: either send buffers or free them. */
	while (1) {
		reinit_completion(&priv->status_return_completion);

		if (count > 0) {
			for (i = 0; i < NUM_SVC_BUFS; i++)
				if (!test_and_set_bit_lock(
					 SVC_BUF_LOCK, &priv->svc_bufs[i].lock))
					break;

			if (i == NUM_SVC_BUFS)
				/* wait for a free buffer */
				continue;

			sent = s10_send_buf(mgr, i, buf, count);
			/*
			 * If service queue was full, we won't get a callback.
			 * Wait and try again
			 */
			if (sent < 0)
				continue;

			count -= sent;
			buf += sent;
		} else {
			s10_free_buffers(mgr);
			if (s10_free_buffer_count(mgr) == NUM_SVC_BUFS)
				return 0;

			ret = s10_svc_send_msg(
				priv, COMMAND_RECONFIG_DATA_CLAIM,
				NULL, 0);
			if (ret < 0)
				break;
		}

		/*
		 * If callback hasn't already happened, wait for buffers to be
		 * returned from service layer
		 */
		if (priv->status)
			ret = 0;
		else
			ret = wait_for_completion_interruptible_timeout(
				&priv->status_return_completion, timeout);

		if (test_and_clear_bit(
				SVC_STATUS_RECONFIG_BUFFER_DONE, &priv->status))
			continue;

		if (test_and_clear_bit(SVC_STATUS_RECONFIG_BUFFER_SUBMITTED,
				       &priv->status))
			continue;

		if (test_and_clear_bit(SVC_STATUS_RECONFIG_ERROR,
				       &priv->status)) {
			dev_err(dev, "ERROR - giving up - SVC_STATUS_RECONFIG_ERROR\n");
			ret = -EFAULT;
			break;
		}

		if (!ret) {
			dev_err(dev, "timeout waiting for svc layer buffers\n");
			ret = -ETIMEDOUT;
			break;
		}
		if (ret < 0) {
			dev_err(dev,
				"error (%d) waiting for svc layer buffers\n",
				ret);
			break;
		}
	}

	s10_free_buffers(mgr);
	if (s10_free_buffer_count(mgr) != NUM_SVC_BUFS)
		dev_err(dev, "%s not all buffers were freed\n", __func__);

	return ret;
}

/**
 * s10_ops_write_complete
 * Wait for FPGA configuration to be done
 * @mgr: fpga manager
 * @info: fpga image info
 * Returns 0 for success negative errno.
 */
static int s10_ops_write_complete(struct fpga_manager *mgr,
				  struct fpga_image_info *info)
{
	struct s10_priv *priv = mgr->priv;
	struct device *dev = priv->client.dev;
	unsigned long timeout;
	int ret;

	timeout = usecs_to_jiffies(info->config_complete_timeout_us);

	do {
		reinit_completion(&priv->status_return_completion);

		ret = s10_svc_send_msg(priv, COMMAND_RECONFIG_STATUS, NULL, 0);
		if (ret < 0)
			return ret;

		ret = wait_for_completion_interruptible_timeout(
			&priv->status_return_completion, timeout);
		if (!ret) {
			dev_err(dev,
				"timeout waiting for RECONFIG_COMPLETED\n");
			return -ETIMEDOUT;
		}
		if (ret < 0) {
			dev_err(dev,
				"error (%d) waiting for RECONFIG_COMPLETED\n",
				ret);
			return ret;
		}
		/* Not error or timeout, so ret is # of jiffies until timeout */
		timeout = ret;

		if (test_and_clear_bit(SVC_STATUS_RECONFIG_COMPLETED,
				       &priv->status))
			break;

		if (test_and_clear_bit(SVC_STATUS_RECONFIG_ERROR,
				       &priv->status)) {
			dev_err(dev, "ERROR - giving up - SVC_STATUS_RECONFIG_ERROR\n");
			return -EFAULT;
		}
	} while (1);

	return 0;
}

static enum fpga_mgr_states s10_ops_state(struct fpga_manager *mgr)
{
	return FPGA_MGR_STATE_UNKNOWN;
}

static const struct fpga_manager_ops s10_ops = {
	.state = s10_ops_state,
	.write_init = s10_ops_write_init,
	.write = s10_ops_write,
	.write_complete = s10_ops_write_complete,
};

static int s10_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s10_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client.dev = dev;
	priv->client.receive_cb = s10_receive_callback;
	priv->client.priv = priv;

	priv->chan = request_svc_channel_byname(&priv->client,
						SVC_CLIENT_FPGA);
	if (IS_ERR(priv->chan)) {
		dev_err(dev, "couldn't get service channel (%s)\n",
			SVC_CLIENT_FPGA);
		return PTR_ERR(priv->chan);
	}

	init_completion(&priv->status_return_completion);

	ret = fpga_mgr_register(dev, "Stratix10 SOC FPGA Manager",
				&s10_ops, priv);

	if (ret)
		free_svc_channel(priv->chan);

	return ret;
}

static int s10_remove(struct platform_device *pdev)
{
	struct fpga_manager *mgr = platform_get_drvdata(pdev);
	struct s10_priv *priv = mgr->priv;

	fpga_mgr_unregister(&pdev->dev);
	free_svc_channel(priv->chan);

	return 0;
}

static const struct of_device_id s10_of_match[] = {
	{ .compatible = "intel,stratix10-soc-fpga-mgr", },
	{},
};

MODULE_DEVICE_TABLE(of, s10_of_match);

static struct platform_driver s10_driver = {
	.probe = s10_probe,
	.remove = s10_remove,
	.driver = {
		.name	= "Stratix10 SoC FPGA manager",
		.of_match_table = of_match_ptr(s10_of_match),
	},
};

module_platform_driver(s10_driver);

MODULE_AUTHOR("Alan Tull <atull@kernel.org>");
MODULE_DESCRIPTION("Intel Stratix 10 SOC FPGA Manager");
MODULE_LICENSE("GPL v2");
