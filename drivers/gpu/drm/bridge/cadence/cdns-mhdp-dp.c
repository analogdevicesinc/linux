// SPDX-License-Identifier: GPL-2.0-or-later

#include <asm/unaligned.h>
#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_print.h>
#include <linux/io.h>

#define LINK_TRAINING_TIMEOUT_MS	500
#define LINK_TRAINING_RETRY_MS		20

int cdns_mhdp_dpcd_read(struct cdns_mhdp_device *mhdp,
			u32 addr, u8 *data, u16 len)
{
	u8 msg[5], reg[5];
	int ret;

	put_unaligned_be16(len, msg);
	put_unaligned_be24(addr, msg + 2);

	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
				     DPTX_READ_DPCD, sizeof(msg), msg);
	if (ret)
		goto err_dpcd_read;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_DP_TX,
						 DPTX_READ_DPCD,
						 sizeof(reg) + len);
	if (ret)
		goto err_dpcd_read;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, reg, sizeof(reg));
	if (ret)
		goto err_dpcd_read;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, data, len);

err_dpcd_read:
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_dpcd_read);

int cdns_mhdp_i2c_read(struct cdns_mhdp_device *mhdp, u8 addr, u8 *data,
	 u16 len, u8 mot, u16 *respLength)
{
	u8 msg[5], reg[3];
	int ret;

	put_unaligned_be16(len, msg);
	msg[2] = addr;
	msg[3] = mot;

	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
				     DPTX_I2C_READ, sizeof(msg), msg);
	if (ret)
		goto err_i2c_read;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_DP_TX,
						 DPTX_I2C_READ,
						 sizeof(reg) + len);
	if (ret)
		goto err_i2c_read;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, reg, sizeof(reg));
	if (ret)
		goto err_i2c_read;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, data, len);
	*respLength = (reg[0] << 8u) + reg[1];

err_i2c_read:
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "i2c read failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_i2c_read);

int cdns_mhdp_dpcd_write(struct cdns_mhdp_device *mhdp, u32 addr, u8 value)
{
	u8 msg[6], reg[5];
	int ret;

	put_unaligned_be16(1, msg);
	put_unaligned_be24(addr, msg + 2);
	msg[5] = value;

	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
				     DPTX_WRITE_DPCD, sizeof(msg), msg);
	if (ret)
		goto err_dpcd_write;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_DP_TX,
						 DPTX_WRITE_DPCD, sizeof(reg));
	if (ret)
		goto err_dpcd_write;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, reg, sizeof(reg));
	if (ret)
		goto err_dpcd_write;

	if (addr != get_unaligned_be24(reg + 2))
		ret = -EINVAL;

err_dpcd_write:
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "dpcd write failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_dpcd_write);

int cdns_mhdp_i2c_write(struct cdns_mhdp_device *mhdp, u8 addr, u8 *value,
	 u8 mot, u16 len, u16 *respLength)
{
	u8 msg[4+DP_AUX_MAX_PAYLOAD_BYTES], reg[3];
	int ret;

	put_unaligned_be16(len, msg);
	msg[2] = addr;
	msg[3] = mot;
	memcpy(&msg[4], value, len);


	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
				     DPTX_I2C_WRITE, sizeof(msg), msg);
	if (ret)
		goto err_i2c_write;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_DP_TX,
						 DPTX_I2C_WRITE, sizeof(reg));
	if (ret)
		goto err_i2c_write;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, reg, sizeof(reg));
	if (ret)
		goto err_i2c_write;

	if (addr != reg[2])
		ret = -EINVAL;

	*respLength = (reg[0]<<8u) + reg[1];

err_i2c_write:
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "i2c write failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_i2c_write);


int cdns_mhdp_get_last_i2c_status(struct cdns_mhdp_device *mhdp, u8 *resp)
{
	u8 status[1];
	int ret;

	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
				     DPTX_GET_LAST_I2C_STATUS, 0, NULL);
	if (ret)
		goto err_get_i2c_status;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_DP_TX,
						 DPTX_GET_LAST_I2C_STATUS,
						 sizeof(status));
	if (ret)
		goto err_get_i2c_status;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, status, sizeof(status));
	if (ret)
		goto err_get_i2c_status;

	*resp = status[0];

err_get_i2c_status:
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "get i2c status failed: %d\n",
			      ret);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_get_last_i2c_status);

static int cdns_mhdp_training_start(struct cdns_mhdp_device *mhdp)
{
	unsigned long timeout;
	u8 msg, event[2];
	int ret;

	msg = LINK_TRAINING_RUN;

	/* start training */
	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
				     DPTX_TRAINING_CONTROL, sizeof(msg), &msg);
	if (ret)
		goto err_training_start;

	timeout = jiffies + msecs_to_jiffies(LINK_TRAINING_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		msleep(LINK_TRAINING_RETRY_MS);
		ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
					     DPTX_READ_EVENT, 0, NULL);
		if (ret)
			goto err_training_start;

		ret = cdns_mhdp_mailbox_validate_receive(mhdp,
							 MB_MODULE_ID_DP_TX,
							 DPTX_READ_EVENT,
							 sizeof(event));
		if (ret)
			goto err_training_start;

		ret = cdns_mhdp_mailbox_read_receive(mhdp, event,
						     sizeof(event));
		if (ret)
			goto err_training_start;

		if (event[1] & CLK_RECOVERY_FAILED)
			DRM_DEV_ERROR(mhdp->dev, "clock recovery failed\n");
		else if (event[1] & EQ_PHASE_FINISHED)
			return 0;
	}

	ret = -ETIMEDOUT;

err_training_start:
	DRM_DEV_ERROR(mhdp->dev, "training failed: %d\n", ret);
	return ret;
}

static int cdns_mhdp_get_training_status(struct cdns_mhdp_device *mhdp)
{
	u8 status[13];
	int ret;

	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_DP_TX,
				     DPTX_READ_LINK_STAT, 0, NULL);
	if (ret)
		goto err_get_training_status;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_DP_TX,
						 DPTX_READ_LINK_STAT,
						 sizeof(status));
	if (ret)
		goto err_get_training_status;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, status, sizeof(status));
	if (ret)
		goto err_get_training_status;

	mhdp->dp.rate = drm_dp_bw_code_to_link_rate(status[0]);
	mhdp->dp.num_lanes = status[1];

err_get_training_status:
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "get training status failed: %d\n",
			      ret);
	return ret;
}

int cdns_mhdp_train_link(struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret = cdns_mhdp_training_start(mhdp);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to start training %d\n",
			      ret);
		return ret;
	}

	ret = cdns_mhdp_get_training_status(mhdp);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to get training stat %d\n",
			      ret);
		return ret;
	}

	DRM_DEV_DEBUG_KMS(mhdp->dev, "rate:0x%x, lanes:%d\n", mhdp->dp.rate,
			  mhdp->dp.num_lanes);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_train_link);
