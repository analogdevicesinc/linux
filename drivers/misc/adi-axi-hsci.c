// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices AXI HSCI interface
 *
 * Copyright 2023-2025 Analog Devices Inc.
 *
 * Wiki: https://wiki.analog.com/resources/fpga/docs/hsci
 */
//#define DEBUG
#include "linux/of.h"
#include <linux/mod_devicetable.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/unaligned.h>

/*
 * +----------+----------------------------+---------+-----------------------------------------------------------------------+
 * |  ADDRESS | REG NAME                   |  BITS   |  BITFIELD                                                             |
 * +----------+----------------------------+---------+-----------------------------------------------------------------------+
 * | 00000000 | REVISION_ID                |         |                                                                       |
 * |          |                            |  [15:0] | revision_id_hsci_revision_id                             (r)          |
 * | 00008001 | HSCI_MASTER_MODE           |         |                                                                       |
 * |          |                            |   [1:0] | hsci_master_mode_hsci_xfer_mode                        (r/w)          |
 * |          |                            |     [4] | ver_b__na                                              (r/w)          |
 * | 00008002 | HSCI_MASTER_XFER_NUM       |         |                                                                       |
 * |          |                            |  [15:0] | hsci_master_xfer_num_hsci_xfer_num                     (r/w)          |
 * | 00008003 | HSCI_MASTER_ADDR_SIZE      |         |                                                                       |
 * |          |                            |   [2:0] | hsci_master_addr_size_hsci_addr_size                   (r/w)          |
 * | 00008004 | HSCI_MASTER_BYTE_NUM       |         |                                                                       |
 * |          |                            |  [16:0] | hsci_master_byte_num_hsci_byte_num                     (r/w)          |
 * | 00008005 | HSCI_MASTER_TARGET         |         |                                                                       |
 * |          |                            |  [31:0] | hsci_master_target_spi_target                          (r/w)          |
 * | 00008006 | HSCI_MASTER_CTRL           |         |                                                                       |
 * |          |                            |   [1:0] | hsci_master_ctrl_hsci_cmd_sel                          (r/w)          |
 * |          |                            |   [5:4] | hsci_master_ctrl_hsci_slave_ahb_tsize                  (r/w)          |
 * | 00008007 | HSCI_MASTER_BRAM_ADDRESS   |         |                                                                       |
 * |          |                            |  [15:0] | hsci_master_bram_address_hsci_bram_start_address       (r/w)          |
 * | 00008008 | HSCI_MASTER_RUN            |         |                                                                       |
 * |          |                            |     [0] | hsci_master_run_hsci_run                               (r/w)          |
 * | 00008009 | HSCI_MASTER_STATUS         |         |                                                                       |
 * |          |                            |     [0] | hsci_master_status_master_done                           (r) Volatile |
 * |          |                            |     [1] | hsci_master_status_master_running                        (r) Volatile |
 * |          |                            |     [2] | hsci_master_status_master_wr_in_prog                     (r) Volatile |
 * |          |                            |     [3] | hsci_master_status_master_rd_in_prog                     (r) Volatile |
 * |          |                            |     [4] | hsci_master_status_miso_test_lfsr_acq                    (r) Volatile |
 * | 0000800a | HSCI_MASTER_LINKUP_CTRL    |         |                                                                       |
 * |          |                            |   [9:0] | hsci_master_linkup_ctrl_hsci_man_linkup_word           (r/w)          |
 * |          |                            |    [10] | hsci_master_linkup_ctrl_hsci_man_linkup                (r/w)          |
 * |          |                            |    [11] | hsci_master_linkup_ctrl_hsci_auto_linkup               (r/w)          |
 * |          |                            |    [12] | mosi_clk_inv                                           (r/w)          |
 * |          |                            |    [13] | miso_clk_inv                                           (r/w)          |
 * | 0000800b | HSCI_MASTER_TEST_CTRL      |         |                                                                       |
 * |          |                            |     [0] | hsci_master_test_ctrl_hsci_mosi_test_mode              (r/w)          |
 * |          |                            |     [1] | hsci_master_test_ctrl_hsci_miso_test_mode              (r/w)          |
 * |          |                            |     [2] | hsci_master_test_ctrl_hsci_capture_mode                (r/w)          |
 * | 0000800c | HSCI_MASTER_LINKUP_STATUS  |         |                                                                       |
 * |          |                            |     [0] | hsci_master_linkup_status_link_active                    (r) Volatile |
 * |          |                            |   [7:4] | hsci_master_linkup_status_alink_txclk_adj                (r) Volatile |
 * |          |                            |     [8] | hsci_master_linkup_status_alink_txclk_inv                (r) Volatile |
 * |          |                            |    [10] | txclk_adj_mismatch                                       (r) Volatile |
 * |          |                            |    [11] | txclk_inv_mismatch                                       (r) Volatile |
 * | 0000800d | HSCI_MASTER_LINKUP_STATUS2 |         |                                                                       |
 * |          |                            |  [15:0] | hsci_master_linkup_status2_alink_table                   (r) Volatile |
 * |          |                            | [19:16] | hsci_master_linkup_status2_alink_fsm                     (r) Volatile |
 * | 0000800e | HSCI_DEBUG_STATUS          |         |                                                                       |
 * |          |                            |   [3:0] | hsci_debug_status_enc_fsm                                (r) Volatile |
 * |          |                            |   [6:4] | hsci_debug_status_dec_fsm                                (r) Volatile |
 * |          |                            |  [17:8] | hsci_debug_status_capture_word                           (r) Volatile |
 * |          |                            |    [18] | hsci_debug_status_parity_error                           (r) Volatile |
 * |          |                            |    [19] | hsci_debug_status_unkown_instruction_error               (r) Volatile |
 * |          |                            | [27:20] | hsci_debug_status_slave_error_code                       (r) Volatile |
 * | 0000800f | MISO_TEST_BER              |         |                                                                       |
 * |          |                            |  [31:0] | miso_test_ber_miso_test_ber                              (r) Volatile |
 * | 00008010 | HSCI_MASTER_LINK_ERR_INFO  |         |                                                                       |
 * |          |                            |  [30:0] | hsci_link_err_info                                       (r) Volatile |
 * | 00008011 | HSCI_RATE_CTRL             |         |                                                                       |
 * |          |                            |     [0] | hsci_mmcm_drp_trig                                     (r/w)          |
 * |          |                            |   [2:1] | hsci_rate_sel                                          (r/w)          |
 * |          |                            |     [8] | hsci_pll_reset                                         (r/w)          |
 * | 00008012 | HSCI_MASTER_RST            |         |                                                                       |
 * |          |                            |     [4] | hsci_master_clear_errors                               (r/w)          |
 * |          |                            |     [8] | hsci_master_rstn                                       (r/w)          |
 * | 00008013 | HSCI_PHY_STATUS            |         |                                                                       |
 * |          |                            |     [0] | hsci_reset_seq_done                                      (r) Volatile |
 * |          |                            |     [1] | hsci_phy_pll_locked                                      (r) Volatile |
 * |          |                            |     [2] | hsci_vtc_rdy_tx                                          (r) Volatile |
 * |          |                            |     [3] | hsci_vtc_rdy_rx                                          (r) Volatile |
 * |          |                            |     [4] | hsci_dly_rdy_tx                                          (r) Volatile |
 * |          |                            |     [5] | hsci_dly_rdy_rx                                          (r) Volatile |
 * | 0000801f | HSCI_MASTER_SCRATCH        |         |                                                                       |
 * |          |                            |  [31:0] | scratch_reg                                            (r/w)          |
 * +----------+----------------------------+---------+-----------------------------------------------------------------------+
 */

/* Register map */
/* definition of register offset for hsci controller */
#define AXI_HSCI_REVID                  0x0000
#define AXI_HSCI_BUF_RDDATA             0x0001
#define AXI_HSCI_MODE                   0x8001
#define AXI_HSCI_XFER_NUM               0x8002
#define AXI_HSCI_ADDR_SIZE              0x8003
#define AXI_HSCI_BYTE_NUM               0x8004
#define AXI_HSCI_TARGET                 0x8005
#define AXI_HSCI_CTRL                   0x8006
#define AXI_HSCI_BRAM_ADDR              0x8007
#define AXI_HSCI_RUN                    0x8008
#define AXI_HSCI_STATUS                 0x8009
#define AXI_HSCI_LINKUP_CTRL            0x800A
#define AXI_HSCI_TEST_CTRL              0x800B
#define AXI_HSCI_LINKUP_STAT            0x800C
#define AXI_HSCI_LINKUP_STAT2           0x800D
#define AXI_HSCI_DEBUG_STAT             0x800E
#define AXI_MISO_TEST_BER               0x800F
#define AXI_HSCI_MASTER_LINK_ERR_INFO   0x8010
#define AXI_HSCI_RATE_CTRL_ADDR         0x8011
#define AXI_HSCI_MASTER_RST_ADDR        0x8012
#define AXI_HSCI_PHY_STATUS_ADDR        0x8013
#define AXI_HSCI_SCRATCH                0x801F
#define AXI_HSCI_BUF_RDDATA_SIZE        0x7FFF
#define AXI_HSCI_BUF_WRDATA_SIZE        (AXI_HSCI_BUF_RDDATA_SIZE - 4)

#define AXI_HSCI_RATE_CTRL_MSK          GENMASK(2, 1)
#define AXI_HSCI_LINKUP_CTRL_MOSI_CLK_INV       BIT(12)
#define AXI_HSCI_LINKUP_CTRL_MISO_CLK_INV       BIT(13)

#define HSCI_READ_LEN                   0xBEEF0001
#define HSCI_READ_CNT                   0xBEEF0002
#define HSCI_WRITE_LEN                  0xBEEF0003
#define HSCI_WRITE_CNT                  0xBEEF0004

/* Private driver data */

struct axi_hsci_state {
	struct device                           *dev;
	struct list_head                        entry;
	struct clk                              *pclk;
	void __iomem                            *regs;
	struct dentry                           *debugdir;

	/*
	 * The update lock is used to prevent races when updating
	 * partial registers, see axi_hsci_write_mask.
	 * Additionally, lifecycle changes are managed by this
	 * mutex.
	 */
	struct mutex                            update_lock;
	u32                                     version;
	struct kref                             kref;
	bool                                    initialized;
	bool                                    silent;
	struct device_node                      *of_node;
	u32                                     linkup_ctrl;
	u64                                     stat_read_len;
	u64                                     stat_read_cnt;
	u64                                     stat_write_len;
	u64                                     stat_write_cnt;
};

static LIST_HEAD(probed_devices);
static DEFINE_MUTEX(probed_devices_lock);
static int32_t axi_hsci_wait_done(struct axi_hsci_state *st);

/**
 * axi_hsci_write - Write a value to a register in the AXI HSCI device
 * @st: Pointer to the axi_hsci_state structure representing the device state
 * @reg: Register address to write to
 * @val: Value to write to the register
 *
 * This function writes a value to a register in the AXI HSCI device. It first prints a debug message
 * indicating the register address, physical address, and value being written. Then, it uses the iowrite32()
 * function to write the value to the register. If the debug flag is enabled, it reads the value from the
 * register and compares it with the value that was written. If the values do not match, it prints an error
 * message indicating the register address, physical address, and the expected and actual values.
 */
static void axi_hsci_write(struct axi_hsci_state *st, u32 reg, u32 val)
{
	dev_dbg(st->dev, "write: 0x%X (0x%px), 0x%X\n", reg, st->regs + (reg * 4),
		val);

	iowrite32(val, st->regs + (reg * 4));

	if (0) {
		u32 val2;

		if (reg == 0x8008)
			return;

		val2 = ioread32(st->regs + (reg * 4));

		if (val != val2)
			dev_err(st->dev, "FAILED write: 0x%X (0x%px), 0x%X!=0x%x\n", reg,
				st->regs + (reg * 4), val, val2);
	}
}

/**
 * axi_hsci_read - Read a value from a register in the AXI HSCI device
 * @st: Pointer to the AXI HSCI device state structure
 * @reg: Register offset to read from
 *
 * This function reads a value from a register in the AXI HSCI device.
 * It uses the ioread32() function to read the value from the device's
 * memory-mapped registers. The register offset is multiplied by 4 to
 * convert it to the byte offset.
 *
 * Returns the value read from the register.
 */
static u32 axi_hsci_read(struct axi_hsci_state *st, u32 reg)
{
	u32 val = ioread32(st->regs + (reg * 4));

	dev_dbg(st->dev, "read: 0x%X, 0x%X\n", reg, val);

	return val;
}

/**
 * axi_hsci_write_mask - Write a value to a register with a mask
 * @st: Pointer to the axi_hsci_state structure
 * @reg: Register address
 * @val: Value to be written
 * @mask: Mask to apply to the value before writing
 *
 * This function writes a value to a register with a mask. It first reads the
 * current value of the register, clears the bits specified by the mask, and
 * then sets the bits specified by the value and mask. Finally, it writes the
 * modified value back to the register.
 */
static void axi_hsci_write_mask(struct axi_hsci_state *st, u32 reg, u32 val,
				u32 mask)
{
	u32 data;

	data = axi_hsci_read(st, reg) & ~mask;
	data |= val & mask;

	axi_hsci_write(st, reg, data);
}

/**
 * axi_hsci_wait_link_active - Waits for the HSCI link to become active.
 * @st: Pointer to the HSCI state structure.
 *
 * This function polls the HSCI link status register to check if the link
 * is active. It reads the status register up to 1000 times or until the
 * link becomes active, whichever comes first.
 *
 * Return: 0 if the link becomes active, -EIO otherwise.
 */
static int axi_hsci_wait_link_active(struct axi_hsci_state *st)
{
	u32 cnt = 0, status = 0;
	int ret = -EIO;

	do {
		status = axi_hsci_read(st, AXI_HSCI_LINKUP_STAT);

		if ((status & 0x1) == 1)
			ret = 0;

	} while ((cnt++ < 1000) && (ret != 0));

	return ret;
}

/**
 * axi_hsci_silent - Enable or disable silent mode for AXI HSCI
 * @st: Pointer to the AXI HSCI state structure
 * @enable: Boolean value indicating whether to enable or disable silent mode
 *
 * This function is used to enable or disable silent mode for the AXI HSCI
 * driver. When silent mode is enabled, the driver will suppress debug and
 * informational messages. The @st parameter is a pointer to the AXI HSCI
 * state structure. The @enable parameter is a boolean value indicating
 * whether to enable or disable silent mode.
 */
void axi_hsci_silent(struct axi_hsci_state *st, bool enable)
{
	st->silent = enable;
}
EXPORT_SYMBOL_GPL(axi_hsci_silent);

/**
 * axi_hsci_manual_linkup - Manually control the link-up status of AXI HSCI
 * @st: Pointer to the AXI HSCI state structure
 * @enable: Flag to enable or disable link-up control (0 - disable, 1 - enable)
 * @link_up_signal_bits: Bitmask representing the link-up signal bits
 *
 * This function allows manual control of the link-up status of the AXI HSCI.
 * It sets the link-up control register with the specified enable flag and
 * link-up signal bits. The enable flag determines whether link-up control is
 * enabled or disabled, while the link-up signal bits represent the specific
 * bits to be set in the link-up control register.
 *
 * Return: Always returns 0.
 */
int axi_hsci_manual_linkup(struct axi_hsci_state *st, u8 enable,
			   uint16_t link_up_signal_bits)
{
	dev_dbg(st->dev, "%s:%d\n", __func__, __LINE__);

	axi_hsci_write(st, AXI_HSCI_LINKUP_CTRL, st->linkup_ctrl |
		       ((enable & 0x01) << 10) | ((link_up_signal_bits & 0x3FF)));

	return 0;
}
EXPORT_SYMBOL_GPL(axi_hsci_manual_linkup);

/**
 * axi_hsci_auto_linkup - Configures the automatic linkup for the HSCI interface.
 * @st: Pointer to the axi_hsci_state structure.
 * @enable: Enable or disable the automatic linkup (1 to enable, 0 to disable).
 * @hscim_mosi_clk_inv: Invert the MOSI clock (1 to invert, 0 to keep as is).
 * @hscim_miso_clk_inv: Invert the MISO clock (1 to invert, 0 to keep as is).
 *
 * This function configures the automatic linkup control register of the HSCI
 * interface based on the provided parameters. If the linkup is disabled, it
 * also performs a reset sequence on the HSCI master.
 *
 * Return: Always returns 0.
 */
int axi_hsci_auto_linkup(struct axi_hsci_state *st, u8 enable,
			 u8 hscim_mosi_clk_inv, u8 hscim_miso_clk_inv)
{
	dev_dbg(st->dev, "%s:%d\n", __func__, __LINE__);

	axi_hsci_write(st, AXI_HSCI_LINKUP_CTRL, ((enable & 0x01) << 11) |
		       ((hscim_mosi_clk_inv & 0x01) << 12) |
		       ((hscim_miso_clk_inv & 0x01) << 13));

	if (!enable) {
		axi_hsci_write(st, AXI_HSCI_MASTER_RST_ADDR, 0x00);
		axi_hsci_write(st, AXI_HSCI_MASTER_RST_ADDR, 0x100);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(axi_hsci_auto_linkup);

/**
 * axi_hsci_alink_tbl_get - Retrieve the ALINK table value.
 * @st: Pointer to the axi_hsci_state structure.
 * @hscim_alink_table: Pointer to a u16 variable where the ALINK table value will be stored.
 *
 * This function reads the ALINK table value from the hardware and stores it in the provided
 * variable. It first waits for the link to become active. If the link is active, it reads the
 * ALINK table value from the AXI_HSCI_LINKUP_STAT2 register and stores the lower 16 bits of the
 * read value in the provided variable. If the link is not active, it returns an error.
 *
 * Return: 0 on success, -EIO if the link is not active.
 */
int axi_hsci_alink_tbl_get(struct axi_hsci_state *st, u16 *hscim_alink_table)
{
	u32 reg_read;

	if (axi_hsci_wait_link_active(st) == 0) {
		reg_read = axi_hsci_read(st, AXI_HSCI_LINKUP_STAT2);
		*hscim_alink_table = reg_read & 0xFFFF;
	} else {
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(axi_hsci_alink_tbl_get);

/**
 * axi_hsci_readm - Read data from AXI HSCI device
 * @st: Pointer to the axi_hsci_state structure representing the device state
 * @tx_data: Pointer to the transmit data buffer
 * @rx_data: Pointer to the receive data buffer
 * @num_tx_rx_bytes: Number of bytes to transmit and receive
 * @addr_len: Address length in bytes
 * @data_len: Data length in bytes
 * @stream_len: Number of streams to read
 *
 * This function reads data from the AXI HSCI device. It takes the transmit data buffer,
 * receive data buffer, number of bytes to transmit and receive, address length, data length,
 * and number of streams as input parameters. It returns 0 on success and a negative error
 * code on failure.
 *
 * The function first checks if the address length is supported. If not, it returns an error.
 * It then calculates the total length of data to be read and updates the statistics.
 * If the data length is not supported, the function returns an error.
 *
 * The function then writes the transmit data to the BRAM (Block RAM) of the device and starts
 * the transaction. It waits for the transaction to complete and then reads the data from the BRAM.
 * If the number of streams is greater than or equal to 2, the function reads the data in chunks
 * of 4 bytes and stores it in the receive data buffer. If the data length is 1, 2, or 4 bytes,
 * the function reads the data accordingly and stores it in the receive data buffer.
 *
 * If the transaction is successful, the function updates the statistics and returns 0.
 * If the transaction fails or the device is busy, the function returns an error.
 *
 * Returns: 0 on success, negative error code on failure
 */
int axi_hsci_readm(struct axi_hsci_state *st, const u8 tx_data[],
		   u8 rx_data[], uint32_t num_tx_rx_bytes,
		   u8 addr_len,
		   u8 data_len,
		   uint32_t stream_len)
{
	int ret = -EFAULT;
	u32 addr = 0;
	u32 bram_addr = 0x1000;
	u32 rdData = 0;
	u32 i, word_size, byte_size, len;

	if (addr_len !=  4) {
		dev_err(st->dev, "HSCI addr size not supported\n");
		return -EINVAL;
	}

	len = data_len * stream_len;

	st->stat_read_len += len;

	if (len >= AXI_HSCI_BUF_RDDATA_SIZE) {
		dev_err(st->dev, "HSCI data size not supported\n");
		return -EINVAL;
	}

	addr = get_unaligned_le32(tx_data);

	dev_dbg(st->dev, "%s: addr 0x%x len %d\n", __func__, addr, len);

	mutex_lock(&st->update_lock);

	if (axi_hsci_wait_done(st) == 0) {
		//Write BRAM
		axi_hsci_write(st, AXI_HSCI_BRAM_ADDR, bram_addr);
		axi_hsci_write(st, bram_addr, addr); //Fill the buffer
		axi_hsci_write(st, AXI_HSCI_XFER_NUM, 1);
		axi_hsci_write(st, AXI_HSCI_ADDR_SIZE, 3); //32B Addr space supported
		axi_hsci_write(st, AXI_HSCI_BYTE_NUM,
			       len);  // bytes can be transferred for single transaction
		axi_hsci_write(st, AXI_HSCI_CTRL,
			       ((0 & 0x3) << 4) | (1 & 0x3)); //HSCI SLAVE AHB TSIZE and CMD SEL
		axi_hsci_write(st, AXI_HSCI_RUN, 1); //Start transaction

		if (axi_hsci_wait_done(st) == 0) {
			//Read BRAM
			if (stream_len >= 2) { //Stream read
				word_size = len / 4;
				byte_size = len % 4;
				for (i = 0; i < word_size; i++) {
					rdData = axi_hsci_read(st, AXI_HSCI_BUF_RDDATA + i);
					put_unaligned_be32(rdData, &rx_data[4 + (i * 4)]);
				}
				if (byte_size != 0)
					rdData = axi_hsci_read(st, AXI_HSCI_BUF_RDDATA + word_size);

				for (i = 0; i < byte_size; i++)
					rx_data[4 + (word_size * 4) + i] = (rdData >> (8 * i)) & 0xFF;

				ret = 0;
			} else {
				if (data_len == 1) { // 8-bit (1-Byte)
					rdData = axi_hsci_read(st, AXI_HSCI_BUF_RDDATA) & 0xFF;
					rx_data[4] = rdData;
					ret = 0;
				} else if (data_len == 2) { // 16-bits (2-Bytes)
					rdData = axi_hsci_read(st, AXI_HSCI_BUF_RDDATA) & 0xFFFF;
					put_unaligned_be16(rdData, &rx_data[4]);
					ret = 0;
				} else if (data_len == 4) { // 32-bits (4-Bytes)
					rdData = axi_hsci_read(st, AXI_HSCI_BUF_RDDATA);
					put_unaligned_be32(rdData, &rx_data[4]);
					ret = 0;
				} else {
					// To-Do: 48 -64 bit support.
					// Needed in Future.
					mutex_unlock(&st->update_lock);
					return -EFAULT;
				}
			}
		}
	} else {
		ret = -EBUSY;
		dev_err(st->dev, "HSCI Master busy. Transaction Failed!");
	}
	// dev_err(st->dev, "READ Data_32 %d, %d, %d: 0x%08X.\n", data_len, stream_len, err,  rdData);
	st->stat_read_cnt++;
	mutex_unlock(&st->update_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(axi_hsci_readm);

/**
 * axi_hsci_writem - Write data to the AXI HSCI device
 * @st: Pointer to the axi_hsci_state structure representing the device state
 * @tx_data: Array of bytes containing the data to be written
 * @num_tx_rx_bytes: Number of bytes to be transmitted and received
 * @addr_len: Length of the address in bytes
 * @data_len: Length of the data in bytes
 * @stream_len: Length of the stream in bytes
 *
 * This function writes data to the AXI HSCI device. It takes the device state,
 * the data to be written, the number of bytes to be transmitted and received,
 * the length of the address, the length of the data, and the length of the stream
 * as parameters. It returns 0 on success or a negative error code on failure.
 */
int axi_hsci_writem(struct axi_hsci_state *st, const u8 tx_data[],
		    u32 num_tx_rx_bytes,
		    u8 addr_len,
		    u8 data_len,
		    uint32_t stream_len)
{
	int ret = -EFAULT;
	u32 addr = 0;
	u32 bram_addr = 0x0001;
	u32 wrData = 0;
	u32 i, word_size, byte_size, len;

	if (addr_len !=  4) {
		dev_err(st->dev, "HSCI addr size not supported\n");
		return -EINVAL;
	}

	len = data_len * stream_len;
	st->stat_write_len += len;

	if (len >=  AXI_HSCI_BUF_WRDATA_SIZE) {
		dev_err(st->dev, "HSCI data size not supported\n");
		return -EINVAL;
	}

	addr = get_unaligned_le32(tx_data);

	dev_dbg(st->dev, "%s: addr 0x%x len %d\n", __func__, addr, len);

	mutex_lock(&st->update_lock);

	if (axi_hsci_wait_done(st) == 0) {
		//Write BRAM
		axi_hsci_write(st, AXI_HSCI_BRAM_ADDR, bram_addr);
		axi_hsci_write(st, bram_addr, addr); //Fill the buffer with addr
		bram_addr = bram_addr + 1; // bram addr for data

		if (stream_len >= 2) { //Stream read
			word_size = len / 4;
			byte_size = len % 4;
			for (i = 0; i < word_size; i++) {
				wrData = get_unaligned_le32(&tx_data[4 + (i * 4)]);
				axi_hsci_write(st, bram_addr + i, wrData);
			}
			if (word_size != 0) {
				axi_hsci_write(st, AXI_HSCI_XFER_NUM, 1);
				axi_hsci_write(st, AXI_HSCI_ADDR_SIZE, 3); //32B Addr space supported
				axi_hsci_write(st, AXI_HSCI_BYTE_NUM,
					       (word_size * 4));  // bytes can be transferred for single transaction
				axi_hsci_write(st, AXI_HSCI_CTRL,
					       ((1 & 0x3) << 4) | (0 & 0x3)); //HSCI SLAVE AHB TSIZE and CMD SEL
				axi_hsci_write(st, AXI_HSCI_RUN, 1); //Start transaction
				if (axi_hsci_wait_done(st) == 0)
					ret = 0;
			}
			wrData = 0;
			for (i = 0; i < byte_size; i++)
				wrData |= (tx_data[4 + (word_size * 4) + i] << (8 * i));

			if (byte_size != 0) {
				axi_hsci_write(st, AXI_HSCI_BRAM_ADDR, bram_addr);
				axi_hsci_write(st, bram_addr,
					       addr + (word_size * 4)); //Fill the buffer with addr
				axi_hsci_write(st, bram_addr + 1, wrData);
				axi_hsci_write(st, AXI_HSCI_XFER_NUM, 1);
				axi_hsci_write(st, AXI_HSCI_ADDR_SIZE, 3); //32B Addr space supported
				axi_hsci_write(st, AXI_HSCI_BYTE_NUM,
					       byte_size);  // bytes can be transferred for single transaction
				axi_hsci_write(st, AXI_HSCI_CTRL,
					       ((0 & 0x3) << 4) | (0 & 0x3)); //HSCI SLAVE AHB TSIZE and CMD SEL
				axi_hsci_write(st, AXI_HSCI_RUN, 1); //Start transaction
				if (axi_hsci_wait_done(st) == 0)
					ret = 0;
			}
		} else {
			if (data_len == 1) { // 8-bit (1-Byte)
				wrData = tx_data[4];
				axi_hsci_write(st, bram_addr, wrData);
			} else if (data_len == 2) { // 16-bits (2-Bytes)
				wrData = get_unaligned_le16(&tx_data[4]);
				axi_hsci_write(st, bram_addr, wrData);
			} else if (data_len == 4) { // 32-bits (4-Bytes)
				wrData = get_unaligned_le32(&tx_data[4]);
				axi_hsci_write(st, bram_addr, wrData);
			} else {
				// To-Do: 48 -64 bit support.
				// Needed in Future.
				mutex_unlock(&st->update_lock);
				return -EINVAL;
			}
			axi_hsci_write(st, AXI_HSCI_XFER_NUM, 1);
			axi_hsci_write(st, AXI_HSCI_ADDR_SIZE, 3); //32B Addr space supported
			axi_hsci_write(st, AXI_HSCI_BYTE_NUM,
				       data_len);  // bytes can be transferred for single transaction
			axi_hsci_write(st, AXI_HSCI_CTRL,
				       ((0 & 0x3) << 4) | (0 & 0x3)); //HSCI SLAVE AHB TSIZE and CMD SEL
			axi_hsci_write(st, AXI_HSCI_RUN, 1); //Start transaction
			if (axi_hsci_wait_done(st) == 0)
				ret = 0;
		}
	} else {
		ret = -EBUSY;
		dev_err(st->dev, "HSCI Master busy. Transaction Failed!");
	}

	st->stat_write_cnt++;
	mutex_unlock(&st->update_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(axi_hsci_writem);

struct axi_hsci_info {
	u32 version;
};

static const struct axi_hsci_info axi_hsci_1_0_a_info = {
	.version = 1,
};

/* Match table for of_platform binding */
static const struct of_device_id axi_hsci_of_match[] = {
	{ .compatible = "adi,axi-hsci-1.0.a", .data = &axi_hsci_1_0_a_info },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, axi_hsci_of_match);

/**
 * axi_hsci_dbg_get - Get debug information from AXI HSCI device
 * @data: Pointer to the AXI HSCI device state structure
 * @val: Pointer to store the retrieved value
 * @offset: Offset of the debug information to retrieve
 * @mask: Mask to apply to the retrieved value
 * @shift: Number of bits to shift the retrieved value
 *
 * This function is used to retrieve debug information from the AXI HSCI device.
 * The debug information is specified by the @offset parameter, which determines
 * which debug information to retrieve. The retrieved value is stored in the
 * memory pointed to by @val.
 *
 * Return: 0 on success, negative error code on failure
 */
static int axi_hsci_dbg_get(void *data, u64 *val, size_t offset, u32 mask,
			    int shift)
{
	struct axi_hsci_state *st = data;
	u64 rdata;

	mutex_lock(&st->update_lock);

	switch (offset) {
	case HSCI_READ_LEN:
		rdata = st->stat_read_len;
		break;
	case HSCI_READ_CNT:
		rdata = st->stat_read_cnt;
		break;
	case HSCI_WRITE_LEN:
		rdata = st->stat_write_len;
		break;
	case HSCI_WRITE_CNT:
		rdata = st->stat_write_cnt;
		break;
	default:
		rdata = (axi_hsci_read(st, offset) & mask) >> shift;
	}
	mutex_unlock(&st->update_lock);

	*val = rdata;

	return 0;
}

/**
 * axi_hsci_dbg_set - Set a debug value in the AXI HSCI device
 * @data: Pointer to the AXI HSCI device state structure
 * @val: Value to be set
 * @offset: Offset of the register to be modified
 * @mask: Bit mask to apply to the register value
 * @shift: Number of bits to shift the value before applying the mask
 *
 * This function sets a debug value in the AXI HSCI device. It takes a pointer to
 * the AXI HSCI device state structure, the value to be set, the offset of the
 * register to be modified, the bit mask to apply to the register value, and the
 * number of bits to shift the value before applying the mask.
 *
 * If the input value is greater than UINT_MAX, indicating that it doesn't fit
 * into 32 bits, the function returns -EINVAL.
 *
 * The function acquires the update lock to ensure exclusive access to the device
 * state. It then writes the modified value to the specified register using the
 * provided offset, mask, and shift. Finally, it releases the update lock.
 *
 * Return: 0 on success, -EINVAL if the input value is too large
 */
static int axi_hsci_dbg_set(void *data, u64 val, size_t offset, u32 mask,
			    int shift)
{
	struct axi_hsci_state *st = data;
	u32 n = val;

	/* Input value didn't fit into 32 bit */
	if (val > UINT_MAX)
		return -EINVAL;

	mutex_lock(&st->update_lock);
	axi_hsci_write_mask(st, offset, n << shift, mask);
	mutex_unlock(&st->update_lock);

	return 0;
}

struct axi_do_dbg_attr {
	char                            *name;
	umode_t                         mode;
	const struct file_operations    *fops;
	int (*get)(void *t, u64 *v);
};

static struct dentry *axi_hsci_dbg_parent;

#define ADI_REG_DEVICE_ATTR(_name, _mode, _off, _mask, _shift, _fmt)    \
static int axi_hsci_dbg_ ## _name ## _get(void *data, u64 *val)         \
{                                                                       \
        return axi_hsci_dbg_get(data, val, _off, _mask, _shift);        \
}                                                                       \
static int axi_hsci_dbg_ ## _name ## _set(void *data, u64 val)          \
{                                                                       \
        return axi_hsci_dbg_set(data, val, _off, _mask, _shift);        \
}                                                                       \
DEFINE_DEBUGFS_ATTRIBUTE(axi_hsci_dbg_ ## _name,                        \
                         axi_hsci_dbg_ ## _name ## _get,                \
                         axi_hsci_dbg_ ## _name ## _set,                \
                         _fmt);                                         \
static struct axi_do_dbg_attr axi_hsci_dbg_ ## _name ## _attr = {       \
        .name   = __stringify(_name),                                   \
        .mode   = _mode,                                                \
        .fops   = &axi_hsci_dbg_ ## _name,                              \
        .get    = &axi_hsci_dbg_ ## _name ## _get                       \
}

ADI_REG_DEVICE_ATTR(revision_id_hsci_revision_id, 0444, AXI_HSCI_REVID,
		    GENMASK(15, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(_hsci_xfer_mode, 0644, AXI_HSCI_MODE, GENMASK(1, 0), 0,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(_ver_b_na, 0644, AXI_HSCI_MODE, BIT(4), 4, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_xfer_num_hsci_xfer_num, 0644, AXI_HSCI_XFER_NUM,
		    GENMASK(15, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_addr_size_hsci_addr_size, 0644,
		    AXI_HSCI_ADDR_SIZE, GENMASK(2, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_byte_num_hsci_byte_num, 0644, AXI_HSCI_BYTE_NUM,
		    GENMASK(16, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_target_spi_target, 0644, AXI_HSCI_TARGET,
		    GENMASK(31, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_ctrl_hsci_cmd_sel, 0644, AXI_HSCI_CTRL,
		    GENMASK(1, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_ctrl_hsci_slave_ahb_tsize, 0644, AXI_HSCI_CTRL,
		    GENMASK(5, 4), 4, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_bram_address_hsci_bram_start_address, 0644,
		    AXI_HSCI_BRAM_ADDR, GENMASK(15, 0), 0, "0x%llX\n");
ADI_REG_DEVICE_ATTR(hsci_master_run_hsci_run, 0644, AXI_HSCI_RUN, BIT(0), 0,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_status_master_done, 0444, AXI_HSCI_STATUS,
		    BIT(0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_status_master_running, 0444, AXI_HSCI_STATUS,
		    BIT(1), 1, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_status_master_wr_in_prog, 0444, AXI_HSCI_STATUS,
		    BIT(2), 2, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_status_master_rd_in_prog, 0444, AXI_HSCI_STATUS,
		    BIT(3), 3, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_status_miso_test_lfsr_acq, 0444,
		    AXI_HSCI_STATUS, BIT(4), 4, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_ctrl_hsci_man_linkup_word, 0644,
		    AXI_HSCI_LINKUP_CTRL, GENMASK(9, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_ctrl_hsci_man_linkup, 0644,
		    AXI_HSCI_LINKUP_CTRL, BIT(10), 10, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_ctrl_hsci_auto_linkup, 0644,
		    AXI_HSCI_LINKUP_CTRL, BIT(11), 11, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_ctrl_mosi_clk_inv, 0644,
		    AXI_HSCI_LINKUP_CTRL, BIT(12), 12, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_ctrl_miso_clk_inv, 0644,
		    AXI_HSCI_LINKUP_CTRL, BIT(13), 13, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_test_ctrl_hsci_mosi_test_mode, 0644,
		    AXI_HSCI_TEST_CTRL, BIT(0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_test_ctrl_hsci_miso_test_mode, 0644,
		    AXI_HSCI_TEST_CTRL, BIT(1), 1, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_test_ctrl_hsci_capture_mode, 0644,
		    AXI_HSCI_TEST_CTRL, BIT(2), 2, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_status_link_active, 0444,
		    AXI_HSCI_LINKUP_STAT, BIT(0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_status_alink_txclk_adj, 0444,
		    AXI_HSCI_LINKUP_STAT, GENMASK(7, 4), 4, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_status_alink_txclk_inv, 0444,
		    AXI_HSCI_LINKUP_STAT, BIT(8), 8, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_status_txclk_adj_mismatch, 0444,
		    AXI_HSCI_LINKUP_STAT, BIT(10), 10, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_status_txclk_inv_mismatch, 0444,
		    AXI_HSCI_LINKUP_STAT, BIT(11), 11, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_status2_alink_table, 0444,
		    AXI_HSCI_LINKUP_STAT2, GENMASK(15, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_linkup_status2_alink_fsm, 0444,
		    AXI_HSCI_LINKUP_STAT2, GENMASK(19, 16), 16, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_debug_status_enc_fsm, 0444, AXI_HSCI_DEBUG_STAT,
		    GENMASK(3, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_debug_status_dec_fsm, 0444, AXI_HSCI_DEBUG_STAT,
		    GENMASK(6, 4), 4, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_debug_status_capture_word, 0444, AXI_HSCI_DEBUG_STAT,
		    GENMASK(17, 8), 8, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_debug_status_parity_error, 0444, AXI_HSCI_DEBUG_STAT,
		    BIT(18), 18, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_debug_status_unkown_instruction_error, 0444,
		    AXI_HSCI_DEBUG_STAT, BIT(19), 19, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_debug_status_slave_error_code, 0444,
		    AXI_HSCI_DEBUG_STAT, GENMASK(27, 20), 20, "%llu\n");
ADI_REG_DEVICE_ATTR(miso_test_ber_miso_test_ber, 0444, AXI_MISO_TEST_BER,
		    GENMASK(31, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_link_err_info, 0444,
		    AXI_HSCI_MASTER_LINK_ERR_INFO, GENMASK(30, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_mmcm_drp_trig, 0644, AXI_HSCI_RATE_CTRL_ADDR, BIT(0),
		    0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_rate_sel, 0644, AXI_HSCI_RATE_CTRL_ADDR,
		    BIT(1) | BIT(2), 1, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_pll_reset, 0644, AXI_HSCI_RATE_CTRL_ADDR, BIT(8), 8,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_clear_errors, 0644, AXI_HSCI_MASTER_RST_ADDR,
		    BIT(4), 4, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_rstn, 0644, AXI_HSCI_MASTER_RST_ADDR, BIT(8), 8,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_reset_seq_done, 0444, AXI_HSCI_PHY_STATUS_ADDR, BIT(0),
		    0, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_phy_pll_locked, 0444, AXI_HSCI_PHY_STATUS_ADDR, BIT(1),
		    1, "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_vtc_rdy_tx, 0444, AXI_HSCI_PHY_STATUS_ADDR, BIT(2), 2,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_vtc_rdy_rx, 0444, AXI_HSCI_PHY_STATUS_ADDR, BIT(3), 3,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_dly_rdy_tx, 0444, AXI_HSCI_PHY_STATUS_ADDR, BIT(4), 4,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_dly_rdy_rx, 0444, AXI_HSCI_PHY_STATUS_ADDR, BIT(5), 5,
		    "%llu\n");
ADI_REG_DEVICE_ATTR(hsci_master_scratch_scratch_reg, 0444, AXI_HSCI_SCRATCH,
		    GENMASK(31, 0), 0, "%llu\n");
ADI_REG_DEVICE_ATTR(stat_read_len, 0444, HSCI_READ_LEN, 0, 0, "%llu\n");
ADI_REG_DEVICE_ATTR(stat_read_count, 0444, HSCI_READ_CNT, 0, 0, "%llu\n");
ADI_REG_DEVICE_ATTR(stat_write_len, 0444, HSCI_WRITE_LEN, 0, 0, "%llu\n");
ADI_REG_DEVICE_ATTR(stat_write_count, 0444, HSCI_WRITE_CNT, 0, 0, "%llu\n");

static struct axi_do_dbg_attr *axi_hsci_dbg_attrs[] = {
	&axi_hsci_dbg_revision_id_hsci_revision_id_attr,
	&axi_hsci_dbg__hsci_xfer_mode_attr,
	&axi_hsci_dbg__ver_b_na_attr,
	&axi_hsci_dbg_hsci_master_xfer_num_hsci_xfer_num_attr,
	&axi_hsci_dbg_hsci_master_addr_size_hsci_addr_size_attr,
	&axi_hsci_dbg_hsci_master_byte_num_hsci_byte_num_attr,
	&axi_hsci_dbg_hsci_master_target_spi_target_attr,
	&axi_hsci_dbg_hsci_master_ctrl_hsci_cmd_sel_attr,
	&axi_hsci_dbg_hsci_master_ctrl_hsci_slave_ahb_tsize_attr,
	&axi_hsci_dbg_hsci_master_bram_address_hsci_bram_start_address_attr,
	&axi_hsci_dbg_hsci_master_run_hsci_run_attr,
	&axi_hsci_dbg_hsci_master_status_master_done_attr,
	&axi_hsci_dbg_hsci_master_status_master_running_attr,
	&axi_hsci_dbg_hsci_master_status_master_wr_in_prog_attr,
	&axi_hsci_dbg_hsci_master_status_master_rd_in_prog_attr,
	&axi_hsci_dbg_hsci_master_status_miso_test_lfsr_acq_attr,
	&axi_hsci_dbg_hsci_master_linkup_ctrl_hsci_man_linkup_word_attr,
	&axi_hsci_dbg_hsci_master_linkup_ctrl_hsci_man_linkup_attr,
	&axi_hsci_dbg_hsci_master_linkup_ctrl_hsci_auto_linkup_attr,
	&axi_hsci_dbg_hsci_master_linkup_ctrl_mosi_clk_inv_attr,
	&axi_hsci_dbg_hsci_master_linkup_ctrl_miso_clk_inv_attr,
	&axi_hsci_dbg_hsci_master_test_ctrl_hsci_mosi_test_mode_attr,
	&axi_hsci_dbg_hsci_master_test_ctrl_hsci_miso_test_mode_attr,
	&axi_hsci_dbg_hsci_master_test_ctrl_hsci_capture_mode_attr,
	&axi_hsci_dbg_hsci_master_linkup_status_link_active_attr,
	&axi_hsci_dbg_hsci_master_linkup_status_alink_txclk_adj_attr,
	&axi_hsci_dbg_hsci_master_linkup_status_alink_txclk_inv_attr,
	&axi_hsci_dbg_hsci_master_linkup_status_txclk_adj_mismatch_attr,
	&axi_hsci_dbg_hsci_master_linkup_status_txclk_inv_mismatch_attr,
	&axi_hsci_dbg_hsci_master_linkup_status2_alink_table_attr,
	&axi_hsci_dbg_hsci_master_linkup_status2_alink_fsm_attr,
	&axi_hsci_dbg_hsci_debug_status_enc_fsm_attr,
	&axi_hsci_dbg_hsci_debug_status_dec_fsm_attr,
	&axi_hsci_dbg_hsci_debug_status_capture_word_attr,
	&axi_hsci_dbg_hsci_debug_status_parity_error_attr,
	&axi_hsci_dbg_hsci_debug_status_unkown_instruction_error_attr,
	&axi_hsci_dbg_hsci_debug_status_slave_error_code_attr,
	&axi_hsci_dbg_miso_test_ber_miso_test_ber_attr,
	&axi_hsci_dbg_hsci_master_link_err_info_attr,
	&axi_hsci_dbg_hsci_mmcm_drp_trig_attr,
	&axi_hsci_dbg_hsci_rate_sel_attr,
	&axi_hsci_dbg_hsci_pll_reset_attr,
	&axi_hsci_dbg_hsci_master_clear_errors_attr,
	&axi_hsci_dbg_hsci_master_rstn_attr,
	&axi_hsci_dbg_hsci_reset_seq_done_attr,
	&axi_hsci_dbg_hsci_phy_pll_locked_attr,
	&axi_hsci_dbg_hsci_vtc_rdy_tx_attr,
	&axi_hsci_dbg_hsci_vtc_rdy_rx_attr,
	&axi_hsci_dbg_hsci_dly_rdy_tx_attr,
	&axi_hsci_dbg_hsci_dly_rdy_rx_attr,
	&axi_hsci_dbg_hsci_master_scratch_scratch_reg_attr,
	&axi_hsci_dbg_stat_read_len_attr,
	&axi_hsci_dbg_stat_read_count_attr,
	&axi_hsci_dbg_stat_write_len_attr,
	&axi_hsci_dbg_stat_write_count_attr,
	NULL /* END OF LIST MARKER */
};

/**
 * axi_hsci_wait_done - Wait for the AXI HSCI operation to complete
 * @st: Pointer to the axi_hsci_state structure
 *
 * This function waits for the AXI HSCI operation to complete by continuously
 * checking the status register. It returns 0 if the operation completes
 * successfully, or -ETIMEDOUT if the operation times out.
 *
 * The function also handles the case when the operation times out. If the
 * operation times out, it unlocks the update lock, prints debug information,
 * resets the device, and returns the error code.
 *
 * Return: 0 if the operation completes successfully, or -ETIMEDOUT if the
 * operation times out.
 */

static int32_t axi_hsci_wait_done(struct axi_hsci_state *st)
{
	int ret = -ETIMEDOUT, i = 0;
	u32 cnt = 0;
	u32 status = 0;
	unsigned long long val;

	do {
		status = axi_hsci_read(st, AXI_HSCI_STATUS);
		if ((status & 0x1) == 1)
			ret = 0;
	} while ((cnt++ < 60000) && (ret != 0));

	if (ret) {
		if (!st->silent) {
			mutex_unlock(&st->update_lock);
			while (axi_hsci_dbg_attrs[i]) {
				axi_hsci_dbg_attrs[i]->get(st, &val);
				dev_err(st->dev, "%s: %llu\n", axi_hsci_dbg_attrs[i]->name, val);
				i++;
			}
			mutex_lock(&st->update_lock);

			dev_err(st->dev, "%s:%d RESET !!!!! status %x\n", __func__, __LINE__, status);
		}
		axi_hsci_write(st, AXI_HSCI_MASTER_RST_ADDR, 0x00);
		axi_hsci_write(st, AXI_HSCI_MASTER_RST_ADDR, 0x100);
	}

	return ret;
}

/**
 * axi_hsci_dbg_cleanup - Clean up debugfs entries for AXI HSCI driver
 * @data: Pointer to the axi_hsci_state structure
 *
 * This function is responsible for cleaning up the debugfs entries created
 * for the AXI HSCI driver. It removes the debug directory and all its
 * subdirectories and files recursively.
 */
static void axi_hsci_dbg_cleanup(void *data)
{
	struct axi_hsci_state *st = data;

	debugfs_remove_recursive(st->debugdir);
}

#define kref_to_data_offload(x) container_of(x, struct axi_hsci_state, kref)

/**
 * axi_hsci_release - Release function for AXI HSCI driver
 * @kref: Pointer to the reference counter structure
 *
 * This function is called when the reference count of the AXI HSCI driver
 * reaches zero. It releases the resources associated with the driver.
 *
 * @kref: Pointer to the reference counter structure
 */
static void axi_hsci_release(struct kref *kref)
{
	struct axi_hsci_state *st = kref_to_data_offload(kref);

	of_node_put(st->of_node);
	kfree(st);
}

/**
 * axi_hsci_put - Release the reference to the AXI HSCI state
 * @data: Pointer to the AXI HSCI state structure
 *
 * This function releases the reference to the AXI HSCI state structure.
 * It is called when the reference count reaches zero, indicating that
 * the structure is no longer in use.
 */
static void axi_hsci_put(void *data)
{
	struct axi_hsci_state *st = data;

	kref_put(&st->kref, axi_hsci_release);
}

/**
 * axi_hsci_unregister - Unregister an AXI HSCI device
 * @data: Pointer to the AXI HSCI device state
 *
 * This function is used to unregister an AXI HSCI device. It removes the device
 * state from the list of probed devices and sets the device's initialized flag
 * to false.
 */
static void axi_hsci_unregister(void *data)
{
	struct axi_hsci_state *st = data;

	mutex_lock(&probed_devices_lock);
	list_del(&st->entry);
	mutex_unlock(&probed_devices_lock);

	mutex_lock(&st->update_lock);
	st->initialized = false;
	mutex_unlock(&st->update_lock);
}

/**
 * axi_hsci_register - Register an AXI HSCI device
 * @dev: Pointer to the device structure
 * @st: Pointer to the AXI HSCI state structure
 *
 * This function is used to register an AXI HSCI device with the Linux kernel.
 * It adds the device to the list of probed devices and associates an action
 * to unregister the device in case of failure.
 *
 * Return: 0 on success, negative error code on failure
 */
static int axi_hsci_register(struct device *dev, struct axi_hsci_state *st)
{
	mutex_lock(&probed_devices_lock);
	list_add(&st->entry, &probed_devices);
	mutex_unlock(&probed_devices_lock);

	return devm_add_action_or_reset(dev, axi_hsci_unregister, st);
}

/**
 * devm_axi_hsci_get_optional - Get an optional AXI HSCI device state
 * @dev: Pointer to the device structure
 *
 * This function is used to get the optional AXI HSCI (High Speed Communication Interface)
 * device state associated with the given device. It searches for a device node in the device's
 * device tree that is connected to the AXI HSCI and returns the corresponding device state.
 *
 * Return: Pointer to the AXI HSCI device state on success, NULL if no device node is found,
 *         or an error pointer if an error occurs.
 */
struct axi_hsci_state *devm_axi_hsci_get_optional(struct device *dev)
{
	struct axi_hsci_state *st;
	int ret;
	struct device_node *of_node;

	of_node = of_parse_phandle(dev->of_node, "adi,axi-hsci-connected", 0);
	if (!of_node)
		return NULL;

	mutex_lock(&probed_devices_lock);

	list_for_each_entry(st, &probed_devices, entry) {
		if (st->of_node != of_node)
			continue;

		kref_get(&st->kref);

		mutex_unlock(&probed_devices_lock);
		of_node_put(of_node);

		ret = devm_add_action_or_reset(dev, axi_hsci_put, st);
		if (ret)
			return ERR_PTR(ret);

		return st;
	}

	dev_dbg(dev, "Failed to find requested hsci \"%s\", try again later!\n",
		of_node->name);

	mutex_unlock(&probed_devices_lock);
	of_node_put(of_node);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(devm_axi_hsci_get_optional);

/**
 * axi_hsci_probe - Probe function for the AXI HSCI driver
 * @pdev: Pointer to the platform device structure
 *
 * This function is called when a platform device is being probed.
 * It initializes the AXI HSCI driver by performing the following steps:
 * - Retrieves the device tree node associated with the platform device
 * - Retrieves the AXI HSCI device information from the device tree
 * - Allocates memory for the AXI HSCI state structure
 * - Initializes the AXI HSCI state structure
 * - Retrieves the reference input clock for the AXI HSCI
 * - Reads the interface speed from the device tree and sets the corresponding rate selection
 * - Sets the clock rate for the AXI HSCI
 * - Enables the reference input clock
 * - Registers a cleanup action to release resources on device removal
 * - Maps the AXI HSCI registers to virtual memory
 * - Reads the IP core version and checks if it is compatible with the driver
 * - Creates a debugfs directory for the AXI HSCI device and registers debugfs files
 * - Writes the rate selection to the AXI HSCI rate control register
 * - Checks the status of the HSCI Master PHY
 * - Enables the HSCI Master
 * - Registers the AXI HSCI device with other drivers
 *
 * Return: 0 on success, negative error code on failure
 */
static int axi_hsci_probe(struct platform_device *pdev)
{
	int ret;
	struct axi_hsci_state *st;
	const struct axi_hsci_info *info;
	struct device_node *np = pdev->dev.of_node;
	struct axi_do_dbg_attr **i;
	u32 reg_val, interface_rate, hsci_rate_sel;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n", np->name);

	info = of_device_get_match_data(&pdev->dev);
	if (!info)
		return -ENODEV;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->dev = &pdev->dev;
	st->of_node = of_node_get(np);
	mutex_init(&st->update_lock);
	kref_init(&st->kref);

	st->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(st->pclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->pclk),
				     "failed to get the reference input clock\n");

	interface_rate = 800;
	ret = of_property_read_u32(np, "adi,hsci-interface-speed-mhz", &interface_rate);

	switch (interface_rate) {
	case 800:
		hsci_rate_sel = 0;
		break;
	case 400:
		hsci_rate_sel = 1;
		break;
	case 200:
		hsci_rate_sel = 2;
		break;
	default:
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "Invalid adi,hsci-interface-speed-mhz\n");
	}

	if (of_property_read_bool(np, "adi,hsci-miso-clk-inv-en"))
		st->linkup_ctrl |= AXI_HSCI_LINKUP_CTRL_MISO_CLK_INV;

	if (of_property_read_bool(np, "adi,hsci-mosi-clk-inv-en"))
		st->linkup_ctrl |= AXI_HSCI_LINKUP_CTRL_MOSI_CLK_INV;

	ret = clk_set_rate(st->pclk, interface_rate * 1000000 / 4);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "clk_set_rate\n");

	ret = clk_prepare_enable(st->pclk);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "clk_prepare_enable\n");

	ret = devm_add_action_or_reset(&pdev->dev, axi_hsci_put, st);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "devm_add_action_or_reset\n");

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->regs),
				     "devm_platform_ioremap_resource\n");

	st->version = axi_hsci_read(st, AXI_HSCI_REVID);
	if (info->version > st->version) {
		dev_err(&pdev->dev,
			"IP core version is too old. Expected %d.0.0, Reported %d.0.0\n",
			info->version, st->version);
		return -ENODEV;
	}

	if (!IS_ERR(axi_hsci_dbg_parent)) {
		st->debugdir = debugfs_create_dir(np->name, axi_hsci_dbg_parent);
		if (!IS_ERR(st->debugdir)) {
			for (i = axi_hsci_dbg_attrs; *i; i++)
				debugfs_create_file_unsafe((*i)->name, (*i)->mode,
							   st->debugdir, st, (*i)->fops);

			ret = devm_add_action_or_reset(&pdev->dev, axi_hsci_dbg_cleanup, st);
			if (ret)
				return ret;
		}
	}
	axi_hsci_write_mask(st, AXI_HSCI_RATE_CTRL_ADDR,
			    FIELD_PREP(AXI_HSCI_RATE_CTRL_MSK, hsci_rate_sel),
			    AXI_HSCI_RATE_CTRL_MSK);

	/* Check HSCI Master PHY status */
	reg_val = axi_hsci_read(st, AXI_HSCI_PHY_STATUS_ADDR);
	if (reg_val != 0x3F)
		dev_err(&pdev->dev, "HSCI PHY is 0x%X, expected 0x3F\n", reg_val);

	/* Enable the HSCI Master */
	axi_hsci_write(st, AXI_HSCI_MASTER_RST_ADDR, 0x100);

	/* Check HSCI Master PHY status */
	reg_val = axi_hsci_read(st, AXI_HSCI_PHY_STATUS_ADDR);
	if (reg_val != 0x3F)
		dev_err(&pdev->dev, "HSCI PHY is 0x%X, expected 0x3F\n", reg_val);

	st->initialized = true;

	/* Register device for other drivers to access */
	ret = axi_hsci_register(&pdev->dev, st);
	if (ret)
		return ret;

	/* Done */
	dev_info(&pdev->dev, "AXI HSCI IP core (%d.0.0) probed @ %u MHz\n",
		 st->version, interface_rate);

	return 0;
}

static struct platform_driver axi_hsci_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = axi_hsci_of_match,
	},
	.probe = axi_hsci_probe
};

static int axi_hsci_driver_register(struct platform_driver *driver)
{
	axi_hsci_dbg_parent = debugfs_create_dir(KBUILD_MODNAME, NULL);
	return platform_driver_register(driver);
}

static void axi_hsci_driver_unregister(struct platform_driver *driver)
{
	debugfs_remove_recursive(axi_hsci_dbg_parent);
	platform_driver_unregister(driver);
}

module_driver(axi_hsci_driver,
	      axi_hsci_driver_register,
	      axi_hsci_driver_unregister);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices hsci interface driver");
MODULE_LICENSE("Dual BSD/GPL");
