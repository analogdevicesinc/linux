/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef BCM577X5_MS_H
#define BCM577X5_MS_H

#include <linux/compiler_types.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct memstick_host;
struct memstick_request;
struct pci_dev;
struct dentry;

struct bcm577x5_device {
	struct pci_dev *pci_dev;
	struct memstick_host	*host;		/* host backpointer */
	struct memstick_request *req;		/* current request */

	/* Registers, IRQ */
	void __iomem *mmio;
	int irq;

	/* Command execution */
	struct work_struct req_work;
	struct mutex req_lock;		/* prevent concurrent requests */
	struct completion cmd_done;
	struct completion fifo_done;
	atomic_t int_seen;
	unsigned char prev_tpc;
	u8 auto_int;
	bool get_int_after_done;
	int mode;

	/* DMA execution */
	bool enable_dma;
	struct dentry *debugfs;
	struct completion dma_done;
	struct completion data_done;
	void *dma_buffer;
	dma_addr_t dma_handle;

	/* Device properties */
	int base_clock;
};

#define BCM577x5_MS_POLL_DELAY_US 10
#define BCM577x5_MS_REQ_TIMEOUT_MS 450

/*
 * Controller registers (from sdhci.h)
 * Registers are almost exactly the same as sdhci registers, reuse them
 */

#define BCM577x5_MS_DMA_ADDRESS		0x00
#define BCM577x5_MS_BLOCK_SIZE		0x04
#define BCM577x5_MS_BLOCK_COUNT		0x06
#define BCM577x5_MS_ARGUMENT		0x08

#define BCM577x5_MS_TPC_REG		0x0C
#define BCM577x5_MS_MAKE_TPC(c) ((((c) << 4) | ((c) ^ 0xF)) << 24)
#define  BCM577x5_MS_TRNS_DMA		0x01
#define  BCM577x5_MS_TRNS_BLK_CNT_EN	0x02
#define  BCM577x5_MS_TRNS_READ		0x10
#define  BCM577x5_MS_TRNS_MULTI		0x20

#define BCM577x5_MS_RESPONSE		0x10
#define BCM577x5_MS_RESPONSE2		0x14
#define BCM577x5_MS_BUFFER		0x20

/* This can be used as a logic analyzer */
#define BCM577x5_MS_PRESENT_STATE	0x24
#define  BCM577x5_MS_DATA_LVL_MASK	0x00F00000
#define  BCM577x5_MS_CMD_LVL		0x01000000

#define BCM577x5_MS_HOST_CONTROL	0x28
#define  BCM577x5_MS_CTRL_4BITBUS	0x02
#define  BCM577x5_MS_CTRL_8BITBUS	0x20

#define BCM577x5_MS_POWER_CONTROL	0x29
#define  BCM577x5_MS_POWER_ON		0x01
#define  BCM577x5_MS_POWER_330		0x0E

#define BCM577x5_MS_CLOCK_CONTROL	0x2C
#define  BCM577x5_MS_DIVIDER_SHIFT	8
#define  BCM577x5_MS_DIVIDER_HI_SHIFT	6
#define  BCM577x5_MS_DIV_MASK		0xFF
#define  BCM577x5_MS_DIV_HI_MASK	0x300
#define  BCM577x5_MS_CLOCK_CARD_EN	0x0004
#define  BCM577x5_MS_CLOCK_INT_STABLE	0x0002
#define  BCM577x5_MS_CLOCK_INT_EN	0x0001

#define BCM577x5_MS_TIMEOUT_CONTROL	0x2E

#define BCM577x5_MS_SOFTWARE_RESET	0x2F
#define  BCM577x5_MS_RESET_ALL		0x01
#define  BCM577x5_MS_RESET_CMD		0x02
#define  BCM577x5_MS_RESET_DATA		0x04

#define BCM577x5_MS_INT_STATUS		0x30
#define BCM577x5_MS_INT_ENABLE		0x34
#define BCM577x5_MS_SIGNAL_ENABLE	0x38
#define  BCM577x5_MS_INT_RESPONSE	0x00000001
#define  BCM577x5_MS_INT_DATA_END	0x00000002
#define  BCM577x5_MS_INT_BLK_GAP	0x00000004
#define  BCM577x5_MS_INT_DMA_END	0x00000008
#define  BCM577x5_MS_INT_SPACE_AVAIL	0x00000010
#define  BCM577x5_MS_INT_DATA_AVAIL	0x00000020
#define  BCM577x5_MS_INT_CARD_INSERT_2	0x00000040
#define  BCM577x5_MS_INT_CARD_REMOVE	0x00000080
#define  BCM577x5_MS_INT_CARD_INT	0x00000100
#define  BCM577x5_MS_INT_RETUNE		0x00001000
#define  BCM577x5_MS_INT_CQE		0x00004000
#define  BCM577x5_MS_INT_ERROR		0x00008000
#define  BCM577x5_MS_INT_TIMEOUT	0x00010000
#define  BCM577x5_MS_INT_CRC		0x00020000
#define  BCM577x5_MS_INT_END_BIT	0x00040000
#define  BCM577x5_MS_INT_INDEX		0x00080000
#define  BCM577x5_MS_INT_DATA_TIMEOUT	0x00100000
#define  BCM577x5_MS_INT_DATA_CRC	0x00200000
#define  BCM577x5_MS_INT_DATA_END_BIT	0x00400000
#define  BCM577x5_MS_INT_BUS_POWER	0x00800000
#define  BCM577x5_MS_INT_AUTO_CMD_ERR	0x01000000
#define  BCM577x5_MS_INT_ADMA_ERROR	0x02000000
#define  BCM577x5_MS_INT_CARD_INSERT	0x40000000

#define BCM577x5_MS_INT_DEFAULT                                                \
	(BCM577x5_MS_INT_RESPONSE | BCM577x5_MS_INT_DATA_END |                 \
	 BCM577x5_MS_INT_DMA_END | BCM577x5_MS_INT_SPACE_AVAIL |               \
	 BCM577x5_MS_INT_DATA_AVAIL | BCM577x5_MS_INT_CARD_INSERT_2 |          \
	 BCM577x5_MS_INT_CARD_REMOVE | BCM577x5_MS_INT_TIMEOUT |               \
	 BCM577x5_MS_INT_CRC | BCM577x5_MS_INT_END_BIT |                       \
	 BCM577x5_MS_INT_INDEX | BCM577x5_MS_INT_DATA_TIMEOUT |                \
	 BCM577x5_MS_INT_DATA_CRC | BCM577x5_MS_INT_DATA_END_BIT |             \
	 BCM577x5_MS_INT_BUS_POWER | BCM577x5_MS_INT_ADMA_ERROR |              \
	 BCM577x5_MS_INT_CARD_INSERT)

#define BCM577x5_MS_ARGUMENT2	0xF4

#define BCM577x5_MS_BRIDGE_MODE				0x180
#define  BCM577x5_MS_BRIDGE_MODE_PARALLEL		0x02000000

#define BCM577x5_MS_STATUS				0x190
#define  BCM577x5_MS_STATUS_INT_ASSERTED		0x00000002
#define  BCM577x5_MS_STATUS_MS_DETECT			0x00004000

#define BCM577x5_MS_STATUS_MUX_CONTROL			0x198
#define  BCM577x5_MS_CLOCK_SOURCE_62_5MHZ		0x00003000

#define BCM577x5_MS_CAPABILITY_SLOT2			0x1A8
#define  BCM577x5_MS_CAPABILITY_SDMA			0x00100000
#define BCM577x5_MS_CAPABILITY_BASE_CLOCK_MASK		0x00007F80
#define  BCM577x5_MS_CAPABILITY_BASE_CLOCK_SHIFT	7
#define BCM577x5_MS_CAPABILITY_TIMEOUT_CLOCK_MASK	0x0000003F

#endif /* BCM577X5_MS_H */
