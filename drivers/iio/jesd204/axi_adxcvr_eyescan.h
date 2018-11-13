/*
 * ADI AXI-ADXCVR Module
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_adxcvr
 */

#ifndef AXI_ADXCVR_EYESCAN_H_
#define AXI_ADXCVR_EYESCAN_H_

/* GTH4 DRP Registers */
#define ES_DRP_CONTROL_ADDR 0x3c
#define ES_DRP_HOFFSET_ADDR 0x4f
#define ES_DRP_VOFFSET_ADDR 0x97
#define ES_DRP_STATUS_ADDR 0x253
#define ES_DRP_SCNT_ADDR 0x252
#define ES_DRP_ECNT_ADDR 0x251

#define ES_DRP_QUAL_MASK0 0x44
#define ES_DRP_QUAL_MASK1 0x45
#define ES_DRP_QUAL_MASK2 0x46
#define ES_DRP_QUAL_MASK3 0x47
#define ES_DRP_QUAL_MASK4 0x48

#define ES_DRP_QUALIFIER0 0x3f
#define ES_DRP_QUALIFIER1 0x40
#define ES_DRP_QUALIFIER2 0x41
#define ES_DRP_QUALIFIER3 0x42
#define ES_DRP_QUALIFIER4 0x43

#define ES_DRP_SDATA_MASK0 0x49
#define ES_DRP_SDATA_MASK1 0x4a
#define ES_DRP_SDATA_MASK2 0x4b
#define ES_DRP_SDATA_MASK3 0x4c
#define ES_DRP_SDATA_MASK4 0x4d

/* XCVR Eye Scan Registers */
#define ADXCVR_REG_ES_SEL		0x0080
#define ADXCVR_REG_ES_REQ		0x00A0
#define ADXCVR_REG_ES_CONTROL_1		0x00A4
#define ADXCVR_REG_ES_CONTROL_2		0x00A8
#define ADXCVR_REG_ES_CONTROL_3		0x00AC
#define ADXCVR_REG_ES_CONTROL_4		0x00B0
#define ADXCVR_REG_ES_CONTROL_5		0x00B4
#define ADXCVR_REG_ES_STATUS		0x00B8
#define ADXCVR_REG_ES_RESET		0x00BC

/* XCVR Eye Scan Masks */
#define ADXCVR_ES_SEL(x)		((x) & 0xFF)
#define ADXCVR_ES_REQ			BIT(0)

#define ADXCVR_ES_PRESCALE(x)		((x) & 0x1F)

#define ADXCVR_ES_VOFFSET_RANGE(x)	(((x) & 0x3) << 24)
#define ADXCVR_ES_VOFFSET_STEP(x)	(((x) & 0xFF) << 16)
#define ADXCVR_ES_VOFFSET_MAX(x)	(((x) & 0xFF) << 8)
#define ADXCVR_ES_VOFFSET_MIN(x)	(((x) & 0xFF) << 0)

#define ADXCVR_ES_HOFFSET_MAX(x)	(((x) & 0xFFF) << 16)
#define ADXCVR_ES_HOFFSET_MIN(x)	(((x) & 0xFFF) << 0)

#define ADXCVR_ES_HOFFSET_STEP(x)	(((x) & 0xFFF) << 0)

#define ADXCVR_ES_STATUS		BIT(0)

/* XCVR Eye Scan defines */
#define ES_HSIZE_FULL			65
#define ES_HSIZE_HALF			129
#define ES_HSIZE_QRTR			257
#define ES_HSIZE_OCT			513
#define ES_HSIZE_HEX			1025

#define ES_VSIZE			255

#include "axi_adxcvr.h"

struct adxcvr_eyescan {
	struct device		*dev;
	struct work_struct	work;
	struct bin_attribute	bin;
	struct completion	complete;
	struct adxcvr_state	*st;

	void			*buf_virt;
	dma_addr_t		buf_phys;

	int			lane;
	int			prescale;
};

int adxcvr_eyescan_register(struct adxcvr_state *st);
int adxcvr_eyescan_unregister(struct adxcvr_state *st);

#endif /* AXI_ADXCVR_EYESCAN_H_ */
