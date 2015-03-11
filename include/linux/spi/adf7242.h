/*
 * Analog Devices ADF7242 Low-Power IEEE 802.15.4 Transceiver
 *
 * Copyright 2009-2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __LINUX_SPI_ADF7242_H
#define __LINUX_SPI_ADF7242_H

struct adf7242_platform_data {

#define ADF_IEEE802154_HW_AACK         (1 << 1)
#define ADF_IEEE802154_AUTO_CSMA_CA    (1 << 2)
#define ADF_IEEE802154_PROMISCUOUS_MODE        (1 << 3)	/* no address filtering, turns off HW_AACK */
#define ADF_IEEE802154_REPORT_ACK        (1 << 4)	/* Report Ack frames */

	int mode;

	/*
	 * Specifies number of attempts to
	 * retransmit unacknowledged
	 * frames while in automatic CSMA-CA
	 * Tx mode.
	 */
	int max_frame_retries;

	/*
	 * Specifies number of attempts to
	 * repeat CSMA-CA algorithm prior to
	 * cancellation of RC_TX command.
	 * Valid range is 0 to 5;
	 * 7: CSMA-CA algorithm is off
	 */
	int max_cca_retries;

	/*
	 * Specifies the maximum back-off
	 * exponent used in the CSMA-CA
	 * algorithm; valid range is 4 to 8
	 *
	 */
	int max_csma_be;

	/*
	 * Specifies the minimum back-off
	 * exponent used in the CSMA-CA
	 * algorithm; valid range is 0 to
	 * csma_max_be
	 */
	int min_csma_be;
};
#endif
