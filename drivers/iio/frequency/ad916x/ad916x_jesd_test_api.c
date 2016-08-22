#include "api_config.h"
#include "AD916x.h"
#include "ad916x_reg.h"
#include "api_errors.h"
#include "utils.h"

/*  TODO:
	The JESD204B receiver in the AD916x supports the short
	transport layer (STPL) test as described in the JESD204B
	standard. This test can be used to verify the data mapping
	between the JESD204B transmitter and receiver. To perform
	this test, this function must be implemented in the logic device
	and enabled there. Before running the test on the receiver side,
	the link must be established and running without errors.
	The STPL test ensures that each sample from each converter is
	mapped appropriately according to the number of converters
	(M) and the number of samples per converter (S). As specified
	in the JESD204B standard, the converter manufacturer specifies
	what test samples are transmitted. Each sample must have a
	unique value. For example, if M = 2 and S = 2, there are 4
	unique samples transmitted repeatedly until the test is stopped.
	The expected sample must be programmed into the device and
	the expected sample is compared to the received sample one
	sample at a time until all have been tested.
*/
ADI_API int ad916x_jesd_short_tpl_test(ad916x_handle_t *h)
{
	if (h != INVALID_POINTER) {
		return API_ERROR_OK;
	}
	return API_ERROR_INVALID_HANDLE_PTR;
}

ADI_API int ad916x_jesd_phy_prbs_test(ad916x_handle_t *h, 
					const jesd_prbs_pattern_t prbs_pattern,
					const uint8_t lanes_en, const uint32_t prbs_error_threshold,
					ad916x_prbs_test_t *result)
{
	int err;
	uint8_t tmp_reg;

	if (h != INVALID_POINTER) {
		if (prbs_error_threshold > ADI_MAXUINT24) {
			return API_ERROR_INVALID_PARAM;
		}
		/* Select PRBS Pattern */
		err = ad916x_register_read(h, AD916x_REG_PHY_PRBS_CTRL, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= ~(3u << 2);

		/* Clear the PHY_TEST_RESET bit just in case. We'll toggle it later. */
		tmp_reg &= 0xFE;

		switch (prbs_pattern)
		{
		case PRBS7:
			tmp_reg |= (0 << 2);
			break;
		case PRBS15:
			tmp_reg |= (1u << 2);
			break;
		case PRBS31:
			tmp_reg |= (2u << 2);
			break;
		default:
			return API_ERROR_INVALID_PARAM;
			break;
		}

		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_CTRL, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/* Enable PHY test for all lanes */
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_EN, lanes_en);
		if (err != API_ERROR_OK) {
			return err;
		}

		/* Toggle PHY_TEST_RESET */
		/* We hope 'tmp_reg' holds the actual value of reg
		AD916x_REG_PHY_PRBS_CTRL */
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_CTRL, tmp_reg | 1u);
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_CTRL, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/* Set PHY_PRBS_ERROR_THRESHOLD */
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_ERROR_THRESHOLD_LO, (uint8_t)(prbs_error_threshold & 0xFF));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_ERROR_THRESHOLD_MID, (uint8_t)((prbs_error_threshold >> 8) & 0xFF));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_ERROR_THRESHOLD_HI, (uint8_t)((prbs_error_threshold >> 16) & 0xFF));
		if (err != API_ERROR_OK) {
			return err;
		}

		/* PHY_TEST_START */
		/* We still hope 'tmp_reg' holds the actual value of reg
		AD916x_REG_PHY_PRBS_CTRL */
		tmp_reg &= ~(1u << 1);
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_CTRL, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_CTRL, tmp_reg | (1u << 1));
		if (err != API_ERROR_OK) {
			return err;
		}

		/* Wait 500 mS */
		err = h->delay_us(h->user_data, MS_TO_US(500));
		if (err != 0) {
			return API_ERROR_US_DELAY;
		}

		/* Stop the test */
		err = ad916x_register_write(h, AD916x_REG_PHY_PRBS_CTRL, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/* Read PRBS Results */
		err = ad916x_register_read(h, AD916x_REG_PHY_PRBS_ERRCNT_2, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		result->phy_prbs_err_cnt = tmp_reg;
		result->phy_prbs_err_cnt <<= 8;
		err = ad916x_register_read(h, AD916x_REG_PHY_PRBS_ERRCNT_1, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		result->phy_prbs_err_cnt |= tmp_reg;
		result->phy_prbs_err_cnt <<= 8;
		err = ad916x_register_read(h, AD916x_REG_PHY_PRBS_ERRCNT_0, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		result->phy_prbs_err_cnt |= tmp_reg;

		err = ad916x_register_read(h, AD916x_REG_PHY_PRBS_CTRL, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		result->phy_src_err_cnt = (tmp_reg >> 4) & 0x07;

		/* Read PHY_PRBS_PASS */
		err = ad916x_register_read(h, AD916x_REG_PHY_PRBS_STAT, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		result->phy_prbs_pass = tmp_reg;

		if ((tmp_reg & lanes_en) != lanes_en) {
			return API_ERROR_TEST_FAILED;
		}

		return API_ERROR_OK;
	}
	return API_ERROR_INVALID_HANDLE_PTR;
}
/*!  TODO:
	As per section 5.3.3.8.2 of the JESD204B specification, the
	AD916x can check that a constant stream of /K28.5/ characters
	is being received, or that CGS followed by a constant stream of
	ILAS is being received.
	To run a repeated CGS test, send a constant stream of /K28.5/
	characters to the AD9144 SERDES inputs. Next, set up the
	device and enable the links. Ensure that the /K28.5/ characters
	are being received by verifying that the SYNCOUTB has been
	de-asserted and that CGS has passed for all enabled link lanes
	by reading Register 0x470.

	To run the CGS followed by a repeated ILAS sequence test,
	follow the procedure to set up the links, but before performing
	the last write (enabling the links), enable the ILAS test mode by
	writing a 1 to Register 0x477[7]. Then, enable the links. When
	the device recognizes 4 CGS characters on each lane, it de-
	asserts the SYNCOUTB. At this point, the transmitter starts
	sending a repeated ILAS sequence.
	Read Register 0x473 to verify that initial lane synchronization has
	passed for all enabled link lanes.
*/
ADI_API int ad916x_jesd_cgs_and_ilas_test(ad916x_handle_t *h)
{
	if (h != INVALID_POINTER) {
		return API_ERROR_OK;
	}
	return API_ERROR_INVALID_HANDLE_PTR;
}
