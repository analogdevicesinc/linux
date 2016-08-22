#include "api_config.h"
#include "AD916x.h"
#include "ad916x_reg.h"
#include "api_errors.h"
#include "utils.h"
#include <linux/regmap.h>

#define IN_OUT_BUFF_SZ 3

ADI_API int ad916x_register_write(ad916x_handle_t *h, 
								const uint16_t address, const uint8_t data)
{
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	return regmap_write(h->user_data, address, data);
}

ADI_API int ad916x_register_read(ad916x_handle_t *h, 
								const uint16_t address, uint8_t *data)
{
	unsigned int val;
	int ret;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	ret = regmap_read(h->user_data, address, &val);

	*data = val;

	return ret;
}

int ad916x_register_write_tbl(ad916x_handle_t *h, struct
							ad916x_reg_data *tbl, uint32_t count)
{
	uint16_t i = 0;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	for (i = 0; i<count; i++) {
		ad916x_register_write(h, tbl[i].reg, tbl[i].val);
	}

	return API_ERROR_OK;
}
