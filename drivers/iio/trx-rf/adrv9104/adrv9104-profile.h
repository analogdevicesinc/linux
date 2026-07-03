/* SPDX-License-Identifier: GPL-2.0 */

#ifndef IIO_ADRV9104_PROFILE_H_
#define IIO_ADRV9104_PROFILE_H_

#include <linux/types.h>
#include <linux/units.h>

struct adrv9104_rf_phy;
struct iio_dev;

#define ADRV9104_PROFILE_DEFAULT_NAME	"adrv9104_profile.json"
#define ADRV9104_PROFILE_MAX_SZ		(128 * KILO)

int adrv9104_profile_load(struct adrv9104_rf_phy *phy);
int adrv9104_profile_register(struct adrv9104_rf_phy *phy, struct iio_dev *indio_dev);
int adrv9104_profile_serialize(struct adrv9104_rf_phy *phy, char *out, size_t outlen);

#endif
