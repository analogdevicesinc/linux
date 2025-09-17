/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef ADI_I2C_H
#define ADI_I2C_H

int adi_twi_do_smbus_xfer(struct i2c_adapter *adap, u16 addr,
			  unsigned short flags, char read_write,
			  u8 command, int size, union i2c_smbus_data *data,
			  bool polling);

int adi_twi_smbus_xfer(struct i2c_adapter *adap, u16 addr,
		       unsigned short flags, char read_write,
		       u8 command, int size, union i2c_smbus_data *data);

int adi_twi_smbus_xfer_atomic(struct i2c_adapter *adap, u16 addr,
			      unsigned short flags, char read_write,
			      u8 command, int size,
			      union i2c_smbus_data *data);

#endif
