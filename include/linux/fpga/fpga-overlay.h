/*
 * mwgeneric_of_overlay.h
 *
 *  Created on: Jan 8, 2016
 *      Author: mfornero
 */

#ifndef __FPGA_OVERLAY_H_
#define __FPGA_OVERLAY_H_

#include <linux/of.h>
#include <linux/device.h>
#include <linux/mutex.h>

struct fpga_overlay_ops;

struct fpga_overlay_dev {
	struct device 				dev;
	struct device				*parent;
	struct fpga_overlay_ops 	*ops;
	void 						*priv;
	int 						overlay_id;
	char 						*name;
	char						*firmware_name;
	struct mutex 				sysfs_mutex;
	struct mutex 				overlay_mutex;
	const struct firmware 		*fw;
};

#define to_fpga_overlay(d) container_of(d, struct fpga_overlay_dev, dev)

struct fpga_overlay_ops {
	int (*apply_init) (struct fpga_overlay_dev *overlay_dev);
	int (*load_fw) (struct fpga_overlay_dev *overlay_dev);
	int (*update_fragment) (struct fpga_overlay_dev *overlay_dev, struct device_node *fragment);
	void (*release_fw)  (void *opaque);
	void (*apply_term) (void *opaque);
};

extern int fpga_overlay_is_fragment(struct device_node *fragment);
extern int fpga_overlay_apply(struct fpga_overlay_dev *overlay_dev);
extern int fpga_overlay_remove(struct fpga_overlay_dev *overlay_dev);
extern struct fpga_overlay_dev *fpga_overlay_dev_register(struct device * dev, char *name, struct fpga_overlay_ops *ops, void *priv);
extern void fpga_overlay_dev_unregister(struct fpga_overlay_dev *overlay_dev);

#endif /* __FPGA_OVERLAY_H_ */
