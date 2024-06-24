/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Internal GPIO functions.
 *
 * Copyright (C) 2013, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 */

#ifndef GPIOLIB_H
#define GPIOLIB_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h> /* for enum gpiod_flags */
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/srcu.h>

#define GPIOCHIP_NAME	"gpiochip"

/**
 * struct gpio_device - internal state container for GPIO devices
 * @dev: the GPIO device struct
 * @chrdev: character device for the GPIO device
 * @id: numerical ID number for the GPIO chip
 * @mockdev: class device used by the deprecated sysfs interface (may be
 * NULL)
 * @owner: helps prevent removal of modules exporting active GPIOs
 * @chip: pointer to the corresponding gpiochip, holding static
 * data for this device
 * @descs: array of ngpio descriptors.
 * @desc_srcu: ensures consistent state of GPIO descriptors exposed to users
 * @ngpio: the number of GPIO lines on this GPIO device, equal to the size
 * of the @descs array.
 * @can_sleep: indicate whether the GPIO chip driver's callbacks can sleep
 * implying that they cannot be used from atomic context
 * @base: GPIO base in the DEPRECATED global Linux GPIO numberspace, assigned
 * at device creation time.
 * @label: a descriptive name for the GPIO device, such as the part number
 * or name of the IP component in a System on Chip.
 * @data: per-instance data assigned by the driver
 * @list: links gpio_device:s together for traversal
 * @line_state_notifier: used to notify subscribers about lines being
 *                       requested, released or reconfigured
 * @device_notifier: used to notify character device wait queues about the GPIO
 *                   device being unregistered
 * @srcu: protects the pointer to the underlying GPIO chip
 * @pin_ranges: range of pins served by the GPIO driver
 *
 * This state container holds most of the runtime variable data
 * for a GPIO device and can hold references and live on after the
 * GPIO chip has been removed, if it is still being used from
 * userspace.
 */
struct gpio_device {
	struct device		dev;
	struct cdev		chrdev;
	int			id;
	struct device		*mockdev;
	struct module		*owner;
	struct gpio_chip __rcu	*chip;
	struct gpio_desc	*descs;
	struct srcu_struct	desc_srcu;
	unsigned int		base;
	u16			ngpio;
	bool			can_sleep;
	const char		*label;
	void			*data;
	struct list_head        list;
	struct blocking_notifier_head line_state_notifier;
	struct blocking_notifier_head device_notifier;
	struct srcu_struct	srcu;

#ifdef CONFIG_PINCTRL
	/*
	 * If CONFIG_PINCTRL is enabled, then gpio controllers can optionally
	 * describe the actual pin range which they serve in an SoC. This
	 * information would be used by pinctrl subsystem to configure
	 * corresponding pins for gpio usage.
	 */
	struct list_head pin_ranges;
#endif
};

static inline struct gpio_device *to_gpio_device(struct device *dev)
{
	return container_of(dev, struct gpio_device, dev);
}

/* gpio suffixes used for ACPI and device tree lookup */
static __maybe_unused const char * const gpio_suffixes[] = { "gpios", "gpio" };

/**
 * struct gpio_array - Opaque descriptor for a structure of GPIO array attributes
 *
 * @desc:		Array of pointers to the GPIO descriptors
 * @size:		Number of elements in desc
 * @chip:		Parent GPIO chip
 * @get_mask:		Get mask used in fastpath
 * @set_mask:		Set mask used in fastpath
 * @invert_mask:	Invert mask used in fastpath
 *
 * This structure is attached to struct gpiod_descs obtained from
 * gpiod_get_array() and can be passed back to get/set array functions in order
 * to activate fast processing path if applicable.
 */
struct gpio_array {
	struct gpio_desc	**desc;
	unsigned int		size;
	struct gpio_chip	*chip;
	unsigned long		*get_mask;
	unsigned long		*set_mask;
	unsigned long		invert_mask[];
};

#define for_each_gpio_desc(gc, desc)					\
	for (unsigned int __i = 0;					\
	     __i < gc->ngpio && (desc = gpiochip_get_desc(gc, __i));	\
	     __i++)							\

#define for_each_gpio_desc_with_flag(gc, desc, flag)			\
	for_each_gpio_desc(gc, desc)					\
		if (!test_bit(flag, &desc->flags)) {} else

int gpiod_get_array_value_complex(bool raw, bool can_sleep,
				  unsigned int array_size,
				  struct gpio_desc **desc_array,
				  struct gpio_array *array_info,
				  unsigned long *value_bitmap);
int gpiod_set_array_value_complex(bool raw, bool can_sleep,
				  unsigned int array_size,
				  struct gpio_desc **desc_array,
				  struct gpio_array *array_info,
				  unsigned long *value_bitmap);

int gpiod_set_transitory(struct gpio_desc *desc, bool transitory);

void gpiod_line_state_notify(struct gpio_desc *desc, unsigned long action);

struct gpio_desc_label {
	struct rcu_head rh;
	char str[];
};

/**
 * struct gpio_desc - Opaque descriptor for a GPIO
 *
 * @gdev:		Pointer to the parent GPIO device
 * @flags:		Binary descriptor flags
 * @label:		Name of the consumer
 * @name:		Line name
 * @hog:		Pointer to the device node that hogs this line (if any)
 *
 * These are obtained using gpiod_get() and are preferable to the old
 * integer-based handles.
 *
 * Contrary to integers, a pointer to a &struct gpio_desc is guaranteed to be
 * valid until the GPIO is released.
 */
struct gpio_desc {
	struct gpio_device	*gdev;
	unsigned long		flags;
/* flag symbols are bit numbers */
#define FLAG_REQUESTED	0
#define FLAG_IS_OUT	1
#define FLAG_EXPORT	2	/* protected by sysfs_lock */
#define FLAG_SYSFS	3	/* exported via /sys/class/gpio/control */
#define FLAG_ACTIVE_LOW	6	/* value has active low */
#define FLAG_OPEN_DRAIN	7	/* Gpio is open drain type */
#define FLAG_OPEN_SOURCE 8	/* Gpio is open source type */
#define FLAG_USED_AS_IRQ 9	/* GPIO is connected to an IRQ */
#define FLAG_IRQ_IS_ENABLED 10	/* GPIO is connected to an enabled IRQ */
#define FLAG_IS_HOGGED	11	/* GPIO is hogged */
#define FLAG_TRANSITORY 12	/* GPIO may lose value in sleep or reset */
#define FLAG_PULL_UP    13	/* GPIO has pull up enabled */
#define FLAG_PULL_DOWN  14	/* GPIO has pull down enabled */
#define FLAG_BIAS_DISABLE    15	/* GPIO has pull disabled */
#define FLAG_EDGE_RISING     16	/* GPIO CDEV detects rising edge events */
#define FLAG_EDGE_FALLING    17	/* GPIO CDEV detects falling edge events */
#define FLAG_EVENT_CLOCK_REALTIME	18 /* GPIO CDEV reports REALTIME timestamps in events */
#define FLAG_EVENT_CLOCK_HTE		19 /* GPIO CDEV reports hardware timestamps in events */

	/* Connection label */
	struct gpio_desc_label __rcu *label;
	/* Name of the GPIO */
	const char		*name;
#ifdef CONFIG_OF_DYNAMIC
	struct device_node	*hog;
#endif
};

#define gpiod_not_found(desc)		(IS_ERR(desc) && PTR_ERR(desc) == -ENOENT)

struct gpio_chip_guard {
	struct gpio_device *gdev;
	struct gpio_chip *gc;
	int idx;
};

DEFINE_CLASS(gpio_chip_guard,
	     struct gpio_chip_guard,
	     srcu_read_unlock(&_T.gdev->srcu, _T.idx),
	     ({
		struct gpio_chip_guard _guard;

		_guard.gdev = desc->gdev;
		_guard.idx = srcu_read_lock(&_guard.gdev->srcu);
		_guard.gc = srcu_dereference(_guard.gdev->chip,
					     &_guard.gdev->srcu);

		_guard;
	     }),
	     struct gpio_desc *desc)

int gpiod_request(struct gpio_desc *desc, const char *label);
void gpiod_free(struct gpio_desc *desc);

static inline int gpiod_request_user(struct gpio_desc *desc, const char *label)
{
	int ret;

	ret = gpiod_request(desc, label);
	if (ret == -EPROBE_DEFER)
		ret = -ENODEV;

	return ret;
}

struct gpio_desc *gpiod_find_and_request(struct device *consumer,
					 struct fwnode_handle *fwnode,
					 const char *con_id,
					 unsigned int idx,
					 enum gpiod_flags flags,
					 const char *label,
					 bool platform_lookup_allowed);

int gpiod_configure_flags(struct gpio_desc *desc, const char *con_id,
		unsigned long lflags, enum gpiod_flags dflags);
int gpio_set_debounce_timeout(struct gpio_desc *desc, unsigned int debounce);
int gpiod_hog(struct gpio_desc *desc, const char *name,
		unsigned long lflags, enum gpiod_flags dflags);
int gpiochip_get_ngpios(struct gpio_chip *gc, struct device *dev);
const char *gpiod_get_label(struct gpio_desc *desc);

/*
 * Return the GPIO number of the passed descriptor relative to its chip
 */
static inline int gpio_chip_hwgpio(const struct gpio_desc *desc)
{
	return desc - &desc->gdev->descs[0];
}

/* With descriptor prefix */

#define gpiod_err(desc, fmt, ...) \
do { \
	scoped_guard(srcu, &desc->gdev->desc_srcu) { \
		pr_err("gpio-%d (%s): " fmt, desc_to_gpio(desc), \
		       gpiod_get_label(desc) ? : "?", ##__VA_ARGS__); \
	} \
} while (0)

#define gpiod_warn(desc, fmt, ...) \
do { \
	scoped_guard(srcu, &desc->gdev->desc_srcu) { \
		pr_warn("gpio-%d (%s): " fmt, desc_to_gpio(desc), \
			gpiod_get_label(desc) ? : "?", ##__VA_ARGS__); \
	} \
} while (0)

#define gpiod_dbg(desc, fmt, ...) \
do { \
	scoped_guard(srcu, &desc->gdev->desc_srcu) { \
		pr_debug("gpio-%d (%s): " fmt, desc_to_gpio(desc), \
			 gpiod_get_label(desc) ? : "?", ##__VA_ARGS__); \
	} \
} while (0)

/* With chip prefix */

#define chip_err(gc, fmt, ...)					\
	dev_err(&gc->gpiodev->dev, "(%s): " fmt, gc->label, ##__VA_ARGS__)
#define chip_warn(gc, fmt, ...)					\
	dev_warn(&gc->gpiodev->dev, "(%s): " fmt, gc->label, ##__VA_ARGS__)
#define chip_info(gc, fmt, ...)					\
	dev_info(&gc->gpiodev->dev, "(%s): " fmt, gc->label, ##__VA_ARGS__)
#define chip_dbg(gc, fmt, ...)					\
	dev_dbg(&gc->gpiodev->dev, "(%s): " fmt, gc->label, ##__VA_ARGS__)

#endif /* GPIOLIB_H */
