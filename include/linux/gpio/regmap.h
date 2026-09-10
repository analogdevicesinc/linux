/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _LINUX_GPIO_REGMAP_H
#define _LINUX_GPIO_REGMAP_H

struct device;
struct fwnode_handle;
struct gpio_regmap;
struct gpio_chip;
struct irq_domain;
struct regmap;

#define GPIO_REGMAP_ADDR_ZERO ((unsigned int)(-1))
#define GPIO_REGMAP_ADDR(addr) ((addr) ? : GPIO_REGMAP_ADDR_ZERO)

/**
 * enum gpio_regmap_operation - Operation type for reg_mask_xlate_op callback
 *
 * Traditionally, the operation type was inferred from the base register.
 * However, that approach does not always work — for example, when all control
 * bits of a single GPIO reside in the same register. This enum allows the
 * reg_mask_xlate_op callback to explicitly distinguish between operation
 * types. The user is free to choose which method to use.
 *
 * Read operation:
 * @GPIO_REGMAP_GET_OP: Indicates a read operation to get the current GPIO value.
 *
 * Write operation:
 * @GPIO_REGMAP_SET_OP: Indicates a write operation to set the GPIO output value.
 *
 * Direction operations:
 * @GPIO_REGMAP_GET_DIR_OP: Indicates a read operation to get the GPIO direction.
 * @GPIO_REGMAP_SET_DIR_OP: Indicates a write operation to set the GPIO direction.
 */
enum gpio_regmap_operation {
	GPIO_REGMAP_GET_OP,
	GPIO_REGMAP_SET_OP,
	GPIO_REGMAP_GET_DIR_OP,
	GPIO_REGMAP_SET_DIR_OP,
};

/**
 * struct gpio_regmap_config - Description of a generic regmap gpio_chip.
 * @parent:		The parent device
 * @regmap:		The regmap used to access the registers
 *			given, the name of the device is used
 * @pm_dev:		(Optional) Device to use for runtime power management.
 * @fwnode:		(Optional) The firmware node.
 *			If not given, the fwnode of the parent is used.
 * @label:		(Optional) Descriptive name for GPIO controller.
 *			If not given, the name of the device is used.
 * @ngpio:		Number of GPIOs
 * @names:		(Optional) Array of names for gpios
 * @reg_dat_base:	(Optional) (in) register base address
 * @reg_set_base:	(Optional) set register base address
 * @reg_clr_base:	(Optional) clear register base address
 * @reg_dir_in_base:	(Optional) in setting register base address
 * @reg_dir_out_base:	(Optional) out setting register base address
 * @reg_stride:		(Optional) May be set if the registers (of the
 *			same type, dat, set, etc) are not consecutive.
 * @ngpio_per_reg:	Number of GPIOs per register
 * @irq_domain:		(Optional) IRQ domain if the controller is
 *			interrupt-capable
 * @reg_mask_xlate:     (Optional) Translates base address and GPIO
 *			offset to a register/bitmask pair. If not
 *			given the default gpio_regmap_simple_xlate()
 *			is used.
 * @reg_mask_xlate_op:	(Optional) Operation-aware variant of
 *			@reg_mask_xlate. Takes precedence when both are set.
 * @drvdata:		(Optional) Pointer to driver specific data which is
 *			not used by gpio-remap but is provided "as is" to the
 *			driver callback(s).
 * @init_valid_mask:	(Optional) Routine to initialize @valid_mask, to be used
 *			if not all GPIOs are valid.
 *
 * The ->reg_mask_xlate and ->reg_mask_xlate_op callbacks translate a given
 * base address and GPIO offset to a register and mask pair. The base address
 * is one of the given register base addresses in this structure.
 *
 * Although all register base addresses are marked as optional, there are
 * several rules:
 *     1. if you only have @reg_dat_base set, then it is input-only
 *     2. if you only have @reg_set_base set, then it is output-only
 *     3. if you have either @reg_dir_in_base or @reg_dir_out_base set, then
 *        you have to set both @reg_dat_base and @reg_set_base
 *     4. if you have @reg_set_base set, you may also set @reg_clr_base to have
 *        two different registers for setting and clearing the output. This is
 *        also valid for the output-only case.
 *     5. @reg_dir_in_base and @reg_dir_out_base are exclusive; is there really
 *        hardware which has redundant registers?
 *
 * Note: All base addresses may have the special value %GPIO_REGMAP_ADDR_ZERO
 * which forces the address to the value 0.
 */
struct gpio_regmap_config {
	struct device *parent;
	struct regmap *regmap;
	struct device *pm_dev;
	struct fwnode_handle *fwnode;

	const char *label;
	int ngpio;
	const char *const *names;

	unsigned int reg_dat_base;
	unsigned int reg_set_base;
	unsigned int reg_clr_base;
	unsigned int reg_dir_in_base;
	unsigned int reg_dir_out_base;
	int reg_stride;
	int ngpio_per_reg;
	struct irq_domain *irq_domain;

	int (*reg_mask_xlate)(struct gpio_regmap *gpio, unsigned int base,
			      unsigned int offset, unsigned int *reg,
			      unsigned int *mask);
	int (*reg_mask_xlate_op)(struct gpio_regmap *gpio,
				 enum gpio_regmap_operation op,
				 unsigned int base, unsigned int offset,
				 unsigned int *reg, unsigned int *mask);

	int (*init_valid_mask)(struct gpio_chip *gc,
			       unsigned long *valid_mask,
			       unsigned int ngpios);

	void *drvdata;
};

struct gpio_regmap *gpio_regmap_register(const struct gpio_regmap_config *config);
void gpio_regmap_unregister(struct gpio_regmap *gpio);
struct gpio_regmap *devm_gpio_regmap_register(struct device *dev,
					      const struct gpio_regmap_config *config);
void *gpio_regmap_get_drvdata(struct gpio_regmap *gpio);

#endif /* _LINUX_GPIO_REGMAP_H */
