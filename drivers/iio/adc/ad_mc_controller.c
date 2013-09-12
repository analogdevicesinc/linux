/*
 * Analog Devices MC-Controller Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/io.h>

#include <linux/of_device.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define MOTOR_CONTROLLER_REG_VERSION            0x00
#define MOTOR_CONTROLLER_REG_ID                 0x04
#define MOTOR_CONTROLLER_REG_SCRATCH            0x08
#define MOTOR_CONTROLLER_REG_START_SPEED        0x0C
#define MOTOR_CONTROLLER_REG_CONTROL            0x10
#define MOTOR_CONTROLLER_REG_REFERENCE_SPEED    0x14
#define MOTOR_CONTROLLER_REG_KP                 0x18
#define MOTOR_CONTROLLER_REG_KI                 0x1C
#define MOTOR_CONTROLLER_REG_KD                 0x20
#define MOTOR_CONTROLLER_REG_KP1                0x24
#define MOTOR_CONTROLLER_REG_KI1                0x28
#define MOTOR_CONTROLLER_REG_KD1                0x2C
#define MOTOR_CONTROLLER_REG_PWM_OPEN           0x30
#define MOTOR_CONTROLLER_REG_PWM_BREAK          0x34
#define MOTOR_CONTROLLER_REG_STATUS             0x38
#define MOTOR_CONTROLLER_REG_ERR                0x3C

#define MOTOR_CONTROLLER_CONTROL_RUN(x)         ((x & 0x1) << 0)
#define MOTOR_CONTROLLER_CONTROL_DELTA(x)       ((x & 0x1) << 4)
#define MOTOR_CONTROLLER_CONTROL_SENSORS(x)     ((x & 0x3) << 8)
#define MOTOR_CONTROLLER_CONTROL_MATLAB(x)      ((x & 0x1) << 12)

const char motor_controller_sensors[3][8] = {"hall", "bemf", "resolver"};

struct motor_controller_state {
    struct mutex            lock;
    struct iio_info         iio_info;
    void __iomem            *regs;
};

static inline void motor_controller_write(struct motor_controller_state *st, unsigned reg, unsigned val)
{
    iowrite32(val, st->regs + reg);
}

static inline unsigned int motor_controller_read(struct motor_controller_state *st, unsigned reg)
{
    return ioread32(st->regs + reg);
}

static int motor_controller_reg_access(struct iio_dev *indio_dev,
                                       unsigned reg, unsigned writeval,
                                       unsigned *readval)
{
    struct motor_controller_state *st = iio_priv(indio_dev);

    mutex_lock(&indio_dev->mlock);
    if (readval == NULL) {
        motor_controller_write(st, reg & 0xFFFF, writeval);
    } else {
        *readval = motor_controller_read(st, reg & 0xFFFF);
    }
    mutex_unlock(&indio_dev->mlock);

    return 0;
}

enum motor_controller_iio_dev_attr {
    MOTOR_CONTROLLER_RUN,
    MOTOR_CONTROLLER_DELTA,
    MOTOR_CONTROLLER_SENSORS_AVAIL,
    MOTOR_CONTROLLER_SENSORS,
    MOTOR_CONTROLLER_PWM,
    MOTOR_CONTROLLER_KP,
    MOTOR_CONTROLLER_KI,
    MOTOR_CONTROLLER_KD,
    MOTOR_CONTROLLER_REF_SPEED,
    MOTOR_CONTROLLER_MATLAB,
};

static ssize_t motor_controller_show(struct device *dev,
            struct device_attribute *attr,
            char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
    struct motor_controller_state *st = iio_priv(indio_dev);
    int ret = 0;
    bool setting;
    u32 reg_val;
    u32 setting2;

    mutex_lock(&indio_dev->mlock);
    switch ((u32)this_attr->address) {
    case MOTOR_CONTROLLER_RUN:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        setting = (reg_val & MOTOR_CONTROLLER_CONTROL_RUN(-1));
        ret = sprintf(buf, "%u\n", setting);
        break;
    case MOTOR_CONTROLLER_DELTA:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        setting = (reg_val & MOTOR_CONTROLLER_CONTROL_DELTA(-1));
        ret = sprintf(buf, "%u\n", setting);
        break;
    case MOTOR_CONTROLLER_SENSORS_AVAIL:
        ret = sprintf(buf, "%s\n", "hall bemf resolver");
        break;
    case MOTOR_CONTROLLER_SENSORS:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        setting2 = (reg_val & MOTOR_CONTROLLER_CONTROL_SENSORS(-1)) >> 8;
        ret = sprintf(buf, "%s\n", motor_controller_sensors[setting2]);
        break;
    case MOTOR_CONTROLLER_PWM:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_PWM_OPEN);
        ret = sprintf(buf, "0x%x\n", reg_val);
        break;

    case MOTOR_CONTROLLER_KI:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_KI);
        ret = sprintf(buf, "%d\n", reg_val);
        break;
    case MOTOR_CONTROLLER_KD:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_KD);
        ret = sprintf(buf, "%d\n", reg_val);
        break;
    case MOTOR_CONTROLLER_KP:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_KP);
        ret = sprintf(buf, "%d\n", reg_val);
        break;
    case MOTOR_CONTROLLER_REF_SPEED:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_REFERENCE_SPEED);
        ret = sprintf(buf, "%d\n", reg_val);
        break;
    case MOTOR_CONTROLLER_MATLAB:
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        setting = (reg_val & MOTOR_CONTROLLER_CONTROL_MATLAB(-1));
        ret = sprintf(buf, "%u\n", setting);
        break;
    default:
        ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);

    return ret;
}

static ssize_t motor_controller_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t len)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
    struct motor_controller_state *st = iio_priv(indio_dev);
    int ret = 0;
    bool setting;
    u32 reg_val;
    u32 setting2;

    mutex_lock(&indio_dev->mlock);
    switch ((u32)this_attr->address) {
    case MOTOR_CONTROLLER_RUN:
        ret = strtobool(buf, &setting);
        if (ret < 0)
            break;
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        reg_val &= ~MOTOR_CONTROLLER_CONTROL_RUN(-1);
        reg_val |= MOTOR_CONTROLLER_CONTROL_RUN(setting);
        motor_controller_write(st, MOTOR_CONTROLLER_REG_CONTROL, reg_val);
        break;
    case MOTOR_CONTROLLER_DELTA:
        ret = strtobool(buf, &setting);
        if (ret < 0)
            break;
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        reg_val &= ~MOTOR_CONTROLLER_CONTROL_DELTA(-1);
        reg_val |= MOTOR_CONTROLLER_CONTROL_DELTA(setting);
        motor_controller_write(st, MOTOR_CONTROLLER_REG_CONTROL, reg_val);
        break;
    case MOTOR_CONTROLLER_SENSORS:
        if (sysfs_streq(buf, "hall"))
            setting2 = 0;
        else if (sysfs_streq(buf, "bemf"))
            setting2 = 1;
        else if (sysfs_streq(buf, "resolver"))
            setting2 = 2;
        else
            break;
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        reg_val &= ~MOTOR_CONTROLLER_CONTROL_SENSORS(-1);
        reg_val |= MOTOR_CONTROLLER_CONTROL_SENSORS(setting2);
        motor_controller_write(st, MOTOR_CONTROLLER_REG_CONTROL, reg_val);
        break;
    case MOTOR_CONTROLLER_PWM:
        ret = kstrtou32(buf, 16, &reg_val);
        if (ret < 0)
            break;
        motor_controller_write(st, MOTOR_CONTROLLER_REG_PWM_OPEN, reg_val);
        break;
    case MOTOR_CONTROLLER_KI:
        ret = kstrtou32(buf, 10, &reg_val);
        if (ret < 0)
            break;
        motor_controller_write(st, MOTOR_CONTROLLER_REG_KI, reg_val);
        break;
     case MOTOR_CONTROLLER_KP:
        ret = kstrtou32(buf, 10, &reg_val);
        if (ret < 0)
            break;
        motor_controller_write(st, MOTOR_CONTROLLER_REG_KP, reg_val);
        break;
     case MOTOR_CONTROLLER_KD:
        ret = kstrtou32(buf, 10, &reg_val);
        if (ret < 0)
            break;
        motor_controller_write(st, MOTOR_CONTROLLER_REG_KD, reg_val);
        break;
    case MOTOR_CONTROLLER_REF_SPEED:
        ret = kstrtou32(buf, 10, &reg_val);
        if (ret < 0)
            break;
        motor_controller_write(st, MOTOR_CONTROLLER_REG_REFERENCE_SPEED, reg_val);
        break;
    case MOTOR_CONTROLLER_MATLAB:
        ret = strtobool(buf, &setting);
        if (ret < 0)
            break;
        reg_val = motor_controller_read(st, MOTOR_CONTROLLER_REG_CONTROL);
        reg_val &= ~MOTOR_CONTROLLER_CONTROL_MATLAB(-1);
        reg_val |= MOTOR_CONTROLLER_CONTROL_MATLAB(setting);
        motor_controller_write(st, MOTOR_CONTROLLER_REG_CONTROL, reg_val);
        break;
    default:
        ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);

    return ret ? ret : len;
}

static IIO_DEVICE_ATTR(motor_controller_run, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_RUN);

static IIO_DEVICE_ATTR(motor_controller_delta, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_DELTA);

static IIO_DEVICE_ATTR(motor_controller_sensors_available, S_IRUGO,
            motor_controller_show,
            NULL,
            MOTOR_CONTROLLER_SENSORS_AVAIL);

static IIO_DEVICE_ATTR(motor_controller_sensors, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_SENSORS);

static IIO_DEVICE_ATTR(motor_controller_pwm, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_PWM);

static IIO_DEVICE_ATTR(motor_controller_ki, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_KI);

static IIO_DEVICE_ATTR(motor_controller_kd, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_KD);

static IIO_DEVICE_ATTR(motor_controller_kp, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_KP);

static IIO_DEVICE_ATTR(motor_controller_ref_speed, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_REF_SPEED);

static IIO_DEVICE_ATTR(motor_controller_matlab, S_IRUGO | S_IWUSR,
            motor_controller_show,
            motor_controller_store,
            MOTOR_CONTROLLER_MATLAB);

static struct attribute *motor_controller_attributes[] = {
    &iio_dev_attr_motor_controller_run.dev_attr.attr,
    &iio_dev_attr_motor_controller_delta.dev_attr.attr,
    &iio_dev_attr_motor_controller_sensors_available.dev_attr.attr,
    &iio_dev_attr_motor_controller_sensors.dev_attr.attr,
    &iio_dev_attr_motor_controller_pwm.dev_attr.attr,
    &iio_dev_attr_motor_controller_ki.dev_attr.attr,
    &iio_dev_attr_motor_controller_kd.dev_attr.attr,
    &iio_dev_attr_motor_controller_kp.dev_attr.attr,
    &iio_dev_attr_motor_controller_ref_speed.dev_attr.attr,
    &iio_dev_attr_motor_controller_matlab.dev_attr.attr,
    NULL,
};

static const struct attribute_group motor_controller_attribute_group = {
    .attrs = motor_controller_attributes,
};

static const struct iio_info motor_controller_info = {
    .driver_module = THIS_MODULE,
    .debugfs_reg_access = &motor_controller_reg_access,
    .attrs = &motor_controller_attribute_group,
};

static int motor_controller_of_probe(struct platform_device *op)
{
    struct iio_dev *indio_dev;
    struct device *dev = &op->dev;
    struct motor_controller_state *st;
    struct resource r_mem;
    resource_size_t remap_size, phys_addr;
    int ret;

    ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
    if (ret) {
        dev_err(dev, "Invalid address\n");
        return ret;
    }

    indio_dev = iio_device_alloc(sizeof(*st));
    if (indio_dev == NULL)
        return -ENOMEM;

    st = iio_priv(indio_dev);

    dev_set_drvdata(dev, indio_dev);
    mutex_init(&st->lock);

    phys_addr = r_mem.start;
    remap_size = resource_size(&r_mem);
    if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
        dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
            (unsigned long long)phys_addr);
        ret = -EBUSY;
        goto failed;
    }

    st->regs = ioremap(phys_addr, remap_size);
    if (st->regs == NULL) {
        dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
            (unsigned long long)phys_addr);
        ret = -EFAULT;
        goto failed;
    }

    indio_dev->dev.parent = dev;
    indio_dev->name = op->dev.of_node->name;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->info = &motor_controller_info;

    ret = iio_device_register(indio_dev);
    if (ret)
        goto failed;

    return 0;

failed:
    return -1;
}

static int motor_controller_of_remove(struct platform_device *op)
{
    return 0;
}

static const struct of_device_id motor_controller_of_match[] = {
    { .compatible = "xlnx,axi-ad-mc-controller-1.00.a", },
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, motor_controller_of_match);

static struct platform_driver motor_controller_of_driver = {
    .driver = {
        .name = KBUILD_MODNAME,
        .owner = THIS_MODULE,
        .of_match_table = motor_controller_of_match,
    },
    .probe      = motor_controller_of_probe,
    .remove     = motor_controller_of_remove,
};

module_platform_driver(motor_controller_of_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-Controller");
MODULE_LICENSE("GPL v2");
