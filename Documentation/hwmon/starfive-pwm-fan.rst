.. SPDX-License-Identifier: GPL-2.0

Kernel driver starfive-pwm-fan
==============================

Supported chips:

  * StarFive JHB100

    Prefix: 'starfive_pwm_fan'

    Addresses scanned: -

Authors:
      - Changhuang Liang <changhuang.liang@starfivetech.com>

Description
-----------

This driver implements support for the PWM fan controller found on the
StarFive JHB100 SoC. Fan speed is controlled through a PWM signal supplied
by 8 PWM channels of an external PWM controller, and measured by a hardware
tachometer block with 16 independent tachometer inputs. Each tachometer channel
measures the number of pulses within a fixed 100 ms window.

Each fan-N child node in the devicetree references one PWM channel, which
may drive up to two fans, and lists the tachometer inputs wired to them.

Sysfs entries
-------------

===================== =======================================================
pwm[1-8]              PWM duty cycle (read/write), 0-255. pwmN corresponds
                      to the Nth fan child node.
fan[1-16]_input       Fan speed in RPM (read-only), without a fan connected,
                      reading this value can be significantly slow.
fan[1-16]_min         Lower fan speed limit in RPM (read/write)
fan[1-16]_enable      Enable/disable the tachometer channel (read/write)
fan[1-16]_fault       Fan stall indication (read-only), with the fan running
                      normally, reading this value can be significantly slow.
fan[1-16]_min_alarm   Fan speed below fan[1-16]_min (read-only), when
                      fan[1-16]_input is greater than or equal to
                      fan[1-16]_min, reading this value can be significantly
                      slow.
===================== =======================================================
