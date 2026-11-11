.. SPDX-License-Identifier: GPL-2.0-only

Microchip LAN969x fan controller
================================

Supported chips:

  * Microchip LAN969x

    Prefix: 'lan969x_fan'

    Addresses scanned: -

    Datasheet: https://microchip-ung.github.io/lan969x-industrial_reginfo/

Description
-----------

The LAN969x fan controller provides a tachometer input and a PWM output.

The tachometer counts pulses over one-second intervals, independently of the
PWM output. The fan child's pulses-per-revolution property specifies how many
pulses represent one revolution; it defaults to two.

The fan child's pwms property selects the initial PWM period and polarity.
The output is dedicated to this fan and is not available to external PWM
consumers. The pwm1 value describes the logical duty cycle independently of
the output polarity. Frequency requests are rounded to the nearest hardware
divider value; requests outside the divider range are rejected. pwm1_freq
reports the resulting frequency.

Sysfs entries
-------------

======================= ========================================================
fan1_input              Fan speed in RPM.
pwm1                    PWM duty cycle, from 0 to 255.
pwm1_freq               PWM frequency in Hz.
======================= ========================================================

Thermal cooling
---------------

With device-tree thermal support enabled, the fan can also act as a thermal
cooling device. The fan child's cooling-levels property maps cooling states
to PWM values from 0 to 255, in ascending order. When cooling levels are
provided, the driver initially selects the highest level.

Manual pwm1 writes remain available and update the reported cooling state
to the highest state whose PWM value does not exceed the requested value,
or state zero if the value is below the first level. Subsequent thermal
cooling requests can override manual PWM settings. Each cooling request
programs the exact PWM value associated with the requested state.
