.. SPDX-License-Identifier: GPL-2.0

==============================
ADI Trigger Routing Unit (TRU)
==============================

The adi-tru driver exports a simple sysfs interface.  This allows you to
control TRU and view its status.

Files
=====

Each TRU device will have a set of enable, reset, status, mtr and ssr/ssrN
files.

The enable file is used to enable/disable the device.

The reset file is used to do a soft reset of the device.

The status file is used to view the error status of the device.

The mtr file is used to manually generate triggers by writing source IDs.

The ssr/ssrN files are used to connect source trigger IDs to target trigger IDs.

Example
=======

Locate the device in your sysfs tree.  This is probably easiest by going into
the platform devices directory and locating the device by its base address::

	# ls /sys/bus/platform/devices/
	...
	3fe0b000.tru
	3fe0c000.tru
	...

Let's take a look at one of the TRU devices (unrelated sysfs entries have been
trimmed)::

	# ls /sys/bus/platform/devices/3fe0b000.tru
	enable mtr reset ssr status
	# ls /sys/bus/platform/devices/3fe0b000.tru/ssr
	ssr0    ssr108  ssr118  ssr2   ssr3   ssr4   ssr5   ssr6   ssr7   ssr8   ssr9
	ssr1    ssr109  ssr119  ssr20  ssr30  ssr40  ssr50  ssr60  ssr70  ssr80  ssr90
	...

You can use simple reads/writes to access these files::

	# cd /sys/bus/platform/devices/3fe0b000.tru

	# cat enable
	1
	# echo 0 > enable
	# cat enable
	0
	# echo 1 > enable
	# cat enable
	1

	# cat status
	0: no address error, no lock write error

	# cat ssr/ssr0
	0
	# echo 23 > ssr/ssr0
	# cat ssr/ssr0
	23

	# cat ssr/ssr1
	0
	# echo "45,locked" > ssr/ssr1
	# cat ssr/ssr1
	45(locked)
	# echo 0 > ssr/ssr1
	-bash: echo: write error: Invalid argument

	# echo 0x1245 > mtr
	# cat mtr
	0x00001245

	# echo 1 > reset
