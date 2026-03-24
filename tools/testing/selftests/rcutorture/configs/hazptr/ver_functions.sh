#!/bin/bash
# SPDX-License-Identifier: GPL-2.0+
#
# Kernel-version-dependent shell functions for the rest of the scripts.
#
# Claude created this file, and I quote:
#
#	"I created [this file] modeled on the lock torture
#	version. It defines per_version_boot_params to pass
#	hazptrtorture.shutdown_secs=$3, hazptrtorture.stat_interval=15,
#	hazptrtorture.verbose=1, and optional CPU-hotplug parameters to
#	the kernel command line."
#
# I therefore kept locktorture's ver_functions.sh copyright notice:
#
# Copyright (C) Meta Platforms, Inc. and affiliates.
#
# Authors: Paul E. McKenney <paulmck@kernel.org>

# hazptrtorture_param_onoff bootparam-string config-file
#
# Adds onoff hazptrtorture module parameters to kernels having it.
hazptrtorture_param_onoff () {
	if ! bootparam_hotplug_cpu "$1" && configfrag_hotplug_cpu "$2"
	then
		echo CPU-hotplug kernel, adding hazptrtorture onoff. 1>&2
		echo hazptrtorture.onoff_interval=3 hazptrtorture.onoff_holdoff=30
	fi
}

# per_version_boot_params bootparam-string config-file seconds
#
# Adds per-version torture-module parameters to kernels supporting them.
per_version_boot_params () {
	echo	`hazptrtorture_param_onoff "$1" "$2"` \
		hazptrtorture.stat_interval=15 \
		hazptrtorture.shutdown_secs=$3 \
		hazptrtorture.verbose=1 \
		$1
}
