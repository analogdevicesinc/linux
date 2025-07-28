# SPDX-License-Identifier: GPL-2.0
#
#	settings.sh
#	Author: Michael Petlan <mpetlan@redhat.com>
#
#	Description:
#
#		This file contains global settings for the whole testsuite.
#	Its purpose is to make it easier when it is necessary i.e. to
#	change the usual sample command which is used in all of the tests
#	in many files.
#
#		This file is intended to be sourced in the tests.
#

#### which perf to use in the testing
export CMD_PERF=${CMD_PERF:-`which perf`}

#### basic programs examinated by perf
export CMD_BASIC_SLEEP="sleep 0.1"
export CMD_QUICK_SLEEP="sleep 0.01"
export CMD_LONGER_SLEEP="sleep 2"
export CMD_DOUBLE_LONGER_SLEEP="sleep 4"
export CMD_VERY_LONG_SLEEP="sleep 30"
export CMD_SIMPLE="true"

#### testsuite run mode
# define constants:
export RUNMODE_BASIC=0
export RUNMODE_STANDARD=1
export RUNMODE_EXPERIMENTAL=2
# default runmode is STANDARD
export PERFTOOL_TESTSUITE_RUNMODE=${PERFTOOL_TESTSUITE_RUNMODE:-$RUNMODE_STANDARD}

#### common settings
export TESTLOG_VERBOSITY=${TESTLOG_VERBOSITY:-2}
export TESTLOG_FORCE_COLOR=${TESTLOG_FORCE_COLOR:-n}
export TESTLOG_ERR_MSG_MAX_LINES=${TESTLOG_ERR_MSG_MAX_LINES:-20}
export TESTLOG_CLEAN=${TESTLOG_CLEAN:-y}

#### other environment-related settings
export TEST_IGNORE_MISSING_PMU=${TEST_IGNORE_MISSING_PMU:-n}

#### clear locale
export LC_ALL=C

#### colors
if [ -t 1 ] || [ "$TESTLOG_FORCE_COLOR" = "yes" ]; then
	export MPASS="\e[32m"
	export MALLPASS="\e[1;32m"
	export MFAIL="\e[31m"
	export MALLFAIL="\e[1;31m"
	export MWARN="\e[1;35m"
	export MSKIP="\e[33m"
	export MHIGH="\e[1;33m"
	export MEND="\e[m"
else
	export MPASS=""
	export MALLPASS=""
	export MFAIL=""
	export MALLFAIL=""
	export MWARN=""
	export MSKIP=""
	export MHIGH=""
	export MEND=""
fi

### general info
DIR_PATH=`dirname "$(readlink -e "$0")"`

TEST_NAME=`basename $DIR_PATH | sed 's/base/perf/'`; export TEST_NAME
MY_ARCH=`arch`; export MY_ARCH

# storing logs and temporary files variables
if [ -n "$PERFSUITE_RUN_DIR" ]; then
	# when $PERFSUITE_RUN_DIR is set to something, all the logs and temp files will be placed there
	# --> the $PERFSUITE_RUN_DIR/perf_something/examples and $PERFSUITE_RUN_DIR/perf_something/logs
	#     dirs will be used for that
	PERFSUITE_RUN_DIR=`readlink -f $PERFSUITE_RUN_DIR`; export PERFSUITE_RUN_DIR
	export CURRENT_TEST_DIR="$PERFSUITE_RUN_DIR/$TEST_NAME"
	export MAKE_TARGET_DIR="$CURRENT_TEST_DIR/examples"
	export LOGS_DIR="$CURRENT_TEST_DIR/logs"
	export HEADER_TAR_DIR="$CURRENT_TEST_DIR/header_tar"
	test -d "$CURRENT_TEST_DIR" || mkdir -p "$CURRENT_TEST_DIR"
	test -d "$LOGS_DIR" || mkdir -p "$LOGS_DIR"
else
	# when $PERFSUITE_RUN_DIR is not set, logs will be placed here
	export CURRENT_TEST_DIR="."
	export LOGS_DIR="."
	export HEADER_TAR_DIR="./header_tar"
fi


#### test parametrization
if [ ! -d ./common ]; then
	# set parameters based on runmode
	if [ -f ../common/parametrization.$PERFTOOL_TESTSUITE_RUNMODE.sh ]; then
		# shellcheck source=/dev/null
		. ../common/parametrization.$PERFTOOL_TESTSUITE_RUNMODE.sh
	fi
	# if some parameters haven't been set until now, set them to default
	if [ -f ../common/parametrization.sh ]; then
		. ../common/parametrization.sh
	fi
fi
