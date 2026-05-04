RDS self-tests
==============

These scripts provide a coverage test for RDS-TCP by creating two
network namespaces and running rds packets between them. A loopback
network is provisioned with optional probability of packet loss or
corruption. A workload of 50000 hashes, each 64 characters in size,
are passed over an RDS socket on this test network. A passing test means
the RDS-TCP stack was able to recover properly.  The provided config.sh
can be used to compile the kernel with the necessary gcov options.  The
kernel may optionally be configured to omit the coverage report as well.

USAGE:
	run.sh [-d logdir] [-l packet_loss] [-c packet_corruption]
	       [-u packet_duplicate] [-t timeout]

OPTIONS:
	-d	Log directory.  If set, logs will be stored in the
		given dir, or skipped if unset.  Log dir can also be
		set through the RDS_LOG_DIR env variable

	-l	Simulates a percentage of packet loss

	-c	Simulates a percentage of packet corruption

	-u	Simulates a percentage of packet duplication.

	-t	Test timeout.  Defaults to tools/testing/selftests/net/rds/settings

ENV VARIABLES:
	RDS_LOG_DIR	Log directory.  If set, logs will be stored in
			the given dir, or skipped if unset. Log dir
			can also be set with the -d flag.

			Use with --rwdir on the CI path to retain logs after
			test compleation.  Log dir end point must be within
			the specified --rwdir path for logs to persist on
			the host.

EXAMPLE:

    # Create a suitable gcov enabled .config
    tools/testing/selftests/net/rds/config.sh -g

    # Alternatly create a gcov disabled .config
    tools/testing/selftests/net/rds/config.sh

    # Config paths may also be specified with the -c flag
    tools/testing/selftests/net/rds/config.sh -c .config.local

    # build the kernel
    vng --build --config .config

    # launch the tests in a VM
    vng -v --rwdir ./ --run . --user root --cpus 4 -- \
        "export PYTHONPATH=tools/testing/selftests/net/; \
         export RDS_LOG_DIR=tools/testing/selftests/net/rds/rds_logs; \
         tools/testing/selftests/net/rds/run.sh"

