.. description::

   Continuous deployment pipeline and instructions to set up a self-hosted
   GitHub Actions runner using Podman or Docker and systemd without root.

.. _ci:

Continuous integration
======================

Linux has a continuous deployment integration pipeline that works as follows:

::

   push (main)
   pull-request
   ┌──────┐    ┌───────────────┐    ┌──────┐
   │Checks├─#─►│Build Kernel * ├─#─►│Assert│
   └──────┘    └───────────────┘ ▲  └──────┘
   ┌────────────────┐            │
   │Build Kernel arm├────────────┘
   └────────────────┘
   ┌─────────┐    ┌──────────┐
   │Build Doc├─?─►│Deploy Doc│
   └─────────┘    └──────────┘
   -----------------------------------------
   cron
   ┌──────────────┐
   │Update mirrors│
   └──────────────┘

In these sections there are instructions to bring-up your own continuous integration,
either to run locally, in a self-hosted runner or even in a cluster.

Beyond the linear history ``main`` branch, the following upstream mirrors are
provided (``mirror_ci/<remote-name>/<branch>``):

* | ``mirror_ci/next/linux-next/master``: Patches aimed at the next kernel merge window
  | https://git.kernel.org/pub/scm/linux/kernel/git/next/linux-next.git/log/?h=master
* | ``mirror_ci/jic23/iio/testing``: Linux IIO Subsystem (Cameron's branch)
  | https://git.kernel.org/pub/scm/linux/kernel/git/jic23/iio.git/log/?h=testing
* | ``mirror_ci/lee/mfd/for-mfd-next``: MFD Subsystem Tree - Next and Fixes
  | https://git.kernel.org/pub/scm/linux/kernel/git/lee/mfd.git/log/?h=for-mfd-next
* | ``mirror_ci/groeck/linux-staging/hwmon-next``: HWMON Subsystem - Staging
  | https://git.kernel.org/pub/scm/linux/kernel/git/groeck/linux-staging.git/log/?h=hwmon-next

To reduce the load on ``kernel.org``, clone from mirrors such as
https://kernel.googlesource.com/pub/scm/linux/kernel/git/.

When upstreaming a driver, target the pull-request against the mirror.

.. attention::

   Do **not** base your work on the head of the ``mirror_ci/*``, it is
   frequently force pushed either by upstream (they are testing branches after
   all). Instead, find a common stable base, for example the previous tag such
   as `v6.19-rc1 <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/commit/?h=v6.19-rc1&id=8f0b4cce4481fb22653697cced8d0d04027cb1e8>`__

All of them are mirrors from the links shown with a single commit on top
that includes the CI workflows, :git-linux:`as simple as <ci:.github/workflows/mirror.yml>`:

.. shell::

   $git checkout origin/main -- .github
   $git checkout origin/main -- ci
   $git add -f .github ci
   $git commit -m "ci: Patch workflows" -m "CI generated commit" -s

The cron job is set to update the branches at the end of every day.

Features
--------

The continuous integration has some features that are described in this section.

Warning and error handling
~~~~~~~~~~~~~~~~~~~~~~~~~~

Each tool will have its own outputs and the standard to consider something
an error, a warning or something else.
So the workflows separate the steps outputs into steps events named
``fail``, and ``warn``:

* ``fail``: Indicates that a warning or error deemed strict was raised, and
  or what the method that must succeed returns. Will fail the step if not
  captured (``|| true``). The CI allows some steps to fail, in order to
  collect all failures and assert the job at the end.
* ``warn``: Indicates that a warning or error non-deemed strict was raised.
  Is collected at the assert job and **does not** end the run with failure.

When ``continue-on-error: true`` is used at the job level, the following
topology rules must be followed based on the immediate downstream step:

* If downstream is an assert job, the ci already fails on ``step_fail_*``, so a
  ``failure()`` step should just set ``set_step_fail "assert_state"``, as well
  the job-scoped ``fatal=true`` environment variable.
* If downstream is an regular job, the job must export an ``fail=$fail`` output,
  to be used alongside the ``needs`` rule, for example:

  .. code:: yaml

     build_gcc_aarch64:
       needs: [checks]
       if: needs.checks.outputs.fatal != 'true'

These rules ensure that the downstream jobs do not run on fatal errors.
Optional steps may also set ``fatal`` job-scoped environment variable, if taking
care of calling ``set_step_fail``.

Checking steps description
~~~~~~~~~~~~~~~~~~~~~~~~~~

The purpose of the continuous integrations is to run all the latest and greatest
code checkers.
To reduce noise, the checkers are run on the commit range or changed files only,
when possible.

The checkers are, followed by which step event they raise:

* Check job:

  - | ``checkpatch``: runs on every commit of the commit range.
    | ``fail``: ``error`` logged
    | ``warn``: ``warning`` logged
  - | ``coccicheck``: every coccifile is applied on every changed *.c* file, using ``spatch``.
    | ``warn``: ``warning`` logged
  - | ``dt_binding_check``: on every changed *.yaml* file.
    | ``warn``: ``warning`` logged
    | ``fail``: contain file name or dts example return error code
  - | ``cppcheck``: runs on every changed file.
    | ``fail``: ``error`` logged
    | ``warn``: ``warning`` logged

* Build matrix job:

  - | ``make defconfig``: generate the defconfig.
    | ``err``: returned error code
  - | ``make``: compile kernel at head commit.
    | ``err``: returned error code
  - | ``assert compiled``: check if touched *.c* have been compiled.
    | ``fail``: *.o* file not found for touched *.c* file


* Build matrix job with checks:

  - | ``sparse``: the changed files are touched and recompiled with ``C=1``.
    | ``fail``: ``error`` logged
    | ``warn``: ``warning`` logged
  - | ``gcc fanalyzer``: the changed files are recompiled appending the ``-fanalyzer`` flag.
    | It uses the *compile_commands.json* file to extract the correct compilation flags.
    | ``fail``: ``error`` or ``warning`` logged
  - | ``smatch``: the changed files are touched and recompiled with ``C=1 CHECK="smatch -p=kernel"``.
    | ``fail``: ``error`` logged
    | ``warn``: ``warning`` logged

Defconfigs
~~~~~~~~~~

If the defconfig of a target doesn't exist, it falls back to the default configuration
of that architecture.

A "temporary" commit can be used to manipulate the defconfigs for faster build times.

Source code manipulation
~~~~~~~~~~~~~~~~~~~~~~~~

Cocci and bash scripts at ``ci/prerun`` are executed right after the ``.config`` is generated
and before it is saved and the kernel compiled.
For the check job, they are applied right after checkout.
It allows manipulating the source code depending on the run conditions, and can be used
as "adapters" when targeting multiple branches, architecture, and so on.

Each cocci/bash is executed taking each touched file as the argument,
so ensure to filter on the scripts themselves which file they manipulate.
Cocci files are applied only to ``.c`` files.

Here is a simple example that changes a method argument type:

.. code-block::
   :caption: backport.cocci

   @ change_arg_type @
   identifier func = adc_write_event_config;
   identifier arg;
   type T = enum iio_event_direction;
   @@

   func(...,
   - T arg
   + enum iio_event_direction_ex arg
   , ... ) { ... }

Setting up and running
----------------------

In this section there are instructions to bring-up your own continuous integration,
either to run locally, in a self-hosted runner or even in a cluster.

The container engine you use, like as ``podman`` or ``docker``, is up to you.
Limited instructions for each are provided in this section, you should consult
their source documentation for detailed information.

.. _conf-podman:

Configure podman
~~~~~~~~~~~~~~~~

Below are suggested instructions for setting up ``podman`` on a Linux environment,
if you wish to use it as your container engine. If you already use something else
like ``docker``, **keep it** and skip this section.

Adjust to your preference as needed, and skip the steps marked in :green:`green`
if not using WSL2.

Install ``podman`` from your package manager.

:green:`Ensure cgroup v2 on wsl2's .wslconfig:`

::

   [wsl2]
   kernelCommandLine = cgroup_no_v1=all systemd.unified_cgroup_hierarchy=1

:green:`Restart wsl2.`

Enable ``podman`` service for your user.

.. shell::

   $systemctl enable --now --user podman.socket
   $systemctl start --user podman.socket

Set the ``DOCKER_HOST`` variable on your *~/.bashrc*:

.. code-block:: bash

   export DOCKER_HOST=unix://$XDG_RUNTIME_DIR/podman/podman.sock

.. _podman sssd:

Network users & partitions
^^^^^^^^^^^^^^^^^^^^^^^^^^

Podman default configuration expects a local user to be able to create a user
namespace where multiple IDs are mapped and a compatible partition to use as
the storage location ``graphRoot``.

.. note::

   The ideal solution is to create a local **non-root** user and storage
   location. Podman processes should then be started under this user UID.

Network systems using solutions such as `SSSD <https://sssd.io/>`__ do not
append the user to the system (is not listed in ``/etc/subuid``), so automatic
user namespace is not possible. To be compatible with this configuration, a
single UID within a user space needs to be used, achieved with the
``ignore_chown_errors`` parameter.

Normally these systems also mount an network file system (nfs) as the home folder,
which is also not supported.
In this case, the ``graphRoot`` location needs to be set to somewhere else
(an easy test location is ``/tmp``).

This is an example of *~/.config/containers/storage.conf* to support such
environments:

.. code:: ini

   [storage]
   driver = "overlay"
   # Set to a path in a non-nfs partition
   graphRoot = "/tmp"

   [storage.options.overlay]
   # Single UID
   ignore_chown_errors = "true"

Ensure apply with ``podman system migrate`` and see the changed settings with
``podman info``.

An alternative mitigation for nfs is to create a xfs disk image and mount, but
since mount requires a root permission it is unlikely to be helpful for most
users:

.. code:: bash

   truncate -s 100g ~/.local/share/containers-xfs.img
   mkfs.xfs -m reflink=1  ~/.local/share/containers-xfs.img -m bigtime=1,inobtcount=1 -i nrext64=0
   sudo mount ~/.local/share/containers-xfs.img ~/.local/share/containers

.. _image-podman:

Build the container image
~~~~~~~~~~~~~~~~~~~~~~~~~

To build the container image, use your favorite container engine from the
:git-linux:`ci branch <ci:>`:

.. shell::

   $cd ~/linux
   $git branch
    * ci
      main
   $alias container=podman # or docker, ...
   $container build --tag adi/linux:v4 container

You may want to build the container in a host, where you have all your tools installed,
and then deploy to a server.
In this case, export the image and then import on the server:

.. shell::
   :show-user:
   :user: user
   :group: host

   ~/linux
   $container save -o adi-linux.tar adi/linux:v4
   $scp adi-linux.tar server:/tmp/

.. shell::
   :show-user:
   :user: admin
   :group: server

   /tmp
   $container load -i adi-linux.tar

Or if you are feeling adventurous:

.. shell::
   :show-user:
   :user: user
   :group: host

   ~/linux
   $container save adi/linux:v4 | ssh server "cat - | podman load"

.. _interactive-run:

Interactive run
~~~~~~~~~~~~~~~

The :git-doctools:`container-run.sh <ci/scripts/container-run.sh>` is a
suggested container command to interactive login into an image, mounting the
provided path and preparting the :git-linux:`_ci <ci:>` worktree to leverage the
same scripts used by the continuous integration.

You can use it to compile/runs checks using persistent cache, for example:

.. shell::

   ~/linux
   $cr adi/linux:v4
   $set_arch gcc_aarch64
    ARCH=arm64
    CXX=gcc-14
    CROSS_COMPILE=aarch64-suse-linux-
   $make adi_ci_defconfig
    #
    # configuration written to .config
    #
   $make -j$(nproc)
    UPD     include/generated/compile.h
    CALL    scripts/checksyscalls.sh
    CC      init/version.o
    AR      init/built-in.a
    [ ... ]
   $exit

.. tip::

   The enviroment variables ``base_sha`` and ``head_sha`` define the range of
   commits the steps are run against, adjust it to match your series.
   By default, it is set to ``@~6..@``.

Or:

.. shell::

   ~/linux
   $cr adi/linux:v4
   $base_sha=@~2
   $check_checkpatch
    checkpatch on range @~6..@
    Collecting ply
    Downloading ply-3.11-py
    [...]
    Documentation/hwmon/adp1050.rst
    drivers/hwmon/pmbus/adp1050.c
    total: 0 errors, 0 warnings, 0 checks, 179 lines checked

Significantly speeding up interactive testing.
Remember to replace ``container_engine`` variable with your preferred container engine.

.. collapsible:: First usage output example.

   .. shell::

      ~/linux
      $cr adi/linux:v4
       Remote 'public' matches 'analogdevicesinc', and has branch 'ci'.
       Fetch (y/n)? y
       Fetching branch 'ci'...
       Fetched CI branch to '_ci'.

       To clean-up, use:
         git worktree remove _ci

       Keep it up to date as well with git pull.

       Sourced methods from '/mnt/wsl/data/repos/linux-factory/ci/_ci/ci/build.sh':
       check_checkpatch                compile_many_devicetrees        assert_compiled
       check_dt_binding_check          compile_kernel                  apply_prerun
       check_coccicheck                compile_kernel_sparse           auto_set_kconfig
       check_license                   compile_kernel_smatch           set_arch
       check_assert_defconfigs         compile_gcc_fanalyzer           set_step_warn
       compile_devicetree              compile_clang_analyzer          set_step_fail
       The git commit range was set to:
       $base_sha..$head_sha: @~6..@
       Adjust '$base_sha' and '$head_sha' to your work range.
       Directory: /mnt/wsl/data/repos/linux-factory/main
       Mon Oct 13 09:53:55 UTC 2025
      $set_arch
       Missing architecture, usage:
         set_arch <arch>
       Available architectures:
         llvm_x86 gcc_arm gcc_aarch64 gcc_x86
      $set_arch gcc_arm
       CXX=gcc-13
       ARCH=arm
       CROSS_COMPILE=arm-suse-linux-gnueabi-
      $base_sha=@~3
      $auto_set_kconfig
       auto_set_kconfig on range @~3..@
       Symbols of touched files:
       {'SENSORS_ADP1050', 'AD9081'}
       Resolved symbols:
       {'HWMON', 'I2C', 'SPI', 'AD9081', 'PMBUS', 'HAS_IOMEM', 'IIO', 'SENSORS_ADP1050'}
       #
       # configuration written to .config
       #
      $compile_kernel
       kernel
       SYNC include/config/auto.conf
       UPD include/generated/compile.h
       CALL scripts/checksyscalls.sh
       CC init/version.o
       AR init/built-in.a
       UPD kernel/config_data
       GZIP kernel/config_data.gz
       CC [M] kernel/configs.o
       CC [M] drivers/hwmon/pmbus/pmbus_core.o
       [..]
       OBJCOPY arch/arm/boot/zImage
       Kernel: arch/arm/boot/zImage is ready
      $assert_compiled
       assert_compiled were compiled on range @~3..@
       drivers/hwmon/pmbus/adp1050.c
       drivers/iio/adc/ad9081.c
      $

.. _podman-run:

Self-hosted runner
~~~~~~~~~~~~~~~~~~

To host your `GitHub Actions Runner <https://github.com/actions/runner>`__,
set up your secrets (``podman`` only):

.. shell::

   # e.g. analogdevicesinc/linux
   $printf ORG_REPOSITORY | podman secret create public_linux_org_repository -
   # e.g. MyVerYSecRunnerToken
   $printf RUNNER_TOKEN | podman secret create public_linux_runner_token -

The runner token is obtained from the GUI at ``https://github.com/<org>/<repository>/settings/actions/runners/new``.

If ``github_token`` from :ref:`cluster-podman` is set, the runner_token
is ignored and a new one is requested.

.. shell::

   ~/linux
   $podman run \
   $    --secret public_linux_org_repository,type=env,target=org_repository \
   $    --secret public_linux_runner_token,type=env,target=runner_token \
   $    --env runner_labels=v1,big_cpu \
   $    adi/linux:v4

.. collapsible:: Docker alternative

   ``docker`` does **not** have a built-in keyring, instead you pass directly
   to ``run`` command. :red:`Consider hardening strategies to mitigate risks`,
   like using another keyring as below.

   .. shell::

      ~/linux
      $docker run \
      $    --env public_linux_org_repository=$(gpg --quiet --batch --decrypt /run/secrets/public_linux_org_repository.gpg) \
      $    --env public_linux_runner_token=$(gpg --quiet --batch --decrypt /run/secrets/public_linux_runner_token.gpg) \
      $    --env runner_labels=v1,big_cpu \
      $    localhost/adi/linux:v4

The environment variable runner_labels (comma-separated), set the runner labels.
If not provided on the Containerfile as ``ENV runner_labels=<labels,>`` or as argument
``--env runner_labels=<labels,>``, it defaults to ``v1``.
Most of the time, you want to use the Containerfile-set environment variable.

If you are in an environment as described in :ref:`podman sssd`, append these flags
to every ``podman run`` command:

* ``--user root``: due to ``ignore_chown_errors`` allowing a single user mapping,
  this user is root (0). Please note that this the container's root user and in
  most images is the only available user.
* ``--env RUNNER_ALLOW_RUNASROOT=1``: suppresses the GitHub Action runner "Must
  not run with sudo". Again, is the container's root.

.. _cluster-podman:

Self-hosted cluster
~~~~~~~~~~~~~~~~~~~

To host a cluster of self-hosted runners, the recommended approach is to use
systemd services, instead of for example, container compose solutions.

Below is a suggested systemd service at *~/.config/systemd/user/container-public-linux@.service*.

.. code:: systemd

   [Unit]
   Description=container public linux ci %i
   Wants=network-online.target

   [Service]
   Restart=on-success
   ExecStart=/bin/podman run \
             --env name_label=%H-%i \
             --secret public_linux_org_repository,type=env,target=org_repository \
             --secret public_linux_runner_token,type=env,target=runner_token \
             --conmon-pidfile %t/%n-pid --cidfile %t/%n-cid \
             --label "io.containers.autoupdate=local" \
             --name=public_linux_%i \
             --memory-swap=20g \
             --memory=16g \
             --cpus=4 \
             -d adi/linux:v4 top
   ExecStop=/bin/sh -c "/bin/podman stop -t 300 $(cat %t/%n-cid) && /bin/podman rm $(cat %t/%n-cid)"
   ExecStopPost=/bin/rm %t/%n-pid %t/%n-cid
   TimeoutStopSec=600
   Type=forking
   PIDFile=%t/%n-pid

   [Install]
   WantedBy=multi-user.target

.. collapsible:: Docker alternative

   .. code:: systemd

      [Unit]
      Description=container public linux ci %i
      Requires=gpg-passphrase.service
      Wants=network-online.target
      After=docker.service

      [Service]
      Restart=on-success
      ExecStart=/bin/sh -c "/bin/docker run \
                --env name_label=%H-%i \
                --env org_repository=$(gpg --quiet --batch --decrypt /run/secrets/public_linux_org_repository.gpg) \
                --env runner_token=$(gpg --quiet --batch --decrypt /run/secrets/public_linux_runner_token.gpg) \
                --cidfile %t/%n-cid \
                --label "io.containers.autoupdate=local" \
                --name=public_linux_%i \
                --memory-swap=20g \
                --memory=16g \
                --cpus=4 \
                --log-driver=journald \
                -d localhost/adi/linux:v4 top"
      RemainAfterExit=yes
      ExecStop=/bin/sh -c "/bin/docker stop -t 300 $(cat %t/%n-cid) && /bin/docker rm $(cat %t/%n-cid)"
      ExecStopPost=/bin/rm %t/%n-cid
      TimeoutStopSec=600
      Type=forking

      [Install]
      WantedBy=multi-user.target

Remember to ``systemctl --user daemon-reload`` after modifying.
With `autoupdate <https://docs.podman.io/en/latest/markdown/podman-auto-update.1.html>`__,
if the image-digest of the container and local storage differ,
the local image is considered to be newer and the systemd unit gets restarted.

Tune the limit flags for your needs.
The ``--cpus`` flag requires a kernel with ``CONFIG_CFS_BANDWIDTH`` enabled.
You can check with ``zgrep CONFIG_CFS_BANDWIDTH= /proc/config.gz``.

Instead of passing ``runner_token``, you can also pass a ``github_token`` to
generate the ``runner_token`` on demand. Using the ``github_token`` is the
recommended approach because during clean-up the original runner_token may have
expired already.

Alternatively, you can mount a FIFO to ``/var/run/secrets/runner_token`` to
generate a token just in time, without ever passing the github_token to the
container (scripts not provided).

However, please note, just like the GitHub Actions generated ``GITHUB_TOKEN``,
the path ``/run/secrets/runner_token`` can be read by workflows, while the
previous option is removed from the environment prior executing the GitHub
Actions runtime.

The order of precedence for authentication token is:

#. ``github_token``: environment variable.
#. ``runner_token``: plain text or FIFO at */run/secrets/runner_token*.
#. ``runner_token``: environment variable.

Please understand the security implications and ensure the token secrecy,
by for example, require manual approval for running workflows PRs from
third party sources and don't relax ``runner`` user permissions.

The required GitHub Fine-Grained token permission should be set as follows:

For `repository runner <https://docs.github.com/en/rest/actions/self-hosted-runners?apiVersion=2022-11-28#create-a-registration-token-for-a-repository--fine-grained-access-tokens>`_:

* ``administration:write``: "Administration" repository permissions (write).

For `org runner <https://docs.github.com/en/rest/actions/self-hosted-runners?apiVersion=2022-11-28#create-a-registration-token-for-an-organization>`__:

* ``organization_self_hosted_runners:write``: "Self-hosted runners" organization permissions (write).
* The user needs to be an org-level admin.

Then update the systemd service.

Enable and start the service

.. code:: shell

   systemctl --user enable container-public-linux@0.service
   systemctl --user start container-public-linux@0.service

.. attention::

   User services are terminated on logout, unless you define
   ``loginctl enable-linger <your-user>`` first.

