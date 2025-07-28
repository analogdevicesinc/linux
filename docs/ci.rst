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
provided (``mirror/<remote-name>/<branch>``):

* | ``mirror/next/linux-next/master``: Patches aimed at the next kernel merge window
  | https://git.kernel.org/pub/scm/linux/kernel/git/next/linux-next.git/log/?h=master
* | ``mirror/jic23/iio/testing``: Linux IIO Subsystem (Cameron's branch)
  | https://git.kernel.org/pub/scm/linux/kernel/git/jic23/iio.git/log/?h=testing

When upstreaming a driver, target the pull-request against the mirror.

All of them are mirrors from the links shown with a single commit on top
that includes the CI workflows, :git-linux:`as simple as <.github/workflows/sync-mirror.yml>`:

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
``err``, ``fail``, and ``warn``:

* ``err``: What a method return, if not captured (``|| true``), fails the CI.
* ``fail``: Indicates that a warning or error deemed strict was raised.
  Is collected at the assert job and ends the run with failure.
* ``warn``: Indicates that a warning or error non-deemed strict was raised.
  Is collected at the assert job and **does not** end the run with failure.

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
  - | ``ggc fanalyzer``: the changed files are recompiled appending the ``-fanalyzer`` flag.
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

.. _conf-podman:

Configure podman
~~~~~~~~~~~~~~~~

Below are suggested instructions for setting up ``podman`` on a Linux environment.

Adjust to your preference as needed, and skip the steps marked in :green:`green`
if not using WSL2.

Install ``podman`` from your package manager.

:green:`Ensure cgroup v2 on wsl2's .wslconfig:`

::

   [wsl2]
   kernelCommandLine = cgroup_no_v1=all systemd.unified_cgroup_hierarchy=1

:green:`Restart wsl2.`

Enable podman service for your user.

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

To build the container image, use your favorite container engine:

.. shell::

   $cd ~/linux
   $podman build --tag adi/linux:latest ci

You may want to build the container in a host, where you have all your tools installed,
and then deploy to a server.
In this case, export the image and then import on the server:

.. shell::
   :show-user:
   :user: user
   :group: host

   ~/linux
   $podman save -o adi-linux.tar adi/linux:latest
   $scp adi-linux.tar server:/tmp/

.. shell::
   :show-user:
   :user: admin
   :group: server

   /tmp
   $podman load -i adi-linux.tar

Or if you are feeling adventurous:

.. shell::
   :show-user:
   :user: user
   :group: host

   ~/linux
   $podman save adi/linux:latest | ssh server "cat - | podman load"

.. _interactive-run:

Interactive run
~~~~~~~~~~~~~~~

The :git-doctools:`container-run.sh <ci/scripts/container-run.sh>`
is a suggested container command to interactive login into an image, mounting
the provided path.

You can leverage it to compile/runs checks using persistent cache, for example:

.. shell::

   ~/linux
   $cr adi/linux:v1
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

Or:

.. shell::

   ~/linux
   $cr adi/linux:v1
   $base_sha=@~2
   $set_arch gcc_arm
    ARCH=arm
    CXX=gcc-14
    CROSS_COMPILE=arm-suse-linux-gnueabi-
   $check_checkpatch
    checkpatch on range @~6..@
    Collecting ply
    Downloading ply-3.11-py...

Significantly speeding up interactive testing.

.. _podman-run:

Self-hosted runner
~~~~~~~~~~~~~~~~~~

To host your `GitHub Actions Runner <https://github.com/actions/runner>`__,
set up your secrets:

.. shell::

   # e.g. analogdevicesinc/linux
   $printf ORG_REPOSITORY | podman secret create public_linux_org_repository -
   # e.g. MyVerYSecRunnerToken
   $printf RUNNER_TOKEN | podman secret create public_linux_runner_token -

The runner token is obtained from the GUI at ``github.com/<org>/<repository>/settings/actions/runners/new``.

If ``github_token`` from :ref:`cluster-podman` is set, the runner_token
is ignored and a new one is requested.

.. shell::

   ~/linux
   $podman run \
   $    --secret public_linux_org_repository,type=env,target=org_repository \
   $    --secret public_linux_runner_token,type=env,target=runner_token \
   $    --env runner_labels=v1,big_cpu \
   $    adi/linux:latest

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
             -d adi/linux:latest top
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
                --env runner_token=$(gpg --quiet --batch --decrypt /run/secrets/public_runner_token.gpg) \
                --cidfile %t/%n-cid \
                --label "io.containers.autoupdate=local" \
                --name=public_linux_%i \
                --memory-swap=20g \
                --memory=16g \
                --cpus=4 \
                --log-driver=journald \
                -d localhost/adi/linux:latest top"
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

