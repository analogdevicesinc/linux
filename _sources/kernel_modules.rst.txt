.. _kernel_modules:

Kernel modules
==============

This section explains how to compile out-of-tree kernel modules with the
kernel artifacts on our `adi/linux (S3) <https://broadcasts.cloudsmith.com/adi/linux>`__.

For each defconfig, three artifacts are provided:

- ``<no-suffix>``: kernel image and modules.
- ``-headers``: ``make headers_install``, userspace headers in ``usr/include``.
- ``-devel``: ``install-extmod-build``, copies ``include/``,
  ``arch/<arch>/include``, the Makefiles and ``scripts/``.

The download link uses the format
``https://dl.cloudsmith.io/public/adi/linux/raw/versions/${sha}/${defconfig}``,
for example, to get a devel artifact:

.. shell::

   $ defconfig=adi_versal_defconfig-gcc-arm64
   $ sha=5ccfc9fe12ed9bc7158da7390ad2b917fe5c83a8
   $ curl -LO \
          https://dl.cloudsmith.io/public/adi/linux/raw/versions/${sha}/${defconfig}-devel
   $ unzip -q ${defconfig}-devel -d devel

Use the ``sha`` of the running kernel, identify with (drop the ``-g``):

.. shell::

   $ uname -r
     6.12.117-g5ccfc9fe12ed

And expand it to the full ``sha`` with:

.. shell::

   $ sha=5ccfc9fe12ed
   # GitHub REST API
   $ curl -sL -H "Accept: application/vnd.github.sha" \
          https://api.github.com/repos/analogdevicesinc/linux/commits/${sha}
     5ccfc9fe12ed9bc7158da7390ad2b917fe5c83a8
   # Cloudsmith REST API
   $ curl -sL "https://api.cloudsmith.io/v1/packages/adi/linux/?query=version:${sha}&page_size=1" \
          | jq -r '.[].version'
     5ccfc9fe12ed9bc7158da7390ad2b917fe5c83a8

The ``*-devel`` artifact is compiled native to the target architecture,
including the host tools in ``scripts/``. To **cross-compile** from another
host (like x86_64), rebuild them in the extracted ``-devel`` directory, for example:

.. shell::

   $ cd devel
   $ gcc -O2 -Iscripts/include \
         -o scripts/basic/fixdep scripts/basic/fixdep.c
   # If CONFIG_MODVERSIONS=y
   $ gcc -O2 -Iscripts/include -Iscripts/genksyms \
         -o scripts/genksyms/genksyms \
         scripts/genksyms/{genksyms.c,lex.lex.c,parse.tab.c}
   $ gcc -O2 -Iscripts/mod -Iscripts/include \
         -o scripts/mod/modpost \
         scripts/mod/{modpost.c,file2alias.c,sumversion.c,symsearch.c}
   $ cd ..

.. note::

   You may also need to re-build the host tools if your target distro
   is behind, for example, compiling from Debian Bookworm raises ``version
   'GLIBC_2.38' not found``; rebuilding the host tools fixes.

With ``CONFIG_MODVERSIONS=y``, every symbol has its CRC computed. When
loading a module, the kernel uses it instead of the kernel release string,
allowing to load a module built for a different kernel, as long as all CRCs
match. More info at :external+upstream:doc:`kbuild/modules`.

Simple driver
-------------

The simple IIO ADC driver ``ad4062`` is compiled with:

.. shell::

   ~
   $ mkdir ad4062 ; cd ad4062
   # Get exact driver match
   $ curl -LO https://raw.githubusercontent.com/analogdevicesinc/linux/${sha}/drivers/iio/adc/ad4062.c
   # Compile
   $ echo 'obj-m := ad4062.o' > Makefile
   $ make -C ../devel \
          ARCH=arm64 CROSS_COMPILE=aarch64-linux- M=$PWD modules
   $ ls *.ko
     ad4062.ko

.. caution::

   ``KBUILD_OUTPUT`` overrides ``-C``.

The ``-devel`` also stores the ``sha`` in the *context.txt* file.

API-based driver
----------------

For a more complex driver that has its own private API, prefer a sparse
clone:

.. shell::

   ~
   $ git init ad9088 ; cd ad9088
   $ git remote add origin https://github.com/analogdevicesinc/linux.git
   # Needs cf_axi_adc.h and adi-axi-hsci.h
   $ git sparse-checkout set \
       drivers/iio/trx-rf/ad9088 \
       drivers/iio/adc/cf_axi_adc.h \
       drivers/misc/adi-axi-hsci.h
   $ git fetch --depth=1 --filter=blob:none origin $sha
   $ git checkout FETCH_HEAD
   $ make -C ../devel \
          ARCH=arm64 CROSS_COMPILE=aarch64-linux- CONFIG_AD9088=m \
          M=$PWD/drivers/iio/trx-rf/ad9088 modules
   $ ls drivers/iio/trx-rf/ad9088/*.ko
     drivers/iio/trx-rf/ad9088/ad9088_drv.ko

*Module.symvers* in the ``-devel`` lists the symbols exported by the kernel
build. So if you add, for example, ``example_method()`` to *drivers/misc/example.c*
with ``EXPORT_SYMBOL_GPL()``, and call it from *ad9088.c*, the build fails
with::

   ERROR: modpost: "example_method" [.../drivers/iio/trx-rf/ad9088/ad9088_drv.ko] undefined!

In this case, build *example.ko* first and pass its *Module.symvers*:

.. shell::

   ~/ad9088
   $ git sparse-checkout add drivers/misc
   $ make -C ../devel \
          ARCH=arm64 CROSS_COMPILE=aarch64-linux- CONFIG_EXAMPLE=m \
          M=$PWD/drivers/misc example.ko
   $ make -C ../devel \
          ARCH=arm64 CROSS_COMPILE=aarch64-linux- CONFIG_AD9088=m \
          KBUILD_EXTRA_SYMBOLS=$PWD/drivers/misc/Module.symvers \
          M=$PWD/drivers/iio/trx-rf/ad9088 modules

Install both *example.ko* and *ad9088_drv.ko*, otherwise loading fails with
``Unknown symbol example_method``.

If the series changes the arguments of an existing exported function, the
build does not fail: *ad9088* keeps the CRC from the ``-devel`` and loads
against the old module with the wrong arguments. So, always build both.

Installing & loading modules
----------------------------

Modules are installed per kernel release, at ``/lib/modules/$(uname -r)/``:

- *kernel/*: modules shipped with the linux distro, may be managed by a package.
- *updates/*: default user modules path.

*updates/* is set by ``INSTALL_MOD_DIR`` in *scripts/Makefile.modinst*, and has
precedence over *kernel/* during ``depmod``.

Copy the module to the target and run ``depmod``:

.. shell::
   :no-path:

   $ mkdir -p /lib/modules/$(uname -r)/updates
   $ cp ad4062.ko /lib/modules/$(uname -r)/updates/
   $ depmod -a
   $ modinfo -n ad4062
     /lib/modules/6.12.0+/updates/ad4062.ko
   $ modprobe ad4062

A new kernel release uses a new */lib/modules/<release>/* directory, so modules
in *updates/* have to be moved/reinstalled (and rebuilt, if the CRC changes).
To use a path that does not change with the release, create a
``/etc/depmod.d/*.conf`` configuration, for example, for a static path
*/usr/local/lib/modules/premium*::

   external * /usr/local/lib/modules/premium
   search external updates built-in

.. shell::
   :no-path:

   $ depmod -a
   $ modinfo -n ad4062
     /usr/local/lib/modules/premium/ad4062.ko

More info at `depmod.d(5) <https://man7.org/linux/man-pages/man5/depmod.d.5.html>`__.

Debugging ``Exec format error``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you encounter::

  modprobe: ERROR: could not insert 'my_driver': Exec format error

Causes are:

- ``CONFIG_MODVERSIONS`` differs between the kernel and the module
  (``modversion`` in *vermagic*).
- Matched ``CONFIG_MODVERSIONS`` state:

  - ``yes``: CRC mismatch, the kernel module is not compatible.
  - ``no``: Mismatch kernel release string.

The error for a CRC mismatch is::

  modprobe: ERROR: could not insert 'ad9088_drv': Invalid argument

with ``disagrees about version of symbol <symbol>`` in ``dmesg``.
