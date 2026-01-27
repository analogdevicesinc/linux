ADI Linux version support
=========================

Mainline Linux versions
-----------------------

There are three types of Linux kernel releases:

- Major releases (e.g. 6.0, 6.1, 6.2)
- Long-term support (LTS) releases (e.g. 6.1, 6.6, 6.12)
- Stable releases (e.g. 6.12.0, 6.12.1, ..., 6.12.66)

Additionally, the Civil Infrastructure Platform (CIP) continues to maintain LTS
releases once mainline support ends. [#cip]_ This is done to meet industrial grade
requirements.

The latest releases are listed on kernel.org_. The `Linux kernel version
history`_ article on Wikipedia provides helpful diagrams for understanding the
various Linux kernel releases. For example, the following diagram shows all
6.x.y releases:

.. image:: https://upload.wikimedia.org/wikipedia/en/timeline/l6dviz7g7u8sv3g7eoysel7pp7vcxoq.png

.. _kernel.org: https://www.kernel.org/
.. _Linux kernel version history: https://en.wikipedia.org/wiki/Linux_kernel_version_history

ADI support
-----------

All ADI changes are intended to be eventually applied upstream in the mainline
Linux kernel. That process can take some time. Therefore the changes are
maintained in the ADI tree on Github.

ADI follows a similar approach to that taken by mainline. It attempts to
maintain ADI changes on top of the latest LTS release and update those changes
with every new LTS release. That process makes it easier to upstream those
changes into the mainline kernel.

ADI can also apply those changes to older LTS releases by backporting changes,
but that is not done automatically given the burden of supporting multiple LTS
versions.

CVE
---

ADI addresses upstream CVEs by applying our changes onto newer stable releases,
which include fixes. Only CVEs targeting ADI changes will be applied in the ADI
tree.

Mainline
^^^^^^^^

On February 13, 2024 kernel.org was added as a CVE Numbering Authority, giving
the Linux development community more control over how CVEs are issued.
[#cna-announcement]_ Greg K-H provided context to that announcement in his blog
post, `Linux is a CNA`_, and it was covered by LWN in `A turning point for CVE
numbers`_.

.. pull-quote::

    Nobody who relies on backporting fixes to a non-mainline kernel will be able
    to keep up with this CVE stream. Any company that is using CVE numbers to
    select kernel patches is going to have to rethink its processes.

    ... distributors will simply fall back on shipping the stable kernel updates
    which, almost by definition, will contain fixes for every known CVE number.

.. _Linux is a CNA: http://www.kroah.com/log/blog/2024/02/13/linux-is-a-cna/
.. _A turning point for CVE numbers: https://lwn.net/Articles/961978/

CVEs for the Linux kernel are announced on the `linux-cve-announce mailing
list`_ and stored in the `security/vulns git repository`_, along with a set of
scripts to parse the data.

.. _linux-cve-announce mailing list: https://lore.kernel.org/linux-cve-announce/
.. _security/vulns git repository: https://git.kernel.org/pub/scm/linux/security/vulns.git/

----

.. [#cip] https://wiki.linuxfoundation.org/civilinfrastructureplatform/cipkernelmaintenance
.. [#cna-announcement] https://www.cve.org/Media/News/item/news/2024/02/13/kernel-org-Added-as-CNA
