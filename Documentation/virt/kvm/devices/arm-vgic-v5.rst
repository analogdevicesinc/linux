.. SPDX-License-Identifier: GPL-2.0

====================================================
ARM Virtual Generic Interrupt Controller v5 (VGICv5)
====================================================


Device types supported:
  - KVM_DEV_TYPE_ARM_VGIC_V5     ARM Generic Interrupt Controller v5.0

Only one VGIC instance may be instantiated through this API.  The created VGIC
will act as the VM interrupt controller, requiring emulated user-space devices
to inject interrupts to the VGIC instead of directly to CPUs.

Creating a guest GICv5 device requires a GICv5 host.  The VGICv5 device supports
PPI, SPI, and LPI interrupts.  The PPI and SPI interrupts can either be injected
from emulated in-kernel devices (such as the Arch Timer, or PMU), or via the
KVM_IRQ_LINE ioctl.  LPIs are not externally injected, but are handled in
hardware via the LPI IST.  Their pending state is driven directly by the guest.

Groups:
  KVM_DEV_ARM_VGIC_GRP_ADDR
   Attributes:

    KVM_VGIC_V5_ADDR_TYPE_IRS (rw, 64-bit)
      Base address in the guest physical address space of the GICv5 IRS
      (Interrupt Routing Service) register mappings. Only valid for
      KVM_DEV_TYPE_ARM_VGIC_V5.  This address needs to be 64K aligned and the
      region covers 128 KByte - the IRS has a CONFIG_FRAME and a SETLPI_FRAME,
      each of which is 64 KBytes in size.

      Setting the address of the IRS in GPA space is mandatory before VGIC
      resources are mapped, as the IRS is responsible for handling SPIs and
      LPIs. Failure to set the IRS address before the first vCPU run results in
      an error.

  KVM_DEV_ARM_VGIC_GRP_NR_IRQS
   Attributes:

    A value describing the number of SPIs for this GIC instance. This is
    GICv5-specific: unlike GICv2/v3, the value does not include SGIs or PPIs.
    The value ranges from 32 to KVM's VGICv5 maximum of 1024 SPIs, in
    increments of 32. If userspace does not set this attribute, KVM uses 32
    SPIs by default.

    kvm_device_attr.addr points to a __u32 value.

  KVM_DEV_ARM_VGIC_GRP_CTRL
   Attributes:

    KVM_DEV_ARM_VGIC_CTRL_INIT
      request the initialization of the VGIC, no additional parameter in
      kvm_device_attr.addr. Must be called after all VCPUs have been created.

   KVM_DEV_ARM_VGIC_USERSPACE_PPIS
      request the mask of userspace-drivable PPIs. Only a subset of the PPIs can
      be directly driven from userspace with GICv5, and the returned mask
      informs userspace of which it is allowed to drive via KVM_IRQ_LINE.

      Userspace must allocate and point to __u64[2] of data in
      kvm_device_attr.addr. When this call returns, the provided memory will be
      populated with the userspace PPI mask. The lower __u64 contains the mask
      for the lower 64 PPIS, with the remaining 64 being in the second __u64.

      This is a read-only attribute, and cannot be set. Attempts to set it are
      rejected.

  Errors:

    =======  ========================================================
    -ENXIO   VGIC not properly configured as required prior to calling
             this attribute
    -ENODEV  no online VCPU
    -ENOMEM  memory shortage when allocating vgic internal data
    -EFAULT  Invalid guest ram access
    -EBUSY   One or more VCPUS are running
    =======  ========================================================

  KVM_DEV_ARM_VGIC_GRP_CPU_SYSREGS
   Attributes:

    The attr field of kvm_device_attr encodes two values::

      bits:     | 63      ....       32 | 31  ....  16 | 15  ....  0 |
      values:   |         mpidr         |      RES     |    instr    |

    The mpidr field encodes the CPU ID based on the affinity information in the
    architecture defined MPIDR, and the field is encoded as follows::

      | 63 .... 56 | 55 .... 48 | 47 .... 40 | 39 .... 32 |
      |    Aff3    |    Aff2    |    Aff1    |    Aff0    |

    The instr field encodes the system register to access based on the fields
    defined in the A64 instruction set encoding for system register access
    (RES means the bits are reserved for future use and should be zero)::

      | 15 ... 14 | 13 ... 11 | 10 ... 7 | 6 ... 3 | 2 ... 0 |
      |   Op 0    |    Op1    |    CRn   |   CRm   |   Op2   |

    All system regs accessed through this API are (rw, 64-bit) and
    kvm_device_attr.addr points to a __u64 value.

    KVM_DEV_ARM_VGIC_GRP_CPU_SYSREGS accesses the CPU interface registers for the
    CPU specified by the mpidr field.

    The available registers are:

    =======================  ===================================================
    ICC_ICSR_EL1
    ICC_PPI_ENABLER0_EL1
    ICC_PPI_ENABLER1_EL1
    ICC_PPI_SACTIVER0_EL1    ICC_PPI_CACTIVER0_EL1 is not supported. Writes to
                             ICC_PPI_SACTIVER0_EL1 are treated as RAW writes of
                             the underlying state.
    ICC_PPI_SACTIVER1_EL1    ICC_PPI_CACTIVER1_EL1 is not supported. Writes to
                             ICC_PPI_SACTIVER1_EL1 are treated as RAW writes of
                             the underlying state.
    ICC_PPI_SPENDR0_EL1      ICC_PPI_CPENDR0_EL1 is not supported. Writes to
                             ICC_PPI_SPENDR0_EL1 are treated as RAW writes of
                             the underlying state.
    ICC_PPI_SPENDR1_EL1      ICC_PPI_CPENDR1_EL1 is not supported. Writes to
                             ICC_PPI_SPENDR1_EL1 are treated as RAW writes of
                             the underlying state.
    ICC_PPI_PRIORITYR0_EL1
    ICC_PPI_PRIORITYR1_EL1
    ICC_PPI_PRIORITYR2_EL1
    ICC_PPI_PRIORITYR3_EL1
    ICC_PPI_PRIORITYR4_EL1
    ICC_PPI_PRIORITYR5_EL1
    ICC_PPI_PRIORITYR6_EL1
    ICC_PPI_PRIORITYR7_EL1
    ICC_PPI_PRIORITYR8_EL1
    ICC_PPI_PRIORITYR9_EL1
    ICC_PPI_PRIORITYR10_EL1
    ICC_PPI_PRIORITYR11_EL1
    ICC_PPI_PRIORITYR12_EL1
    ICC_PPI_PRIORITYR13_EL1
    ICC_PPI_PRIORITYR14_EL1
    ICC_PPI_PRIORITYR15_EL1
    ICC_APR_EL1
    ICC_CR0_EL1
    ICC_PCR_EL1
    =======================  ===================================================

  Errors:

    =======  =============================================================
    -ENXIO   Getting or setting this register is not supported
    -EBUSY   VCPU is running, or write attempted after a VCPU has run
    -EINVAL  Invalid mpidr or register value supplied
    =======  =============================================================
