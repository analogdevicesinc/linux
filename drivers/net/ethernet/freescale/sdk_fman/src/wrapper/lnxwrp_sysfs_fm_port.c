/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lnxwrp_sysfs.h"
#include "lnxwrp_fm.h"
#include "debug_ext.h"
#include "lnxwrp_sysfs_fm_port.h"
#include "lnxwrp_sysfs_fm.h"

#include "../../sdk_fman/Peripherals/FM/Port/fm_port.h"
#include "../../sdk_fman/Peripherals/FM/Port/fm_port_dsar.h"

#if defined(__ERR_MODULE__)
#undef __ERR_MODULE__
#endif

#include "../../sdk_fman/Peripherals/FM/fm.h"

static const struct sysfs_stats_t portSysfsStats[] = {
	/* RX/TX/OH common statistics */
	{
	 .stat_name = "port_frame",
	 .stat_counter = e_FM_PORT_COUNTERS_FRAME,
	 },
	{
	 .stat_name = "port_discard_frame",
	 .stat_counter = e_FM_PORT_COUNTERS_DISCARD_FRAME,
	 },
	{
	 .stat_name = "port_dealloc_buf",
	 .stat_counter = e_FM_PORT_COUNTERS_DEALLOC_BUF,
	 },
	{
	 .stat_name = "port_enq_total",
	 .stat_counter = e_FM_PORT_COUNTERS_ENQ_TOTAL,
	 },
	/* TX/OH */
	{
	 .stat_name = "port_length_err",
	 .stat_counter = e_FM_PORT_COUNTERS_LENGTH_ERR,
	 },
	{
	 .stat_name = "port_unsupprted_format",
	 .stat_counter = e_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT,
	 },
	{
	 .stat_name = "port_deq_total",
	 .stat_counter = e_FM_PORT_COUNTERS_DEQ_TOTAL,
	 },
	{
	 .stat_name = "port_deq_from_default",
	 .stat_counter = e_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT,
	 },
	{
	 .stat_name = "port_deq_confirm",
	 .stat_counter = e_FM_PORT_COUNTERS_DEQ_CONFIRM,
	 },
	/* RX/OH */
	{
	 .stat_name = "port_rx_bad_frame",
	 .stat_counter = e_FM_PORT_COUNTERS_RX_BAD_FRAME,
	 },
	{
	 .stat_name = "port_rx_large_frame",
	 .stat_counter = e_FM_PORT_COUNTERS_RX_LARGE_FRAME,
	 },
	{
	 .stat_name = "port_rx_out_of_buffers_discard",
	 .stat_counter = e_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD,
	 },
	{
	 .stat_name = "port_rx_filter_frame",
	 .stat_counter = e_FM_PORT_COUNTERS_RX_FILTER_FRAME,
	 },
	/* TODO: Particular statistics for OH ports */
	{}
};

static ssize_t show_fm_port_stats(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
	t_LnxWrpFmDev *p_LnxWrpFmDev;
	unsigned long flags;
	int n = 0;
	uint8_t counter = 0;

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

	p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_LnxWrpFmPortDev == NULL))
		return -EINVAL;

	p_LnxWrpFmDev = (t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev;
	if (WARN_ON(p_LnxWrpFmDev == NULL))
		return -EINVAL;

	if (!p_LnxWrpFmDev->active || !p_LnxWrpFmDev->h_Dev)
		return -EIO;

	if (!p_LnxWrpFmPortDev->h_Dev) {
		n = snprintf(buf, PAGE_SIZE, "\tFM Port not configured...\n");
		return n;
	}

	counter = fm_find_statistic_counter_by_name(
			attr->attr.name,
			portSysfsStats, NULL);

	if (counter == e_FM_PORT_COUNTERS_RX_LIST_DMA_ERR) {
		uint32_t fmRev = 0;
		fmRev = 0xffff &
			ioread32(UINT_TO_PTR(p_LnxWrpFmDev->fmBaseAddr +
			0x000c30c4));

		if (fmRev == 0x0100) {
			local_irq_save(flags);
			n = snprintf(buf, PAGE_SIZE,
				"counter not available for revision 1\n");
			local_irq_restore(flags);
		}
		return n;
	}

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE, "\t%s counter: %u\n",
		p_LnxWrpFmPortDev->name,
		FM_PORT_GetCounter(p_LnxWrpFmPortDev->h_Dev,
					(e_FmPortCounters) counter));
	local_irq_restore(flags);

	return n;
}

/* FM PORT RX/TX/OH statistics */
static DEVICE_ATTR(port_frame, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_discard_frame, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_dealloc_buf, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_enq_total, S_IRUGO, show_fm_port_stats, NULL);
/* FM PORT TX/OH statistics */
static DEVICE_ATTR(port_length_err, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_unsupprted_format, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_deq_total, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_deq_from_default, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_deq_confirm, S_IRUGO, show_fm_port_stats, NULL);
/* FM PORT RX/OH statistics */
static DEVICE_ATTR(port_rx_bad_frame, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_rx_large_frame, S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_rx_out_of_buffers_discard, S_IRUGO,
		show_fm_port_stats, NULL);
static DEVICE_ATTR(port_rx_filter_frame, S_IRUGO, show_fm_port_stats, NULL);

/* FM PORT TX statistics */
static struct attribute *fm_tx_port_dev_stats_attributes[] = {
	&dev_attr_port_frame.attr,
	&dev_attr_port_discard_frame.attr,
	&dev_attr_port_dealloc_buf.attr,
	&dev_attr_port_enq_total.attr,
	&dev_attr_port_length_err.attr,
	&dev_attr_port_unsupprted_format.attr,
	&dev_attr_port_deq_total.attr,
	&dev_attr_port_deq_from_default.attr,
	&dev_attr_port_deq_confirm.attr,
	NULL
};

static const struct attribute_group fm_tx_port_dev_stats_attr_grp = {
	.name = "statistics",
	.attrs = fm_tx_port_dev_stats_attributes
};

/* FM PORT RX statistics */
static struct attribute *fm_rx_port_dev_stats_attributes[] = {
	&dev_attr_port_frame.attr,
	&dev_attr_port_discard_frame.attr,
	&dev_attr_port_dealloc_buf.attr,
	&dev_attr_port_enq_total.attr,
	&dev_attr_port_rx_bad_frame.attr,
	&dev_attr_port_rx_large_frame.attr,
	&dev_attr_port_rx_out_of_buffers_discard.attr,
	&dev_attr_port_rx_filter_frame.attr,
	NULL
};

static const struct attribute_group fm_rx_port_dev_stats_attr_grp = {
	.name = "statistics",
	.attrs = fm_rx_port_dev_stats_attributes
};

/* TODO: add particular OH ports statistics */
static struct attribute *fm_oh_port_dev_stats_attributes[] = {
	&dev_attr_port_frame.attr,
	&dev_attr_port_discard_frame.attr,
	&dev_attr_port_dealloc_buf.attr,
	&dev_attr_port_enq_total.attr,
	/*TX*/ &dev_attr_port_length_err.attr,
	&dev_attr_port_unsupprted_format.attr,
	&dev_attr_port_deq_total.attr,
	&dev_attr_port_deq_from_default.attr,
	&dev_attr_port_deq_confirm.attr,
	/* &dev_attr_port_rx_bad_frame.attr, */
	/* &dev_attr_port_rx_large_frame.attr, */
	&dev_attr_port_rx_out_of_buffers_discard.attr,
	/*&dev_attr_port_rx_filter_frame.attr, */
	NULL
};

static const struct attribute_group fm_oh_port_dev_stats_attr_grp = {
	.name = "statistics",
	.attrs = fm_oh_port_dev_stats_attributes
};

static ssize_t show_fm_port_regs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
#endif
	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_LnxWrpFmPortDev =
		(t_LnxWrpFmPortDev *) dev_get_drvdata(dev);


	local_irq_save(flags);

	if (!p_LnxWrpFmPortDev->h_Dev) {
		n = snprintf(buf, PAGE_SIZE, "\tFM Port not configured...\n");
		return n;
	} else {
		n = snprintf(buf, PAGE_SIZE,
				"FM port driver registers dump.\n");
		n = fm_port_dump_regs(p_LnxWrpFmPortDev->h_Dev, buf, n);
	}

	local_irq_restore(flags);

	return n;
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

	return n;
#endif
}
static int fm_port_dsar_dump_mem(void *h_dev, char *buf, int nn)
{
	t_FmPort *p_FmPort;
	t_Fm *p_Fm;
	uint8_t hardwarePortId;
	uint32_t *param_page;
	t_ArCommonDesc *ArCommonDescPtr;
	uint32_t *mem;
	int i, n = nn;

	p_FmPort = (t_FmPort *)h_dev;
	hardwarePortId = p_FmPort->hardwarePortId;
	p_Fm = (t_Fm *)p_FmPort->h_Fm;

	if (!FM_PORT_IsInDsar(p_FmPort))
	{
		FM_DMP_LN(buf, n, "port %u is not a DSAR port\n",
			hardwarePortId);
		return n;
	}
	FM_DMP_LN(buf, n, "port %u DSAR mem\n", hardwarePortId);
	FM_DMP_LN(buf, n, "========================\n");

	/* do I need request_mem_region here? */
	param_page = ioremap(p_FmPort->fmMuramPhysBaseAddr + ioread32be(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rgpr), 4);
	ArCommonDescPtr = (t_ArCommonDesc*)(ioremap(p_FmPort->fmMuramPhysBaseAddr + ioread32be(param_page), 300*4)); /* this should be changed*/
	mem = (uint32_t*)ArCommonDescPtr;
	for (i = 0; i < 300; i+=4)
		FM_DMP_LN(buf, n, "%08x: %08x %08x %08x %08x\n", i*4, mem[i], mem[i + 1], mem[i + 2], mem[i + 3]);
	iounmap(ArCommonDescPtr);
	iounmap(param_page);
	return n;
}

static int fm_port_dsar_dump_regs(void *h_dev, char *buf, int nn)
{
	t_FmPort *p_FmPort;
	t_Fm *p_Fm;
	uint8_t hardwarePortId;
	uint32_t *param_page;
	t_ArCommonDesc *ArCommonDescPtr;
	int i, n = nn;

	p_FmPort = (t_FmPort *)h_dev;
	hardwarePortId = p_FmPort->hardwarePortId;
	p_Fm = (t_Fm *)p_FmPort->h_Fm;

	if (!FM_PORT_IsInDsar(p_FmPort))
	{
		FM_DMP_LN(buf, n, "port %u is not a DSAR port\n",
			hardwarePortId);
		return n;
	}
	FM_DMP_LN(buf, n, "port %u DSAR information\n", hardwarePortId);
	FM_DMP_LN(buf, n, "========================\n");

	/* do I need request_mem_region here? */
	param_page = ioremap(p_FmPort->fmMuramPhysBaseAddr + ioread32be(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rgpr), 4);
	ArCommonDescPtr = (t_ArCommonDesc*)(ioremap(p_FmPort->fmMuramPhysBaseAddr + ioread32be(param_page), sizeof(t_ArCommonDesc))); /* this should be changed*/
	FM_DMP_LN(buf, n, "Tx port: 0x%x\n", ArCommonDescPtr->arTxPort);
	FM_DMP_LN(buf, n, "Active HPNIA: 0x%08x\n", ArCommonDescPtr->activeHPNIA);
	FM_DMP_LN(buf, n, "Snmp port: 0x%x\n", ArCommonDescPtr->snmpPort);
	FM_DMP_LN(buf, n, "MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", ArCommonDescPtr->macStationAddr[0],
		ArCommonDescPtr->macStationAddr[1], ArCommonDescPtr->macStationAddr[2],
		ArCommonDescPtr->macStationAddr[3], ArCommonDescPtr->macStationAddr[4],
		ArCommonDescPtr->macStationAddr[5]);
	FM_DMP_LN(buf, n, "filterControl: 0x%02x\n", ArCommonDescPtr->filterControl);
	FM_DMP_LN(buf, n, "tcpControlPass: 0x%04x\n", ArCommonDescPtr->tcpControlPass);
	FM_DMP_LN(buf, n, "ipProtocolTblSize: 0x%x\n", ArCommonDescPtr->ipProtocolTblSize);
	FM_DMP_LN(buf, n, "udpPortTblSize: 0x%x\n", ArCommonDescPtr->udpPortTblSize);
	FM_DMP_LN(buf, n, "tcpPortTblSize: 0x%x\n", ArCommonDescPtr->tcpPortTblSize);
	if (ArCommonDescPtr->p_ArStats)
	{
		t_ArStatistics *arStatistics = (t_ArStatistics*)
			ioremap(ioread32be(&ArCommonDescPtr->p_ArStats) +
			p_FmPort->fmMuramPhysBaseAddr,
			sizeof (t_ArStatistics));
		FM_DMP_LN(buf, n, "\nDSAR statistics\n");
		FM_DMP_LN(buf, n, "DSAR_Discarded:            0x%x\n", arStatistics->dsarDiscarded);
		FM_DMP_LN(buf, n, "DSAR_Err_Discarded:        0x%x\n", arStatistics->dsarErrDiscarded);
		FM_DMP_LN(buf, n, "DSAR_Frag_Discarded:       0x%x\n", arStatistics->dsarFragDiscarded);
		FM_DMP_LN(buf, n, "DSAR_Tunnel_Discarded:     0x%x\n", arStatistics->dsarTunnelDiscarded);
		FM_DMP_LN(buf, n, "DSAR_ARP_Discarded:        0x%x\n", arStatistics->dsarArpDiscarded);
		FM_DMP_LN(buf, n, "DSAR_IP_Discarded:         0x%x\n", arStatistics->dsarIpDiscarded);
		FM_DMP_LN(buf, n, "DSAR_TCP_Discarded:        0x%x\n", arStatistics->dsarTcpDiscarded);
		FM_DMP_LN(buf, n, "DSAR_UDP_Discarded:        0x%x\n", arStatistics->dsarUdpDiscarded);
		FM_DMP_LN(buf, n, "DSAR_ICMPv6_Checksum_Err:  0x%x\n", arStatistics->dsarIcmpV6ChecksumErr);
		FM_DMP_LN(buf, n, "DSAR_ICMPv6_Other_Type:    0x%x\n", arStatistics->dsarIcmpV6OtherType);
		FM_DMP_LN(buf, n, "DSAR_ICMPv4_Other_Type:    0x%x\n", arStatistics->dsarIcmpV4OtherType);
		
		iounmap(arStatistics);
	}
	if (ArCommonDescPtr->p_ArpDescriptor)
	{
		t_DsarArpDescriptor* ArpDescriptor = (t_DsarArpDescriptor*)
			ioremap(ioread32be(&ArCommonDescPtr->p_ArpDescriptor) +
			p_FmPort->fmMuramPhysBaseAddr,
			sizeof (t_DsarArpDescriptor));
		FM_DMP_LN(buf, n, "\nARP\n");
		FM_DMP_LN(buf, n, "===\n");
		FM_DMP_LN(buf, n, "control bits 0x%04x\n", ArpDescriptor->control);
		if (ArpDescriptor->numOfBindings)
		{
			char ip_str[100];
			t_DsarArpBindingEntry* bindings = ioremap(
				ioread32be(&ArpDescriptor->p_Bindings) +
				p_FmPort->fmMuramPhysBaseAddr,
				ArpDescriptor->numOfBindings *
				sizeof(t_DsarArpBindingEntry));
			uint8_t* ip_addr = (uint8_t*)&bindings->ipv4Addr;
			FM_DMP_LN(buf, n, "      ip          vlan id\n");
			for (i = 0; i < ArpDescriptor->numOfBindings; i++)
			{
				n += snprintf(ip_str, 100, "%d.%d.%d.%d",
					ip_addr[0], ip_addr[1],
					ip_addr[2], ip_addr[3]);
				FM_DMP_LN(buf, n, "%-15s     0x%x\n",
					ip_str, bindings->vlanId);
			}
			iounmap(bindings);
		}
		if (ArpDescriptor->p_Statistics)
		{
			t_DsarArpStatistics* arpStats = ioremap(
				ioread32be(&ArpDescriptor->p_Statistics) +
				p_FmPort->fmMuramPhysBaseAddr,
				sizeof(t_DsarArpStatistics));
			FM_DMP_LN(buf, n, "statistics\n");
			FM_DMP_LN(buf, n, "INVAL_CNT:  0x%x\n", arpStats->invalCnt);
			FM_DMP_LN(buf, n, "ECHO_CNT:   0x%x\n", arpStats->echoCnt);
			FM_DMP_LN(buf, n, "CD_CNT:     0x%x\n", arpStats->cdCnt);
			FM_DMP_LN(buf, n, "AR_CNT:     0x%x\n", arpStats->arCnt);
			FM_DMP_LN(buf, n, "RATM_CNT:   0x%x\n", arpStats->ratmCnt);
			FM_DMP_LN(buf, n, "UKOP_CNT:   0x%x\n", arpStats->ukopCnt);
			FM_DMP_LN(buf, n, "NMTP_CNT:   0x%x\n", arpStats->nmtpCnt);
			FM_DMP_LN(buf, n, "NMVLAN_CNT: 0x%x\n", arpStats->nmVlanCnt);
			iounmap(arpStats);
		}
		
		iounmap(ArpDescriptor);
	}
	if (ArCommonDescPtr->p_IcmpV4Descriptor)
	{
		t_DsarIcmpV4Descriptor* ICMPV4Descriptor =
			(t_DsarIcmpV4Descriptor*)ioremap(ioread32be(
			&ArCommonDescPtr->p_IcmpV4Descriptor) +
			p_FmPort->fmMuramPhysBaseAddr,
			sizeof (t_DsarIcmpV4Descriptor));
		FM_DMP_LN(buf, n, "\nEcho ICMPv4\n");
		FM_DMP_LN(buf, n, "===========\n");
		FM_DMP_LN(buf, n, "control bits 0x%04x\n", ICMPV4Descriptor->control);
		if (ICMPV4Descriptor->numOfBindings)
		{
			char ip_str[100];
			t_DsarArpBindingEntry* bindings = ioremap(
				ioread32be(&ICMPV4Descriptor->p_Bindings) +
				p_FmPort->fmMuramPhysBaseAddr,
				ICMPV4Descriptor->numOfBindings *
				sizeof(t_DsarArpBindingEntry));
			uint8_t* ip_addr = (uint8_t*)&bindings->ipv4Addr;
			FM_DMP_LN(buf, n, "      ip          vlan id\n");
			for (i = 0; i < ICMPV4Descriptor->numOfBindings; i++)
			{
				n += snprintf(ip_str, 100, "%d.%d.%d.%d",
					ip_addr[0], ip_addr[1],
					ip_addr[2], ip_addr[3]);
				FM_DMP_LN(buf, n, "%-15s     0x%x\n",
					ip_str, bindings->vlanId);
			}
			iounmap(bindings);
		}
		if (ICMPV4Descriptor->p_Statistics)
		{
			t_DsarIcmpV4Statistics* icmpv4Stats = ioremap(
				ioread32be(&ICMPV4Descriptor->p_Statistics) +
				p_FmPort->fmMuramPhysBaseAddr,
				sizeof(t_DsarIcmpV4Statistics));
			FM_DMP_LN(buf, n, "statistics\n");
			FM_DMP_LN(buf, n, "INVAL_CNT:  0x%x\n", icmpv4Stats->invalCnt);
			FM_DMP_LN(buf, n, "NMVLAN_CNT: 0x%x\n", icmpv4Stats->nmVlanCnt);
			FM_DMP_LN(buf, n, "NMIP_CNT:   0x%x\n", icmpv4Stats->nmIpCnt);
			FM_DMP_LN(buf, n, "AR_CNT:     0x%x\n", icmpv4Stats->arCnt);
			FM_DMP_LN(buf, n, "CSERR_CNT:  0x%x\n", icmpv4Stats->cserrCnt);
			iounmap(icmpv4Stats);
		}
		iounmap(ICMPV4Descriptor);
	}
	if (ArCommonDescPtr->p_NdDescriptor)
	{
		t_DsarNdDescriptor *NDDescriptor =
			(t_DsarNdDescriptor*)ioremap(ioread32be(
			&ArCommonDescPtr->p_NdDescriptor) + p_FmPort->
			fmMuramPhysBaseAddr, sizeof (t_DsarNdDescriptor));
		FM_DMP_LN(buf, n, "\nNDP\n");
		FM_DMP_LN(buf, n, "===\n");
		FM_DMP_LN(buf, n, "control bits 0x%04x\n", NDDescriptor->control);
		FM_DMP_LN(buf, n, "solicited address 0x%08x\n", NDDescriptor->solicitedAddr);
		if (NDDescriptor->numOfBindings)
		{
			char ip_str[100];
			t_DsarIcmpV6BindingEntry* bindings = ioremap(
				ioread32be(&NDDescriptor->p_Bindings) +
				p_FmPort->fmMuramPhysBaseAddr,
				NDDescriptor->numOfBindings *
				sizeof(t_DsarIcmpV6BindingEntry));
			uint16_t* ip_addr = (uint16_t*)&bindings->ipv6Addr;
			FM_DMP_LN(buf, n, "                  ip                        vlan id\n");
			for (i = 0; i < NDDescriptor->numOfBindings; i++)
			{
				n += snprintf(ip_str, 100,
					"%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x",
				ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3],
				ip_addr[4], ip_addr[5], ip_addr[6], ip_addr[7]);
				FM_DMP_LN(buf, n, "%s     0x%x\n", ip_str, bindings->vlanId);
			}
			iounmap(bindings);
		}
		if (NDDescriptor->p_Statistics)
		{
			t_NdStatistics* ndStats = ioremap(
				ioread32be(&NDDescriptor->p_Statistics) +
				p_FmPort->fmMuramPhysBaseAddr,
				sizeof(t_NdStatistics));
			FM_DMP_LN(buf, n, "statistics\n");
			FM_DMP_LN(buf, n, "INVAL_CNT:    0x%x\n", ndStats->invalCnt);
			FM_DMP_LN(buf, n, "NMVLAN_CNT:   0x%x\n", ndStats->nmVlanCnt);
			FM_DMP_LN(buf, n, "NMIP_CNT:     0x%x\n", ndStats->nmIpCnt);
			FM_DMP_LN(buf, n, "AR_CNT:       0x%x\n", ndStats->arCnt);
			FM_DMP_LN(buf, n, "USADVERT_CNT: 0x%x\n", ndStats->usadvertCnt);
			FM_DMP_LN(buf, n, "NMMCAST_CNT:  0x%x\n", ndStats->nmmcastCnt);
			FM_DMP_LN(buf, n, "NSLLA_CNT:    0x%x\n", ndStats->nsllaCnt);
			iounmap(ndStats);
		}
		iounmap(NDDescriptor);
	}
	if (ArCommonDescPtr->p_IcmpV6Descriptor)
	{
		t_DsarIcmpV6Descriptor *ICMPV6Descriptor =
			(t_DsarIcmpV6Descriptor*)ioremap(ioread32be(
			&ArCommonDescPtr->p_IcmpV6Descriptor) + p_FmPort->
			fmMuramPhysBaseAddr, sizeof (t_DsarIcmpV6Descriptor));
		FM_DMP_LN(buf, n, "\nEcho ICMPv6\n");
		FM_DMP_LN(buf, n, "===========\n");
		FM_DMP_LN(buf, n, "control bits 0x%04x\n", ICMPV6Descriptor->control);
		if (ICMPV6Descriptor->numOfBindings)
		{
			char ip_str[100];
			t_DsarIcmpV6BindingEntry* bindings = ioremap(
				ioread32be(&ICMPV6Descriptor->p_Bindings) +
				p_FmPort->fmMuramPhysBaseAddr,
				ICMPV6Descriptor->numOfBindings *
				sizeof(t_DsarIcmpV6BindingEntry));
			uint16_t* ip_addr = (uint16_t*)&bindings->ipv6Addr;
			FM_DMP_LN(buf, n, "                  ip                        vlan id\n");
			for (i = 0; i < ICMPV6Descriptor->numOfBindings; i++)
			{
				n += snprintf(ip_str, 100,
					"%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x",
				ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3],
				ip_addr[4], ip_addr[5], ip_addr[6], ip_addr[7]);
				FM_DMP_LN(buf, n, "%s     0x%x\n", ip_str, bindings->vlanId);
			}
			iounmap(bindings);
		}
		if (ICMPV6Descriptor->p_Statistics)
		{
			t_DsarIcmpV6Statistics* icmpv6Stats = ioremap(
				ioread32be(&ICMPV6Descriptor->p_Statistics) +
				p_FmPort->fmMuramPhysBaseAddr,
				sizeof(t_DsarIcmpV6Statistics));
			FM_DMP_LN(buf, n, "statistics\n");
			FM_DMP_LN(buf, n, "INVAL_CNT:    0x%x\n", icmpv6Stats->invalCnt);
			FM_DMP_LN(buf, n, "NMVLAN_CNT:   0x%x\n", icmpv6Stats->nmVlanCnt);
			FM_DMP_LN(buf, n, "NMIP_CNT:     0x%x\n", icmpv6Stats->nmIpCnt);
			FM_DMP_LN(buf, n, "AR_CNT:       0x%x\n", icmpv6Stats->arCnt);
			iounmap(icmpv6Stats);
		}
		iounmap(ICMPV6Descriptor);
	}
	if (ArCommonDescPtr->p_SnmpDescriptor)
	{
		t_DsarSnmpDescriptor *SnmpDescriptor =
			(t_DsarSnmpDescriptor*)ioremap(ioread32be(
			&ArCommonDescPtr->p_SnmpDescriptor) + p_FmPort->
			fmMuramPhysBaseAddr, sizeof (t_DsarSnmpDescriptor));
		FM_DMP_LN(buf, n, "\nSNMP\n");
		FM_DMP_LN(buf, n, "===========\n");
		FM_DMP_LN(buf, n, "control bits 0x%04x\n", SnmpDescriptor->control);
		FM_DMP_LN(buf, n, "max message length 0x%04x\n", SnmpDescriptor->maxSnmpMsgLength);
		if (SnmpDescriptor->numOfIpv4Addresses)
		{
			char ip_str[100];
			t_DsarSnmpIpv4AddrTblEntry* addrs = ioremap(
				ioread32be(&SnmpDescriptor->p_Ipv4AddrTbl) +
				p_FmPort->fmMuramPhysBaseAddr,
				SnmpDescriptor->numOfIpv4Addresses *
				sizeof(t_DsarSnmpIpv4AddrTblEntry));
			uint8_t* ip_addr = (uint8_t*)&addrs->ipv4Addr;
			FM_DMP_LN(buf, n, "      ip          vlan id\n");
			for (i = 0; i < SnmpDescriptor->numOfIpv4Addresses; i++)
			{
				n += snprintf(ip_str, 100, "%d.%d.%d.%d",
					ip_addr[0], ip_addr[1],
					ip_addr[2], ip_addr[3]);
				FM_DMP_LN(buf, n, "%-15s     0x%x\n", ip_str, addrs->vlanId);
			}
			iounmap(addrs);
		}
		if (SnmpDescriptor->p_Statistics)
		{
			t_DsarSnmpStatistics* snmpStats = ioremap(
				ioread32be(&SnmpDescriptor->p_Statistics) +
				p_FmPort->fmMuramPhysBaseAddr,
				sizeof(t_DsarSnmpStatistics));
			FM_DMP_LN(buf, n, "statistics\n");
			FM_DMP_LN(buf, n, "snmpErrCnt:          0x%x\n", snmpStats->snmpErrCnt);
			FM_DMP_LN(buf, n, "snmpCommunityErrCnt: 0x%x\n", snmpStats->snmpCommunityErrCnt);
			FM_DMP_LN(buf, n, "snmpTotalDiscardCnt: 0x%x\n", snmpStats->snmpTotalDiscardCnt);
			FM_DMP_LN(buf, n, "snmpGetReqCnt:       0x%x\n", snmpStats->snmpGetReqCnt);
			FM_DMP_LN(buf, n, "snmpGetNextReqCnt:   0x%x\n", snmpStats->snmpGetNextReqCnt);
			iounmap(snmpStats);
		}
		iounmap(SnmpDescriptor);
	}
	iounmap(ArCommonDescPtr);
	iounmap(param_page);
	return n;
}

static ssize_t show_fm_port_dsar_mem(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
#endif
	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_LnxWrpFmPortDev =
		(t_LnxWrpFmPortDev *) dev_get_drvdata(dev);

	local_irq_save(flags);

	if (!p_LnxWrpFmPortDev->h_Dev) {
		n = snprintf(buf, PAGE_SIZE, "\tFM Port not configured...\n");
		return n;
	} else {
		n = snprintf(buf, PAGE_SIZE,
				"FM port driver registers dump.\n");
		n = fm_port_dsar_dump_mem(p_LnxWrpFmPortDev->h_Dev, buf, n);
	}

	local_irq_restore(flags);

	return n;
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

	return n;
#endif
}

static ssize_t show_fm_port_dsar_regs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
#endif
	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_LnxWrpFmPortDev =
		(t_LnxWrpFmPortDev *) dev_get_drvdata(dev);

	local_irq_save(flags);

	if (!p_LnxWrpFmPortDev->h_Dev) {
		n = snprintf(buf, PAGE_SIZE, "\tFM Port not configured...\n");
		return n;
	} else {
		n = snprintf(buf, PAGE_SIZE,
				"FM port driver registers dump.\n");
		n = fm_port_dsar_dump_regs(p_LnxWrpFmPortDev->h_Dev, buf, n);
	}

	local_irq_restore(flags);

	return n;
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

	return n;
#endif
}

#if (DPAA_VERSION >= 11)
static ssize_t show_fm_port_ipv4_options(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_LnxWrpFmPortDev =
		(t_LnxWrpFmPortDev *) dev_get_drvdata(dev);

	local_irq_save(flags);

	if (!p_LnxWrpFmPortDev->h_Dev) {
		n = snprintf(buf, PAGE_SIZE, "\tFM Port not configured...\n");
		return n;
	} else if (((t_FmPort *)p_LnxWrpFmPortDev->h_Dev)->p_ParamsPage
					== NULL) {
		n = snprintf(buf, PAGE_SIZE,
			"\tPort: FMan-controller params page not set\n");
		return n;
	} else {
		n = snprintf(buf, PAGE_SIZE,
			"Counter for fragmented pkt with IP header options\n");
		n = fm_port_dump_ipv4_opt(p_LnxWrpFmPortDev->h_Dev, buf, n);
	}

	local_irq_restore(flags);

	return n;
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

	return n;
#endif
}

#endif

static ssize_t show_fm_port_bmi_regs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_LnxWrpFmPortDev =
		(t_LnxWrpFmPortDev *) dev_get_drvdata(dev);

	local_irq_save(flags);

	if (!p_LnxWrpFmPortDev->h_Dev) {
		n = snprintf(buf, PAGE_SIZE, "\tFM Port not configured...\n");
		return n;
	} else {
		n = snprintf(buf, PAGE_SIZE,
				"FM port driver registers dump.\n");
		n = fm_port_dump_regs_bmi(p_LnxWrpFmPortDev->h_Dev, buf, n);
	}

	local_irq_restore(flags);

	return n;
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

	return n;
#endif
}

static ssize_t show_fm_port_qmi_regs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_LnxWrpFmPortDev =
		(t_LnxWrpFmPortDev *) dev_get_drvdata(dev);

	local_irq_save(flags);

	if (!p_LnxWrpFmPortDev->h_Dev) {
		n = snprintf(buf, PAGE_SIZE, "\tFM Port not configured...\n");
		return n;
	} else {
		n = snprintf(buf, PAGE_SIZE,
				"FM port driver registers dump.\n");
		n = fm_port_dump_regs_qmi(p_LnxWrpFmPortDev->h_Dev, buf, n);
	}

	local_irq_restore(flags);

	return n;
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

	return n;
#endif
}

static DEVICE_ATTR(fm_port_regs, S_IRUGO | S_IRUSR, show_fm_port_regs, NULL);
static DEVICE_ATTR(fm_port_qmi_regs, S_IRUGO | S_IRUSR, show_fm_port_qmi_regs, NULL);
static DEVICE_ATTR(fm_port_bmi_regs, S_IRUGO | S_IRUSR, show_fm_port_bmi_regs, NULL);
#if (DPAA_VERSION >= 11)
static DEVICE_ATTR(fm_port_ipv4_opt, S_IRUGO | S_IRUSR, show_fm_port_ipv4_options, NULL);
#endif
static DEVICE_ATTR(fm_port_dsar_regs, S_IRUGO | S_IRUSR, show_fm_port_dsar_regs, NULL);
static DEVICE_ATTR(fm_port_dsar_mem, S_IRUGO | S_IRUSR, show_fm_port_dsar_mem, NULL);

int fm_port_sysfs_create(struct device *dev)
{
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;

	if (dev == NULL)
		return -EINVAL;

	p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_LnxWrpFmPortDev == NULL))
		return -EINVAL;

	/* store to remove them when module is disabled */
	p_LnxWrpFmPortDev->dev_attr_regs = &dev_attr_fm_port_regs;
	p_LnxWrpFmPortDev->dev_attr_qmi_regs = &dev_attr_fm_port_qmi_regs;
	p_LnxWrpFmPortDev->dev_attr_bmi_regs = &dev_attr_fm_port_bmi_regs;
#if (DPAA_VERSION >= 11)
	p_LnxWrpFmPortDev->dev_attr_ipv4_opt = &dev_attr_fm_port_ipv4_opt;
#endif
	p_LnxWrpFmPortDev->dev_attr_dsar_regs = &dev_attr_fm_port_dsar_regs;
	p_LnxWrpFmPortDev->dev_attr_dsar_mem = &dev_attr_fm_port_dsar_mem;
	/* Registers dump entry - in future will be moved to debugfs */
	if (device_create_file(dev, &dev_attr_fm_port_regs) != 0)
		return -EIO;
	if (device_create_file(dev, &dev_attr_fm_port_qmi_regs) != 0)
		return -EIO;
	if (device_create_file(dev, &dev_attr_fm_port_bmi_regs) != 0)
		return -EIO;
#if (DPAA_VERSION >= 11)
	if (device_create_file(dev, &dev_attr_fm_port_ipv4_opt) != 0)
		return -EIO;
#endif
	if (device_create_file(dev, &dev_attr_fm_port_dsar_regs) != 0)
		return -EIO;
	if (device_create_file(dev, &dev_attr_fm_port_dsar_mem) != 0)
		return -EIO;
		
	/* FM Ports statistics */
	switch (p_LnxWrpFmPortDev->settings.param.portType) {
	case e_FM_PORT_TYPE_TX:
	case e_FM_PORT_TYPE_TX_10G:
		if (sysfs_create_group
			(&dev->kobj, &fm_tx_port_dev_stats_attr_grp) != 0)
			return -EIO;
		break;
	case e_FM_PORT_TYPE_RX:
	case e_FM_PORT_TYPE_RX_10G:
		if (sysfs_create_group
			(&dev->kobj, &fm_rx_port_dev_stats_attr_grp) != 0)
			return -EIO;
		break;
	case e_FM_PORT_TYPE_DUMMY:
	case e_FM_PORT_TYPE_OH_OFFLINE_PARSING:
		if (sysfs_create_group
			(&dev->kobj, &fm_oh_port_dev_stats_attr_grp) != 0)
			return -EIO;
		break;
	default:
		WARN(1, "FMD: failure at %s:%d/%s()!\n", __FILE__, __LINE__,
			__func__);
		return -EINVAL;
		break;
	};

	return 0;
}

void fm_port_sysfs_destroy(struct device *dev)
{
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = NULL;

	/* this function has never been tested !!! */

	if (WARN_ON(dev == NULL))
		return;

	p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_LnxWrpFmPortDev == NULL))
		return;

	/* The name attribute will be freed also by these 2 functions? */
	switch (p_LnxWrpFmPortDev->settings.param.portType) {
	case e_FM_PORT_TYPE_TX:
	case e_FM_PORT_TYPE_TX_10G:
		sysfs_remove_group(&dev->kobj, &fm_tx_port_dev_stats_attr_grp);
		break;
	case e_FM_PORT_TYPE_RX:
	case e_FM_PORT_TYPE_RX_10G:
		sysfs_remove_group(&dev->kobj, &fm_rx_port_dev_stats_attr_grp);
		break;
	case e_FM_PORT_TYPE_DUMMY:
	case e_FM_PORT_TYPE_OH_OFFLINE_PARSING:
		sysfs_remove_group(&dev->kobj, &fm_oh_port_dev_stats_attr_grp);
		break;
	default:
		WARN(1, "FMD: failure at %s:%d/%s()!\n", __FILE__, __LINE__,
		     __func__);
		break;
	};

	device_remove_file(dev, p_LnxWrpFmPortDev->dev_attr_regs);
	device_remove_file(dev, p_LnxWrpFmPortDev->dev_attr_qmi_regs);
	device_remove_file(dev, p_LnxWrpFmPortDev->dev_attr_bmi_regs);
#if (DPAA_VERSION >= 11)
	device_remove_file(dev, p_LnxWrpFmPortDev->dev_attr_ipv4_opt);
#endif
	device_remove_file(dev, p_LnxWrpFmPortDev->dev_attr_dsar_regs);
	device_remove_file(dev, p_LnxWrpFmPortDev->dev_attr_dsar_mem);
}


int fm_port_dump_regs(void *h_dev, char *buf, int nn)
{
	t_FmPort *p_FmPort;
	t_Fm *p_Fm;
	uint8_t hardwarePortId;
	int n = nn;

	p_FmPort = (t_FmPort *)h_dev;
	hardwarePortId = p_FmPort->hardwarePortId;
	p_Fm = (t_Fm *)p_FmPort->h_Fm;

	FM_DMP_TITLE(buf, n, &p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId - 1],
			"fmbm_pp for port %u", hardwarePortId);
	FM_DMP_MEM_32(buf, n,
			&p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId - 1]);

	FM_DMP_TITLE(buf, n, &p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId - 1],
			"fmbm_pfs for port %u", hardwarePortId);
	FM_DMP_MEM_32(buf, n,
		&p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId - 1]);

	FM_DMP_TITLE(buf, n,
			&p_Fm->p_FmBmiRegs->fmbm_spliodn[hardwarePortId - 1],
			"fmbm_spliodn for port %u", hardwarePortId);
	FM_DMP_MEM_32(buf, n,
			&p_Fm->p_FmBmiRegs->fmbm_spliodn[hardwarePortId - 1]);

	FM_DMP_TITLE(buf, n, &p_Fm->p_FmFpmRegs->fmfp_ps[hardwarePortId],
			"fmfp_psfor port %u", hardwarePortId);
	FM_DMP_MEM_32(buf, n, &p_Fm->p_FmFpmRegs->fmfp_ps[hardwarePortId]);

	FM_DMP_TITLE(buf, n, &p_Fm->p_FmDmaRegs->fmdmplr[hardwarePortId / 2],
			"fmdmplrfor port %u", hardwarePortId);
	FM_DMP_MEM_32(buf, n,
			&p_Fm->p_FmDmaRegs->fmdmplr[hardwarePortId / 2]);
	return n;
}

#if (DPAA_VERSION >= 11)

int fm_port_dump_ipv4_opt(void *h_dev, char *buf, int nn)
{
	t_FmPort *p_FmPort;
	int n = nn;

	p_FmPort = (t_FmPort *)h_dev;

	FM_DMP_V32(buf, n, p_FmPort->p_ParamsPage, ipfOptionsCounter);

	FM_DMP_SUBTITLE(buf, n, "\n");

	return n;
}
#endif

int fm_port_dump_regs_bmi(void *h_dev, char *buf, int nn)
{
	t_FmPort *p_FmPort;
	u_FmPortBmiRegs *p_bmi;

	char		arr[20];
	uint8_t		flag;
	int		i = 0;
	int		n = nn;

	p_FmPort = (t_FmPort *)h_dev;
	p_bmi = p_FmPort->p_FmPortBmiRegs;

	memset(arr, 0, sizeof(arr));
	switch (p_FmPort->portType) {
	case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
		strcpy(arr, "OFFLINE-PARSING");
		flag = 0;
		break;
	case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
		strcpy(arr, "HOST-COMMAND");
		flag = 0;
		break;
	case (e_FM_PORT_TYPE_RX):
		strcpy(arr, "RX");
		flag = 1;
		break;
	case (e_FM_PORT_TYPE_RX_10G):
		strcpy(arr, "RX-10G");
		flag = 1;
		break;
	case (e_FM_PORT_TYPE_TX):
		strcpy(arr, "TX");
		flag = 2;
		break;
	case (e_FM_PORT_TYPE_TX_10G):
		strcpy(arr, "TX-10G");
		flag = 2;
		break;
	default:
		return -EINVAL;
	}

	FM_DMP_TITLE(buf, n, NULL,
		"FMan-Port (%s #%d) registers:",
		arr, p_FmPort->portId);

	FM_DMP_TITLE(buf, n, p_bmi, "Bmi Port Regs");

	switch (flag) {
	case (0):
		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ocfg);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ost);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_oda);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_oicp);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofdne);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofne);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofca);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofpne);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_opso);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_opp);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_occb);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_oim);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofp);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofed);

		FM_DMP_TITLE(buf, n,
			&(p_bmi->ohPortBmiRegs.fmbm_oprai), "fmbm_oprai");
		for (i = 0; i < FM_PORT_PRS_RESULT_NUM_OF_WORDS; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->ohPortBmiRegs.fmbm_oprai[i]));
		}
		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofqid);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_oefqid);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofsdm);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofsem);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofene);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_orlmts);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_orlmt);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ocmne);
		{
#ifndef FM_NO_OP_OBSERVED_POOLS
		if (p_FmPort->fmRevInfo.majorRev == 4) {
			FM_DMP_TITLE(buf, n,
				&p_bmi->ohPortBmiRegs.fmbm_oebmpi,
				"fmbm_oebmpi");

			for (i = 0; i < FM_PORT_MAX_NUM_OF_OBSERVED_EXT_POOLS; ++i) {
				FM_DMP_MEM_32(buf, n,
					&(p_bmi->ohPortBmiRegs.fmbm_oebmpi[i]));
			}
			FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ocgm);
		}
#endif /* !FM_NO_OP_OBSERVED_POOLS */
		}
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ostc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofrc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofdc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofledc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofufdc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_offc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofwdc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofldec);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_opc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_opcp);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_occn);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_otuc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_oduc);
		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ofuc);
		FM_DMP_TITLE(buf, n, &(p_bmi->ohPortBmiRegs.fmbm_odcfg),
				"fmbm_odcfg");
		for (i = 0; i < 3; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->ohPortBmiRegs.fmbm_odcfg[i]));
		}
		FM_DMP_SUBTITLE(buf, n, "\n");

		FM_DMP_V32(buf, n, &p_bmi->ohPortBmiRegs, fmbm_ogpr);
	break;
	case (1):
		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rcfg);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rst);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rda);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfp);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_reth);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfed);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_ricp);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rebm);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfne);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfca);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfpne);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rpso);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rpp);
		FM_DMP_TITLE(buf, n, &(p_bmi->rxPortBmiRegs.fmbm_rprai),
			"fmbm_rprai");
		for (i = 0; i < FM_PORT_PRS_RESULT_NUM_OF_WORDS; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->rxPortBmiRegs.fmbm_rprai[i]));
		}
		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfqid);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_refqid);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfsdm);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfsem);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfene);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rcmne);
		FM_DMP_TITLE(buf, n, &p_bmi->rxPortBmiRegs.fmbm_ebmpi,
				"fmbm_ebmpi");
		for (i = 0; i < FM_PORT_MAX_NUM_OF_EXT_POOLS; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->rxPortBmiRegs.fmbm_ebmpi[i]));
		}
		FM_DMP_TITLE(buf, n, &p_bmi->rxPortBmiRegs.fmbm_acnt,
				"fmbm_acnt");
		for (i = 0; i < FM_PORT_MAX_NUM_OF_EXT_POOLS; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->rxPortBmiRegs.fmbm_acnt[i]));
		}
		FM_DMP_TITLE(buf, n, &p_bmi->rxPortBmiRegs.fmbm_rcgm,
				"fmbm_rcgm");
		for (i = 0; i < FM_PORT_NUM_OF_CONGESTION_GRPS / 32; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->rxPortBmiRegs.fmbm_rcgm[i]));
		}

		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rmpd);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rstc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfrc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfbc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rlfc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rffc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfcd);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfldec);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rodc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rpc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rpcp);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rccn);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rtuc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rrquc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rduc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rfuc);
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rpac);
		FM_DMP_TITLE(buf, n, &(p_bmi->rxPortBmiRegs.fmbm_rdcfg),
				"fmbm_rdcfg");
		for (i = 0; i < 3; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->rxPortBmiRegs.fmbm_rdcfg[i]));
		}
		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->rxPortBmiRegs, fmbm_rgpr);
		break;
	case (2):
		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tcfg);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tst);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tda);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfp);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfed);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_ticp);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfdne);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfca);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tcfqid);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfeqid);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfene);
#if (DPAA_VERSION >= 11)
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfne);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tcmne);
#endif /* (DPAA_VERSION >= 11) */
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_trlmts);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_trlmt);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tstc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfrc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfdc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfledc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfufdc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tpc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tpcp);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tccn);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_ttuc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_ttcquc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tduc);
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tfuc);
		FM_DMP_TITLE(buf, n, &(p_bmi->txPortBmiRegs.fmbm_tdcfg),
				"fmbm_tdcfg");
		for (i = 0; i < 3 ; ++i) {
			FM_DMP_MEM_32(buf, n,
				&(p_bmi->txPortBmiRegs.fmbm_tdcfg[i]));
		}
		FM_DMP_SUBTITLE(buf, n, "\n");
		FM_DMP_V32(buf, n, &p_bmi->txPortBmiRegs, fmbm_tgpr);
		break;
	}

	FM_DMP_SUBTITLE(buf, n, "\n");

	return n;
}

int fm_port_dump_regs_qmi(void *h_dev, char *buf, int nn)
{
	t_FmPort *p_FmPort;
	int n = nn;

	p_FmPort = (t_FmPort *)h_dev;

	FM_DMP_TITLE(buf, n, p_FmPort->p_FmPortQmiRegs, "Qmi Port Regs");

	FM_DMP_V32(buf, n, p_FmPort->p_FmPortQmiRegs, fmqm_pnc);
	FM_DMP_V32(buf, n, p_FmPort->p_FmPortQmiRegs, fmqm_pns);
	FM_DMP_V32(buf, n, p_FmPort->p_FmPortQmiRegs, fmqm_pnts);
	FM_DMP_V32(buf, n, p_FmPort->p_FmPortQmiRegs, fmqm_pnen);
	FM_DMP_V32(buf, n, p_FmPort->p_FmPortQmiRegs, fmqm_pnetfc);
	FM_DMP_V32(buf, n,
		&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs, fmqm_pndn);
	FM_DMP_V32(buf, n,
		&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs, fmqm_pndc);
	FM_DMP_V32(buf, n,
		&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs, fmqm_pndtfc);
	FM_DMP_V32(buf, n,
		&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs, fmqm_pndfdc);
	FM_DMP_V32(buf, n,
		&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs, fmqm_pndcc);

	FM_DMP_SUBTITLE(buf, n, "\n");

	return n;
}

