// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author:
 *  Zhigang.Wei <zhigang.wei@mediatek.com>
 *  Chunfeng.Yun <chunfeng.yun@mediatek.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "xhci.h"
#include "xhci-mtk.h"

#define SSP_BW_BOUNDARY	130000
#define SS_BW_BOUNDARY	51000
/* table 5-5. High-speed Isoc Transaction Limits in usb_20 spec */
#define HS_BW_BOUNDARY	6144
/* usb2 spec section11.18.1: at most 188 FS bytes per microframe */
#define FS_PAYLOAD_MAX 188
#define LS_PAYLOAD_MAX 18
/* section 11.18.1, per fs frame */
#define FS_BW_BOUNDARY	1157
#define LS_BW_BOUNDARY	144

/*
 * max number of microframes for split transfer, assume extra-cs budget is 0
 * for fs isoc in : 1 ss + 1 idle + 6 cs (roundup(1023/188))
 */
#define TT_MICROFRAMES_MAX	8
/* offset from SS for fs/ls isoc/intr ep (ss + idle) */
#define CS_OFFSET	2

#define DBG_BUF_EN	64

/* schedule error type */
#define ESCH_SS_Y6		1001
#define ESCH_SS_OVERLAP		1002
#define ESCH_CS_OVERFLOW	1003
#define ESCH_BW_OVERFLOW	1004
#define ESCH_FIXME		1005

/* mtk scheduler bitmasks */
#define EP_BPKTS(p)	((p) & 0x7f)
#define EP_BCSCOUNT(p)	(((p) & 0x7) << 8)
#define EP_BBM(p)	((p) << 11)
#define EP_BOFFSET(p)	((p) & 0x3fff)
#define EP_BREPEAT(p)	(((p) & 0x7fff) << 16)

static char *sch_error_string(int err_num)
{
	switch (err_num) {
	case ESCH_SS_Y6:
		return "Can't schedule Start-Split in Y6";
	case ESCH_SS_OVERLAP:
		return "Can't find a suitable Start-Split location";
	case ESCH_CS_OVERFLOW:
		return "The last Complete-Split is greater than 7";
	case ESCH_BW_OVERFLOW:
		return "Bandwidth exceeds the maximum limit";
	case ESCH_FIXME:
		return "FIXME, to be resolved";
	default:
		return "Unknown";
	}
}

static int is_fs_or_ls(enum usb_device_speed speed)
{
	return speed == USB_SPEED_FULL || speed == USB_SPEED_LOW;
}

static const char *
decode_ep(struct usb_host_endpoint *ep, enum usb_device_speed speed)
{
	static char buf[DBG_BUF_EN];
	struct usb_endpoint_descriptor *epd = &ep->desc;
	unsigned int interval;
	const char *unit;

	interval = usb_decode_interval(epd, speed);
	if (interval % 1000) {
		unit = "us";
	} else {
		unit = "ms";
		interval /= 1000;
	}

	snprintf(buf, DBG_BUF_EN, "%s ep%d%s %s, mpkt:%d, interval:%d/%d%s",
		 usb_speed_string(speed), usb_endpoint_num(epd),
		 usb_endpoint_dir_in(epd) ? "in" : "out",
		 usb_ep_type_string(usb_endpoint_type(epd)),
		 usb_endpoint_maxp(epd), epd->bInterval, interval, unit);

	return buf;
}

static u32 get_bw_boundary(enum usb_device_speed speed)
{
	u32 boundary;

	switch (speed) {
	case USB_SPEED_SUPER_PLUS:
		boundary = SSP_BW_BOUNDARY;
		break;
	case USB_SPEED_SUPER:
		boundary = SS_BW_BOUNDARY;
		break;
	default:
		boundary = HS_BW_BOUNDARY;
		break;
	}

	return boundary;
}

/*
* get the bandwidth domain which @ep belongs to.
*
* the bandwidth domain array is saved to @sch_array of struct xhci_hcd_mtk,
* each HS root port is treated as a single bandwidth domain,
* but each SS root port is treated as two bandwidth domains, one for IN eps,
* one for OUT eps.
*/
static struct mu3h_sch_bw_info *
get_bw_info(struct xhci_hcd_mtk *mtk, struct usb_device *udev,
	    struct usb_host_endpoint *ep)
{
	struct xhci_hcd *xhci = hcd_to_xhci(mtk->hcd);
	struct xhci_virt_device *virt_dev;
	int bw_index;

	virt_dev = xhci->devs[udev->slot_id];
	if (!virt_dev->rhub_port) {
		WARN_ONCE(1, "%s invalid rhub port\n", dev_name(&udev->dev));
		return NULL;
	}

	if (udev->speed >= USB_SPEED_SUPER) {
		if (usb_endpoint_dir_out(&ep->desc))
			bw_index = (virt_dev->rhub_port->hw_portnum) * 2;
		else
			bw_index = (virt_dev->rhub_port->hw_portnum) * 2 + 1;
	} else {
		/* add one more for each SS port */
		bw_index = virt_dev->rhub_port->hw_portnum + xhci->usb3_rhub.num_ports;
	}

	return &mtk->sch_array[bw_index];
}

static u32 get_esit(struct xhci_ep_ctx *ep_ctx)
{
	u32 esit;

	esit = 1 << CTX_TO_EP_INTERVAL(le32_to_cpu(ep_ctx->ep_info));
	if (esit > XHCI_MTK_MAX_ESIT)
		esit = XHCI_MTK_MAX_ESIT;

	return esit;
}

static struct mu3h_sch_tt *find_tt(struct usb_device *udev)
{
	struct usb_tt *utt = udev->tt;
	struct mu3h_sch_tt *tt, **tt_index, **ptt;
	bool allocated_index = false;

	if (!utt)
		return NULL;	/* Not below a TT */

	/*
	 * Find/create our data structure.
	 * For hubs with a single TT, we get it directly.
	 * For hubs with multiple TTs, there's an extra level of pointers.
	 */
	tt_index = NULL;
	if (utt->multi) {
		tt_index = utt->hcpriv;
		if (!tt_index) {	/* Create the index array */
			tt_index = kcalloc(utt->hub->maxchild,
					sizeof(*tt_index), GFP_KERNEL);
			if (!tt_index)
				return ERR_PTR(-ENOMEM);
			utt->hcpriv = tt_index;
			allocated_index = true;
		}
		ptt = &tt_index[udev->ttport - 1];
	} else {
		ptt = (struct mu3h_sch_tt **) &utt->hcpriv;
	}

	tt = *ptt;
	if (!tt) {	/* Create the mu3h_sch_tt */
		tt = kzalloc(sizeof(*tt), GFP_KERNEL);
		if (!tt) {
			if (allocated_index) {
				utt->hcpriv = NULL;
				kfree(tt_index);
			}
			return ERR_PTR(-ENOMEM);
		}
		INIT_LIST_HEAD(&tt->ep_list);
		*ptt = tt;
	}

	return tt;
}

/* Release the TT above udev, if it's not in use */
static void drop_tt(struct usb_device *udev)
{
	struct usb_tt *utt = udev->tt;
	struct mu3h_sch_tt *tt, **tt_index, **ptt;
	int i, cnt;

	if (!utt || !utt->hcpriv)
		return;		/* Not below a TT, or never allocated */

	cnt = 0;
	if (utt->multi) {
		tt_index = utt->hcpriv;
		ptt = &tt_index[udev->ttport - 1];
		/*  How many entries are left in tt_index? */
		for (i = 0; i < utt->hub->maxchild; ++i)
			cnt += !!tt_index[i];
	} else {
		tt_index = NULL;
		ptt = (struct mu3h_sch_tt **)&utt->hcpriv;
	}

	tt = *ptt;
	if (!tt || !list_empty(&tt->ep_list))
		return;		/* never allocated , or still in use*/

	*ptt = NULL;
	kfree(tt);

	if (cnt == 1) {
		utt->hcpriv = NULL;
		kfree(tt_index);
	}
}

static struct mu3h_sch_ep_info *
create_sch_ep(struct xhci_hcd_mtk *mtk, struct usb_device *udev,
	      struct usb_host_endpoint *ep, struct xhci_ep_ctx *ep_ctx)
{
	struct mu3h_sch_ep_info *sch_ep;
	struct mu3h_sch_bw_info *bw_info;
	struct mu3h_sch_tt *tt = NULL;
	u32 len;

	bw_info = get_bw_info(mtk, udev, ep);
	if (!bw_info)
		return ERR_PTR(-ENODEV);

	if (is_fs_or_ls(udev->speed))
		len = TT_MICROFRAMES_MAX;
	else if ((udev->speed >= USB_SPEED_SUPER) &&
		 usb_endpoint_xfer_isoc(&ep->desc))
		len = get_esit(ep_ctx);
	else
		len = 1;

	sch_ep = kzalloc(struct_size(sch_ep, bw_budget_table, len), GFP_KERNEL);
	if (!sch_ep)
		return ERR_PTR(-ENOMEM);

	if (is_fs_or_ls(udev->speed)) {
		tt = find_tt(udev);
		if (IS_ERR(tt)) {
			kfree(sch_ep);
			return ERR_PTR(-ENOMEM);
		}
	}

	sch_ep->bw_info = bw_info;
	sch_ep->sch_tt = tt;
	sch_ep->ep = ep;
	sch_ep->speed = udev->speed;
	INIT_LIST_HEAD(&sch_ep->endpoint);
	INIT_LIST_HEAD(&sch_ep->tt_endpoint);
	INIT_HLIST_NODE(&sch_ep->hentry);

	return sch_ep;
}

static void setup_sch_info(struct xhci_ep_ctx *ep_ctx,
			   struct mu3h_sch_ep_info *sch_ep)
{
	u32 ep_type;
	u32 maxpkt;
	u32 max_burst;
	u32 mult;
	u32 esit_pkts;
	u32 max_esit_payload;
	u32 bw_per_microframe;
	u32 *bwb_table;
	int i;

	bwb_table = sch_ep->bw_budget_table;
	ep_type = CTX_TO_EP_TYPE(le32_to_cpu(ep_ctx->ep_info2));
	maxpkt = MAX_PACKET_DECODED(le32_to_cpu(ep_ctx->ep_info2));
	max_burst = CTX_TO_MAX_BURST(le32_to_cpu(ep_ctx->ep_info2));
	mult = CTX_TO_EP_MULT(le32_to_cpu(ep_ctx->ep_info));
	max_esit_payload =
		(CTX_TO_MAX_ESIT_PAYLOAD_HI(
			le32_to_cpu(ep_ctx->ep_info)) << 16) |
		 CTX_TO_MAX_ESIT_PAYLOAD(le32_to_cpu(ep_ctx->tx_info));

	sch_ep->esit = get_esit(ep_ctx);
	sch_ep->num_esit = XHCI_MTK_MAX_ESIT / sch_ep->esit;
	sch_ep->ep_type = ep_type;
	sch_ep->maxpkt = maxpkt;
	sch_ep->offset = 0;
	sch_ep->burst_mode = 0;
	sch_ep->repeat = 0;

	if (sch_ep->speed == USB_SPEED_HIGH) {
		sch_ep->cs_count = 0;

		/*
		 * usb_20 spec section5.9
		 * a single microframe is enough for HS synchromous endpoints
		 * in a interval
		 */
		sch_ep->num_budget_microframes = 1;

		/*
		 * xHCI spec section6.2.3.4
		 * @max_burst is the number of additional transactions
		 * opportunities per microframe
		 */
		sch_ep->pkts = max_burst + 1;
		bwb_table[0] = maxpkt * sch_ep->pkts;
	} else if (sch_ep->speed >= USB_SPEED_SUPER) {
		/* usb3_r1 spec section4.4.7 & 4.4.8 */
		sch_ep->cs_count = 0;
		sch_ep->burst_mode = 1;
		/*
		 * some device's (d)wBytesPerInterval is set as 0,
		 * then max_esit_payload is 0, so evaluate esit_pkts from
		 * mult and burst
		 */
		esit_pkts = DIV_ROUND_UP(max_esit_payload, maxpkt);
		if (esit_pkts == 0)
			esit_pkts = (mult + 1) * (max_burst + 1);

		if (ep_type == INT_IN_EP || ep_type == INT_OUT_EP) {
			sch_ep->pkts = esit_pkts;
			sch_ep->num_budget_microframes = 1;
			bwb_table[0] = maxpkt * sch_ep->pkts;
		}

		if (ep_type == ISOC_IN_EP || ep_type == ISOC_OUT_EP) {

			if (sch_ep->esit == 1)
				sch_ep->pkts = esit_pkts;
			else if (esit_pkts <= sch_ep->esit)
				sch_ep->pkts = 1;
			else
				sch_ep->pkts = roundup_pow_of_two(esit_pkts)
					/ sch_ep->esit;

			sch_ep->num_budget_microframes =
				DIV_ROUND_UP(esit_pkts, sch_ep->pkts);

			sch_ep->repeat = !!(sch_ep->num_budget_microframes > 1);
			bw_per_microframe = maxpkt * sch_ep->pkts;

			for (i = 0; i < sch_ep->num_budget_microframes - 1; i++)
				bwb_table[i] = bw_per_microframe;

			/* last one <= bw_per_microframe */
			bwb_table[i] = maxpkt * esit_pkts - i * bw_per_microframe;
		}
	} else if (is_fs_or_ls(sch_ep->speed)) {
		sch_ep->pkts = 1; /* at most one packet for each microframe */

		/*
		 * @cs_count will be updated to add extra-cs when
		 * check TT for INT_OUT_EP, ISOC/INT_IN_EP type
		 * @maxpkt <= 1023;
		 */
		sch_ep->cs_count = DIV_ROUND_UP(maxpkt, FS_PAYLOAD_MAX);
		sch_ep->num_budget_microframes = sch_ep->cs_count;

		/* init budget table */
		if (ep_type == ISOC_OUT_EP) {
			for (i = 0; i < sch_ep->cs_count - 1; i++)
				bwb_table[i] = FS_PAYLOAD_MAX;

			bwb_table[i] = maxpkt - i * FS_PAYLOAD_MAX;
		} else if (ep_type == INT_OUT_EP) {
			/* only first one used (maxpkt <= 64), others zero */
			bwb_table[0] = maxpkt;
		} else { /* INT_IN_EP or ISOC_IN_EP */
			bwb_table[0] = 0; /* start split */
			bwb_table[1] = 0; /* idle */
			/*
			 * @cs_count will be updated according to cs position
			 * (add 1 or 2 extra-cs), but assume only first
			 * @num_budget_microframes elements will be used later,
			 * although in fact it does not (extra-cs budget many receive
			 * some data for IN ep);
			 * @cs_count is 1 for INT_IN_EP (maxpkt <= 64);
			 */
			for (i = 0; i < sch_ep->cs_count - 1; i++)
				bwb_table[i + CS_OFFSET] = FS_PAYLOAD_MAX;

			bwb_table[i + CS_OFFSET] = maxpkt - i * FS_PAYLOAD_MAX;
			/* ss + idle */
			sch_ep->num_budget_microframes += CS_OFFSET;
		}
	}
}

/* Get maximum bandwidth when we schedule at offset slot. */
static u32 get_max_bw(struct mu3h_sch_bw_info *sch_bw,
	struct mu3h_sch_ep_info *sch_ep, u32 offset)
{
	u32 max_bw = 0;
	u32 bw;
	int i, j, k;

	for (i = 0; i < sch_ep->num_esit; i++) {
		u32 base = offset + i * sch_ep->esit;

		for (j = 0; j < sch_ep->num_budget_microframes; j++) {
			k = XHCI_MTK_BW_INDEX(base + j);
			bw = sch_bw->bus_bw[k] + sch_ep->bw_budget_table[j];
			if (bw > max_bw)
				max_bw = bw;
		}
	}
	return max_bw;
}

/*
 * for OUT: get first SS consumed bw;
 * for IN: get first CS consumed bw;
 */
static u16 get_fs_bw(struct mu3h_sch_ep_info *sch_ep, int offset)
{
	struct mu3h_sch_tt *tt = sch_ep->sch_tt;
	u16 fs_bw;

	if (sch_ep->ep_type == ISOC_OUT_EP || sch_ep->ep_type == INT_OUT_EP)
		fs_bw = tt->fs_bus_bw_out[XHCI_MTK_BW_INDEX(offset)];
	else	/* skip ss + idle */
		fs_bw = tt->fs_bus_bw_in[XHCI_MTK_BW_INDEX(offset + CS_OFFSET)];

	return fs_bw;
}

static void update_bus_bw(struct mu3h_sch_bw_info *sch_bw,
	struct mu3h_sch_ep_info *sch_ep, bool used)
{
	u32 base;
	int i, j, k;

	for (i = 0; i < sch_ep->num_esit; i++) {
		base = sch_ep->offset + i * sch_ep->esit;
		for (j = 0; j < sch_ep->num_budget_microframes; j++) {
			k = XHCI_MTK_BW_INDEX(base + j);
			if (used)
				sch_bw->bus_bw[k] += sch_ep->bw_budget_table[j];
			else
				sch_bw->bus_bw[k] -= sch_ep->bw_budget_table[j];
		}
	}
}

static int check_ls_budget_microframes(struct mu3h_sch_ep_info *sch_ep, int offset)
{
	struct mu3h_sch_tt *tt = sch_ep->sch_tt;
	int i;

	if (sch_ep->speed != USB_SPEED_LOW)
		return 0;

	if (sch_ep->ep_type == INT_OUT_EP)
		i = XHCI_MTK_BW_INDEX(offset);
	else if (sch_ep->ep_type == INT_IN_EP)
		i = XHCI_MTK_BW_INDEX(offset + CS_OFFSET); /* skip ss + idle */
	else
		return -EINVAL;

	if (tt->ls_bus_bw[i] + sch_ep->maxpkt > LS_PAYLOAD_MAX)
		return -ESCH_BW_OVERFLOW;

	return 0;
}

static int check_fs_budget_microframes(struct mu3h_sch_ep_info *sch_ep, int offset)
{
	struct mu3h_sch_tt *tt = sch_ep->sch_tt;
	u32 tmp;
	int i, k;

	/*
	 * for OUT eps, will transfer exactly assigned length of data,
	 * so can't allocate more than 188 bytes;
	 * but it's not for IN eps, usually it can't receive full
	 * 188 bytes in a uframe, if it not assign full 188 bytes,
	 * can add another one;
	 */
	for (i = 0; i < sch_ep->num_budget_microframes; i++) {
		k = XHCI_MTK_BW_INDEX(offset + i);
		if (sch_ep->ep_type == ISOC_OUT_EP || sch_ep->ep_type == INT_OUT_EP)
			tmp = tt->fs_bus_bw_out[k] + sch_ep->bw_budget_table[i];
		else /* ep_type : ISOC IN / INTR IN */
			tmp = tt->fs_bus_bw_in[k];

		if (tmp > FS_PAYLOAD_MAX)
			return -ESCH_BW_OVERFLOW;
	}

	return 0;
}

static int check_fs_budget_frames(struct mu3h_sch_ep_info *sch_ep, int offset)
{
	struct mu3h_sch_tt *tt = sch_ep->sch_tt;
	u32 head, tail;
	int i, j, k;

	/* bugdet scheduled may cross at most two fs frames */
	j = XHCI_MTK_BW_INDEX(offset) / UFRAMES_PER_FRAME;
	k = XHCI_MTK_BW_INDEX(offset + sch_ep->num_budget_microframes - 1) / UFRAMES_PER_FRAME;

	if (j != k) {
		head = tt->fs_frame_bw[j];
		tail = tt->fs_frame_bw[k];
	} else {
		head = tt->fs_frame_bw[j];
		tail = 0;
	}

	j = roundup(offset, UFRAMES_PER_FRAME);
	for (i = 0; i < sch_ep->num_budget_microframes; i++) {
		if ((offset + i) < j)
			head += sch_ep->bw_budget_table[i];
		else
			tail += sch_ep->bw_budget_table[i];
	}

	if (head > FS_BW_BOUNDARY || tail > FS_BW_BOUNDARY)
		return -ESCH_BW_OVERFLOW;

	return 0;
}

static int check_fs_bus_bw(struct mu3h_sch_ep_info *sch_ep, int offset)
{
	int i, base;
	int ret = 0;

	for (i = 0; i < sch_ep->num_esit; i++) {
		base = offset + i * sch_ep->esit;

		ret = check_ls_budget_microframes(sch_ep, base);
		if (ret)
			goto err;

		ret = check_fs_budget_microframes(sch_ep, base);
		if (ret)
			goto err;

		ret = check_fs_budget_frames(sch_ep, base);
		if (ret)
			goto err;
	}

err:
	return ret;
}

static int check_ss_and_cs(struct mu3h_sch_ep_info *sch_ep, u32 offset)
{
	u32 start_ss, last_ss;
	u32 start_cs, last_cs;

	start_ss = offset % UFRAMES_PER_FRAME;

	if (sch_ep->ep_type == ISOC_OUT_EP) {
		last_ss = start_ss + sch_ep->cs_count - 1;

		/*
		 * usb_20 spec section11.18:
		 * must never schedule Start-Split in Y6
		 */
		if (!(start_ss == 7 || last_ss < 6))
			return -ESCH_SS_Y6;

	} else {
		/* maxpkt <= 1023, cs <= 6 */
		u32 cs_count = DIV_ROUND_UP(sch_ep->maxpkt, FS_PAYLOAD_MAX);

		/*
		 * usb_20 spec section11.18:
		 * must never schedule Start-Split in Y6
		 */
		if (start_ss == 6)
			return -ESCH_SS_Y6;

		/* one uframe for ss + one uframe for idle */
		start_cs = (start_ss + CS_OFFSET) % UFRAMES_PER_FRAME;
		last_cs = start_cs + cs_count - 1;
		if (last_cs > 7)
			return -ESCH_CS_OVERFLOW;

		/* add extra-cs */
		cs_count += (last_cs == 7) ? 1 : 2;
		if (cs_count > 7)
			cs_count = 7; /* HW limit */

		sch_ep->cs_count = cs_count;

	}

	return 0;
}

/*
 * when isoc-out transfers 188 bytes in a uframe, and send isoc/intr's
 * ss token in the uframe, may cause 'bit stuff error' in downstream
 * port;
 * when isoc-out transfer less than 188 bytes in a uframe, shall send
 * isoc-in's ss after isoc-out's ss (but hw can't ensure the sequence,
 * so just avoid overlap).
 */
static int check_isoc_ss_overlap(struct mu3h_sch_ep_info *sch_ep, u32 offset)
{
	struct mu3h_sch_tt *tt = sch_ep->sch_tt;
	int base;
	int i, j, k;

	if (!tt)
		return 0;

	for (i = 0; i < sch_ep->num_esit; i++) {
		base = offset + i * sch_ep->esit;

		if (sch_ep->ep_type == ISOC_OUT_EP) {
			for (j = 0; j < sch_ep->num_budget_microframes; j++) {
				k = XHCI_MTK_BW_INDEX(base + j);
				if (tt->in_ss_cnt[k])
					return -ESCH_SS_OVERLAP;
			}
		} else if (sch_ep->ep_type == ISOC_IN_EP || sch_ep->ep_type == INT_IN_EP) {
			k = XHCI_MTK_BW_INDEX(base);
			/* only check IN's ss */
			if (tt->fs_bus_bw_out[k])
				return -ESCH_SS_OVERLAP;
		}
	}

	return 0;
}

static int check_sch_tt_budget(struct mu3h_sch_ep_info *sch_ep, u32 offset)
{
	int ret;

	ret = check_ss_and_cs(sch_ep, offset);
	if (ret)
		return ret;

	ret = check_isoc_ss_overlap(sch_ep, offset);
	if (ret)
		return ret;

	return check_fs_bus_bw(sch_ep, offset);
}

/* allocate microframes in the ls/fs frame */
static int alloc_sch_portion_of_frame(struct mu3h_sch_ep_info *sch_ep)
{
	struct mu3h_sch_bw_info *sch_bw = sch_ep->bw_info;
	const u32 bw_boundary = get_bw_boundary(sch_ep->speed);
	u32 bw_max, fs_bw_min;
	u32 offset, offset_min;
	u16 fs_bw;
	int frames;
	int i, j;
	int ret;

	frames = sch_ep->esit / UFRAMES_PER_FRAME;

	for (i = 0; i < UFRAMES_PER_FRAME; i++) {
		fs_bw_min = FS_PAYLOAD_MAX;
		offset_min = XHCI_MTK_MAX_ESIT;

		for (j = 0; j < frames; j++) {
			offset = (i + j * UFRAMES_PER_FRAME) % sch_ep->esit;

			ret = check_sch_tt_budget(sch_ep, offset);
			if (ret)
				continue;

			/* check hs bw domain */
			bw_max = get_max_bw(sch_bw, sch_ep, offset);
			if (bw_max > bw_boundary) {
				ret = -ESCH_BW_OVERFLOW;
				continue;
			}

			/* use best-fit between frames */
			fs_bw = get_fs_bw(sch_ep, offset);
			if (fs_bw < fs_bw_min) {
				fs_bw_min = fs_bw;
				offset_min = offset;
			}

			if (!fs_bw_min)
				break;
		}

		/* use first-fit between microframes in a frame */
		if (offset_min < XHCI_MTK_MAX_ESIT)
			break;
	}

	if (offset_min == XHCI_MTK_MAX_ESIT)
		return -ESCH_BW_OVERFLOW;

	sch_ep->offset = offset_min;

	return 0;
}

static void update_sch_tt(struct mu3h_sch_ep_info *sch_ep, bool used)
{
	struct mu3h_sch_tt *tt = sch_ep->sch_tt;
	u16 *fs_bus_bw;
	u32 base;
	int i, j, k, f;

	if (sch_ep->ep_type == ISOC_OUT_EP || sch_ep->ep_type == INT_OUT_EP)
		fs_bus_bw = tt->fs_bus_bw_out;
	else
		fs_bus_bw = tt->fs_bus_bw_in;

	for (i = 0; i < sch_ep->num_esit; i++) {
		base = sch_ep->offset + i * sch_ep->esit;

		for (j = 0; j < sch_ep->num_budget_microframes; j++) {
			k = XHCI_MTK_BW_INDEX(base + j);
			f = k / UFRAMES_PER_FRAME;
			if (used) {
				if (sch_ep->speed == USB_SPEED_LOW)
					tt->ls_bus_bw[k] += (u8)sch_ep->bw_budget_table[j];

				fs_bus_bw[k] += (u16)sch_ep->bw_budget_table[j];
				tt->fs_frame_bw[f] += (u16)sch_ep->bw_budget_table[j];
			} else {
				if (sch_ep->speed == USB_SPEED_LOW)
					tt->ls_bus_bw[k] -= (u8)sch_ep->bw_budget_table[j];

				fs_bus_bw[k] -= (u16)sch_ep->bw_budget_table[j];
				tt->fs_frame_bw[f] -= (u16)sch_ep->bw_budget_table[j];
			}
		}

		if (sch_ep->ep_type == ISOC_IN_EP || sch_ep->ep_type == INT_IN_EP) {
			k = XHCI_MTK_BW_INDEX(base);
			if (used)
				tt->in_ss_cnt[k]++;
			else
				tt->in_ss_cnt[k]--;
		}
	}

	if (used)
		list_add_tail(&sch_ep->tt_endpoint, &tt->ep_list);
	else
		list_del(&sch_ep->tt_endpoint);
}

static int load_ep_bw(struct mu3h_sch_bw_info *sch_bw,
		      struct mu3h_sch_ep_info *sch_ep, bool loaded)
{
	if (sch_ep->sch_tt)
		update_sch_tt(sch_ep, loaded);

	/* update bus bandwidth info */
	update_bus_bw(sch_bw, sch_ep, loaded);
	sch_ep->allocated = loaded;

	return 0;
}

/* allocate microframes for hs/ss/ssp */
static int alloc_sch_microframes(struct mu3h_sch_ep_info *sch_ep)
{
	struct mu3h_sch_bw_info *sch_bw = sch_ep->bw_info;
	const u32 bw_boundary = get_bw_boundary(sch_ep->speed);
	u32 offset;
	u32 worst_bw;
	u32 min_bw = ~0;
	int min_index = -1;

	/*
	 * Search through all possible schedule microframes.
	 * and find a microframe where its worst bandwidth is minimum.
	 */
	for (offset = 0; offset < sch_ep->esit; offset++) {

		worst_bw = get_max_bw(sch_bw, sch_ep, offset);
		if (worst_bw > bw_boundary)
			continue;

		if (min_bw > worst_bw) {
			min_bw = worst_bw;
			min_index = offset;
		}
	}

	if (min_index < 0)
		return -ESCH_BW_OVERFLOW;

	sch_ep->offset = min_index;

	return 0;
}

static int check_sch_bw(struct mu3h_sch_ep_info *sch_ep)
{
	int ret;

	if (sch_ep->sch_tt)
		ret = alloc_sch_portion_of_frame(sch_ep);
	else
		ret = alloc_sch_microframes(sch_ep);

	if (ret)
		return ret;

	return load_ep_bw(sch_ep->bw_info, sch_ep, true);
}

static void destroy_sch_ep(struct xhci_hcd_mtk *mtk, struct usb_device *udev,
			   struct mu3h_sch_ep_info *sch_ep)
{
	/* only release ep bw check passed by check_sch_bw() */
	if (sch_ep->allocated)
		load_ep_bw(sch_ep->bw_info, sch_ep, false);

	if (sch_ep->sch_tt)
		drop_tt(udev);

	list_del(&sch_ep->endpoint);
	hlist_del(&sch_ep->hentry);
	kfree(sch_ep);
}

static bool need_bw_sch(struct usb_device *udev,
			struct usb_host_endpoint *ep)
{
	bool has_tt = udev->tt && udev->tt->hub->parent;

	/* only for periodic endpoints */
	if (usb_endpoint_xfer_control(&ep->desc)
		|| usb_endpoint_xfer_bulk(&ep->desc))
		return false;

	/*
	 * for LS & FS periodic endpoints which its device is not behind
	 * a TT are also ignored, root-hub will schedule them directly,
	 * but need set @bpkts field of endpoint context to 1.
	 */
	if (is_fs_or_ls(udev->speed) && !has_tt)
		return false;

	/* skip endpoint with zero maxpkt */
	if (usb_endpoint_maxp(&ep->desc) == 0)
		return false;

	return true;
}

int xhci_mtk_sch_init(struct xhci_hcd_mtk *mtk)
{
	struct xhci_hcd *xhci = hcd_to_xhci(mtk->hcd);
	struct mu3h_sch_bw_info *sch_array;
	int num_usb_bus;

	/* ss IN and OUT are separated */
	num_usb_bus = xhci->usb3_rhub.num_ports * 2 + xhci->usb2_rhub.num_ports;

	sch_array = kcalloc(num_usb_bus, sizeof(*sch_array), GFP_KERNEL);
	if (sch_array == NULL)
		return -ENOMEM;

	mtk->sch_array = sch_array;

	INIT_LIST_HEAD(&mtk->bw_ep_chk_list);
	hash_init(mtk->sch_ep_hash);

	return 0;
}

void xhci_mtk_sch_exit(struct xhci_hcd_mtk *mtk)
{
	kfree(mtk->sch_array);
}

static int add_ep_quirk(struct usb_hcd *hcd, struct usb_device *udev,
			struct usb_host_endpoint *ep)
{
	struct xhci_hcd_mtk *mtk = hcd_to_mtk(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_virt_device *virt_dev;
	struct mu3h_sch_ep_info *sch_ep;
	unsigned int ep_index;

	virt_dev = xhci->devs[udev->slot_id];
	ep_index = xhci_get_endpoint_index(&ep->desc);
	ep_ctx = xhci_get_ep_ctx(xhci, virt_dev->in_ctx, ep_index);

	if (!need_bw_sch(udev, ep)) {
		/*
		 * set @bpkts to 1 if it is LS or FS periodic endpoint, and its
		 * device does not connected through an external HS hub
		 */
		if (usb_endpoint_xfer_int(&ep->desc)
			|| usb_endpoint_xfer_isoc(&ep->desc))
			ep_ctx->reserved[0] = cpu_to_le32(EP_BPKTS(1));

		return 0;
	}

	xhci_dbg(xhci, "%s %s\n", __func__, decode_ep(ep, udev->speed));

	sch_ep = create_sch_ep(mtk, udev, ep, ep_ctx);
	if (IS_ERR_OR_NULL(sch_ep))
		return -ENOMEM;

	setup_sch_info(ep_ctx, sch_ep);

	list_add_tail(&sch_ep->endpoint, &mtk->bw_ep_chk_list);
	hash_add(mtk->sch_ep_hash, &sch_ep->hentry, (unsigned long)ep);

	return 0;
}

static void drop_ep_quirk(struct usb_hcd *hcd, struct usb_device *udev,
			  struct usb_host_endpoint *ep)
{
	struct xhci_hcd_mtk *mtk = hcd_to_mtk(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	struct mu3h_sch_ep_info *sch_ep;
	struct hlist_node *hn;

	if (!need_bw_sch(udev, ep))
		return;

	xhci_dbg(xhci, "%s %s\n", __func__, decode_ep(ep, udev->speed));

	hash_for_each_possible_safe(mtk->sch_ep_hash, sch_ep,
				    hn, hentry, (unsigned long)ep) {
		if (sch_ep->ep == ep) {
			destroy_sch_ep(mtk, udev, sch_ep);
			break;
		}
	}
}

int xhci_mtk_check_bandwidth(struct usb_hcd *hcd, struct usb_device *udev)
{
	struct xhci_hcd_mtk *mtk = hcd_to_mtk(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	struct xhci_virt_device *virt_dev = xhci->devs[udev->slot_id];
	struct mu3h_sch_ep_info *sch_ep;
	int ret;

	xhci_dbg(xhci, "%s() udev %s\n", __func__, dev_name(&udev->dev));

	list_for_each_entry(sch_ep, &mtk->bw_ep_chk_list, endpoint) {
		struct xhci_ep_ctx *ep_ctx;
		struct usb_host_endpoint *ep = sch_ep->ep;
		unsigned int ep_index = xhci_get_endpoint_index(&ep->desc);

		ret = check_sch_bw(sch_ep);
		if (ret) {
			xhci_err(xhci, "Not enough bandwidth! (%s)\n",
				 sch_error_string(-ret));
			return -ENOSPC;
		}

		ep_ctx = xhci_get_ep_ctx(xhci, virt_dev->in_ctx, ep_index);
		ep_ctx->reserved[0] = cpu_to_le32(EP_BPKTS(sch_ep->pkts)
			| EP_BCSCOUNT(sch_ep->cs_count)
			| EP_BBM(sch_ep->burst_mode));
		ep_ctx->reserved[1] = cpu_to_le32(EP_BOFFSET(sch_ep->offset)
			| EP_BREPEAT(sch_ep->repeat));

		xhci_dbg(xhci, " PKTS:%x, CSCOUNT:%x, BM:%x, OFFSET:%x, REPEAT:%x\n",
			sch_ep->pkts, sch_ep->cs_count, sch_ep->burst_mode,
			sch_ep->offset, sch_ep->repeat);
	}

	ret = xhci_check_bandwidth(hcd, udev);
	if (!ret)
		list_del_init(&mtk->bw_ep_chk_list);

	return ret;
}

void xhci_mtk_reset_bandwidth(struct usb_hcd *hcd, struct usb_device *udev)
{
	struct xhci_hcd_mtk *mtk = hcd_to_mtk(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	struct mu3h_sch_ep_info *sch_ep, *tmp;

	xhci_dbg(xhci, "%s() udev %s\n", __func__, dev_name(&udev->dev));

	list_for_each_entry_safe(sch_ep, tmp, &mtk->bw_ep_chk_list, endpoint)
		destroy_sch_ep(mtk, udev, sch_ep);

	xhci_reset_bandwidth(hcd, udev);
}

int xhci_mtk_add_ep(struct usb_hcd *hcd, struct usb_device *udev,
		    struct usb_host_endpoint *ep)
{
	int ret;

	ret = xhci_add_endpoint(hcd, udev, ep);
	if (ret)
		return ret;

	if (ep->hcpriv)
		ret = add_ep_quirk(hcd, udev, ep);

	return ret;
}

int xhci_mtk_drop_ep(struct usb_hcd *hcd, struct usb_device *udev,
		     struct usb_host_endpoint *ep)
{
	int ret;

	ret = xhci_drop_endpoint(hcd, udev, ep);
	if (ret)
		return ret;

	/* needn't check @ep->hcpriv, xhci_endpoint_disable set it NULL */
	drop_ep_quirk(hcd, udev, ep);

	return 0;
}
