// SPDX-License-Identifier: GPL-2.0+
/*
 * ASUS Aura RGB over SCSI for ROG external NVMe enclosures
 * (e.g. ROG STRIX Arion, USB 0b05:1932).
 *
 * USB mass-storage device, no HID; the ENE LED controller is driven via
 * vendor SCSI commands on the disk's LUN. Matched by INQUIRY (vendor
 * "ROG", model "ESD-S1C"); the sdev is not claimed, sd keeps owning the
 * disk. Attach manually until the SCSI side agrees on an in-tree hook:
 * echo asus_aura > /sys/block/sdX/device/dh_state
 *
 * Each of the 4 LEDs is a multicolor LED class device,
 * /sys/class/leds/asus-arion-<H-C-T-L>::led-0..led-3. A colour change
 * writes only that LED's slots in the ENE colour tables, then applies
 * without saving (runtime state, no flash wear); MODE must precede
 * every sequence or the device ignores it. The register layout is
 * described by struct ene_cdb.
 *
 * brightness_set (LED core fast path) only caches the colour and marks
 * the LED pending under a spinlock; a single work item per enclosure
 * snapshots all pending LEDs and runs one ENE sequence for them. One
 * work item also serializes the sequences against each other: the
 * MODE/colour/APPLY chain must never interleave between
 * concurrent LED updates.
 */

#include <linux/array_size.h>
#include <linux/blk-mq.h>
#include <linux/blk_types.h>
#include <linux/blkdev.h>
#include <linux/build_bug.h>
#include <linux/byteorder/generic.h>
#include <linux/cache.h>
#include <linux/compiler.h>
#include <linux/compiler_attributes.h>
#include <linux/container_of.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/led-class-multicolor.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sprintf.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_dh.h>

#define ARION_INQ_VENDOR	"ROG"
#define ARION_INQ_MODEL		"ESD-S1C"

/* The colour table has 16 slots; 4 of them drive LEDs. */
#define ARION_NUM_LEDS		4

#define ENE_OPCODE		0xec

/* ENE register map (BE16 on the wire). */
#define ENE_REG_MODE		0x8021	/* 1 = Static selects the colour tables */
#define ENE_REG_APPLY		0x80a0	/* 0x01 = apply, 0xaa = apply and save */
#define ENE_REG_COLORS_EFFECT	0x8160	/* colour table, 3 bytes per LED */
#define ENE_REG_COLORS_DIRECT	0x8100	/* second table some firmware uses */

#define ENE_MODE_STATIC		1
#define ENE_APPLY		0x01
#define ENE_TIMEOUT		(5 * HZ)

/* The colour tables are R, B, G on the wire; the subleds are R, G, B. */
#define ENE_WIRE_R		0
#define ENE_WIRE_B		1
#define ENE_WIRE_G		2
#define ENE_RGB_LEN		3

/*
 * Vendor CDB: opcode, 'A''S' signature, register, argument count in
 * byte 13. scsi_execute_cmd() cannot be used for it, see the commit
 * message.
 */
struct ene_cdb {
	u8		opcode;
	u8		sig[2];
	__be16		reg;
	u8		reserved[8];
	u8		arg_count;
	u8		pad[2];
} __packed;

static_assert(sizeof(struct ene_cdb) == 16);

struct asus_aura_led {
	struct led_classdev_mc	mc_cdev;
	struct mc_subled	subled[ENE_RGB_LEN];

	/* Cached colour, consumed by the work item. */
	u8			rgb[ENE_RGB_LEN];
	bool			pending;

	struct asus_aura	*aura;
};

struct asus_aura {
	struct scsi_device	*sdev;
	struct asus_aura_led	leds[ARION_NUM_LEDS];

	/* Serializes the cached colours and pending flags. */
	spinlock_t		lock;
	struct work_struct	work;

	/*
	 * DMA-safe staging buffer, the only memory ever mapped for DMA.
	 * Cacheline-aligned so that invalidation on non-coherent
	 * architectures cannot clobber the members above.
	 */
	u8			tx[ENE_RGB_LEN] ____cacheline_aligned;
};

static int ene_write_reg(struct asus_aura *aura, u16 reg,
			 const u8 *tx, u8 len)
{
	struct scsi_device *sdev = aura->sdev;
	struct request *rq;
	struct scsi_cmnd *scmd;
	struct ene_cdb cdb = {
		.opcode		= ENE_OPCODE,
		.sig		= { 'A', 'S' },
		.reg		= cpu_to_be16(reg),
		.arg_count	= len,
	};
	int ret;

	rq = scsi_alloc_request(sdev->request_queue, REQ_OP_DRV_OUT, 0);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	/*
	 * Stack memory is not DMA-safe (VMAP_STACK), so the payload is
	 * copied into the per-device buffer first.
	 */
	memcpy(aura->tx, tx, len);
	ret = blk_rq_map_kern(rq, aura->tx, len, GFP_NOIO);
	if (ret)
		goto out;

	scmd = blk_mq_rq_to_pdu(rq);
	scmd->cmd_len = sizeof(cdb);
	memcpy(scmd->cmnd, &cdb, sizeof(cdb));
	scmd->allowed = 1;
	rq->timeout = ENE_TIMEOUT;
	rq->rq_flags |= RQF_QUIET;

	blk_execute_rq(rq, true);

	/* GOOD status is zero in the fields we care about. */
	ret = scmd->result ? -EIO : 0;
out:
	blk_mq_free_request(rq);
	return ret;
}

static void asus_aura_work(struct work_struct *work)
{
	struct asus_aura *aura = container_of(work, struct asus_aura, work);
	u8 rgb[ARION_NUM_LEDS][ENE_RGB_LEN];
	u8 mode = ENE_MODE_STATIC;
	u8 apply = ENE_APPLY;
	bool pending[ARION_NUM_LEDS];
	bool any = false;
	unsigned long flags;
	int ret;

	/*
	 * Fast path without the lock: if nothing is pending, return
	 * before touching the device. A colour arriving right after
	 * the check re-queues this work, so nothing is lost.
	 */
	for (int i = 0; i < ARRAY_SIZE(aura->leds); i++) {
		if (READ_ONCE(aura->leds[i].pending)) {
			any = true;
			break;
		}
	}
	if (!any)
		return;

	/* Snapshot the cached colours and clear the pending flags. */
	spin_lock_irqsave(&aura->lock, flags);
	for (int i = 0; i < ARRAY_SIZE(aura->leds); i++) {
		pending[i] = aura->leds[i].pending;
		aura->leds[i].pending = false;
		memcpy(rgb[i], aura->leds[i].rgb, ENE_RGB_LEN);
	}
	spin_unlock_irqrestore(&aura->lock, flags);

	if (!scsi_device_online(aura->sdev))
		return;

	/* Mode first: without it the device ignores the sequence. */
	ret = ene_write_reg(aura, ENE_REG_MODE, &mode, sizeof(mode));
	if (ret)
		goto err;

	for (int i = 0; i < ARRAY_SIZE(aura->leds); i++) {
		if (!pending[i])
			continue;

		/*
		 * Both colour tables are written: firmware revisions
		 * pull from either of them.
		 */
		ret = ene_write_reg(aura,
				    ENE_REG_COLORS_EFFECT + i * ENE_RGB_LEN,
				    rgb[i], sizeof(rgb[i]));
		if (ret)
			goto err;

		ret = ene_write_reg(aura,
				    ENE_REG_COLORS_DIRECT + i * ENE_RGB_LEN,
				    rgb[i], sizeof(rgb[i]));
		if (ret)
			goto err;
	}

	/*
	 * Apply without save: the LED class interface is runtime state
	 * and does not promise persistence across power cycles, and a
	 * save on every update would wear the controller flash.
	 */
	ret = ene_write_reg(aura, ENE_REG_APPLY, &apply, sizeof(apply));
	if (ret)
		goto err;

	return;

err:
	/*
	 * This runs detached from the LED core, which has no error
	 * channel for brightness_set. The write is dropped and logged
	 * ratelimited; the LED keeps the last successfully applied
	 * colour until the next user update.
	 */
	dev_err_ratelimited(&aura->sdev->sdev_gendev,
			    "asus_aura: colour update failed: %d\n", ret);
}

/* Non-blocking LED callback (LED core fast path). Cache colour, defer SCSI. */
static void asus_aura_set(struct led_classdev *cdev,
			  enum led_brightness brightness)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(cdev);
	struct asus_aura_led *led = container_of(mc, struct asus_aura_led, mc_cdev);
	struct asus_aura *aura = led->aura;
	unsigned long flags;

	/*
	 * led_mc_calc_color_components() writes the shared subled_info
	 * array. Sysfs stores are serialized by the LED core, but
	 * trigger events call led_set_brightness() without that lock,
	 * so compute and cache under the aura lock.
	 */
	spin_lock_irqsave(&aura->lock, flags);

	led_mc_calc_color_components(mc, brightness);

	led->rgb[ENE_WIRE_R] = led->subled[0].brightness;
	led->rgb[ENE_WIRE_B] = led->subled[2].brightness;
	led->rgb[ENE_WIRE_G] = led->subled[1].brightness;
	led->pending = true;

	spin_unlock_irqrestore(&aura->lock, flags);

	schedule_work(&aura->work);
}

static int asus_aura_register_led(struct asus_aura *aura, int index)
{
	struct asus_aura_led *led = &aura->leds[index];
	struct led_classdev *cdev = &led->mc_cdev.led_cdev;
	char hctl[32];
	int ret;

	led->aura = aura;

	led->subled[0].color_index = LED_COLOR_ID_RED;
	led->subled[1].color_index = LED_COLOR_ID_GREEN;
	led->subled[2].color_index = LED_COLOR_ID_BLUE;
	led->mc_cdev.num_colors = ENE_RGB_LEN;
	led->mc_cdev.subled_info = led->subled;

	/*
	 * The sdev's H:C:T:L keeps the names unique when more than one
	 * enclosure is connected; with a static name the LED core would
	 * register a second enclosure's LEDs under renamed nodes
	 * (asus-arion::led-0_1), the wrong device identity. The colons
	 * are flattened to dashes, the color section stays empty
	 * (multicolor, palette via multi_intensity) and the four
	 * identical zones take a "-N" ordinal, as
	 * Documentation/leds/leds-class.rst asks for.
	 */
	strscpy(hctl, dev_name(&aura->sdev->sdev_gendev), sizeof(hctl));
	strreplace(hctl, ':', '-');
	cdev->name = kasprintf(GFP_KERNEL, "asus-arion-%s::led-%d", hctl, index);
	if (!cdev->name)
		return -ENOMEM;

	cdev->max_brightness = 255;
	cdev->brightness_set = asus_aura_set;

	led_mc_calc_color_components(&led->mc_cdev, cdev->brightness);

	/*
	 * NULL parent: a device reference on the sdev would block its
	 * final release on unplug, which is what calls detach() to
	 * unregister the LEDs. The cycle leaked nodes and the module
	 * refcount on every hot-unplug.
	 */
	ret = led_classdev_multicolor_register(NULL, &led->mc_cdev);
	if (ret)
		kfree(cdev->name);
	return ret;
}

/*
 * Unregister the LEDs before cancelling the work: unregistering removes
 * the sysfs attributes, so no new brightness_set can schedule the work
 * afterwards. Cancelling first would leave a window where a brightness
 * write requeues the work after cancel_work_sync() returned.
 */
static void asus_aura_release(struct asus_aura *aura, int num_leds)
{
	for (int i = 0; i < num_leds; i++) {
		led_classdev_multicolor_unregister(&aura->leds[i].mc_cdev);
		kfree(aura->leds[i].mc_cdev.led_cdev.name);
	}

	cancel_work_sync(&aura->work);
	kfree(aura);
}

static int asus_aura_attach(struct scsi_device *sdev)
{
	struct asus_aura *aura;
	int ret;

	if (strncmp(sdev->vendor, ARION_INQ_VENDOR, strlen(ARION_INQ_VENDOR)) ||
	    strncmp(sdev->model, ARION_INQ_MODEL, strlen(ARION_INQ_MODEL)))
		return SCSI_DH_DEV_UNSUPP;

	aura = kzalloc_obj(*aura, GFP_KERNEL);
	if (!aura)
		return SCSI_DH_NOMEM;

	aura->sdev = sdev;
	spin_lock_init(&aura->lock);
	INIT_WORK(&aura->work, asus_aura_work);

	for (int i = 0; i < ARRAY_SIZE(aura->leds); i++) {
		ret = asus_aura_register_led(aura, i);
		if (ret) {
			asus_aura_release(aura, i);
			return SCSI_DH_NOMEM;
		}
	}

	sdev->handler_data = aura;
	return SCSI_DH_OK;
}

static void asus_aura_detach(struct scsi_device *sdev)
{
	struct asus_aura *aura = sdev->handler_data;

	if (!aura)
		return;

	asus_aura_release(aura, ARION_NUM_LEDS);
	sdev->handler_data = NULL;
}

static struct scsi_device_handler asus_aura_dh = {
	.name	= "asus_aura",
	.module	= THIS_MODULE,
	.attach	= asus_aura_attach,
	.detach	= asus_aura_detach,
};

static int __init asus_aura_init(void)
{
	return scsi_register_device_handler(&asus_aura_dh);
}

static void __exit asus_aura_exit(void)
{
	scsi_unregister_device_handler(&asus_aura_dh);
}

module_init(asus_aura_init);
module_exit(asus_aura_exit);

MODULE_DESCRIPTION("ASUS Aura RGB over SCSI for ROG NVMe enclosures");
MODULE_AUTHOR("Liang Haowen");
MODULE_LICENSE("GPL");
