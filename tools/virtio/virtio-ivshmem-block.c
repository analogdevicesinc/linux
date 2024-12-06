// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio block over uio_ivshmem back-end device
 *
 * Copyright (c) Siemens AG, 2019
 */

/*
 * HACK warnings:
 *  - little-endian hosts only
 *  - no proper input validation (specifically addresses)
 *  - may miss a couple of barriers
 *  - ignores a couple of mandatory properties, e.g. notification control
 *  - could implement some optional block features
 *  - might eat your data
 */

#include <assert.h>
#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <linux/virtio_blk.h>
#include <linux/virtio_ring.h>

#ifndef VIRTIO_F_ORDER_PLATFORM
#define VIRTIO_F_ORDER_PLATFORM		36
#endif

struct ivshm_regs {
	uint32_t id;
	uint32_t max_peers;
	uint32_t int_control;
	uint32_t doorbell;
	uint32_t state;
};

#define VIRTIO_STATE_RESET		0
#define VIRTIO_STATE_READY		1

struct virtio_queue_config {
	uint16_t size;
	uint16_t device_vector;
	uint16_t driver_vector;
	uint16_t enable;
	uint64_t desc;
	uint64_t driver;
	uint64_t device;
};

struct virtio_ivshmem_block {
	uint32_t revision;
	uint32_t size;

	uint32_t write_transaction;

	uint32_t device_features;
	uint32_t device_features_sel;
	uint32_t driver_features;
	uint32_t driver_features_sel;

	uint32_t queue_sel;
	struct virtio_queue_config queue_config;

	uint8_t config_event;
	uint8_t queue_event;
	uint8_t __reserved[2];
	uint32_t device_status;

	uint32_t config_generation;

	struct virtio_blk_config config;
};

#define VI_REG_OFFSET(reg) \
	__builtin_offsetof(struct virtio_ivshmem_block, reg)

static struct ivshm_regs *regs;
static int uio_fd, image_fd;
static struct stat image_stat;
static struct virtio_ivshmem_block *vb;
static struct vring vring;
static uint16_t next_idx;
static void *shmem;
static uint32_t peer_id;

static inline uint32_t mmio_read32(void *address)
{
	return *(volatile uint32_t *)address;
}

static inline void mmio_write32(void *address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static void wait_for_interrupt(struct ivshm_regs *regs)
{
	uint32_t dummy;

	if (read(uio_fd, &dummy, 4) < 0)
		error(1, errno, "UIO read failed");
	mmio_write32(&regs->int_control, 1);
}

static int process_queue(void)
{
	struct virtio_blk_outhdr *req;
	struct vring_desc *desc;
	int idx, used_idx, ret;
	size_t size, len;
	uint8_t status;

	if (next_idx == vring.avail->idx)
		return 0;

	idx = vring.avail->ring[next_idx % vring.num];

	desc = &vring.desc[idx];
	assert(desc->len == sizeof(*req));
	assert(desc->flags & 1);
	req = shmem + desc->addr;

	len = 1;

	switch (req->type) {
	case VIRTIO_BLK_T_IN:
		desc = &vring.desc[desc->next];
		assert(desc->flags & 1);
		size = desc->len;
		ret = pread(image_fd, shmem + desc->addr, size,
			    req->sector * 512);
		if (ret == size) {
			status = VIRTIO_BLK_S_OK;
			len += size;
		} else {
			status = VIRTIO_BLK_S_IOERR;
		}
		break;
	case VIRTIO_BLK_T_OUT:
		desc = &vring.desc[desc->next];
		assert(desc->flags & 1);
		size = desc->len;
		ret = pwrite(image_fd, shmem + desc->addr, size,
			     req->sector * 512);
		status = ret == size ? VIRTIO_BLK_S_OK : VIRTIO_BLK_S_IOERR;
		break;
	case VIRTIO_BLK_T_FLUSH:
		ret = fsync(image_fd);
		status = ret == 0 ? VIRTIO_BLK_S_OK : VIRTIO_BLK_S_IOERR;
		break;
	case VIRTIO_BLK_T_GET_ID:
		desc = &vring.desc[desc->next];
		assert(desc->flags & 1);
		len = desc->len > 0 ? 1 : 0;
		memset(shmem + desc->addr, 0, len);
		status = VIRTIO_BLK_S_OK;
		break;
	default:
		printf("unknown request %d\n", req->type);
		status = VIRTIO_BLK_S_UNSUPP;
		break;
	}

	desc = &vring.desc[desc->next];
	assert(!(desc->flags & 1));

	*(uint8_t *)(shmem + desc->addr) = status;

	used_idx = vring.used->idx % vring.num;
	vring.used->ring[used_idx].id = idx;
	vring.used->ring[used_idx].len = len;

	__sync_synchronize();
	vring.used->idx++;
	next_idx++;

	vb->queue_event = 1;
	__sync_synchronize();
	mmio_write32(&regs->doorbell,
		     (peer_id << 16) | vb->queue_config.driver_vector);

	return 1;
}

static int process_write_transaction(void)
{
	switch (vb->write_transaction) {
	case 0:
		return 0;
	case VI_REG_OFFSET(device_features_sel):
		printf("device_features_sel: %d\n", vb->device_features_sel);
		if (vb->device_features_sel == 1) {
			vb->device_features =
				(1 << (VIRTIO_F_VERSION_1 - 32)) |
				(1 << (VIRTIO_F_IOMMU_PLATFORM - 32)) |
				(1 << (VIRTIO_F_ORDER_PLATFORM - 32));
		} else {
			vb->device_features =
				(1 << VIRTIO_BLK_F_SIZE_MAX) |
				(1 << VIRTIO_BLK_F_SEG_MAX) |
				(1 << VIRTIO_BLK_F_FLUSH);
		}
		break;
	case VI_REG_OFFSET(driver_features_sel):
		printf("driver_features_sel: %d\n", vb->driver_features_sel);
		break;
	case VI_REG_OFFSET(driver_features):
		printf("driver_features[%d]: 0x%x\n", vb->driver_features_sel,
		       vb->driver_features);
		break;
	case VI_REG_OFFSET(queue_sel):
		printf("queue_sel: %d\n", vb->queue_sel);
		break;
	case VI_REG_OFFSET(queue_config.size):
		printf("queue size: %d\n", vb->queue_config.size);
		break;
	case VI_REG_OFFSET(queue_config.driver_vector):
		printf("queue driver vector: %d\n",
		       vb->queue_config.driver_vector);
		break;
	case VI_REG_OFFSET(queue_config.enable):
		printf("queue enable: %d\n", vb->queue_config.enable);
		if (vb->queue_config.enable) {
			vring.num = vb->queue_config.size;
			vring.desc = shmem + vb->queue_config.desc;
			vring.avail = shmem + vb->queue_config.driver;
			vring.used = shmem + vb->queue_config.device;
			next_idx = 0;
		}
		break;
	case VI_REG_OFFSET(queue_config.desc):
		printf("queue desc: 0x%llx\n",
		       (unsigned long long)vb->queue_config.desc);
		break;
	case VI_REG_OFFSET(queue_config.driver):
		printf("queue driver: 0x%llx\n",
		       (unsigned long long)vb->queue_config.driver);
		break;
	case VI_REG_OFFSET(queue_config.device):
		printf("queue device: 0x%llx\n",
		       (unsigned long long)vb->queue_config.device);
		break;
	case VI_REG_OFFSET(device_status):
		printf("device_status: 0x%x\n", vb->device_status);
		break;
	default:
		printf("unknown write transaction for %x\n",
		       vb->write_transaction);
		break;
	}

	__sync_synchronize();
	vb->write_transaction = 0;

	return 1;
}

int main(int argc, char *argv[])
{
	int pagesize = getpagesize();
	unsigned long long shmem_sz;
	volatile uint32_t *state;
	int event, size_fd, ret;
	char sysfs_path[64];
	char size_str[64];
	char *uio_devname;

	if (argc < 3) {
		fprintf(stderr, "usage: %s UIO-DEVICE IMAGE\n", argv[0]);
		return 1;
	}

	image_fd = open(argv[2], O_RDWR);
	if (image_fd < 0)
		error(1, errno, "cannot open %s", argv[2]);

	ret = fstat(image_fd, &image_stat);
	if (ret < 0)
		error(1, errno, "fstat failed");

	uio_fd = open(argv[1], O_RDWR);
	if (uio_fd < 0)
		error(1, errno, "cannot open %s", argv[1]);

	regs = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, uio_fd, 0);
	if (!regs)
		error(1, errno, "mmap of registers failed");
	state = mmap(NULL, 4096, PROT_READ, MAP_SHARED, uio_fd, pagesize);
	if (!state)
		error(1, errno, "mmap of state table failed");

	uio_devname = strstr(argv[1], "/uio");
	snprintf(sysfs_path, sizeof(sysfs_path),
		 "/sys/class/uio%s/maps/map2/size",
		 uio_devname);
	size_fd = open(sysfs_path, O_RDONLY);
	if (size_fd < 0)
		error(1, errno, "cannot open %s", sysfs_path);
	if (read(size_fd, size_str, sizeof(size_str)) < 0)
		error(1, errno, "read from %s failed", sysfs_path);
	shmem_sz = strtoll(size_str, NULL, 16);

	shmem = mmap(NULL, shmem_sz, PROT_READ | PROT_WRITE, MAP_SHARED,
		     uio_fd, 2 * pagesize);
	if (!shmem)
		error(1, errno, "mmap of shared memory failed");

	peer_id = !mmio_read32(&regs->id);

	mmio_write32(&regs->int_control, 1);

	while (1) {
		mmio_write32(&regs->state, VIRTIO_STATE_RESET);
		while (state[peer_id] != VIRTIO_STATE_RESET) {
			printf("Waiting for peer to reset...\n");
			wait_for_interrupt(regs);
		}

		vb = shmem;
		memset(vb, 0, sizeof(*vb));
		vb->revision = 1;
		vb->size = sizeof(*vb);

		memset(&vb->queue_config, 0, sizeof(vb->queue_config));
		vb->queue_config.size = 8;
		vb->queue_config.device_vector = 1;

		vb->config.capacity = image_stat.st_size / 512;
		vb->config.size_max = (shmem_sz / 8) & ~(pagesize - 1);
		vb->config.seg_max = 1;

		mmio_write32(&regs->state, VIRTIO_STATE_READY);
		while (state[peer_id] != VIRTIO_STATE_READY) {
			printf("Waiting for peer to be ready...\n");
			wait_for_interrupt(regs);
		}

		printf("Starting virtio device\n");

		while (state[peer_id] == VIRTIO_STATE_READY) {
			event = process_write_transaction();

			if (vb->device_status == 0xf)
				event |= process_queue();

			if (!event)
				wait_for_interrupt(regs);
		}
	}
}
