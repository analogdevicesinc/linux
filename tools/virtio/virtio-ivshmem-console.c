// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio console over uio_ivshmem back-end device
 *
 * Copyright (c) Siemens AG, 2019
 */

/*
 * HACK warnings:
 *  - little-endian hosts only
 *  - no proper input validation (specifically addresses)
 *  - may miss a couple of barriers
 *  - ignores a couple of mandatory properties, e.g. notification control
 *  - could implement some optional console features
 */

#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/virtio_console.h>
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

struct virtio_ivshmem_console {
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

	struct virtio_console_config config;
};

#define VI_REG_OFFSET(reg) \
	__builtin_offsetof(struct virtio_ivshmem_console, reg)

static struct ivshm_regs *regs;
static int uio_fd;
static struct virtio_ivshmem_console *vc;
static struct virtio_queue_config queue_config[2];
static int current_queue;
static struct vring vring[2];
static uint16_t next_idx[2];
static void *shmem;
static struct termios orig_termios;
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

static int process_rx_queue(void)
{
	struct vring_desc *desc;
	int idx, used_idx, ret;

	if (next_idx[0] == vring[0].avail->idx)
		return 0;

	idx = vring[0].avail->ring[next_idx[0] % vring[0].num];
	desc = &vring[0].desc[idx];

	ret = read(STDIN_FILENO, shmem + desc->addr, desc->len);
	if (ret <= 0)
		return 0;

	used_idx = vring[0].used->idx % vring[0].num;
	vring[0].used->ring[used_idx].id = idx;
	vring[0].used->ring[used_idx].len = ret;
	__sync_synchronize();
	vring[0].used->idx++;
	next_idx[0]++;

	vc->queue_event = 1;
	__sync_synchronize();
	mmio_write32(&regs->doorbell,
		     (peer_id << 16) | queue_config[0].driver_vector);

	return 1;
}

static int process_tx_queue(void)
{
	ssize_t written, remaining, res;
	struct vring_desc *desc;
	int idx, used_idx;

	if (next_idx[1] == vring[1].avail->idx)
		return 0;

	idx = vring[1].avail->ring[next_idx[1] % vring[1].num];
	desc = &vring[1].desc[idx];

	written = 0;
	remaining = desc->len;
	while (remaining > written) {
		res = write(STDOUT_FILENO, shmem + desc->addr + written,
			    remaining);
		if (res > 0) {
			written += res;
			remaining -= res;
		}
	}

	used_idx = vring[1].used->idx % vring[1].num;
	vring[1].used->ring[used_idx].id = idx;
	vring[1].used->ring[used_idx].len = 0;

	__sync_synchronize();
	vring[1].used->idx++;
	next_idx[1]++;

	return 1;
}

static int process_write_transaction(void)
{
	unsigned int new_queue;

	switch (vc->write_transaction) {
	case 0:
		return 0;
	case VI_REG_OFFSET(device_features_sel):
		printf("device_features_sel: %d\n", vc->device_features_sel);
		if (vc->device_features_sel == 1) {
			vc->device_features =
				(1 << (VIRTIO_F_VERSION_1 - 32)) |
				(1 << (VIRTIO_F_IOMMU_PLATFORM - 32)) |
				(1 << (VIRTIO_F_ORDER_PLATFORM - 32));
		} else {
			vc->device_features = 1 << VIRTIO_CONSOLE_F_SIZE;
		}
		break;
	case VI_REG_OFFSET(driver_features_sel):
		printf("driver_features_sel: %d\n", vc->driver_features_sel);
		break;
	case VI_REG_OFFSET(driver_features):
		printf("driver_features[%d]: 0x%x\n", vc->driver_features_sel,
		       vc->driver_features);
		break;
	case VI_REG_OFFSET(queue_sel):
		new_queue = vc->queue_sel;
		printf("queue_sel: %d\n", new_queue);
		if (new_queue > 1)
			break;

		if (current_queue >= 0)
			memcpy(&queue_config[current_queue], &vc->queue_config,
			    sizeof(struct virtio_queue_config));

		current_queue = new_queue;
		memcpy(&vc->queue_config, &queue_config[current_queue],
		       sizeof(struct virtio_queue_config));
		break;
	case VI_REG_OFFSET(queue_config.size):
		printf("queue size: %d\n", vc->queue_config.size);
		break;
	case VI_REG_OFFSET(queue_config.driver_vector):
		printf("queue driver vector: %d\n",
		       vc->queue_config.driver_vector);
		break;
	case VI_REG_OFFSET(queue_config.enable):
		printf("queue enable: %d\n", vc->queue_config.enable);
		if (current_queue >= 0 && vc->queue_config.enable) {
			memcpy(&queue_config[current_queue], &vc->queue_config,
			    sizeof(struct virtio_queue_config));
			vring[current_queue].num = vc->queue_config.size;
			vring[current_queue].desc =
				shmem + vc->queue_config.desc;
			vring[current_queue].avail =
				shmem + vc->queue_config.driver;
			vring[current_queue].used =
				shmem + vc->queue_config.device;
			next_idx[current_queue] = 0;
		}
		break;
	case VI_REG_OFFSET(queue_config.desc):
		printf("queue desc: 0x%llx\n",
		       (unsigned long long)vc->queue_config.desc);
		break;
	case VI_REG_OFFSET(queue_config.driver):
		printf("queue driver: 0x%llx\n",
		       (unsigned long long)vc->queue_config.driver);
		break;
	case VI_REG_OFFSET(queue_config.device):
		printf("queue device: 0x%llx\n",
		       (unsigned long long)vc->queue_config.device);
		break;
	case VI_REG_OFFSET(device_status):
		printf("device_status: 0x%x\n", vc->device_status);
		if (vc->device_status == 0xf) {
			vc->config_event = 1;
			__sync_synchronize();
			mmio_write32(&regs->doorbell, peer_id << 16);
		}
		break;
	default:
		printf("unknown write transaction for %x\n",
		       vc->write_transaction);
		break;
	}

	__sync_synchronize();
	vc->write_transaction = 0;

	return 1;
}

static void restore_stdin(void)
{
	tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

int main(int argc, char *argv[])
{
	int pagesize = getpagesize();
	unsigned long long shmem_sz;
	volatile uint32_t *state;
	struct pollfd pollfd[2];
	int event, size_fd, ret;
	struct termios termios;
	struct winsize winsize;
	char sysfs_path[64];
	char size_str[64];
	char *uio_devname;

	if (argc < 2) {
		fprintf(stderr, "usage: %s UIO-DEVICE\n", argv[0]);
		return 1;
	}

	pollfd[0].fd = STDIN_FILENO;
	pollfd[0].events = POLLIN;

	ret = fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
	if (ret)
		error(1, errno, "fcntl failed");

	ret = tcgetattr(STDIN_FILENO, &orig_termios);
	if (ret)
		error(1, errno, "tcgetattr failed");
	atexit(restore_stdin);
	termios = orig_termios;
	termios.c_iflag &= ~(ICRNL | IXON);
	termios.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
	ret = tcsetattr(STDIN_FILENO, TCSAFLUSH, &termios);
	if (ret)
		error(1, errno, "tcsetattr failed");

	ret = ioctl(STDOUT_FILENO, TIOCGWINSZ, &winsize);
	if (ret)
		error(1, errno, "TIOCGWINSZ failed");

	uio_fd = open(argv[1], O_RDWR);
	if (uio_fd < 0)
		error(1, errno, "cannot open %s", argv[1]);

	pollfd[1].fd = uio_fd;
	pollfd[1].events = POLLIN;

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

		vc = shmem;
		memset(vc, 0, sizeof(*vc));
		vc->revision = 1;
		vc->size = sizeof(*vc);

		memset(queue_config, 0, sizeof(queue_config));
		queue_config[0].size = 8;
		queue_config[0].device_vector = 0;
		queue_config[1].size = 8;
		queue_config[1].device_vector = 0;
		current_queue = -1;

		vc->config.cols = winsize.ws_col;
		vc->config.rows = winsize.ws_row;

		mmio_write32(&regs->state, VIRTIO_STATE_READY);
		while (state[peer_id] != VIRTIO_STATE_READY) {
			printf("Waiting for peer to be ready...\n");
			wait_for_interrupt(regs);
		}

		printf("Starting virtio device\n");

		while (state[peer_id] == VIRTIO_STATE_READY) {
			event = process_write_transaction();

			if (vc->device_status == 0xf) {
				event |= process_rx_queue();
				event |= process_tx_queue();
			}

			if (!event) {
				ret = poll(pollfd, 2, -1);
				if (ret < 0)
					error(1, errno, "poll failed");
				if (pollfd[1].revents & POLLIN)
					wait_for_interrupt(regs);
			}
		}
	}
}
