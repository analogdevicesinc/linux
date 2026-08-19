// SPDX-License-Identifier: GPL-2.0

#include <assert.h>
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <link.h>
#include <linux/coredump.h>
#include <linux/fs.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../filesystems/wrappers.h"

#include "coredump_test_helpers.h"

#if __ELF_NATIVE_CLASS == 64
#define COREDUMP_ELFCLASS ELFCLASS64
#else
#define COREDUMP_ELFCLASS ELFCLASS32
#endif

void *do_nothing(void *arg)
{
	(void)arg;
	while (1)
		pause();

	return NULL;
}

void crashing_child(void)
{
	pthread_t thread;
	int i;

	for (i = 0; i < NUM_THREAD_SPAWN; ++i)
		pthread_create(&thread, NULL, do_nothing, NULL);

	/* crash on purpose */
	i = *(volatile int *)NULL;
}

void crashing_child_sparse(size_t size)
{
	char *p;

	/*
	 * Touch the first page only. The whole mapping is dumped because
	 * it has been written to, but all of it save that one page is a
	 * hole.
	 */
	p = mmap(NULL, size, PROT_READ | PROT_WRITE,
		 MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
	if (p != MAP_FAILED)
		p[0] = 'x';

	/* crash on purpose */
	*(volatile int *)NULL = 0;
}

/* Sink a reassembled record stream is handed to, record by record. */
struct coredump_record_sink {
	/* @len bytes of coredump data that belong at @offset. */
	int (*data)(void *ctx, const void *buf, size_t len, __u64 offset);
	/* @len zero bytes that belong at @offset. */
	int (*zero)(void *ctx, __u64 offset, __u64 len);
	void *ctx;
};

/* Read @len bytes off the socket and hand them to @sink, if there is one. */
static ssize_t recv_record_bytes(int fd_coredump, __u64 len,
				 const struct coredump_record_sink *sink,
				 __u64 offset)
{
	ssize_t received = 0;

	while (len) {
		char buffer[PAGE_SIZE];
		size_t chunk = len < sizeof(buffer) ? len : sizeof(buffer);
		ssize_t ret;

		ret = recv(fd_coredump, buffer, chunk, MSG_WAITALL);
		if (ret <= 0) {
			fprintf(stderr, "%s: short read %zd: %m\n",
				__func__, ret);
			return -1;
		}

		if (sink && sink->data(sink->ctx, buffer, ret, offset + received))
			return -1;

		received += ret;
		len -= ret;
	}

	return received;
}

/* Put the data where the records say it goes and leave the holes alone. */
static int file_sink_data(void *ctx, const void *buf, size_t len, __u64 offset)
{
	int fd = *(int *)ctx;

	if (pwrite(fd, buf, len, offset) != (ssize_t)len) {
		fprintf(stderr, "%s: pwrite failed: %m\n", __func__);
		return -1;
	}

	return 0;
}

static int file_sink_zero(void *ctx, __u64 offset, __u64 len)
{
	/* Nothing has to be written for a hole. */
	return 0;
}

/*
 * Read a coredump strea and funnel it into @sink. Allow to pass in a
 * @fd_peer_pidfd to simulate coredump truncation by killing it after having
 * received a coredump record.
 */
static ssize_t __recv_coredump_records(int fd_coredump,
				       const struct coredump_record_sink *sink,
				       off_t *coredump_size, bool *truncated,
				       int fd_peer_pidfd)
{
	ssize_t received = 0;
	off_t size = 0;
	bool is_truncated = false;
	bool ended = false;
	char trailing;

	while (!ended) {
		struct coredump_record_header record = {};
		size_t known_size;
		ssize_t ret;

		/* Peek the header size the way read_coredump_req() does. */
		ret = recv(fd_coredump, &record, sizeof(record.size),
			   MSG_PEEK | MSG_WAITALL);
		if (ret == 0) {
			/* Nothing closed the stream, so the coredump was cut short. */
			if (truncated) {
				is_truncated = true;
				break;
			}
			fprintf(stderr, "%s: stream ended without an end record\n",
				__func__);
			return -1;
		}
		if (ret != sizeof(record.size)) {
			fprintf(stderr, "%s: short record peek %zd: %m\n",
				__func__, ret);
			return -1;
		}

		if (record.size < COREDUMP_RECORD_HEADER_SIZE_VER0) {
			fprintf(stderr, "%s: header size %u below minimum %u\n",
				__func__, record.size,
				COREDUMP_RECORD_HEADER_SIZE_VER0);
			return -1;
		}

		/* Consume as much of the header as we know about. */
		known_size = record.size < sizeof(record) ? record.size : sizeof(record);
		ret = recv(fd_coredump, &record, known_size, MSG_WAITALL);
		if (ret != (ssize_t)known_size) {
			fprintf(stderr, "%s: short record read %zd: %m\n",
				__func__, ret);
			return -1;
		}
		received += ret;

		/*
		 * A flag changes what the record means, so refuse one we
		 * don't know rather than guess.
		 */
		if (record.flags) {
			fprintf(stderr, "%s: unknown header flags 0x%llx\n",
				__func__, (unsigned long long)record.flags);
			return -1;
		}

		/* Discard any part of the header we have no use for. */
		ret = recv_record_bytes(fd_coredump, record.size - known_size,
					NULL, 0);
		if (ret < 0)
			return -1;
		received += ret;

		/* Records are sent in order and they don't leave gaps. */
		if (record.offset != (__u64)size) {
			fprintf(stderr, "%s: record at %llu, expected %llu\n",
				__func__, (unsigned long long)record.offset,
				(unsigned long long)size);
			return -1;
		}

		switch (record.type) {
		case COREDUMP_RECORD_ZERO:
			/* A hole. It comes with no data and needs none. */
			if (sink->zero(sink->ctx, record.offset, record.len))
				return -1;
			break;
		case COREDUMP_RECORD_DATA:
			ret = recv_record_bytes(fd_coredump, record.len, sink,
						record.offset);
			if (ret < 0)
				return -1;
			received += ret;
			if (fd_peer_pidfd >= 0) {
				if (sys_pidfd_send_signal(fd_peer_pidfd, SIGKILL,
							  NULL, 0)) {
					fprintf(stderr, "%s: kill failed: %m\n",
						__func__);
					return -1;
				}
				fd_peer_pidfd = -1;
			}
			break;
		case COREDUMP_RECORD_END:
			/* The coredump ends here and nothing follows it. */
			if (record.len) {
				fprintf(stderr, "%s: end record covers %llu bytes\n",
					__func__,
					(unsigned long long)record.len);
				return -1;
			}
			ended = true;
			break;
		default:
			fprintf(stderr, "%s: unknown record type %u\n",
				__func__, record.type);
			return -1;
		}

		size += record.len;
	}

	/* The end record is the last thing on the wire. */
	if (recv(fd_coredump, &trailing, sizeof(trailing), MSG_DONTWAIT) > 0) {
		fprintf(stderr, "%s: data after the end record\n", __func__);
		return -1;
	}

	if (truncated)
		*truncated = is_truncated;

	*coredump_size = size;

	fprintf(stderr, "Received %zd bytes for a %s coredump of %llu bytes\n",
		received, is_truncated ? "truncated" : "complete",
		(unsigned long long)size);
	return received;
}

/* Reassemble a record stream into the coredump it describes. */
ssize_t recv_coredump_records(int fd_coredump, int fd_core_file,
			      off_t *coredump_size, bool *truncated,
			      int fd_peer_pidfd)
{
	struct coredump_record_sink sink = {
		.data	= file_sink_data,
		.zero	= file_sink_zero,
		.ctx	= &fd_core_file,
	};
	ssize_t received;
	off_t size = 0;

	received = __recv_coredump_records(fd_coredump, &sink, &size, truncated,
					   fd_peer_pidfd);
	if (received < 0)
		return -1;

	/*
	 * Nothing is written for a hole, so grow the file to the size the
	 * records describe in case the coredump ended in one.
	 */
	if (ftruncate(fd_core_file, size) < 0) {
		fprintf(stderr, "%s: ftruncate to %llu failed: %m\n",
			__func__, (unsigned long long)size);
		return -1;
	}

	if (coredump_size)
		*coredump_size = size;

	return received;
}

/* The ELF header of a native core file. */
static bool is_core_ehdr(const ElfW(Ehdr) *ehdr)
{
	return !memcmp(ehdr->e_ident, ELFMAG, SELFMAG) &&
	       ehdr->e_ident[EI_CLASS] == COREDUMP_ELFCLASS &&
	       ehdr->e_type == ET_CORE;
}

/* Whatever the server ends up with has to be an ELF core file. */
bool is_elf_core(int fd)
{
	ElfW(Ehdr) ehdr;

	if (pread(fd, &ehdr, sizeof(ehdr), 0) != sizeof(ehdr)) {
		fprintf(stderr, "%s: short read: %m\n", __func__);
		return false;
	}

	if (!is_core_ehdr(&ehdr)) {
		fprintf(stderr, "%s: not an ELF core file\n", __func__);
		return false;
	}

	return true;
}

int create_detached_tmpfs(void)
{
	int fd_context, fd_tmpfs;

	fd_context = sys_fsopen("tmpfs", 0);
	if (fd_context < 0)
		return -1;

	if (sys_fsconfig(fd_context, FSCONFIG_CMD_CREATE, NULL, NULL, 0) < 0)
		return -1;

	fd_tmpfs = sys_fsmount(fd_context, 0, 0);
	close(fd_context);
	return fd_tmpfs;
}

int create_and_listen_unix_socket(const char *path)
{
	struct sockaddr_un addr = {
		.sun_family = AF_UNIX,
	};
	assert(strlen(path) < sizeof(addr.sun_path) - 1);
	strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
	size_t addr_len =
		offsetof(struct sockaddr_un, sun_path) + strlen(path) + 1;
	int fd, ret;

	fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (fd < 0)
		goto out;

	ret = bind(fd, (const struct sockaddr *)&addr, addr_len);
	if (ret < 0)
		goto out;

	ret = listen(fd, 128);
	if (ret < 0)
		goto out;

	return fd;

out:
	fprintf(stderr, "%s: %s: %m\n", __func__, path);
	if (fd >= 0)
		close(fd);
	return -1;
}

bool set_core_pattern(const char *pattern)
{
	int fd;
	ssize_t ret;

	fd = open("/proc/sys/kernel/core_pattern", O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return false;

	ret = write(fd, pattern, strlen(pattern));
	close(fd);
	if (ret < 0)
		return false;

	fprintf(stderr, "Set core_pattern to '%s' | %zu == %zu\n", pattern, ret, strlen(pattern));
	return ret == strlen(pattern);
}

int get_peer_pidfd(int fd)
{
	int fd_peer_pidfd;
	socklen_t fd_peer_pidfd_len = sizeof(fd_peer_pidfd);
	int ret = getsockopt(fd, SOL_SOCKET, SO_PEERPIDFD, &fd_peer_pidfd,
			     &fd_peer_pidfd_len);
	if (ret < 0) {
		fprintf(stderr, "get_peer_pidfd: getsockopt(SO_PEERPIDFD) failed: %m\n");
		return -1;
	}
	fprintf(stderr, "get_peer_pidfd: successfully retrieved pidfd %d\n", fd_peer_pidfd);
	return fd_peer_pidfd;
}

bool get_pidfd_info(int fd_peer_pidfd, struct pidfd_info *info)
{
	int ret;
	memset(info, 0, sizeof(*info));
	info->mask = PIDFD_INFO_EXIT | PIDFD_INFO_COREDUMP | PIDFD_INFO_COREDUMP_SIGNAL;
	ret = ioctl(fd_peer_pidfd, PIDFD_GET_INFO, info);
	if (ret < 0) {
		fprintf(stderr, "get_pidfd_info: ioctl(PIDFD_GET_INFO) failed: %m\n");
		return false;
	}
	fprintf(stderr, "get_pidfd_info: mask=0x%llx, coredump_mask=0x%x, coredump_signal=%d, coredump_code=%d\n",
		(unsigned long long)info->mask, info->coredump_mask, info->coredump_signal, info->coredump_code);
	return true;
}

/* Protocol helper functions */

ssize_t recv_marker(int fd)
{
	enum coredump_mark mark = COREDUMP_MARK_REQACK;
	ssize_t ret;

	ret = recv(fd, &mark, sizeof(mark), MSG_WAITALL);
	if (ret != sizeof(mark))
		return -1;

	switch (mark) {
	case COREDUMP_MARK_REQACK:
		fprintf(stderr, "Received marker: ReqAck\n");
		return COREDUMP_MARK_REQACK;
	case COREDUMP_MARK_MINSIZE:
		fprintf(stderr, "Received marker: MinSize\n");
		return COREDUMP_MARK_MINSIZE;
	case COREDUMP_MARK_MAXSIZE:
		fprintf(stderr, "Received marker: MaxSize\n");
		return COREDUMP_MARK_MAXSIZE;
	case COREDUMP_MARK_UNSUPPORTED:
		fprintf(stderr, "Received marker: Unsupported\n");
		return COREDUMP_MARK_UNSUPPORTED;
	case COREDUMP_MARK_CONFLICTING:
		fprintf(stderr, "Received marker: Conflicting\n");
		return COREDUMP_MARK_CONFLICTING;
	default:
		fprintf(stderr, "Received unknown marker: %u\n", mark);
		break;
	}
	return -1;
}

bool read_marker(int fd, enum coredump_mark mark)
{
	ssize_t ret;

	ret = recv_marker(fd);
	if (ret < 0)
		return false;
	return ret == mark;
}

bool read_coredump_req(int fd, struct coredump_req *req)
{
	ssize_t ret;
	size_t field_size, user_size, known_size, kernel_size, remaining_size;

	memset(req, 0, sizeof(*req));
	field_size = sizeof(req->size);

	/* Peek the size of the coredump request. */
	ret = recv(fd, req, field_size, MSG_PEEK | MSG_WAITALL);
	if (ret != field_size) {
		fprintf(stderr, "read_coredump_req: peek failed (got %zd, expected %zu): %m\n",
			ret, field_size);
		return false;
	}
	kernel_size = req->size;

	if (kernel_size < COREDUMP_REQ_SIZE_VER0) {
		fprintf(stderr, "read_coredump_req: kernel_size %zu < min %d\n",
			kernel_size, COREDUMP_REQ_SIZE_VER0);
		return false;
	}
	if (kernel_size >= PAGE_SIZE) {
		fprintf(stderr, "read_coredump_req: kernel_size %zu >= PAGE_SIZE %d\n",
			kernel_size, PAGE_SIZE);
		return false;
	}

	/* Consume as much of the request as we know about. */
	user_size = sizeof(struct coredump_req);
	known_size = user_size < kernel_size ? user_size : kernel_size;
	ret = recv(fd, req, known_size, MSG_WAITALL);
	if (ret != known_size)
		return false;

	fprintf(stderr, "Read coredump request with size %u and mask 0x%llx\n",
		req->size, (unsigned long long)req->mask);

	if (kernel_size > user_size)
		remaining_size = kernel_size - user_size;
	else
		remaining_size = 0;

	if (PAGE_SIZE <= remaining_size)
		return false;

	/*
	 * Discard any additional data if the kernel's request was larger than
	 * what we knew about or cared about.
	 */
	if (remaining_size) {
		char buffer[PAGE_SIZE];

		ret = recv(fd, buffer, remaining_size, MSG_WAITALL);
		if (ret != remaining_size)
			return false;
		fprintf(stderr, "Discarded %zu bytes of data after coredump request\n", remaining_size);
	}

	return true;
}

bool send_coredump_ack(int fd, const struct coredump_req *req,
		       __u64 mask, size_t size_ack)
{
	ssize_t ret;
	/*
	 * Wrap struct coredump_ack in a larger struct so we can
	 * simulate sending to much data to the kernel.
	 */
	struct large_ack_for_size_testing {
		struct coredump_ack ack;
		char buffer[PAGE_SIZE];
	} large_ack = {};

	if (!size_ack)
		size_ack = sizeof(struct coredump_ack) < req->size_ack ?
				   sizeof(struct coredump_ack) :
				   req->size_ack;
	large_ack.ack.mask = mask;
	large_ack.ack.size = size_ack;
	ret = send(fd, &large_ack, size_ack, MSG_NOSIGNAL);
	if (ret != size_ack) {
		fprintf(stderr, "%s: short send %zd: %m\n", __func__, ret);
		return false;
	}

	fprintf(stderr, "Sent coredump ack with size %zu and mask 0x%llx\n",
		size_ack, (unsigned long long)mask);
	return true;
}

/* Every option the kernel is expected to advertise in coredump_req->mask. */
#define TEST_REQ_MASK_ALL					\
	(COREDUMP_KERNEL | COREDUMP_USERSPACE |			\
	 COREDUMP_REJECT | COREDUMP_WAIT |			\
	 COREDUMP_RECORDS | COREDUMP_SPARSE)

bool check_coredump_req(const struct coredump_req *req)
{
	if (req->size < COREDUMP_REQ_SIZE_VER0) {
		fprintf(stderr, "%s: size %u below minimum %d\n",
			__func__, req->size, COREDUMP_REQ_SIZE_VER0);
		return false;
	}
	if (req->mask != TEST_REQ_MASK_ALL) {
		fprintf(stderr, "%s: mask 0x%llx, expected 0x%llx\n",
			__func__, (unsigned long long)req->mask,
			(unsigned long long)TEST_REQ_MASK_ALL);
		return false;
	}
	return true;
}

int open_coredump_tmpfile(int fd_tmpfs_detached)
{
	return openat(fd_tmpfs_detached, ".", O_TMPFILE | O_RDWR | O_EXCL, 0600);
}

void process_coredump_worker(int fd_coredump, int fd_peer_pidfd, int fd_core_file)
{
	int epfd = -1;
	int exit_code = EXIT_FAILURE;
	struct epoll_event ev;
	int flags;

	/* Set socket to non-blocking mode for edge-triggered epoll */
	flags = fcntl(fd_coredump, F_GETFL, 0);
	if (flags < 0) {
		fprintf(stderr, "Worker: fcntl(F_GETFL) failed: %m\n");
		goto out;
	}
	if (fcntl(fd_coredump, F_SETFL, flags | O_NONBLOCK) < 0) {
		fprintf(stderr, "Worker: fcntl(F_SETFL, O_NONBLOCK) failed: %m\n");
		goto out;
	}

	epfd = epoll_create1(0);
	if (epfd < 0) {
		fprintf(stderr, "Worker: epoll_create1() failed: %m\n");
		goto out;
	}

	ev.events = EPOLLIN | EPOLLRDHUP | EPOLLET;
	ev.data.fd = fd_coredump;
	if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd_coredump, &ev) < 0) {
		fprintf(stderr, "Worker: epoll_ctl(EPOLL_CTL_ADD) failed: %m\n");
		goto out;
	}

	for (;;) {
		struct epoll_event events[1];
		int n = epoll_wait(epfd, events, 1, -1);
		if (n < 0) {
			fprintf(stderr, "Worker: epoll_wait() failed: %m\n");
			break;
		}

		if (events[0].events & (EPOLLIN | EPOLLRDHUP)) {
			for (;;) {
				char buffer[4096];
				ssize_t bytes_read = read(fd_coredump, buffer, sizeof(buffer));
				if (bytes_read < 0) {
					if (errno == EAGAIN || errno == EWOULDBLOCK)
						break;
					fprintf(stderr, "Worker: read() failed: %m\n");
					goto out;
				}
				if (bytes_read == 0)
					goto done;
				ssize_t bytes_write = write(fd_core_file, buffer, bytes_read);
				if (bytes_write != bytes_read) {
					if (bytes_write < 0 && errno == ENOSPC)
						continue;
					fprintf(stderr, "Worker: write() failed (read=%zd, write=%zd): %m\n",
						bytes_read, bytes_write);
					goto out;
				}
			}
		}
	}

done:
	exit_code = EXIT_SUCCESS;
	fprintf(stderr, "Worker: completed successfully\n");
out:
	if (epfd >= 0)
		close(epfd);
	if (fd_core_file >= 0)
		close(fd_core_file);
	if (fd_peer_pidfd >= 0)
		close(fd_peer_pidfd);
	if (fd_coredump >= 0)
		close(fd_coredump);
	_exit(exit_code);
}
