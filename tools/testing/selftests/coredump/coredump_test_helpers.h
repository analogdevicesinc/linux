/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __COREDUMP_TEST_HELPERS_H
#define __COREDUMP_TEST_HELPERS_H

#include <stdbool.h>
#include <sys/types.h>
#include <linux/coredump.h>

#include "../pidfd/pidfd.h"

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif

#define NUM_THREAD_SPAWN 128

/* Size of the mostly unpopulated mapping the sparse coredump test maps. */
#define SPARSE_MAPPING_SIZE (256 * 1024 * 1024)

/* Shared helper function declarations */
void *do_nothing(void *arg);
void crashing_child(void);
void crashing_child_sparse(size_t size);
ssize_t recv_coredump_records(int fd_coredump, int fd_core_file,
			      off_t *coredump_size, bool *truncated,
			      int fd_peer_pidfd);
bool is_elf_core(int fd);
int create_detached_tmpfs(void);
int create_and_listen_unix_socket(const char *path);
bool set_core_pattern(const char *pattern);
int get_peer_pidfd(int fd);
bool get_pidfd_info(int fd_peer_pidfd, struct pidfd_info *info);

/* Protocol helper function declarations */
ssize_t recv_marker(int fd);
bool read_marker(int fd, enum coredump_mark mark);
bool read_coredump_req(int fd, struct coredump_req *req);
bool send_coredump_ack(int fd, const struct coredump_req *req,
		       __u64 mask, size_t size_ack);
bool check_coredump_req(const struct coredump_req *req);
int open_coredump_tmpfile(int fd_tmpfs_detached);
void process_coredump_worker(int fd_coredump, int fd_peer_pidfd, int fd_core_file);

#endif /* __COREDUMP_TEST_HELPERS_H */
