/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef _UAPI_LINUX_COREDUMP_H
#define _UAPI_LINUX_COREDUMP_H

#include <linux/types.h>

/**
 * coredump_{req,ack} flags
 * @COREDUMP_KERNEL: kernel writes coredump
 * @COREDUMP_USERSPACE: userspace writes coredump
 * @COREDUMP_REJECT: don't generate coredump
 * @COREDUMP_WAIT: wait for coredump server
 * @COREDUMP_RECORDS: send the coredump as a sequence of records instead of
 *                    as a plain byte stream, see struct coredump_record_header;
 *                    requires COREDUMP_KERNEL
 * @COREDUMP_SPARSE: describe the holes in the coredump as zero records
 *                   instead of transferring them; requires COREDUMP_RECORDS
 */
enum {
	COREDUMP_KERNEL		= (1ULL << 0),
	COREDUMP_USERSPACE	= (1ULL << 1),
	COREDUMP_REJECT		= (1ULL << 2),
	COREDUMP_WAIT		= (1ULL << 3),
	COREDUMP_RECORDS	= (1ULL << 4),
	COREDUMP_SPARSE		= (1ULL << 5),
};

/**
 * struct coredump_req - message kernel sends to userspace
 * @size: size of struct coredump_req
 * @size_ack: known size of struct coredump_ack on this kernel
 * @mask: supported features
 *
 * When a coredump happens the kernel will connect to the coredump
 * socket and send a coredump request to the coredump server. The @size
 * member is set to the size of struct coredump_req and provides a hint
 * to userspace how much data can be read. Userspace may use MSG_PEEK to
 * peek the size of struct coredump_req and then choose to consume it in
 * one go. Userspace may also simply read a COREDUMP_REQ_SIZE_VER0
 * request. If the size the kernel sends is larger userspace simply
 * discards any remaining data.
 *
 * The coredump_req->mask member is set to the currently known features.
 * Userspace may only set coredump_ack->mask to the bits raised by the
 * kernel in coredump_req->mask.
 *
 * The coredump_req->size_ack member is set by the kernel to the size of
 * struct coredump_ack the kernel knows. Userspace may only send up to
 * coredump_req->size_ack bytes to the kernel and must set
 * coredump_ack->size accordingly.
 */
struct coredump_req {
	__u32 size;
	__u32 size_ack;
	__u64 mask;
};

enum {
	COREDUMP_REQ_SIZE_VER0 = 16U, /* size of first published struct */
};

/**
 * struct coredump_ack - message userspace sends to kernel
 * @size: size of the struct
 * @spare: unused
 * @mask: features kernel is supposed to use
 *
 * The @size member must be set to the size of struct coredump_ack. It
 * may never exceed what the kernel returned in coredump_req->size_ack
 * but it may of course be smaller (>= COREDUMP_ACK_SIZE_VER0 and <=
 * coredump_req->size_ack).
 *
 * The @mask member must be set to the features the coredump server
 * wants the kernel to use. Only bits the kernel returned in
 * coredump_req->mask may be set.
 */
struct coredump_ack {
	__u32 size;
	__u32 spare;
	__u64 mask;
};

enum {
	COREDUMP_ACK_SIZE_VER0 = 16U, /* size of first published struct */
};

/**
 * enum coredump_mark - Markers for the coredump socket
 *
 * The kernel will place a single byte on the coredump socket. The
 * markers notify userspace whether the coredump ack succeeded or
 * failed.
 *
 * @COREDUMP_MARK_MINSIZE: the provided coredump_ack size was too small
 * @COREDUMP_MARK_MAXSIZE: the provided coredump_ack size was too big
 * @COREDUMP_MARK_UNSUPPORTED: the provided coredump_ack mask was invalid
 * @COREDUMP_MARK_CONFLICTING: the provided coredump_ack mask has conflicting options
 * @COREDUMP_MARK_REQACK: the coredump request and ack was successful
 * @__COREDUMP_MARK_MAX: the maximum coredump mark value
 */
enum coredump_mark {
	COREDUMP_MARK_REQACK		= 0U,
	COREDUMP_MARK_MINSIZE		= 1U,
	COREDUMP_MARK_MAXSIZE		= 2U,
	COREDUMP_MARK_UNSUPPORTED	= 3U,
	COREDUMP_MARK_CONFLICTING	= 4U,
	__COREDUMP_MARK_MAX		= (1U << 31),
};

/**
 * enum coredump_record_type - Type of a coredump record
 *
 * @COREDUMP_RECORD_DATA: the header is followed by ->len bytes of data
 * @COREDUMP_RECORD_END: the coredump ends here, the header is not followed
 *                       by any data and no further record is sent
 * @COREDUMP_RECORD_ZERO: the header stands for ->len zero bytes and is not
 *                        followed by any data
 * @__COREDUMP_RECORD_TYPE_MAX: the maximum coredump record type value
 */
enum coredump_record_type {
	COREDUMP_RECORD_DATA		= 0U,
	COREDUMP_RECORD_END		= 1U,
	COREDUMP_RECORD_ZERO		= 2U,
	__COREDUMP_RECORD_TYPE_MAX	= (1U << 31),
};

/**
 * struct coredump_record_header - header of a coredump record
 * @size: size of struct coredump_record_header
 * @type: one of enum coredump_record_type
 * @flags: modifiers for this record
 * @offset: offset in the coredump this record starts at
 * @len: number of coredump bytes this record accounts for
 *
 * If the coredump server raises COREDUMP_RECORDS in coredump_ack->mask
 * the kernel doesn't send the coredump as a plain byte stream. It sends
 * a sequence of records instead. A COREDUMP_RECORD_DATA record is
 * followed by @len bytes of actual coredump data. A
 * COREDUMP_RECORD_ZERO record is followed by nothing and stands for
 * @len zero bytes. A server that didn't raise COREDUMP_SPARSE never
 * sees a zero record. Records arrive in order and leave no gaps. So
 * @offset is the sum of the @len of all records before it.
 *
 * The last record is a COREDUMP_RECORD_END record. It is followed by
 * nothing. Its @len is zero. Its @offset is the size of the coredump.
 * The kernel only sends it once it has written the whole coredump. A
 * server that hits end-of-file without having seen an end record must
 * treat the coredump as incomplete.
 *
 * The @size member is set to the size of struct coredump_record_header
 * the kernel knows and lets the header grow later. It comes first so it
 * can be peeked. Userspace must consume @size bytes and discard
 * anything beyond what it knows. It must refuse a @size smaller than
 * COREDUMP_RECORD_HEADER_SIZE_VER0. @size covers the header alone.
 * @offset and @len count coredump bytes.
 *
 * The @flags member carries modifiers that change how the record is to
 * be interpreted. No flag is defined yet. Userspace must refuse a
 * record carrying a flag or a type it doesn't know. Every new record
 * type is raised in coredump_req->mask as a feature of its own. A
 * server only ever sees the types it asked for.
 *
 * COREDUMP_RECORDS must be combined with COREDUMP_KERNEL, and
 * COREDUMP_SPARSE with COREDUMP_RECORDS.
 */
struct coredump_record_header {
	__u32 size;
	__u32 type;
	__u64 flags;
	__u64 offset;
	__u64 len;
};

enum {
	COREDUMP_RECORD_HEADER_SIZE_VER0 = 32U, /* size of first published struct */
};

#endif /* _UAPI_LINUX_COREDUMP_H */
