/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014, LINBIT HA-Solutions GmbH.
 */

#ifndef __DRBD_STRINGS_H
#define __DRBD_STRINGS_H

struct state_names {
	const char * const *names;
	unsigned int size;
};

extern struct state_names drbd_conn_state_names;
extern struct state_names drbd_role_state_names;
extern struct state_names drbd_disk_state_names;
extern struct state_names drbd_error_messages;
extern struct state_names drbd_packet_names;

enum drbd_packet;

const char *drbd_conn_str(enum drbd_conns s);
const char *drbd_role_str(enum drbd_role s);
const char *drbd_disk_str(enum drbd_disk_state s);
const char *drbd_set_st_err_str(enum drbd_state_rv err);
const char *drbd_packet_name(enum drbd_packet cmd);


#endif  /* __DRBD_STRINGS_H */
