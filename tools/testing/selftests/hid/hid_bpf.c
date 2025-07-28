// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022-2024 Red Hat */
#include "hid.skel.h"
#include "hid_common.h"
#include <bpf/bpf.h>

struct attach_prog_args {
	int prog_fd;
	unsigned int hid;
	int retval;
	int insert_head;
};

struct hid_hw_request_syscall_args {
	__u8 data[10];
	unsigned int hid;
	int retval;
	size_t size;
	enum hid_report_type type;
	__u8 request_type;
};

FIXTURE(hid_bpf) {
	int dev_id;
	int uhid_fd;
	int hidraw_fd;
	int hid_id;
	pthread_t tid;
	struct hid *skel;
	struct bpf_link *hid_links[3]; /* max number of programs loaded in a single test */
};
static void detach_bpf(FIXTURE_DATA(hid_bpf) * self)
{
	int i;

	if (self->hidraw_fd)
		close(self->hidraw_fd);
	self->hidraw_fd = 0;

	if (!self->skel)
		return;

	hid__detach(self->skel);

	for (i = 0; i < ARRAY_SIZE(self->hid_links); i++) {
		if (self->hid_links[i])
			bpf_link__destroy(self->hid_links[i]);
	}

	hid__destroy(self->skel);
	self->skel = NULL;
}

FIXTURE_TEARDOWN(hid_bpf) {
	void *uhid_err;

	uhid_destroy(_metadata, self->uhid_fd);

	detach_bpf(self);
	pthread_join(self->tid, &uhid_err);
}
#define TEARDOWN_LOG(fmt, ...) do { \
	TH_LOG(fmt, ##__VA_ARGS__); \
	hid_bpf_teardown(_metadata, self, variant); \
} while (0)

FIXTURE_SETUP(hid_bpf)
{
	time_t t;
	int err;

	/* initialize random number generator */
	srand((unsigned int)time(&t));

	self->dev_id = rand() % 1024;

	self->uhid_fd = setup_uhid(_metadata, self->dev_id);

	/* locate the uev, self, variant);ent file of the created device */
	self->hid_id = get_hid_id(self->dev_id);
	ASSERT_GT(self->hid_id, 0)
		TEARDOWN_LOG("Could not locate uhid device id: %d", self->hid_id);

	err = uhid_start_listener(_metadata, &self->tid, self->uhid_fd);
	ASSERT_EQ(0, err) TEARDOWN_LOG("could not start udev listener: %d", err);
}

struct test_program {
	const char *name;
	int insert_head;
};
#define LOAD_PROGRAMS(progs) \
	load_programs(progs, ARRAY_SIZE(progs), _metadata, self, variant)
#define LOAD_BPF \
	load_programs(NULL, 0, _metadata, self, variant)
static void load_programs(const struct test_program programs[],
			  const size_t progs_count,
			  struct __test_metadata *_metadata,
			  FIXTURE_DATA(hid_bpf) * self,
			  const FIXTURE_VARIANT(hid_bpf) * variant)
{
	struct bpf_map *iter_map;
	int err = -EINVAL;

	ASSERT_LE(progs_count, ARRAY_SIZE(self->hid_links))
		TH_LOG("too many programs are to be loaded");

	/* open the bpf file */
	self->skel = hid__open();
	ASSERT_OK_PTR(self->skel) TEARDOWN_LOG("Error while calling hid__open");

	for (int i = 0; i < progs_count; i++) {
		struct bpf_program *prog;
		struct bpf_map *map;
		int *ops_hid_id;

		prog = bpf_object__find_program_by_name(*self->skel->skeleton->obj,
							programs[i].name);
		ASSERT_OK_PTR(prog) TH_LOG("can not find program by name '%s'", programs[i].name);

		bpf_program__set_autoload(prog, true);

		map = bpf_object__find_map_by_name(*self->skel->skeleton->obj,
							  programs[i].name + 4);
		ASSERT_OK_PTR(map) TH_LOG("can not find struct_ops by name '%s'",
					  programs[i].name + 4);

		/* hid_id is the first field of struct hid_bpf_ops */
		ops_hid_id = bpf_map__initial_value(map, NULL);
		ASSERT_OK_PTR(ops_hid_id) TH_LOG("unable to retrieve struct_ops data");

		*ops_hid_id = self->hid_id;
	}

	/* we disable the auto-attach feature of all maps because we
	 * only want the tested one to be manually attached in the next
	 * call to bpf_map__attach_struct_ops()
	 */
	bpf_object__for_each_map(iter_map, *self->skel->skeleton->obj)
		bpf_map__set_autoattach(iter_map, false);

	err = hid__load(self->skel);
	ASSERT_OK(err) TH_LOG("hid_skel_load failed: %d", err);

	for (int i = 0; i < progs_count; i++) {
		struct bpf_map *map;

		map = bpf_object__find_map_by_name(*self->skel->skeleton->obj,
							  programs[i].name + 4);
		ASSERT_OK_PTR(map) TH_LOG("can not find struct_ops by name '%s'",
					  programs[i].name + 4);

		self->hid_links[i] = bpf_map__attach_struct_ops(map);
		ASSERT_OK_PTR(self->hid_links[i]) TH_LOG("failed to attach struct ops '%s'",
							 programs[i].name + 4);
	}

	hid__attach(self->skel);

	self->hidraw_fd = open_hidraw(self->dev_id);
	ASSERT_GE(self->hidraw_fd, 0) TH_LOG("open_hidraw");
}

/*
 * A simple test to see if the fixture is working fine.
 * If this fails, none of the other tests will pass.
 */
TEST_F(hid_bpf, test_create_uhid)
{
}

/*
 * Attach hid_first_event to the given uhid device,
 * retrieve and open the matching hidraw node,
 * inject one event in the uhid device,
 * check that the program sees it and can change the data
 */
TEST_F(hid_bpf, raw_event)
{
	const struct test_program progs[] = {
		{ .name = "hid_first_event" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* check that the program is correctly loaded */
	ASSERT_EQ(self->skel->data->callback_check, 52) TH_LOG("callback_check1");
	ASSERT_EQ(self->skel->data->callback2_check, 52) TH_LOG("callback2_check1");

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* check that hid_first_event() was executed */
	ASSERT_EQ(self->skel->data->callback_check, 42) TH_LOG("callback_check1");

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[2], 47);

	/* inject another event */
	memset(buf, 0, sizeof(buf));
	buf[0] = 1;
	buf[1] = 47;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* check that hid_first_event() was executed */
	ASSERT_EQ(self->skel->data->callback_check, 47) TH_LOG("callback_check1");

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[2], 52);
}

/*
 * Attach hid_first_event to the given uhid device,
 * retrieve and open the matching hidraw node,
 * inject one event in the uhid device,
 * check that the program sees it and can change the data
 */
TEST_F(hid_bpf, subprog_raw_event)
{
	const struct test_program progs[] = {
		{ .name = "hid_subprog_first_event" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[2], 47);

	/* inject another event */
	memset(buf, 0, sizeof(buf));
	buf[0] = 1;
	buf[1] = 47;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[2], 52);
}

/*
 * Attach hid_first_event to the given uhid device,
 * attempt at re-attaching it, we should not lock and
 * return an invalid struct bpf_link
 */
TEST_F(hid_bpf, multiple_attach)
{
	const struct test_program progs[] = {
		{ .name = "hid_first_event" },
	};
	struct bpf_link *link;

	LOAD_PROGRAMS(progs);

	link = bpf_map__attach_struct_ops(self->skel->maps.first_event);
	ASSERT_NULL(link) TH_LOG("unexpected return value when re-attaching the struct_ops");
}

/*
 * Ensures that we can attach/detach programs
 */
TEST_F(hid_bpf, test_attach_detach)
{
	const struct test_program progs[] = {
		{ .name = "hid_first_event" },
		{ .name = "hid_second_event" },
	};
	struct bpf_link *link;
	__u8 buf[10] = {0};
	int err, link_fd;

	LOAD_PROGRAMS(progs);

	link = self->hid_links[0];
	ASSERT_OK_PTR(link) TH_LOG("HID-BPF link not created");

	link_fd = bpf_link__fd(link);
	ASSERT_GE(link_fd, 0) TH_LOG("HID-BPF link FD not valid");

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[2], 47);

	/* make sure both programs are run */
	ASSERT_EQ(buf[3], 52);

	/* pin the first program and immediately unpin it */
#define PIN_PATH "/sys/fs/bpf/hid_first_event"
	err = bpf_obj_pin(link_fd, PIN_PATH);
	ASSERT_OK(err) TH_LOG("error while calling bpf_obj_pin");
	remove(PIN_PATH);
#undef PIN_PATH
	usleep(100000);

	/* detach the program */
	detach_bpf(self);

	self->hidraw_fd = open_hidraw(self->dev_id);
	ASSERT_GE(self->hidraw_fd, 0) TH_LOG("open_hidraw");

	/* inject another event */
	memset(buf, 0, sizeof(buf));
	buf[0] = 1;
	buf[1] = 47;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw_no_bpf");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[1], 47);
	ASSERT_EQ(buf[2], 0);
	ASSERT_EQ(buf[3], 0);

	/* re-attach our program */

	LOAD_PROGRAMS(progs);

	/* inject one event */
	memset(buf, 0, sizeof(buf));
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[2], 47);
	ASSERT_EQ(buf[3], 52);
}

/*
 * Attach hid_change_report_id to the given uhid device,
 * retrieve and open the matching hidraw node,
 * inject one event in the uhid device,
 * check that the program sees it and can change the data
 */
TEST_F(hid_bpf, test_hid_change_report)
{
	const struct test_program progs[] = {
		{ .name = "hid_change_report_id" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 9) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 2);
	ASSERT_EQ(buf[1], 42);
	ASSERT_EQ(buf[2], 0) TH_LOG("leftovers_from_previous_test");
}

/*
 * Call hid_bpf_input_report against the given uhid device,
 * check that the program is called and does the expected.
 */
TEST_F(hid_bpf, test_hid_user_input_report_call)
{
	struct hid_hw_request_syscall_args args = {
		.retval = -1,
		.size = 10,
	};
	DECLARE_LIBBPF_OPTS(bpf_test_run_opts, tattrs,
			    .ctx_in = &args,
			    .ctx_size_in = sizeof(args),
	);
	__u8 buf[10] = {0};
	int err, prog_fd;

	LOAD_BPF;

	args.hid = self->hid_id;
	args.data[0] = 1; /* report ID */
	args.data[1] = 2; /* report ID */
	args.data[2] = 42; /* report ID */

	prog_fd = bpf_program__fd(self->skel->progs.hid_user_input_report);

	/* check that there is no data to read from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, -1) TH_LOG("read_hidraw");

	err = bpf_prog_test_run_opts(prog_fd, &tattrs);

	ASSERT_OK(err) TH_LOG("error while calling bpf_prog_test_run_opts");

	ASSERT_EQ(args.retval, 0);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[1], 2);
	ASSERT_EQ(buf[2], 42);
}

/*
 * Call hid_bpf_hw_output_report against the given uhid device,
 * check that the program is called and does the expected.
 */
TEST_F(hid_bpf, test_hid_user_output_report_call)
{
	struct hid_hw_request_syscall_args args = {
		.retval = -1,
		.size = 10,
	};
	DECLARE_LIBBPF_OPTS(bpf_test_run_opts, tattrs,
			    .ctx_in = &args,
			    .ctx_size_in = sizeof(args),
	);
	int err, cond_err, prog_fd;
	struct timespec time_to_wait;

	LOAD_BPF;

	args.hid = self->hid_id;
	args.data[0] = 1; /* report ID */
	args.data[1] = 2; /* report ID */
	args.data[2] = 42; /* report ID */

	prog_fd = bpf_program__fd(self->skel->progs.hid_user_output_report);

	pthread_mutex_lock(&uhid_output_mtx);

	memset(output_report, 0, sizeof(output_report));
	clock_gettime(CLOCK_REALTIME, &time_to_wait);
	time_to_wait.tv_sec += 2;

	err = bpf_prog_test_run_opts(prog_fd, &tattrs);
	cond_err = pthread_cond_timedwait(&uhid_output_cond, &uhid_output_mtx, &time_to_wait);

	ASSERT_OK(err) TH_LOG("error while calling bpf_prog_test_run_opts");
	ASSERT_OK(cond_err) TH_LOG("error while calling waiting for the condition");

	ASSERT_EQ(args.retval, 3);

	ASSERT_EQ(output_report[0], 1);
	ASSERT_EQ(output_report[1], 2);
	ASSERT_EQ(output_report[2], 42);

	pthread_mutex_unlock(&uhid_output_mtx);
}

/*
 * Call hid_hw_raw_request against the given uhid device,
 * check that the program is called and does the expected.
 */
TEST_F(hid_bpf, test_hid_user_raw_request_call)
{
	struct hid_hw_request_syscall_args args = {
		.retval = -1,
		.type = HID_FEATURE_REPORT,
		.request_type = HID_REQ_GET_REPORT,
		.size = 10,
	};
	DECLARE_LIBBPF_OPTS(bpf_test_run_opts, tattrs,
			    .ctx_in = &args,
			    .ctx_size_in = sizeof(args),
	);
	int err, prog_fd;

	LOAD_BPF;

	args.hid = self->hid_id;
	args.data[0] = 1; /* report ID */

	prog_fd = bpf_program__fd(self->skel->progs.hid_user_raw_request);

	err = bpf_prog_test_run_opts(prog_fd, &tattrs);
	ASSERT_OK(err) TH_LOG("error while calling bpf_prog_test_run_opts");

	ASSERT_EQ(args.retval, 2);

	ASSERT_EQ(args.data[1], 2);
}

/*
 * Call hid_hw_raw_request against the given uhid device,
 * check that the program is called and prevents the
 * call to uhid.
 */
TEST_F(hid_bpf, test_hid_filter_raw_request_call)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_filter_raw_request" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* first check that we did not attach to device_event */

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[1], 42);
	ASSERT_EQ(buf[2], 0) TH_LOG("leftovers_from_previous_test");

	/* now check that our program is preventing hid_hw_raw_request() */

	/* emit hid_hw_raw_request from hidraw */
	/* Get Feature */
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x1; /* Report Number */
	err = ioctl(self->hidraw_fd, HIDIOCGFEATURE(sizeof(buf)), buf);
	ASSERT_LT(err, 0) TH_LOG("unexpected success while reading HIDIOCGFEATURE: %d", err);
	ASSERT_EQ(errno, 20) TH_LOG("unexpected error code while reading HIDIOCGFEATURE: %d",
				    errno);

	/* remove our bpf program and check that we can now emit commands */

	/* detach the program */
	detach_bpf(self);

	self->hidraw_fd = open_hidraw(self->dev_id);
	ASSERT_GE(self->hidraw_fd, 0) TH_LOG("open_hidraw");

	err = ioctl(self->hidraw_fd, HIDIOCGFEATURE(sizeof(buf)), buf);
	ASSERT_GE(err, 0) TH_LOG("error while reading HIDIOCGFEATURE: %d", err);
}

/*
 * Call hid_hw_raw_request against the given uhid device,
 * check that the program is called and can issue the call
 * to uhid and transform the answer.
 */
TEST_F(hid_bpf, test_hid_change_raw_request_call)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_hidraw_raw_request" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* emit hid_hw_raw_request from hidraw */
	/* Get Feature */
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x1; /* Report Number */
	err = ioctl(self->hidraw_fd, HIDIOCGFEATURE(sizeof(buf)), buf);
	ASSERT_EQ(err, 3) TH_LOG("unexpected returned size while reading HIDIOCGFEATURE: %d", err);

	ASSERT_EQ(buf[0], 2);
	ASSERT_EQ(buf[1], 3);
	ASSERT_EQ(buf[2], 4);
}

/*
 * Call hid_hw_raw_request against the given uhid device,
 * check that the program is not making infinite loops.
 */
TEST_F(hid_bpf, test_hid_infinite_loop_raw_request_call)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_infinite_loop_raw_request" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* emit hid_hw_raw_request from hidraw */
	/* Get Feature */
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x1; /* Report Number */
	err = ioctl(self->hidraw_fd, HIDIOCGFEATURE(sizeof(buf)), buf);
	ASSERT_EQ(err, 3) TH_LOG("unexpected returned size while reading HIDIOCGFEATURE: %d", err);
}

/*
 * Call hid_hw_output_report against the given uhid device,
 * check that the program is called and prevents the
 * call to uhid.
 */
TEST_F(hid_bpf, test_hid_filter_output_report_call)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_filter_output_report" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* first check that we did not attach to device_event */

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[1], 42);
	ASSERT_EQ(buf[2], 0) TH_LOG("leftovers_from_previous_test");

	/* now check that our program is preventing hid_hw_output_report() */

	buf[0] = 1; /* report ID */
	buf[1] = 2;
	buf[2] = 42;

	err = write(self->hidraw_fd, buf, 3);
	ASSERT_LT(err, 0) TH_LOG("unexpected success while sending hid_hw_output_report: %d", err);
	ASSERT_EQ(errno, 25) TH_LOG("unexpected error code while sending hid_hw_output_report: %d",
				    errno);

	/* remove our bpf program and check that we can now emit commands */

	/* detach the program */
	detach_bpf(self);

	self->hidraw_fd = open_hidraw(self->dev_id);
	ASSERT_GE(self->hidraw_fd, 0) TH_LOG("open_hidraw");

	err = write(self->hidraw_fd, buf, 3);
	ASSERT_GE(err, 0) TH_LOG("error while sending hid_hw_output_report: %d", err);
}

/*
 * Call hid_hw_output_report against the given uhid device,
 * check that the program is called and can issue the call
 * to uhid and transform the answer.
 */
TEST_F(hid_bpf, test_hid_change_output_report_call)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_hidraw_output_report" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* emit hid_hw_output_report from hidraw */
	buf[0] = 1; /* report ID */
	buf[1] = 2;
	buf[2] = 42;

	err = write(self->hidraw_fd, buf, 10);
	ASSERT_EQ(err, 2) TH_LOG("unexpected returned size while sending hid_hw_output_report: %d",
				 err);
}

/*
 * Call hid_hw_output_report against the given uhid device,
 * check that the program is not making infinite loops.
 */
TEST_F(hid_bpf, test_hid_infinite_loop_output_report_call)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_infinite_loop_output_report" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* emit hid_hw_output_report from hidraw */
	buf[0] = 1; /* report ID */
	buf[1] = 2;
	buf[2] = 42;

	err = write(self->hidraw_fd, buf, 8);
	ASSERT_EQ(err, 2) TH_LOG("unexpected returned size while sending hid_hw_output_report: %d",
				 err);
}

/*
 * Attach hid_multiply_event_wq to the given uhid device,
 * retrieve and open the matching hidraw node,
 * inject one event in the uhid device,
 * check that the program sees it and can add extra data
 */
TEST_F(hid_bpf, test_multiply_events_wq)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_multiply_events_wq" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[1], 47);

	usleep(100000);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 9) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 2);
	ASSERT_EQ(buf[1], 3);
}

/*
 * Attach hid_multiply_event to the given uhid device,
 * retrieve and open the matching hidraw node,
 * inject one event in the uhid device,
 * check that the program sees it and can add extra data
 */
TEST_F(hid_bpf, test_multiply_events)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_multiply_events" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* inject one event */
	buf[0] = 1;
	buf[1] = 42;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 9) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 2);
	ASSERT_EQ(buf[1], 47);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 9) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 2);
	ASSERT_EQ(buf[1], 52);
}

/*
 * Call hid_bpf_input_report against the given uhid device,
 * check that the program is not making infinite loops.
 */
TEST_F(hid_bpf, test_hid_infinite_loop_input_report_call)
{
	const struct test_program progs[] = {
		{ .name = "hid_test_infinite_loop_input_report" },
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* emit hid_hw_output_report from hidraw */
	buf[0] = 1; /* report ID */
	buf[1] = 2;
	buf[2] = 42;

	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[1], 3);

	/* read the data from hidraw: hid_bpf_try_input_report should work exactly one time */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[0], 1);
	ASSERT_EQ(buf[1], 4);

	/* read the data from hidraw: there should be none */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, -1) TH_LOG("read_hidraw");
}

/*
 * Attach hid_insert{0,1,2} to the given uhid device,
 * retrieve and open the matching hidraw node,
 * inject one event in the uhid device,
 * check that the programs have been inserted in the correct order.
 */
TEST_F(hid_bpf, test_hid_attach_flags)
{
	const struct test_program progs[] = {
		{
			.name = "hid_test_insert2",
			.insert_head = 0,
		},
		{
			.name = "hid_test_insert1",
			.insert_head = 1,
		},
		{
			.name = "hid_test_insert3",
			.insert_head = 0,
		},
	};
	__u8 buf[10] = {0};
	int err;

	LOAD_PROGRAMS(progs);

	/* inject one event */
	buf[0] = 1;
	uhid_send_event(_metadata, self->uhid_fd, buf, 6);

	/* read the data from hidraw */
	memset(buf, 0, sizeof(buf));
	err = read(self->hidraw_fd, buf, sizeof(buf));
	ASSERT_EQ(err, 6) TH_LOG("read_hidraw");
	ASSERT_EQ(buf[1], 1);
	ASSERT_EQ(buf[2], 2);
	ASSERT_EQ(buf[3], 3);
}

/*
 * Attach hid_rdesc_fixup to the given uhid device,
 * retrieve and open the matching hidraw node,
 * check that the hidraw report descriptor has been updated.
 */
TEST_F(hid_bpf, test_rdesc_fixup)
{
	struct hidraw_report_descriptor rpt_desc = {0};
	const struct test_program progs[] = {
		{ .name = "hid_rdesc_fixup" },
	};
	int err, desc_size;

	LOAD_PROGRAMS(progs);

	/* check that hid_rdesc_fixup() was executed */
	ASSERT_EQ(self->skel->data->callback2_check, 0x21);

	/* read the exposed report descriptor from hidraw */
	err = ioctl(self->hidraw_fd, HIDIOCGRDESCSIZE, &desc_size);
	ASSERT_GE(err, 0) TH_LOG("error while reading HIDIOCGRDESCSIZE: %d", err);

	/* ensure the new size of the rdesc is bigger than the old one */
	ASSERT_GT(desc_size, sizeof(rdesc));

	rpt_desc.size = desc_size;
	err = ioctl(self->hidraw_fd, HIDIOCGRDESC, &rpt_desc);
	ASSERT_GE(err, 0) TH_LOG("error while reading HIDIOCGRDESC: %d", err);

	ASSERT_EQ(rpt_desc.value[4], 0x42);
}

static int libbpf_print_fn(enum libbpf_print_level level,
			   const char *format, va_list args)
{
	char buf[1024];

	if (level == LIBBPF_DEBUG)
		return 0;

	snprintf(buf, sizeof(buf), "# %s", format);

	vfprintf(stdout, buf, args);
	return 0;
}

int main(int argc, char **argv)
{
	/* Use libbpf 1.0 API mode */
	libbpf_set_strict_mode(LIBBPF_STRICT_ALL);
	libbpf_set_print(libbpf_print_fn);

	return test_harness_run(argc, argv);
}
