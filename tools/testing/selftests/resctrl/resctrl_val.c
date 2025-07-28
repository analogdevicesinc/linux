// SPDX-License-Identifier: GPL-2.0
/*
 * Memory bandwidth monitoring and allocation library
 *
 * Copyright (C) 2018 Intel Corporation
 *
 * Authors:
 *    Sai Praneeth Prakhya <sai.praneeth.prakhya@intel.com>,
 *    Fenghua Yu <fenghua.yu@intel.com>
 */
#include "resctrl.h"

#define UNCORE_IMC		"uncore_imc"
#define READ_FILE_NAME		"events/cas_count_read"
#define WRITE_FILE_NAME		"events/cas_count_write"
#define DYN_PMU_PATH		"/sys/bus/event_source/devices"
#define SCALE			0.00006103515625
#define MAX_IMCS		20
#define MAX_TOKENS		5
#define READ			0
#define WRITE			1

#define CON_MBM_LOCAL_BYTES_PATH		\
	"%s/%s/mon_data/mon_L3_%02d/mbm_local_bytes"

struct membw_read_format {
	__u64 value;         /* The value of the event */
	__u64 time_enabled;  /* if PERF_FORMAT_TOTAL_TIME_ENABLED */
	__u64 time_running;  /* if PERF_FORMAT_TOTAL_TIME_RUNNING */
	__u64 id;            /* if PERF_FORMAT_ID */
};

struct imc_counter_config {
	__u32 type;
	__u64 event;
	__u64 umask;
	struct perf_event_attr pe;
	struct membw_read_format return_value;
	int fd;
};

static char mbm_total_path[1024];
static int imcs;
static struct imc_counter_config imc_counters_config[MAX_IMCS][2];
static const struct resctrl_test *current_test;

void membw_initialize_perf_event_attr(int i, int j)
{
	memset(&imc_counters_config[i][j].pe, 0,
	       sizeof(struct perf_event_attr));
	imc_counters_config[i][j].pe.type = imc_counters_config[i][j].type;
	imc_counters_config[i][j].pe.size = sizeof(struct perf_event_attr);
	imc_counters_config[i][j].pe.disabled = 1;
	imc_counters_config[i][j].pe.inherit = 1;
	imc_counters_config[i][j].pe.exclude_guest = 0;
	imc_counters_config[i][j].pe.config =
		imc_counters_config[i][j].umask << 8 |
		imc_counters_config[i][j].event;
	imc_counters_config[i][j].pe.sample_type = PERF_SAMPLE_IDENTIFIER;
	imc_counters_config[i][j].pe.read_format =
		PERF_FORMAT_TOTAL_TIME_ENABLED | PERF_FORMAT_TOTAL_TIME_RUNNING;
}

void membw_ioctl_perf_event_ioc_reset_enable(int i, int j)
{
	ioctl(imc_counters_config[i][j].fd, PERF_EVENT_IOC_RESET, 0);
	ioctl(imc_counters_config[i][j].fd, PERF_EVENT_IOC_ENABLE, 0);
}

void membw_ioctl_perf_event_ioc_disable(int i, int j)
{
	ioctl(imc_counters_config[i][j].fd, PERF_EVENT_IOC_DISABLE, 0);
}

/*
 * get_event_and_umask:	Parse config into event and umask
 * @cas_count_cfg:	Config
 * @count:		iMC number
 * @op:			Operation (read/write)
 */
void get_event_and_umask(char *cas_count_cfg, int count, bool op)
{
	char *token[MAX_TOKENS];
	int i = 0;

	strcat(cas_count_cfg, ",");
	token[0] = strtok(cas_count_cfg, "=,");

	for (i = 1; i < MAX_TOKENS; i++)
		token[i] = strtok(NULL, "=,");

	for (i = 0; i < MAX_TOKENS; i++) {
		if (!token[i])
			break;
		if (strcmp(token[i], "event") == 0) {
			if (op == READ)
				imc_counters_config[count][READ].event =
				strtol(token[i + 1], NULL, 16);
			else
				imc_counters_config[count][WRITE].event =
				strtol(token[i + 1], NULL, 16);
		}
		if (strcmp(token[i], "umask") == 0) {
			if (op == READ)
				imc_counters_config[count][READ].umask =
				strtol(token[i + 1], NULL, 16);
			else
				imc_counters_config[count][WRITE].umask =
				strtol(token[i + 1], NULL, 16);
		}
	}
}

static int open_perf_event(int i, int cpu_no, int j)
{
	imc_counters_config[i][j].fd =
		perf_event_open(&imc_counters_config[i][j].pe, -1, cpu_no, -1,
				PERF_FLAG_FD_CLOEXEC);

	if (imc_counters_config[i][j].fd == -1) {
		fprintf(stderr, "Error opening leader %llx\n",
			imc_counters_config[i][j].pe.config);

		return -1;
	}

	return 0;
}

/* Get type and config (read and write) of an iMC counter */
static int read_from_imc_dir(char *imc_dir, int count)
{
	char cas_count_cfg[1024], imc_counter_cfg[1024], imc_counter_type[1024];
	FILE *fp;

	/* Get type of iMC counter */
	sprintf(imc_counter_type, "%s%s", imc_dir, "type");
	fp = fopen(imc_counter_type, "r");
	if (!fp) {
		ksft_perror("Failed to open iMC counter type file");

		return -1;
	}
	if (fscanf(fp, "%u", &imc_counters_config[count][READ].type) <= 0) {
		ksft_perror("Could not get iMC type");
		fclose(fp);

		return -1;
	}
	fclose(fp);

	imc_counters_config[count][WRITE].type =
				imc_counters_config[count][READ].type;

	/* Get read config */
	sprintf(imc_counter_cfg, "%s%s", imc_dir, READ_FILE_NAME);
	fp = fopen(imc_counter_cfg, "r");
	if (!fp) {
		ksft_perror("Failed to open iMC config file");

		return -1;
	}
	if (fscanf(fp, "%s", cas_count_cfg) <= 0) {
		ksft_perror("Could not get iMC cas count read");
		fclose(fp);

		return -1;
	}
	fclose(fp);

	get_event_and_umask(cas_count_cfg, count, READ);

	/* Get write config */
	sprintf(imc_counter_cfg, "%s%s", imc_dir, WRITE_FILE_NAME);
	fp = fopen(imc_counter_cfg, "r");
	if (!fp) {
		ksft_perror("Failed to open iMC config file");

		return -1;
	}
	if  (fscanf(fp, "%s", cas_count_cfg) <= 0) {
		ksft_perror("Could not get iMC cas count write");
		fclose(fp);

		return -1;
	}
	fclose(fp);

	get_event_and_umask(cas_count_cfg, count, WRITE);

	return 0;
}

/*
 * A system can have 'n' number of iMC (Integrated Memory Controller)
 * counters, get that 'n'. For each iMC counter get it's type and config.
 * Also, each counter has two configs, one for read and the other for write.
 * A config again has two parts, event and umask.
 * Enumerate all these details into an array of structures.
 *
 * Return: >= 0 on success. < 0 on failure.
 */
static int num_of_imcs(void)
{
	char imc_dir[512], *temp;
	unsigned int count = 0;
	struct dirent *ep;
	int ret;
	DIR *dp;

	dp = opendir(DYN_PMU_PATH);
	if (dp) {
		while ((ep = readdir(dp))) {
			temp = strstr(ep->d_name, UNCORE_IMC);
			if (!temp)
				continue;

			/*
			 * imc counters are named as "uncore_imc_<n>", hence
			 * increment the pointer to point to <n>. Note that
			 * sizeof(UNCORE_IMC) would count for null character as
			 * well and hence the last underscore character in
			 * uncore_imc'_' need not be counted.
			 */
			temp = temp + sizeof(UNCORE_IMC);

			/*
			 * Some directories under "DYN_PMU_PATH" could have
			 * names like "uncore_imc_free_running", hence, check if
			 * first character is a numerical digit or not.
			 */
			if (temp[0] >= '0' && temp[0] <= '9') {
				sprintf(imc_dir, "%s/%s/", DYN_PMU_PATH,
					ep->d_name);
				ret = read_from_imc_dir(imc_dir, count);
				if (ret) {
					closedir(dp);

					return ret;
				}
				count++;
			}
		}
		closedir(dp);
		if (count == 0) {
			ksft_print_msg("Unable to find iMC counters\n");

			return -1;
		}
	} else {
		ksft_perror("Unable to open PMU directory");

		return -1;
	}

	return count;
}

int initialize_mem_bw_imc(void)
{
	int imc, j;

	imcs = num_of_imcs();
	if (imcs <= 0)
		return imcs;

	/* Initialize perf_event_attr structures for all iMC's */
	for (imc = 0; imc < imcs; imc++) {
		for (j = 0; j < 2; j++)
			membw_initialize_perf_event_attr(imc, j);
	}

	return 0;
}

static void perf_close_imc_mem_bw(void)
{
	int mc;

	for (mc = 0; mc < imcs; mc++) {
		if (imc_counters_config[mc][READ].fd != -1)
			close(imc_counters_config[mc][READ].fd);
		if (imc_counters_config[mc][WRITE].fd != -1)
			close(imc_counters_config[mc][WRITE].fd);
	}
}

/*
 * perf_open_imc_mem_bw - Open perf fds for IMCs
 * @cpu_no: CPU number that the benchmark PID is bound to
 *
 * Return: = 0 on success. < 0 on failure.
 */
static int perf_open_imc_mem_bw(int cpu_no)
{
	int imc, ret;

	for (imc = 0; imc < imcs; imc++) {
		imc_counters_config[imc][READ].fd = -1;
		imc_counters_config[imc][WRITE].fd = -1;
	}

	for (imc = 0; imc < imcs; imc++) {
		ret = open_perf_event(imc, cpu_no, READ);
		if (ret)
			goto close_fds;
		ret = open_perf_event(imc, cpu_no, WRITE);
		if (ret)
			goto close_fds;
	}

	return 0;

close_fds:
	perf_close_imc_mem_bw();
	return -1;
}

/*
 * do_mem_bw_test - Perform memory bandwidth test
 *
 * Runs memory bandwidth test over one second period. Also, handles starting
 * and stopping of the IMC perf counters around the test.
 */
static void do_imc_mem_bw_test(void)
{
	int imc;

	for (imc = 0; imc < imcs; imc++) {
		membw_ioctl_perf_event_ioc_reset_enable(imc, READ);
		membw_ioctl_perf_event_ioc_reset_enable(imc, WRITE);
	}

	sleep(1);

	/* Stop counters after a second to get results (both read and write) */
	for (imc = 0; imc < imcs; imc++) {
		membw_ioctl_perf_event_ioc_disable(imc, READ);
		membw_ioctl_perf_event_ioc_disable(imc, WRITE);
	}
}

/*
 * get_mem_bw_imc - Memory bandwidth as reported by iMC counters
 * @bw_report: Bandwidth report type (reads, writes)
 *
 * Memory bandwidth utilized by a process on a socket can be calculated
 * using iMC counters. Perf events are used to read these counters.
 *
 * Return: = 0 on success. < 0 on failure.
 */
static int get_mem_bw_imc(const char *bw_report, float *bw_imc)
{
	float reads, writes, of_mul_read, of_mul_write;
	int imc;

	/* Start all iMC counters to log values (both read and write) */
	reads = 0, writes = 0, of_mul_read = 1, of_mul_write = 1;

	/*
	 * Get results which are stored in struct type imc_counter_config
	 * Take overflow into consideration before calculating total bandwidth.
	 */
	for (imc = 0; imc < imcs; imc++) {
		struct imc_counter_config *r =
			&imc_counters_config[imc][READ];
		struct imc_counter_config *w =
			&imc_counters_config[imc][WRITE];

		if (read(r->fd, &r->return_value,
			 sizeof(struct membw_read_format)) == -1) {
			ksft_perror("Couldn't get read bandwidth through iMC");
			return -1;
		}

		if (read(w->fd, &w->return_value,
			 sizeof(struct membw_read_format)) == -1) {
			ksft_perror("Couldn't get write bandwidth through iMC");
			return -1;
		}

		__u64 r_time_enabled = r->return_value.time_enabled;
		__u64 r_time_running = r->return_value.time_running;

		if (r_time_enabled != r_time_running)
			of_mul_read = (float)r_time_enabled /
					(float)r_time_running;

		__u64 w_time_enabled = w->return_value.time_enabled;
		__u64 w_time_running = w->return_value.time_running;

		if (w_time_enabled != w_time_running)
			of_mul_write = (float)w_time_enabled /
					(float)w_time_running;
		reads += r->return_value.value * of_mul_read * SCALE;
		writes += w->return_value.value * of_mul_write * SCALE;
	}

	if (strcmp(bw_report, "reads") == 0) {
		*bw_imc = reads;
		return 0;
	}

	if (strcmp(bw_report, "writes") == 0) {
		*bw_imc = writes;
		return 0;
	}

	*bw_imc = reads + writes;
	return 0;
}

/*
 * initialize_mem_bw_resctrl:	Appropriately populate "mbm_total_path"
 * @param:	Parameters passed to resctrl_val()
 * @domain_id:	Domain ID (cache ID; for MB, L3 cache ID)
 */
void initialize_mem_bw_resctrl(const struct resctrl_val_param *param,
			       int domain_id)
{
	sprintf(mbm_total_path, CON_MBM_LOCAL_BYTES_PATH, RESCTRL_PATH,
		param->ctrlgrp, domain_id);
}

/*
 * Open file to read MBM local bytes from resctrl FS
 */
static FILE *open_mem_bw_resctrl(const char *mbm_bw_file)
{
	FILE *fp;

	fp = fopen(mbm_bw_file, "r");
	if (!fp)
		ksft_perror("Failed to open total memory bandwidth file");

	return fp;
}

/*
 * Get MBM Local bytes as reported by resctrl FS
 */
static int get_mem_bw_resctrl(FILE *fp, unsigned long *mbm_total)
{
	if (fscanf(fp, "%lu\n", mbm_total) <= 0) {
		ksft_perror("Could not get MBM local bytes");
		return -1;
	}
	return 0;
}

static pid_t bm_pid, ppid;

void ctrlc_handler(int signum, siginfo_t *info, void *ptr)
{
	/* Only kill child after bm_pid is set after fork() */
	if (bm_pid)
		kill(bm_pid, SIGKILL);
	umount_resctrlfs();
	if (current_test && current_test->cleanup)
		current_test->cleanup();
	ksft_print_msg("Ending\n\n");

	exit(EXIT_SUCCESS);
}

/*
 * Register CTRL-C handler for parent, as it has to kill
 * child process before exiting.
 */
int signal_handler_register(const struct resctrl_test *test)
{
	struct sigaction sigact = {};
	int ret = 0;

	bm_pid = 0;

	current_test = test;
	sigact.sa_sigaction = ctrlc_handler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = SA_SIGINFO;
	if (sigaction(SIGINT, &sigact, NULL) ||
	    sigaction(SIGTERM, &sigact, NULL) ||
	    sigaction(SIGHUP, &sigact, NULL)) {
		ksft_perror("sigaction");
		ret = -1;
	}
	return ret;
}

/*
 * Reset signal handler to SIG_DFL.
 * Non-Value return because the caller should keep
 * the error code of other path even if sigaction fails.
 */
void signal_handler_unregister(void)
{
	struct sigaction sigact = {};

	current_test = NULL;
	sigact.sa_handler = SIG_DFL;
	sigemptyset(&sigact.sa_mask);
	if (sigaction(SIGINT, &sigact, NULL) ||
	    sigaction(SIGTERM, &sigact, NULL) ||
	    sigaction(SIGHUP, &sigact, NULL)) {
		ksft_perror("sigaction");
	}
}

static void parent_exit(pid_t ppid)
{
	kill(ppid, SIGKILL);
	umount_resctrlfs();
	exit(EXIT_FAILURE);
}

/*
 * print_results_bw:	the memory bandwidth results are stored in a file
 * @filename:		file that stores the results
 * @bm_pid:		child pid that runs benchmark
 * @bw_imc:		perf imc counter value
 * @bw_resc:		memory bandwidth value
 *
 * Return:		0 on success, < 0 on error.
 */
static int print_results_bw(char *filename, pid_t bm_pid, float bw_imc,
			    unsigned long bw_resc)
{
	unsigned long diff = fabs(bw_imc - bw_resc);
	FILE *fp;

	if (strcmp(filename, "stdio") == 0 || strcmp(filename, "stderr") == 0) {
		printf("Pid: %d \t Mem_BW_iMC: %f \t ", (int)bm_pid, bw_imc);
		printf("Mem_BW_resc: %lu \t Difference: %lu\n", bw_resc, diff);
	} else {
		fp = fopen(filename, "a");
		if (!fp) {
			ksft_perror("Cannot open results file");

			return -1;
		}
		if (fprintf(fp, "Pid: %d \t Mem_BW_iMC: %f \t Mem_BW_resc: %lu \t Difference: %lu\n",
			    (int)bm_pid, bw_imc, bw_resc, diff) <= 0) {
			ksft_print_msg("Could not log results\n");
			fclose(fp);

			return -1;
		}
		fclose(fp);
	}

	return 0;
}

/*
 * measure_mem_bw - Measures memory bandwidth numbers while benchmark runs
 * @uparams:		User supplied parameters
 * @param:		Parameters passed to resctrl_val()
 * @bm_pid:		PID that runs the benchmark
 * @bw_report:		Bandwidth report type (reads, writes)
 *
 * Measure memory bandwidth from resctrl and from another source which is
 * perf imc value or could be something else if perf imc event is not
 * available. Compare the two values to validate resctrl value. It takes
 * 1 sec to measure the data.
 */
int measure_mem_bw(const struct user_params *uparams,
		   struct resctrl_val_param *param, pid_t bm_pid,
		   const char *bw_report)
{
	unsigned long bw_resc, bw_resc_start, bw_resc_end;
	FILE *mem_bw_fp;
	float bw_imc;
	int ret;

	bw_report = get_bw_report_type(bw_report);
	if (!bw_report)
		return -1;

	mem_bw_fp = open_mem_bw_resctrl(mbm_total_path);
	if (!mem_bw_fp)
		return -1;

	ret = perf_open_imc_mem_bw(uparams->cpu);
	if (ret < 0)
		goto close_fp;

	ret = get_mem_bw_resctrl(mem_bw_fp, &bw_resc_start);
	if (ret < 0)
		goto close_imc;

	rewind(mem_bw_fp);

	do_imc_mem_bw_test();

	ret = get_mem_bw_resctrl(mem_bw_fp, &bw_resc_end);
	if (ret < 0)
		goto close_imc;

	ret = get_mem_bw_imc(bw_report, &bw_imc);
	if (ret < 0)
		goto close_imc;

	perf_close_imc_mem_bw();
	fclose(mem_bw_fp);

	bw_resc = (bw_resc_end - bw_resc_start) / MB;

	return print_results_bw(param->filename, bm_pid, bw_imc, bw_resc);

close_imc:
	perf_close_imc_mem_bw();
close_fp:
	fclose(mem_bw_fp);
	return ret;
}

/*
 * run_benchmark - Run a specified benchmark or fill_buf (default benchmark)
 *		   in specified signal. Direct benchmark stdio to /dev/null.
 * @signum:	signal number
 * @info:	signal info
 * @ucontext:	user context in signal handling
 */
static void run_benchmark(int signum, siginfo_t *info, void *ucontext)
{
	int operation, ret, memflush;
	char **benchmark_cmd;
	size_t span;
	bool once;
	FILE *fp;

	benchmark_cmd = info->si_ptr;

	/*
	 * Direct stdio of child to /dev/null, so that only parent writes to
	 * stdio (console)
	 */
	fp = freopen("/dev/null", "w", stdout);
	if (!fp) {
		ksft_perror("Unable to direct benchmark status to /dev/null");
		parent_exit(ppid);
	}

	if (strcmp(benchmark_cmd[0], "fill_buf") == 0) {
		/* Execute default fill_buf benchmark */
		span = strtoul(benchmark_cmd[1], NULL, 10);
		memflush =  atoi(benchmark_cmd[2]);
		operation = atoi(benchmark_cmd[3]);
		if (!strcmp(benchmark_cmd[4], "true")) {
			once = true;
		} else if (!strcmp(benchmark_cmd[4], "false")) {
			once = false;
		} else {
			ksft_print_msg("Invalid once parameter\n");
			parent_exit(ppid);
		}

		if (run_fill_buf(span, memflush, operation, once))
			fprintf(stderr, "Error in running fill buffer\n");
	} else {
		/* Execute specified benchmark */
		ret = execvp(benchmark_cmd[0], benchmark_cmd);
		if (ret)
			ksft_perror("execvp");
	}

	fclose(stdout);
	ksft_print_msg("Unable to run specified benchmark\n");
	parent_exit(ppid);
}

/*
 * resctrl_val:	execute benchmark and measure memory bandwidth on
 *			the benchmark
 * @test:		test information structure
 * @uparams:		user supplied parameters
 * @benchmark_cmd:	benchmark command and its arguments
 * @param:		parameters passed to resctrl_val()
 *
 * Return:		0 when the test was run, < 0 on error.
 */
int resctrl_val(const struct resctrl_test *test,
		const struct user_params *uparams,
		const char * const *benchmark_cmd,
		struct resctrl_val_param *param)
{
	struct sigaction sigact;
	int ret = 0, pipefd[2];
	char pipe_message = 0;
	union sigval value;
	int domain_id;

	if (strcmp(param->filename, "") == 0)
		sprintf(param->filename, "stdio");

	ret = get_domain_id(test->resource, uparams->cpu, &domain_id);
	if (ret < 0) {
		ksft_print_msg("Could not get domain ID\n");
		return ret;
	}

	/*
	 * If benchmark wasn't successfully started by child, then child should
	 * kill parent, so save parent's pid
	 */
	ppid = getpid();

	if (pipe(pipefd)) {
		ksft_perror("Unable to create pipe");

		return -1;
	}

	/*
	 * Fork to start benchmark, save child's pid so that it can be killed
	 * when needed
	 */
	fflush(stdout);
	bm_pid = fork();
	if (bm_pid == -1) {
		ksft_perror("Unable to fork");

		return -1;
	}

	if (bm_pid == 0) {
		/*
		 * Mask all signals except SIGUSR1, parent uses SIGUSR1 to
		 * start benchmark
		 */
		sigfillset(&sigact.sa_mask);
		sigdelset(&sigact.sa_mask, SIGUSR1);

		sigact.sa_sigaction = run_benchmark;
		sigact.sa_flags = SA_SIGINFO;

		/* Register for "SIGUSR1" signal from parent */
		if (sigaction(SIGUSR1, &sigact, NULL)) {
			ksft_perror("Can't register child for signal");
			parent_exit(ppid);
		}

		/* Tell parent that child is ready */
		close(pipefd[0]);
		pipe_message = 1;
		if (write(pipefd[1], &pipe_message, sizeof(pipe_message)) <
		    sizeof(pipe_message)) {
			ksft_perror("Failed signaling parent process");
			close(pipefd[1]);
			return -1;
		}
		close(pipefd[1]);

		/* Suspend child until delivery of "SIGUSR1" from parent */
		sigsuspend(&sigact.sa_mask);

		ksft_perror("Child is done");
		parent_exit(ppid);
	}

	ksft_print_msg("Benchmark PID: %d\n", (int)bm_pid);

	/*
	 * The cast removes constness but nothing mutates benchmark_cmd within
	 * the context of this process. At the receiving process, it becomes
	 * argv, which is mutable, on exec() but that's after fork() so it
	 * doesn't matter for the process running the tests.
	 */
	value.sival_ptr = (void *)benchmark_cmd;

	/* Taskset benchmark to specified cpu */
	ret = taskset_benchmark(bm_pid, uparams->cpu, NULL);
	if (ret)
		goto out;

	/* Write benchmark to specified control&monitoring grp in resctrl FS */
	ret = write_bm_pid_to_resctrl(bm_pid, param->ctrlgrp, param->mongrp);
	if (ret)
		goto out;

	if (param->init) {
		ret = param->init(param, domain_id);
		if (ret)
			goto out;
	}

	/* Parent waits for child to be ready. */
	close(pipefd[1]);
	while (pipe_message != 1) {
		if (read(pipefd[0], &pipe_message, sizeof(pipe_message)) <
		    sizeof(pipe_message)) {
			ksft_perror("Failed reading message from child process");
			close(pipefd[0]);
			goto out;
		}
	}
	close(pipefd[0]);

	/* Signal child to start benchmark */
	if (sigqueue(bm_pid, SIGUSR1, value) == -1) {
		ksft_perror("sigqueue SIGUSR1 to child");
		ret = -1;
		goto out;
	}

	/* Give benchmark enough time to fully run */
	sleep(1);

	/* Test runs until the callback setup() tells the test to stop. */
	while (1) {
		ret = param->setup(test, uparams, param);
		if (ret == END_OF_TESTS) {
			ret = 0;
			break;
		}
		if (ret < 0)
			break;

		ret = param->measure(uparams, param, bm_pid);
		if (ret)
			break;
	}

out:
	kill(bm_pid, SIGKILL);

	return ret;
}
