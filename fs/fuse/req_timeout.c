// SPDX-License-Identifier: GPL-2.0-only

#include "dev.h"
#include "sysctl.h"
#include "fuse_i.h"
#include "fuse_dev_i.h"
#include "dev_uring_i.h"

/* Frequency (in seconds) of request timeout checks, if opted into */
#define FUSE_TIMEOUT_TIMER_FREQ 15

/* Frequency (in jiffies) of request timeout checks, if opted into */
static const unsigned long fuse_timeout_timer_freq =
	secs_to_jiffies(FUSE_TIMEOUT_TIMER_FREQ);

/*
 * Default timeout (in seconds) for the server to reply to a request
 * before the connection is aborted, if no timeout was specified on mount.
 *
 * Exported via sysctl
 */
unsigned int fuse_default_req_timeout;

/*
 * Max timeout (in seconds) for the server to reply to a request before
 * the connection is aborted.
 *
 * Exported via sysctl
 */
unsigned int fuse_max_req_timeout;

bool fuse_request_expired(struct fuse_conn *fc, struct list_head *list)
{
	struct fuse_req *req;

	req = list_first_entry_or_null(list, struct fuse_req, list);
	if (!req)
		return false;
	return time_is_before_jiffies(req->create_time + fc->timeout.req_timeout);
}

static bool fuse_fpq_processing_expired(struct fuse_conn *fc, struct list_head *processing)
{
	int i;

	for (i = 0; i < FUSE_PQ_HASH_SIZE; i++)
		if (fuse_request_expired(fc, &processing[i]))
			return true;

	return false;
}

/*
 * Check if any requests aren't being completed by the time the request timeout
 * elapses. To do so, we:
 * - check the fiq pending list
 * - check the bg queue
 * - check the fpq io and processing lists
 *
 * To make this fast, we only check against the head request on each list since
 * these are generally queued in order of creation time (eg newer requests get
 * queued to the tail). We might miss a few edge cases (eg requests transitioning
 * between lists, re-sent requests at the head of the pending list having a
 * later creation time than other requests on that list, etc.) but that is fine
 * since if the request never gets fulfilled, it will eventually be caught.
 */
static void fuse_check_timeout(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fuse_conn *fc = container_of(dwork, struct fuse_conn,
					    timeout.work);
	struct fuse_iqueue *fiq = &fc->iq;
	struct fuse_dev *fud;
	struct fuse_pqueue *fpq;
	bool expired = false;

	if (!atomic_read(&fc->num_waiting))
		goto out;

	spin_lock(&fiq->lock);
	expired = fuse_request_expired(fc, &fiq->pending);
	spin_unlock(&fiq->lock);
	if (expired)
		goto abort_conn;

	spin_lock(&fc->bg_lock);
	expired = fuse_request_expired(fc, &fc->bg_queue);
	spin_unlock(&fc->bg_lock);
	if (expired)
		goto abort_conn;

	spin_lock(&fc->lock);
	if (!fc->connected) {
		spin_unlock(&fc->lock);
		return;
	}
	list_for_each_entry(fud, &fc->devices, entry) {
		fpq = &fud->pq;
		spin_lock(&fpq->lock);
		if (fuse_request_expired(fc, &fpq->io) ||
		    fuse_fpq_processing_expired(fc, fpq->processing)) {
			spin_unlock(&fpq->lock);
			spin_unlock(&fc->lock);
			goto abort_conn;
		}

		spin_unlock(&fpq->lock);
	}
	spin_unlock(&fc->lock);

	if (fuse_uring_request_expired(fc))
		goto abort_conn;

out:
	queue_delayed_work(system_percpu_wq, &fc->timeout.work,
			   fuse_timeout_timer_freq);
	return;

abort_conn:
	fuse_abort_conn(fc);
}

static void set_request_timeout(struct fuse_conn *fc, unsigned int timeout)
{
	fc->timeout.req_timeout = secs_to_jiffies(timeout);
	INIT_DELAYED_WORK(&fc->timeout.work, fuse_check_timeout);
	queue_delayed_work(system_percpu_wq, &fc->timeout.work,
			   fuse_timeout_timer_freq);
}

void fuse_init_server_timeout(struct fuse_conn *fc, unsigned int timeout)
{
	if (!timeout && !fuse_max_req_timeout && !fuse_default_req_timeout)
		return;

	if (!timeout)
		timeout = fuse_default_req_timeout;

	if (fuse_max_req_timeout) {
		if (timeout)
			timeout = min(fuse_max_req_timeout, timeout);
		else
			timeout = fuse_max_req_timeout;
	}

	timeout = max(FUSE_TIMEOUT_TIMER_FREQ, timeout);

	set_request_timeout(fc, timeout);
}

