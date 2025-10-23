// SPDX-License-Identifier: GPL-2.0
#include <Python.h>
#include <structmember.h>
#include <inttypes.h>
#include <poll.h>
#include <linux/err.h>
#include <perf/cpumap.h>
#ifdef HAVE_LIBTRACEEVENT
#include <event-parse.h>
#endif
#include <perf/mmap.h>
#include "callchain.h"
#include "counts.h"
#include "evlist.h"
#include "evsel.h"
#include "event.h"
#include "expr.h"
#include "print_binary.h"
#include "record.h"
#include "strbuf.h"
#include "thread_map.h"
#include "tp_pmu.h"
#include "trace-event.h"
#include "metricgroup.h"
#include "mmap.h"
#include "util/sample.h"
#include <internal/lib.h>

PyMODINIT_FUNC PyInit_perf(void);

#define member_def(type, member, ptype, help) \
	{ #member, ptype, \
	  offsetof(struct pyrf_event, event) + offsetof(struct type, member), \
	  0, help }

#define sample_member_def(name, member, ptype, help) \
	{ #name, ptype, \
	  offsetof(struct pyrf_event, sample) + offsetof(struct perf_sample, member), \
	  0, help }

struct pyrf_event {
	PyObject_HEAD
	struct evsel *evsel;
	struct perf_sample sample;
	union perf_event   event;
};

#define sample_members \
	sample_member_def(sample_ip, ip, T_ULONGLONG, "event ip"),			 \
	sample_member_def(sample_pid, pid, T_INT, "event pid"),			 \
	sample_member_def(sample_tid, tid, T_INT, "event tid"),			 \
	sample_member_def(sample_time, time, T_ULONGLONG, "event timestamp"),		 \
	sample_member_def(sample_addr, addr, T_ULONGLONG, "event addr"),		 \
	sample_member_def(sample_id, id, T_ULONGLONG, "event id"),			 \
	sample_member_def(sample_stream_id, stream_id, T_ULONGLONG, "event stream id"), \
	sample_member_def(sample_period, period, T_ULONGLONG, "event period"),		 \
	sample_member_def(sample_cpu, cpu, T_UINT, "event cpu"),

static const char pyrf_mmap_event__doc[] = PyDoc_STR("perf mmap event object.");

static PyMemberDef pyrf_mmap_event__members[] = {
	sample_members
	member_def(perf_event_header, type, T_UINT, "event type"),
	member_def(perf_event_header, misc, T_UINT, "event misc"),
	member_def(perf_record_mmap, pid, T_UINT, "event pid"),
	member_def(perf_record_mmap, tid, T_UINT, "event tid"),
	member_def(perf_record_mmap, start, T_ULONGLONG, "start of the map"),
	member_def(perf_record_mmap, len, T_ULONGLONG, "map length"),
	member_def(perf_record_mmap, pgoff, T_ULONGLONG, "page offset"),
	member_def(perf_record_mmap, filename, T_STRING_INPLACE, "backing store"),
	{ .name = NULL, },
};

static PyObject *pyrf_mmap_event__repr(const struct pyrf_event *pevent)
{
	PyObject *ret;
	char *s;

	if (asprintf(&s, "{ type: mmap, pid: %u, tid: %u, start: %#" PRI_lx64 ", "
			 "length: %#" PRI_lx64 ", offset: %#" PRI_lx64 ", "
			 "filename: %s }",
		     pevent->event.mmap.pid, pevent->event.mmap.tid,
		     pevent->event.mmap.start, pevent->event.mmap.len,
		     pevent->event.mmap.pgoff, pevent->event.mmap.filename) < 0) {
		ret = PyErr_NoMemory();
	} else {
		ret = PyUnicode_FromString(s);
		free(s);
	}
	return ret;
}

static PyTypeObject pyrf_mmap_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.mmap_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_mmap_event__doc,
	.tp_members	= pyrf_mmap_event__members,
	.tp_repr	= (reprfunc)pyrf_mmap_event__repr,
};

static const char pyrf_task_event__doc[] = PyDoc_STR("perf task (fork/exit) event object.");

static PyMemberDef pyrf_task_event__members[] = {
	sample_members
	member_def(perf_event_header, type, T_UINT, "event type"),
	member_def(perf_record_fork, pid, T_UINT, "event pid"),
	member_def(perf_record_fork, ppid, T_UINT, "event ppid"),
	member_def(perf_record_fork, tid, T_UINT, "event tid"),
	member_def(perf_record_fork, ptid, T_UINT, "event ptid"),
	member_def(perf_record_fork, time, T_ULONGLONG, "timestamp"),
	{ .name = NULL, },
};

static PyObject *pyrf_task_event__repr(const struct pyrf_event *pevent)
{
	return PyUnicode_FromFormat("{ type: %s, pid: %u, ppid: %u, tid: %u, "
				   "ptid: %u, time: %" PRI_lu64 "}",
				   pevent->event.header.type == PERF_RECORD_FORK ? "fork" : "exit",
				   pevent->event.fork.pid,
				   pevent->event.fork.ppid,
				   pevent->event.fork.tid,
				   pevent->event.fork.ptid,
				   pevent->event.fork.time);
}

static PyTypeObject pyrf_task_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.task_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_task_event__doc,
	.tp_members	= pyrf_task_event__members,
	.tp_repr	= (reprfunc)pyrf_task_event__repr,
};

static const char pyrf_comm_event__doc[] = PyDoc_STR("perf comm event object.");

static PyMemberDef pyrf_comm_event__members[] = {
	sample_members
	member_def(perf_event_header, type, T_UINT, "event type"),
	member_def(perf_record_comm, pid, T_UINT, "event pid"),
	member_def(perf_record_comm, tid, T_UINT, "event tid"),
	member_def(perf_record_comm, comm, T_STRING_INPLACE, "process name"),
	{ .name = NULL, },
};

static PyObject *pyrf_comm_event__repr(const struct pyrf_event *pevent)
{
	return PyUnicode_FromFormat("{ type: comm, pid: %u, tid: %u, comm: %s }",
				   pevent->event.comm.pid,
				   pevent->event.comm.tid,
				   pevent->event.comm.comm);
}

static PyTypeObject pyrf_comm_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.comm_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_comm_event__doc,
	.tp_members	= pyrf_comm_event__members,
	.tp_repr	= (reprfunc)pyrf_comm_event__repr,
};

static const char pyrf_throttle_event__doc[] = PyDoc_STR("perf throttle event object.");

static PyMemberDef pyrf_throttle_event__members[] = {
	sample_members
	member_def(perf_event_header, type, T_UINT, "event type"),
	member_def(perf_record_throttle, time, T_ULONGLONG, "timestamp"),
	member_def(perf_record_throttle, id, T_ULONGLONG, "event id"),
	member_def(perf_record_throttle, stream_id, T_ULONGLONG, "event stream id"),
	{ .name = NULL, },
};

static PyObject *pyrf_throttle_event__repr(const struct pyrf_event *pevent)
{
	const struct perf_record_throttle *te = (const struct perf_record_throttle *)
		(&pevent->event.header + 1);

	return PyUnicode_FromFormat("{ type: %sthrottle, time: %" PRI_lu64 ", id: %" PRI_lu64
				   ", stream_id: %" PRI_lu64 " }",
				   pevent->event.header.type == PERF_RECORD_THROTTLE ? "" : "un",
				   te->time, te->id, te->stream_id);
}

static PyTypeObject pyrf_throttle_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.throttle_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_throttle_event__doc,
	.tp_members	= pyrf_throttle_event__members,
	.tp_repr	= (reprfunc)pyrf_throttle_event__repr,
};

static const char pyrf_lost_event__doc[] = PyDoc_STR("perf lost event object.");

static PyMemberDef pyrf_lost_event__members[] = {
	sample_members
	member_def(perf_record_lost, id, T_ULONGLONG, "event id"),
	member_def(perf_record_lost, lost, T_ULONGLONG, "number of lost events"),
	{ .name = NULL, },
};

static PyObject *pyrf_lost_event__repr(const struct pyrf_event *pevent)
{
	PyObject *ret;
	char *s;

	if (asprintf(&s, "{ type: lost, id: %#" PRI_lx64 ", "
			 "lost: %#" PRI_lx64 " }",
		     pevent->event.lost.id, pevent->event.lost.lost) < 0) {
		ret = PyErr_NoMemory();
	} else {
		ret = PyUnicode_FromString(s);
		free(s);
	}
	return ret;
}

static PyTypeObject pyrf_lost_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.lost_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_lost_event__doc,
	.tp_members	= pyrf_lost_event__members,
	.tp_repr	= (reprfunc)pyrf_lost_event__repr,
};

static const char pyrf_read_event__doc[] = PyDoc_STR("perf read event object.");

static PyMemberDef pyrf_read_event__members[] = {
	sample_members
	member_def(perf_record_read, pid, T_UINT, "event pid"),
	member_def(perf_record_read, tid, T_UINT, "event tid"),
	{ .name = NULL, },
};

static PyObject *pyrf_read_event__repr(const struct pyrf_event *pevent)
{
	return PyUnicode_FromFormat("{ type: read, pid: %u, tid: %u }",
				   pevent->event.read.pid,
				   pevent->event.read.tid);
	/*
 	 * FIXME: return the array of read values,
 	 * making this method useful ;-)
 	 */
}

static PyTypeObject pyrf_read_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.read_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_read_event__doc,
	.tp_members	= pyrf_read_event__members,
	.tp_repr	= (reprfunc)pyrf_read_event__repr,
};

static const char pyrf_sample_event__doc[] = PyDoc_STR("perf sample event object.");

static PyMemberDef pyrf_sample_event__members[] = {
	sample_members
	member_def(perf_event_header, type, T_UINT, "event type"),
	{ .name = NULL, },
};

static void pyrf_sample_event__delete(struct pyrf_event *pevent)
{
	perf_sample__exit(&pevent->sample);
	Py_TYPE(pevent)->tp_free((PyObject*)pevent);
}

static PyObject *pyrf_sample_event__repr(const struct pyrf_event *pevent)
{
	PyObject *ret;
	char *s;

	if (asprintf(&s, "{ type: sample }") < 0) {
		ret = PyErr_NoMemory();
	} else {
		ret = PyUnicode_FromString(s);
		free(s);
	}
	return ret;
}

#ifdef HAVE_LIBTRACEEVENT
static bool is_tracepoint(const struct pyrf_event *pevent)
{
	return pevent->evsel->core.attr.type == PERF_TYPE_TRACEPOINT;
}

static PyObject*
tracepoint_field(const struct pyrf_event *pe, struct tep_format_field *field)
{
	struct tep_handle *pevent = field->event->tep;
	void *data = pe->sample.raw_data;
	PyObject *ret = NULL;
	unsigned long long val;
	unsigned int offset, len;

	if (field->flags & TEP_FIELD_IS_ARRAY) {
		offset = field->offset;
		len    = field->size;
		if (field->flags & TEP_FIELD_IS_DYNAMIC) {
			val     = tep_read_number(pevent, data + offset, len);
			offset  = val;
			len     = offset >> 16;
			offset &= 0xffff;
			if (tep_field_is_relative(field->flags))
				offset += field->offset + field->size;
		}
		if (field->flags & TEP_FIELD_IS_STRING &&
		    is_printable_array(data + offset, len)) {
			ret = PyUnicode_FromString((char *)data + offset);
		} else {
			ret = PyByteArray_FromStringAndSize((const char *) data + offset, len);
			field->flags &= ~TEP_FIELD_IS_STRING;
		}
	} else {
		val = tep_read_number(pevent, data + field->offset,
				      field->size);
		if (field->flags & TEP_FIELD_IS_POINTER)
			ret = PyLong_FromUnsignedLong((unsigned long) val);
		else if (field->flags & TEP_FIELD_IS_SIGNED)
			ret = PyLong_FromLong((long) val);
		else
			ret = PyLong_FromUnsignedLong((unsigned long) val);
	}

	return ret;
}

static PyObject*
get_tracepoint_field(struct pyrf_event *pevent, PyObject *attr_name)
{
	struct evsel *evsel = pevent->evsel;
	struct tep_event *tp_format = evsel__tp_format(evsel);
	struct tep_format_field *field;

	if (IS_ERR_OR_NULL(tp_format))
		return NULL;

	PyObject *obj = PyObject_Str(attr_name);
	if (obj == NULL)
		return NULL;

	const char *str = PyUnicode_AsUTF8(obj);
	if (str == NULL) {
		Py_DECREF(obj);
		return NULL;
	}

	field = tep_find_any_field(tp_format, str);
	Py_DECREF(obj);
	return field ? tracepoint_field(pevent, field) : NULL;
}
#endif /* HAVE_LIBTRACEEVENT */

static PyObject*
pyrf_sample_event__getattro(struct pyrf_event *pevent, PyObject *attr_name)
{
	PyObject *obj = NULL;

#ifdef HAVE_LIBTRACEEVENT
	if (is_tracepoint(pevent))
		obj = get_tracepoint_field(pevent, attr_name);
#endif

	return obj ?: PyObject_GenericGetAttr((PyObject *) pevent, attr_name);
}

static PyTypeObject pyrf_sample_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.sample_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_sample_event__doc,
	.tp_members	= pyrf_sample_event__members,
	.tp_repr	= (reprfunc)pyrf_sample_event__repr,
	.tp_getattro	= (getattrofunc) pyrf_sample_event__getattro,
};

static const char pyrf_context_switch_event__doc[] = PyDoc_STR("perf context_switch event object.");

static PyMemberDef pyrf_context_switch_event__members[] = {
	sample_members
	member_def(perf_event_header, type, T_UINT, "event type"),
	member_def(perf_record_switch, next_prev_pid, T_UINT, "next/prev pid"),
	member_def(perf_record_switch, next_prev_tid, T_UINT, "next/prev tid"),
	{ .name = NULL, },
};

static PyObject *pyrf_context_switch_event__repr(const struct pyrf_event *pevent)
{
	PyObject *ret;
	char *s;

	if (asprintf(&s, "{ type: context_switch, next_prev_pid: %u, next_prev_tid: %u, switch_out: %u }",
		     pevent->event.context_switch.next_prev_pid,
		     pevent->event.context_switch.next_prev_tid,
		     !!(pevent->event.header.misc & PERF_RECORD_MISC_SWITCH_OUT)) < 0) {
		ret = PyErr_NoMemory();
	} else {
		ret = PyUnicode_FromString(s);
		free(s);
	}
	return ret;
}

static PyTypeObject pyrf_context_switch_event__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.context_switch_event",
	.tp_basicsize	= sizeof(struct pyrf_event),
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_context_switch_event__doc,
	.tp_members	= pyrf_context_switch_event__members,
	.tp_repr	= (reprfunc)pyrf_context_switch_event__repr,
};

static int pyrf_event__setup_types(void)
{
	int err;
	pyrf_mmap_event__type.tp_new =
	pyrf_task_event__type.tp_new =
	pyrf_comm_event__type.tp_new =
	pyrf_lost_event__type.tp_new =
	pyrf_read_event__type.tp_new =
	pyrf_sample_event__type.tp_new =
	pyrf_context_switch_event__type.tp_new =
	pyrf_throttle_event__type.tp_new = PyType_GenericNew;

	pyrf_sample_event__type.tp_dealloc = (destructor)pyrf_sample_event__delete,

	err = PyType_Ready(&pyrf_mmap_event__type);
	if (err < 0)
		goto out;
	err = PyType_Ready(&pyrf_lost_event__type);
	if (err < 0)
		goto out;
	err = PyType_Ready(&pyrf_task_event__type);
	if (err < 0)
		goto out;
	err = PyType_Ready(&pyrf_comm_event__type);
	if (err < 0)
		goto out;
	err = PyType_Ready(&pyrf_throttle_event__type);
	if (err < 0)
		goto out;
	err = PyType_Ready(&pyrf_read_event__type);
	if (err < 0)
		goto out;
	err = PyType_Ready(&pyrf_sample_event__type);
	if (err < 0)
		goto out;
	err = PyType_Ready(&pyrf_context_switch_event__type);
	if (err < 0)
		goto out;
out:
	return err;
}

static PyTypeObject *pyrf_event__type[] = {
	[PERF_RECORD_MMAP]	 = &pyrf_mmap_event__type,
	[PERF_RECORD_LOST]	 = &pyrf_lost_event__type,
	[PERF_RECORD_COMM]	 = &pyrf_comm_event__type,
	[PERF_RECORD_EXIT]	 = &pyrf_task_event__type,
	[PERF_RECORD_THROTTLE]	 = &pyrf_throttle_event__type,
	[PERF_RECORD_UNTHROTTLE] = &pyrf_throttle_event__type,
	[PERF_RECORD_FORK]	 = &pyrf_task_event__type,
	[PERF_RECORD_READ]	 = &pyrf_read_event__type,
	[PERF_RECORD_SAMPLE]	 = &pyrf_sample_event__type,
	[PERF_RECORD_SWITCH]	 = &pyrf_context_switch_event__type,
	[PERF_RECORD_SWITCH_CPU_WIDE]  = &pyrf_context_switch_event__type,
};

static PyObject *pyrf_event__new(const union perf_event *event)
{
	struct pyrf_event *pevent;
	PyTypeObject *ptype;

	if ((event->header.type < PERF_RECORD_MMAP ||
	     event->header.type > PERF_RECORD_SAMPLE) &&
	    !(event->header.type == PERF_RECORD_SWITCH ||
	      event->header.type == PERF_RECORD_SWITCH_CPU_WIDE)) {
		PyErr_Format(PyExc_TypeError, "Unexpected header type %u",
			     event->header.type);
		return NULL;
	}

	// FIXME this better be dynamic or we need to parse everything
	// before calling perf_mmap__consume(), including tracepoint fields.
	if (sizeof(pevent->event) < event->header.size) {
		PyErr_Format(PyExc_TypeError, "Unexpected event size: %zd < %u",
			     sizeof(pevent->event), event->header.size);
		return NULL;
	}

	ptype = pyrf_event__type[event->header.type];
	pevent = PyObject_New(struct pyrf_event, ptype);
	if (pevent != NULL)
		memcpy(&pevent->event, event, event->header.size);
	return (PyObject *)pevent;
}

struct pyrf_cpu_map {
	PyObject_HEAD

	struct perf_cpu_map *cpus;
};

static int pyrf_cpu_map__init(struct pyrf_cpu_map *pcpus,
			      PyObject *args, PyObject *kwargs)
{
	static char *kwlist[] = { "cpustr", NULL };
	char *cpustr = NULL;

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|s",
					 kwlist, &cpustr))
		return -1;

	pcpus->cpus = perf_cpu_map__new(cpustr);
	if (pcpus->cpus == NULL)
		return -1;
	return 0;
}

static void pyrf_cpu_map__delete(struct pyrf_cpu_map *pcpus)
{
	perf_cpu_map__put(pcpus->cpus);
	Py_TYPE(pcpus)->tp_free((PyObject*)pcpus);
}

static Py_ssize_t pyrf_cpu_map__length(PyObject *obj)
{
	struct pyrf_cpu_map *pcpus = (void *)obj;

	return perf_cpu_map__nr(pcpus->cpus);
}

static PyObject *pyrf_cpu_map__item(PyObject *obj, Py_ssize_t i)
{
	struct pyrf_cpu_map *pcpus = (void *)obj;

	if (i >= perf_cpu_map__nr(pcpus->cpus)) {
		PyErr_SetString(PyExc_IndexError, "Index out of range");
		return NULL;
	}

	return Py_BuildValue("i", perf_cpu_map__cpu(pcpus->cpus, i).cpu);
}

static PySequenceMethods pyrf_cpu_map__sequence_methods = {
	.sq_length = pyrf_cpu_map__length,
	.sq_item   = pyrf_cpu_map__item,
};

static const char pyrf_cpu_map__doc[] = PyDoc_STR("cpu map object.");

static PyTypeObject pyrf_cpu_map__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.cpu_map",
	.tp_basicsize	= sizeof(struct pyrf_cpu_map),
	.tp_dealloc	= (destructor)pyrf_cpu_map__delete,
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_cpu_map__doc,
	.tp_as_sequence	= &pyrf_cpu_map__sequence_methods,
	.tp_init	= (initproc)pyrf_cpu_map__init,
};

static int pyrf_cpu_map__setup_types(void)
{
	pyrf_cpu_map__type.tp_new = PyType_GenericNew;
	return PyType_Ready(&pyrf_cpu_map__type);
}

struct pyrf_thread_map {
	PyObject_HEAD

	struct perf_thread_map *threads;
};

static int pyrf_thread_map__init(struct pyrf_thread_map *pthreads,
				 PyObject *args, PyObject *kwargs)
{
	static char *kwlist[] = { "pid", "tid", NULL };
	int pid = -1, tid = -1;

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|ii",
					 kwlist, &pid, &tid))
		return -1;

	pthreads->threads = thread_map__new(pid, tid);
	if (pthreads->threads == NULL)
		return -1;
	return 0;
}

static void pyrf_thread_map__delete(struct pyrf_thread_map *pthreads)
{
	perf_thread_map__put(pthreads->threads);
	Py_TYPE(pthreads)->tp_free((PyObject*)pthreads);
}

static Py_ssize_t pyrf_thread_map__length(PyObject *obj)
{
	struct pyrf_thread_map *pthreads = (void *)obj;

	return perf_thread_map__nr(pthreads->threads);
}

static PyObject *pyrf_thread_map__item(PyObject *obj, Py_ssize_t i)
{
	struct pyrf_thread_map *pthreads = (void *)obj;

	if (i >= perf_thread_map__nr(pthreads->threads)) {
		PyErr_SetString(PyExc_IndexError, "Index out of range");
		return NULL;
	}

	return Py_BuildValue("i", perf_thread_map__pid(pthreads->threads, i));
}

static PySequenceMethods pyrf_thread_map__sequence_methods = {
	.sq_length = pyrf_thread_map__length,
	.sq_item   = pyrf_thread_map__item,
};

static const char pyrf_thread_map__doc[] = PyDoc_STR("thread map object.");

static PyTypeObject pyrf_thread_map__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.thread_map",
	.tp_basicsize	= sizeof(struct pyrf_thread_map),
	.tp_dealloc	= (destructor)pyrf_thread_map__delete,
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_thread_map__doc,
	.tp_as_sequence	= &pyrf_thread_map__sequence_methods,
	.tp_init	= (initproc)pyrf_thread_map__init,
};

static int pyrf_thread_map__setup_types(void)
{
	pyrf_thread_map__type.tp_new = PyType_GenericNew;
	return PyType_Ready(&pyrf_thread_map__type);
}

/**
 * A python wrapper for perf_pmus that are globally owned by the pmus.c code.
 */
struct pyrf_pmu {
	PyObject_HEAD

	struct perf_pmu *pmu;
};

static void pyrf_pmu__delete(struct pyrf_pmu *ppmu)
{
	Py_TYPE(ppmu)->tp_free((PyObject *)ppmu);
}

static PyObject *pyrf_pmu__name(PyObject *self)
{
	struct pyrf_pmu *ppmu = (void *)self;

	return PyUnicode_FromString(ppmu->pmu->name);
}

static bool add_to_dict(PyObject *dict, const char *key, const char *value)
{
	PyObject *pkey, *pvalue;
	bool ret;

	if (value == NULL)
		return true;

	pkey = PyUnicode_FromString(key);
	pvalue = PyUnicode_FromString(value);

	ret = pkey && pvalue && PyDict_SetItem(dict, pkey, pvalue) == 0;
	Py_XDECREF(pkey);
	Py_XDECREF(pvalue);
	return ret;
}

static int pyrf_pmu__events_cb(void *state, struct pmu_event_info *info)
{
	PyObject *py_list = state;
	PyObject *dict = PyDict_New();

	if (!dict)
		return -ENOMEM;

	if (!add_to_dict(dict, "name", info->name) ||
	    !add_to_dict(dict, "alias", info->alias) ||
	    !add_to_dict(dict, "scale_unit", info->scale_unit) ||
	    !add_to_dict(dict, "desc", info->desc) ||
	    !add_to_dict(dict, "long_desc", info->long_desc) ||
	    !add_to_dict(dict, "encoding_desc", info->encoding_desc) ||
	    !add_to_dict(dict, "topic", info->topic) ||
	    !add_to_dict(dict, "event_type_desc", info->event_type_desc) ||
	    !add_to_dict(dict, "str", info->str) ||
	    !add_to_dict(dict, "deprecated", info->deprecated ? "deprecated" : NULL) ||
	    PyList_Append(py_list, dict) != 0) {
		Py_DECREF(dict);
		return -ENOMEM;
	}
	Py_DECREF(dict);
	return 0;
}

static PyObject *pyrf_pmu__events(PyObject *self)
{
	struct pyrf_pmu *ppmu = (void *)self;
	PyObject *py_list = PyList_New(0);
	int ret;

	if (!py_list)
		return NULL;

	ret = perf_pmu__for_each_event(ppmu->pmu,
				       /*skip_duplicate_pmus=*/false,
				       py_list,
				       pyrf_pmu__events_cb);
	if (ret) {
		Py_DECREF(py_list);
		errno = -ret;
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}
	return py_list;
}

static PyObject *pyrf_pmu__repr(PyObject *self)
{
	struct pyrf_pmu *ppmu = (void *)self;

	return PyUnicode_FromFormat("pmu(%s)", ppmu->pmu->name);
}

static const char pyrf_pmu__doc[] = PyDoc_STR("perf Performance Monitoring Unit (PMU) object.");

static PyMethodDef pyrf_pmu__methods[] = {
	{
		.ml_name  = "events",
		.ml_meth  = (PyCFunction)pyrf_pmu__events,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("Returns a sequence of events encoded as a dictionaries.")
	},
	{
		.ml_name  = "name",
		.ml_meth  = (PyCFunction)pyrf_pmu__name,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("Name of the PMU including suffixes.")
	},
	{ .ml_name = NULL, }
};

/** The python type for a perf.pmu. */
static PyTypeObject pyrf_pmu__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.pmu",
	.tp_basicsize	= sizeof(struct pyrf_pmu),
	.tp_dealloc	= (destructor)pyrf_pmu__delete,
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_pmu__doc,
	.tp_methods	= pyrf_pmu__methods,
	.tp_str         = pyrf_pmu__name,
	.tp_repr        = pyrf_pmu__repr,
};

static int pyrf_pmu__setup_types(void)
{
	pyrf_pmu__type.tp_new = PyType_GenericNew;
	return PyType_Ready(&pyrf_pmu__type);
}


/** A python iterator for pmus that has no equivalent in the C code. */
struct pyrf_pmu_iterator {
	PyObject_HEAD
	struct perf_pmu *pmu;
};

static void pyrf_pmu_iterator__dealloc(struct pyrf_pmu_iterator *self)
{
	Py_TYPE(self)->tp_free((PyObject *) self);
}

static PyObject *pyrf_pmu_iterator__new(PyTypeObject *type, PyObject *args __maybe_unused,
					PyObject *kwds __maybe_unused)
{
	struct pyrf_pmu_iterator *itr = (void *)type->tp_alloc(type, 0);

	if (itr != NULL)
		itr->pmu = perf_pmus__scan(/*pmu=*/NULL);

	return (PyObject *) itr;
}

static PyObject *pyrf_pmu_iterator__iter(PyObject *self)
{
	Py_INCREF(self);
	return self;
}

static PyObject *pyrf_pmu_iterator__iternext(PyObject *self)
{
	struct pyrf_pmu_iterator *itr = (void *)self;
	struct pyrf_pmu *ppmu;

	if (itr->pmu == NULL) {
		PyErr_SetNone(PyExc_StopIteration);
		return NULL;
	}
	// Create object to return.
	ppmu = PyObject_New(struct pyrf_pmu, &pyrf_pmu__type);
	if (ppmu) {
		ppmu->pmu = itr->pmu;
		// Advance iterator.
		itr->pmu = perf_pmus__scan(itr->pmu);
	}
	return (PyObject *)ppmu;
}

/** The python type for the PMU iterator. */
static PyTypeObject pyrf_pmu_iterator__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name = "pmus.iterator",
	.tp_doc = "Iterator for the pmus string sequence.",
	.tp_basicsize = sizeof(struct pyrf_pmu_iterator),
	.tp_itemsize = 0,
	.tp_flags = Py_TPFLAGS_DEFAULT,
	.tp_new = pyrf_pmu_iterator__new,
	.tp_dealloc = (destructor) pyrf_pmu_iterator__dealloc,
	.tp_iter = pyrf_pmu_iterator__iter,
	.tp_iternext = pyrf_pmu_iterator__iternext,
};

static int pyrf_pmu_iterator__setup_types(void)
{
	return PyType_Ready(&pyrf_pmu_iterator__type);
}

static PyObject *pyrf__pmus(PyObject *self, PyObject *args)
{
	// Calling the class creates an instance of the iterator.
	return PyObject_CallObject((PyObject *) &pyrf_pmu_iterator__type, /*args=*/NULL);
}

struct pyrf_counts_values {
	PyObject_HEAD

	struct perf_counts_values values;
};

static const char pyrf_counts_values__doc[] = PyDoc_STR("perf counts values object.");

static void pyrf_counts_values__delete(struct pyrf_counts_values *pcounts_values)
{
	Py_TYPE(pcounts_values)->tp_free((PyObject *)pcounts_values);
}

#define counts_values_member_def(member, ptype, help) \
	{ #member, ptype, \
	  offsetof(struct pyrf_counts_values, values.member), \
	  0, help }

static PyMemberDef pyrf_counts_values_members[] = {
	counts_values_member_def(val, T_ULONG, "Value of event"),
	counts_values_member_def(ena, T_ULONG, "Time for which enabled"),
	counts_values_member_def(run, T_ULONG, "Time for which running"),
	counts_values_member_def(id, T_ULONG, "Unique ID for an event"),
	counts_values_member_def(lost, T_ULONG, "Num of lost samples"),
	{ .name = NULL, },
};

static PyObject *pyrf_counts_values_get_values(struct pyrf_counts_values *self, void *closure)
{
	PyObject *vals = PyList_New(5);

	if (!vals)
		return NULL;
	for (int i = 0; i < 5; i++)
		PyList_SetItem(vals, i, PyLong_FromLong(self->values.values[i]));

	return vals;
}

static int pyrf_counts_values_set_values(struct pyrf_counts_values *self, PyObject *list,
					 void *closure)
{
	Py_ssize_t size;
	PyObject *item = NULL;

	if (!PyList_Check(list)) {
		PyErr_SetString(PyExc_TypeError, "Value assigned must be a list");
		return -1;
	}

	size = PyList_Size(list);
	for (Py_ssize_t i = 0; i < size; i++) {
		item = PyList_GetItem(list, i);
		if (!PyLong_Check(item)) {
			PyErr_SetString(PyExc_TypeError, "List members should be numbers");
			return -1;
		}
		self->values.values[i] = PyLong_AsLong(item);
	}

	return 0;
}

static PyGetSetDef pyrf_counts_values_getset[] = {
	{"values", (getter)pyrf_counts_values_get_values, (setter)pyrf_counts_values_set_values,
		"Name field", NULL},
	{ .name = NULL, },
};

static PyTypeObject pyrf_counts_values__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.counts_values",
	.tp_basicsize	= sizeof(struct pyrf_counts_values),
	.tp_dealloc	= (destructor)pyrf_counts_values__delete,
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_counts_values__doc,
	.tp_members	= pyrf_counts_values_members,
	.tp_getset	= pyrf_counts_values_getset,
};

static int pyrf_counts_values__setup_types(void)
{
	pyrf_counts_values__type.tp_new = PyType_GenericNew;
	return PyType_Ready(&pyrf_counts_values__type);
}

struct pyrf_evsel {
	PyObject_HEAD

	struct evsel evsel;
};

static int pyrf_evsel__init(struct pyrf_evsel *pevsel,
			    PyObject *args, PyObject *kwargs)
{
	struct perf_event_attr attr = {
		.type = PERF_TYPE_HARDWARE,
		.config = PERF_COUNT_HW_CPU_CYCLES,
		.sample_type = PERF_SAMPLE_PERIOD | PERF_SAMPLE_TID,
	};
	static char *kwlist[] = {
		"type",
		"config",
		"sample_freq",
		"sample_period",
		"sample_type",
		"read_format",
		"disabled",
		"inherit",
		"pinned",
		"exclusive",
		"exclude_user",
		"exclude_kernel",
		"exclude_hv",
		"exclude_idle",
		"mmap",
		"context_switch",
		"comm",
		"freq",
		"inherit_stat",
		"enable_on_exec",
		"task",
		"watermark",
		"precise_ip",
		"mmap_data",
		"sample_id_all",
		"wakeup_events",
		"bp_type",
		"bp_addr",
		"bp_len",
		 NULL
	};
	u64 sample_period = 0;
	u32 disabled = 0,
	    inherit = 0,
	    pinned = 0,
	    exclusive = 0,
	    exclude_user = 0,
	    exclude_kernel = 0,
	    exclude_hv = 0,
	    exclude_idle = 0,
	    mmap = 0,
	    context_switch = 0,
	    comm = 0,
	    freq = 1,
	    inherit_stat = 0,
	    enable_on_exec = 0,
	    task = 0,
	    watermark = 0,
	    precise_ip = 0,
	    mmap_data = 0,
	    sample_id_all = 1;
	int idx = 0;

	if (!PyArg_ParseTupleAndKeywords(args, kwargs,
					 "|iKiKKiiiiiiiiiiiiiiiiiiiiiiKK", kwlist,
					 &attr.type, &attr.config, &attr.sample_freq,
					 &sample_period, &attr.sample_type,
					 &attr.read_format, &disabled, &inherit,
					 &pinned, &exclusive, &exclude_user,
					 &exclude_kernel, &exclude_hv, &exclude_idle,
					 &mmap, &context_switch, &comm, &freq, &inherit_stat,
					 &enable_on_exec, &task, &watermark,
					 &precise_ip, &mmap_data, &sample_id_all,
					 &attr.wakeup_events, &attr.bp_type,
					 &attr.bp_addr, &attr.bp_len, &idx))
		return -1;

	/* union... */
	if (sample_period != 0) {
		if (attr.sample_freq != 0)
			return -1; /* FIXME: throw right exception */
		attr.sample_period = sample_period;
	}

	/* Bitfields */
	attr.disabled	    = disabled;
	attr.inherit	    = inherit;
	attr.pinned	    = pinned;
	attr.exclusive	    = exclusive;
	attr.exclude_user   = exclude_user;
	attr.exclude_kernel = exclude_kernel;
	attr.exclude_hv	    = exclude_hv;
	attr.exclude_idle   = exclude_idle;
	attr.mmap	    = mmap;
	attr.context_switch = context_switch;
	attr.comm	    = comm;
	attr.freq	    = freq;
	attr.inherit_stat   = inherit_stat;
	attr.enable_on_exec = enable_on_exec;
	attr.task	    = task;
	attr.watermark	    = watermark;
	attr.precise_ip	    = precise_ip;
	attr.mmap_data	    = mmap_data;
	attr.sample_id_all  = sample_id_all;
	attr.size	    = sizeof(attr);

	evsel__init(&pevsel->evsel, &attr, idx);
	return 0;
}

static void pyrf_evsel__delete(struct pyrf_evsel *pevsel)
{
	evsel__exit(&pevsel->evsel);
	Py_TYPE(pevsel)->tp_free((PyObject*)pevsel);
}

static PyObject *pyrf_evsel__open(struct pyrf_evsel *pevsel,
				  PyObject *args, PyObject *kwargs)
{
	struct evsel *evsel = &pevsel->evsel;
	struct perf_cpu_map *cpus = NULL;
	struct perf_thread_map *threads = NULL;
	PyObject *pcpus = NULL, *pthreads = NULL;
	int group = 0, inherit = 0;
	static char *kwlist[] = { "cpus", "threads", "group", "inherit", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|OOii", kwlist,
					 &pcpus, &pthreads, &group, &inherit))
		return NULL;

	if (pthreads != NULL)
		threads = ((struct pyrf_thread_map *)pthreads)->threads;

	if (pcpus != NULL)
		cpus = ((struct pyrf_cpu_map *)pcpus)->cpus;

	evsel->core.attr.inherit = inherit;
	/*
	 * This will group just the fds for this single evsel, to group
	 * multiple events, use evlist.open().
	 */
	if (evsel__open(evsel, cpus, threads) < 0) {
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *pyrf_evsel__cpus(struct pyrf_evsel *pevsel)
{
	struct pyrf_cpu_map *pcpu_map = PyObject_New(struct pyrf_cpu_map, &pyrf_cpu_map__type);

	if (pcpu_map)
		pcpu_map->cpus = perf_cpu_map__get(pevsel->evsel.core.cpus);

	return (PyObject *)pcpu_map;
}

static PyObject *pyrf_evsel__threads(struct pyrf_evsel *pevsel)
{
	struct pyrf_thread_map *pthread_map =
		PyObject_New(struct pyrf_thread_map, &pyrf_thread_map__type);

	if (pthread_map)
		pthread_map->threads = perf_thread_map__get(pevsel->evsel.core.threads);

	return (PyObject *)pthread_map;
}

/*
 * Ensure evsel's counts and prev_raw_counts are allocated, the latter
 * used by tool PMUs to compute the cumulative count as expected by
 * stat's process_counter_values.
 */
static int evsel__ensure_counts(struct evsel *evsel)
{
	int nthreads, ncpus;

	if (evsel->counts != NULL)
		return 0;

	nthreads = perf_thread_map__nr(evsel->core.threads);
	ncpus = perf_cpu_map__nr(evsel->core.cpus);

	evsel->counts = perf_counts__new(ncpus, nthreads);
	if (evsel->counts == NULL)
		return -ENOMEM;

	evsel->prev_raw_counts = perf_counts__new(ncpus, nthreads);
	if (evsel->prev_raw_counts == NULL)
		return -ENOMEM;

	return 0;
}

static PyObject *pyrf_evsel__read(struct pyrf_evsel *pevsel,
				  PyObject *args, PyObject *kwargs)
{
	struct evsel *evsel = &pevsel->evsel;
	int cpu = 0, cpu_idx, thread = 0, thread_idx;
	struct perf_counts_values *old_count, *new_count;
	struct pyrf_counts_values *count_values = PyObject_New(struct pyrf_counts_values,
							       &pyrf_counts_values__type);

	if (!count_values)
		return NULL;

	if (!PyArg_ParseTuple(args, "ii", &cpu, &thread))
		return NULL;

	cpu_idx = perf_cpu_map__idx(evsel->core.cpus, (struct perf_cpu){.cpu = cpu});
	if (cpu_idx < 0) {
		PyErr_Format(PyExc_TypeError, "CPU %d is not part of evsel's CPUs", cpu);
		return NULL;
	}
	thread_idx = perf_thread_map__idx(evsel->core.threads, thread);
	if (thread_idx < 0) {
		PyErr_Format(PyExc_TypeError, "Thread %d is not part of evsel's threads",
			     thread);
		return NULL;
	}

	if (evsel__ensure_counts(evsel))
		return PyErr_NoMemory();

	/* Set up pointers to the old and newly read counter values. */
	old_count = perf_counts(evsel->prev_raw_counts, cpu_idx, thread_idx);
	new_count = perf_counts(evsel->counts, cpu_idx, thread_idx);
	/* Update the value in evsel->counts. */
	evsel__read_counter(evsel, cpu_idx, thread_idx);
	/* Copy the value and turn it into the delta from old_count. */
	count_values->values = *new_count;
	count_values->values.val -= old_count->val;
	count_values->values.ena -= old_count->ena;
	count_values->values.run -= old_count->run;
	/* Save the new count over the old_count for the next read. */
	*old_count = *new_count;
	return (PyObject *)count_values;
}

static PyObject *pyrf_evsel__str(PyObject *self)
{
	struct pyrf_evsel *pevsel = (void *)self;
	struct evsel *evsel = &pevsel->evsel;

	return PyUnicode_FromFormat("evsel(%s/%s/)", evsel__pmu_name(evsel), evsel__name(evsel));
}

static PyMethodDef pyrf_evsel__methods[] = {
	{
		.ml_name  = "open",
		.ml_meth  = (PyCFunction)pyrf_evsel__open,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("open the event selector file descriptor table.")
	},
	{
		.ml_name  = "cpus",
		.ml_meth  = (PyCFunction)pyrf_evsel__cpus,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("CPUs the event is to be used with.")
	},
	{
		.ml_name  = "threads",
		.ml_meth  = (PyCFunction)pyrf_evsel__threads,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("threads the event is to be used with.")
	},
	{
		.ml_name  = "read",
		.ml_meth  = (PyCFunction)pyrf_evsel__read,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("read counters")
	},
	{ .ml_name = NULL, }
};

#define evsel_member_def(member, ptype, help) \
	{ #member, ptype, \
	  offsetof(struct pyrf_evsel, evsel.member), \
	  0, help }

#define evsel_attr_member_def(member, ptype, help) \
	{ #member, ptype, \
	  offsetof(struct pyrf_evsel, evsel.core.attr.member), \
	  0, help }

static PyMemberDef pyrf_evsel__members[] = {
	evsel_member_def(tracking, T_BOOL, "tracking event."),
	evsel_attr_member_def(type, T_UINT, "attribute type."),
	evsel_attr_member_def(size, T_UINT, "attribute size."),
	evsel_attr_member_def(config, T_ULONGLONG, "attribute config."),
	evsel_attr_member_def(sample_period, T_ULONGLONG, "attribute sample_period."),
	evsel_attr_member_def(sample_type, T_ULONGLONG, "attribute sample_type."),
	evsel_attr_member_def(read_format, T_ULONGLONG, "attribute read_format."),
	evsel_attr_member_def(wakeup_events, T_UINT, "attribute wakeup_events."),
	{ .name = NULL, },
};

static const char pyrf_evsel__doc[] = PyDoc_STR("perf event selector list object.");

static PyTypeObject pyrf_evsel__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.evsel",
	.tp_basicsize	= sizeof(struct pyrf_evsel),
	.tp_dealloc	= (destructor)pyrf_evsel__delete,
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_doc		= pyrf_evsel__doc,
	.tp_members	= pyrf_evsel__members,
	.tp_methods	= pyrf_evsel__methods,
	.tp_init	= (initproc)pyrf_evsel__init,
	.tp_str         = pyrf_evsel__str,
	.tp_repr        = pyrf_evsel__str,
};

static int pyrf_evsel__setup_types(void)
{
	pyrf_evsel__type.tp_new = PyType_GenericNew;
	return PyType_Ready(&pyrf_evsel__type);
}

struct pyrf_evlist {
	PyObject_HEAD

	struct evlist evlist;
};

static int pyrf_evlist__init(struct pyrf_evlist *pevlist,
			     PyObject *args, PyObject *kwargs __maybe_unused)
{
	PyObject *pcpus = NULL, *pthreads = NULL;
	struct perf_cpu_map *cpus;
	struct perf_thread_map *threads;

	if (!PyArg_ParseTuple(args, "OO", &pcpus, &pthreads))
		return -1;

	threads = ((struct pyrf_thread_map *)pthreads)->threads;
	cpus = ((struct pyrf_cpu_map *)pcpus)->cpus;
	evlist__init(&pevlist->evlist, cpus, threads);
	return 0;
}

static void pyrf_evlist__delete(struct pyrf_evlist *pevlist)
{
	evlist__exit(&pevlist->evlist);
	Py_TYPE(pevlist)->tp_free((PyObject*)pevlist);
}

static PyObject *pyrf_evlist__all_cpus(struct pyrf_evlist *pevlist)
{
	struct pyrf_cpu_map *pcpu_map = PyObject_New(struct pyrf_cpu_map, &pyrf_cpu_map__type);

	if (pcpu_map)
		pcpu_map->cpus = perf_cpu_map__get(pevlist->evlist.core.all_cpus);

	return (PyObject *)pcpu_map;
}

static PyObject *pyrf_evlist__metrics(struct pyrf_evlist *pevlist)
{
	PyObject *list = PyList_New(/*len=*/0);
	struct rb_node *node;

	if (!list)
		return NULL;

	for (node = rb_first_cached(&pevlist->evlist.metric_events.entries); node;
	     node = rb_next(node)) {
		struct metric_event *me = container_of(node, struct metric_event, nd);
		struct list_head *pos;

		list_for_each(pos, &me->head) {
			struct metric_expr *expr = container_of(pos, struct metric_expr, nd);
			PyObject *str = PyUnicode_FromString(expr->metric_name);

			if (!str || PyList_Append(list, str) != 0) {
				Py_DECREF(list);
				return NULL;
			}
			Py_DECREF(str);
		}
	}
	return list;
}

static int prepare_metric(const struct metric_expr *mexp,
			  const struct evsel *evsel,
			  struct expr_parse_ctx *pctx,
			  int cpu_idx, int thread_idx)
{
	struct evsel * const *metric_events = mexp->metric_events;
	struct metric_ref *metric_refs = mexp->metric_refs;

	for (int i = 0; metric_events[i]; i++) {
		char *n = strdup(evsel__metric_id(metric_events[i]));
		double val, ena, run;
		int source_count = evsel__source_count(metric_events[i]);
		int ret;
		struct perf_counts_values *old_count, *new_count;

		if (!n)
			return -ENOMEM;

		if (source_count == 0)
			source_count = 1;

		ret = evsel__ensure_counts(metric_events[i]);
		if (ret)
			return ret;

		/* Set up pointers to the old and newly read counter values. */
		old_count = perf_counts(metric_events[i]->prev_raw_counts, cpu_idx, thread_idx);
		new_count = perf_counts(metric_events[i]->counts, cpu_idx, thread_idx);
		/* Update the value in metric_events[i]->counts. */
		evsel__read_counter(metric_events[i], cpu_idx, thread_idx);

		val = new_count->val - old_count->val;
		ena = new_count->ena - old_count->ena;
		run = new_count->run - old_count->run;

		if (ena != run && run != 0)
			val = val * ena / run;
		ret = expr__add_id_val_source_count(pctx, n, val, source_count);
		if (ret)
			return ret;
	}

	for (int i = 0; metric_refs && metric_refs[i].metric_name; i++) {
		int ret = expr__add_ref(pctx, &metric_refs[i]);

		if (ret)
			return ret;
	}

	return 0;
}

static PyObject *pyrf_evlist__compute_metric(struct pyrf_evlist *pevlist,
					     PyObject *args, PyObject *kwargs)
{
	int ret, cpu = 0, cpu_idx = 0, thread = 0, thread_idx = 0;
	const char *metric;
	struct rb_node *node;
	struct metric_expr *mexp = NULL;
	struct expr_parse_ctx *pctx;
	double result = 0;

	if (!PyArg_ParseTuple(args, "sii", &metric, &cpu, &thread))
		return NULL;

	for (node = rb_first_cached(&pevlist->evlist.metric_events.entries);
	     mexp == NULL && node;
	     node = rb_next(node)) {
		struct metric_event *me = container_of(node, struct metric_event, nd);
		struct list_head *pos;

		list_for_each(pos, &me->head) {
			struct metric_expr *e = container_of(pos, struct metric_expr, nd);

			if (strcmp(e->metric_name, metric))
				continue;

			if (e->metric_events[0] == NULL)
				continue;

			cpu_idx = perf_cpu_map__idx(e->metric_events[0]->core.cpus,
						    (struct perf_cpu){.cpu = cpu});
			if (cpu_idx < 0)
				continue;

			thread_idx = perf_thread_map__idx(e->metric_events[0]->core.threads,
							  thread);
			if (thread_idx < 0)
				continue;

			mexp = e;
			break;
		}
	}
	if (!mexp) {
		PyErr_Format(PyExc_TypeError, "Unknown metric '%s' for CPU '%d' and thread '%d'",
			     metric, cpu, thread);
		return NULL;
	}

	pctx = expr__ctx_new();
	if (!pctx)
		return PyErr_NoMemory();

	ret = prepare_metric(mexp, mexp->metric_events[0], pctx, cpu_idx, thread_idx);
	if (ret) {
		expr__ctx_free(pctx);
		errno = -ret;
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}
	if (expr__parse(&result, pctx, mexp->metric_expr))
		result = 0.0;

	expr__ctx_free(pctx);
	return PyFloat_FromDouble(result);
}

static PyObject *pyrf_evlist__mmap(struct pyrf_evlist *pevlist,
				   PyObject *args, PyObject *kwargs)
{
	struct evlist *evlist = &pevlist->evlist;
	static char *kwlist[] = { "pages", "overwrite", NULL };
	int pages = 128, overwrite = false;

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|ii", kwlist,
					 &pages, &overwrite))
		return NULL;

	if (evlist__mmap(evlist, pages) < 0) {
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *pyrf_evlist__poll(struct pyrf_evlist *pevlist,
				   PyObject *args, PyObject *kwargs)
{
	struct evlist *evlist = &pevlist->evlist;
	static char *kwlist[] = { "timeout", NULL };
	int timeout = -1, n;

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|i", kwlist, &timeout))
		return NULL;

	n = evlist__poll(evlist, timeout);
	if (n < 0) {
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}

	return Py_BuildValue("i", n);
}

static PyObject *pyrf_evlist__get_pollfd(struct pyrf_evlist *pevlist,
					 PyObject *args __maybe_unused,
					 PyObject *kwargs __maybe_unused)
{
	struct evlist *evlist = &pevlist->evlist;
        PyObject *list = PyList_New(0);
	int i;

	for (i = 0; i < evlist->core.pollfd.nr; ++i) {
		PyObject *file;
		file = PyFile_FromFd(evlist->core.pollfd.entries[i].fd, "perf", "r", -1,
				     NULL, NULL, NULL, 0);
		if (file == NULL)
			goto free_list;

		if (PyList_Append(list, file) != 0) {
			Py_DECREF(file);
			goto free_list;
		}

		Py_DECREF(file);
	}

	return list;
free_list:
	return PyErr_NoMemory();
}


static PyObject *pyrf_evlist__add(struct pyrf_evlist *pevlist,
				  PyObject *args,
				  PyObject *kwargs __maybe_unused)
{
	struct evlist *evlist = &pevlist->evlist;
	PyObject *pevsel;
	struct evsel *evsel;

	if (!PyArg_ParseTuple(args, "O", &pevsel))
		return NULL;

	Py_INCREF(pevsel);
	evsel = &((struct pyrf_evsel *)pevsel)->evsel;
	evsel->core.idx = evlist->core.nr_entries;
	evlist__add(evlist, evsel);

	return Py_BuildValue("i", evlist->core.nr_entries);
}

static struct mmap *get_md(struct evlist *evlist, int cpu)
{
	int i;

	for (i = 0; i < evlist->core.nr_mmaps; i++) {
		struct mmap *md = &evlist->mmap[i];

		if (md->core.cpu.cpu == cpu)
			return md;
	}

	return NULL;
}

static PyObject *pyrf_evlist__read_on_cpu(struct pyrf_evlist *pevlist,
					  PyObject *args, PyObject *kwargs)
{
	struct evlist *evlist = &pevlist->evlist;
	union perf_event *event;
	int sample_id_all = 1, cpu;
	static char *kwlist[] = { "cpu", "sample_id_all", NULL };
	struct mmap *md;
	int err;

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "i|i", kwlist,
					 &cpu, &sample_id_all))
		return NULL;

	md = get_md(evlist, cpu);
	if (!md) {
		PyErr_Format(PyExc_TypeError, "Unknown CPU '%d'", cpu);
		return NULL;
	}

	if (perf_mmap__read_init(&md->core) < 0)
		goto end;

	event = perf_mmap__read_event(&md->core);
	if (event != NULL) {
		PyObject *pyevent = pyrf_event__new(event);
		struct pyrf_event *pevent = (struct pyrf_event *)pyevent;
		struct evsel *evsel;

		if (pyevent == NULL)
			return PyErr_NoMemory();

		evsel = evlist__event2evsel(evlist, event);
		if (!evsel) {
			Py_DECREF(pyevent);
			Py_INCREF(Py_None);
			return Py_None;
		}

		pevent->evsel = evsel;

		perf_mmap__consume(&md->core);

		err = evsel__parse_sample(evsel, &pevent->event, &pevent->sample);
		if (err) {
			Py_DECREF(pyevent);
			return PyErr_Format(PyExc_OSError,
					    "perf: can't parse sample, err=%d", err);
		}

		return pyevent;
	}
end:
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *pyrf_evlist__open(struct pyrf_evlist *pevlist,
				   PyObject *args, PyObject *kwargs)
{
	struct evlist *evlist = &pevlist->evlist;

	if (evlist__open(evlist) < 0) {
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *pyrf_evlist__close(struct pyrf_evlist *pevlist)
{
	struct evlist *evlist = &pevlist->evlist;

	evlist__close(evlist);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *pyrf_evlist__config(struct pyrf_evlist *pevlist)
{
	struct record_opts opts = {
		.sample_time	     = true,
		.mmap_pages	     = UINT_MAX,
		.user_freq	     = UINT_MAX,
		.user_interval	     = ULLONG_MAX,
		.freq		     = 4000,
		.target		     = {
			.uses_mmap   = true,
			.default_per_cpu = true,
		},
		.nr_threads_synthesize = 1,
		.ctl_fd              = -1,
		.ctl_fd_ack          = -1,
		.no_buffering        = true,
		.no_inherit          = true,
	};
	struct evlist *evlist = &pevlist->evlist;

	evlist__config(evlist, &opts, &callchain_param);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *pyrf_evlist__disable(struct pyrf_evlist *pevlist)
{
	evlist__disable(&pevlist->evlist);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *pyrf_evlist__enable(struct pyrf_evlist *pevlist)
{
	evlist__enable(&pevlist->evlist);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyMethodDef pyrf_evlist__methods[] = {
	{
		.ml_name  = "all_cpus",
		.ml_meth  = (PyCFunction)pyrf_evlist__all_cpus,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("CPU map union of all evsel CPU maps.")
	},
	{
		.ml_name  = "metrics",
		.ml_meth  = (PyCFunction)pyrf_evlist__metrics,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("List of metric names within the evlist.")
	},
	{
		.ml_name  = "compute_metric",
		.ml_meth  = (PyCFunction)pyrf_evlist__compute_metric,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("compute metric for given name, cpu and thread")
	},
	{
		.ml_name  = "mmap",
		.ml_meth  = (PyCFunction)pyrf_evlist__mmap,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("mmap the file descriptor table.")
	},
	{
		.ml_name  = "open",
		.ml_meth  = (PyCFunction)pyrf_evlist__open,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("open the file descriptors.")
	},
	{
		.ml_name  = "close",
		.ml_meth  = (PyCFunction)pyrf_evlist__close,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("close the file descriptors.")
	},
	{
		.ml_name  = "poll",
		.ml_meth  = (PyCFunction)pyrf_evlist__poll,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("poll the file descriptor table.")
	},
	{
		.ml_name  = "get_pollfd",
		.ml_meth  = (PyCFunction)pyrf_evlist__get_pollfd,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("get the poll file descriptor table.")
	},
	{
		.ml_name  = "add",
		.ml_meth  = (PyCFunction)pyrf_evlist__add,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("adds an event selector to the list.")
	},
	{
		.ml_name  = "read_on_cpu",
		.ml_meth  = (PyCFunction)pyrf_evlist__read_on_cpu,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("reads an event.")
	},
	{
		.ml_name  = "config",
		.ml_meth  = (PyCFunction)pyrf_evlist__config,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("Apply default record options to the evlist.")
	},
	{
		.ml_name  = "disable",
		.ml_meth  = (PyCFunction)pyrf_evlist__disable,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("Disable the evsels in the evlist.")
	},
	{
		.ml_name  = "enable",
		.ml_meth  = (PyCFunction)pyrf_evlist__enable,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("Enable the evsels in the evlist.")
	},
	{ .ml_name = NULL, }
};

static Py_ssize_t pyrf_evlist__length(PyObject *obj)
{
	struct pyrf_evlist *pevlist = (void *)obj;

	return pevlist->evlist.core.nr_entries;
}

static PyObject *pyrf_evlist__item(PyObject *obj, Py_ssize_t i)
{
	struct pyrf_evlist *pevlist = (void *)obj;
	struct evsel *pos;

	if (i >= pevlist->evlist.core.nr_entries) {
		PyErr_SetString(PyExc_IndexError, "Index out of range");
		return NULL;
	}

	evlist__for_each_entry(&pevlist->evlist, pos) {
		if (i-- == 0)
			break;
	}

	return Py_BuildValue("O", container_of(pos, struct pyrf_evsel, evsel));
}

static PyObject *pyrf_evlist__str(PyObject *self)
{
	struct pyrf_evlist *pevlist = (void *)self;
	struct evsel *pos;
	struct strbuf sb = STRBUF_INIT;
	bool first = true;
	PyObject *result;

	strbuf_addstr(&sb, "evlist([");
	evlist__for_each_entry(&pevlist->evlist, pos) {
		if (!first)
			strbuf_addch(&sb, ',');
		if (!pos->pmu)
			strbuf_addstr(&sb, evsel__name(pos));
		else
			strbuf_addf(&sb, "%s/%s/", pos->pmu->name, evsel__name(pos));
		first = false;
	}
	strbuf_addstr(&sb, "])");
	result = PyUnicode_FromString(sb.buf);
	strbuf_release(&sb);
	return result;
}

static PySequenceMethods pyrf_evlist__sequence_methods = {
	.sq_length = pyrf_evlist__length,
	.sq_item   = pyrf_evlist__item,
};

static const char pyrf_evlist__doc[] = PyDoc_STR("perf event selector list object.");

static PyTypeObject pyrf_evlist__type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	.tp_name	= "perf.evlist",
	.tp_basicsize	= sizeof(struct pyrf_evlist),
	.tp_dealloc	= (destructor)pyrf_evlist__delete,
	.tp_flags	= Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,
	.tp_as_sequence	= &pyrf_evlist__sequence_methods,
	.tp_doc		= pyrf_evlist__doc,
	.tp_methods	= pyrf_evlist__methods,
	.tp_init	= (initproc)pyrf_evlist__init,
	.tp_repr        = pyrf_evlist__str,
	.tp_str         = pyrf_evlist__str,
};

static int pyrf_evlist__setup_types(void)
{
	pyrf_evlist__type.tp_new = PyType_GenericNew;
	return PyType_Ready(&pyrf_evlist__type);
}

#define PERF_CONST(name) { #name, PERF_##name }

struct perf_constant {
	const char *name;
	int	    value;
};

static const struct perf_constant perf__constants[] = {
	PERF_CONST(TYPE_HARDWARE),
	PERF_CONST(TYPE_SOFTWARE),
	PERF_CONST(TYPE_TRACEPOINT),
	PERF_CONST(TYPE_HW_CACHE),
	PERF_CONST(TYPE_RAW),
	PERF_CONST(TYPE_BREAKPOINT),

	PERF_CONST(COUNT_HW_CPU_CYCLES),
	PERF_CONST(COUNT_HW_INSTRUCTIONS),
	PERF_CONST(COUNT_HW_CACHE_REFERENCES),
	PERF_CONST(COUNT_HW_CACHE_MISSES),
	PERF_CONST(COUNT_HW_BRANCH_INSTRUCTIONS),
	PERF_CONST(COUNT_HW_BRANCH_MISSES),
	PERF_CONST(COUNT_HW_BUS_CYCLES),
	PERF_CONST(COUNT_HW_CACHE_L1D),
	PERF_CONST(COUNT_HW_CACHE_L1I),
	PERF_CONST(COUNT_HW_CACHE_LL),
	PERF_CONST(COUNT_HW_CACHE_DTLB),
	PERF_CONST(COUNT_HW_CACHE_ITLB),
	PERF_CONST(COUNT_HW_CACHE_BPU),
	PERF_CONST(COUNT_HW_CACHE_OP_READ),
	PERF_CONST(COUNT_HW_CACHE_OP_WRITE),
	PERF_CONST(COUNT_HW_CACHE_OP_PREFETCH),
	PERF_CONST(COUNT_HW_CACHE_RESULT_ACCESS),
	PERF_CONST(COUNT_HW_CACHE_RESULT_MISS),

	PERF_CONST(COUNT_HW_STALLED_CYCLES_FRONTEND),
	PERF_CONST(COUNT_HW_STALLED_CYCLES_BACKEND),

	PERF_CONST(COUNT_SW_CPU_CLOCK),
	PERF_CONST(COUNT_SW_TASK_CLOCK),
	PERF_CONST(COUNT_SW_PAGE_FAULTS),
	PERF_CONST(COUNT_SW_CONTEXT_SWITCHES),
	PERF_CONST(COUNT_SW_CPU_MIGRATIONS),
	PERF_CONST(COUNT_SW_PAGE_FAULTS_MIN),
	PERF_CONST(COUNT_SW_PAGE_FAULTS_MAJ),
	PERF_CONST(COUNT_SW_ALIGNMENT_FAULTS),
	PERF_CONST(COUNT_SW_EMULATION_FAULTS),
	PERF_CONST(COUNT_SW_DUMMY),

	PERF_CONST(SAMPLE_IP),
	PERF_CONST(SAMPLE_TID),
	PERF_CONST(SAMPLE_TIME),
	PERF_CONST(SAMPLE_ADDR),
	PERF_CONST(SAMPLE_READ),
	PERF_CONST(SAMPLE_CALLCHAIN),
	PERF_CONST(SAMPLE_ID),
	PERF_CONST(SAMPLE_CPU),
	PERF_CONST(SAMPLE_PERIOD),
	PERF_CONST(SAMPLE_STREAM_ID),
	PERF_CONST(SAMPLE_RAW),

	PERF_CONST(FORMAT_TOTAL_TIME_ENABLED),
	PERF_CONST(FORMAT_TOTAL_TIME_RUNNING),
	PERF_CONST(FORMAT_ID),
	PERF_CONST(FORMAT_GROUP),

	PERF_CONST(RECORD_MMAP),
	PERF_CONST(RECORD_LOST),
	PERF_CONST(RECORD_COMM),
	PERF_CONST(RECORD_EXIT),
	PERF_CONST(RECORD_THROTTLE),
	PERF_CONST(RECORD_UNTHROTTLE),
	PERF_CONST(RECORD_FORK),
	PERF_CONST(RECORD_READ),
	PERF_CONST(RECORD_SAMPLE),
	PERF_CONST(RECORD_MMAP2),
	PERF_CONST(RECORD_AUX),
	PERF_CONST(RECORD_ITRACE_START),
	PERF_CONST(RECORD_LOST_SAMPLES),
	PERF_CONST(RECORD_SWITCH),
	PERF_CONST(RECORD_SWITCH_CPU_WIDE),

	PERF_CONST(RECORD_MISC_SWITCH_OUT),
	{ .name = NULL, },
};

static PyObject *pyrf__tracepoint(struct pyrf_evsel *pevsel,
				  PyObject *args, PyObject *kwargs)
{
	static char *kwlist[] = { "sys", "name", NULL };
	char *sys  = NULL;
	char *name = NULL;

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|ss", kwlist,
					 &sys, &name))
		return NULL;

	return PyLong_FromLong(tp_pmu__id(sys, name));
}

static PyObject *pyrf_evsel__from_evsel(struct evsel *evsel)
{
	struct pyrf_evsel *pevsel = PyObject_New(struct pyrf_evsel, &pyrf_evsel__type);

	if (!pevsel)
		return NULL;

	memset(&pevsel->evsel, 0, sizeof(pevsel->evsel));
	evsel__init(&pevsel->evsel, &evsel->core.attr, evsel->core.idx);

	evsel__clone(&pevsel->evsel, evsel);
	if (evsel__is_group_leader(evsel))
		evsel__set_leader(&pevsel->evsel, &pevsel->evsel);
	return (PyObject *)pevsel;
}

static int evlist__pos(struct evlist *evlist, struct evsel *evsel)
{
	struct evsel *pos;
	int idx = 0;

	evlist__for_each_entry(evlist, pos) {
		if (evsel == pos)
			return idx;
		idx++;
	}
	return -1;
}

static struct evsel *evlist__at(struct evlist *evlist, int idx)
{
	struct evsel *pos;
	int idx2 = 0;

	evlist__for_each_entry(evlist, pos) {
		if (idx == idx2)
			return pos;
		idx2++;
	}
	return NULL;
}

static PyObject *pyrf_evlist__from_evlist(struct evlist *evlist)
{
	struct pyrf_evlist *pevlist = PyObject_New(struct pyrf_evlist, &pyrf_evlist__type);
	struct evsel *pos;
	struct rb_node *node;

	if (!pevlist)
		return NULL;

	memset(&pevlist->evlist, 0, sizeof(pevlist->evlist));
	evlist__init(&pevlist->evlist, evlist->core.all_cpus, evlist->core.threads);
	evlist__for_each_entry(evlist, pos) {
		struct pyrf_evsel *pevsel = (void *)pyrf_evsel__from_evsel(pos);

		evlist__add(&pevlist->evlist, &pevsel->evsel);
	}
	evlist__for_each_entry(&pevlist->evlist, pos) {
		struct evsel *leader = evsel__leader(pos);

		if (pos != leader) {
			int idx = evlist__pos(evlist, leader);

			if (idx >= 0)
				evsel__set_leader(pos, evlist__at(&pevlist->evlist, idx));
			else if (leader == NULL)
				evsel__set_leader(pos, pos);
		}
	}
	metricgroup__copy_metric_events(&pevlist->evlist, /*cgrp=*/NULL,
					&pevlist->evlist.metric_events,
					&evlist->metric_events);
	for (node = rb_first_cached(&pevlist->evlist.metric_events.entries); node;
	     node = rb_next(node)) {
		struct metric_event *me = container_of(node, struct metric_event, nd);
		struct list_head *mpos;
		int idx = evlist__pos(evlist, me->evsel);

		if (idx >= 0)
			me->evsel = evlist__at(&pevlist->evlist, idx);
		list_for_each(mpos, &me->head) {
			struct metric_expr *e = container_of(mpos, struct metric_expr, nd);

			for (int j = 0; e->metric_events[j]; j++) {
				idx = evlist__pos(evlist, e->metric_events[j]);
				if (idx >= 0)
					e->metric_events[j] = evlist__at(&pevlist->evlist, idx);
			}
		}
	}
	return (PyObject *)pevlist;
}

static PyObject *pyrf__parse_events(PyObject *self, PyObject *args)
{
	const char *input;
	struct evlist evlist = {};
	struct parse_events_error err;
	PyObject *result;
	PyObject *pcpus = NULL, *pthreads = NULL;
	struct perf_cpu_map *cpus;
	struct perf_thread_map *threads;

	if (!PyArg_ParseTuple(args, "s|OO", &input, &pcpus, &pthreads))
		return NULL;

	threads = pthreads ? ((struct pyrf_thread_map *)pthreads)->threads : NULL;
	cpus = pcpus ? ((struct pyrf_cpu_map *)pcpus)->cpus : NULL;

	parse_events_error__init(&err);
	evlist__init(&evlist, cpus, threads);
	if (parse_events(&evlist, input, &err)) {
		parse_events_error__print(&err, input);
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}
	result = pyrf_evlist__from_evlist(&evlist);
	evlist__exit(&evlist);
	return result;
}

static PyObject *pyrf__parse_metrics(PyObject *self, PyObject *args)
{
	const char *input, *pmu = NULL;
	struct evlist evlist = {};
	PyObject *result;
	PyObject *pcpus = NULL, *pthreads = NULL;
	struct perf_cpu_map *cpus;
	struct perf_thread_map *threads;
	int ret;

	if (!PyArg_ParseTuple(args, "s|sOO", &input, &pmu, &pcpus, &pthreads))
		return NULL;

	threads = pthreads ? ((struct pyrf_thread_map *)pthreads)->threads : NULL;
	cpus = pcpus ? ((struct pyrf_cpu_map *)pcpus)->cpus : NULL;

	evlist__init(&evlist, cpus, threads);
	ret = metricgroup__parse_groups(&evlist, pmu ?: "all", input,
					/*metric_no_group=*/ false,
					/*metric_no_merge=*/ false,
					/*metric_no_threshold=*/ true,
					/*user_requested_cpu_list=*/ NULL,
					/*system_wide=*/true,
					/*hardware_aware_grouping=*/ false);
	if (ret) {
		errno = -ret;
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}
	result = pyrf_evlist__from_evlist(&evlist);
	evlist__exit(&evlist);
	return result;
}

static PyObject *pyrf__metrics_groups(const struct pmu_metric *pm)
{
	PyObject *groups = PyList_New(/*len=*/0);
	const char *mg = pm->metric_group;

	if (!groups)
		return NULL;

	while (mg) {
		PyObject *val = NULL;
		const char *sep = strchr(mg, ';');
		size_t len = sep ? (size_t)(sep - mg) : strlen(mg);

		if (len > 0) {
			val = PyUnicode_FromStringAndSize(mg, len);
			if (val)
				PyList_Append(groups, val);

			Py_XDECREF(val);
		}
		mg = sep ? sep + 1 : NULL;
	}
	return groups;
}

static int pyrf__metrics_cb(const struct pmu_metric *pm,
			    const struct pmu_metrics_table *table __maybe_unused,
			    void *vdata)
{
	PyObject *py_list = vdata;
	PyObject *dict = PyDict_New();
	PyObject *key = dict ? PyUnicode_FromString("MetricGroup") : NULL;
	PyObject *value = key ? pyrf__metrics_groups(pm) : NULL;

	if (!value || PyDict_SetItem(dict, key, value) != 0) {
		Py_XDECREF(key);
		Py_XDECREF(value);
		Py_XDECREF(dict);
		return -ENOMEM;
	}

	if (!add_to_dict(dict, "MetricName", pm->metric_name) ||
	    !add_to_dict(dict, "PMU", pm->pmu) ||
	    !add_to_dict(dict, "MetricExpr", pm->metric_expr) ||
	    !add_to_dict(dict, "MetricThreshold", pm->metric_threshold) ||
	    !add_to_dict(dict, "ScaleUnit", pm->unit) ||
	    !add_to_dict(dict, "Compat", pm->compat) ||
	    !add_to_dict(dict, "BriefDescription", pm->desc) ||
	    !add_to_dict(dict, "PublicDescription", pm->long_desc) ||
	    PyList_Append(py_list, dict) != 0) {
		Py_DECREF(dict);
		return -ENOMEM;
	}
	Py_DECREF(dict);
	return 0;
}

static PyObject *pyrf__metrics(PyObject *self, PyObject *args)
{
	const struct pmu_metrics_table *table = pmu_metrics_table__find();
	PyObject *list = PyList_New(/*len=*/0);
	int ret;

	if (!list)
		return NULL;

	ret = pmu_metrics_table__for_each_metric(table, pyrf__metrics_cb, list);
	if (!ret)
		ret = pmu_for_each_sys_metric(pyrf__metrics_cb, list);

	if (ret) {
		Py_DECREF(list);
		errno = -ret;
		PyErr_SetFromErrno(PyExc_OSError);
		return NULL;
	}
	return list;
}

static PyMethodDef perf__methods[] = {
	{
		.ml_name  = "metrics",
		.ml_meth  = (PyCFunction) pyrf__metrics,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR(
			"Returns a list of metrics represented as string values in dictionaries.")
	},
	{
		.ml_name  = "tracepoint",
		.ml_meth  = (PyCFunction) pyrf__tracepoint,
		.ml_flags = METH_VARARGS | METH_KEYWORDS,
		.ml_doc	  = PyDoc_STR("Get tracepoint config.")
	},
	{
		.ml_name  = "parse_events",
		.ml_meth  = (PyCFunction) pyrf__parse_events,
		.ml_flags = METH_VARARGS,
		.ml_doc	  = PyDoc_STR("Parse a string of events and return an evlist.")
	},
	{
		.ml_name  = "parse_metrics",
		.ml_meth  = (PyCFunction) pyrf__parse_metrics,
		.ml_flags = METH_VARARGS,
		.ml_doc	  = PyDoc_STR(
			"Parse a string of metrics or metric groups and return an evlist.")
	},
	{
		.ml_name  = "pmus",
		.ml_meth  = (PyCFunction) pyrf__pmus,
		.ml_flags = METH_NOARGS,
		.ml_doc	  = PyDoc_STR("Returns a sequence of pmus.")
	},
	{ .ml_name = NULL, }
};

PyMODINIT_FUNC PyInit_perf(void)
{
	PyObject *obj;
	int i;
	PyObject *dict;
	static struct PyModuleDef moduledef = {
		PyModuleDef_HEAD_INIT,
		"perf",			/* m_name */
		"",			/* m_doc */
		-1,			/* m_size */
		perf__methods,		/* m_methods */
		NULL,			/* m_reload */
		NULL,			/* m_traverse */
		NULL,			/* m_clear */
		NULL,			/* m_free */
	};
	PyObject *module = PyModule_Create(&moduledef);

	if (module == NULL ||
	    pyrf_event__setup_types() < 0 ||
	    pyrf_evlist__setup_types() < 0 ||
	    pyrf_evsel__setup_types() < 0 ||
	    pyrf_thread_map__setup_types() < 0 ||
	    pyrf_cpu_map__setup_types() < 0 ||
	    pyrf_pmu_iterator__setup_types() < 0 ||
	    pyrf_pmu__setup_types() < 0 ||
	    pyrf_counts_values__setup_types() < 0)
		return module;

	/* The page_size is placed in util object. */
	page_size = sysconf(_SC_PAGE_SIZE);

	Py_INCREF(&pyrf_evlist__type);
	PyModule_AddObject(module, "evlist", (PyObject*)&pyrf_evlist__type);

	Py_INCREF(&pyrf_evsel__type);
	PyModule_AddObject(module, "evsel", (PyObject*)&pyrf_evsel__type);

	Py_INCREF(&pyrf_mmap_event__type);
	PyModule_AddObject(module, "mmap_event", (PyObject *)&pyrf_mmap_event__type);

	Py_INCREF(&pyrf_lost_event__type);
	PyModule_AddObject(module, "lost_event", (PyObject *)&pyrf_lost_event__type);

	Py_INCREF(&pyrf_comm_event__type);
	PyModule_AddObject(module, "comm_event", (PyObject *)&pyrf_comm_event__type);

	Py_INCREF(&pyrf_task_event__type);
	PyModule_AddObject(module, "task_event", (PyObject *)&pyrf_task_event__type);

	Py_INCREF(&pyrf_throttle_event__type);
	PyModule_AddObject(module, "throttle_event", (PyObject *)&pyrf_throttle_event__type);

	Py_INCREF(&pyrf_task_event__type);
	PyModule_AddObject(module, "task_event", (PyObject *)&pyrf_task_event__type);

	Py_INCREF(&pyrf_read_event__type);
	PyModule_AddObject(module, "read_event", (PyObject *)&pyrf_read_event__type);

	Py_INCREF(&pyrf_sample_event__type);
	PyModule_AddObject(module, "sample_event", (PyObject *)&pyrf_sample_event__type);

	Py_INCREF(&pyrf_context_switch_event__type);
	PyModule_AddObject(module, "switch_event", (PyObject *)&pyrf_context_switch_event__type);

	Py_INCREF(&pyrf_thread_map__type);
	PyModule_AddObject(module, "thread_map", (PyObject*)&pyrf_thread_map__type);

	Py_INCREF(&pyrf_cpu_map__type);
	PyModule_AddObject(module, "cpu_map", (PyObject*)&pyrf_cpu_map__type);

	Py_INCREF(&pyrf_counts_values__type);
	PyModule_AddObject(module, "counts_values", (PyObject *)&pyrf_counts_values__type);

	dict = PyModule_GetDict(module);
	if (dict == NULL)
		goto error;

	for (i = 0; perf__constants[i].name != NULL; i++) {
		obj = PyLong_FromLong(perf__constants[i].value);
		if (obj == NULL)
			goto error;
		PyDict_SetItemString(dict, perf__constants[i].name, obj);
		Py_DECREF(obj);
	}

error:
	if (PyErr_Occurred())
		PyErr_SetString(PyExc_ImportError, "perf: Init failed!");
	return module;
}
