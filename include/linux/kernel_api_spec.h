/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026 Sasha Levin <sashal@kernel.org>
 *
 * kernel_api_spec.h - Kernel API Specification Framework
 *
 * Structures and macros for specifying kernel APIs in a human and
 * machine-readable form: parameters, return values, error conditions,
 * and constraints.
 */

#ifndef _LINUX_KERNEL_API_SPEC_H
#define _LINUX_KERNEL_API_SPEC_H

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/kapi_syscall.h>
#include <linux/kernel.h>
#include <linux/stringify.h>
#include <linux/types.h>

struct sigaction;

#define KAPI_MAX_PARAMS		16
#define KAPI_MAX_ERRORS		32
#define KAPI_MAX_CONSTRAINTS	32
#define KAPI_MAX_LOCKS		16
#define KAPI_MAX_SIGNALS	32
#define KAPI_MAX_NAME_LEN	128
#define KAPI_MAX_DESC_LEN	512
#define KAPI_MAX_CAPABILITIES	8

/* Magic numbers for section validation (ASCII mnemonics) */
#define KAPI_MAGIC_PARAMS	0x4B415031	/* 'KAP1' */
#define KAPI_MAGIC_RETURN	0x4B415232	/* 'KAR2' */
#define KAPI_MAGIC_ERRORS	0x4B414533	/* 'KAE3' */
#define KAPI_MAGIC_LOCKS	0x4B414C34	/* 'KAL4' */
#define KAPI_MAGIC_CONSTRAINTS	0x4B414335	/* 'KAC5' */
#define KAPI_MAGIC_INFO		0x4B414936	/* 'KAI6' */
#define KAPI_MAGIC_SIGNALS	0x4B415337	/* 'KAS7' */
#define KAPI_MAGIC_SIGMASK	0x4B414D38	/* 'KAM8' */
#define KAPI_MAGIC_STRUCTS	0x4B415439	/* 'KAT9' */
#define KAPI_MAGIC_EFFECTS	0x4B414641	/* 'KAFA' */
#define KAPI_MAGIC_TRANS	0x4B415442	/* 'KATB' */
#define KAPI_MAGIC_CAPS		0x4B414343	/* 'KACC' */

/**
 * enum kapi_param_type - Parameter type classification
 * @KAPI_TYPE_VOID: void type
 * @KAPI_TYPE_INT: Integer types (int, long, etc.)
 * @KAPI_TYPE_UINT: Unsigned integer types
 * @KAPI_TYPE_PTR: Pointer types
 * @KAPI_TYPE_STRUCT: Structure types
 * @KAPI_TYPE_UNION: Union types
 * @KAPI_TYPE_ENUM: Enumeration types
 * @KAPI_TYPE_FUNC_PTR: Function pointer types
 * @KAPI_TYPE_ARRAY: Array types
 * @KAPI_TYPE_FD: File descriptor - range-checked only
 * @KAPI_TYPE_USER_PTR: User space pointer - validated for access and size
 * @KAPI_TYPE_PATH: Pathname - validated for access and path limits
 * @KAPI_TYPE_CUSTOM: Custom/complex types
 */
enum kapi_param_type {
	KAPI_TYPE_VOID = 0,
	KAPI_TYPE_INT,
	KAPI_TYPE_UINT,
	KAPI_TYPE_PTR,
	KAPI_TYPE_STRUCT,
	KAPI_TYPE_UNION,
	KAPI_TYPE_ENUM,
	KAPI_TYPE_FUNC_PTR,
	KAPI_TYPE_ARRAY,
	KAPI_TYPE_FD,		/* File descriptor - range-checked only */
	KAPI_TYPE_USER_PTR,	/* User space pointer - validated for access and size */
	KAPI_TYPE_PATH,		/* Pathname - validated for access and path limits */
	KAPI_TYPE_CUSTOM,
};

/**
 * enum kapi_param_flags - Parameter attribute flags
 * @KAPI_PARAM_IN: Input parameter
 * @KAPI_PARAM_OUT: Output parameter
 * @KAPI_PARAM_INOUT: Input/output parameter
 * @KAPI_PARAM_OPTIONAL: Optional parameter (can be NULL)
 * @KAPI_PARAM_CONST: Const qualified parameter
 * @KAPI_PARAM_VOLATILE: Volatile qualified parameter
 * @KAPI_PARAM_USER: User space pointer
 * @KAPI_PARAM_DMA: DMA-capable memory required
 * @KAPI_PARAM_ALIGNED: Alignment requirements
 */
enum kapi_param_flags {
	KAPI_PARAM_IN		= (1 << 0),
	KAPI_PARAM_OUT		= (1 << 1),
	KAPI_PARAM_INOUT	= (KAPI_PARAM_IN | KAPI_PARAM_OUT),
	KAPI_PARAM_OPTIONAL	= (1 << 3),
	KAPI_PARAM_CONST	= (1 << 4),
	KAPI_PARAM_VOLATILE	= (1 << 5),
	KAPI_PARAM_USER		= (1 << 6),
	KAPI_PARAM_DMA		= (1 << 7),
	KAPI_PARAM_ALIGNED	= (1 << 8),
};

/**
 * enum kapi_context_flags - Function execution context flags
 * @KAPI_CTX_PROCESS: Can be called from process context
 * @KAPI_CTX_SOFTIRQ: Can be called from softirq context
 * @KAPI_CTX_HARDIRQ: Can be called from hardirq context
 * @KAPI_CTX_NMI: Can be called from NMI context
 * @KAPI_CTX_ATOMIC: Must be called in atomic context
 * @KAPI_CTX_SLEEPABLE: May sleep
 * @KAPI_CTX_PREEMPT_DISABLED: Requires preemption disabled
 * @KAPI_CTX_IRQ_DISABLED: Requires interrupts disabled
 */
enum kapi_context_flags {
	KAPI_CTX_PROCESS	= (1 << 0),
	KAPI_CTX_SOFTIRQ	= (1 << 1),
	KAPI_CTX_HARDIRQ	= (1 << 2),
	KAPI_CTX_NMI		= (1 << 3),
	KAPI_CTX_ATOMIC		= (1 << 4),
	KAPI_CTX_SLEEPABLE	= (1 << 5),
	KAPI_CTX_PREEMPT_DISABLED = (1 << 6),
	KAPI_CTX_IRQ_DISABLED	= (1 << 7),
};

/**
 * enum kapi_lock_type - Lock types used/required by the function
 * @KAPI_LOCK_NONE: No locking requirements
 * @KAPI_LOCK_MUTEX: Mutex lock
 * @KAPI_LOCK_SPINLOCK: Spinlock
 * @KAPI_LOCK_RWLOCK: Read-write lock
 * @KAPI_LOCK_SEQLOCK: Sequence lock
 * @KAPI_LOCK_RCU: RCU lock
 * @KAPI_LOCK_SEMAPHORE: Semaphore
 * @KAPI_LOCK_CUSTOM: Custom locking mechanism
 */
enum kapi_lock_type {
	KAPI_LOCK_NONE = 0,
	KAPI_LOCK_MUTEX,
	KAPI_LOCK_SPINLOCK,
	KAPI_LOCK_RWLOCK,
	KAPI_LOCK_SEQLOCK,
	KAPI_LOCK_RCU,
	KAPI_LOCK_SEMAPHORE,
	KAPI_LOCK_CUSTOM,
};

/**
 * enum kapi_constraint_type - Types of parameter constraints
 * @KAPI_CONSTRAINT_NONE: No constraint
 * @KAPI_CONSTRAINT_RANGE: Numeric range constraint
 * @KAPI_CONSTRAINT_MASK: Bitmask constraint
 * @KAPI_CONSTRAINT_ENUM: Enumerated values constraint
 * @KAPI_CONSTRAINT_ALIGNMENT: Alignment constraint (must be aligned to specified boundary)
 * @KAPI_CONSTRAINT_POWER_OF_TWO: Value must be a power of two
 * @KAPI_CONSTRAINT_PAGE_ALIGNED: Value must be page-aligned
 * @KAPI_CONSTRAINT_NONZERO: Value must be non-zero
 * @KAPI_CONSTRAINT_USER_STRING: Userspace null-terminated string with length range
 * @KAPI_CONSTRAINT_USER_PATH: Userspace pathname string (validated for accessibility and PATH_MAX)
 * @KAPI_CONSTRAINT_USER_PTR: Userspace pointer (validated for accessibility and size)
 * @KAPI_CONSTRAINT_BUFFER: Userspace buffer pointer (validated by copy_to/from_user)
 * @KAPI_CONSTRAINT_CUSTOM: Custom validation function
 */
enum kapi_constraint_type {
	KAPI_CONSTRAINT_NONE = 0,
	KAPI_CONSTRAINT_RANGE,
	KAPI_CONSTRAINT_MASK,
	KAPI_CONSTRAINT_ENUM,
	KAPI_CONSTRAINT_ALIGNMENT,
	KAPI_CONSTRAINT_POWER_OF_TWO,
	KAPI_CONSTRAINT_PAGE_ALIGNED,
	KAPI_CONSTRAINT_NONZERO,
	KAPI_CONSTRAINT_USER_STRING,
	KAPI_CONSTRAINT_USER_PATH,
	KAPI_CONSTRAINT_USER_PTR,
	KAPI_CONSTRAINT_BUFFER,
	KAPI_CONSTRAINT_CUSTOM,
};

/**
 * struct kapi_param_spec - Parameter specification
 * @name: Parameter name
 * @type_name: Type name as string
 * @type: Parameter type classification
 * @flags: Parameter attribute flags
 * @size: Size in bytes (for arrays/buffers)
 * @alignment: Required alignment
 * @min_value: Minimum valid value (for numeric types)
 * @max_value: Maximum valid value (for numeric types)
 * @valid_mask: Valid bits mask (for flag parameters)
 * @enum_values: Array of valid enumerated values
 * @enum_count: Number of valid enumerated values
 * @constraint_type: Type of constraint applied
 * @validate: Custom validation function
 * @description: Human-readable description
 * @constraints: Additional constraints description
 * @size_param_idx: 1-based index of the parameter that determines size,
 *                  or 0 if this parameter has a fixed size
 * @size_multiplier: Multiplier for size calculation (e.g., sizeof(struct))
 */
struct kapi_param_spec {
	const char *name;
	const char *type_name;
	enum kapi_param_type type;
	u32 flags;
	size_t size;
	size_t alignment;
	s64 min_value;
	s64 max_value;
	u64 valid_mask;
	const s64 *enum_values;
	u32 enum_count;
	enum kapi_constraint_type constraint_type;
	bool (*validate)(s64 value);
	const char *description;
	const char *constraints;
	int size_param_idx;	/* 1-based param index for dynamic size; 0 if N/A */
	size_t size_multiplier;	/* Size per unit (e.g., sizeof(struct epoll_event)) */
};

/**
 * struct kapi_error_spec - Error condition specification
 * @error_code: Error code value
 * @name: Error code name (e.g., "EINVAL")
 * @condition: Condition that triggers this error
 * @description: Detailed error description
 */
struct kapi_error_spec {
	int error_code;
	const char *name;
	const char *condition;
	const char *description;
};

/**
 * enum kapi_return_check_type - Return value check types
 * @KAPI_RETURN_EXACT: Success is an exact value
 * @KAPI_RETURN_RANGE: Success is within a range
 * @KAPI_RETURN_ERROR_CHECK: Success is when NOT in error list
 * @KAPI_RETURN_FD: Return value is a file descriptor (>= 0 is success)
 * @KAPI_RETURN_CUSTOM: Custom validation function
 * @KAPI_RETURN_NO_RETURN: Function does not return (e.g., exec on success)
 */
enum kapi_return_check_type {
	KAPI_RETURN_EXACT,
	KAPI_RETURN_RANGE,
	KAPI_RETURN_ERROR_CHECK,
	KAPI_RETURN_FD,
	KAPI_RETURN_CUSTOM,
	KAPI_RETURN_NO_RETURN,
};

/**
 * struct kapi_return_spec - Return value specification
 * @type_name: Return type name
 * @type: Return type classification
 * @check_type: Type of success check to perform
 * @success_value: Exact value indicating success (for EXACT)
 * @success_min: Minimum success value (for RANGE)
 * @success_max: Maximum success value (for RANGE)
 * @error_values: Array of error values (for ERROR_CHECK)
 * @error_count: Number of error values
 * @is_success: Custom function to check success
 * @description: Return value description
 */
struct kapi_return_spec {
	const char *type_name;
	enum kapi_param_type type;
	enum kapi_return_check_type check_type;
	s64 success_value;
	s64 success_min;
	s64 success_max;
	const s64 *error_values;
	u32 error_count;
	bool (*is_success)(s64 retval);
	const char *description;
};

/**
 * enum kapi_lock_scope - Lock acquisition/release scope
 * @KAPI_LOCK_INTERNAL: Lock is acquired and released within the function (common case)
 * @KAPI_LOCK_ACQUIRES: Function acquires lock but does not release it
 * @KAPI_LOCK_RELEASES: Function releases lock (must be held on entry)
 * @KAPI_LOCK_CALLER_HELD: Lock must be held by caller throughout the call
 */
enum kapi_lock_scope {
	KAPI_LOCK_INTERNAL = 0,
	KAPI_LOCK_ACQUIRES,
	KAPI_LOCK_RELEASES,
	KAPI_LOCK_CALLER_HELD,
};

/**
 * struct kapi_lock_spec - Lock requirement specification
 * @lock_name: Name of the lock
 * @lock_type: Type of lock
 * @scope: Lock scope (internal, acquires, releases, or caller-held)
 * @description: Additional lock requirements
 */
struct kapi_lock_spec {
	const char *lock_name;
	enum kapi_lock_type lock_type;
	enum kapi_lock_scope scope;
	const char *description;
};

/**
 * struct kapi_constraint_spec - Additional constraint specification
 * @name: Constraint name
 * @description: Constraint description
 * @expression: Formal expression (if applicable)
 */
struct kapi_constraint_spec {
	const char *name;
	const char *description;
	const char *expression;
};

/**
 * enum kapi_signal_direction - Signal flow direction
 * @KAPI_SIGNAL_RECEIVE: Function may receive this signal
 * @KAPI_SIGNAL_SEND: Function may send this signal
 * @KAPI_SIGNAL_HANDLE: Function handles this signal specially
 * @KAPI_SIGNAL_BLOCK: Function blocks this signal
 * @KAPI_SIGNAL_IGNORE: Function ignores this signal
 */
enum kapi_signal_direction {
	KAPI_SIGNAL_RECEIVE	= (1 << 0),
	KAPI_SIGNAL_SEND	= (1 << 1),
	KAPI_SIGNAL_HANDLE	= (1 << 2),
	KAPI_SIGNAL_BLOCK	= (1 << 3),
	KAPI_SIGNAL_IGNORE	= (1 << 4),
};

/**
 * enum kapi_signal_action - What the function does with the signal
 * @KAPI_SIGNAL_ACTION_DEFAULT: Default signal action applies
 * @KAPI_SIGNAL_ACTION_TERMINATE: Causes termination
 * @KAPI_SIGNAL_ACTION_COREDUMP: Causes termination with core dump
 * @KAPI_SIGNAL_ACTION_STOP: Stops the process
 * @KAPI_SIGNAL_ACTION_CONTINUE: Continues a stopped process
 * @KAPI_SIGNAL_ACTION_CUSTOM: Custom handling described in notes
 * @KAPI_SIGNAL_ACTION_RETURN: Returns from syscall with EINTR
 * @KAPI_SIGNAL_ACTION_RESTART: Restarts the syscall
 * @KAPI_SIGNAL_ACTION_QUEUE: Queues the signal for later delivery
 * @KAPI_SIGNAL_ACTION_DISCARD: Discards the signal
 * @KAPI_SIGNAL_ACTION_TRANSFORM: Transforms to another signal
 */
enum kapi_signal_action {
	KAPI_SIGNAL_ACTION_DEFAULT = 0,
	KAPI_SIGNAL_ACTION_TERMINATE,
	KAPI_SIGNAL_ACTION_COREDUMP,
	KAPI_SIGNAL_ACTION_STOP,
	KAPI_SIGNAL_ACTION_CONTINUE,
	KAPI_SIGNAL_ACTION_CUSTOM,
	KAPI_SIGNAL_ACTION_RETURN,
	KAPI_SIGNAL_ACTION_RESTART,
	KAPI_SIGNAL_ACTION_QUEUE,
	KAPI_SIGNAL_ACTION_DISCARD,
	KAPI_SIGNAL_ACTION_TRANSFORM,
};

/**
 * struct kapi_signal_spec - Signal specification
 * @signal_num: Signal number (e.g., SIGKILL, SIGTERM)
 * @signal_name: Signal name as string
 * @direction: Direction flags (OR of kapi_signal_direction)
 * @action: What happens when signal is received
 * @target: Description of target process/thread for sent signals
 * @condition: Condition under which signal is sent/received/handled
 * @description: Detailed description of signal handling
 * @restartable: Whether syscall is restartable after this signal
 * @sa_flags_required: Required signal action flags (SA_*)
 * @sa_flags_forbidden: Forbidden signal action flags
 * @error_on_signal: Error code returned when signal occurs (-EINTR, etc)
 * @transform_to: Signal number to transform to (if action is TRANSFORM)
 * @timing: When signal can occur ("entry", "during", "exit", "anytime")
 * @priority: Signal handling priority (lower processed first)
 * @interruptible: Whether this operation is interruptible by this signal
 * @queue_behavior: How signal is queued ("realtime", "standard", "coalesce")
 * @state_required: Required process state for signal to be delivered
 * @state_forbidden: Forbidden process state for signal delivery
 */
struct kapi_signal_spec {
	int signal_num;
	const char *signal_name;
	u32 direction;
	enum kapi_signal_action action;
	const char *target;
	const char *condition;
	const char *description;
	bool restartable;
	u32 sa_flags_required;
	u32 sa_flags_forbidden;
	int error_on_signal;
	int transform_to;
	const char *timing;
	u8 priority;
	bool interruptible;
	const char *queue_behavior;
	u32 state_required;
	u32 state_forbidden;
};

/**
 * struct kapi_signal_mask_spec - Signal mask specification
 * @mask_name: Name of the signal mask
 * @signals: Array of signal numbers in the mask
 * @signal_count: Number of signals in the mask
 * @description: Description of what this mask represents
 */
struct kapi_signal_mask_spec {
	const char *mask_name;
	int signals[KAPI_MAX_SIGNALS];
	u32 signal_count;
	const char *description;
};

/**
 * struct kapi_struct_field - Structure field specification
 * @name: Field name
 * @type: Field type classification
 * @type_name: Type name as string
 * @offset: Offset within structure
 * @size: Size of field in bytes
 * @flags: Field attribute flags
 * @constraint_type: Type of constraint applied
 * @min_value: Minimum valid value (for numeric types)
 * @max_value: Maximum valid value (for numeric types)
 * @valid_mask: Valid bits mask (for flag fields)
 * @enum_values: Comma-separated list of valid enum values (for enum types)
 * @description: Field description
 */
struct kapi_struct_field {
	const char *name;
	enum kapi_param_type type;
	const char *type_name;
	size_t offset;
	size_t size;
	u32 flags;
	enum kapi_constraint_type constraint_type;
	s64 min_value;
	s64 max_value;
	u64 valid_mask;
	const char *enum_values;	/* Comma-separated list of valid enum values */
	const char *description;
};

/**
 * struct kapi_struct_spec - Structure type specification
 * @name: Structure name
 * @size: Total size of structure
 * @alignment: Required alignment
 * @field_count: Number of fields
 * @fields: Field specifications
 * @description: Structure description
 */
struct kapi_struct_spec {
	const char *name;
	size_t size;
	size_t alignment;
	u32 field_count;
	struct kapi_struct_field fields[KAPI_MAX_PARAMS];
	const char *description;
};

/**
 * enum kapi_capability_action - What the capability allows
 * @KAPI_CAP_BYPASS_CHECK: Bypasses a check entirely
 * @KAPI_CAP_INCREASE_LIMIT: Increases or removes a limit
 * @KAPI_CAP_OVERRIDE_RESTRICTION: Overrides a restriction
 * @KAPI_CAP_GRANT_PERMISSION: Grants permission that would otherwise be denied
 * @KAPI_CAP_MODIFY_BEHAVIOR: Changes the behavior of the operation
 * @KAPI_CAP_ACCESS_RESOURCE: Allows access to restricted resources
 * @KAPI_CAP_PERFORM_OPERATION: Allows performing a privileged operation
 */
enum kapi_capability_action {
	KAPI_CAP_BYPASS_CHECK = 0,
	KAPI_CAP_INCREASE_LIMIT,
	KAPI_CAP_OVERRIDE_RESTRICTION,
	KAPI_CAP_GRANT_PERMISSION,
	KAPI_CAP_MODIFY_BEHAVIOR,
	KAPI_CAP_ACCESS_RESOURCE,
	KAPI_CAP_PERFORM_OPERATION,
};

/**
 * struct kapi_capability_spec - Capability requirement specification
 * @capability: The capability constant (e.g., CAP_IPC_LOCK)
 * @cap_name: Capability name as string
 * @action: What the capability allows (kapi_capability_action)
 * @allows: Description of what the capability allows
 * @without_cap: What happens without the capability
 * @check_condition: Condition when capability is checked
 * @priority: Check priority (lower checked first)
 * @alternative: Alternative capabilities that can be used
 * @alternative_count: Number of alternative capabilities
 */
struct kapi_capability_spec {
	int capability;
	const char *cap_name;
	enum kapi_capability_action action;
	const char *allows;
	const char *without_cap;
	const char *check_condition;
	u8 priority;
	int alternative[KAPI_MAX_CAPABILITIES];
	u32 alternative_count;
};

/**
 * enum kapi_side_effect_type - Types of side effects
 * @KAPI_EFFECT_NONE: No side effects
 * @KAPI_EFFECT_ALLOC_MEMORY: Allocates memory
 * @KAPI_EFFECT_FREE_MEMORY: Frees memory
 * @KAPI_EFFECT_MODIFY_STATE: Modifies global/shared state
 * @KAPI_EFFECT_SIGNAL_SEND: Sends signals
 * @KAPI_EFFECT_FILE_POSITION: Modifies file position
 * @KAPI_EFFECT_LOCK_ACQUIRE: Acquires locks
 * @KAPI_EFFECT_LOCK_RELEASE: Releases locks
 * @KAPI_EFFECT_RESOURCE_CREATE: Creates system resources (FDs, PIDs, etc)
 * @KAPI_EFFECT_RESOURCE_DESTROY: Destroys system resources
 * @KAPI_EFFECT_SCHEDULE: May cause scheduling/context switch
 * @KAPI_EFFECT_HARDWARE: Interacts with hardware
 * @KAPI_EFFECT_NETWORK: Network I/O operation
 * @KAPI_EFFECT_FILESYSTEM: Filesystem modification
 * @KAPI_EFFECT_PROCESS_STATE: Modifies process state
 * @KAPI_EFFECT_IRREVERSIBLE: Effect cannot be undone
 */
enum kapi_side_effect_type {
	KAPI_EFFECT_NONE = 0,
	KAPI_EFFECT_ALLOC_MEMORY = (1 << 0),
	KAPI_EFFECT_FREE_MEMORY = (1 << 1),
	KAPI_EFFECT_MODIFY_STATE = (1 << 2),
	KAPI_EFFECT_SIGNAL_SEND = (1 << 3),
	KAPI_EFFECT_FILE_POSITION = (1 << 4),
	KAPI_EFFECT_LOCK_ACQUIRE = (1 << 5),
	KAPI_EFFECT_LOCK_RELEASE = (1 << 6),
	KAPI_EFFECT_RESOURCE_CREATE = (1 << 7),
	KAPI_EFFECT_RESOURCE_DESTROY = (1 << 8),
	KAPI_EFFECT_SCHEDULE = (1 << 9),
	KAPI_EFFECT_HARDWARE = (1 << 10),
	KAPI_EFFECT_NETWORK = (1 << 11),
	KAPI_EFFECT_FILESYSTEM = (1 << 12),
	KAPI_EFFECT_PROCESS_STATE = (1 << 13),
	KAPI_EFFECT_IRREVERSIBLE = (1 << 14),
};

/**
 * struct kapi_side_effect - Side effect specification
 * @type: Bitmask of effect types
 * @target: What is affected (e.g., "process memory", "file descriptor table")
 * @condition: Condition under which effect occurs
 * @description: Detailed description of the effect
 * @reversible: Whether the effect can be undone
 */
struct kapi_side_effect {
	u32 type;
	const char *target;
	const char *condition;
	const char *description;
	bool reversible;
};

/**
 * struct kapi_state_transition - State transition specification
 * @from_state: Starting state description
 * @to_state: Ending state description
 * @condition: Condition for transition
 * @object: Object whose state changes
 * @description: Detailed description
 */
struct kapi_state_transition {
	const char *from_state;
	const char *to_state;
	const char *condition;
	const char *object;
	const char *description;
};

#define KAPI_MAX_STRUCT_SPECS	8
#define KAPI_MAX_SIDE_EFFECTS	32
#define KAPI_MAX_STATE_TRANS	8

/**
 * struct kernel_api_spec - Complete kernel API specification
 * @name: Function name
 * @version: API version
 * @description: Brief description
 * @long_description: Detailed description
 * @context_flags: Execution context flags
 * @param_count: Number of parameters
 * @params: Parameter specifications
 * @return_spec: Return value specification
 * @error_count: Number of possible errors
 * @errors: Error specifications
 * @lock_count: Number of lock specifications
 * @locks: Lock requirement specifications
 * @constraint_count: Number of additional constraints
 * @constraints: Additional constraint specifications
 * @examples: Usage examples
 * @notes: Additional notes
 * @signal_count: Number of signal specifications
 * @signals: Signal handling specifications
 * @signal_mask_count: Number of signal mask specifications
 * @signal_masks: Signal mask specifications
 * @struct_spec_count: Number of structure specifications
 * @struct_specs: Structure type specifications
 * @side_effect_count: Number of side effect specifications
 * @side_effects: Side effect specifications
 * @state_trans_count: Number of state transition specifications
 * @state_transitions: State transition specifications
 * @capability_count: Number of required capabilities
 * @capabilities: Required capability specifications
 * @param_magic: Magic value marking the start of the params array
 * @return_magic: Magic value marking the return spec
 * @error_magic: Magic value marking the start of the errors array
 * @lock_magic: Magic value marking the start of the locks array
 * @constraint_magic: Magic value marking the constraints array
 * @info_magic: Magic value marking the info block (examples, notes)
 * @signal_magic: Magic value marking the start of the signals array
 * @sigmask_magic: Magic value marking the signal masks array
 * @struct_magic: Magic value marking the struct specs array
 * @effect_magic: Magic value marking the side effects array
 * @trans_magic: Magic value marking the state transitions array
 * @cap_magic: Magic value marking the capabilities array
 */
struct kernel_api_spec {
	const char *name;
	u32 version;
	const char *description;
	const char *long_description;
	u32 context_flags;

	/* Parameters */
	u32 param_magic;  /* 0x4B415031 = 'KAP1' */
	u32 param_count;
	struct kapi_param_spec params[KAPI_MAX_PARAMS];

	/* Return value */
	u32 return_magic; /* 0x4B415232 = 'KAR2' */
	struct kapi_return_spec return_spec;

	/* Errors */
	u32 error_magic;  /* 0x4B414533 = 'KAE3' */
	u32 error_count;
	struct kapi_error_spec errors[KAPI_MAX_ERRORS];

	/* Locking */
	u32 lock_magic;   /* 0x4B414C34 = 'KAL4' */
	u32 lock_count;
	struct kapi_lock_spec locks[KAPI_MAX_LOCKS];

	/* Constraints */
	u32 constraint_magic; /* 0x4B414335 = 'KAC5' */
	u32 constraint_count;
	struct kapi_constraint_spec constraints[KAPI_MAX_CONSTRAINTS];

	/* Additional information */
	u32 info_magic;   /* 0x4B414936 = 'KAI6' */
	const char *examples;
	const char *notes;

	/* Signal specifications */
	u32 signal_magic; /* 0x4B415337 = 'KAS7' */
	u32 signal_count;
	struct kapi_signal_spec signals[KAPI_MAX_SIGNALS];

	/* Signal mask specifications */
	u32 sigmask_magic; /* 0x4B414D38 = 'KAM8' */
	u32 signal_mask_count;
	struct kapi_signal_mask_spec signal_masks[KAPI_MAX_SIGNALS];

	/* Structure specifications */
	u32 struct_magic; /* 0x4B415439 = 'KAT9' */
	u32 struct_spec_count;
	struct kapi_struct_spec struct_specs[KAPI_MAX_STRUCT_SPECS];

	/* Side effects */
	u32 effect_magic; /* 0x4B414641 = 'KAFA' */
	u32 side_effect_count;
	struct kapi_side_effect side_effects[KAPI_MAX_SIDE_EFFECTS];

	/* State transitions */
	u32 trans_magic;  /* 0x4B415442 = 'KATB' */
	u32 state_trans_count;
	struct kapi_state_transition state_transitions[KAPI_MAX_STATE_TRANS];

	/* Capability specifications */
	u32 cap_magic;    /* 0x4B414343 = 'KACC' */
	u32 capability_count;
	struct kapi_capability_spec capabilities[KAPI_MAX_CAPABILITIES];
};

/* Macros for defining API specifications */

/**
 * DEFINE_KERNEL_API_SPEC - Define a kernel API specification
 * @func_name: Function name to specify
 *
 * The ``.kapi_specs`` section holds an array of pointers to
 * fully-defined ``kernel_api_spec`` instances, tightly packed so
 * iteration ``for (pp = __start_kapi_specs; pp < __stop_kapi_specs;
 * pp++)`` advances by one pointer each step regardless of the real
 * spec struct size.
 */
#define DEFINE_KERNEL_API_SPEC(func_name)					\
	extern const struct kernel_api_spec __kapi_spec_##func_name;		\
	static const struct kernel_api_spec * const				\
	__kapi_spec_ptr_##func_name __used __section(".kapi_specs") =		\
		&__kapi_spec_##func_name;					\
	const struct kernel_api_spec __kapi_spec_##func_name = {		\
		.name = __stringify(func_name),					\
		.version = 1,

/**
 * KAPI_DESCRIPTION - Set API description
 * @desc: Description string
 */
#define KAPI_DESCRIPTION(desc) \
	.description = desc,

/**
 * KAPI_LONG_DESC - Set detailed API description
 * @desc: Detailed description string
 */
#define KAPI_LONG_DESC(desc) \
	.long_description = desc,

/**
 * KAPI_CONTEXT - Set execution context flags
 * @flags: Context flags (OR'ed KAPI_CTX_* values)
 */
#define KAPI_CONTEXT(flags) \
	.context_flags = flags,

/**
 * KAPI_PARAM - Define a parameter specification
 * @idx: Parameter index (0-based)
 * @pname: Parameter name
 * @ptype: Type name string
 * @pdesc: Parameter description
 */
#define KAPI_PARAM(idx, pname, ptype, pdesc) \
	.params[idx] = {			\
		.name = pname,			\
		.type_name = ptype,		\
		.description = pdesc,

#define KAPI_PARAM_TYPE(ptype) \
		.type = ptype,

#define KAPI_PARAM_FLAGS(pflags) \
		.flags = pflags,

#define KAPI_PARAM_SIZE(psize) \
		.size = psize,

#define KAPI_PARAM_RANGE(pmin, pmax) \
		.min_value = pmin,	\
		.max_value = pmax,

#define KAPI_PARAM_CONSTRAINT_TYPE(ctype) \
		.constraint_type = ctype,

#define KAPI_PARAM_CONSTRAINT(desc) \
		.constraints = desc,

#define KAPI_PARAM_VALID_MASK(mask) \
		.valid_mask = mask,

/**
 * KAPI_PARAM_ENUM_VALUES - Set the valid values of an enumerated parameter
 * @...: Variadic list of valid values
 */
#define KAPI_PARAM_ENUM_VALUES(...) \
		.enum_values = (const s64[]){ __VA_ARGS__ }, \
		.enum_count = sizeof((const s64[]){ __VA_ARGS__ }) / sizeof(s64),

#define KAPI_PARAM_ALIGNMENT(align) \
		.alignment = align,

/*
 * Store the 1-based parameter index so the zero-initialised default
 * (no dynamic sizing) remains distinguishable from "uses param 0".
 */
#define KAPI_PARAM_SIZE_PARAM(idx) \
		.size_param_idx = (idx) + 1,

/**
 * KAPI_PARAM_COUNT - Set the number of parameters
 * @n: Number of parameters
 */
#define KAPI_PARAM_COUNT(n) \
	.param_magic = KAPI_MAGIC_PARAMS, \
	.param_count = n,

/**
 * KAPI_RETURN - Define return value specification
 * @rtype: Return type name
 * @rdesc: Return value description
 */
#define KAPI_RETURN(rtype, rdesc) \
	.return_magic = KAPI_MAGIC_RETURN, \
	.return_spec = {		\
		.type_name = rtype,	\
		.description = rdesc,

#define KAPI_RETURN_SUCCESS(val, ...) \
		.success_value = val,

#define KAPI_RETURN_TYPE(rtype) \
		.type = rtype,

#define KAPI_RETURN_CHECK_TYPE(ctype) \
		.check_type = ctype,

#define KAPI_RETURN_ERROR_VALUES(values) \
		.error_values = values,

#define KAPI_RETURN_ERROR_COUNT(count) \
		.error_count = count,

#define KAPI_RETURN_SUCCESS_RANGE(min, max) \
		.success_min = min, \
		.success_max = max,

/**
 * KAPI_ERROR - Define an error condition
 * @idx: Error index
 * @ecode: Error code value
 * @ename: Error name
 * @econd: Error condition
 * @edesc: Error description
 */
#define KAPI_ERROR(idx, ecode, ename, econd, edesc) \
	.errors[idx] = {			\
		.error_code = ecode,		\
		.name = ename,			\
		.condition = econd,		\
		.description = edesc,		\
	},

/**
 * KAPI_ERROR_COUNT - Set the number of errors
 * @n: Number of errors
 */
#define KAPI_ERROR_COUNT(n) \
	.error_magic = KAPI_MAGIC_ERRORS, \
	.error_count = n,

/**
 * KAPI_LOCK - Define a lock requirement
 * @idx: Lock index
 * @lname: Lock name
 * @ltype: Lock type
 */
#define KAPI_LOCK(idx, lname, ltype) \
	.locks[idx] = {			\
		.lock_name = lname,	\
		.lock_type = ltype,

#define KAPI_LOCK_ACQUIRED \
		.scope = KAPI_LOCK_ACQUIRES,

#define KAPI_LOCK_RELEASED \
		.scope = KAPI_LOCK_RELEASES,

#define KAPI_LOCK_HELD_ENTRY \
		.scope = KAPI_LOCK_CALLER_HELD,

#define KAPI_LOCK_HELD_EXIT \
		.scope = KAPI_LOCK_CALLER_HELD,

#define KAPI_LOCK_DESC(ldesc) \
		.description = ldesc,

/**
 * KAPI_CONSTRAINT - Define an additional constraint
 * @idx: Constraint index
 * @cname: Constraint name
 * @cdesc: Constraint description
 */
#define KAPI_CONSTRAINT(idx, cname, cdesc) \
	.constraints[idx] = {		\
		.name = cname,		\
		.description = cdesc,

#define KAPI_CONSTRAINT_EXPR(expr) \
		.expression = expr,

/**
 * KAPI_EXAMPLES - Set API usage examples
 * @ex: Examples string
 */
#define KAPI_EXAMPLES(ex) \
	.info_magic = KAPI_MAGIC_INFO, \
	.examples = ex,

/**
 * KAPI_NOTES - Set API notes
 * @n: Notes string
 */
#define KAPI_NOTES(n) \
	.notes = n,


/**
 * KAPI_SIGNAL - Define a signal specification
 * @idx: Signal index
 * @signum: Signal number (e.g., SIGKILL)
 * @signame: Signal name string
 * @dir: Direction flags
 * @act: Action taken
 */
#define KAPI_SIGNAL(idx, signum, signame, dir, act) \
	.signals[idx] = {			\
		.signal_num = signum,		\
		.signal_name = signame,		\
		.direction = dir,		\
		.action = act,

#define KAPI_SIGNAL_TARGET(tgt) \
		.target = tgt,

#define KAPI_SIGNAL_CONDITION(cond) \
		.condition = cond,

#define KAPI_SIGNAL_DESC(desc) \
		.description = desc,

#define KAPI_SIGNAL_RESTARTABLE \
		.restartable = true,

#define KAPI_SIGNAL_SA_FLAGS_REQ(flags) \
		.sa_flags_required = flags,

#define KAPI_SIGNAL_SA_FLAGS_FORBID(flags) \
		.sa_flags_forbidden = flags,

#define KAPI_SIGNAL_ERROR(err) \
		.error_on_signal = err,

#define KAPI_SIGNAL_TRANSFORM(sig) \
		.transform_to = sig,

#define KAPI_SIGNAL_TIMING(when) \
		.timing = when,

#define KAPI_SIGNAL_PRIORITY(prio) \
		.priority = prio,

#define KAPI_SIGNAL_INTERRUPTIBLE \
		.interruptible = true,

#define KAPI_SIGNAL_QUEUE(behavior) \
		.queue_behavior = behavior,

#define KAPI_SIGNAL_STATE_REQ(state) \
		.state_required = state,

#define KAPI_SIGNAL_STATE_FORBID(state) \
		.state_forbidden = state,

#define KAPI_SIGNAL_COUNT(n) \
	.signal_magic = KAPI_MAGIC_SIGNALS, \
	.signal_count = n,

/**
 * KAPI_SIGNAL_MASK - Define a signal mask specification
 * @idx: Mask index
 * @name: Mask name
 * @desc: Mask description
 */
#define KAPI_SIGNAL_MASK(idx, name, desc) \
	.signal_masks[idx] = {		\
		.mask_name = name,	\
		.description = desc,

/*
 * KAPI_SIGNAL_MASK_SIGNALS - Specify signals in a signal mask
 * @...: Variadic list of signal numbers
 *
 * Usage:
 *   KAPI_SIGNAL_MASK(0, "blocked", "Signals blocked during operation")
 *   KAPI_SIGNAL_MASK_SIGNALS(SIGINT, SIGTERM, SIGQUIT)
 *   },
 */
#define KAPI_SIGNAL_MASK_SIGNALS(...) \
		.signals = { __VA_ARGS__ }, \
		.signal_count = sizeof((int[]){ __VA_ARGS__ }) / sizeof(int),

/**
 * KAPI_SIGNAL_MASK_COUNT - Set the number of signal mask specifications
 * @n: Number of signal masks
 */
#define KAPI_SIGNAL_MASK_COUNT(n) \
	.sigmask_magic = KAPI_MAGIC_SIGMASK, \
	.signal_mask_count = n,

/**
 * KAPI_STRUCT_SPEC - Define a structure specification
 * @idx: Structure spec index
 * @sname: Structure name
 * @sdesc: Structure description
 */
#define KAPI_STRUCT_SPEC(idx, sname, sdesc) \
	.struct_specs[idx] = {		\
		.name = #sname,		\
		.description = sdesc,

#define KAPI_STRUCT_SIZE(ssize, salign) \
		.size = ssize,		\
		.alignment = salign,

#define KAPI_STRUCT_FIELD_COUNT(n) \
		.field_count = n,

/**
 * KAPI_STRUCT_FIELD - Define a structure field
 * @fidx: Field index
 * @fname: Field name
 * @ftype: Field type (KAPI_TYPE_*)
 * @ftype_name: Type name as string
 * @fdesc: Field description
 */
#define KAPI_STRUCT_FIELD(fidx, fname, ftype, ftype_name, fdesc) \
		.fields[fidx] = {	\
			.name = fname,	\
			.type = ftype,	\
			.type_name = ftype_name, \
			.description = fdesc,

#define KAPI_FIELD_OFFSET(foffset) \
			.offset = foffset,

#define KAPI_FIELD_SIZE(fsize) \
			.size = fsize,

#define KAPI_FIELD_FLAGS(fflags) \
			.flags = fflags,

#define KAPI_FIELD_CONSTRAINT_RANGE(min, max) \
			.constraint_type = KAPI_CONSTRAINT_RANGE, \
			.min_value = min, \
			.max_value = max,

#define KAPI_FIELD_CONSTRAINT_MASK(mask) \
			.constraint_type = KAPI_CONSTRAINT_MASK, \
			.valid_mask = mask,

#define KAPI_FIELD_CONSTRAINT_ENUM(values) \
			.constraint_type = KAPI_CONSTRAINT_ENUM, \
			.enum_values = values,

/* Counter for structure specifications */
#define KAPI_STRUCT_SPEC_COUNT(n) \
	.struct_magic = KAPI_MAGIC_STRUCTS, \
	.struct_spec_count = n,

/* Additional lock-related macros */
#define KAPI_LOCK_COUNT(n) \
	.lock_magic = KAPI_MAGIC_LOCKS, \
	.lock_count = n,

/**
 * KAPI_SIDE_EFFECT - Define a side effect
 * @idx: Side effect index
 * @etype: Effect type bitmask (OR'ed KAPI_EFFECT_* values)
 * @etarget: What is affected
 * @edesc: Effect description
 */
#define KAPI_SIDE_EFFECT(idx, etype, etarget, edesc) \
	.side_effects[idx] = {		\
		.type = etype,		\
		.target = etarget,	\
		.description = edesc,

#define KAPI_EFFECT_CONDITION(cond) \
		.condition = cond,

#define KAPI_EFFECT_REVERSIBLE \
		.reversible = true,

/**
 * KAPI_STATE_TRANS - Define a state transition
 * @idx: State transition index
 * @obj: Object whose state changes
 * @from: From state
 * @to: To state
 * @desc: Transition description
 */
#define KAPI_STATE_TRANS(idx, obj, from, to, desc) \
	.state_transitions[idx] = {	\
		.object = obj,		\
		.from_state = from,	\
		.to_state = to,		\
		.description = desc,

#define KAPI_STATE_TRANS_COND(cond) \
		.condition = cond,

/* Counters for side effects and state transitions */
#define KAPI_SIDE_EFFECT_COUNT(n) \
	.effect_magic = KAPI_MAGIC_EFFECTS, \
	.side_effect_count = n,

#define KAPI_STATE_TRANS_COUNT(n) \
	.trans_magic = KAPI_MAGIC_TRANS, \
	.state_trans_count = n,

/* Helper macros for common side effect patterns */
#define KAPI_EFFECTS_MEMORY	(KAPI_EFFECT_ALLOC_MEMORY | KAPI_EFFECT_FREE_MEMORY)
#define KAPI_EFFECTS_LOCKING	(KAPI_EFFECT_LOCK_ACQUIRE | KAPI_EFFECT_LOCK_RELEASE)
#define KAPI_EFFECTS_RESOURCES	(KAPI_EFFECT_RESOURCE_CREATE | KAPI_EFFECT_RESOURCE_DESTROY)
#define KAPI_EFFECTS_IO		(KAPI_EFFECT_NETWORK | KAPI_EFFECT_FILESYSTEM)

/* Common signal timing constants */
#define KAPI_SIGNAL_TIME_ENTRY		"entry"
#define KAPI_SIGNAL_TIME_DURING		"during"
#define KAPI_SIGNAL_TIME_EXIT		"exit"
#define KAPI_SIGNAL_TIME_ANYTIME	"anytime"
#define KAPI_SIGNAL_TIME_BLOCKING	"while_blocked"
#define KAPI_SIGNAL_TIME_SLEEPING	"while_sleeping"
#define KAPI_SIGNAL_TIME_BEFORE		"before"
#define KAPI_SIGNAL_TIME_AFTER		"after"

/* Common signal queue behaviors */
#define KAPI_SIGNAL_QUEUE_STANDARD	"standard"
#define KAPI_SIGNAL_QUEUE_REALTIME	"realtime"
#define KAPI_SIGNAL_QUEUE_COALESCE	"coalesce"
#define KAPI_SIGNAL_QUEUE_REPLACE	"replace"
#define KAPI_SIGNAL_QUEUE_DISCARD	"discard"

/* Process state flags for signal delivery */
#define KAPI_SIGNAL_STATE_RUNNING	BIT(0)
#define KAPI_SIGNAL_STATE_SLEEPING	BIT(1)
#define KAPI_SIGNAL_STATE_STOPPED	BIT(2)
#define KAPI_SIGNAL_STATE_TRACED	BIT(3)
#define KAPI_SIGNAL_STATE_ZOMBIE	BIT(4)
#define KAPI_SIGNAL_STATE_DEAD		BIT(5)

/* Capability specification macros */

/**
 * KAPI_CAPABILITY - Define a capability requirement
 * @idx: Capability index
 * @cap: Capability constant (e.g., CAP_IPC_LOCK)
 * @name: Capability name string
 * @act: Action type (kapi_capability_action)
 */
#define KAPI_CAPABILITY(idx, cap, name, act) \
	.capabilities[idx] = {		\
		.capability = cap,	\
		.cap_name = name,	\
		.action = act,

#define KAPI_CAP_ALLOWS(desc) \
		.allows = desc,

#define KAPI_CAP_WITHOUT(desc) \
		.without_cap = desc,

#define KAPI_CAP_CONDITION(cond) \
		.check_condition = cond,

#define KAPI_CAP_PRIORITY(prio) \
		.priority = prio,

/*
 * KAPI_CAP_ALTERNATIVE - Capabilities that can be used instead
 * @...: Variadic list of capability numbers
 */
#define KAPI_CAP_ALTERNATIVE(...) \
		.alternative = { __VA_ARGS__ }, \
		.alternative_count = sizeof((int[]){ __VA_ARGS__ }) / sizeof(int),

/* Counter for capability specifications */
#define KAPI_CAPABILITY_COUNT(n) \
	.cap_magic = KAPI_MAGIC_CAPS, \
	.capability_count = n,

/* Validation and runtime checking */

#ifdef CONFIG_KAPI_RUNTIME_CHECKS
bool kapi_validate_param(const struct kapi_param_spec *param_spec, s64 value);
bool kapi_validate_param_with_context(const struct kapi_param_spec *param_spec,
				       s64 value, const s64 *all_params, int param_count);
bool kapi_check_return_success(const struct kapi_return_spec *return_spec, s64 retval);
bool kapi_validate_return_value(const struct kernel_api_spec *spec, s64 retval);
void kapi_check_context(const struct kernel_api_spec *spec);
#else
static inline bool kapi_validate_param(const struct kapi_param_spec *param_spec, s64 value)
{
	return true;
}
static inline bool
kapi_validate_param_with_context(const struct kapi_param_spec *param_spec,
				 s64 value, const s64 *all_params, int param_count)
{
	return true;
}
static inline bool kapi_check_return_success(const struct kapi_return_spec *return_spec, s64 retval)
{
	return true;
}
static inline bool kapi_validate_return_value(const struct kernel_api_spec *spec, s64 retval)
{
	return true;
}
static inline void kapi_check_context(const struct kernel_api_spec *spec) {}
#endif

/* Export/query functions */
int kapi_export_json(const struct kernel_api_spec *spec, char *buf, size_t size);

/* Registration for dynamic APIs */
int kapi_register_spec(const struct kernel_api_spec *spec);
void kapi_unregister_spec(const char *name);

#define KAPI_CONSTRAINT_COUNT(n) \
	.constraint_magic = KAPI_MAGIC_CONSTRAINTS, \
	.constraint_count = n,

#endif /* _LINUX_KERNEL_API_SPEC_H */
