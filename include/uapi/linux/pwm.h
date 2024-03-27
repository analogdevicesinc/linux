/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */

#ifndef _UAPI_PWM_H_
#define _UAPI_PWM_H_

#include <linux/ioctl.h>
#include <linux/types.h>

struct pwmchip_state {
	unsigned int hwpwm;
	__u64 period;
	__u64 duty_cycle;
	__u64 duty_offset;
};

#define PWM_IOCTL_GET_NUM_PWMS	_IO(0x75, 0)
#define PWM_IOCTL_REQUEST	_IOW(0x75, 1, unsigned int)
#define PWM_IOCTL_FREE		_IOW(0x75, 2, unsigned int)
/* reserve nr = 3 for rounding */
#define PWM_IOCTL_GET		_IOWR(0x75, 4, struct pwmchip_state)
#define PWM_IOCTL_APPLY		_IOW(0x75, 5, struct pwmchip_state)

#endif /* _UAPI_PWM_H_ */
