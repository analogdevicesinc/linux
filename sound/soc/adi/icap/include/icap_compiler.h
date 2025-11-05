/* SPDX-License-Identifier: (GPL-2.0-or-later OR Apache-2.0) */

/*
 *  Copyright 2021-2022 Analog Devices Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*
 * Copyright (C) 2021-2022 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

/*
 * Authors:
 *   Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 */

#ifndef _ICAP_COMPILER_H_
#define _ICAP_COMPILER_H_

/**
 * @file icap_compiler.h
 * @brief Compiler specific definitions
 *
 */

#if defined(__CCESVERSION__)
/* Cross Code Embedded Studio project */
#define ICAP_PACKED_BEGIN _Pragma("pack(1)")
#define ICAP_PACKED_END _Pragma("pack()")

#else
/* GCC */
#define ICAP_PACKED_BEGIN
#define ICAP_PACKED_END __packed
#endif

#endif /* _ICAP_COMPILER_H_ */
