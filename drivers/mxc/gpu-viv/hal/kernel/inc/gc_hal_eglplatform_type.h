/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2023 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2023 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


 *                                                                            *
 ******************************************************************************/

#ifndef __gc_hal_eglplatform_type_h_
#define __gc_hal_eglplatform_type_h_

#ifdef __cplusplus
extern "C" {
#endif

/* Structure that defined keyboard mapping. */
typedef struct _halKeyMap {
    /* Normal key. */
    halKeys normal;

    /* Extended key. */
    halKeys extended;
} halKeyMap;

/* Event structure. */
typedef struct _halEvent {
    /* Event type. */
    halEventType type;

    /* Event data union. */
    union _halEventData {
        /* Event data for keyboard. */
        struct _halKeyboard {
            /* Scancode. */
            halKeys scancode;

            /* ASCII characte of the key pressed. */
            char    key;

            /* Flag whether the key was pressed (1) or released (0). */
            char    pressed;
        } keyboard;

        /* Event data for pointer. */
        struct _halPointer {
            /* Current pointer coordinate. */
            int     x;
            int     y;
        } pointer;

        /* Event data for mouse buttons. */
        struct _halButton {
            /* Left button state. */
            int     left;

            /* Middle button state. */
            int     middle;

            /* Right button state. */
            int     right;

            /* Current pointer coordinate. */
            int     x;
            int     y;
        } button;
    } data;
} halEvent;

/* VFK_DISPLAY_INFO structure defining information returned by
 * vdkGetDisplayInfoEx.
 */
typedef struct _halDISPLAY_INFO {
    /* The size of the display in pixels. */
    int                 width;
    int                 height;

    /* The stride of the dispay. -1 is returned if the stride is not known
     * for the specified display.
     */
    int                 stride;

    /* The color depth of the display in bits per pixel. */
    int                 bitsPerPixel;

    /* The logical pointer to the display memory buffer. NULL is returned
     * if the pointer is not known for the specified display.
     */
    void                *logical;

    /* The physical address of the display memory buffer. ~0 is returned
     * if the address is not known for the specified display.
     */
    unsigned long       physical;

    /* Can be wraped as surface. */
    int                 wrapFB;

    /* FB_MULTI_BUFFER support */
    int                 multiBuffer;
    int                 backBufferY;

    /* Tiled buffer / tile status support. */
    int                 tiledBuffer;
    int                 tileStatus;
    int                 compression;

    /* The color info of the display. */
    unsigned int        alphaLength;
    unsigned int        alphaOffset;
    unsigned int        redLength;
    unsigned int        redOffset;
    unsigned int        greenLength;
    unsigned int        greenOffset;
    unsigned int        blueLength;
    unsigned int        blueOffset;

    /* Display flip support. */
    int flip;
} halDISPLAY_INFO;

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_eglplatform_type_h_ */


