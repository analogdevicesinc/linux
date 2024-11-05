/* Copyright (c) 2008-2012 Freescale Semiconductor, Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**************************************************************************//**
 @File          mm_ext.h

 @Description   Memory Manager Application Programming Interface
*//***************************************************************************/
#ifndef __MM_EXT
#define __MM_EXT

#include "std_ext.h"

#define MM_MAX_ALIGNMENT    20  /* Alignments from 2 to 128 are available
                                   where maximum alignment defined as
                                   MM_MAX_ALIGNMENT power of 2 */

#define MM_MAX_NAME_LEN     32

/**************************************************************************//**
 @Group         etc_id   Utility Library Application Programming Interface

 @Description   External routines.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         mm_grp Flexible Memory Manager

 @Description   Flexible Memory Manager module functions,definitions and enums.
                (All of the following functions,definitions and enums can be found in mm_ext.h)

 @{
*//***************************************************************************/


/**************************************************************************//**
 @Function      MM_Init

 @Description   Initializes a new MM object.

                It initializes a new memory block consisting of base address
                and size of the available memory by calling to MemBlock_Init
                routine. It is also initializes a new free block for each
                by calling FreeBlock_Init routine, which is pointed to
                the almost all memory started from the required alignment
                from the base address and to the end of the memory.
                The handle to the new MM object is returned via "MM"
                argument (passed by reference).

 @Param[in]     h_MM    - Handle to the MM object.
 @Param[in]     base    - Base address of the MM.
 @Param[in]     size    - Size of the MM.

 @Return        E_OK is returned on success. E_NOMEMORY is returned if the new MM object or a new free block can not be initialized.
*//***************************************************************************/
t_Error     MM_Init(t_Handle *h_MM, uint64_t base, uint64_t size);

/**************************************************************************//**
 @Function      MM_Get

 @Description   Allocates a block of memory according to the given size and the alignment.

                The Alignment argument tells from which
                free list allocate a block of memory. 2^alignment indicates
                the alignment that the base address of the allocated block
                should have. So, the only values 1, 2, 4, 8, 16, 32 and 64
                are available for the alignment argument.
                The routine passes through the specific free list of free
                blocks and seeks for a first block that have anough memory
                that  is required (best fit).
                After the block is found and data is allocated, it calls
                the internal MM_CutFree routine to update all free lists
                do not include a just allocated block. Of course, each
                free list contains a free blocks with the same alignment.
                It is also creates a busy block that holds
                information about an allocated block.

 @Param[in]     h_MM        - Handle to the MM object.
 @Param[in]     size        - Size of the MM.
 @Param[in]     alignment   - Index as a power of two defines a required
                              alignment (in bytes); Should be 1, 2, 4, 8, 16, 32 or 64
 @Param[in]     name        - The name that specifies an allocated block.

 @Return        base address of an allocated block ILLEGAL_BASE if can't allocate a block
*//***************************************************************************/
uint64_t    MM_Get(t_Handle h_MM, uint64_t size, uint64_t alignment, char *name);

/**************************************************************************//**
 @Function      MM_GetForce

 @Description   Force memory allocation.

                It means to allocate a block of memory of the given
                size from the given base address.
                The routine checks if the required block can be allocated
                (that is it is free) and then, calls the internal MM_CutFree
                routine to update all free lists do not include that block.

 @Param[in]     h_MM    - Handle to the MM object.
 @Param[in]     base    - Base address of the MM.
 @Param[in]     size    - Size of the MM.
 @Param[in]     name    - Name that specifies an allocated block.

 @Return        base address of an allocated block, ILLEGAL_BASE if can't allocate a block.
*//***************************************************************************/
uint64_t    MM_GetForce(t_Handle h_MM, uint64_t base, uint64_t size, char *name);

/**************************************************************************//**
 @Function      MM_Put

 @Description   Puts a block of memory of the given base address back to the memory.

                It checks if there is a busy block with the
                given base address. If not, it returns 0, that
                means can't free a block. Otherwise, it gets parameters of
                the busy block and after it updates lists of free blocks,
                removes that busy block from the list by calling to MM_CutBusy
                routine.
                After that it calls to MM_AddFree routine to add a new free
                block to the free lists.

 @Param[in]     h_MM    - Handle to the MM object.
 @Param[in]     base    - Base address of the MM.

 @Return         The size of bytes released, 0 if failed.
*//***************************************************************************/
uint64_t    MM_Put(t_Handle h_MM, uint64_t base);

/**************************************************************************//**
 @Function      MM_Dump

 @Description   Prints results of free and busy lists.

 @Param[in]     h_MM        - Handle to the MM object.
*//***************************************************************************/
void        MM_Dump(t_Handle h_MM);

/**************************************************************************//**
 @Function      MM_Free

 @Description   Releases memory allocated for MM object.

 @Param[in]     h_MM - Handle of the MM object.
*//***************************************************************************/
void        MM_Free(t_Handle h_MM);

/**************************************************************************//**
 @Function      MM_GetFreeMemSize

 @Description   Returns the size (in bytes) of free memory.

 @Param[in]     h_MM    - Handle to the MM object.

 @Return        Free memory size in bytes.
*//***************************************************************************/
uint64_t MM_GetFreeMemSize(t_Handle h_MM);


/** @} */ /* end of mm_grp group */
/** @} */ /* end of etc_id group */

#endif /* __MM_EXT_H */
