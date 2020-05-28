/*
 * Copyright (c) 2010, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DCE_PRIV_H__
#define __DCE_PRIV_H__
#include <ti/utils/osal/trace.h>
#include <ti/xdais/dm/xdm.h>
#include <ti/sdo/ce/video3/viddec3.h>
#include <ti/sdo/ce/video2/videnc2.h>

Bool dce_init(void);

/* these acquire/release functions should be implemented by the platform.
 * These are called from dce.c before/after the process() call.
 */
void ivahd_acquire(void);
void ivahd_release(void);
void ivahd_idle_check(void);
void ivahd_init(uint32_t chipset_id);
void ivahd_boot();

XDM_DataSyncGetFxn H264E_GetDataFxn(XDM_DataSyncHandle dataSyncHandle, XDM_DataSyncDesc *dataSyncDesc);
XDM_DataSyncPutFxn H264D_PutDataFxn(XDM_DataSyncHandle dataSyncHandle, XDM_DataSyncDesc *dataSyncDesc);

#ifndef   DIM
#  define DIM(a) (sizeof((a)) / sizeof((a)[0]))
#endif


#ifndef TRUE
#  define TRUE 1
#endif
#ifndef FALSE
#  define FALSE 0
#endif
#ifndef NULL
#  define NULL ((void *)0)
#endif

typedef struct MemHeader {
    uint32_t size;
    void    *ptr;       /* when used for BIOS heap blocks, just a raw ptr */
    int32_t dma_buf_fd; /* shared dma buf fd */
    uint32_t region;    /* mem region the buffer allocated from */
    /* internal meta data for the buffer */
    uint32_t offset;    /* offset for the actual data with in the buffer */
    int32_t map_fd;     /* mmapped fd */
    void * handle;      /*custom handle for the HLOS memallocator*/
    int32_t flags;      /*Flag to hold memory attributes*/
} MemHeader;

#define P2H(p) (&(((MemHeader *)(p))[-1]))
#define H2P(h) ((void *)&(h)[1])

#ifndef __packed
#  define __packed __attribute__((packed))
#endif

#endif /* __DCE_PRIV_H__ */
