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

#ifndef __IPU_TRACE_H__
#define __IPU_TRACE_H__

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern uint32_t    dce_debug;

/********************* MACROS ************************/
/* Need to make it OS specific and support different trace levels */

/* set desired trace level:
 *   1 - error
 *   2 - error, debug
 *   3 - error, debug, info  (very verbose)
 *   4 - error, info, debug, verbose (very very verbose)
 */
#ifdef DCE_DEBUG_ENABLE
#define ERROR(FMT, ...)   TRACE(1, "ERROR: " FMT, ##__VA_ARGS__)
#define DEBUG(FMT, ...)   TRACE(2, "DEBUG: " FMT, ##__VA_ARGS__)
#define INFO(FMT, ...)    TRACE(3, "INFO: " FMT, ##__VA_ARGS__)
#define VERB(FMT, ...)    TRACE(4, "INFO: " FMT, ##__VA_ARGS__)
#else
#define ERROR(FMT, ...)
#define DEBUG(FMT, ...)
#define INFO(FMT, ...)
#define VERB(FMT, ...)
#endif

#define MAX_DEBUG_LEVEL (4)
#define TRACE(lvl, FMT, ...)  do { if((lvl) <= dce_debug ) { \
                                       System_printf("%s:%d:\t%s\t" FMT "\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__); \
                                   } } while( 0 )

#endif /* __IPU_TRACE_H__ */
