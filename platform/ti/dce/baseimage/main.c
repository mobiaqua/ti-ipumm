/*
 * Copyright (c) 2011-2015, Texas Instruments Incorporated
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

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/ipc/rpmsg/_RPMessage.h>
#include <ti/ipc/remoteproc/Resource.h>

#include <ti/utils/osal/trace.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <platform/ti/dce/baselib/ipumm_main.h>

// Include the custom resource table for memory configuration.
#if (defined VAYU_ES10)
   #if (defined BUILD_FOR_QNX)
   #include "qnx_custom_rsc_table_vayu_ipu.h"
   #else
   #include "custom_rsc_table_vayu_ipu.h"
   #endif
#elif (defined OMAP5432_ES20)
#include "custom_rsc_table_omap5_ipu.h"
#elif (defined BUILD_FOR_OMAP4)
#include "custom_rsc_table_omap4_ipu.h"
#endif

extern uint32_t    dce_debug;
extern Uint32 kpi_control;

static unsigned int SyslinkMemUtils_VirtToPhys(Ptr Addr)
{
    unsigned int    pa;

    if( !Addr || Resource_virtToPhys((unsigned int) Addr, &pa)) {
        return (0);
    }
    return (pa);
}

#pragma DATA_SECTION(ipummversion, ".ipummversion")
char ipummversion[128] = ducati_ver_tag;

void tools_ShowVersion()
{
    // ipummversion is needed here or else the compiler will remove ipummversion thinking that it is not used.
    System_printf("\n\n **** IPUMM VERSION INFO **** \n\nCompile DATE %s TIME %s \n", __DATE__, __TIME__, ipummversion);

    System_printf("CODEC-VER BEGIN: \n");

    System_printf("\t%s\n\t%s\n\t%s\n\t%s\n\t%s\n \n",
                  ducati_ver_h264d, ducati_ver_mpeg4d, ducati_ver_mpeg2d, ducati_ver_vc1d, ducati_ver_mjpegd);

    System_printf("\t%s\n\t%s\n\t%s\n",
                  ducati_ver_h264e, ducati_ver_mpeg4e, ducati_ver_mjpege);

    System_printf("CODEC-VER END: \n");

    System_printf("\n** IPUMM VERSION INFO END ** \n");

    System_printf("Trace level PA 0x%x Trace Level %d\
                   \nTrace Usage: level:[0-4: 0-no trace, 1-err, 2-debug, 3-info, 4-CE,FC,IPC traces] \n\n",
                  SyslinkMemUtils_VirtToPhys(&dce_debug), dce_debug);
    System_printf("Trace Buffer PA 0x%x kpi_control (PA 0x%x value 0x%x)\n", SyslinkMemUtils_VirtToPhys((Ptr)(TRACEBUFADDR)), SyslinkMemUtils_VirtToPhys((Ptr)(&kpi_control)), kpi_control);
}

int main(int argc, char * *argv)
{
    IPUMM_Main(argc, argv);

    /* Dump Tools version */
    tools_ShowVersion();

    BIOS_start();

    return (0);
}
