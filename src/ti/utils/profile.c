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
/*
*   @file  profile.c
*   This file contains kpi psi instrumentation code for cpu load as well
*   as tracing codec + BIOS Events
*
*   @path src/ti/utils
*
*  @rev 1.0
*/

/* PSI_KPI profiler */
#include "profile.h"

#include <string.h>
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>
#ifdef  BUILD_FOR_SMP
#include <ti/sysbios/hal/Core.h>
#endif //BUILD_FOR_SMP
#include <ti/sysbios/family/arm/ducati/TimestampProvider.h>
#include <ti/pm/IpcPower.h>
#include <ti/ipc/remoteproc/Resource.h>

#define REG_32K (0x4AE04030) //J6 and OMAP5
#define TRACEGRP 0
#define MAX_STRINGNAME_SIZE 255

/* private function prototypes */
void kpi_IVA_profiler_init  (void);
void kpi_CPU_profiler_init  (void);
void kpi_IVA_profiler_print (void);
void kpi_CPU_profiler_print (void);
void psi_kpi_task_test(Task_Handle prev, Task_Handle next);


/***************************************************************
 * psi_tsk_info
 * -------------------------------------------------------------
 * Structure maintaining task data for Duacti CPU load
 *
 ***************************************************************/
#define KPI_CONTEXT_TASK  0
#define KPI_CONTEXT_SWI   1
#define KPI_CONTEXT_HWI   2

typedef struct {
    void         *handle;     /* Pointer to task handle, used to identify task */
    char          name[50];   /* Task name */
    unsigned long total_time; /* Total time spent in the task context */
    unsigned long nb_switch;  /* Number of times the context get activated */
} psi_context_info;


/***************************************************************
 * psi_bios_kpi
 * -------------------------------------------------------------
 * Structure maintaining data for Ducati CPU load
 *
 ***************************************************************/
#define CTX_DIRECT_NUM   256
#define KPI_MAX_NB_TASKS 64
#define KPI_MAX_NB_SWI   16
#define KPI_MAX_NB_HWI   16
#define MAX_NB_IT 32

typedef struct {
    unsigned long prev_t32;                       /* Store T32K value of previous task switch */
    unsigned long total_time;                     /* Total processing time */
    unsigned long kpi_time;                       /* Total instrumentation processing time */
    unsigned long kpi_count;                      /* Number of time instrumentation run */
    unsigned long nb_switch;                      /* Number of context switches */

    psi_context_info tasks[KPI_MAX_NB_TASKS];     /* Tasks info DB */
    unsigned long    nb_tasks;                    /* Number of tasks in task info */
    psi_context_info hwi[KPI_MAX_NB_SWI];         /* HWI info DB */
    unsigned long    nb_hwi;                      /* Number of HWI in HWI info */
    psi_context_info swi[KPI_MAX_NB_HWI];         /* SWI info DB */
    unsigned long    nb_swi;                      /* Number of SWI in SWI info */

    psi_context_info *context_cache[CTX_DIRECT_NUM]; /* pointers to tasks info DB */

    void   *context_stack[MAX_NB_IT];             /* to keep handles because of swi hwi */
    void * *pt_context_stack;                     /* stack pointer */

    unsigned long *ptr_idle_total;                /* to access to idle task total directly */
    unsigned long  idle_prev;                     /* total ticks of idle task previously */
    unsigned long  idle_time_prev;                /* time of idle_prev measure */

    unsigned long trace_time;                     /* time spent tracing */
} psi_bios_kpi;


/***************************************************************
 * ivahd_kpi
 * -------------------------------------------------------------
 * Structure maintaining data for IVA-HD load and fps
 *
 ***************************************************************/
typedef struct {
    unsigned long ivahd_t_tot;       /* IVA-HD tot processing time per frame */
    unsigned long ivahd_t_max;       /* IVA-HD max processing time per frame */
    unsigned long ivahd_t_min;       /* IVA-HD min processing time per frame */
    unsigned long ivahd_t_max_frame;
    unsigned long ivahd_t_min_frame;
    unsigned long ivahd_MHz;
    unsigned long ivahd_MHzTime;

    unsigned long nb_frames;     /* Number of frames      */

    unsigned long before_time;       /* T32K before codec execution */
    unsigned long after_time;        /* T32K after codec execution */

    unsigned long t32k_start;        /* T32K value at the beginning of video decode */
    unsigned long t32k_end;          /* T32K value at the end of video decode */
    unsigned long t32k_mpu_time;     /* T32K value at MPU side */
} psi_iva_kpi;


/***************************************************************
 * Globals
 ***************************************************************/
unsigned long    kpi_control = 0;    /* instrumentation control (set with omapconf) */
unsigned long    kpi_status  = 0;    /* instrumentation status variables */

psi_iva_kpi     iva_kpi;             /* IVA data base */
psi_bios_kpi    bios_kpi[2];         /* CPU data base (2 cores) */


/***************************************************************
 * Functions
 ***************************************************************/

#ifndef BUILD_FOR_SMP
/***************************************************************
 * Core_getId
 * -------------------------------------------------------------
 * Temporary function to remove with SMP BIOS
 *
 * @params: none
 *
 * @return: 1 to force CoreId to 1
 *
 ***************************************************************/
unsigned long Core_getId(void)
{
    return (1);
}

#else

#endif //BUILD_FOR_SMP

/***************************************************************
 * get_32k
 * -------------------------------------------------------------
 * Function used to get 32k timer value
 *
 * @params: none
 *
 * @return: T32k value
 *
 ***************************************************************/
unsigned long get_32k(void)
{
    Uint32 va;
    volatile Uint32 pa = REG_32K;
    if(!Resource_physToVirt(pa, &va)) {
        //DEBUG("pa = 0x%x, va = 0x%x\n", pa, va);
        return *( (volatile Uint32*)va);
    }
    else {
        System_printf("Unable to read the timer value\n");
        return 0;
    }
}

/***************************************************************
 * get_time
 * -------------------------------------------------------------
 * Left here for test
 *
 * @params: none
 *
 * @return: CTM timer value / 4096 (~100KHz)
 *          or 32K value (slower to read)
 *
 ***************************************************************/
inline unsigned long get_time(void)
{
    return (get_32k());
}

/***************************************************************
 * get_time_core
 * -------------------------------------------------------------
 * Left here for test
 *
 * @params: int core
 *
 * @return: CTM timer value / 4096 (~100KHz)
 *          or 32K value (slower to read)
 *
 * This function is equivalent to get_time().
 * it can be called when interrupts have been masked
 * and when current core Id is known already (therefore faster)
 ***************************************************************/
inline unsigned long get_time_core(int core)
{
    return (get_32k());
}

/***************************************************************
 * set_WKUPAON
 * -------------------------------------------------------------
 * Function used to get fast access to the 32k timer value
 *
 * @params: int force_restore (1 or 0)
 *
 * @return: none
 *
 * This function prevents sleep mode to the L4 WKUPAON_CM
 * register, to avoid large stalls when reading the 32k timer
 ***************************************************************/
/* Commenting set_WKUPAON() to make sure it isn't used. Need to
 * investigate if this function is still needed. This code is
 * attempting to directly access a physical address when only
 * virtual addresses should be accessed. This function is not
 * currently called. If we don't need this function, can we
 * remove it?
 */
#if 0
extern void *MEMUTILS_getPhysicalAddr(Ptr vaddr);
void set_WKUPAON(int force_restore)
{
    static unsigned long    clktrctrl_01=0;
    unsigned long  reg = *((volatile Uint32 *)
                          MEMUTILS_getPhysicalAddr((Ptr)0xAAE07800));

    /* Force nosleep mode or restore original configuration */
    if( force_restore == 1 ) {
        clktrctrl_01 = reg & 0x3;     /* save clktrctrl */
        reg &= 0xfffffffc;
        //reg |= 0x2;                     /* force SW_WAKEUP */
        reg |= 0;                     /* force NO_SLEEP */
    } else {
        reg &= 0xfffffffc;
        reg |= clktrctrl_01;          /* restore bits 01 */
        clktrctrl_01 = 0;             /* restore done */
    }
    *(Uint32 *)0xAAE07800 = reg;
}
#endif

/***************************************************************
 * PSI_TracePrintf
 * -------------------------------------------------------------
 * Function used to get 32k timer value
 *
 * @params: TIMM_OSAL_TRACEGRP eTraceGrp, TIMM_OSAL_CHAR *pcFormat, ...
 *
 * @return: none
 *
 ***************************************************************/
void PSI_TracePrintf(Uint32 eTraceGrp, char *pcFormat, ...)
{
    //  static TIMM_OSAL_PTR MyMutex = NULL;
    unsigned long    tstart = get_time();
    unsigned long    key = Task_disable();
    unsigned long    CoreId = Core_getId();

    va_list    varArgs;

    va_start(varArgs, pcFormat);
    System_vprintf(pcFormat, varArgs);
    va_end(varArgs);

    Task_restore(key);

    /* count overhead due to tracing when running (phase 1)*/
    if( kpi_status & KPI_INST_RUN ) {
        bios_kpi[CoreId].trace_time += get_time() - tstart;
    }

}

/***************************************************************
 * kpi_instInit
 * -------------------------------------------------------------
 * Function used to do the aquisition of the instrumentation
 * setting. Then initialising the DBs and variables
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void kpi_instInit(void)
{
    /* don't change setup when already active */
    if( kpi_status & KPI_INST_RUN ) {
        return;
    }

    /* read control from the memory */

    /* Note:
     * kpi_control |= KPI_END_SUMMARY;     bit0 : Activate the IVA and CPU load measure and print summary
     * kpi_control |= KPI_IVA_DETAILS;     bit1 : Activate the IVA trace
     * kpi_control |= KPI_COMP_DETAILS;    bit2 : Activate the COMP trace
     * kpi_control |= KPI_CPU_DETAILS;     bit3 : Activate the CPU Idle trace
     */

    /* reset kpi_status */
    kpi_status = 0;

    /* nothing to do if no instrumentation required */
    if( !kpi_control ) {
        return;
    }

    /* force clktrctrl to no sleep mode (fast 32k access) */
    //set_WKUPAON( 1 );
#ifdef  USE_CTM_TIMER
    IpcPower_wakeLock();
#endif /*USE_CTM_TIMER*/

    /* IVA load setup */
    if( kpi_control & (KPI_END_SUMMARY | KPI_IVA_DETAILS)) {
        /* Initialize the IVA data base */
        kpi_IVA_profiler_init();
        kpi_status |= KPI_IVA_LOAD;
        if( kpi_control & KPI_IVA_DETAILS ) {
            kpi_status |= KPI_IVA_TRACE;
        }
    }

    /* CPU load setup */
    if( kpi_control & (KPI_END_SUMMARY | KPI_CPU_DETAILS)) {
        /* Initialize the CPU data base */
        kpi_CPU_profiler_init();
        kpi_status |= KPI_CPU_LOAD;
        if( kpi_control & KPI_CPU_DETAILS ) {
            kpi_status |= KPI_CPU_TRACE;
        }
    }

    /* COMP trace setup */
    if( kpi_control & KPI_COMP_DETAILS ) {
        kpi_status |= KPI_COMP_TRACE;
    }

    /* Mark as running */
    kpi_status |= KPI_INST_RUN;

}

/***************************************************************
 * kpi_instDeinit
 * -------------------------------------------------------------
 * Function used stop the instrumentation
 * Then initialising the summary if required
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void kpi_instDeinit(void)
{
    unsigned long    cpu_summary=0;
    unsigned long    iva_summary=0;

    /* noting to do when not running */
    if( !(kpi_status & KPI_INST_RUN)) {
        return;
    }

    if( kpi_control & KPI_END_SUMMARY ) {
        cpu_summary = kpi_status & KPI_CPU_LOAD;
        iva_summary = kpi_status & KPI_IVA_LOAD;
    }

    /* now stop everything since measure is completed */
    kpi_status = 0;

    /* Print the summarys */
    PSI_TracePrintf(TRACEGRP, "\n<KPI> Profiler Deinit %-8lu\n", get_32k());
    if( iva_summary ) {
        kpi_IVA_profiler_print();
    }
    if( cpu_summary ) {
        kpi_CPU_profiler_print();
    }

    /* restore clktrctrl register */
    //set_WKUPAON( 0 );
#ifdef  USE_CTM_TIMER
    IpcPower_wakeUnlock();
#endif /*USE_CTM_TIMER*/

}

/***************************/
/* IVA fps and IVA-HD load */
/***************************/

#ifdef  CPU_LOAD_DETAILS
/***************************************************************
 * psi_cpu_load_details
 * -------------------------------------------------------------
 * Function to be called once a frame
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void psi_cpu_load_details(void)
{
    unsigned long    time_curr = get_time();
    unsigned long    CoreId, time_delta, idle_delta, idle;
    psi_bios_kpi    *core_kpi;
    char             trace[2][50];

    /* calculate delta_time and delta_idle */
    for( CoreId = 0; CoreId < 2; CoreId++ ) {
        core_kpi = &bios_kpi[CoreId];
        System_sprintf(trace[CoreId], "");
        idle_delta= *(core_kpi->ptr_idle_total) - core_kpi->idle_prev;
        time_delta = time_curr - core_kpi->idle_time_prev;
        /* calculate idle value only if it was updated (idle task switch happend at least once!) */
        if( idle_delta ) {
            core_kpi->idle_time_prev = time_curr;
            core_kpi->idle_prev = *(core_kpi->ptr_idle_total);
            if( time_delta ) {
                /* calculate Idle %time */
                idle = ((idle_delta * 100) + time_delta / 2) / time_delta;
                System_sprintf(trace[CoreId], "Idle.%lu : %lu%%", CoreId, idle);
            }
        }
    }

    PSI_TracePrintf(TRACEGRP, "%s   %s\n", trace[0], trace[1]);
}

#endif //CPU_LOAD_DETAILS


/***************************************************************
 * kpi_IVA_profiler_init
 * -------------------------------------------------------------
 * Function to be called at start of processing.
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void kpi_IVA_profiler_init(void)
{

    PSI_TracePrintf(TRACEGRP, "<KPI> IVA Profiler Init %-8lu\n", get_32k());

    iva_kpi.ivahd_t_tot       = 0;       /* IVA-HD tot processing time per frame */
    iva_kpi.ivahd_t_max       = 0;       /* IVA-HD max processing time per frame */
    iva_kpi.ivahd_t_min       =~0;       /* IVA-HD min processing time per frame */
    iva_kpi.ivahd_t_max_frame = 0;
    iva_kpi.ivahd_t_min_frame = 0;
    iva_kpi.ivahd_MHzTime     = 0;

    iva_kpi.nb_frames     = 0;       /* Number of frames      */

    iva_kpi.before_time       = 0;
    iva_kpi.after_time        = 0;

    iva_kpi.t32k_start        = 0;
    iva_kpi.t32k_end          = 0;
    iva_kpi.t32k_mpu_time     = 0;

}

/***************************************************************
 * kpi_before_codec
 * -------------------------------------------------------------
 * Function to be called before codec execution.
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void kpi_before_codec(void)
{
    unsigned long    start, prev, delta;

#ifndef COMP_ENABLE_DUCATI_LOAD
    /* Started 1st time */
    if( !(kpi_status & KPI_INST_RUN)) {
        kpi_instInit();
    }
#endif //COMP_ENABLE_DUCATI_LOAD

    if( kpi_status & KPI_IVA_LOAD ) {
        /* read the 32k timer */
        start = get_32k();

        prev = iva_kpi.before_time;
        iva_kpi.before_time = start;

        /* record very 1st time codec is used */
        if( !iva_kpi.t32k_start ) {
            iva_kpi.t32k_start = iva_kpi.before_time;
            iva_kpi.t32k_mpu_time = 0;  /* before the 1st codec call */
        } else {
            /* calculate the time from after codec done (iva_kpi.after_time) until now (iva_kpi.before_time) which is the time on MPU side */
            iva_kpi.t32k_mpu_time += iva_kpi.before_time - iva_kpi.after_time;
        }

        /* calculate delta frame time in ticks */
        delta = start - prev;
        if( !iva_kpi.nb_frames ) {
            delta = 0;
        }

#ifdef  IVA_DETAILS
        if( kpi_status & KPI_IVA_TRACE ) {
            PSI_TracePrintf(TRACEGRP, "BEG %-7s %-4u %-8lu %d\n",
                            "IVA", iva_kpi.nb_frames + 1, start, (delta * 100000) / 32768);
        }
#endif /*IVA_DETAILS*/

    }

#ifdef  CPU_LOAD_DETAILS
    if( kpi_status & KPI_CPU_TRACE ) {
        kpi_status |= KPI_IVA_USED; /* mark IVA actually used */
        psi_cpu_load_details();
    }
#endif //CPU_LOAD_DETAILS

}

/***************************************************************
 * kpi_after_codec
 * -------------------------------------------------------------
 * Function to be called at the end of processing.
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void kpi_after_codec(void)
{
    unsigned long    processing_time;

    if( kpi_status & KPI_IVA_LOAD ) {
        /* Read 32k timer */
        iva_kpi.after_time = get_32k();
        iva_kpi.t32k_end   = iva_kpi.after_time;

        /* Process IVA-HD working time */
        processing_time = iva_kpi.after_time - iva_kpi.before_time;

#ifdef  IVA_DETAILS
        if( kpi_status & KPI_IVA_TRACE ) {
            /* transform 32KHz ticks into ms */
            PSI_TracePrintf(TRACEGRP, "END %-7s %-4u %-8lu %d\n",
                            "IVA", iva_kpi.nb_frames + 1, iva_kpi.after_time, (processing_time * 100000) / 32768);
        }
#endif /*IVA_DETAILS*/

        /* Total for Average */
        iva_kpi.ivahd_t_tot = (iva_kpi.ivahd_t_tot + processing_time);

        /* Max */
        if( processing_time > iva_kpi.ivahd_t_max ) {
            iva_kpi.ivahd_t_max       = processing_time;
            iva_kpi.ivahd_t_max_frame = iva_kpi.nb_frames + 1;
        }

        /* Min */
        if( processing_time < iva_kpi.ivahd_t_min ) {
            iva_kpi.ivahd_t_min       = processing_time;
            iva_kpi.ivahd_t_min_frame = iva_kpi.nb_frames + 1;
        }

        iva_kpi.nb_frames++;

        /* Processing time x MHz */
        iva_kpi.ivahd_MHzTime += (iva_kpi.ivahd_MHz * processing_time);
    }

}

/***************************************************************
 * kpi_IVA_new_freq
 * -------------------------------------------------------------
 * Function to be called when changing IVA frequency
 *
 * @params: unsigned long freq
 *
 * @return: none
 *
 ***************************************************************/
void kpi_IVA_new_freq( unsigned long freq )
{

     iva_kpi.ivahd_MHz = freq;
     if( kpi_status & KPI_IVA_TRACE ) {
            PSI_TracePrintf(TRACEGRP, "MHz %-7s %-4u \n", "IVA", freq);
     }
}

/***************************************************************
 * kpi_IVA_profiler_print
 * -------------------------------------------------------------
 * Function to be called after codec execution.
 *
 * It is printing all results. syslink_trace_daemon must be enabled
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void kpi_IVA_profiler_print(void)
{
    unsigned long    total_time, fps_x100, fps, frtick_x10, Ivatick_x10, Iva_pct, Iva_mhz;
    total_time = fps_x100 = fps = frtick_x10 = Ivatick_x10 = Iva_pct = Iva_mhz = 0;
    unsigned long    iva_frtick_x10, iva_fps_x100, iva_fps;

    /* Calculate the total time */
    total_time = iva_kpi.t32k_end - iva_kpi.t32k_start;

    if( total_time ) {

        /* Calculate the frame period and the framerate */
        if( iva_kpi.nb_frames ) {
            frtick_x10 = total_time * 10 / iva_kpi.nb_frames;
            fps_x100 = 32768 * 1000 / frtick_x10;
            fps = fps_x100 / 100;

            iva_frtick_x10 = iva_kpi.ivahd_t_tot * 10 / iva_kpi.nb_frames;
            iva_fps_x100 = 32768 * 1000 / iva_frtick_x10;
            iva_fps = iva_fps_x100 / 100;
        }

        /* Calculate the IVA load */
        if( iva_kpi.nb_frames ) {
            Ivatick_x10 = (iva_kpi.ivahd_t_tot * 10 / iva_kpi.nb_frames);
            Iva_pct = Ivatick_x10 * 100 / frtick_x10;
        }

        /* Cakculate the IVA MHz used */
        Iva_mhz = iva_kpi.ivahd_MHzTime / total_time;

        PSI_TracePrintf(TRACEGRP, "\n");
        PSI_TracePrintf(TRACEGRP, "----------------------------------\n");
        PSI_TracePrintf(TRACEGRP, "M4 stats:\n");
        PSI_TracePrintf(TRACEGRP, "-------------\n");
        PSI_TracePrintf(TRACEGRP, "                 IVA  1st beg: %lu\n", iva_kpi.t32k_start);
        PSI_TracePrintf(TRACEGRP, "                 IVA last end: %lu\n", iva_kpi.t32k_end);
        PSI_TracePrintf(TRACEGRP, "            Total time at MPU: %lu\n", iva_kpi.t32k_mpu_time);
        PSI_TracePrintf(TRACEGRP, "        Total time at IVA (A): %lu\n", iva_kpi.ivahd_t_tot);
        PSI_TracePrintf(TRACEGRP, "     Total (IVA+MPU) time (B): %lu\n", total_time);
        PSI_TracePrintf(TRACEGRP, "        Number of samples (C): %lu\n", iva_kpi.nb_frames);
        PSI_TracePrintf(TRACEGRP, "  IVA period per sample (A/C): %lu\n", iva_frtick_x10 / 10);
        PSI_TracePrintf(TRACEGRP, "Total period per sample (B/C): %lu\n", frtick_x10 / 10);
        PSI_TracePrintf(TRACEGRP, "  fps based on total IVA time: %2d.%02d\n", iva_fps, iva_fps_x100 - (iva_fps * 100));
        PSI_TracePrintf(TRACEGRP, "      fps based on total time: %2d.%02d\n", fps, fps_x100 - (fps * 100));

        /* stat existing only if frames were processed */
        if( iva_kpi.nb_frames ) {
            PSI_TracePrintf(TRACEGRP, "\n");
            PSI_TracePrintf(TRACEGRP, "----------------------------------\n");
            PSI_TracePrintf(TRACEGRP, "IVA-HD processing time:\n");
            PSI_TracePrintf(TRACEGRP, "-----------------------\n");
            PSI_TracePrintf(TRACEGRP, "  IVA average: %d\n", iva_kpi.ivahd_t_tot / iva_kpi.nb_frames);
            PSI_TracePrintf(TRACEGRP, "      IVA max: %d frame: %d\n", iva_kpi.ivahd_t_max, iva_kpi.ivahd_t_max_frame);
            PSI_TracePrintf(TRACEGRP, "      IVA min: %d frame: %d\n", iva_kpi.ivahd_t_min, iva_kpi.ivahd_t_min_frame);
            PSI_TracePrintf(TRACEGRP, "      IVA use: %d %%\n", Iva_pct);
            if (Iva_mhz) {
                PSI_TracePrintf(TRACEGRP, "      IVA MHz: %d MHz\n\n", Iva_mhz);
            }
        }
    }

}

/***************************************************************
 * psi_component
 * -------------------------------------------------------------
 * Structure maintenaing data for Ducati Component traces
 *
 ***************************************************************/
typedef struct {
    void *         hComponent;
    char           name[50];
} psi_component;

/* we trace up to MAX_COMPONENTS components */
#define MAX_COMPONENTS 8

/***************************************************************
 * kpi_comp_monitor
 * -------------------------------------------------------------
 * Contains up to 8 components data
 *
 ***************************************************************/
psi_component    kpi_comp_monitor[MAX_COMPONENTS]; /* we trace up to MAX_COMPONENTS components */
Uint32              kpi_comp_monitor_cnt=0;      /* no component yet */


/***************************************************************
 * kpi_comp_init
 * -------------------------------------------------------------
 * setup monitor data
 *
 * @params: void* hComponent : Object of the component
 *
 * @return: none
 *
 ***************************************************************/
void kpi_comp_init(void *hComponent)
{

    Uint32            comp_cnt;

#ifdef  COMP_ENABLE_DUCATI_LOAD
    /* Started 1st time */
    if( !(kpi_status & KPI_INST_RUN)) {
        kpi_instInit();
    }
#endif //COMP_ENABLE_DUCATI_LOAD

    if( kpi_status & KPI_COMP_TRACE ) {

        /* First init: clear kpi_comp_monitor components */
        if( kpi_comp_monitor_cnt == 0 ) {
            for( comp_cnt=0; comp_cnt < MAX_COMPONENTS; comp_cnt++ ) {
                /*clear handler registery */
                kpi_comp_monitor[comp_cnt].hComponent = 0;
            }
        }

        /* identify the 1st component free in the table (maybe one was removed previously) */
        for( comp_cnt=0; comp_cnt < MAX_COMPONENTS; comp_cnt++ ) {
            if( kpi_comp_monitor[comp_cnt].hComponent == 0 ) {
                break;
            }
        }

        if( comp_cnt >= MAX_COMPONENTS ) {
            return;
        }

        /* current comp num and update */
        kpi_comp_monitor_cnt++;

        /* register the component handle */
        kpi_comp_monitor[comp_cnt].hComponent = hComponent;


        //TBD : Need to figure out how to get component name here. Setting default to rpmsg-dce
        strcpy(kpi_comp_monitor[comp_cnt].name, "rpmsg-dce");
        System_sprintf(kpi_comp_monitor[comp_cnt].name, "%s%d", kpi_comp_monitor[comp_cnt].name, comp_cnt); // Add index to the name
        /* trace component init */
        PSI_TracePrintf(TRACEGRP, "<KPI> DCE %-8s Init %-8lu \n", kpi_comp_monitor[comp_cnt].name, get_32k());

    }

}

/***************************************************************
 * kpi_comp_deinit
 * -------------------------------------------------------------
 * deinit monitor data
 *
 * @params: void * hComponent
 *
 * @return: none
 *
 ***************************************************************/
void kpi_comp_deinit(void* hComponent)
{

    Uint32    comp_cnt;

    if( kpi_comp_monitor_cnt > 0 ) {
        /* identify the component from the registery */
        for( comp_cnt=0; comp_cnt < MAX_COMPONENTS; comp_cnt++ ) {
            if( kpi_comp_monitor[comp_cnt].hComponent == hComponent ) {
                break;
            }
        }

        /* trace component deinit */
        if( comp_cnt < MAX_COMPONENTS ) {
            PSI_TracePrintf(TRACEGRP, "<KPI> DCE %-7s Deinit %-8lu\n", kpi_comp_monitor[comp_cnt].name, get_32k());

            /* unregister the component */
            kpi_comp_monitor[comp_cnt].hComponent = 0;
        }

        kpi_comp_monitor_cnt--;
    }

    /* stop the instrumentation */
    if( kpi_comp_monitor_cnt == 0 ) {
        kpi_instDeinit();
    }

}

/***************************************************************
 * kpi_load_pct_x_100
 * -------------------------------------------------------------
 * calculate percentage with 2 digit such as 78.34 %
 *
 * @params: unsigned long load, total
 *
 * @return: unsigned long result
 *
 ***************************************************************/
unsigned long kpi_load_pct_x_100(unsigned long load, unsigned long total)
{
    unsigned long    mult = 100 * 100;

    while( total > (32768 * 8)) {
        total /= 2;
        mult /= 2;
    }

    /* load = 100 * 100 * load / total */
    load = ((load * mult) + total / 2) / total;
    return (load);
}

/***************************************************************
 * kpi_CPU_profiler_init
 * -------------------------------------------------------------
 * Initialize profiler data
 *
 * @params: none
 *
 * @return: none
 *
 ***************************************************************/
void kpi_CPU_profiler_init(void)
{
    unsigned int    CoreId, i;

    PSI_TracePrintf(TRACEGRP, "<KPI> CPU Profiler Init %-8lu\n", get_32k());

    for( CoreId = 0; CoreId < 2; CoreId++ ) {
        psi_bios_kpi   *core_kpi = &bios_kpi[CoreId];

        core_kpi->total_time       = 0;
        core_kpi->kpi_time         = 0;
        core_kpi->kpi_count        = 0;
        core_kpi->nb_switch        = 0;

        core_kpi->nb_tasks         = 0;
        core_kpi->nb_hwi           = 0;
        core_kpi->nb_swi           = 0;

        core_kpi->tasks[0].handle  = 0;                         /* for startup set task[0].handle to 0 */

        /* reserve last task slot in DB to others task that wouldn't fit in the DB */
        core_kpi->tasks[KPI_MAX_NB_TASKS - 1].handle = (void *) 0x11000011;
        core_kpi->tasks[KPI_MAX_NB_TASKS - 1].total_time = 0;
        core_kpi->tasks[KPI_MAX_NB_TASKS - 1].nb_switch  = 0;
        strcpy(core_kpi->tasks[KPI_MAX_NB_TASKS - 1].name, "Other tasks");
        /* reserve last swi slot in DB to others task that wouldn't fit in the DB */
        core_kpi->swi[KPI_MAX_NB_SWI - 1].handle = (void *) 0x22000022;
        core_kpi->swi[KPI_MAX_NB_SWI - 1].total_time = 0;
        core_kpi->swi[KPI_MAX_NB_SWI - 1].nb_switch  = 0;
        strcpy(core_kpi->swi[KPI_MAX_NB_SWI - 1].name, "Other swis");
        /* reserve last hwi slot in DB to others task that wouldn't fit in the DB */
        core_kpi->hwi[KPI_MAX_NB_HWI - 1].handle = (void *) 0x33000033;
        core_kpi->hwi[KPI_MAX_NB_HWI - 1].total_time = 0;
        core_kpi->hwi[KPI_MAX_NB_HWI - 1].nb_switch  = 0;
        strcpy(core_kpi->hwi[KPI_MAX_NB_HWI - 1].name, "Other hwis");

        /* clear the context pointers table */
        for( i=0; i < CTX_DIRECT_NUM; i++ ) {
            core_kpi->context_cache[i]= &(core_kpi->tasks[0]);  /* point to the Data Base */
        }

        core_kpi->context_stack[0] = &(core_kpi->tasks[0]);     /* point to 1st context element */
        core_kpi->pt_context_stack = &(core_kpi->context_stack[0]); /* stack beginning */

        core_kpi->ptr_idle_total   = &(core_kpi->idle_prev);    /* will point later on to idle_total_ticks */
        core_kpi->idle_prev        = 0;
        core_kpi->idle_time_prev   = 0;

        core_kpi->trace_time       = 0;

    }

    /* set current time into prev_t32 in each data based */
    bios_kpi[0].prev_t32 = get_time();
    bios_kpi[1].prev_t32 = bios_kpi[0].prev_t32;

}

/***************************************************************
 * kpi_CPU_profiler_print
 * -------------------------------------------------------------
 * Print profiler data
 *
 * @params: none
 *
 * @return: none
 *    if (prev != NULL)

 ***************************************************************/
void kpi_CPU_profiler_print(void)
{
    unsigned long    load, ld00, Idle, CoreId, i;
    psi_bios_kpi    *core_kpi;

#ifdef  COST_AFTER
    unsigned long    instx1024;
    /* calculate instrumentation cost a posteriori */
    {
        Task_Handle      test;
        unsigned long    key = Task_disable();
        core_kpi = &bios_kpi[ Core_getId() ];
        test = core_kpi->tasks[0].handle;
        instx1024 = get_time();

        for( i = 0; i < 1024; i++ ) {
            psi_kpi_task_test(test, test);
        }

        instx1024 = get_time() - instx1024;
        Task_restore(key);
    }
#endif //COST_AFTER

    /* Print the results for each core */
    for( CoreId = 0; CoreId < 2; CoreId++ ) {
        core_kpi = &bios_kpi[CoreId];

        /* Reconstruct global counts */
        for( i=0; i < core_kpi->nb_tasks; i++ ) {
            core_kpi->nb_switch += core_kpi->tasks[i].nb_switch; /* nb_switch  (nb interrupts + task switchs) */
            core_kpi->kpi_count += core_kpi->tasks[i].nb_switch; /* kpi_count (nunber of times the intrumentation run) */
            core_kpi->total_time+= core_kpi->tasks[i].total_time; /* total_time (all times measured) */
        }

        for( i=0; i < core_kpi->nb_swi; i++ ) {
            core_kpi->nb_switch += core_kpi->swi[i].nb_switch;
            core_kpi->kpi_count += core_kpi->swi[i].nb_switch * 2; /* 2 runs per interrupts */
            core_kpi->total_time+= core_kpi->swi[i].total_time;
        }

        for( i=0; i < core_kpi->nb_hwi; i++ ) {
            core_kpi->nb_switch += core_kpi->hwi[i].nb_switch;
            core_kpi->kpi_count += core_kpi->hwi[i].nb_switch * 2; /* 2 runs per interrupts */
            core_kpi->total_time+= core_kpi->hwi[i].total_time;
        }

        /* add cost of measured if stored as a separate task */
        core_kpi->total_time += core_kpi->kpi_time;

        if( core_kpi->total_time ) {
            /* Print global stats */
            PSI_TracePrintf(TRACEGRP, "----------------------------------\n");
            PSI_TracePrintf(TRACEGRP, "Core %d :\n", CoreId);
            PSI_TracePrintf(TRACEGRP, "--------\n");
            PSI_TracePrintf(TRACEGRP, "  type  #: handle   ticks    counts  ( %%cpu )   instance-name\n");
            PSI_TracePrintf(TRACEGRP, "----------------------------------\n");
            PSI_TracePrintf(TRACEGRP, "  Total test        %-8lu %-7ld\n", core_kpi->total_time, core_kpi->nb_switch);
            PSI_TracePrintf(TRACEGRP, "----------------------------------\n");

            if( core_kpi->kpi_time ) {
                ld00 = kpi_load_pct_x_100(core_kpi->kpi_time, core_kpi->total_time); /* ex. 7833 -> 78.33 % */
                load = ld00 / 100;                                        /* 78.33 % -> 78 */
                ld00 = ld00 - load * 100;                                 /* 78.33 % -> 33 */
                PSI_TracePrintf(TRACEGRP, "  Measure: Overhead %-8lu ----    (%2d.%02d %%)  CPU load-inst-cost\n",
                                core_kpi->kpi_time, load, ld00);
            }

            for( i=0; i < core_kpi->nb_tasks; i++ ) {
                ld00 = kpi_load_pct_x_100(core_kpi->tasks[i].total_time, core_kpi->total_time);
                load = ld00 / 100;                                      /* 78.33 % -> 78 */
                ld00 = ld00 - load * 100;                               /* 78.33 % -> 33 */
                PSI_TracePrintf(TRACEGRP, "  task %2d: %-8lx %-8lu %-7ld (%2d.%02d %%)  %s\n",
                                i, core_kpi->tasks[i].handle, core_kpi->tasks[i].total_time, core_kpi->tasks[i].nb_switch, load, ld00, core_kpi->tasks[i].name);
            }

            for( i=0; i < core_kpi->nb_swi; i++ ) {
                ld00 = kpi_load_pct_x_100(core_kpi->swi[i].total_time, core_kpi->total_time);
                load = ld00 / 100;                                      /* 78.33 % -> 78 */
                ld00 = ld00 - load * 100;                               /* 78.33 % -> 33 */
                PSI_TracePrintf(TRACEGRP, "   swi %2d: %-8lx %-8lu %-7ld (%2d.%02d %%)  %s\n",
                                i, core_kpi->swi[i].handle, core_kpi->swi[i].total_time, core_kpi->swi[i].nb_switch, load, ld00, core_kpi->swi[i].name);
            }

            for( i=0; i < core_kpi->nb_hwi; i++ ) {
                ld00 = kpi_load_pct_x_100(core_kpi->hwi[i].total_time, core_kpi->total_time);
                load = ld00 / 100;                                      /* 78.33 % -> 78 */
                ld00 = ld00 - load * 100;                               /* 78.33 % -> 33 */
                PSI_TracePrintf(TRACEGRP, "   hwi %2d: %-8lx %-8lu %-7ld (%2d.%02d %%)  %s\n",
                                i, core_kpi->hwi[i].handle, core_kpi->hwi[i].total_time, core_kpi->hwi[i].nb_switch, load, ld00, core_kpi->hwi[i].name);
            }

            PSI_TracePrintf(TRACEGRP, "----------------------------------\n");

#ifdef  COST_AFTER
            if( !core_kpi->kpi_time ) {
                /* calculate the cost in the instrumentation */
                core_kpi->kpi_time = (core_kpi->kpi_count * instx1024) / 1024;
                ld00 = kpi_load_pct_x_100(core_kpi->kpi_time, core_kpi->total_time); /* ex. 7833 -> 78.33 % */
                load = ld00 / 100;                                        /* 78.33 % -> 78 */
                ld00 = ld00 - load * 100;                                 /* 78.33 % -> 33 */
                PSI_TracePrintf(TRACEGRP, "  CPU load measure overhead  %2d.%02d %%\n", load, ld00);
            }
#endif //COST_AFTER

            if( core_kpi->trace_time ) {
                ld00 = kpi_load_pct_x_100(core_kpi->trace_time, core_kpi->total_time);
                load = ld00 / 100;                                      /* 78.33 % -> 78 */
                ld00 = ld00 - load * 100;                               /* 78.33 % -> 33 */
                PSI_TracePrintf(TRACEGRP, "  Real time traces overhead  %2d.%02d %%\n", load, ld00);
            }

            /* restore to Idle Inst cost + print cost if idle exists */
            if( *(core_kpi->ptr_idle_total)) {
                Idle = *(core_kpi->ptr_idle_total) + core_kpi->kpi_time + core_kpi->trace_time;
                ld00 = kpi_load_pct_x_100(Idle, core_kpi->total_time);
                load = ld00 / 100;                                      /* 78.33 % -> 78 */
                ld00 = ld00 - load * 100;                               /* 78.33 % -> 33 */
                PSI_TracePrintf(TRACEGRP, "----------------------------------\n");
                PSI_TracePrintf(TRACEGRP, "  Idle task compensated  =>  %2d.%02d %% \n", load, ld00);
                PSI_TracePrintf(TRACEGRP, "----------------------------------\n\n");
            }

        }
    }
}

/***************************************************************
 * psi_kpi_search_create_context_task
 * -------------------------------------------------------------
 * Search for a context entry to context DB.
 * Create a new on if not existing
 *
 * @params: void *task, unsigned int CoreId
 *
 * @return: psi_context_info *db_ptr
 *
 ***************************************************************/
psi_context_info *psi_kpi_search_create_context_task(void *task, psi_bios_kpi *core_kpi)
{
    int                 i, nb_items, strLenght;
    String              taskName;
    psi_context_info   *db_ptr;

    /* KPI_CONTEXT_TASK */
    nb_items = core_kpi->nb_tasks;
    db_ptr   = core_kpi->tasks;

    /* Search handle existing in task table */
    for( i=0; i < nb_items; i++ ) {
        if( task == db_ptr[i].handle ) {
            break;
        }
    }

    /* handle found in our DB */
    if( i < nb_items ) {
        return (&db_ptr[i]);
    }

    /* check for space left in the DB. When last-1 reached use last for others tasks together */
    if( nb_items >= KPI_MAX_NB_TASKS - 1 ) {
        core_kpi->nb_tasks = KPI_MAX_NB_TASKS;
        return (&db_ptr[KPI_MAX_NB_TASKS - 1]);
    }

    /* add the new task */
    core_kpi->nb_tasks = nb_items + 1;

    /* get the task name Make sure task name fits into the item.name */
    taskName = Task_Handle_name((Task_Handle)task);
    strLenght = strlen(taskName);
    if( strLenght > 40 ) {
        strLenght = 40;
    }

    /* For Idle direct access */
    if( strstr(taskName, "Idle")) {
        core_kpi->ptr_idle_total = &(db_ptr[i].total_time);
    }

    /* create the new entry */
    db_ptr[i].handle     = task;
    db_ptr[i].total_time = 0;
    db_ptr[i].nb_switch  = 0;
    strncpy(db_ptr[i].name, taskName, strLenght);
    *(db_ptr[i].name + strLenght) = '\0';                // complete the chain of char

    return (&(db_ptr[i]));
}

/***************************************************************
 * psi_kpi_search_create_context_swi
 * -------------------------------------------------------------
 * Search for a context entry to context DB.
 * Create a new on if not existing
 *
 * @params: void *task, unsigned int CoreId
 *
 * @return: psi_context_info *db_ptr
 *
 ***************************************************************/
psi_context_info *psi_kpi_search_create_context_swi(void *task, psi_bios_kpi *core_kpi)
{
    int                 i, nb_items, strLenght;
    String              taskName;
    psi_context_info   *db_ptr;

    /* KPI_CONTEXT_SWI */
    nb_items = core_kpi->nb_swi;
    db_ptr   = core_kpi->swi;

    /* Search handle existing in task table */
    for( i=0; i < nb_items; i++ ) {
        if( task == db_ptr[i].handle ) {
            break;
        }
    }

    /* handle found in our DB */
    if( i < nb_items ) {
        return (&db_ptr[i]);
    }

    /* check for space left in the DB. When last-1 reached use last for others swi together */
    if( nb_items >= KPI_MAX_NB_SWI - 1 ) {
        core_kpi->nb_swi = KPI_MAX_NB_SWI;
        return (&db_ptr[KPI_MAX_NB_SWI - 1]);
    }

    /* add the new swi */
    core_kpi->nb_swi = nb_items + 1;

    /* get the task name Make sure task name fits into the item.name */
    taskName = Swi_Handle_name((Swi_Handle)task);
    strLenght = strlen(taskName);
    if( strLenght > 40 ) {
        strLenght = 40;
    }

    /* create the new entry */
    db_ptr[i].handle     = task;
    db_ptr[i].total_time = 0;
    db_ptr[i].nb_switch  = 0;
    strncpy(db_ptr[i].name, taskName, strLenght);
    *(db_ptr[i].name + strLenght) = '\0';                // complete the chain of char

    return (&(db_ptr[i]));
}

/***************************************************************
 * psi_kpi_search_create_context_hwi
 * -------------------------------------------------------------
 * Search for a context entry to context DB.
 * Create a new on if not existing
 *
 * @params: void *task, unsigned int CoreId
 *
 * @return: psi_context_info *db_ptr
 *
 ***************************************************************/
psi_context_info *psi_kpi_search_create_context_hwi(void *task, psi_bios_kpi *core_kpi)
{
    int                 i, nb_items, strLenght;
    String              taskName;
    psi_context_info   *db_ptr;

    /* KPI_CONTEXT_HWI */
    nb_items = core_kpi->nb_hwi;
    db_ptr   = core_kpi->hwi;

    /* Search handle existing in task table */
    for( i=0; i < nb_items; i++ ) {
        if( task == db_ptr[i].handle ) {
            break;
        }
    }

    /* handle found in our DB */
    if( i < nb_items ) {
        return (&db_ptr[i]);
    }

    /* check for space left in the DB. When last-1 reached use last for others hwi together */
    if( nb_items >= KPI_MAX_NB_HWI - 1 ) {
        core_kpi->nb_hwi = KPI_MAX_NB_HWI;
        return (&db_ptr[KPI_MAX_NB_HWI - 1]);
    }

    /* add the new hwi */
    core_kpi->nb_hwi = nb_items + 1;

    /* get the task name Make sure task name fits into the item.name */
    taskName = Hwi_Handle_name((Hwi_Handle)task);
    strLenght = strlen(taskName);
    if( strLenght > 40 ) {
        strLenght = 40;
    }

    /* create the new entry */
    db_ptr[i].handle     = task;
    db_ptr[i].total_time = 0;
    db_ptr[i].nb_switch  = 0;
    strncpy(db_ptr[i].name, taskName, strLenght);
    *(db_ptr[i].name + strLenght) = '\0';                // complete the chain of char

    return (&(db_ptr[i]));
}

/***************************************************************
 * psi_kpi_task_switch
 * -------------------------------------------------------------
 * Task switch hook:
 *  - identify new tasks
 *  - accumulate task load
 *  - process total execution time
 *
 * @params: Task_Handle prev, Task_Handle next
 *
 * @return: none
 *
 ***************************************************************/
void psi_kpi_task_switch(Task_Handle prev, Task_Handle next)
{
    if( kpi_status & KPI_CPU_LOAD ) {
        unsigned long       key    = Core_hwiDisable();
        unsigned long       CoreId = Core_getId();
        unsigned long       tick   = get_time_core(CoreId);
        psi_bios_kpi       *core_kpi = &bios_kpi[CoreId];
        psi_context_info   *context_next, *context;
        unsigned long       cachePtr;

        /* Ending context */
        context = (psi_context_info *) *core_kpi->pt_context_stack;

        /* Starting context */
        cachePtr= ((((long) next) >> 4) ^ (((long) next) >> 12)) & 0xff; /* build a simple 8 bits address for trying cache DB search */
        context_next = core_kpi->context_cache[ cachePtr ];
        if( context_next->handle != next ) {                      /* if NOT pointing to our handle make exhaustive seartch */
            context_next = psi_kpi_search_create_context_task(next, core_kpi);
            core_kpi->context_cache[ cachePtr ] = context_next;   /* update direct table (this may evict one already here) */
        }

        /* Maintain stack for interrupts: save context starting */
        *core_kpi->pt_context_stack = (void *) context_next;

        /* update tasks stats */
        context->total_time  += tick - core_kpi->prev_t32;        /* count the time spend in the ending task */
        context->nb_switch++;                                     /* count the switch */

#ifdef  INST_COST
        /* will start processing the task now */
        {
            unsigned long    tick2 = get_time_core(CoreId);
            core_kpi->kpi_time += tick2 - tick;                   /* update kpi_time (instrumentation cost) */
            core_kpi->prev_t32  = tick2;                          /* store tick: time when task actually starts */
        }
#else //INST_COST
        core_kpi->prev_t32    = tick;                             /* store tick: time when task actually starts */
#endif //INST_COST

        Core_hwiRestore(key);
    }
}

/***************************************************************
 * psi_kpi_swi_begin
 * -------------------------------------------------------------
 * SWI begin hook: memorizes SWI start time
 *
 * @params: Swi_Handle swi
 *
 * @return: none
 *
 ***************************************************************/
void psi_kpi_swi_begin(Swi_Handle swi)
{
    if( kpi_status & KPI_CPU_LOAD ) {
        unsigned long       key    = Core_hwiDisable();
        unsigned long       CoreId = Core_getId();
        unsigned long       tick   = get_time_core(CoreId);
        psi_bios_kpi       *core_kpi = &bios_kpi[CoreId];
        psi_context_info   *context_next, *context;
        unsigned long       cachePtr;

        /* Ending context */
        context = (psi_context_info *) *core_kpi->pt_context_stack++; /* Going from a TASK or SWI to new SWI */

        /* Starting context */
        cachePtr= ((((long) swi) >> 4) ^ (((long) swi) >> 12)) & 0xff; /* build a simple 8 bits address for trying cache DB search */
        context_next = core_kpi->context_cache[ cachePtr ];
        if( context_next->handle != swi ) {                       /* if NOT pointing to our handle make exhaustive seartch */
            context_next = psi_kpi_search_create_context_swi(swi, core_kpi);
            core_kpi->context_cache[ cachePtr ] = context_next;   /* update direct table (this may evict one already here) */
        }

        /* Maintain stack for interrupts: save context starting */
        *core_kpi->pt_context_stack = (void *) context_next;

        /* update tasks stats */
        context->total_time  += tick - core_kpi->prev_t32;        /* count the time spend in the ending task */

#ifdef  INST_COST
        /* will start processing the task now */
        {
            unsigned long    tick2 = get_time_core(CoreId);
            core_kpi->kpi_time += tick2 - tick;                   /* update kpi_time (instrumentation cost) */
            core_kpi->prev_t32  = tick2;                          /* store tick: time when task actually starts */
        }
#else //INST_COST
        core_kpi->prev_t32    = tick;                             /* store tick: time when task actually starts */
#endif //INST_COST

        Core_hwiRestore(key);
    }
}

/***************************************************************
 * psi_kpi_swi_end
 * -------------------------------------------------------------
 * SWI end hook: accumulates SWI execution time
 *
 * @params: Swi_Handle swi
 *
 * @return: none
 *
 ***************************************************************/
void psi_kpi_swi_end(Swi_Handle swi)
{
    if( kpi_status & KPI_CPU_LOAD ) {
        unsigned long       key    = Core_hwiDisable();
        unsigned long       CoreId = Core_getId();
        unsigned long       tick   = get_time_core(CoreId);
        psi_bios_kpi       *core_kpi = &bios_kpi[CoreId];
        psi_context_info   *context;

        /* Ending context */
        context = (psi_context_info *) *core_kpi->pt_context_stack--; /* Going back to interrupted TASK or SWI */

        /* update tasks stats */
        context->total_time  += tick - core_kpi->prev_t32;        /* count the time spend in the ending task */
        context->nb_switch++;                                     /* count the switch */

#ifdef  INST_COST
        /* will start processing the task now */
        {
            unsigned long    tick2 = get_time_core(CoreId);
            core_kpi->kpi_time += tick2 - tick;                   /* update kpi_time (instrumentation cost) */
            core_kpi->prev_t32  = tick2;                          /* store tick: time when task actually starts */
        }
#else //INST_COST
        core_kpi->prev_t32    = tick;                             /* store tick: time when task actually starts */
#endif //INST_COST

        Core_hwiRestore(key);
    }
}

/***************************************************************
 * psi_kpi_hwi_begin
 * -------------------------------------------------------------
 * HWI begin hook: memorizes HWI start time
 *
 * @params: Hwi_Handle hwi
 *
 * @return: none
 *
 ***************************************************************/
void psi_kpi_hwi_begin(Hwi_Handle hwi)
{
    if( kpi_status & KPI_CPU_LOAD ) {
        unsigned long       key    = Core_hwiDisable();
        unsigned long       CoreId = Core_getId();
        unsigned long       tick   = get_time_core(CoreId);
        psi_bios_kpi       *core_kpi = &bios_kpi[CoreId];
        psi_context_info   *context_next, *context;
        unsigned long       cachePtr;

        /* Ending context */
        context = (psi_context_info *) *core_kpi->pt_context_stack++; /* Going from previous TASK or SWI to a HWI */

        /* Starting context */
        cachePtr= ((((long) hwi) >> 4) ^ (((long) hwi) >> 12)) & 0xff; /* build a simple 8 bits address for trying cache DB search */
        context_next = core_kpi->context_cache[ cachePtr ];
        if( context_next->handle != hwi ) {                       /* if NOT pointing to our handle make exhaustive seartch */
            context_next = psi_kpi_search_create_context_hwi(hwi, core_kpi);
            core_kpi->context_cache[ cachePtr ] = context_next;   /* update direct table (this may evict one already here) */
        }

        /* Maintain stack for interrupts: save context starting */
        *core_kpi->pt_context_stack = (void *) context_next;

        /* update tasks stats */
        context->total_time  += tick - core_kpi->prev_t32;        /* count the time spend in the ending task */

#ifdef  INST_COST
        /* will start processing the task now */
        {
            unsigned long    tick2 = get_time_core(CoreId);
            core_kpi->kpi_time += tick2 - tick;                   /* update kpi_time (instrumentation cost) */
            core_kpi->prev_t32  = tick2;                          /* store tick: time when task actually starts */
        }
#else //INST_COST
        core_kpi->prev_t32    = tick;                             /* store tick: time when task actually starts */
#endif //INST_COST

        Core_hwiRestore(key);
    }
}

/***************************************************************
 * psi_kpi_hwi_end
 * -------------------------------------------------------------
 * HWI end hook: accumulates HWI execution time
 *
 * @params: Hwi_Handle hwi
 *
 * @return: none
 *
 ***************************************************************/
void psi_kpi_hwi_end(Hwi_Handle hwi)
{
    if( kpi_status & KPI_CPU_LOAD ) {
        unsigned long       key    = Core_hwiDisable();
        unsigned long       CoreId = Core_getId();
        unsigned long       tick   = get_time_core(CoreId);
        psi_bios_kpi       *core_kpi = &bios_kpi[CoreId];
        psi_context_info   *context;

        /* Ending context */
        context = (psi_context_info *) *core_kpi->pt_context_stack--; /* Going back to interrupted TASK or SWI or HWI */

        /* update tasks stats */
        context->total_time  += tick - core_kpi->prev_t32;        /* count the time spend in the ending task */
        context->nb_switch++;                                     /* count the switch */

#ifdef  INST_COST
        /* will start processing the task now */
        {
            unsigned long    tick2 = get_time_core(CoreId);
            core_kpi->kpi_time += tick2 - tick;                   /* update kpi_time (instrumentation cost) */
            core_kpi->prev_t32  = tick2;                          /* store tick: time when task actually starts */
        }
#else //INST_COST
        core_kpi->prev_t32    = tick;                             /* store tick: time when task actually starts */
#endif //INST_COST

        Core_hwiRestore(key);
    }
}

/***************************************************************
 * psi_kpi_task_test
 * -------------------------------------------------------------
 * Task switch hook:
 *  - identify new tasks
 *  - used for measuring execution time of the instrumentation
 *
 * @params: Task_Handle prev, Task_Handle next
 *
 * @return: none
 *
 ***************************************************************/
void psi_kpi_task_test(Task_Handle prev, Task_Handle next)
{
    unsigned long       key    = Core_hwiDisable();
    unsigned long       CoreId = Core_getId();
    unsigned long       tick   = get_time_core(CoreId);
    psi_bios_kpi       *core_kpi = &bios_kpi[CoreId];
    psi_context_info   *context_next;
    unsigned long       cachePtr;

    /* Starting context */
    cachePtr= ((((long) next) >> 4) ^ (((long) next) >> 12)) & 0xff; /* build a simple 8 bits address for trying cache DB search */
    context_next = core_kpi->context_cache[ cachePtr ];
    if( context_next->handle != next ) {                           /* if NOT pointing to our handle make exhaustive seartch */
        context_next = psi_kpi_search_create_context_task(next, core_kpi);
        core_kpi->context_cache[ cachePtr ] = context_next;       /* update direct table (this may evict one already here) */
    }

    /* Maintain stack for interrupts: save context starting */
    *core_kpi->pt_context_stack = (void *) context_next;

    core_kpi->prev_t32    = tick;                                 /* store tick: time when task actually starts */

    Core_hwiRestore(key);
}
