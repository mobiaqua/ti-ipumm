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
*   @file  profile.h
*   This file contains the function prototypes of the instrumentation
*   and the flags for defining the functionality
*
*   @path \WTSD_DucatiMMSW\platform\utils\
*
*  @rev 1.0
*/
#ifndef _PROFILE_H
#define _PROFILE_H

#ifdef BUILD_PSI_KPI

#define PSI_KPI                 /* add the instrumentation (cpu load and allowing tracing) */

#endif /*BUILD_PSI_KPI*/



#define IVA_DETAILS             /* for tracing IVA events */
#define COMP_DETAILS             /* for tracing COMP events */
#define PHYSICAL_BUFFER         /* print physical buffer address for COMP events */
#define CPU_LOAD_DETAILS        /* for tracing CPU IDLE % each frame */
//#define CINIT_ENABLE_DUCATI_LOAD  /* measure starts when codec is created */
#define COMP_ENABLE_DUCATI_LOAD  /* measure starts when the first COMP component is created */

//#define USE_CTM_TIMER           /* CTM vs 32K for measuring CPU load */
#define INST_COST               /* measure the instrumentation cost during the test */
//#define COST_AFTER            /* allows estimating the instrumentation cost after completion */

/* Instrumentation Flags control */
typedef enum {
    KPI_END_SUMMARY = (1 << 0),  /* print IVA and Ducati/Benelli summary at end of use case */
    KPI_IVA_DETAILS = (1 << 1),  /* print IVA trace during the use case */
    KPI_COMP_DETAILS = (1 << 2),  /* print COMP trace during the use case */
    KPI_CPU_DETAILS = (1 << 3)   /* print Idle trace during the use case */
} KPI_inst_type;

/* Instrumentation internal Status */
typedef enum {
    KPI_INST_RUN  = (1 << 0),    /* instrumentation running */
    KPI_CPU_LOAD  = (1 << 1),    /* ducati/benelly load on */
    KPI_IVA_LOAD  = (1 << 2),    /* IVA load on */
    KPI_IVA_TRACE = (1 << 3),    /* IVA trace is active */
    KPI_COMP_TRACE = (1 << 4),    /* COMP trace is active */
    KPI_CPU_TRACE = (1 << 5),    /* CPU trace is active */
    KPI_IVA_USED  = (1 << 6)     /* IVA used in the test */
} KPI_inst_status;

/* function protypes */
extern void kpi_instInit        (void);
extern void kpi_instDeinit      (void);
extern void kpi_before_codec    (void);
extern void kpi_after_codec     (void);
extern void kpi_IVA_new_freq    (unsigned long freq);

extern void kpi_comp_init   (void* hComponent);
extern void kpi_comp_deinit (void* hComponent);

/* use disableCoreInts in SMP: faster and appropriate for us, otherwise is disable (non SMP) */
#ifndef BUILD_FOR_SMP
#define  Core_hwiDisable() Hwi_disable()
#define  Core_hwiRestore(x) Hwi_restore(x)
#endif //BUILD_FOR_SMP

#endif /*_PROFILE_H*/
