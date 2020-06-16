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


#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/pm/IpcPower.h>
#include <ti/sdo/ce/Engine.h>
#include <ti/sdo/ce/CERuntime.h>
#include <xdc/cfg/global.h>

#include "dce_priv.h"

#include <ti/xdais/ires.h>
#include <ti/sdo/fc/ires/hdvicp/iresman_hdvicp.h>
#include <ti/sdo/fc/ires/hdvicp/ires_hdvicp2.h>
#include <ti/sdo/fc/ires/tiledmemory/iresman_tiledmemory.h>
#include <ti/sdo/fc/rman/rman.h>
#include <ti/sdo/fc/ires/hdvicp/hdvicp2.h>
#include <ti/sdo/fc/ires/hdvicp/hdvicp2.h>

#include <ti/ipc/remoteproc/Resource.h>

//#define MEMORYSTATS_DEBUG

static uint32_t    ivahd_base = 0;
static uint32_t    ivahd_cm_base = 0;
static uint32_t    ivahd_config_base = 0;

static uint32_t get_ivahd_base(void)
{
    if( !ivahd_base ) {
        ERROR("Chipset ID not set!");

        while( TRUE ) {
            asm (" wfi");
        }
    }
    return (ivahd_base);
}

static uint32_t get_ivahd_cm_base(void)
{
    if( !ivahd_cm_base ) {
        ERROR("Chipset ID not set!");

        while( TRUE ) {
            asm (" wfi");
        }
    }
    return (ivahd_cm_base);
}

static uint32_t get_ivahd_config_base(void)
{
    if( !ivahd_config_base ) {
        ERROR("Chipset ID not set!");

        while( TRUE ) {
            asm (" wfi");
        }
    }
    return (ivahd_config_base);
}

#define IVAHD_REG(off)            (*(volatile unsigned int *)(get_ivahd_base() + (off)))

#define PM_IVAHD_PWRSTCTRL        IVAHD_REG(0x00)
#define RM_IVAHD_RSTCTRL          IVAHD_REG(0x10)
#define RM_IVAHD_RSTST            IVAHD_REG(0x14)

#define IVAHD_CM_REG(off)         (*(volatile unsigned int *)(get_ivahd_cm_base() + (off)))

#if ((defined OMAP5430_ES10) || (defined VAYU_ES10) || (defined BUILD_FOR_OMAP4))
 #define CM_IVAHD_CLKSTCTRL        IVAHD_CM_REG(0x8F00)
 #define CM_IVAHD_CLKCTRL          IVAHD_CM_REG(0x8F20)
 #define CM_IVAHD_SL2_CLKCTRL      IVAHD_CM_REG(0x8F28)
#if ((defined OMAP5430_ES10) || (defined BUILD_FOR_OMAP4))
 #define CM_DIV_DPLL_IVA           IVAHD_CM_REG(0x41BC)
#elif (defined VAYU_ES10)
 #define CM_DIV_DPLL_IVA           IVAHD_CM_REG(0x51B0)
#else
 #error Undefined Board Type
#endif
#elif (defined OMAP5432_ES20)
 #define CM_IVAHD_CLKSTCTRL        IVAHD_CM_REG(0x9200)
 #define CM_IVAHD_CLKCTRL          IVAHD_CM_REG(0x9220)
 #define CM_IVAHD_SL2_CLKCTRL      IVAHD_CM_REG(0x9228)
 #define CM_DIV_DPLL_IVA           IVAHD_CM_REG(0x41BC)
#else
 #error Undefined Board Type
#endif //OMAP5_ES10

#define IVAHD_CONFIG_REG_BASE     (get_ivahd_config_base())
#define ICONT1_ITCM_BASE          (IVAHD_CONFIG_REG_BASE + 0x08000)
#define ICONT2_ITCM_BASE          (IVAHD_CONFIG_REG_BASE + 0x18000)


/*******************************************************************************
 *   Hex code to set for Stack setting, Interrupt vector setting
 *   and instruction to put ICONT in WFI mode.
 *   This shall be placed at TCM_BASE_ADDRESS of given IVAHD, which is
 *   0x0000 locally after reset.
 *******************************************************************************/

const unsigned int    icont_boot[] =
{
    0xEA000006,
    0xEAFFFFFE,
    0xEAFFFFFE,
    0xEAFFFFFE,
    0xEAFFFFFE,
    0xEAFFFFFE,
    0xEAFFFFFE,
    0xEAFFFFFE,
    0xE3A00000,
    0xEE070F9A,
    0xEE070F90,
    0xE3A00000,
    0xEAFFFFFE,
    0xEAFFFFF1
};

#if 0
static inline void sleepms(int ms)
{
    Task_sleep(((ms * 1000 + (Clock_tickPeriod - 1)) / Clock_tickPeriod));
}
#endif

void ivahd_boot(void)
{
    int                      i;
    volatile unsigned int   *icont1_itcm_base_addr =
        (unsigned int *)ICONT1_ITCM_BASE;
    volatile unsigned int   *icont2_itcm_base_addr =
        (unsigned int *)ICONT2_ITCM_BASE;

    /*
     * Reset IVA HD, SL2 and ICONTs
     */

    DEBUG("Booting IVAHD...");


    /* Sequence - J6 system address
     * Apply Reset on IVA-HD (0x4AE06F10 = 0x7)
     * Turn IVA power state to on (0x4AE06F00 = 0x3)
     * Set CM to SW WKUP (0x4A008F00 = 0x2)
     * Set IVA CLK to Auto (0x4A008F20 = 0x1)
     * Set SL2 CLK to Auto (0x4A008F28 = 0x1)
     * Apply reset to ICONT1 and ICONT2 and remove SL2 reset (0x4AE06F10 = 0x3)
     * Code load ICONTs
     * Release Reset for ICONT1 and ICONT2 (0x4AE06F10 = 0x0)
    */

    /* RESET RST_LOGIC, RST_SEQ2, and RST_SEQ1*/
    RM_IVAHD_RSTCTRL = 0x00000007;
    while(! (RM_IVAHD_RSTCTRL & 0x00000007) ) {
        ;
    }

    /*POWERSTATE : IVAHD_PRM:PM_IVAHD_PWRSTCTRL to ON state*/
    PM_IVAHD_PWRSTCTRL = 0x00000003;
    while(! (PM_IVAHD_PWRSTCTRL & 0x00000003) ) {
        ;
    }

    /*IVAHD_CM2:CM_IVAHD_CLKSTCTRL = SW_WKUP*/
    CM_IVAHD_CLKSTCTRL = 0x00000002;
    while(! (CM_IVAHD_CLKSTCTRL & 0x00000002) ) {
        ;
    }

    /*IVAHD_CM2:CM_IVAHD_IVAHD_CLKCTRL - IVA managed by HW*/
    CM_IVAHD_CLKCTRL = 0x00000001;
    while(! (CM_IVAHD_CLKCTRL & 0x00000001) ) {
        ;
    }

    /*IVAHD_CM2:CM_IVAHD_SL2_CLKCTRL - SL2 managed by HW*/
    CM_IVAHD_SL2_CLKCTRL = 0x00000001;
    while(! (CM_IVAHD_SL2_CLKCTRL & 0x00000001) ) {
        ;
    }

    /* put ICONT1 & ICONT2 in reset and clear IVA logic and SL2 reset */
    DEBUG("Putting [ICONTI ICONT2]: RESET and SL2:OutOfRESET...");
    RM_IVAHD_RSTCTRL = 0x00000003;
    while(! (RM_IVAHD_RSTCTRL & 0x00000003) ) {
        ;
    }

    /* Check IVA Clock IDLEST to be functional and STBYST to be standby */
    while( (CM_IVAHD_CLKCTRL & 0x00030001) != 0x00001 ) {
        DEBUG("ivahd_boot waiting for IVA Clock IDLEST to functional and STBYST CM_IVAHD_CLKCTRL 0x%x", CM_IVAHD_CLKCTRL);
    }

    /* Check SL2 Clock IDLEST to be functional */
    while( (CM_IVAHD_SL2_CLKCTRL & 0x00030001) != 0x00001 ) {
        DEBUG("ivahd_boot waiting for SL2 Clock IDLEST to functional\n");
    }

    /* Copy boot code to ICONT1 & ICONT2 memory - Initialized the TCM memory */
    for( i = 0; i < DIM(icont_boot); i++ ) {
        *icont1_itcm_base_addr++ = icont_boot[i];
        *icont2_itcm_base_addr++ = icont_boot[i];
    }

    /* Release Reset - clear SEQ2 and SEQ1 */
    RM_IVAHD_RSTCTRL = 0x00000000;
    while( RM_IVAHD_RSTCTRL != 0x00000000 ) {
        ;
    }

    /*Read CM IVAHD CLOCK STATE CONTROL is running or gating/ungating transition is on-going*/
    DEBUG("Waiting for IVAHD to go out of reset\n");

    while(((CM_IVAHD_CLKSTCTRL) & 0x100) & ~0x100 ) {
        ;
    }

    DEBUG("ivahd_boot() CM_IVAHD_CLKCTRL 0x%x CM_IVAHD_CLKSTCTRL 0x%x", CM_IVAHD_CLKCTRL, CM_IVAHD_CLKSTCTRL);
}

int ivahd_reset(void *handle, void *iresHandle)
{
    /*
     * Reset IVA HD, SL2 and ICONTs
     */

    DEBUG("Resetting IVAHD START CM_IVAHD_CLKCTRL 0x%x CM_IVAHD_CLKSTCTRL 0x%x", CM_IVAHD_CLKCTRL, CM_IVAHD_CLKSTCTRL);


    /* First put IVA into HW Auto mode */
    CM_IVAHD_CLKSTCTRL |= 0x00000003;

    /* Wait for IVA HD to standby */
    while( !((CM_IVAHD_CLKCTRL) & 0x00040000)) {
        ;
    }

    /* Disable IVAHD and SL2 modules */
    CM_IVAHD_CLKCTRL = 0x00000000;
    CM_IVAHD_SL2_CLKCTRL = 0x00000000;

    /* Ensure that IVAHD and SL2 are enabled */
    while( !(CM_IVAHD_CLKCTRL & 0x00030000)) {
        ;
    }

    while( !(CM_IVAHD_SL2_CLKCTRL & 0x00030000)) {
        ;
    }

    /* Precondition - TRM DRA7xx Sec. 3.5.6.5 IVA Subsystem Software Warm Reset Sequence
     * 1. IVA Sequencer CPUS are in IDLE state: CM_IVAHD_CLKCTRL[17:16] IDLEST - has a value 0x2.
     * 2. IVA subsystem is in STANDBY state: CM_IVAHD_CLKCTRL[18] STBYST - has a value of 0x1.
     */
    while( !(CM_IVAHD_CLKCTRL & 0x00060000) ) {
        ;
    }

    /* 3. The functional clock to the IVA subsystem has been gated by the PRCM module
     * CM_IVA_CLKSTCTRL[8] - has a value of 0x0
     */
    while( CM_IVAHD_CLKSTCTRL & 0x100) {
        ;
    }

    /* Reset IVAHD sequencers and SL2 */
    RM_IVAHD_RSTCTRL |= 0x00000007;

    /*
     * Check if modules are reset
     */

    /* First clear the status bits */
    RM_IVAHD_RSTST |= 0x00000007;

    /* Wait for confirmation that the systems have been reset */
    /* THIS CHECK MAY NOT BE NECESSARY, AND MOST OF ALL GIVEN OUR STATE,
     * MAY NOT BE POSSIBLE
     */

    /* Ensure that the wake up mode is set to SW_WAKEUP */
    CM_IVAHD_CLKSTCTRL &= 0x00000002;

    /* Enable IVAHD and SL2 modules */
    CM_IVAHD_CLKCTRL = 0x00000001;
    CM_IVAHD_SL2_CLKCTRL = 0x00000001;

    /* Deassert the SL2 reset */
    RM_IVAHD_RSTCTRL &= 0xFFFFFFFB;

    /* Ensure that IVAHD and SL2 are enabled */
    while( CM_IVAHD_CLKCTRL & 0x00030000 ) {
        ;
    }

    while( CM_IVAHD_SL2_CLKCTRL & 0x00030000 ) {
        ;
    }

    DEBUG("Resetting IVAHD COMPLETED");
    return (TRUE);
}

void crash_reset() {
    ERROR("Received crash_reset");

    IRES_Status ret;

    /* RMAN_unregister IRESMAN_TILEDMEMORY */
    ret = RMAN_unregister(&IRESMAN_TILEDMEMORY);
    if( ret != IRES_OK ) {
        ERROR("RMAN_unregister on IRESMAN_TILEDMEMORY fail with ret %d", ret);
    }

    /* RMAN_unregister IRESMAN_HDVICP */
    ret = RMAN_unregister(&IRESMAN_HDVICP);
    if( ret != IRES_OK ) {
        ERROR("RMAN_unregister on IRESMAN_HDVICP fail with ret %d", ret);
    }

    /* RMAN_exit */
    ret = RMAN_exit();
    if( ret != IRES_OK ) {
        ERROR("RMAN_exit fail with ret %d", ret);
    }

    CERuntime_exit();

    ERROR("crash_reset() CM_IVAHD_CLKCTRL 0x%x CM_IVAHD_CLKSTCTRL 0x%x", CM_IVAHD_CLKCTRL, CM_IVAHD_CLKSTCTRL);

    if( CM_IVAHD_CLKSTCTRL & 0x100 ) {
        ERROR("crash_report detects IVA_GCLK is running or gate transition on-going");
    }

    /* RESET RST_LOGIC, RST_SEQ2, and RST_SEQ1 before crashing */
    RM_IVAHD_RSTCTRL = 0x00000007;

    ERROR("crash_reset() RM_IVAHD_RSTCTRL 0x%x - calling abort NOW", RM_IVAHD_RSTCTRL);
    abort();
}


static int    ivahd_use_cnt = 0;

#ifdef ENABLE_DEAD_CODE
static inline void set_ivahd_opp(int opp)
{
    unsigned int    val;

    switch( opp ) {
        case 0 :
            val = 0x010e;
            break;
        case 50 :
            val = 0x000e;
            break;
        case 100 :
            val = ivahd_m5div;
            break;
        default :
            ERROR("invalid opp");
            return;
    }

    DEBUG("CM_DIV_DPLL_IVA=%08x", CM_DIV_DPLL_IVA);
    /* set HSDIVDER_CLKOUT2_DIV */
    CM_DIV_DPLL_IVA = (CM_DIV_DPLL_IVA & ~0x0000011f) | val;
    DEBUG("CM_DIV_DPLL_IVA=%08x", CM_DIV_DPLL_IVA);
}

#endif //ENABLE_DEAD_CODE
void ivahd_acquire(void)
{
    if( ++ivahd_use_cnt == 1 ) {
        DEBUG("ivahd acquire");
        UInt hwiKey = Hwi_disable();
        /* switch SW_WAKEUP mode */
        CM_IVAHD_CLKSTCTRL = 0x00000002;
        Hwi_restore(hwiKey);
    } else {
        DEBUG("ivahd already acquired");
    }
}

void ivahd_release(void)
{
    if( ivahd_use_cnt-- == 1 ) {
        DEBUG("ivahd release");
        UInt hwiKey = Hwi_disable();
        /* switch HW_AUTO mode */
        CM_IVAHD_CLKSTCTRL = 0x00000003;
        Hwi_restore(hwiKey);
    } else {
        DEBUG("ivahd still in use");
    }
}

/* This function is to check IVA clocks to make sure IVAHD is idle */
void ivahd_idle_check(void)
{
    DEBUG("ivahd_idle check CM_IVAHD_CLKCTRL=0x%x CM_IVAHD_SL2_CLKCTRL=0x%x\n", CM_IVAHD_CLKCTRL, CM_IVAHD_SL2_CLKCTRL);

    /* Ensure that IVAHD and SL2 idle */
    while( !(CM_IVAHD_CLKCTRL & 0x00020000)) {
        ;
    }

    while( !(CM_IVAHD_SL2_CLKCTRL & 0x00020000)) {
        ;
    }

    DEBUG("ivahd_idle_check DONE - IVAHD and SL2 are in IDLE state\n");
}

static Bool allocFxn(IALG_MemRec *memTab, Int numRecs);
static void freeFxn(IALG_MemRec *memTab, Int numRecs);

/* ivahd_init() will be called in 2 situations :
 * - when omapdce kernel module is loaded
 * - when resuming from suspend
 */
void ivahd_init(uint32_t chipset_id)
{
    IRES_Status       ret;
    uint32_t          ivahd_base_pa = 0;
    uint32_t          ivahd_cm_base_pa = 0;
    uint32_t          ivahd_config_base_pa = 0;
    IRESMAN_Params    rman_params =
    {
        .size = sizeof(IRESMAN_Params),
        .allocFxn = allocFxn,
        .freeFxn = freeFxn,
    };


    switch( chipset_id ) {
        case 0x4430 :
            ivahd_base_pa = 0x4A306F00;
            ivahd_cm_base_pa = 0x4A000000;
            ivahd_config_base_pa = 0x5A000000;
            break;
        case 0x4460 :
        case 0x4470 :
            ivahd_base_pa = 0x4A306F00;
            ivahd_cm_base_pa = 0x4A000000;
            ivahd_config_base_pa = 0x5A000000;
            break;
        case 0x5430 :
            ivahd_base_pa = 0x4AE06F00;
            ivahd_cm_base_pa = 0x4A000000;
            ivahd_config_base_pa = 0x5A000000;
            break;
        case 0x5432 :
            ivahd_base_pa = 0x4AE07200;
            ivahd_cm_base_pa = 0x4A000000;
            ivahd_config_base_pa = 0x5A000000;
            break;
        case 0x5436 :
            ivahd_base_pa = 0x4AE06F00;
            ivahd_cm_base_pa = 0x4A000000;
            ivahd_config_base_pa = 0x5A000000;
            break;
        default :
            ERROR("Invalid chipset-id: %x", chipset_id);
            break;
    }

    if (ivahd_base_pa) {
        if (Resource_physToVirt(ivahd_base_pa, &ivahd_base)) {
            ERROR("Unable to get ivahd_base\n");
            return;
        }
    }

    DEBUG("ivahd_base=%08x", ivahd_base);

    if (ivahd_cm_base_pa) {
        if (Resource_physToVirt(ivahd_cm_base_pa, &ivahd_cm_base)) {
            ERROR("Unable to get ivahd_cm_base\n");
            return;
        }
    }

    DEBUG("ivahd_cm_base=%08x", ivahd_cm_base);

    if (ivahd_config_base_pa) {
        if (Resource_physToVirt(ivahd_config_base_pa, &ivahd_config_base)) {
            ERROR("Unable to get ivahd_config_base\n");
            return;
        }
    }

    /* bit of a hack.. not sure if there is a better way for this: */
    HDVICP2_PARAMS.resetControlAddress[0] = ivahd_base + 0x10;

    ivahd_acquire();

    CERuntime_init();

    ret = RMAN_init();
    if( ret != IRES_OK ) {
        ERROR("RMAN_init is failing ret %d", ret);
        CERuntime_exit();
        goto end;
    }

    /* Register HDVICP with RMAN if not already registered */
    ret = RMAN_register(&IRESMAN_HDVICP, &rman_params);
    if((ret != IRES_OK) && (ret != IRES_EEXISTS)) {
        ERROR("could not register IRESMAN_HDVICP: %d", ret);
        RMAN_exit();
        CERuntime_exit();
        goto end;
    }

    /* NOTE: this might try MemMgr allocations over RCM remote
     * call back to host side.  Which will fail if we don't
     * have memsrv.  But will eventually fall back to allocFxn
     * which will allocate from the local heap.
     */
    ret = RMAN_register(&IRESMAN_TILEDMEMORY, &rman_params);
    if((ret != IRES_OK) && (ret != IRES_EEXISTS)) {
        ERROR("could not register IRESMAN_TILEDMEMORY: %d", ret);
        ret = RMAN_unregister(&IRESMAN_HDVICP);
        if( ret != IRES_OK ) {
            ERROR("RMAN_unregister on IRESMAN_HDVICP fail with ret %d", ret);
        }
        RMAN_exit();
        CERuntime_exit();
        goto end;
    }


    ivahd_boot();

    DEBUG("RMAN_register() for HDVICP is successful");

end:
    ivahd_release();
    return;
}

static Bool allocFxn(IALG_MemRec memTab[], Int n)
{
    Int    i;

#ifdef MEMORYSTATS_DEBUG
    Memory_Stats    stats;
#endif

    for( i = 0; i < n; i++ ) {
        Error_Block    eb;
        Uns            pad, size;
        void          *blk;
        MemHeader     *hdr;

        if( memTab[i].alignment > sizeof(MemHeader)) {
            pad = memTab[i].alignment;
        } else {
            pad = sizeof(MemHeader);
        }

        size = memTab[i].size + pad;

#ifdef MEMORYSTATS_DEBUG
        Memory_getStats(NULL, &stats);
        INFO("Total: %d\tFree: %d\tLargest: %d", stats.totalSize,
             stats.totalFreeSize, stats.largestFreeSize);
#endif

        blk = Memory_alloc(NULL, size, memTab[i].alignment, &eb);

        if( !blk ) {
            ERROR("MemTab Allocation failed at %d", i);
            freeFxn(memTab, i);
            return (FALSE);
        } else {
            memTab[i].base = (void *)((char *)blk + pad);
            hdr = P2H(memTab[i].base);
            hdr->size = size;
            hdr->ptr  = blk;
            DEBUG("%d: alloc: %p/%p (%d)", i, hdr->ptr,
                  memTab[i].base, hdr->size);
        }
    }

    DEBUG("MemTab Allocation is Successful");
    return (TRUE);
}

static void freeFxn(IALG_MemRec memTab[], Int n)
{
    Int    i;

#ifdef MEMORYSTATS_DEBUG
    Memory_Stats    stats;
#endif

    for( i = 0; i < n; i++ ) {
        if( memTab[i].base != NULL ) {
            MemHeader   *hdr = P2H(memTab[i].base);

#ifdef MEMORYSTATS_DEBUG
            DEBUG("%d: free: %p/%p (%d)", n, hdr->ptr,
                  memTab[i].base, hdr->size);
#endif
            Memory_free(NULL, hdr->ptr, hdr->size);
        }
#ifdef MEMORYSTATS_DEBUG
        Memory_getStats(NULL, &stats);
        INFO("Total: %d\tFree: %d\tLargest: %d", stats.totalSize,
             stats.totalFreeSize, stats.largestFreeSize);
#endif
    }
}

