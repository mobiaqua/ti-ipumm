/*
 * Copyright (c) 2011, Texas Instruments Incorporated
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <ti/grcm/RcmServer.h>
#include <ti/grcm/RcmTypes.h>
#include <ti/ipc/mm/MmServiceMgr.h>
#include <ti/ipc/mm/MmRpc.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/rpmsg/RPMessage.h>
#include <ti/ipc/rpmsg/NameMap.h>
#include <ti/pm/IpcPower.h>
#include <ti/sdo/ce/global/CESettings.h>
#include <ti/sdo/ce/Engine.h>
#include <ti/sdo/fc/global/FCSettings.h>
#include <ti/sdo/fc/utils/fcutils.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Cache.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/posix/ccs/pthread.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/knl/Thread.h>
#include <xdc/std.h>
#include <ti/sysbios/utils/Load.h>

#include "dce_priv.h"
#include "dce_rpc.h"
#include "ti/utils/profile.h"

static uint32_t    suspend_initialised = 0;
uint32_t           dce_debug = DCE_DEBUG_LEVEL;

#define SERVER_NAME "rpmsg-dce"
#define CALLBACK_SERVER_NAME "dce-callback"

#define MEMORYSTATS_DEBUG
/* Each client is based on a unique id from MmServiceMgr which is the connect identity */
/*   created by IPC per MmRpc_create instances                                         */
#define NUM_CLIENTS 10
/* Each client can have NUM_INSTANCE of ENGINE, DECODE_CODEC, and ENCODE_CODEC handle */
/* Commonly one ENGINE is associated with one DECODE_CODEC or one ENCODE_CODEC handle */
#define NUM_INSTANCE 5

#define MmRpc_NUM_PARAMETERS(size) \
    (size / sizeof(MmType_Param))

/* dce_inv, dce_clean needs to be modified to expect buffers */
/* without headers (relevant for GLP)                        */
static void dce_inv(void *ptr)
{
    if( ptr ) {
        Cache_inv(ptr, P2H(ptr)->size, Cache_Type_ALL, TRUE);
    }
}

static void dce_clean(void *ptr)
{
    if( ptr ) {
        Cache_wbInv(ptr, P2H(ptr)->size, Cache_Type_ALL, TRUE);
    }
}

typedef void * (*CreateFxn)(Engine_Handle, String, void *);
typedef Int32 (*ControlFxn)(void *, int, void *, void *);
typedef Int32 (*ProcessFxn)(void *, void *, void *, void *, void *);
typedef Int32 (*RelocFxn)(void *, uint8_t *ptr, uint32_t len);
typedef void (*DeleteFxn)(void *);

/* DCE Server static function declarations */
static Int32 engine_open(UInt32 size, UInt32 *data);
static Int32 engine_close(UInt32 size, UInt32 *data);

/* Encoder Server static function declarations */
static VIDENC2_Handle videnc2_create(Engine_Handle engine, String name, VIDENC2_Params *params);
static XDAS_Int32 videnc2_control(VIDENC2_Handle codec, VIDENC2_Cmd id, VIDENC2_DynamicParams *dynParams, VIDENC2_Status *status);
static int videnc2_reloc(VIDDEC3_Handle handle, uint8_t *ptr, uint32_t len);

/* Decoder Server static function declarations */
static VIDDEC3_Handle viddec3_create(Engine_Handle engine, String name, VIDDEC3_Params *params);
static XDAS_Int32 viddec3_control(VIDDEC3_Handle codec,VIDDEC3_Cmd id,VIDDEC3_DynamicParams * dynParams,VIDDEC3_Status * status);
static XDAS_Int32 viddec3_process(VIDDEC3_Handle codec, XDM2_BufDesc *inBufs, XDM2_BufDesc *outBufs, VIDDEC3_InArgs *inArgs, VIDDEC3_OutArgs *outArgs);

static Int32 get_rproc_info(UInt32 size, UInt32 *data);

static int viddec3_reloc(VIDDEC3_Handle handle, uint8_t *ptr, uint32_t len);

static Semaphore_Handle sync_process_sem;

typedef struct {
    XDM_DataSyncHandle dataSyncHandle;
    XDM_DataSyncDesc *dataSyncDesc;
    pthread_mutex_t callback_mutex;
    pthread_cond_t synch_callback;
    Uint32 row_mode;
    Uint32 getdata_ready;
    Uint32 codec_request;
    Uint32 putdata_ready;
    Uint32 mpu_crash_indication;
    Uint32 putdata_toclient;
    Uint32 putData_endprocess;
} Callback_data;

typedef struct {
    Uint32 mm_serv_id;  /* value of zero means unused */
    Int refs;           /* reference count on number of engine */
    Engine_Handle engines[NUM_INSTANCE];
    VIDDEC3_Handle decode_codec[NUM_INSTANCE];
    VIDENC2_Handle encode_codec[NUM_INSTANCE];
    Uint32 codec_id;
    Callback_data decode_callback[NUM_INSTANCE];
    Callback_data encode_callback[NUM_INSTANCE];
} Client;
static Client clients[NUM_CLIENTS] = {0};

static inline Client * get_client(Uint32 mm_serv_id)
{
    int i;
    for (i = 0; i < DIM(clients); i++) {
        if (clients[i].mm_serv_id == mm_serv_id) {
            return &clients[i];
        }
    }
    return NULL;
}

static inline Client * get_client_instance(Uint32 codec)
{
    int i, j;

    for (i = 0; i < DIM(clients); i++) {
        for (j = 0; j < DIM(clients[i].decode_codec); j++) {
            if (clients[i].decode_codec[j] == (VIDDEC3_Handle) codec) {
                DEBUG("Found VIDDEC3 clients[%d] = 0x%x, &clients[%d] = 0x%x", i, clients[i], i, &clients[i]);
                return &clients[i];
            }
            if (clients[i].encode_codec[j] == (VIDENC2_Handle) codec) {
                DEBUG("Found VIDENC2 clients[%d] = 0x%x, &clients[%d] = 0x%x", i, clients[i], i, &clients[i]);
                return &clients[i];
            }
        }
    }
    return NULL;
}

static struct {
    CreateFxn  create;
    ControlFxn control;
    ProcessFxn process;
    DeleteFxn  delete;
    RelocFxn   reloc;   /* handle buffer relocation table */
} codec_fxns[] =
{
    [OMAP_DCE_VIDENC2] =
    {
        (CreateFxn)videnc2_create,   (ControlFxn)videnc2_control,
        (ProcessFxn)VIDENC2_process, (DeleteFxn)VIDENC2_delete,
        (RelocFxn)videnc2_reloc,
    },
    [OMAP_DCE_VIDDEC3] =
    {
        (CreateFxn)viddec3_create,   (ControlFxn)viddec3_control,
        (ProcessFxn)viddec3_process, (DeleteFxn)VIDDEC3_delete,
        (RelocFxn)viddec3_reloc,
    },
};


/* Static version string buffer.
 * Note: codec version can be large. For example, h264vdec needs more than
 * 58 characters, or the query will fail. */
#define VERSION_SIZE 128
static char    version_buffer[VERSION_SIZE];

/* the following callbacks are needed for suspend/resume
 * on the linux side.
 * - FC_suspend() waits for all algorithms to get deactivated and
 * and takes care of the resources acquired.
 * - FC_resume() does nothing for now, but we add it just in case
 * it gets populated in the future versions of framework components.
 *
 * Forced off mode during video decode/encode is not supported. */
static void dce_suspend()
{
    INFO("Preparing for suspend...");
    FC_suspend();
}

static void dce_resume()
{
    INFO("Restoring after suspend...");
    FC_resume();
}

static void get_videnc2_version(VIDENC2_Handle h, char *buffer, unsigned size)
{
    VIDENC2_DynamicParams    params =
    {
        .size = sizeof(VIDENC2_DynamicParams),
    };

    VIDENC2_Status    status =
    {
        .size = sizeof(VIDENC2_Status),
        .data =
        {
            .buf = (XDAS_Int8 *)buffer,
            .bufSize = (XDAS_Int32)size,
        },
    };

    XDAS_Int32    s;

    memset(buffer, 0, size);
    s = VIDENC2_control(h, XDM_GETVERSION, &params, &status);

    if( s != VIDENC2_EOK ) {
        ERROR("Unknown version Error = %d:: buffer = %p size = %d", s, buffer, size);
    }
}

// VIDENC2_create wrapper, to display version string in the trace.
static VIDENC2_Handle videnc2_create(Engine_Handle engine, String name, VIDENC2_Params *params)
{
    VIDENC2_Handle    h;

    h = VIDENC2_create(engine, name, params);

    if( h ) {
        get_videnc2_version(h, version_buffer, VERSION_SIZE);
        INFO("Created videnc2 %s: version %s", name, version_buffer);
    }

    return (h);
}

static XDAS_Int32 getBufferFxnStub(XDM_DataSyncHandle handle, XDM_DataSyncDesc *desc)
{
    return (0);
}

static XDAS_Int32 videnc2_control(VIDENC2_Handle codec, VIDENC2_Cmd id,
                                  VIDENC2_DynamicParams *dynParams, VIDENC2_Status *status)
{
    Client* c;

    c = get_client_instance((Uint32) codec);
    if (c) {
        int i;
        for (i = 0; i < DIM(c->encode_codec); i++) {
            if (c->encode_codec[i] == codec) {
                if (c->encode_callback[i].row_mode) {
                    if (id == XDM_SETPARAMS) {
                        c->encode_callback[i].dataSyncHandle = codec;
                        c->encode_callback[i].mpu_crash_indication = FALSE;
                        dynParams->getDataFxn = (XDM_DataSyncGetFxn) H264E_GetDataFxn;
                        dynParams->getDataHandle = codec;
                    }
                }
            }
        }
    }

    dynParams->getBufferFxn = getBufferFxnStub;
    return (VIDENC2_control(codec, id, dynParams, status));
}

static void get_viddec3_version(VIDDEC3_Handle h, char *buffer, unsigned size)
{
    VIDDEC3_DynamicParams    params =
    {
        .size = sizeof(VIDDEC3_DynamicParams),
    };

    VIDDEC3_Status    status =
    {
        .size = sizeof(VIDDEC3_Status),
        .data =
        {
            .buf = (XDAS_Int8 *)buffer,
            .bufSize = (XDAS_Int32)size,
        },
    };

    XDAS_Int32    s;

    memset(buffer, 0, size);
    s = VIDDEC3_control(h, XDM_GETVERSION, &params, &status);

    if( s != VIDDEC3_EOK ) {
        ERROR("Unknown version Error = %d:: buffer = %p size = %d", s, buffer, size);
    }
}

// VIDDEC3_create wrapper, to display version string in the trace.
static VIDDEC3_Handle viddec3_create(Engine_Handle engine, String name, VIDDEC3_Params *params)
{
    VIDDEC3_Handle    h;

    DEBUG(">> engine=%08x, name=%s, params=%p", engine, name, params);
    DEBUG(">> max_height %d max_width %d frame_rate %d", params->maxHeight, params->maxWidth, params->maxFrameRate);

    h = VIDDEC3_create(engine, name, params);

    if( h ) {
        get_viddec3_version(h, version_buffer, VERSION_SIZE);
        INFO("Created viddec3 %s: version %s", name, version_buffer);
    }

    return (h);
}

static XDAS_Int32 viddec3_control(VIDDEC3_Handle codec,VIDDEC3_Cmd id,VIDDEC3_DynamicParams * dynParams,VIDDEC3_Status * status)
{
    Client* c;

    c = get_client_instance((Uint32) codec);
    if (c) {
        int i;
        for (i = 0; i < DIM(c->decode_codec); i++) {
            if (c->decode_codec[i] == codec) {
                if (c->decode_callback[i].row_mode) {
                    DEBUG("Check codec 0x%x control id %d", codec, id);
                    if (id == XDM_SETPARAMS) {
                        c->decode_callback[i].dataSyncHandle = codec;
                        c->decode_callback[i].mpu_crash_indication = FALSE;
                        dynParams->putDataFxn = (XDM_DataSyncPutFxn) H264D_PutDataFxn;
                        dynParams->putDataHandle = codec;
                    }
                }
            }
        }
    }

    //dynParams->putBufferFxn = putBufferFxnStub;
    return (VIDDEC3_control(codec, id, dynParams, status));
}

static XDAS_Int32 viddec3_process(VIDDEC3_Handle codec, XDM2_BufDesc *inBufs, XDM2_BufDesc *outBufs, VIDDEC3_InArgs *inArgs, VIDDEC3_OutArgs *outArgs)
{
    Client* c;
    XDAS_Int32 ret;

    c = get_client_instance((Uint32) codec);
    if (c) {
        int i;
        for (i = 0; i < DIM(c->decode_codec); i++) {
            if (c->decode_codec[i] == codec) {
                if (c->decode_callback[i].row_mode) {
                    DEBUG("Codec 0x%x", codec);
                    c->decode_callback[i].codec_request = 0;
                    c->decode_callback[i].putdata_toclient = 0;
                    c->decode_callback[i].putData_endprocess = 0;
                    ret = VIDDEC3_process(codec, inBufs, outBufs, inArgs, outArgs);

                    // Set flag and signal for put_DataFxn to return.
                    c->decode_callback[i].putData_endprocess = 1;
                    pthread_cond_signal(&(c->decode_callback[i].synch_callback));
                    return (ret);
                }
            }
        }
    }
    return (VIDDEC3_process(codec, inBufs, outBufs, inArgs, outArgs));
}

static int videnc2_reloc(VIDENC2_Handle handle, uint8_t *ptr, uint32_t len)
{
    return (-1); // Not implemented
}

/* Only valid if XDM_MOVEBUFS added in XDC tools */
static int viddec3_reloc(VIDDEC3_Handle handle, uint8_t *ptr, uint32_t len)
{
    return (-1); // Not implemented
#if 0
    static VIDDEC3_DynamicParams    params =
    {
        .size = sizeof(VIDDEC3_DynamicParams),
    };
    VIDDEC3_Status    status =
    {
        .size = sizeof(VIDDEC3_Status),
        .data =
        {
            .buf = (XDAS_Int8 *)ptr,
            .bufSize = (XDAS_Int32)len,
        },
    };
    INFO("status.size=%d", status.size);
    return (VIDDEC3_control(handle, XDM_MOVEBUFS, &params, &status));
#endif
}

/*
 * RPC message handlers
 */
static int connect(void *msg)
{
    dce_connect   *req = msg;

    DEBUG(">> chipset_id=0x%x, debug=%d", req->chipset_id, req->debug);

    if( dce_debug >= MAX_DEBUG_LEVEL ) {
        DEBUG("Enable FC, CE and IPC traces");

        FCSettings_init();
        Diags_setMask(FCSETTINGS_MODNAME "+12345678LEXAIZFS");
        CESettings_init();
        Diags_setMask(CESETTINGS_MODNAME "+12345678LEXAIZFS");

        /*
            * Enable use of runtime Diags_setMask per module:
            *
            * Codes: E = ENTRY, X = EXIT, L = LIFECYCLE, F = INFO, S = STATUS
            */
        Diags_setMask("ti.ipc.rpmsg.RPMessage=EXLFS");
        Diags_setMask("ti.ipc.rpmsg.VirtQueue=EXLFS");
    }

    ivahd_init(req->chipset_id);

    if( !suspend_initialised ) {

        /* registering sysbios-rpmsg callbacks for suspend/resume */
        IpcPower_registerCallback(IpcPower_Event_SUSPEND, (IpcPower_CallbackFuncPtr)dce_suspend, 0);
        IpcPower_registerCallback(IpcPower_Event_RESUME, (IpcPower_CallbackFuncPtr)dce_resume, 0);
        suspend_initialised++;
    }

    DEBUG("<<");

    return (0);
}

static Int32 dce_register_engine(Uint32 mm_serv_id, Engine_Handle engine)
{
    Int32  ret = 0;
    Uint32 clientIndex = 0;
    Client *c;

    c = get_client(mm_serv_id);
    if( c ) {
        int i;
        DEBUG("found mem client: %p refs=%d", c, c->refs);
        c->refs++;
        for (i = 0; i < DIM(c->engines); i++) {
            if (c->engines[i] == NULL) {
                c->engines[i] = engine;
                clientIndex = i;
                DEBUG("registered engine: mm_serv_id=%x engine=%p", mm_serv_id, engine);
                break;
            }
        }

        //If clientIndex is equal to 0 and it is not the register engine, then we know that no more
        //empty spot since clientIndex not getting set in the for loop.
        if( (clientIndex == 0) && (c->engines[clientIndex]) != engine ){
            ERROR("No more empty space for engine");
            ret = -1;
            goto out;
        }
    }else {
        c = get_client(0);
        if (!c) {
            ERROR("too many clients");
            ret = -1;
            goto out;
        }
        DEBUG("new client: %p refs=%d", c, c->refs);

        c->mm_serv_id = mm_serv_id;
        c->refs = 1;
        c->engines[0] = engine;
    }
out:
    if ((ret != 0) && (c)){
        c->mm_serv_id = NULL;
        c->refs--;
        c->engines[0] = NULL;
    }
    return ret;
}

static void dce_unregister_engine(Uint32 mm_serv_id, Engine_Handle engine)
{
    Client *c;

    c = get_client(mm_serv_id);
    if( c ) {
        int i;
        DEBUG("found mem client: %p refs=%d", c, c->refs);

        for( i = 0; i < DIM(c->engines); i++ ) {
            if( c->engines[i] == engine ) {
                c->engines[i] = NULL;
                DEBUG("unregistered engine: mm_serv_id=0x%x engine=%p", mm_serv_id, engine);
                break;
            }
        }

        //If i is equal to the size of Array, then it means it has search the whole array.
        if( i == DIM(c->engines) ) {
            ERROR("Unknown engine received on dce_unregister_engine");
            return;
        }

        DEBUG("dce_unregister_engine: %p refs=%d", c, c->refs);
        c->refs--;

        if( !c->refs ) {
            c->mm_serv_id = NULL;
        }
    }
}

static Int32 dce_register_codec(Uint32 type, Uint32 mm_serv_id, Uint32 codec)
{
    Client *c;
    Int32 ret = 0;
    Uint32 clientIndex = 0;

    c = get_client(mm_serv_id);
    if( c ) {
        int i;
        DEBUG("found mem client: %p refs=%d", c, c->refs);

        if( type == OMAP_DCE_VIDDEC3 ) {
            for( i = 0; i < DIM(c->decode_codec); i++ ) {
                if( c->decode_codec[i] == NULL ) {
                    c->decode_codec[i] = (VIDDEC3_Handle) codec;
                    c->codec_id = type;
                    clientIndex = i;
                    DEBUG("registering codec: decode_codec[%d] codec=%p", i, codec);
                    break;
                }
            }

            //If clientIndex is equal to 0 and it's not the register codec, then we know that no more
            //empty spot since clientIndex not getting set in the for loop.
            if( (clientIndex == 0) && (c->decode_codec[clientIndex]) != (VIDDEC3_Handle) codec ) {
                ERROR("No more empty space for codecs");
                ret = -1;
                goto out;
            }
        }
        else if( type == OMAP_DCE_VIDENC2 ) {
            for( i = 0; i < DIM(c->encode_codec); i++ ) {
                if( c->encode_codec[i] == NULL ) {
                    c->encode_codec[i] = (VIDENC2_Handle) codec;
                    c->codec_id = type;
                    clientIndex = i;
                    DEBUG("registering codec: encode_codec[%d] codec=%p", i, codec);
                    break;
                }
            }

            //If clientIndex is equal to 0 and it's not the register codec, then we know that no more
            //empty spot since clientIndex not getting set in the for loop.
            if( (clientIndex == 0) && (c->encode_codec[clientIndex]) != (VIDENC2_Handle) codec ) {
                ERROR("No more empty space for codecs");
                ret = -1;
                goto out;
            }
        }
    }

out:
    return ret;
}

static void dce_unregister_codec(Uint32 type, Uint32 mm_serv_id, Uint32 codec)
{
    Client *c;

    c = get_client(mm_serv_id);
    if (c) {
        int i;
        DEBUG("found mem client: %p refs=%d", c, c->refs);

        if( type == OMAP_DCE_VIDDEC3 ) {
            for( i = 0; i < DIM(c->decode_codec); i++ ) {
                if( c->decode_codec[i] == (VIDDEC3_Handle) codec ) {
                    c->decode_codec[i] = NULL;
                    DEBUG("unregistered decode_codec[%d] type Decoder codec=%p", i, codec);
                    break;
                }
            }
        }
        else if( type == OMAP_DCE_VIDENC2 ) {
            for( i = 0; i < DIM(c->encode_codec); i++ ) {
                if( c->encode_codec[i] == (VIDENC2_Handle) codec ) {
                    c->encode_codec[i] = NULL;
                    DEBUG("unregistered encode_codec[%d] type Encoder codec=%p", i, codec);
                    break;
                }
            }
        }
    }
}

/*
 * Engine_open:
 */
static Int32 engine_open(UInt32 size, UInt32 *data)
{
    MmType_Param      *payload = (MmType_Param *)data;
    dce_engine_open   *engine_open_msg = (dce_engine_open *)payload[0].data;
    Uint32             mm_serv_id = 0;
    Engine_Handle      eng_handle = NULL;
    Uint32             num_params = MmRpc_NUM_PARAMETERS(size);
    Int32              ret = 0;

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG(">> engine_open");

    if( num_params != 1 ) {
        ERROR("Invalid number of params sent");
        return (-1);
    }

    dce_inv(engine_open_msg);

    eng_handle = Engine_open(engine_open_msg->name, engine_open_msg->engine_attrs, &engine_open_msg->error_code);

    if( eng_handle ) {
        mm_serv_id = MmServiceMgr_getId();
        DEBUG("engine_open mm_serv_id 0x%x eng_handle %08x", mm_serv_id, eng_handle);

        ret = dce_register_engine(mm_serv_id, eng_handle);
        if( ret < 0 ) {
            Engine_close(eng_handle);
            eng_handle = NULL;
        }
    }

    DEBUG("<< engine=%08x, ec=%d", eng_handle, engine_open_msg->error_code);

    dce_clean(engine_open_msg);
    Semaphore_post(sync_process_sem);

    return ((Int32)eng_handle);
}

/*
 * Engine_close:
 */
static Int32 engine_close(UInt32 size, UInt32 *data)
{
    MmType_Param    *payload = (MmType_Param *)data;
    Engine_Handle    eng_handle = (Engine_Handle)payload[0].data;
    Uint32           mm_serv_id = 0;
    Uint32           num_params = MmRpc_NUM_PARAMETERS(size);

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG(">> engine_close %08x", eng_handle);

    if( num_params != 1 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    mm_serv_id = MmServiceMgr_getId();
    DEBUG("engine_close mm_serv_id 0x%x", mm_serv_id);

    dce_unregister_engine(mm_serv_id, eng_handle);

    Engine_close(eng_handle);
    DEBUG("<<");

    Semaphore_post(sync_process_sem);

    return (0);
}


#define INFO_TYPE_CPU_LOAD 0
#define INFO_TYPE_TOTAL_HEAP_SIZE 1
#define INFO_TYPE_AVAILABLE_HEAP_SIZE 2

static Int32 get_rproc_info(UInt32 size, UInt32 *data)
{
    MmType_Param    *payload = (MmType_Param *)data;
    Uint32           info_type = (Uint32)payload[0].data;
    Memory_Stats     stats;
    Uint32           output = 0;

    switch(info_type)
    {
        case INFO_TYPE_CPU_LOAD:
            output = Load_getCPULoad();
            break;

        case INFO_TYPE_TOTAL_HEAP_SIZE:
            Memory_getStats(NULL, &stats);
            output = stats.totalSize;
            break;

        case INFO_TYPE_AVAILABLE_HEAP_SIZE:
            Memory_getStats(NULL, &stats);
            output = stats.totalFreeSize;
            break;

        default:
            System_printf("\n ERROR: Invalid INFO TYPE chosen \n");
            break;
    }

    return output;
}



/*
  * codec_create
  */
static Int32 codec_create(UInt32 size, UInt32 *data)
{
    MmType_Param    *payload = (MmType_Param *)data;
    Uint32           codec_id      = (Uint32)payload[0].data;
    Engine_Handle    engine = (Engine_Handle)payload[1].data;
    char            *codec_name    = (char *)payload[2].data;
    void            *static_params = (void *)payload[3].data;
    Uint32           mm_serv_id = 0;
    Uint32           num_params = MmRpc_NUM_PARAMETERS(size);
    void            *codec_handle;
    Int32            ret = 0;
    Client*          c;

#ifdef MEMORYSTATS_DEBUG
    Memory_Stats    stats;
#endif

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG(">> codec_create on engine %08x", engine);

    if( num_params != 4 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    dce_inv(codec_name);
    dce_inv(static_params);

    /* The next source code statement shouldn't get executed in real world as the client should send */
    /* the correct width and height for the video resolution to be decoded.                          */
    /* It is coded to test the Error Recovery when IPUMM on IPU2 is crashing                         */
    /* by simulating the condition where A15 will send maxHeight as zero                             */
    if( (((VIDDEC3_Params*)static_params)->maxHeight == 0) && (codec_id == OMAP_DCE_VIDDEC3) ) {
        DEBUG("IPC RECOVERY will be performed due to maxHeight is zero which will cause exception!!!!");
        num_params = num_params / ((VIDDEC3_Params*)static_params)->maxHeight;
        System_printf("Crashing the IPU2 after divided by zero num_params %d", num_params);
    }

    ivahd_acquire();

    codec_handle = (void *)codec_fxns[codec_id].create(engine, codec_name, static_params);
    ivahd_release();

    if( codec_handle ) {
        mm_serv_id = MmServiceMgr_getId();

        ret = dce_register_codec(codec_id, mm_serv_id, (Uint32) codec_handle);
        if( ret < 0 ) {
            codec_fxns[codec_id].delete((void *)codec_handle);
            codec_handle = NULL;
        } else {
            if( codec_id == OMAP_DCE_VIDDEC3 ) {
                DEBUG("codec_create for VIDDEC3 codec_handle 0x%x mm_serv_id 0x%x", codec_handle, mm_serv_id);
                if( ((VIDDEC3_Params*)static_params)->outputDataMode == IVIDEO_NUMROWS ) {
                    c = get_client_instance((Uint32) codec_handle);
                    int i;
                    for( i = 0; i < DIM(c->decode_codec); i++ ) {
                        if( c->decode_codec[i] == codec_handle ) {
                            c->decode_callback[i].row_mode = 1;
                            pthread_mutex_init(&(c->decode_callback[i].callback_mutex), NULL);
                            pthread_cond_init(&(c->decode_callback[i].synch_callback), NULL);
                            DEBUG("codec_create client 0x%x c->decode_callback[%d].callback_mutex 0x%x c->decode_callback[%d].row_mode %d",
                                c, i, c->decode_callback[i].callback_mutex, i, c->decode_callback[i].row_mode);
                        }
                    }
                }
            } else if( codec_id == OMAP_DCE_VIDENC2 ) {
                DEBUG("codec_create for VIDENC2 codec_handle 0x%x mm_serv_id 0x%x", codec_handle, mm_serv_id);
                if( ((VIDENC2_Params*)static_params)->inputDataMode == IVIDEO_NUMROWS ) {
                    c = get_client_instance((Uint32) codec_handle);
                    int i;
                    for( i = 0; i < DIM(c->encode_codec); i++ ) {
                        if( c->encode_codec[i] == codec_handle ) {
                            c->encode_callback[i].row_mode = 1;
                            pthread_mutex_init(&(c->encode_callback[i].callback_mutex), NULL);
                            pthread_cond_init(&(c->encode_callback[i].synch_callback), NULL);
                            DEBUG("codec_create client 0x%x c->encode_callback[%d].callback_mutex 0x%x c->encode_callback[%d].row_mode %d",
                                c, i, c->encode_callback[i].callback_mutex, i, c->encode_callback[i].row_mode);
                        }
                    }
                }
            }
        }
    }
    DEBUG("<< codec_handle=%08x on engine %08x", codec_handle, engine);

    dce_clean(static_params);
    dce_clean(codec_name);

    Semaphore_post(sync_process_sem);

#ifdef MEMORYSTATS_DEBUG
    Memory_getStats(NULL, &stats);
    INFO("Total: %d\tFree: %d\tLargest: %d", stats.totalSize,
         stats.totalFreeSize, stats.largestFreeSize);
#endif
#ifdef PSI_KPI
        kpi_comp_init(codec_handle);
#endif /*PSI_KPI*/
    return ((Int32)codec_handle);
}

/*
  * codec_control
  */
static int codec_control(UInt32 size, UInt32 *data)
{
    MmType_Param   *payload = (MmType_Param *)data;
    Uint32          codec_id            = (Uint32)payload[0].data;
    void           *codec_handle = (Engine_Handle)payload[1].data;
    uint32_t        cmd_id              = (Uint32)payload[2].data;
    void           *dyn_params          = (void *)payload[3].data;
    void           *status              = (void *)payload[4].data;
    Uint32          num_params = MmRpc_NUM_PARAMETERS(size);
    Int32           ret = 0;

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG(">> codec_control on codec_handle %08x", codec_handle);

    if( num_params != 5 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    dce_inv(dyn_params);
    dce_inv(status);

    /* Only for cmd_id == XDM_FLUSH/XDM_MOVEBUF ? */
    ivahd_acquire();

    ret = (uint32_t) codec_fxns[codec_id].control(codec_handle, cmd_id, dyn_params, status);
    ivahd_release();

    DEBUG("<< codec_control on codec_handle %08x result=%d", codec_handle, ret);

    dce_clean(dyn_params);
    dce_clean(status);

    Semaphore_post(sync_process_sem);

    return (ret);
}

/*
  * codec get version
  */
static int codec_get_version(UInt32 size, UInt32 *data)
{
    MmType_Param   *payload = (MmType_Param *)data;
    Uint32          codec_id            = (Uint32)payload[0].data;
    void           *codec_handle = (Engine_Handle)payload[1].data;
    void           *dyn_params          = (void *)payload[2].data;
    void           *status              = (void *)payload[3].data;
    Uint32          num_params = MmRpc_NUM_PARAMETERS(size);
    void           *version_buf = NULL;
    Int32           ret = 0;

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG(">> codec_get_version on codec_handle %08x", codec_handle);

    if( num_params != 4 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    dce_inv(dyn_params);
    dce_inv(status);

    if( codec_id == OMAP_DCE_VIDDEC3 ) {
        version_buf = (void *)(H2P((MemHeader *)((IVIDDEC3_Status *)status)->data.buf));
    } else if( codec_id == OMAP_DCE_VIDENC2 ) {
        version_buf = (void *)(H2P((MemHeader *)((IVIDENC2_Status *)status)->data.buf));
    }

    dce_inv(version_buf);

    ivahd_acquire();
    ret = (uint32_t) codec_fxns[codec_id].control(codec_handle, XDM_GETVERSION, dyn_params, status);
    ivahd_release();

    DEBUG("<< codec_get_version on codec_handle %08x result=%d", codec_handle, ret);

    dce_clean(dyn_params);
    dce_clean(status);
    dce_clean(version_buf);

    Semaphore_post(sync_process_sem);\

    return (ret);
}

/* Notes about serialization of process command:
 *
 * Since codec_process code on kernel side is doing buffer mapping/unmapping,
 * and keeping track of codec's locked buffers, it is necessary for it to
 * look into the contents of some of the parameter structs, and in some cases
 * re-write them.  For this reason inArgs/outBufs/inBufs are serialized within
 * the rpmsg rather than just passed by pointer.

XDAS_Int32 VIDDEC3_process(VIDDEC3_Handle handle, XDM2_BufDesc *inBufs,
    XDM2_BufDesc *outBufs, VIDDEC3_InArgs *inArgs, VIDDEC3_OutArgs *outArgs);

  REQ:
    struct dce_rpc_hdr hdr   -> 4
    codec_id                 -> 4
    codec                    -> 4
    reloc length             -> 1   (length/4)
    inArgs length            -> 1   (length/4)
    outBufs length           -> 1   (length/4)
    inBufs length            -> 1   (length/4)
    VIDDEC3_OutArgs *outArgs -> 4   (pass by pointer)
    reloc table              -> 12 * nreloc (typically <= 16)
    VIDDEC3_InArgs   inArgs  -> 12  (need inputID from userspace)
    XDM2_BufDesc     outBufs -> 44  (4 + 2 * 20)
    XDM2_BufDesc     inBufs  -> 24  (4 + 1 * 20)
    -------------------------------
                               99

  RSP
    struct dce_rpc_hdr hdr   -> 4
    result                   -> 4
    inBufs length            -> 1   (length/4)
    XDAS_Int32 freeBufID[]   -> 4*n (n typically 0 or 2, but could be up to 20)
    -------------------------------
                               9-89
    Note: freeBufID[] duplicates what is returned in outArgs, but avoids
    needing to create kernel mappings of these objects which are to big
    to copy inline.  Also it avoids differences between VIDDEC3/VIDDENC2.


XDAS_Int32 VIDENC2_process(VIDENC2_Handle handle, IVIDEO2_BufDesc *inBufs,
    XDM2_BufDesc *outBufs, IVIDENC2_InArgs *inArgs, IVIDENC2_OutArgs *outArgs);

  REQ:
    struct dce_rpc_hdr hdr   -> 4
    codec_id                 -> 4
    codec                    -> 4
    reloc length             -> 1   (length/4)
    inArgs length            -> 1   (length/4)
    outBufs length           -> 1   (length/4)
    inBufs length            -> 1   (length/4)
    VIDENC2_OutArgs *outArgs -> 4   (pass by pointer)
    reloc table              -> ???
    VIDENC2_InArgs   inArgs  -> 12  (need inputID from userspace)
    XDM2_BufDesc     outBufs -> 24  (4 + 1 * 20)
    IVIDEO2_BufDesc  inBufs  -> 252
    -------------------------------
                              307

  RSP
    struct dce_rpc_hdr hdr   -> 4
    result                   -> 4
    inBufs length            -> 1   (length/4)
    XDAS_Int32 freeBufID[]   -> 4*n (n typically 0 or 2, but could be up to 20)
    -------------------------------
                               9-89
 */

static int codec_process(UInt32 size, UInt32 *data)
{
    MmType_Param   *payload = (MmType_Param *)data;
    Uint32          num_params = MmRpc_NUM_PARAMETERS(size);
    Uint32          codec_id = (Uint32) payload[0].data;
    Uint32          codec    = (Uint32) payload[1].data;
    void           *inBufs   = (void *) payload[2].data;
    void           *outBufs  = (void *) payload[3].data;
    void           *inArgs   = (void *) payload[4].data;
    void           *outArgs  = (void *) payload[5].data;
    Int32           ret = 0;

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG(">> codec_process codec=%p", codec);

    if( num_params != 6 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    dce_inv(inBufs);
    dce_inv(outBufs);
    dce_inv(inArgs);
    dce_inv(outArgs);

    DEBUG(">> codec=%p, inBufs=%p, outBufs=%p, inArgs=%p, outArgs=%p codec_id=%d LOCK sync_process_sem 0x%x",
        codec, inBufs, outBufs, inArgs, outArgs, codec_id, sync_process_sem);

#ifdef PSI_KPI
        kpi_before_codec();
#endif /*PSI_KPI*/
    ivahd_acquire();
    // do a reloc()
    ret = codec_fxns[codec_id].process((void *)codec, inBufs, outBufs, inArgs, outArgs);

    ivahd_release();

#ifdef PSI_KPI
        kpi_after_codec();
#endif /*PSI_KPI*/
    DEBUG("<< codec=%p ret=%d extendedError=%08x", codec, ret, ((VIDDEC3_OutArgs *)outArgs)->extendedError);

    dce_clean(inBufs);
    dce_clean(outBufs);
    dce_clean(inArgs);
    dce_clean(outArgs);

    Semaphore_post(sync_process_sem);

    return ((Int32)ret);
}

/*
  * codec delete
  */

static int codec_delete(UInt32 size, UInt32 *data)
{
    MmType_Param   *payload = (MmType_Param *)data;
    Uint32          num_params = MmRpc_NUM_PARAMETERS(size);
    Uint32          codec_id = (Uint32) payload[0].data;
    Uint32          codec    = (Uint32) payload[1].data;
    Uint32          mm_serv_id = 0;
    Client*          c;

#ifdef MEMORYSTATS_DEBUG
    Memory_Stats    stats;
#endif

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG(">> codec_delete on codec 0x%x", codec);

    if( num_params != 2 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    if ( codec_id == OMAP_DCE_VIDDEC3 ) {
        DEBUG("codec_delete for VIDDEC3 codec_handle 0x%x", codec);
        c = get_client_instance((Uint32) codec);
        int i;
        for (i = 0; i < DIM(c->decode_codec); i++ ) {
            if (c->decode_codec[i] == (VIDDEC3_Handle) codec) {
                if (c->decode_callback[i].row_mode) {
                    c->decode_callback[i].row_mode = 0;
                    pthread_mutex_destroy(&(c->decode_callback[i].callback_mutex));
                    pthread_cond_destroy(&(c->decode_callback[i].synch_callback));
                }
                DEBUG("codec_delete client 0x%x c->decode_callback[%d].callback_mutex 0x%x", c, i, c->decode_callback[i].callback_mutex);
            }
        }
    } else if ( codec_id == OMAP_DCE_VIDENC2 ) {
        DEBUG("codec_delete for VIDENC2 codec_handle 0x%x", codec);
        c = get_client_instance((Uint32) codec);
        int i;
        for (i = 0; i < DIM(c->encode_codec); i++ ) {
            if (c->encode_codec[i] == (VIDENC2_Handle) codec) {
                if (c->encode_callback[i].row_mode) {
                    c->encode_callback[i].row_mode = 0;
                    pthread_mutex_destroy(&(c->encode_callback[i].callback_mutex));
                    pthread_cond_destroy(&(c->encode_callback[i].synch_callback));
                }
                DEBUG("codec_delete client 0x%x c->encode_callback[%d].callback_mutex 0x%x", c, i, c->encode_callback[i].callback_mutex);
            }
        }
    }

    codec_fxns[codec_id].delete((void *)codec);

    mm_serv_id = MmServiceMgr_getId();
    DEBUG("codec_delete mm_serv_id 0x%x", mm_serv_id);
    dce_unregister_codec(codec_id, mm_serv_id, codec);

#ifdef MEMORYSTATS_DEBUG
    Memory_getStats(NULL, &stats);
    INFO("Total: %d\tFree: %d\tLargest: %d", stats.totalSize,
         stats.totalFreeSize, stats.largestFreeSize);
#endif

    DEBUG("<< codec_delete");

    Semaphore_post(sync_process_sem);

#ifdef PSI_KPI
        kpi_comp_deinit((void*)codec);
#endif /*PSI_KPI*/
    return (0);
}

/*
 * get_DataFxn : Sync/transfer the input data information from MPU side to DCE Server.
 * DCE Server will pass the information through the IVA-HD callback function:
 * H264E_GetDataFxn.
 */
static int get_DataFxn(UInt32 size, UInt32 *data)
{
    MmType_Param   *payload = (MmType_Param *)data;
    Uint32          num_params = MmRpc_NUM_PARAMETERS(size);
    XDM_DataSyncHandle          dataSyncHandle = (XDM_DataSyncHandle) payload[0].data;
    XDM_DataSyncDesc            *dataSyncDesc    = (void *) payload[1].data;
    Client* c;

    DEBUG(">> get_DataFxn dataSyncHandle 0x%x", dataSyncHandle);

    if( num_params != 2 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    dce_inv(dataSyncDesc);

    c = get_client_instance((Uint32) dataSyncHandle);
    if (c) {
        int i;
        for (i = 0; i < DIM(c->encode_codec); i++) {
            if (c->encode_codec[i] == dataSyncHandle) {
                pthread_mutex_lock(&(c->encode_callback[i].callback_mutex));
                c->encode_callback[i].dataSyncHandle = dataSyncHandle;
                c->encode_callback[i].dataSyncDesc = dataSyncDesc;

                if (c->encode_callback[i].codec_request) {
                    DEBUG("Case#1 get_DataFxn is received while codec has requested first. Signal the H264E_GetDataFxn to continue.");
                    pthread_cond_signal(&(c->encode_callback[i].synch_callback));
                } else {
                    // We received get_DataFxn from libdce/client, need to hold it until the data
                    // is consumed by codec through H264E_GetDataFxn before continuing. Otherwise
                    // we will receive another one since MPU side thinks that the data has been
                    // consumed by the codec.
                    c->encode_callback[i].getdata_ready = 1;
                    DEBUG("Case#2 get_DataFxn is received but codec has not request H264E_GetDataFxn. Wait conditionally.");
                    pthread_cond_wait(&(c->encode_callback[i].synch_callback), &(c->encode_callback[i].callback_mutex));
                    DEBUG("Case#2 get_DataFxn finally gets H264E_GetDataFxn, and the data has been consumed, continue.");
                    c->encode_callback[i].getdata_ready = 0;
                }
                pthread_mutex_unlock(&(c->encode_callback[i].callback_mutex));
            }
        }
    }

    dce_clean(dataSyncDesc);
    return (0);
}

/*
 * put_DataFxn : Sync/transfer the output data information from DCE Server to MPU side.
 * DCE Server will pass the information from the IVA-HD callback function:
 * H264D_PutDataFxn to MPU side.
 */
static int put_DataFxn(UInt32 size, UInt32 *data)
{
    MmType_Param   *payload = (MmType_Param *)data;
    Uint32          num_params = MmRpc_NUM_PARAMETERS(size);
    XDM_DataSyncHandle          dataSyncHandle = (XDM_DataSyncHandle) payload[0].data;
    XDM_DataSyncDesc            *dataSyncDesc    = (void *) payload[1].data;
    Client* c;

    DEBUG(">> put_DataFxn dataSyncHandle 0x%x dataSyncDesc 0x%x", dataSyncHandle, dataSyncDesc);

    if( num_params != 2 ) {
        ERROR("invalid number of params sent");
        return (-1);
    }

    dce_inv(dataSyncDesc);

    c = get_client_instance((Uint32) dataSyncHandle);
    if (c) {
        int i;
        for (i = 0; i < DIM(c->decode_codec); i++) {
            if (c->decode_codec[i] == dataSyncHandle) {
                pthread_mutex_lock(&(c->decode_callback[i].callback_mutex));
                // Found the corresponding entry, check if IVA-HD has already called the callback (codec_request == 1).

                if ((c->decode_callback[i].codec_request) && ((c->decode_callback[i].putdata_toclient) == 0)) {
                    DEBUG("Case#1 put_DataFxn is received while codec has requested first. c->decode_callback[%d].callback_mutex 0x%x",
                        i, c->decode_callback[i].callback_mutex);
                    DEBUG("Case#1 c->decode_callback[i].dataSyncDesc 0x%x dataSyncDesc 0x%x", c->decode_callback[i].dataSyncDesc, dataSyncDesc);
                    // Copy the local structure as H264D_PutDataFxn has already stored codec partial decoded data.
                    dataSyncDesc->size = (c->decode_callback[i].dataSyncDesc)->size;
                    dataSyncDesc->scatteredBlocksFlag = (c->decode_callback[i].dataSyncDesc)->scatteredBlocksFlag;
                    dataSyncDesc->baseAddr = (c->decode_callback[i].dataSyncDesc)->baseAddr;
                    dataSyncDesc->numBlocks = (c->decode_callback[i].dataSyncDesc)->numBlocks;
                    dataSyncDesc->varBlockSizesFlag = (c->decode_callback[i].dataSyncDesc)->varBlockSizesFlag;
                    dataSyncDesc->blockSizes = (c->decode_callback[i].dataSyncDesc)->blockSizes;

                    (c->decode_callback[i].putdata_toclient)++;
                    pthread_mutex_unlock(&(c->decode_callback[i].callback_mutex));

                    dce_clean(dataSyncDesc);
                    return (0);
                } else if ((c->decode_callback[i].codec_request) && (c->decode_callback[i].putdata_toclient)) {
                    // Check if put_data_toclient are set; Otherwise send the data to MPU.
                    DEBUG("MPU has received the earlier put_DataFxn from codec on 0x%x", c->decode_codec[i]);
                    (c->decode_callback[i].putdata_toclient)--;
                    DEBUG("Signal thread H264D_PutDataFxn to continue");
                    pthread_cond_signal(&(c->decode_callback[i].synch_callback));
                    // After signaling H264D_PutDataFxn, it needs to wait for the new data to be returned by codec.
                }

                // We received put_DataFxn from client, need to wait for data from codec through H264D_PutDataFxn before continuing
                c->decode_callback[i].putdata_ready = 1;
                DEBUG("Case#2 put_DataFxn is received. Wait for H264D_PutDataFxn to come; set putdata_ready = 1 c->decode_callback[%d].callback_mutex 0x%x", i, c->decode_callback[i].callback_mutex);
                DEBUG("Case#2 c->decode_callback[i].dataSyncDesc 0x%x dataSyncDesc 0x%x", c->decode_callback[i].dataSyncDesc, dataSyncDesc);
                pthread_cond_wait(&(c->decode_callback[i].synch_callback), &(c->decode_callback[i].callback_mutex));

                DEBUG("put_DataFxn FINALLY gets H264D_PutDataFxn, and the data has been provided, continue.");
                // Signal is received; check if it is from VIDDEC3_process or from H264D_PutDataFxn
                if (c->decode_callback[i].putData_endprocess ) {
                    dataSyncDesc->numBlocks = 0;  // To be returned to MPU side which will be ignored.
                } else {
                    // Copy the local structure as H264D_PutDataFxn has already stored codec partial decoded data to be sent to MPU side.
                    dataSyncDesc->size = (c->decode_callback[i].dataSyncDesc)->size;
                    dataSyncDesc->scatteredBlocksFlag = (c->decode_callback[i].dataSyncDesc)->scatteredBlocksFlag;
                    dataSyncDesc->baseAddr = (c->decode_callback[i].dataSyncDesc)->baseAddr;
                    dataSyncDesc->numBlocks = (c->decode_callback[i].dataSyncDesc)->numBlocks;
                    dataSyncDesc->varBlockSizesFlag = (c->decode_callback[i].dataSyncDesc)->varBlockSizesFlag;
                    dataSyncDesc->blockSizes = (c->decode_callback[i].dataSyncDesc)->blockSizes;
                }
                c->decode_callback[i].putdata_ready = 0;
                // After resetting putdata_ready to 0, this function will return to MPU
                DEBUG("From client dataSyncDesc->size %d ", dataSyncDesc->size);
                DEBUG("From client dataSyncDesc->scatteredBlocksFlag %d ", dataSyncDesc->scatteredBlocksFlag);
                DEBUG("From client dataSyncDesc->baseAddr %d ", dataSyncDesc->baseAddr);
                DEBUG("From client dataSyncDesc->numBlocks %d ", dataSyncDesc->numBlocks);
                DEBUG("From client dataSyncDesc->varBlockSizesFlag %d ", dataSyncDesc->varBlockSizesFlag);
                DEBUG("From client dataSyncDesc->blockSizes %d ", dataSyncDesc->blockSizes);

                (c->decode_callback[i].putdata_toclient)++;
                pthread_mutex_unlock(&(c->decode_callback[i].callback_mutex));
            }
        }
    }

    dce_clean(dataSyncDesc);
    return (0);
}

static int get_BufferFxn(UInt32 size, UInt32 *data)
{
    return (0);
}


/* the server create parameters, must be in persistent memory */
static RcmServer_Params    rpc_Params;

/* DCE Server skel function array */
static RcmServer_FxnDesc    DCEServerFxnAry[] =
{
    { "engine_open",     (RcmServer_MsgFxn) engine_open },
    { "engine_close",    (RcmServer_MsgFxn) engine_close },
    { "codec_create",    (RcmServer_MsgFxn) codec_create },
    { "codec_control",    (RcmServer_MsgFxn) codec_control },
    { "codec_get_version",    (RcmServer_MsgFxn) codec_get_version },
    { "codec_process",   (RcmServer_MsgFxn) codec_process },
    { "codec_delete",    (RcmServer_MsgFxn) codec_delete },
    { "get_rproc_info", (RcmServer_MsgFxn) get_rproc_info }

};

/* DCE Server skel function table */
#define DCEServerFxnAryLen (sizeof(DCEServerFxnAry) / sizeof(DCEServerFxnAry[0]))

static const RcmServer_FxnDescAry    DCEServer_fxnTab =
{
    DCEServerFxnAryLen,
    DCEServerFxnAry
};

static MmType_FxnSig    DCEServer_sigAry[] =
{
    { "engine_open", 2,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 }
      } },
    { "engine_close", 2,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 }
      } },
    { "codec_create", 5,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_PtrType(MmType_Param_VOID), 1 },
          { MmType_Dir_In, MmType_PtrType(MmType_Param_VOID), 1 }
      } },
    { "codec_control", 6,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_PtrType(MmType_Param_VOID), 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 }
      } },
    { "codec_get_version", 5,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_PtrType(MmType_Param_VOID), 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 }
      } },
    { "codec_process", 7,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 }
      } },
    { "codec_delete", 3,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_In, MmType_Param_U32, 1 }
      } },
    { "get_rproc_info", 2,
      {
         { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 }
      } }

};

static MmType_FxnSigTab    dce_fxnSigTab =
{
    MmType_NumElem(DCEServer_sigAry), DCEServer_sigAry
};

/* DCE Callback Server skel function array */
static RcmServer_FxnDesc    DCECallbackServerFxnAry[] =
{
    { "get_DataFxn",     (RcmServer_MsgFxn) get_DataFxn },
    { "put_DataFxn",     (RcmServer_MsgFxn) put_DataFxn },
    { "get_BufferFxn",   (RcmServer_MsgFxn) get_BufferFxn }
};

#define DCECallbackServerFxnAryLen (sizeof(DCECallbackServerFxnAry) / sizeof(DCECallbackServerFxnAry[0]))

static const RcmServer_FxnDescAry    DCECallbackServer_fxnTab =
{
    DCECallbackServerFxnAryLen,
    DCECallbackServerFxnAry
};

static MmType_FxnSig    DCECallbackServer_sigAry[] =
{
    { "get_DataFxn", 3,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 }
      } },
    { "put_DataFxn", 3,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 }
      } },
    { "get_BufferFxn", 3,
      {
          { MmType_Dir_Out, MmType_Param_S32, 1 }, // return
          { MmType_Dir_In, MmType_Param_U32, 1 },
          { MmType_Dir_Bi, MmType_PtrType(MmType_Param_VOID), 1 }
      } }
};

static MmType_FxnSigTab    dce_callback_fxnSigTab =
{
    MmType_NumElem(DCECallbackServer_sigAry), DCECallbackServer_sigAry
};

Void dce_SrvDelNotification(Void)
{
    Client *c;
    int i;
    uint32_t mm_serv_id = 0;

    Semaphore_pend(sync_process_sem, BIOS_WAIT_FOREVER);

    DEBUG("dce_SrvDelNotification: cleanup existing codec and engine\n");

    mm_serv_id = MmServiceMgr_getId();
    DEBUG("cleanup: mm_serv_id=0x%x", mm_serv_id);

    c = get_client(mm_serv_id);
    if( c ) {
        DEBUG("cleanup: mm_serv_id=0x%x c=%p c->refs=%d", mm_serv_id, c, c->refs);

        /* For low latency instance, need to trigger the flag to callback function so that it will return full numblock*/
        for( i = 0; i < DIM(c->decode_codec); i++ ) {
            if (c->decode_callback[i].row_mode) {
                DEBUG("Setting c->decode_callback[%d].mpu_crash_indication = TRUE c->decode_codec[%d].callback_mutex 0x%x", i, i, c->decode_callback[i].callback_mutex);
                c->decode_callback[i].mpu_crash_indication = TRUE;
                pthread_cond_signal(&(c->decode_callback[i].synch_callback));
            }
        }

        for( i = 0; i < DIM(c->encode_codec); i++ ) {
            if (c->encode_callback[i].row_mode) {
                DEBUG("Setting c->encode_callback[%d].mpu_crash_indication = TRUE c->encode_codec[%d].callback_mutex 0x%x", i, i, c->encode_callback[i].callback_mutex);
                c->encode_callback[i].mpu_crash_indication = TRUE;
                pthread_cond_signal(&(c->encode_callback[i].synch_callback));
            }
        }

        /* Make sure IVAHD and SL2 are idle before proceeding */
        ivahd_idle_check();

        /* delete all codecs first */
        for( i = 0; i < DIM(c->decode_codec); i++ ) {
            DEBUG("dce_SrvDelNotification: test c->decode_codec[%d] 0x%x", i, c->decode_codec[i]);
            if( c->decode_codec[i] ) {
                DEBUG("dce_SrvDelNotification: delete decoder codec handle 0x%x c->decode_callback[i] 0x%x c->decode_callback[i].row_mode %d\n",
                    c->decode_codec[i], c->decode_callback[i], c->decode_callback[i].row_mode);
                if (c->decode_callback[i].row_mode) {
                    pthread_mutex_destroy(&(c->decode_callback[i].callback_mutex));
                    pthread_cond_destroy(&(c->decode_callback[i].synch_callback));
                    c->decode_callback[i].row_mode = 0;
                    c->decode_callback[i].mpu_crash_indication = FALSE;
                }
                codec_fxns[OMAP_DCE_VIDDEC3].delete((void *)c->decode_codec[i]);
                c->decode_codec[i] = NULL;
            }
        }

        for( i = 0; i < DIM(c->encode_codec); i++ ) {
            DEBUG("dce_SrvDelNotification: test c->encode_codec[%d] 0x%x", i, c->encode_codec[i]);
            if( c->encode_codec[i] ) {
                DEBUG("dce_SrvDelNotification: delete encoder codec handle 0x%x c->encode_callback[i] 0x%x \n",
                    c->encode_codec[i], c->encode_callback[i]);
                if (c->encode_callback[i].row_mode) {
                    pthread_mutex_destroy(&(c->encode_callback[i].callback_mutex));
                    pthread_cond_destroy(&(c->encode_callback[i].synch_callback));
                    c->encode_callback[i].row_mode = 0;
                    c->encode_callback[i].mpu_crash_indication = FALSE;
                }
                codec_fxns[OMAP_DCE_VIDENC2].delete((void *)c->encode_codec[i]);
                c->encode_codec[i] = NULL;
            }
        }

        /* and lastly close all engines */
        for( i = 0; i < DIM(c->engines); i++ ) {
            if( c->engines[i] ) {
                DEBUG("dce_SrvDelNotification: delete Engine handle 0x%x\n", c->engines[i]);
                Engine_close(c->engines[i]);
                c->engines[i] = NULL;
                DEBUG("dce_SrvDelNotification engine_close: %p refs=%d", c, c->refs);
                c->refs--;
            }
        }

        if( !c->refs ) {
            c->mm_serv_id = NULL;
        }
    }
    DEBUG("dce_SrvDelNotification: COMPLETE exit function \n");

    Semaphore_post(sync_process_sem);
}

Void dceCallback_SrvDelNotification(Void)
{
    System_printf("dceCallback_SrvDelNotification - call dce_SrvDelNotification to clean up\n");
}

/*
 * dce_main : main function for dce-server thread.
 * Registering to MmServiceMgr.
 */
static void dce_main(uint32_t arg0, uint32_t arg1)
{
    int            err = 0;
    dce_connect    dce_connect_msg;

    /* Read the register for ID_CODE to figure out the correct configuration: */
    /* CONTROL_STD_FUSE_ID_CODE[31:0] ID_CODE STD_FUSE_IDCODE */
    /* physical address: 0x4A00 2204 Address offset: 0x204                       */
#ifdef OMAP5430_ES10
    dce_connect_msg.chipset_id = 0x5430;
#elif OMAP5432_ES20
    dce_connect_msg.chipset_id = 0x5432;
#elif OMAP4430_ES20
    dce_connect_msg.chipset_id = 0x4430;
#elif OMAP4460_ES20
    dce_connect_msg.chipset_id = 0x4460;
#elif VAYU_ES10
    dce_connect_msg.chipset_id = 0x5436;
#endif
    dce_connect_msg.debug = dce_debug;
    connect(&dce_connect_msg);

    err = MmServiceMgr_init();  // MmServiceMgr_init() will always return MmServiceMgr_S_SUCCESS.

    // setup the RCM Server create params
    RcmServer_Params_init(&rpc_Params);
    rpc_Params.priority = Thread_Priority_ABOVE_NORMAL;
    rpc_Params.stackSize = 0x1000;
    rpc_Params.fxns.length = DCEServer_fxnTab.length;
    rpc_Params.fxns.elem = DCEServer_fxnTab.elem;

    DEBUG("REGISTER %s\n", SERVER_NAME);

    // Get the Service Manager handle
    err = MmServiceMgr_register(SERVER_NAME, &rpc_Params, &dce_fxnSigTab, dce_SrvDelNotification);
    if( err < 0 ) {
        DEBUG("failed to start " SERVER_NAME " \n");
    } else {
        DEBUG(SERVER_NAME " running through MmServiceMgr");
    }

    MmServiceMgr_exit();

    DEBUG("deleted " SERVER_NAME);

    return;
}

/*
 * dce_callback_main : main function for dce-callback-server thread.
 * Registering to MmServiceMgr.
 */

static void dce_callback_main(uint32_t arg0, uint32_t arg1)
{
    int            err = 0;

    err = MmServiceMgr_init();  // MmServiceMgr_init() will always return MmServiceMgr_S_SUCCESS.

    // setup the RCM Server create params
    RcmServer_Params_init(&rpc_Params);
    rpc_Params.priority = Thread_Priority_ABOVE_NORMAL;
    rpc_Params.stackSize = 0x1000;
    rpc_Params.fxns.length = DCECallbackServer_fxnTab.length;
    rpc_Params.fxns.elem = DCECallbackServer_fxnTab.elem;

    DEBUG("REGISTER %s\n", CALLBACK_SERVER_NAME);

    // Get the Service Manager handle
    err = MmServiceMgr_register(CALLBACK_SERVER_NAME, &rpc_Params, &dce_callback_fxnSigTab, dceCallback_SrvDelNotification);
    if( err < 0 ) {
        DEBUG("failed to start " CALLBACK_SERVER_NAME " \n");
    } else {
        DEBUG(CALLBACK_SERVER_NAME " running through MmServiceMgr");
    }

    MmServiceMgr_exit();

    DEBUG("deleted " CALLBACK_SERVER_NAME);

    return;
}


/*
  * dce init : Startup Function
  */
Bool dce_init(void)
{
    Task_Params    params;
    Task_Params    callback_params;
    Semaphore_Params semParams;

    INFO("Creating DCE server and DCE callback server thread...");

    /* Create DCE task. */
    Task_Params_init(&params);
    params.instance->name = "dce-server";
    params.priority = Thread_Priority_ABOVE_NORMAL;
    Task_create(dce_main, &params, NULL);

    /* Create DCE callback task. */
    Task_Params_init(&callback_params);
    callback_params.instance->name = "dce-callback-server";
    callback_params.priority = Thread_Priority_ABOVE_NORMAL;
    Task_create(dce_callback_main, &callback_params, NULL);

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    sync_process_sem = Semaphore_create(1, &semParams, NULL);

    return (TRUE);
}

/*
  * dce deinit
  */

void dce_deinit(void)
{
    DEBUG("dce_deinit");

    Semaphore_delete(&sync_process_sem);
}

/*
 * H264E_GetDataFxn
 * This is callback function provided for IVA-HD codec to callback for more input data.
 * This function syncs with get_DataFxn which handles the required data from the MPU side.
 * It will continue if DCE Server has received get_DataFxn from MPU side otherwise it will
 * wait until DCE server receive get_DataFxn.
 */
XDM_DataSyncGetFxn H264E_GetDataFxn(XDM_DataSyncHandle dataSyncHandle,
    XDM_DataSyncDesc *dataSyncDesc)
{
    Client* c;

    dce_inv(dataSyncDesc);

    DEBUG("********************H264E_GetDataFxn START*************************** dataSyncHandle 0x%x",
        dataSyncHandle);
    c = get_client_instance((Uint32) dataSyncHandle);
    if (c) {
        int i;
        for (i = 0; i < DIM(c->encode_codec); i++) {
            if (c->encode_codec[i] == dataSyncHandle) {
                pthread_mutex_lock(&(c->encode_callback[i].callback_mutex));
                DEBUG("H264E_GetDataFxn dataSyncHandle 0x%x c->encode_callback[%d].callback_mutex 0x%x", dataSyncHandle, i, c->encode_callback[i].callback_mutex);
                // Check if H264E_GetDataFxn from codec and get_DataFxn from MPU side. Which comes first.
                // Check if MPU has crashed c->encode_callback[i].mpu_crash_indication, if it is then send the highest numBlock to codec so that VIDENC2_process will be returned and IVA back to IDLE.
                if (c->encode_callback[i].mpu_crash_indication) {
                    // Since MPU has crashed, need to send the numBlocks to codec so that VIDENC2_process can be returned and IVA back to IDLE.
                    // Send the numBlocks as expected max value or add functionality to count up to proper numBlocks to be returned to codec.
                    // Current implementation will send arbitrary value 100 numBlocks (height resolution 1600) which will let codec return the process call and move IVA into IDLE state.
                    DEBUG("MPU has crashed, send arbitrary numBlocks 100 so that VIDENC2_process will be returned");
                    dataSyncDesc->size = sizeof(XDM_DataSyncDesc);
                    dataSyncDesc->scatteredBlocksFlag = 0;
                    dataSyncDesc->baseAddr = 0;
                    dataSyncDesc->numBlocks = 100;
                    dataSyncDesc->varBlockSizesFlag = 0;
                    dataSyncDesc->blockSizes = 0;
                } else {
                    if (c->encode_callback[i].getdata_ready) {
                        // Already have data to be passed to codec
                        DEBUG("H264E_GetDataFxn Case#2 c->encode_callback[%d].dataSyncHandle 0x%x c->encode_callback[%d].dataSyncDesc 0x%x",
                            i, c->encode_callback[i].dataSyncHandle, i, c->encode_callback[i].dataSyncDesc);
                        if (dataSyncHandle == c->encode_callback[i].dataSyncHandle) {
                            dataSyncDesc->size = (c->encode_callback[i].dataSyncDesc)->size;
                            dataSyncDesc->scatteredBlocksFlag = (c->encode_callback[i].dataSyncDesc)->scatteredBlocksFlag;
                            dataSyncDesc->baseAddr = (c->encode_callback[i].dataSyncDesc)->baseAddr;
                            dataSyncDesc->numBlocks = (c->encode_callback[i].dataSyncDesc)->numBlocks;
                            dataSyncDesc->varBlockSizesFlag = (c->encode_callback[i].dataSyncDesc)->varBlockSizesFlag;
                            dataSyncDesc->blockSizes = (c->encode_callback[i].dataSyncDesc)->blockSizes;
                        }

                        // Order get_DataFxn to continue request to MPU client side for more data as it is currently pending.
                        pthread_cond_signal(&(c->encode_callback[i].synch_callback));
                    } else {
                        c->encode_callback[i].codec_request = 1;

                        DEBUG("H264E_GetDataFxn wait Case#1");
                        // Wait until get_DataFxn is received from MPU client side. Need the information to be passed to codec as currently not available.
                        pthread_cond_wait(&(c->encode_callback[i].synch_callback), &(c->encode_callback[i].callback_mutex));

                        // Once get_DataFxn is received from MPU side continue by providing it to IVA-HD codec.
                        DEBUG("H264E_GetDataFxn Case#1 c->encode_callback[%d].dataSyncHandle 0x%x c->encode_callback[%d].dataSyncDesc 0x%x",
                            i, c->encode_callback[i].dataSyncHandle, i, c->encode_callback[i].dataSyncDesc);
                        if (dataSyncHandle == c->encode_callback[i].dataSyncHandle) {
                            dataSyncDesc->size = (c->encode_callback[i].dataSyncDesc)->size;
                            dataSyncDesc->scatteredBlocksFlag = (c->encode_callback[i].dataSyncDesc)->scatteredBlocksFlag;
                            dataSyncDesc->baseAddr = (c->encode_callback[i].dataSyncDesc)->baseAddr;
                            dataSyncDesc->numBlocks = (c->encode_callback[i].dataSyncDesc)->numBlocks;
                            dataSyncDesc->varBlockSizesFlag = (c->encode_callback[i].dataSyncDesc)->varBlockSizesFlag;
                            dataSyncDesc->blockSizes = (c->encode_callback[i].dataSyncDesc)->blockSizes;
                        }
                        c->encode_callback[i].codec_request = 0;
                    }
                }
                pthread_mutex_unlock(&(c->encode_callback[i].callback_mutex));
            }
        }
    }

    DEBUG("********************H264E_GetDataFxn END*************************** dataSyncHandle 0x%x", dataSyncHandle);
    dce_clean(dataSyncDesc);
    return (0);
}

/*
 * H264D_PutDataFxn
 * This is callback function provided for IVA-HD codec to callback for notifying client on partial decoded output.
 * This function syncs with put_DataFxn which handles passing the data from codec to the MPU side.
 * Once this callback is received from IVA-HD codec, it will save the codec partial decoded data into local structure
 * for put_DataFxn to pick up.
 * If DCE server has already receive put_DataFxn (putdata_ready == 1), then it notifies through semaphore_post to continue.
 * If DCE server is waiting for put_DataFxn, then it will set codec_request = 1, and wait through semaphore_pend.
 */
XDM_DataSyncPutFxn H264D_PutDataFxn(XDM_DataSyncHandle dataSyncHandle,
    XDM_DataSyncDesc *dataSyncDesc)
{
    Client* c;

    dce_inv(dataSyncDesc);
    DEBUG("********************H264D_PutDataFxn START*************************** dataSyncHandle 0x%x dataSyncDesc->numBlocks %d",
        dataSyncHandle, dataSyncDesc->numBlocks);

    c = get_client_instance((Uint32) dataSyncHandle);
    if (c) {
        int i;
        for (i = 0; i < DIM(c->decode_codec); i++) {
            if (c->decode_codec[i] == dataSyncHandle) {
                pthread_mutex_lock(&(c->decode_callback[i].callback_mutex));
                DEBUG("H264D_PutDataFxn dataSyncHandle 0x%x c->decode_codec[%d].callback_mutex 0x%x", dataSyncHandle, i, c->decode_callback[i].callback_mutex);
                c->decode_callback[i].codec_request = 1;
                // Save the data from IVA-HD codec into local structure for put_DataFxn to pick up.
                if (dataSyncHandle == c->decode_callback[i].dataSyncHandle) {
                    (c->decode_callback[i].dataSyncDesc)->size = dataSyncDesc->size;
                    (c->decode_callback[i].dataSyncDesc)->scatteredBlocksFlag = dataSyncDesc->scatteredBlocksFlag;
                    (c->decode_callback[i].dataSyncDesc)->baseAddr = dataSyncDesc->baseAddr;
                    (c->decode_callback[i].dataSyncDesc)->numBlocks = dataSyncDesc->numBlocks;
                    (c->decode_callback[i].dataSyncDesc)->varBlockSizesFlag = dataSyncDesc->varBlockSizesFlag;
                    (c->decode_callback[i].dataSyncDesc)->blockSizes = dataSyncDesc->blockSizes;
                    DEBUG("From Codec (c->decode_callback[%d].dataSyncDesc)->size %d ", i, (c->decode_callback[i].dataSyncDesc)->size);
                    DEBUG("From Codec (c->decode_callback[%d].dataSyncDesc)->scatteredBlocksFlag %d ", i, (c->decode_callback[i].dataSyncDesc)->scatteredBlocksFlag);
                    DEBUG("From Codec (c->decode_callback[%d].dataSyncDesc)->baseAddr %d ", i, (c->decode_callback[i].dataSyncDesc)->baseAddr);
                    DEBUG("From Codec (c->decode_callback[%d].dataSyncDesc)->numBlocks %d ", i, (c->decode_callback[i].dataSyncDesc)->numBlocks);
                    DEBUG("From Codec (c->decode_callback[%d].dataSyncDesc)->varBlockSizesFlag %d ", i, (c->decode_callback[i].dataSyncDesc)->varBlockSizesFlag);
                    DEBUG("From Codec (c->decode_callback[%d].dataSyncDesc)->blockSizes %d ", i, (c->decode_callback[i].dataSyncDesc)->blockSizes);
                }

                // If MPU has crashed, there is no way MPU will respond after this. Let codec thinking that MPU has received the numblock. Don't wait, just returned.
                if (!(c->decode_callback[i].mpu_crash_indication)) {
                    // MPU is alive. Wait until the put_DataFxn is received from MPU side. Codec has callback with partial decoded data but put_DataFxn is not received yet.
                    // Check if put_DataFxn from MPU side has come before H264D_PutDataFxn.
                    if (c->decode_callback[i].putdata_ready) {
                        // Already receive request from client for partial decoded output information from codec.
                        // Order put_DataFxn to continue sending codec partial decoded data in the local structure to MPU side.
                        DEBUG("H264D_PutDataFxn Case#2 c->decode_callback[%d].dataSyncHandle 0x%x c->decode_callback[%d].dataSyncDesc 0x%x",
                            i, c->decode_callback[i].dataSyncHandle, i, c->decode_callback[i].dataSyncDesc);

                        // After storing the data from codec, send thread signal conditional for put_DataFxn to continue passing the codec data to MPU.
                        pthread_cond_signal(&(c->decode_callback[i].synch_callback));

                        DEBUG("H264D_PutDataFxn Case#2 wait on pthread_cond_wait");
                        // After signal put_DataFxn, needs to wait for put_DataFxn return from A15 before returning to codec.
                        pthread_cond_wait(&(c->decode_callback[i].synch_callback), &(c->decode_callback[i].callback_mutex));
                    } else {
                        DEBUG("H264D_PutDataFxn Case#1 wait on pthread_cond_wait c->decode_codec[%d].callback_mutex 0x%x", i, c->decode_callback[i].callback_mutex);
                        pthread_cond_wait(&(c->decode_callback[i].synch_callback), &(c->decode_callback[i].callback_mutex));
                    }
                }
                c->decode_callback[i].codec_request = 0;
                pthread_mutex_unlock(&(c->decode_callback[i].callback_mutex));
            }
        }
    }

    DEBUG("********************H264D_PutDataFxn END*************************** dataSyncHandle 0x%x", dataSyncHandle);
    dce_clean(dataSyncDesc);
    return (0);
}
