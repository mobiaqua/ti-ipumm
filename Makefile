#/*
# * Copyright (c) 2011-2015, Texas Instruments Incorporated
# * All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions
# * are met:
# *
# * *  Redistributions of source code must retain the above copyright
# *    notice, this list of conditions and the following disclaimer.
# *
# * *  Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the
# *    documentation and/or other materials provided with the distribution.
# *
# * *  Neither the name of Texas Instruments Incorporated nor the names of
# *    its contributors may be used to endorse or promote products derived
# *    from this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

# Repo
BIOSTOOLSROOT	?= /opt/ti
REPO		:= $(BIOSTOOLSROOT)

# Different tool versions can easily be programmed by defining below variables
# in your environment.
XDCVERSION	?= xdctools_3_31_02_38_core
BIOSVERSION	?= bios_6_42_02_29
IPCVERSION	?= ipc_3_40_01_08
CEVERSION	?= codec_engine_3_24_00_08
FCVERSION	?= framework_components_3_40_01_04
XDAISVERSION	?= xdais_7_24_00_04

# TI Compiler Settings
export TMS470CGTOOLPATH ?= $(BIOSTOOLSROOT)/ccsv6/tools/compiler/ti-cgt-arm_5.2.5

# Define where the sources are
DUCATIDCEMMSRC	= $(shell pwd)
TIVIDEOTOOLSROOT ?= $(BIOSTOOLSROOT)

# Generate the full package paths for tools
BIOSPROD	= $(REPO)/$(BIOSVERSION)
CEPROD		= $(TIVIDEOTOOLSROOT)/$(CEVERSION)
FCPROD		= $(TIVIDEOTOOLSROOT)/$(FCVERSION)
XDAISPROD	= $(REPO)/$(XDAISVERSION)
CODECSPROD	= $(CODECSPATH)

# XDC settings
export XDCBUILDCFG = $(DUCATIDCEMMSRC)/build/config.bld

XDCDIST_TREE	= $(REPO)/$(XDCVERSION)
export XDCROOT	= $(XDCDIST_TREE)

export XDCPATH  = $(BIOSPROD)/packages;$(IPCSRC)/packages;$(RPMSGSRC)/src;$(CEPROD)/packages;$(FCPROD)/packages;$(XDAISPROD)/packages;/$(CODECSPROD)/extrel/ti/ivahd_codecs/packages;$(DUCATIDCEMMSRC)/src;

ti.targets.C64P ?=
ti.targets.C64T ?=
ti.targets.C674 ?=

ti.targets.elf.C64P ?=
ti.targets.elf.C64T ?=
ti.targets.elf.C66 ?=
ti.targets.elf.C674 ?=

ti.targets.arm.elf.M3 ?=
ti.targets.arm.elf.M4 ?=
ti.targets.arm.elf.M4F ?=

gnu.targets.arm.M3 ?=
gnu.targets.arm.M4 ?=
gnu.targets.arm.M4F ?=
gnu.targets.arm.A15 ?=

XDCARGS= profile=$(PROFILE) \
    trace_level=$(TRACELEVEL) \
    hw_type=$(HW_TYPE) \
    hw_version=$(HW_VER) \
    BIOS_type=$(BIOS_TYPE) \
    ti.targets.C64P=\"$(ti.targets.C64P)\" \
    ti.targets.C64T=\"$(ti.targets.C64T)\" \
    ti.targets.arm.elf.M3=\"$(ti.targets.arm.elf.M3)\" \
    ti.targets.arm.elf.M4=\"$(ti.targets.arm.elf.M4)\" \
    ti.targets.arm.elf.M4F=\"$(ti.targets.arm.elf.M4F)\" \
    ti.targets.elf.C64P=\"$(ti.targets.elf.C64P)\" \
    ti.targets.elf.C64T=\"$(ti.targets.elf.C64T)\" \
    ti.targets.elf.C66=\"$(ti.targets.elf.C66)\" \
    ti.targets.elf.C674=\"$(ti.targets.elf.C674)\" \
    gnu.targets.arm.M3=\"$(gnu.targets.arm.M3)\" \
    gnu.targets.arm.M4=\"$(gnu.targets.arm.M4)\" \
    gnu.targets.arm.M4F=\"$(gnu.targets.arm.M4F)\"

# Custom settings for build
JOBS		?= 1
# Set profile, always set as release version. Alternate option is "debug"
PROFILE		?= release
# Set debug/trace level from 0 to 4
TRACELEVEL	?= 0
# Offloads core to sysm3 code
OFFLOAD		?= 1
# Set to Non-SMP by default
FORSMP		?= 1
# Set Instrumentation to be allowed (ENABLE to enable it)
SETINST		?= ENABLE
# Set HW revision type- OMAP5:ES20, VAYU:ES10
HWVERSION   ?= ES10
# Set if Profiler needs to ON or OFF for the build
PROFILER    ?= DISABLE

all:
	$(XDCROOT)/xdc XDCARGS="$(XDCARGS)" XDCBUILDCFG=./build/config.bld --jobs=$(JOBS) -PD $(DUCATIDCEMMSRC)/platform/ti/dce/baselib/.
	$(XDCROOT)/xdc XDCARGS="$(XDCARGS)" XDCBUILDCFG=./build/config.bld --jobs=$(JOBS) -PD $(DUCATIDCEMMSRC)/platform/ti/dce/baseimage/.
	cp $(DUCATIDCEMMSRC)/platform/ti/dce/baseimage/out/ipu/$(PROFILE)/ipu.xem3 omap4-ipu-fw-symbols.xem3
	$(ARMTOOLCHAINPATH)/bin/armstrip -p $(DUCATIDCEMMSRC)/platform/ti/dce/baseimage/out/ipu/$(PROFILE)/ipu.xem3 -o=omap4-ipu-fw.xem3

clean:
	$(XDCROOT)/xdc XDCARGS="$(XDCARGS)" XDCBUILDCFG=./build/config.bld --jobs=$(JOBS) clean -PD $(DUCATIDCEMMSRC)/platform/ti/dce/baselib/.
	$(XDCROOT)/xdc XDCARGS="$(XDCARGS)" XDCBUILDCFG=./build/config.bld --jobs=$(JOBS) clean -PD $(DUCATIDCEMMSRC)/platform/ti/dce/baseimage/.
