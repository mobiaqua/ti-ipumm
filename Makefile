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
CODECSPROD	= $(COEDECSPATH)

# XDC settings
export XDCBUILDCFG = $(DUCATIDCEMMSRC)/build/config.bld

XDCDIST_TREE	= $(REPO)/$(XDCVERSION)
export XDCROOT	= $(XDCDIST_TREE)

export XDCPATH  = $(BIOSPROD)/packages;$(IPCSRC)/packages;$(RPMSGSRC)/src;$(CEPROD)/packages;$(FCPROD)/packages;$(XDAISPROD)/packages;/$(CODECSPROD)/extrel/ti/ivahd_codecs/packages;$(DUCATIDCEMMSRC)/src;

# Custom settings for build
JOBS		?= 1
# Set profile, always set as release version. Alternate option is "debug"
PROFILE		?= release
# Set debug/trace level from 0 to 4
TRACELEVEL	?= 0
# Offloads core to sysm3 code
OFFLOAD		?= 1
# Set to Non-SMP by default
FORSMP		?= 0
# Set Instrumentation to be allowed (ENABLE to enable it)
SETINST		?= ENABLE
# Set if Profiler needs to ON or OFF for the build
PROFILER    ?= DISABLE

all: ducatibin

# Include platform build configuration
config:
ifeq (bldcfg.mk,$(wildcard bldcfg.mk))
include bldcfg.mk
else
	@echo "No config selected. Please configure the build first and then try to build."
	@echo "For more info, use 'make help'"
	@exit 1
endif

unconfig:
	@echo "Removed existing configuration"
	@rm -f bldcfg.mk

omap4_smp_config: unconfig
	@echo "Creating new config\c"
	@echo DUCATI_CONFIG = omap4_smp_config > bldcfg.mk
	@echo ".\c"
	@echo MYXDCARGS=\"profile=$(PROFILE) trace_level=$(TRACELEVEL) prof_type=$(PROFILER)\" >> bldcfg.mk
	@echo ".\c"
	@echo FORSMP = 1 >> bldcfg.mk
	@echo ".\c"
	@echo DUCATIBINNAME = "ducati-m3-ipu.xem3" >> bldcfg.mk
	@echo INTBINNAME = "ipu.xem3" >> bldcfg.mk
	@echo ".\c"
	@echo "done"

clean: config
	export XDCARGS=$(MYXDCARGS); \
	 $(XDCROOT)/xdc --jobs=$(JOBS) clean -PD $(DUCATIDCEMMSRC)/platform/ti/dce/baseimage/.

build: config
ifeq ($(IPCSRC),)
	@echo "ERROR: IPCSRC not set. Exiting..."
	@echo "For more info, use 'make help'"
	@exit 1
else ifeq ($(TMS470CGTOOLPATH),)
	@echo "ERROR: TMS470CGTOOLPATH not set. Exiting..."
	@echo "For more info, use 'make help'"
	@exit 1
endif
	export XDCARGS=$(MYXDCARGS); \
	$(XDCROOT)/xdc --jobs=$(JOBS) -PD $(DUCATIDCEMMSRC)/platform/ti/dce/baseimage/.

ducatibin: build
	$(TMS470CGTOOLPATH)/bin/armstrip -p $(DUCATIDCEMMSRC)/platform/ti/dce/baseimage/out/ipu/$(PROFILE)/$(INTBINNAME) -o=$(DUCATIBINNAME)
