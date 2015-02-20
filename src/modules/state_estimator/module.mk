############################################################################
#
#   Copyright (C) 2012 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# Makefile to build the state estimator
#

MODULE_COMMAND		= state_estimator
MODULE_STACKSIZE	= 4096
#MODULE_PRIORITY		= SCHED_PRIORITY_MAX-30
#MAXOPTIMIZATION		= -O0

SRCS		 = state_estimator.c 													\
			state_estimator_params.c 												\
			codegen_state_estimator/initStates.c									\
			codegen_state_estimator/diag.c											\
			codegen_state_estimator/mldivide.c										\
			codegen_state_estimator/norm.c											\
			codegen_state_estimator/propagate.c 									\
			codegen_state_estimator/HyEst_initialize.c								\
			codegen_state_estimator/HyEst_rtwutil.c									\
			codegen_state_estimator/HyEst_terminate.c								\
			codegen_state_estimator/rdivide.c										\
			codegen_state_estimator/rt_nonfinite.c									\
			codegen_state_estimator/rtGetInf.c										\
			codegen_state_estimator/rtGetNaN.c										\
			codegen_state_estimator/updateCompass2.c								\
			codegen_state_estimator/updatePosition.c								\
			codegen_state_estimator/updatePressures.c								\
			codegen_state_estimator/updatePressures_all.c							\
			codegen_state_estimator/updateVelNed.c									\
			codegen_state_estimator/quat2rpy.c										\
			codegen_state_estimator/getAirplane.c									\
			codegen_state_estimator/pinv.c											\
			codegen_state_estimator/autogen_TasUpdate_withAerodynamics.c			\
			codegen_state_estimator/HyEst_emxutil.c									\
			codegen_state_estimator/dot.c											\
			codegen_state_estimator/HyEst_data.c									\
			codegen_state_estimator/magField.c										\
			codegen_state_estimator/cross.c											\
			codegen_state_estimator/inv.c											\
			codegen_state_estimator/mpower.c										\
			codegen_state_estimator/autogen_FPF_const_wind.c						\
			codegen_state_estimator/autogen_FPF.c									\
			codegen_state_estimator/autogen_pressure2.c								\
			codegen_state_estimator/autogen_TasUpdate_withAerodynamics_alpha_set.c
			    			
# XXX this is *horribly* broken
INCLUDE_DIRS		+= $(TOPDIR)/../mavlink/include/mavlink

EXTRACFLAGS = -Wno-float-equal -Wframe-larger-than=4096
#include $(APPDIR)/mk/app.mk