
#
# Copyright (c) 2004 
#      Autonomous Systems Lab, Swiss Federal Institute of Technology.
# All rights reserved.
#
# Redistribution  and  use in source   and binary forms,  with or without
# modification, are permitted provided that  the following conditions are
# met:
#
#   1. Redistributions  of  source code must  retain  the above copyright
#      notice, this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must  reproduce the above copyright
#      notice,  this list of  conditions and  the following disclaimer in
#      the  documentation   and/or  other  materials   provided with  the
#      distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE  AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY  EXPRESS OR IMPLIED WARRANTIES, INCLUDING,  BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES   OF MERCHANTABILITY AND  FITNESS  FOR  A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO  EVENT SHALL THE AUTHOR OR  CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING,  BUT  NOT LIMITED TO, PROCUREMENT  OF
# SUBSTITUTE  GOODS OR SERVICES;  LOSS   OF  USE,  DATA, OR PROFITS;   OR
# BUSINESS  INTERRUPTION) HOWEVER CAUSED AND  ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE  USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# --- RTAI-specific configuration ---------------------------------------

ifeq (powerpc,$(host_cpu))
CFLAGS+= -fsigned-char -ffixed-r2 -mmultiple -mstring
endif

CPPFLAGS += -D__RTAI__ -D__KERNEL__ -DMODULE
CPPFLAGS += -I$(RTAI_INC) -I$(KERNEL_INC)

# produce relocatable objects
BINEXT=.o
LIBEXT=.o
LIBTOOL_COPT=-prefer-non-pic
LIBTOOL_LINKOPT=-static

LIBTOOL_LINKPORTLIB=
LIBTOOL_LINKERRORLIB=
LIBTOOL_LINKCOMLIB=
LIBTOOL_LINKPOSTERLIB=

LTLK_CLIENT_LIB=
