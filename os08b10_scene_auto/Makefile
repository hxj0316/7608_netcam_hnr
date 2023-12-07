
include ../Makefile.param

INC_FLAGS +=-I./include
INC_FLAGS +=-I./../scene
INC_FLAGS +=-I./tools/configaccess/include
#INC_FLAGS +=-I./tools/iniparser/include
INC_FLAGS +=-I$(OPEN_SOURCE_PATH)/iniparser/out
INC_FLAGS +=-I./../common
INC_FLAGS +=-I$(REL_DIR)/include

SMP_SRCS += $(wildcard ./tools/configaccess/src/*.c)
SMP_SRCS += $(wildcard ./src/core/*.c)
SMP_SRCS += $(wildcard ./src/sample/*.c)

INC_FLAGS +=-I$(OPEN_SOURCE_PATH)/iniparser/out
INC_FLAGS +=-I./include
INC_FLAGS +=-I$(REL_DIR)/include

TARGET := os08b10_scene_auto

TARGET_PATH := $(PWD)
INIPARSER_LIB := $(OPEN_SOURCE_PATH)/iniparser/out/libiniparser.a


SCENE_AUTO_MCF_SUPPORT ?= n
SCENE_AUTO_HNR_SUPPORT ?= n

ifeq ($(SCENE_AUTO_MCF_SUPPORT), y)
    CFLAGS += -DMCF_SCENE_AUTO_USED
    INIPARSER_LIB += $(HI_REL_LIB)/libmcf.a
    INIPARSER_LIB += $(HI_REL_LIB)/libhimcfcalibrate.a
    INIPARSER_LIB += $(HI_REL_LIB)/lib_mcf_vi.a
endif

ifeq ($(SCENE_AUTO_HNR_SUPPORT), y)
    CFLAGS += -DHNR_SCENE_AUTO_USED
    MPI_LIBS += -L$(HI_REL_LIB) -lhnr
    INIPARSER_LIB += $(HI_REL_LIB)/libpqp.a
endif


CFLAGS += $(INC_FLAGS)



# compile linux or LITEOS
include $(PWD)/../$(ARM_ARCH)_$(OSTYPE).mak
