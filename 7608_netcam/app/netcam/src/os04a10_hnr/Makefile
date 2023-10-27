include ../Makefile.param

TARGET := os04a10_hnr

TARGET_PATH := $(PWD)

SMP_SRCS := $(wildcard $(PWD)/*.c)

ifeq ($(CONFIG_OT_ISP_HNR_SUPPORT), y)
CFLAGS += -DSAMPLE_HNR_SUPPORT
MPI_LIBS += $(HI_REL_LIB)/libpqp.a
MPI_LIBS += $(HI_REL_LIB)/libhnr.a
endif

# compile linux or LITEOS
include $(PWD)/../$(ARM_ARCH)_$(OSTYPE).mak

