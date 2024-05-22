
include ../Makefile.param

SMP_SRCS := $(wildcard *.c)
TARGET := $(SMP_SRCS:%.c=%)
CFLAGS += -L$(REL_LIB)/npu/stub -lascendcl

COMM_INC += -I$(REL_INC)/npu
COMM_INC += -I$(REL_INC)/svp_npu

TARGET_PATH := $(PWD)

# compile linux or LITEOS
include $(PWD)/../$(ARM_ARCH)_$(OSTYPE).mak
