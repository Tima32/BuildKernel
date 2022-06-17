PWD  := $(shell pwd)

# This is default values until you set them from env.
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

obj-$(CONFIG_GRIF_FPGA_MGR) += grif_fpga_mgr.o
grif_fpga_mgr-objs := main.o bench.o mem_test.o
subdir-ccflags-y += -Wall -Werror -I$(srctree)/drivers/fpga/probe/

#CFLAGS_bench.o += -mfpu=neon -mfloat-abi=softfp
#CFLAGS_bench.o += -ffreestanding

# Enable pr_debug() for all source code file
#ccflags-y += -DDEBUG
