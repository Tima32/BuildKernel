PWD  := $(shell pwd)

# This is default values until you set them from env.
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

MODULE_NAME = ecp5_fpga_mgr

ccflags-y += -Wno-strict-prototypes

obj-$(CONFIG_ECP5_FPGA_MGR) := $(MODULE_NAME).o
$(MODULE_NAME)-objs := main.o
$(MODULE_NAME)-objs += lattice/SSPIEm.o
$(MODULE_NAME)-objs += lattice/intrface.o
$(MODULE_NAME)-objs += lattice/core.o
$(MODULE_NAME)-objs += lattice/util.o
$(MODULE_NAME)-objs += lattice/hardware.o 


# Enable pr_debug() for all source code file
#ccflags-y += -DDEBUG
	
MODULE_KO := $(MODULE_NAME).ko
