PWD  := $(shell pwd)

# This is default values until you set them from env.
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-
subdir-ccflags-y += -I$(srctree)/drivers/fpga/probe/

obj-$(CONFIG_ECP5_HWMON) += ecp5-hwmon.o
ecp5-hwmon-objs := main.o

# Enable pr_debug() for all source code file
# ccflags-y += -DDEBUG
