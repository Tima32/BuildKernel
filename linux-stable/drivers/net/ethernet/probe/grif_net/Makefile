PWD  := $(shell pwd)

# This is default values until you set them from env.
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

subdir-ccflags-y += -I$(srctree)/drivers/fpga/probe/
subdir-ccflags-y += -I$(srctree)/drivers/net/ethernet/probe/
obj-$(CONFIG_GRIF_NET) += grif_net.o
grif_net-objs := main.o net.o ethtool.o mdio.o regs.o 

# Enable pr_debug() for all source code file
# ccflags-y += -DDEBUG
