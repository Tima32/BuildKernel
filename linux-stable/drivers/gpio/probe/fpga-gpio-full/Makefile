PWD  := $(shell pwd)

# This is default values until you set them from env.
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-
subdir-ccflags-y += -I$(srctree)/drivers/fpga/probe/

obj-$(CONFIG_GRIF_GPIO_EXPANDER) += grif_gpio.o
grif_gpio-objs := main.o

# Enable pr_debug() for all source code file
# ccflags-y += -DDEBUG
