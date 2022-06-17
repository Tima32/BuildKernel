PWD  := $(shell pwd)

ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-
KDIR ?= ../imx-linux/

obj-$(CONFIG_FB_MELT_LCD) += mtlcd.o

default:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
