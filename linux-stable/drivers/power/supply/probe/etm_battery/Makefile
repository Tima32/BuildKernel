ifndef KDIR
  $(error KDIR variable is not set)
endif

ifdef DESTDIR
  # The kernel reqires INSTALL_MOD_PATH to be absolute, this line allows user to
  # provide relative DESTDIR
  override DESTDIR := $(shell realpath $(DESTDIR))
endif

CROSS_COMPILE ?= arm-linux-gnueabihf-
ARCH ?= arm

export CROSS_COMPILE
export ARCH

# Modules will be installed to /lib/modules/`uname -r`/$(MOD_DIR)
MOD_DIR ?= probe

KMAKE := $(MAKE) -C $(KDIR) M=$(shell pwd)

all: build

build:
	$(KMAKE) W=1 modules

clean distclean:
	$(KMAKE) clean

install: build
	$(KMAKE)  modules_install \
		INSTALL_MOD_PATH?=$(DESTDIR) \
		INSTALL_MOD_DIR=$(MOD_DIR)

mrproper: distclean
	dh_clean
	rm -rf debian/tmpdocs


.PHONY: all build clean install distclean mrproper deb
