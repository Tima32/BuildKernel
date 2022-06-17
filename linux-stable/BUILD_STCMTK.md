# How to build Linux for our devices

## Synopsis

"linux-stable" building instructions.

## Dependencies

- u-boot-tools
- gcc-arm-linux-gnueabihf (version 4 - 6)

## Building

You have to prepare build environment and configuration at first:

```sh
git clone git@repo.ddg:common/sys/kernels/linux-stable.git
cd linux-stable
git submodule init
git submodule update
```

For deprecated uImage builds we have to specify LOADADDR:
uImage for SOCFPGA-based devices(deprecated):

```sh
export ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- LOADADDR=0x8000 DEBFULLNAME="MyName" DEBEMAIL="my@mail.org" UTS_MACHINE=armhf KBUILD_IMAGE=uImage KDEB_CHANGELOG_DIST="buster"
```

uImage for GRIF-based devices(deprecated):

```sh
export ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- LOADADDR=0x82000000 DEBFULLNAME="MyName" DEBEMAIL="my@mail.org" UTS_MACHINE=armhf KBUILD_IMAGE=uImage KDEB_CHANGELOG_DIST="buster"
```

zImage for GRIF/SOC-based devices:

```sh
export ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- LOADADDR=0x0 DEBFULLNAME="MyName" DEBEMAIL="my@mail.org" UTS_MACHINE=armhf KBUILD_IMAGE=uImage KDEB_CHANGELOG_DIST="buster"
```

```sh
make probe_defconfig O=build
cd build
```

The You can build Debian package:

```sh
make DEBFULLNAME="MyName" DEBEMAIL="my@mail.org" UTS_MACHINE=armhf KBUILD_IMAGE=uImage KDEB_CHANGELOG_DIST="buster" bindeb-pkg
```

or uImage:

```sh
make uImage
```

(see "arch/arm/boot" directory for an uImage).

Run this command to rebuild linux modules:

```sh
make modules
```

You can use "-jX" option with build commands to run building using X threads.
