# Makefile
#
# This sample Makefile may be used to build
# MOST Linux Driver kernel objects
#

ifndef KDIR
    KDIR=/lib/modules/$(shell uname -r)/build
endif

obj-m := mostcore.o
mostcore-y := mostcore/core.o

obj-m += default_conf.o
CFLAGS_default_conf.o := -I$(src)/mostcore

obj-m += aim_cdev.o
aim_cdev-y := aim-cdev/cdev.o
CFLAGS_cdev.o := -I$(src)/mostcore

obj-m += aim_network.o
aim_network-y := aim-network/networking.o
CFLAGS_networking.o := -I$(src)/mostcore

## uncomment below to use ALSA (Linux Sound)
# obj-m += aim_sound.o
# aim_sound-y := aim-sound/sound.o
# CFLAGS_sound.o := -I$(src)/mostcore

## uncomment below to use V4L2 (Video for Linux)
# obj-m += aim_v4l2.o
# aim_v4l2-y := aim-v4l2/video.o
# CFLAGS_video.o := -Idrivers/media/video -I$(src)/mostcore

## uncomment below to use I2C interface
# obj-m += hdm_i2c.o hdm_i2c_pd.o
# hdm_i2c-y := hdm-i2c/hdm_i2c.o
## uncomment only one in the [[
# hdm_i2c_pd-y := hdm-i2c/platform/plat_imx6q.o
# hdm_i2c_pd-y := hdm-i2c/platform/plat_zynq.o
## ]]
# CFLAGS_hdm_i2c.o := -I$(src)/mostcore

## uncomment below to use DIM2 interface
# obj-m += hdm_dim2.o
# hdm_dim2-y := hdm-dim2/dim2_hdm.o hdm-dim2/dim2_hal.o hdm-dim2/dim2_sysfs.o
# CFLAGS_dim2_hdm.o := -I$(src)/mostcore

obj-m += hdm_usb.o
hdm_usb-y := hdm-usb/hdm_usb.o
CFLAGS_hdm_usb.o := -I$(src)/mostcore

## uncomment below to use SPI interface
# obj-m += hdm_spi.o
# hdm_spi-y := hdm-spi/spi-prot.o
# CFLAGS_spi-prot.o := -I$(src)/mostcore

## uncomment below to use I2S interface
# obj-m += hdm_i2s.o
# hdm_i2s-y := hdm-i2s/i2s_hdm.o + hdm-i2s/i2s_clkgen.o


PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules || $(MAKE) backport_error

patch:
	@ ./patchwork patch

unpatch:
	@ ./patchwork unpatch

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

backport_error:
	@echo ''
	@echo 'HAVE YOU APPLIED ALL THE NEEDED BACKPORT PATCHES?'
	@echo ''
	@echo 'Use "make patch" to apply all needed backport patches'
	@echo ''

help:
	@echo 'USAGE'
	@echo '====='
	@echo ''
	@echo '  make [ [<environment variable>=<value> ... ] [<target>]'
	@echo ''
	@echo '<target> may be "patch", "unpatch", "modules", "clean" or "help" (without quotes).'
	@echo 'Default <target> is "modules"'
	@echo ''
	@echo 'Environment variables'
	@echo '====================='
	@echo ''
	@echo 'ARCH defines the target architecture.'
	@echo ''
	@echo 'CROSS_COMPILE defines the binutils prefix.'
	@echo ''
	@echo 'KDIR defines the path to the kernel sources.'
	@echo 'Default: /lib/modules/$$(uname -r)/build'
	@echo ''
	@echo 'EXAMPLES'
	@echo '========'
	@echo ''
	@echo 'Native making for the local host:'
	@echo '  make KDIR=/lib/modules/$$(uname -r)/build'
	@echo 'or'
	@echo '  make'
	@echo ''
	@echo 'Cross making for the embedded platform:'
	@echo '  export KDIR=~/mchp-fsl-bsp/build/tmp/work/imx6qsabreauto-poky-linux-gnueabi/linux-imx/3.14.28-r0/build'
	@echo '  export CROSS_COMPILE=~/mchp-fsl-bsp/build/tmp/sysroots/x86_64-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-'
	@echo '  export ARCH=arm'
	@echo '  make'
	@echo ''
