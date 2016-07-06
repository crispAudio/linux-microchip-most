#!/bin/bash
set -o nounset
set -o errexit

TAG=v4.5

rm -rf .src

get_file() {
	echo "wget $1 ..."
	echo "$1" |grep -q "/" && mkdir -p ".src/$(echo "$1" |sed -r "s,/[^/]+$,,")"
	wget --quiet -O ".src/$1" "https://raw.githubusercontent.com/microchip-ais/linux/${TAG}/drivers/staging/most/$1"
}

cp_file() {
	echo "cp $1 ..."
	echo "$1" |grep -q "/" && mkdir -p ".src/$(echo "$1" |sed -r "s,/[^/]+$,,")"
	cp "../drivers/staging/most/$1" ".src/$1"
}

patch_mld() {
	cat "./patches/$1" |patch -p4 -d .src
}

get_file aim-cdev/cdev.c
get_file aim-network/networking.c
get_file aim-network/networking.h
get_file aim-sound/sound.c
get_file aim-v4l2/video.c
get_file Documentation/ABI/sysfs-class-most.txt
get_file Documentation/driver_usage.txt
get_file hdm-dim2/dim2_errors.h
get_file hdm-dim2/dim2_hal.c
get_file hdm-dim2/dim2_hal.h
get_file hdm-dim2/dim2_hdm.c
get_file hdm-dim2/dim2_hdm.h
get_file hdm-dim2/dim2_reg.h
get_file hdm-dim2/dim2_sysfs.c
get_file hdm-dim2/dim2_sysfs.h
get_file hdm-i2c/hdm_i2c.c
get_file hdm-usb/hdm_usb.c
get_file mostcore/core.c
get_file mostcore/mostcore.h
# get_file README

cp_file hdm-i2c/platform/plat_imx6q.c
cp_file hdm-i2c/platform/plat_zynq.c
cp_file hdm-dim2/platform/dim2_h2_dt.c
cp_file hdm-dim2/platform/dim2_mx6q_dt.c
cp_file hdm-dim2/platform/dim2_zynq_3p.c
cp_file hdm-dim2/platform/dim2_zynq_6p.c
cp_file hdm-dim2/platform/dim2_arwen_mlb6.c
cp_file hdm-dim2/platform/dim2_mx6q.c
cp_file hdm-dim2/platform/dim2_arwen_mlb3.c

patch_mld backport__hdm-dim2__add_module_owner.patch
patch_mld backport__hdm-i2c__add_module_owner.patch
 patch_mld backport__sound__snd_card_new.patch
 patch_mld backport__networking__alloc_netdev.patch
# patch_mld backport__networking__ether_addr_copy.patch
