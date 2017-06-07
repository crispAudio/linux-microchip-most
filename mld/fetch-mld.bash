#!/bin/bash
#
# This script fetches README, example Makefile and all relevant sources of the
# MOST Linux Driver from the github repository.
#
# Usage:
#   ./fetch-mld.bash [<TAG>]
#
# The optional <TAG> here is the git tag of the MOST Linux Driver, such as
# mchp-dev or mld-1.3.12.  The default value for <TAG> is mchp-dev, that is the
# development state of the MOST Linux Driver.
#

set -o nounset
set -o errexit

TAG=${1:-${MLD_TAG:-mchp-dev}}
PRJ=https://raw.githubusercontent.com/microchip-ais/linux
GIT_REPO=https://github.com/microchip-ais/linux.git

_err() {
	echo "  ERR: $@" >&2
	exit 1
}

_warn() {
	echo "  WARN: $@" >&2
	return 0
}

_get_file() {
	local SRC=${PRJ}/${TAG}/$1
	local DST=./$2
	echo "wget ${SRC} ..."
	mkdir -p "$(echo "${DST}" |sed -r "s,/[^/]+$,,")"
	wget --quiet -O "${DST}" "${SRC}"
}

get_if_missing() {
	[[ -e "$1" ]] || _get_file "mld/$1" "$1"
}

get_src() {
	_get_file "drivers/staging/most/$1" ".src/$1" || _err "failed"
}

get_patch() {
	_get_file "mld/patches/$1" ".patches/$1" || _warn "failed"
}

patch_mld() {
	cat "./.patches/$1" |patch --force -p4 -d .src || _err "failed"
}

local_fetch() {
	get_if_missing README
	get_if_missing Makefile

	get_src Documentation/ABI/sysfs-class-most.txt
	get_src Documentation/driver_usage.txt
	get_src aim-cdev/cdev.c
	get_src aim-network/networking.c
	get_src aim-sound/sound.c
	get_src aim-v4l2/video.c
	get_src hdm-dim2/dim2_errors.h
	get_src hdm-dim2/dim2_hal.c
	get_src hdm-dim2/dim2_hal.h
	get_src hdm-dim2/dim2_hdm.c
	get_src hdm-dim2/dim2_hdm.h
	get_src hdm-dim2/dim2_reg.h
	get_src hdm-dim2/dim2_sysfs.c
	get_src hdm-dim2/dim2_sysfs.h
	get_src hdm-dim2/platform/dim2_arwen_mlb3.c
	get_src hdm-dim2/platform/dim2_arwen_mlb6.c
	get_src hdm-dim2/platform/dim2_h2_dt.c
	get_src hdm-dim2/platform/dim2_mx6q.c
	get_src hdm-dim2/platform/dim2_mx6q_dt.c
	get_src hdm-i2c/hdm_i2c.c
	get_src hdm-i2c/platform/plat_imx6q.c
	get_src hdm-i2c/platform/plat_zynq.c
	get_src hdm-i2s/i2s_hdm.h
	get_src hdm-i2s/i2s_clkgen.h
	get_src hdm-i2s/i2s_clkgen.c
	get_src hdm-i2s/i2s_hdm.c
	get_src hdm-i2s/configure.sh
	get_src hdm-usb/hdm_usb.c
	get_src mostcore/core.c
	get_src mostcore/default_conf.c
	get_src mostcore/mostcore.h

	get_patch backport__hdm-dim2__add_module_owner.patch
	get_patch backport__hdm-i2c__add_module_owner.patch
	get_patch backport__sound__snd_card_new.patch
	get_patch backport__sound__snd_pcm_set_ops.patch
	get_patch backport__networking__alloc_netdev.patch
	get_patch backport__networking__ether_addr_copy.patch
	get_patch backport__networking__ether_addr_equal.patch
	get_patch backport__hdm-dim2__devm_ioremap_resource.patch

	# apply the universal patches only
	# the rest depends on your kernel
	patch_mld backport__hdm-dim2__add_module_owner.patch
	patch_mld backport__hdm-i2c__add_module_owner.patch
}

main() {
	which wget >/dev/null || _err "wget is not installed"

	rm -rf .src .patches

	if _get_file "mld/fetch-mld.bash" ".fetch"; then
		cat .fetch |while read x y; do
			case $x in
			(get_if_missing|get_src|get_patch|patch_mld) $x "$y";;
			esac
		done
	else
		echo "file not found, try to fetch using the local fetch function"
		local_fetch
	fi

	echo "add version info ..."

	if which git >/dev/null; then
		LABEL="$(git ls-remote $GIT_REPO |grep "/${TAG}$" |sed "s,\s.*,,")"
	else
		LABEL="$(date --rfc-3339=seconds)"
	fi
	sed -i -r -e "/__init/,/return/s,\<pr_.*init.*,pr_info(\"MOST Linux Driver $TAG ${LABEL}\\\\n\");," \
		.src/mostcore/core.c
	grep --with-filename "MOST Linux Driver " .src/mostcore/core.c ||
		_err "failed to set driver version info"
}

main
