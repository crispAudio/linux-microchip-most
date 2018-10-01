# MOST<sup>&#174;</sup> Linux Driver

This document describes how to build **out-of-tree kernel modules** of the
MOST<sup>&#174;</sup> Linux Driver.

The latest version of this document can be found
[here](https://github.com/microchip-ais/linux/tree/mchp-dev/mld/README.md).

## Download

The steps below download the MOST<sup>&#174;</sup> Linux Driver **sources**,
**default configuration**, **Makefile** example and the kernel specific
**backport patches** into the new directory that is automatically generated or
specified by the user.

```sh
$ wget https://github.com/microchip-ais/linux/raw/mchp-dev/mld/fetch-mld
$ chmod +x fetch-mld
```

get the latest release:
```sh
$ OUT_DIR=mld ./fetch-mld mld-1.7.0 && cd mld
```

or get the latest development state:
```sh
$ OUT_DIR=mld ./fetch-mld mchp-dev && cd mld
```

## Build modules

Export environment variables:
```sh
$ export KDIR=... # the path to the kernel sources, default is /lib/modules/$(uname -r)/build
$ export CROSS_COMPILE=... # the binutils prefix
$ export ARCH=... # the target architecture (arm, arm64, etc.)
```

### Configure

Optionally, you can adjust the default configuration in the `default_conf.c`.

### Patch

This applies the kernel specific backport patches and recommended patches
that are not in the mainstream:
```sh
$ make patch
```

### Build

This builds the basic components (**core**, **default_conf**, **cdev**, **net**, **usb**):
```sh
$ make
```

Built artifacts:
```sh
$ ls *.ko

aim_cdev.ko  aim_network.ko  default_conf.ko  hdm_usb.ko  mostcore.ko
```

If you need to build other components, uncomment corresponding parts in the
`Makefile`.

## Helpful commands

command | action
-|-
`./patchwork test` | shows all necessary kernel specific patches that are not yet applied
`make patch` or `./patchwork patch` | applies all kernel specific backport patches and recommended patches
`make unpatch` or `./patchwork unpatch` | reverts all applied patches
`make` or `make modules` | makes kernel modules
`make clean` | cleans built artifacts
`cat .patched` | shows all applied patches
`ls *.ko` | shows built modules
`make help` | shows help of the Makefile
`./patchwork help` | shows help of the patchwork utility


## History of changes

### mld-1.7.0

- **i2c**: the module parameters *polling_req* and *scan_rate* are replaced by the
  *polling_rate*, where the value zero means the interrupt driven mode and
  other values are used as the polling rate in the polling mode
- **usb**: fixed memory allocation problem on the new kernels and arm64/aarch64
  architectures
- **sound**: added support for flexible PCM format *Nx{8,16,24,32}* to
  enable exotic formats like *4x16*, *5x8*, etc
- **dim2**: added optional property *set_dbr_size*

  This may be used to increase the DBR size in the case where the isoc channel
  is used with the big MOST isoc bandwidth.
- removed property *set_packets_per_xact* and *set_dbr_size* from the non-usb
  channels
- **dim2**: the clock speed module parameter (clk_speed) is moved from the HDM to
  the platform driver
- added **spi** driver
- **core**: the autoconf feature is moved from the source code to the patch
  mld/patches/feature__core_autoconf.patch; use *make patch* to apply
- made diverse code improvements


### mld-1.6.0

- dim2: enabled flow control for the isochronous channels
- net: fixed race between the create/destroy/use of the device
- removed dependency of HDM's on the networking-aim
  (networking-aim must not be loaded if not used)
- net: added carrier information based on the NI state
  ('ip link' shows different status depending on the MOST lock state,
  see also /sys/class/net/meth0/carrier)
- net: added use of the operational state 'dormant'
  (net device starts successfully even without valid MAC address,
  the property /sys/class/net/meth0/operstate varies between 'down', 'dormant'
  and 'up')
- usb: fixed calculation of the final buffer size for the streaming channels in
  the case where the buffer size is near to the 64K bytes
- usb: fixed calculation of the final buffer size for the streaming channels in
  the case where the buffer size is corrected by the driver down to the next
  appropriate for the hardware value


### mld-1.5.0

- core: added automatic configuration ability, that allows to create the
  configurations to automate and speed up the appearance of the devices
- cdev-aim: the character device is destroyed by the disconnecting the channel,
  i.e. the re-appeared channel may be immediately re-linked to the cdev with
  the same name even if the application holds the file handle of the previous
  character device


### mld-1.4.0

- usb-hdm: fixed synchronization procedure for the asynchronous endpoints
- networking-aim: to prevent the non-standard behavior, the networking startup
  call (ifup) remains blocked until the networking interface receives a valid
  MAC address from the INIC (commit 3c3f3af938cd)
- networking-aim: fixed startup of the networking interfaces based on the
  MediaLB and start before the MOST ring is locked (commit 1396c3de7053)
- dim2-hdm: added monitoring of the used Async Tx DBR memory as a workaround
  for the issue, where the Sync channel duplicates the packets while the Async
  channel has the traffic.  This change needs the configured interrupt mlb_int
  additional to ahb0_int in the device tree. (commit f40841ab035c)
- dim2-hdm: increased DBR size of the Control and Async Tx channels to double
  of configured buffer size to improve the throughput of the transmitted
  over the MediaLB messages
- core: the properties add_link and remove_link made write-only
- core: added read-only property 'links' that shows all linked to the AIM
  channels (without link parameters)
- core: the property set_datatype accepts also 'isoc' and returns 'isoc'
  instead of 'isoc_avp'
- core: the property set_direction accepts also 'tx' and 'rx' and returns 'tx'
  and 'rx' instead of correspondingly 'dir_tx' and 'dir_rx'
- core: the properties available_datatypes and available_directions return text
  instead of binary content and may be grep'd now
- cdev-aim: writing for streaming channels (sync/isoc) made possible using the
  buffer of any size.  This allows to use the redirection like
  'cat stream >/dev/mdev0-ca12' instead of 'dd'.
- i2s-hdm: added I2S HDM
- made a lot of diverse code improvements


### mld-1.3.12

- usb-hdm: added support for USB gadget OS81210
- usb-hdm: added arbitrary access to DCI registers
- usb-hdm: added possibility to internally sync the endpoint logic of the
  controller
- made diverse code improvements


### 1.3.11

- dim2-hdm: fixed random data at the beginning of the synchronous streams
- networking-aim: fixed interrupt unsafe spinlocks that may be the reason of
  the kernel hang


### 1.3.10

- usb-hdm: fixed kernel crash possible by closing the USB channel while traffic
- usb-hdm: fixed kernel crash due to the race in the handling of the endpoint
  stall condition
- dim2-hdm: fixed kernel freeze possible when reusing the dim2 channel
- v4l2-aim: fixed kernel crash due to interrupt unsafe spinlocks
- v4l2-aim: assigned different names for the different devices (video devices
  get the hardware dependent names and the V4L devices names may be set
  using the driver configuration which is similar to the cdev-aim)


### 1.3.9

- usb-hdm: fixed kernel crash appeared when closing the active channel
- dim2-hdm: added possibility to configure the parameter (fcnt) representing
  the number of frames per internal DIM2 sub-buffer for synchronous channels


### 1.3.8

- usb-hdm: fixed kernel crash caused by the wrong handling of the endpoint
  stall condition (regression in the version 1.3.7)
- dim2-hdm: fixed race by closing the DIM2 channel


### 1.3.7

- usb-hdm: added product id for OS81119
- usb-hdm: fixed handling of the endpoint stall condition
- made diverse code improvements


### 1.3.6

- core: fixed race between probe_channel and most_start_channel
- core: fixed potential minor number leak
- core: fixed issues by removing the link to channel
- aim-cdev: fixed races by closing the channels
- aim-cdev: fixed potential MBO leak
- aim-cdev: poll: fixed issue with waking the sleeping process
- aim-network: added rx_dropped statistics
- made diverse code improvements


### 1.3.5

- aim-cdev: added support of the poll syscall
- dim2-hdm: fixed reordering by IO operations
- made diverse small code improvements


### 1.3.4

- aim-sound: improved stability of ALSA 5.1
- aim-sound: more information about fails in kernel log
- core: fixed regression in most_deregister_interface appeared in 1.3.2


### 1.3.3

- aim-sound: added ALSA 5.1 support
- core: added fair distribution of buffers
- usb-hdm: fixed doing DMA on stack
- usb-hdm: fixed MAC address representation
- dim2-hdm: fixed too big buffer size for DIM2
- made diverse small code improvements


### 1.3.2

- fixed sync request (over USB interface) for OS81118 Rev.D
- fixed multi-AIM support
  (simultaneous MEP/MDP is possible)
- made diverse small code improvements

known issues

- arbitration for simultaneous MEP/MDP is not fair


### 1.3.1

changes

- fixed 1.3.0 regression: babble on in pipe of USB-HDM
  (fixes synchronous and isochronous Rx over USB)


### 1.3.0

changes

- fixed spin_lock/spin_unlock unmatching by unloading V4L-AIM
- fixed MAC address representation for PCIe-INIC
- fixed incompatibilities to current PCIe-INIC firmware
- fixed race between register_netdev() and ndo_open()
  (no crash by opening device exact after registering)
- fixed bug blocking open of character devices
- fixed endianess issues in USB-HDM if running with USB-INIC RevD
- removed noisy kernel logs
- fixed resource leak by closing character device
- made diverse small code improvements

known issues

- possible endianess issues in representation of MAC address by USB-HDM
  if running with USB-INIC RevC
- multi-AIM support doesn't work
  (simultaneous MEP/MDP is not possible)
- USB-HDM is not working properly in virtual environment
  (babble on IN pipe)
