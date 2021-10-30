# Firmwares for Cortex-M4 user core on iMX8 SoC

## Prerequisites

### Accompanying kernel

Kernel repository: https://github.com/geomatsi/linux

Branch: imx8qxp-cm4-can-tests

Driver: CAN_RPMSG_IMX in drivers/net/can

Boards:
 - ccimx8x-sbc-pro-can.dts: Digi8x SBC Pro with 2 [WaveShare 2-CH CAN](https://www.waveshare.com/wiki/2-CH_CAN_HAT) hats
 - imx8qxp-icore-minimal.dts: Engicam iCore8x SoM

### Install MCUXpresso SDK

Select and install original MCUXpresso SDK version. For convenience the required SDK tarball is included into this repository.

```bash
$ mkdir /path/to/mcuxpresso/sdks/
$ tar xfvj sdk/sdk-2.8.0-mek-mimx8qx.tar.bz2 -C /path/to/mcuxpresso/sdks/
$ ls /path/to/mcuxpresso/sdks/sdk-2.8.0-mek-mimx8qx/
```

### Install required host packages

Install arm gcc toolchain and other build tools:

```bash
$ sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi ninja-build
```

## Build instructions

Select appropriate firmware:

```bash
$ cd freertos-imx8qxp-cm4-can-tests
```

### Build using Ninja

Configure build:
```bash
$ cmake -DMCUXPRESSO_PATH=/path/to/mcuxpresso/sdks/sdk-2.8.0-mek-mimx8qx -DCMAKE_TOOLCHAIN_FILE=cmake_toolchain_files/arm-none-gcc.cmake -DCMAKE_BUILD_TYPE=release -DBOARD=digi8x -G Ninja -B build -S .
```
Note that currently the following two boards are supported: digi8x, icore8x.

Build firmware:
```bash
$ ninja -C build
```

### Build using Make

Configure build:
```bash
$ cmake -DMCUXPRESSO_PATH=/path/to/mcuxpresso/sdks/sdk-2.8.0-mek-mimx8qx -DCMAKE_TOOLCHAIN_FILE=cmake_toolchain_files/arm-none-gcc.cmake -DCMAKE_BUILD_TYPE=release -G "Unix Makefiles" -B build -S .

```

Build firmware:
```bash
$ make -C build
```

## Testing firmware on Digi8x SBC Pro board

For pingpong example start cortex-m4 firmware and load kernel module:

```bash
$ cp freertos_rpmsg_can.elf /lib/firmware/rproc-imx-rproc-fw
$ echo start > /sys/class/remoteproc/remoteproc0/state
$ modprobe imx_rpmsg_pingpong
```

For CAN offload example start cortex-m4 firmware. If udev works properly,
then can-rpmsg-imx module will be loaded automatically:

```bash
$ cp freertos_rpmsg_can.elf /lib/firmware/rproc-imx-rproc-fw
$ echo start > /sys/class/remoteproc/remoteproc0/state
$ ip link set dev can1 txqueuelen 1000 up
$ ip link set dev can2 txqueuelen 1000 up
$ ip link set dev can3 txqueuelen 1000 up
$ ip link set dev can4 txqueuelen 1000 up
$ ip link set dev can5 txqueuelen 1000 up
$ ip link set dev can6 txqueuelen 1000 up
```
