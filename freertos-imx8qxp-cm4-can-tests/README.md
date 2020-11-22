# Firmwares for Cortex-M4 user core on iMX8 SoC

## Prerequisites

### Install MCUXpresso SDK

Download and install original MCUXpresso SDK version:

```bash
$ mkdir /path/to/mcuxpresso/sdks/sdk-2.8.0-mek-mimx8qx
$ tar xfvj SDK_2.8.0_MEK-MIMX8QX_doc.tar.gz -C /path/to/mcuxpresso/sdks/sdk-2.8.0-mek-mimx8qx/
$ ls /path/to/mcuxpresso/sdks/sdk-2.8.0-mek-mimx8qx/
```

### Install required host packages

Install arm gcc toolchain and other build tools:

```bash
$ sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi ninja-build
```

## Build instructions

### Build using Ninja

Configure build:
```bash
$ cmake -DMCUXPRESSO_PATH=/path/to/mcuxpresso/sdks/sdk-2.8.0-mek-mimx8qx -DCMAKE_TOOLCHAIN_FILE=cmake_toolchain_files/arm-none-gcc.cmake -DCMAKE_BUILD_TYPE=release -G Ninja -B build -S .

```

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
