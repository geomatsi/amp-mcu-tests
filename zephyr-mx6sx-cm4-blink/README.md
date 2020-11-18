# Overview
TODO

# How to build
It is assumed that Zephyr is properly installed in the workspace at the following location:

```bash
$ ls /path/to/zephyr/
  bootloader  modules  tools  zephyr
```

## Build using west

```bash
$ ZEPHYR_BASE=/path/to/zephyr/zephyr west build -p auto  -b udoo_neo_full_m4
```

## Build using cmake/ninja

```bash
$ cmake -B build -GNinja -DBOARD=udoo_neo_full_m4
$ ninja -C build                                 
ninja: Entering directory `build'
[1/122] Preparing syscall dependency handling

[117/122] Linking C executable zephyr/zephyr_prebuilt.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       12084 B        32 KB     36.88%
            SRAM:        3880 B        32 KB     11.84%
        IDT_LIST:         120 B         2 KB      5.86%
[122/122] Linking C executable zephyr/zephyr.elf
```

# How to use
TODO
