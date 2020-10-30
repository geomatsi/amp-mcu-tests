# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mx6sx-cm4-ram-console)

target_sources(app PRIVATE src/main.c)

zephyr_linker_sources(SECTIONS rsc_table.ld)
