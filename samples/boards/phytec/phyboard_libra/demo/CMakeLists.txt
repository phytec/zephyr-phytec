cmake_minimum_required(VERSION 3.28.0)
# Copyright (c) 2020 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(demo_openamp)

target_include_directories(app PRIVATE ${LIBMETAL_INCLUDE_DIR} ${OPENAMP_INCLUDE_DIR} ${PLATFORM_DIR})

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Mes fichiers UI : tout en DDR
zephyr_code_relocate(FILES src/ui.c src/show_hour.c LOCATION DDRAPP_TEXT NOCOPY)
zephyr_code_relocate(FILES src/ui.c src/show_hour.c LOCATION DDRAPP_RODATA NOCOPY)
zephyr_code_relocate(FILES src/ui.c src/show_hour.c LOCATION DDRAPP_DATA NOCOPY)
zephyr_code_relocate(FILES src/ui.c src/show_hour.c LOCATION DDRAPP_BSS)

# Lib LVGL (cœur + glue Zephyr, polices incluses)
zephyr_code_relocate(LIBRARY modules__lvgl LOCATION DDRAPP_TEXT NOCOPY)
zephyr_code_relocate(LIBRARY modules__lvgl LOCATION DDRAPP_RODATA NOCOPY)
zephyr_code_relocate(LIBRARY modules__lvgl LOCATION DDRAPP_DATA NOCOPY)
zephyr_code_relocate(LIBRARY modules__lvgl LOCATION DDRAPP_BSS)
