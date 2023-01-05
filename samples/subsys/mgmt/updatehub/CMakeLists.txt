# Copyright (c) 2018-2023 O.S.Systems
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# This application has its own Kconfig options.
set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(updatehub)

if (CONFIG_BUILD_WITH_TFM)
    target_include_directories(app PRIVATE
    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/install/interface/include
    )
endif()

target_sources(app PRIVATE src/main.c)

if(CONFIG_UPDATEHUB_DTLS)
    set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)

    foreach(inc_file
            servercert.der
            privkey.der
            )
        generate_inc_file_for_target(
            ${ZEPHYR_CURRENT_LIBRARY}
            src/certificates/${inc_file}
            ${gen_dir}/${inc_file}.inc
            )
    endforeach()
endif()
