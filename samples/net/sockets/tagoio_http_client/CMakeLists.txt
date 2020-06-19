# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tagoio_http_client)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/sockets.c)
target_sources_ifdef(CONFIG_WIFI app PRIVATE src/wifi.c)

set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)

generate_inc_file_for_target(
    app
    src/https-cert.der
    ${gen_dir}/https-cert.der.inc
    )
