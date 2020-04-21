# Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=atsam4lc4b")
board_runner_args(jlink "--speed=4000")
#board_runner_args(jlink "--iface=JTAG")
#board_runner_args(jlink "--tool-opt=-jtagconf -1,-1")
#board_runner_args(jlink "--tool-opt=-autoconnect 1")
board_runner_args(jlink "--reset-after-load")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
