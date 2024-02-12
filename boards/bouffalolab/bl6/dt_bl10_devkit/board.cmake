# Copyright (c) 2021, ATL Electronics
# SPDX-License-Identifier: Apache-2.0

include(${ZEPHYR_BASE}/boards/common/blflash.board.cmake)

board_runner_args(bflb_mcu_tool --chipname bl602)
include(${ZEPHYR_BASE}/boards/common/bflb_mcu_tool.board.cmake)

board_set_flasher(bflb_mcu_tool)
