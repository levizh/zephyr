# Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(pyocd "--target=HC32F460" "--frequency=10000000")
board_runner_args(jlink "--device=HC32F460" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)