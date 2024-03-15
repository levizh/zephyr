# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(spi_slave_hc32)

target_sources_ifdef(CONFIG_SPI_SLAVE app PRIVATE src/spi_slave.c)

