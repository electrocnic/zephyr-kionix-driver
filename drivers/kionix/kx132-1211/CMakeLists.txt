#
# Copyright (c) 2021 Neela Nurseries
#

zephyr_library()
#zephyr_library_sources(kx132-1211.c)
#zephyr_library_sources(kx132-i2c.c)
#zephyr_library_sources(kx132-spi.c)
#zephyr_library_sources(kx132-registers.c)
#zephyr_library_sources(kx132-triggers.c)

zephyr_library_sources(
  kx132-1211.c
  kx132-i2c.c
  kx132-spi.c
  kx132-registers.c
  kx132-triggers.c
  kx132-conversions.c
)
