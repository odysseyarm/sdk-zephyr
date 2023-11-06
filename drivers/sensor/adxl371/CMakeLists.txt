#
#  Copyright (c) 2018 Analog Devices Inc.
#
#  SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(adxl371.c)
zephyr_library_sources(adxl371_spi.c)
zephyr_library_sources(adxl371_i2c.c)
zephyr_library_sources_ifdef(CONFIG_ADXL371_TRIGGER adxl371_trigger.c)
