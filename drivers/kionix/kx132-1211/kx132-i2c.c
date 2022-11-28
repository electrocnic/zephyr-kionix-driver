/*
 *  Kionix KX132 Zephyr driver
 *
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Based on:
 *    ST Microelectronics IIS2DH 3-axis accelerometer driver
 *    Copyright (c) 2020 STMicroelectronics
 *    SPDX-License-Identifier: Apache-2.0
 *    Datasheet:  https://www.st.com/resource/en/datasheet/iis2dh.pdf
 */

#define DT_DRV_COMPAT kionix_kx132_1211

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "kx132-1211.h"

#include <stdio.h>                 // Only needed for development printk() calls, production code won't need this - TMH



#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

static int kx132_i2c_read(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint16_t len)
{
#if 0
    char lbuf[240];
#endif
    int rstatus = 0;
    int sensor_reg_addr = reg_addr;
    int *reg_addr_ptr = &sensor_reg_addr;
    const struct kx132_device_config *config = dev->config;

// diag 2 here

//    return i2c_burst_read_dt(&config->i2c, reg_addr | 0x80, value, len);
    rstatus = i2c_write_read_dt(&config->i2c, reg_addr_ptr, 1, value, len);

// diag 2 here

    return rstatus;
}



// REF https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/i2c.h#L77  <-- i2c_dt_spec definition here

static int kx132_i2c_write(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint16_t len)
{
    int rstatus = 0;
    const struct kx132_device_config *config = dev->config;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// NOTE 2022-11-22, trouble getting Zephyr I2C burst reads and writes
//  to work.  Alternate I2C API selection here unfortunately compells a
//  change in how calling code sets up and passes sensor internal
//  register address:
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#if 1 // - DEV 1122 --
    char lbuf[128] = {0};
    snprintf(lbuf, sizeof(lbuf), "- kx132-i2c.c - writing internal reg 0x%02X with 0x%02X . . .\n",
      value[0], value[1]);
    printk("%s", lbuf);
#endif

//    return i2c_burst_write_dt(&config->i2c, reg_addr | 0x80, value, len);
    rstatus = i2c_write_dt(&config->i2c, value, len);

    return rstatus;
}



kionix_ctx_t kx132_i2c_ctx = {
	.read_reg = (kionix_read_ptr) kx132_i2c_read,
	.write_reg = (kionix_write_ptr) kx132_i2c_write,
};



int kx132_i2c_init(const struct device *dev)
{
	struct kx132_device_data *data = dev->data;
	const struct kx132_device_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device in KX132 driver, I2C part is not ready");
		return -ENODEV;
	}
	else
	{
		LOG_ERR("I2C bus in KX132 driver tests ready!");
		printk("- kx132 i2c driver - I2C bus in KX132 driver tests ready!\n");
	}

	data->ctx = &kx132_i2c_ctx;
	data->ctx->handle = (void *)dev;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */



//----------------------------------------------------------------------
// - SECTION - notes
//----------------------------------------------------------------------

#if 0

// diag 1
.........................
snprintf(lbuf, sizeof(lbuf), "- DEV 1120 - about to read kx132 internal register 0x%02x, requesting %u bytes . . .\n",
  reg_addr, len);
printk("%s", lbuf);
.........................


// diag 2
.........................
#warning "--- DEV 1120 --- compiling kx132_12c_read() function . . ."
snprintf(lbuf, sizeof(lbuf), "- DEV 1120 - in KX132 driver, I2C part got first byte %u out of %u bytes read\n",
  value[0], len);
printk("%s", lbuf);
.........................


#endif

// --- EOF ---
