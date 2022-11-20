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

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

static int kx132_i2c_read(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint16_t len)
{
	const struct kx132_device_config *config = dev->config;

	return i2c_burst_read_dt(&config->i2c, reg_addr | 0x80, value, len);

#warning "--- DEV 1120 --- compiling kx132_12c_read() function . . ."
   printk("***\n*** DEV 1120 --- kx132_i2c_read() got value 0x%02x, %c of length %u\n***\n", (uint32_t)value, (char)value, len);
}

static int kx132_i2c_write(const struct device *dev, uint8_t reg_addr, uint8_t *value,
			    uint16_t len)
{
	const struct kx132_device_config *config = dev->config;

	return i2c_burst_write_dt(&config->i2c, reg_addr | 0x80, value, len);
}

kionix_ctx_t kx132_i2c_ctx = {
	.read_reg = (kionix_read_ptr) kx132_i2c_read,
	.write_reg = (kionix_write_ptr) kx132_i2c_write,
};

int kx132_i2c_init(const struct device *dev)
{
	struct kx132_1211_data *data = dev->data;
	const struct kx132_device_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	data->ctx = &kx132_i2c_ctx;
	data->ctx->handle = (void *)dev;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
