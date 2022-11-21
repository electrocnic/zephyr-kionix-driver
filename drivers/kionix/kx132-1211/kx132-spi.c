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

#define DT_DRV_COMPAT kionix_kx132

#include <string.h>
#include "kx132-1211.h"
#include <zephyr/logging/log.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#define KX132_SPI_READM   (3 << 6) /* 0xC0 */
#define KX132_SPI_WRITEM  (1 << 6) /* 0x40 */

LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

static int kx132_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct kx132_device_config *config = dev->config;
	uint8_t buffer_tx[2] = { reg | KX132_SPI_READM, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	if (spi_transceive_dt(&config->spi, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}

static int kx132_spi_write(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct kx132_device_config *config = dev->config;
	uint8_t buffer_tx[1] = { reg | KX132_SPI_WRITEM };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};


	if (spi_write_dt(&config->spi, &tx)) {
		return -EIO;
	}

	return 0;
}

kionix_ctx_t kx132_spi_ctx = {
	.read_reg = (kionix_read_ptr) kx132_spi_read,
	.write_reg = (kionix_write_ptr) kx132_spi_write,
};

int kx132_spi_init(const struct device *dev)
{
	struct kx132_1211_data *data = dev->data;
	const struct kx132_device_config *config = dev->config;

	if (!spi_is_ready(&config->spi)) {
		LOG_ERR("Bus device in KX132 driver, SPI part is not ready");
		return -ENODEV;
	}

	data->ctx = &kx132_spi_ctx;
	data->ctx->handle = (void *)dev;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
