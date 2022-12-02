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
#include "kx132-1211.h"
#include <zephyr/logging/log.h>

// For development only:
#include <stdio.h>



#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#define KX132_SPI_READM   (3 << 6) /* 0xC0  . . . bits to read multiple registers, whose addrs auto-increment */
#define KX132_SPI_WRITEM  (1 << 6) /* 0x40 */

LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

static int kx132_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct kx132_device_config *config = dev->config;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#warning "--- DEV 1120 --- compiling kx132_spi_read() function . . ."
    char lbuf[240];
    snprintf(lbuf, sizeof(lbuf), "- DEV 1202 - SPI read called with reg %u, requesting %u bytes . . .\n",
      reg, len);
    printk("%s", lbuf);
#endif

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

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#warning "--- DEV 1120 --- compiling kx132_spi_write() function . . ."
    char lbuf[240];
    snprintf(lbuf, sizeof(lbuf), "- DEV 1202 - SPI write called with reg %u, first write data of %u . . .\n",
      reg, data[0]);
    printk("%s", lbuf);
#endif

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
	struct kx132_device_data *data = dev->data;
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

#endif // 0
