/**
 * @project Kionix Sensor Drivers
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */



#include <zephyr/drivers/sensor.h>

#include "kx132-1211.h"



/**
 * kx132_trigger_set - link external trigger to event data ready
 *
 * NOTE this code copied and adapted from STMicro iis2dh_trigger.c, in Zephyr RTOS 3.2.0.
 */

int kx132_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct kx132_data *kx132 = dev->data;
	const struct kx132_device_config *cfg = dev->config;
//	int16_t raw[3];
//	int state = (handler != NULL) ? PROPERTY_ENABLE : PROPERTY_DISABLE;

	if (!cfg->int_gpio.port) {
		return -ENOTSUP;
	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		kx132->drdy_handler = handler;
//		if (state) {
//			/* dummy read: re-trigger interrupt */
//			kx132_acceleration_raw_get(kx132->ctx, raw);
//		}
		return kx132_enable_drdy(dev, SENSOR_TRIG_DATA_READY, state);
	default:
		LOG_ERR("KX132 driver - Unsupported sensor trigger");
		return -ENOTSUP;
	}
}

