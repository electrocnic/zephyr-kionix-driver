/**
 * @project Kionix Sensor Drivers
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 *
 * NOTE this code copied and adapted from STMicro iis2dh_trigger.c, in Zephyr RTOS 3.2.0.
 */



#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h> 

LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

#include "kx132-1211.h"            // to provide structs kx132_device_config, and kx132_1211_data
#include "kx132-registers.h"       // to provide KX132_INC1 and similar
#include "out-of-tree-drivers.h"



/**
 * KX132 enable interrupt - enable sensor's INT1 pin to generate interrupt
 *
 * Note:  parameter 'enable' serves as a boolean flag, which
 *        reflects whether a interrupt handler function is
 *        found to be value, or not NULL at compile time.
 */

static int kx132_enable_drdy(const struct device *dev,
                             enum sensor_trigger_type type,
                             int enable)
{
    struct kx132_1211_data *sensor = dev->data;
//        KX132_INC1
    uint8_t register_value = 0; // iis2dh_ctrl_reg3_t reg3;
    uint8_t *read_buffer = &register_value;
    uint32_t rstatus = 0;

    rstatus = kx132_read_reg(sensor->ctx, KX132_INC1, read_buffer, 1);

    /* set interrupt for pin INT1 */
//    iis2dh_pin_int1_config_get(iis2dh->ctx, &reg3);

// conditionally set interrupt for pin INT1:

    if ( enable )
    {
        uint8_t data_to_write[] = {KX132_INC1, (register_value |= 0x30)};
        uint8_t *write_buffer = data_to_write;
        rstatus = kx132_write_reg(sensor->ctx, KX132_INC1, write_buffer, 1);
    }

// # REF https://github.com/zephyrproject-rtos/hal_st/blob/master/sensor/stmemsc/iis2dh_STdC/driver/iis2dh_reg.c#L1537
//    rstatus = iis2dh_pin_int1_config_set(iis2dh->ctx, &reg3);
//
// Above line from iis2dh_trigger.c realized by our conditional call
// to kx132_write_reg() few lines above, in `if (enable)' block.

    return rstatus;
}



/**
 * kx132_trigger_set - link external trigger to event data ready
 */

int kx132_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct kx132_1211_data *kx132 = dev->data;
	const struct kx132_device_config *cfg = dev->config;
//	int16_t raw[3];  // NEED review this array, needed in IIS2DH to clear interrupt but may not be needed in KX132.
	int state = (handler != NULL) ? PROPERTY_ENABLE : PROPERTY_DISABLE;

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

