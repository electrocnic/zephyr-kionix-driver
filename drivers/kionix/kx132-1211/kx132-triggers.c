/**
 * @project Kionix Sensor Drivers
 *
 * @file kx132-triggers.c
 *
 * @author Ted Havelka
 *
 * @license Apache 2.0 licensed.
 *
 * NOTE this code copied and adapted from STMicro iis2dh_trigger.c, in Zephyr RTOS 3.2.0.
 */


// 2022-11-24 - stdio.h for debugging only:
#include <stdio.h>                 // to provide printk()
#include <string.h>                // to provide strlen()


#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h> 

LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

#include "kx132-1211.h"            // to provide structs kx132_device_config, and kx132_device_data
#include "kx132-registers.h"       // to provide KX132_INC1 and similar
#include "out-of-tree-drivers.h"
#include "kx132-triggers.h"



/**
 * KX132 enable interrupt - enable sensor's INT1 pin to generate interrupt
 *
 * Note:  parameter 'enable' serves as a boolean flag, which
 *        reflects whether a interrupt handler function is
 *        found to be value, or not NULL at compile time.
 */


#if CONFIG_KX132_TRIGGER_NONE
// skip compilation of trigger related routines
#else




//----------------------------------------------------------------------
// - SECTION - KX132 specific functions
//----------------------------------------------------------------------

int kx132_reinitialize_interrupt_port(const struct device *dev, uint32_t option)
{
    struct kx132_device_data *data = dev->data;
//    const struct kx132_device_config *cfg = dev->config;
    struct kx132_device_config *cfg = dev->config; // NOTE we are breaking Zephyr device rule that configuration data is read only - TMH
    uint32_t rstatus = 0;

    (void)option;

    if ( data->drdy_port_status != DRDY_PORT_INITIALIZED )
    {
        cfg->int_gpio.port = NULL;
#define KX132_1_NODE DT_NODELABEL(kionix_sensor_1)
//        cfg->int_gpio.port = GPIO_DT_SPEC_GET(DT_NODELABEL(kionix_sensor_1), gpios);
        cfg->int_gpio.port = DT_GPIO_CTLR(KX132_1_NODE, drdy_gpios);

        if ( strlen(cfg->int_gpio.port->name) > MINIMUM_EXPECTED_GPIO_PORT_NAME_LENGTH )
        {
            if (!device_is_ready(cfg->int_gpio.port))
            {
                rstatus = -ENODEV;
                printk("- KX132 triggers - after drdy port reinitialization port still tests not ready\n");
            }
            else
            {
                printk("- KX132 triggers - drdy 'data ready' port reinitialization looks good\n");
            }
        }
        else
        {
            cfg->int_gpio.port = NULL;
            data->drdy_port_status = DRDY_PORT_NOT_INITIALIZED;
            rstatus = ROUTINE_STATUS__GPIO_DRDY_INTERRUPT_REINIT_FAIL;
            printk("- KX132 triggers - per port name length, drdy 'data ready' port failed to reinitialize\n");
        }
    }
    else
    {
        printk("- KX132 triggers - drdy 'data ready' port marked initialized, --force_reinit not yet implemented\n");
    }
    return rstatus;
}



//----------------------------------------------------------------------
// - SECTION - STMicro IIS2DH adapted functions:
//----------------------------------------------------------------------

static int kx132_enable_drdy(const struct device *dev,
                             enum sensor_trigger_type type,
                             int enable)
{
    struct kx132_device_data *sensor = dev->data;
    uint8_t register_value = 0; // iis2dh_ctrl_reg3_t reg3;
    uint8_t *read_buffer = &register_value;
    uint32_t rstatus = 0;

#warning "- KX132 triggers - compiling routine kx132_enable_drdy()"
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
//#if CONFIG_KX132_TRIGGER_NONE
//    return -ENOTSUP;
//#else
	struct kx132_device_data *kx132 = dev->data;
	const struct kx132_device_config *cfg = dev->config;
//	int16_t raw[3];  // NEED review this array, needed in IIS2DH to clear interrupt but may not be needed in KX132.
	int state = (handler != NULL) ? PROPERTY_ENABLE : PROPERTY_DISABLE;

#warning "- KX132 triggers - compiling routine kx132_trigger_set()"
        printk("- kx132 driver trigger - assigned to state value of %u, non-zero means handler routine not null\n", state);

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
//#endif
}



static int kx132_handle_drdy_int(const struct device *dev)
{
//#if CONFIG_KX132_TRIGGER_NONE
//    return -ENOTSUP;
//#else
        struct kx132_device_data *data = dev->data;

#warning "- KX132 triggers - compiling routine kx132_handle_drdy_int()"
        struct sensor_trigger drdy_trig = { 
                .type = SENSOR_TRIG_DATA_READY,
                .chan = SENSOR_CHAN_ALL,
        };

        if (data->drdy_handler) {
                data->drdy_handler(dev, &drdy_trig);
        }

        return 0;
//#endif
}



/**
 * Following code taken from iis2dh_trigger.c:
 *
 * iis2dh_handle_interrupt - handle the drdy event
 * read data and call handler if registered any
 */
static void kx132_handle_interrupt(const struct device *dev)
{
//#if CONFIG_KX132_TRIGGER_NONE
//    return;
//#else
#warning "- KX132 triggers - compiling routine kx132_handle_interrupt()"
        const struct kx132_device_config *cfg = dev->config;

        kx132_handle_drdy_int(dev);

        gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
//#endif
}



static void kx132_gpio_callback(const struct device *dev,
                                 struct gpio_callback *cb, uint32_t pins)
{
#warning "- KX132 triggers - compiling routine kx132_gpio_callback()"
        struct kx132_device_data *kx132 =
                CONTAINER_OF(cb, struct kx132_device_data, gpio_cb);
        const struct kx132_device_config *cfg = kx132->dev->config;

        if ((pins & BIT(cfg->int_gpio.pin)) == 0U) {
                return;
        }

        gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);

#if defined(CONFIG_KX132_TRIGGER_OWN_THREAD)
        k_sem_give(&kx132->gpio_sem);
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
        k_work_submit(&kx132->work);
#endif /* CONFIG_KX132_TRIGGER_OWN_THREAD */
}

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
static void kx132_thread(struct kx132_device_data *kx132)
{
#warning "- KX132 triggers - compiling routine kx132_thread()"
        while (1) {
                k_sem_take(&kx132->gpio_sem, K_FOREVER);
                kx132_handle_interrupt(kx132->dev);
        }
}
#endif /* CONFIG_KX132_TRIGGER_OWN_THREAD */



#ifdef CONFIG_KX132_TRIGGER_GLOBAL_THREAD
static void kx132_work_cb(struct k_work *work)
{
#warning "- KX132 triggers - compiling routine kx132_work_cb()"
        struct kx132_device_data *kx132 =
                CONTAINER_OF(work, struct kx132_device_data, work);

        kx132_handle_interrupt(kx132->dev);
}
#endif /* CONFIG_KX132_TRIGGER_GLOBAL_THREAD */



int kx132_init_interrupt(const struct device *dev)
{
    struct kx132_device_data *kx132 = dev->data;
    const struct kx132_device_config *cfg = dev->config;
    uint32_t rstatus;

#warning "- KX132 triggers - compiling routine kx132_init_interrupt()"

// DEBUG 1124 BEGIN
    printk("- MARK 2 - kx132 triggers driver sub-part\n");
    printk("- sensor device name is '%s'\n", dev->name);
    if ( cfg == NULL )
        { printk("- WARNING - sensor config data structure found null!\n"); }
    else
        { printk("- INFO - sensor config data structure not null,\n"); }

//    if ( cfg.int_gpio == NULL )
//        { printk("- WARNING - sensor config.int_gpio data structure found null!\n"); }
//    else
//        { printk("- INFO - sensor config.int_gpio data structure not null,\n"); }

    if ( cfg->int_gpio.port == NULL )
        { printk("- WARNING - sensor config->int_gpio.port data structure found null!\n"); }
    else
        { printk("- INFO - sensor config->int_gpio.port data structure not null,\n"); }

    printk("- INFO - cfg->int_gpio.port->name holds '%s'\n", cfg->int_gpio.port->name);

// Are we able to successfully call `device_is_ready()` from here?
    if ( device_is_ready(dev) )
        { printk("- INFO - KX132-1211 device ready,\n"); }
    else
        { printk("- INFO - KX132-1211 device not ready!\n"); }

// 'port' is of type Zephyr device, whose struct entails? . . . :
//

//        { printk("\n"); }

    printk("- MARK 3 -\n");
    if ( cfg->int_gpio.port->state != NULL )
        { printk("- INFO - cfg->int_gpio.port->state not null,\n"); }
    printk("- MARK 4 -\n");

    if ( strlen(cfg->int_gpio.port->name) < 3 )
    {
        printk("- kx132-triggers.c - drdy port looks mal-assigned, deferring further port checks for now . . .\n");
        kx132->drdy_port_status = DRDY_PORT_MAL_INITIALIZED;
        return ROUTINE_STATUS__GPIO_DRDY_INTERRUPT_LOOKS_MAL_ASSIGNED;
    }
// DEV NOTE 2022-11-28:  mark 4 is the last mark we see, let's see if a test of port.name length
//  allow this driver to introspect and possibly retry gpio_dt_spec member type
//  assignment later in or after boot times . . .

    printk("- INFO - cfg->int_gpio.port->state->initialized holds %u,\n", (uint8_t)cfg->int_gpio.port->state->initialized);
    printk("- INFO - cfg->int_gpio.port->state->init_res holds %u,\n", cfg->int_gpio.port->state->init_res);
    printk("- MARK 5 -\n");


// DEBUG 1124 END


// # REF https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/gpio.h#L271
    if (!device_is_ready(cfg->int_gpio.port))
    {
        printk("- MARK 3 - kx132 triggers, GPIO interrupt port not ready!\n");
        LOG_ERR("%s: device %s is not ready", dev->name, cfg->int_gpio.port->name);
        return -ENODEV;
    }

// QUESTION:  is following a cyclic assignment, where above *kx132 points to dev->data,
//  and now here kx132->dev points to dev?  Is this saying:
//      +--------------+
//      V              |
//     dev -> data -> dev

    printk("- MARK 4 - kx132 triggers, pointing sensor->data->dev back to 'sensor',\n");
    kx132->dev = dev;

#if defined(CONFIG_KX132_TRIGGER_OWN_THREAD)
    k_sem_init(&kx132->gpio_sem, 0, K_SEM_MAX_LIMIT);

    k_thread_create(&kx132->thread, kx132->thread_stack,
                    CONFIG_KX132_THREAD_STACK_SIZE,
                    (k_thread_entry_t)kx132_thread, kx132,
                    0, NULL, K_PRIO_COOP(CONFIG_KX132_THREAD_PRIORITY),
                    0, K_NO_WAIT);
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
    printk("- MARK 4 - kx132 triggers, assigning sensor->data->work.handler 'kx132_work_cb',\n");
    kx132->work.handler = kx132_work_cb;
#endif /* CONFIG_KX132_TRIGGER_OWN_THREAD */

    rstatus = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
    if ( rstatus < 0 )
    {
        LOG_DBG("KX132:  Could not configure gpio");
        return rstatus;
    }

    gpio_init_callback(&kx132->gpio_cb,
                       kx132_gpio_callback,
                       BIT(cfg->int_gpio.pin));

    if ( gpio_add_callback(cfg->int_gpio.port, &kx132->gpio_cb) < 0 )
    {
        LOG_DBG("Could not set gpio callback");
        return -EIO;
    }

#if 0 // not applicable to KX132:
	/* enable drdy on int1 in pulse mode */
	if (iis2dh_int1_pin_notification_mode_set(iis2dh->ctx, IIS2DH_INT1_PULSED)) {
		return -EIO;
	}
#endif
	return gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

#endif // CONFIG_KX132_TRIGGER_NONE



// --- EOF ---
