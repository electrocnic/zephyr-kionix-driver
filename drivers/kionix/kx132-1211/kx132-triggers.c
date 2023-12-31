/**
 * @project Kionix Sensor Drivers
 *
 * @file kx132-triggers.c
 *
 * @author Ted Havelka
 *
 * @license Apache 2.0 licensed.
 *
 * NOTE this code adapted from STMicro iis2dh_trigger.c, in Zephyr RTOS 3.2.0.
 */



//----------------------------------------------------------------------
// - SECTION - includes
//----------------------------------------------------------------------

// 2022-11-24 - stdio.h for debugging only:
#include <stdio.h>                 // to provide printk()
#include <string.h>                // to provide strlen()

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h> 
LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

#include "kx132-1211.h"            // to provide structs kx132_device_config, and kx132_device_data
#include "kx132-registers.h"       // to provide KX132_INC1 and similar
#include "out-of-tree-drivers.h"   // to provide sensor driver routine return values enum
#include "kx132-triggers.h"        // to provide prototyps to public API functions



//----------------------------------------------------------------------
// - SECTION - defines
//----------------------------------------------------------------------

// Original active config for GPIO interrupt, taken from IIS2DH driver:
#define GPIO_INT__KX132_SETTING   GPIO_INT_EDGE_TO_ACTIVE
//#define GPIO_INT__KX132_SETTING   GPIO_INT_EDGE_RISING

#define DEV_1201_INTERRUPT_STRING "A5A5A5A5\n"

#define DEV__KX132_TRIGGERS__MESSAGES_MARK_ENABLED
#define DEV__KX132_TRIGGERS__MESSAGES_INFO_ENABLED
#define DEV__KX132_TRIGGERS__GPIO_PORT_NULL_CHECK
#define DEV__KX132_TRIGGERS__DEVICE_READY_CHECK
#define DEV__KX132_TRIGGERS__SHOW_PORT_INIT_STATAE

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

    uint32_t rstatus = 0;

#if 0 // - DEV 1209 disabling this routine 

    struct kx132_device_data *data = dev->data;
//    const struct kx132_device_config *cfg = dev->config;
//    struct kx132_device_config *cfg = dev->config; // NOTE we are breaking Zephyr device rule that configuration data is read only - TMH
    uint32_t rstatus = 0;

    (void)option;

    if ( data->drdy_port_status != DRDY_PORT_INITIALIZED )
    {

//#define DT_DRV_COMPAT kionix_kx132_1211
#define KX132_1_NODE DT_NODELABEL(kionix_sensor_1)

//        data->int_gpio = (struct gpio_dt_spec)NULL;

//        cfg->int_gpio.port = GPIO_DT_SPEC_GET(DT_NODELABEL(kionix_sensor_1), drdy_gpios);  // zephyr/include/zephyr/drivers/gpio.h:337:2: error: expected expression before '{' token
//        cfg->int_gpio.port = DT_GPIO_CTLR(KX132_1_NODE, drdy_gpios);               // error: 'DT_N_S_soc_S_peripheral_50000000_S_gpio_1' undeclared
//        cfg->int_gpio.port = GPIO_DT_SPEC_GET_BY_IDX(KX132_1_NODE, drdy_gpios, 0); // zephyr/include/zephyr/drivers/gpio.h:337:2: error: expected expression before '{' token
//        cfg->int_gpio.port = DT_GPIO_CTLR_BY_IDX(KX132_1_NODE, drdy_gpios, 0);     // error: 'DT_N_S_soc_S_peripheral_50000000_S_gpio_1' undeclared

// ./include/zephyr/devicetree/gpio.h:235: * consider using @c DEVICE_DT_GET(DT_INST_GPIO_CTLR_BY_IDX(node, gpio_pha, idx)).

//        cfg->int_gpio = DEVICE_DT_GET(DT_INST_GPIO_CTLR_BY_IDX(KX132_1_NODE, drdy_gpios, 0));  // error: pasting ")" and "_ORD" does not give a valid preprocessing token


// From spi_bitbang sample app:
// 131    .gpio = GPIO_DT_SPEC_GET(SPIBB_NODE, cs_gpios),

//        data->int_gpio = GPIO_DT_SPEC_GET(KX132_1_NODE, drdy_gpios);

/*
 336 #define GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx)                            \
 337         {                                                                      \
 338                 .port = DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(node_id, prop, idx)),\
 339                 .pin = DT_GPIO_PIN_BY_IDX(node_id, prop, idx),                 \
 340                 .dt_flags = DT_GPIO_FLAGS_BY_IDX(node_id, prop, idx),          \
 341         }
*/


// - DEV 1129 - New tact, attempt to use device_get_binding():
//
// Note:  if the following device_get_binding() call works as needed,
//  we may be looking at finding an amendment to this driver so that
//  app code may pass string data to it.  Zephyr API device_get_binding()
//  takes a device::name value as its sole parameter, and this is a
//  string value.

//        data->int_gpio.port = DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(KX132_1_NODE, drdy_gpios, 0);
        data->int_gpio.port = device_get_binding("arduino_gpio");   // this is a nodelabel in file lpcxpresso55s69.dtsi
//        data->int_gpio.pin = DT_GPIO_PIN_BY_IDX(KX132_1_NODE, drdy_gpios, 0);
//        data->int_gpio.dt_flags = DT_GPIO_FLAGS_BY_IDX(KX132_1_NODE, drdy_gpios, 0);

// - DEV 1129 QUESTION:  can we call device_get_binding() above in this way?  Does
//  this driver code see the device tree overlays that our application sees?
//  A quick test:

// DT_REG_ADDR(DT_NODELABEL(arduino_gpio))

    printk("- DEV 1129 - device node for gpio controller has register block start address 0x%08X\n", DT_REG_ADDR(DT_NODELABEL(arduino_gpio)));

// Per NXP UM11126.pdf, LPC55S69 user manual page 17 of 1190, above
// trace statement gives '0x5008000', the secure version of the
// GPIO port's base address.  This implies our driver can see device
// tree dtsi and overlay file information, just as our app "is able".

// DT_PROP(DT_NODELABEL(kionix_sensor_1), drdy_gpios)

//    printk("- DEV 1129 - Kionix sensor node has drdy_gpios property value of '%s'\n", DT_PROP(DT_NODELABEL(kionix_sensor_1), drdy_gpios));
// build time error, devicetree_generated.h holds symbol that's not defined.

#if DT_NODE_HAS_PROP(DT_NODELABEL(kionix_sensor_1), drdy_gpios)
#warning "- DEV 1129 - Kionix sensor node has 'drdy_gpios' property"
#endif


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Note:  we would check cfg->int_gpio.port, but Zephyr's convention
//  with driver code is to place sensor data which does not change at 
//  run time in the sensor driver's config structure.  The sensor's data
//  structure is meant for data which we expect to change during
//  firmware run times.  This said, we're tracing out a GPIO port
//  initialization error, and are attempting some run time tests to
//  reinitialize the port after boot time.  These tests are to 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        if ( strlen(data->int_gpio.port->name) > MINIMUM_EXPECTED_GPIO_PORT_NAME_LENGTH )
        {
            if (!device_is_ready(data->int_gpio.port))
            {
                rstatus = -ENODEV;
                printk("- KX132 triggers - after reinitialization device_is_ready() still fails port for drdy interrupt\n");
                printk("- KX132 triggers - port->name holds '%s'\n", data->int_gpio.port->name);
            }
            else
            {
                printk("- KX132 triggers - drdy 'data ready' port reinitialization looks good\n");
                printk("- KX132 triggers - port->name holds '%s'\n", data->int_gpio.port->name);
            }
        }
        else
        {
//            data->int_gpio.port = NULL;
            data->drdy_port_status = DRDY_PORT_NOT_INITIALIZED;
            rstatus = ROUTINE_STATUS__GPIO_DRDY_INTERRUPT_REINIT_FAIL;
            printk("- KX132 triggers - per port name length, drdy 'data ready' port failed to reinitialize\n");
        }
    }
    else
    {
        printk("- KX132 triggers - drdy 'data ready' port marked initialized, --force_reinit not yet implemented\n");
    }

#endif // 0 - per DEV 1209 disabling this routine - TMH

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

#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- KX132 triggers - compiling routine kx132_enable_drdy()"
#endif
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

#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- KX132 triggers - compiling routine kx132_trigger_set()"
#endif
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

#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- KX132 triggers - compiling routine kx132_handle_drdy_int()"
#endif
        struct sensor_trigger drdy_trig = { 
                .type = SENSOR_TRIG_DATA_READY,
                .chan = SENSOR_CHAN_ALL,
        };

    printk(DEV_1201_INTERRUPT_STRING);

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
#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- KX132 triggers - compiling routine kx132_handle_interrupt()"
#endif
    const struct kx132_device_config *cfg = dev->config;

    printk(DEV_1201_INTERRUPT_STRING);
    printk("- DIAG 1130 - kx132_handle_interrupt() in driver called\n");

    kx132_handle_drdy_int(dev);

    gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT__KX132_SETTING);
//#endif
}



static void kx132_gpio_callback(const struct device *dev,
                                 struct gpio_callback *cb, uint32_t pins)
{
#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- KX132 triggers - compiling routine kx132_gpio_callback()"
#endif
    struct kx132_device_data *kx132 =
                CONTAINER_OF(cb, struct kx132_device_data, gpio_cb);
//    const struct kx132_device_config *cfg = kx132->dev->config;
    const struct kx132_device_config *cfg = dev->config;
//    const struct kx132_device_data *data = dev->data;

// zephyr/include/zephyr/sys/util_macro.h:38:#define BIT(n)  (1 << (n))
// zephyr/include/zephyr/sys/util_macro.h:44:#define BIT(n)  (1UL << (n))

//    printk(DEV_1201_INTERRUPT_STRING);
//    printk("kx132_gpio_callback - cfg pins, BIT(cfg->int_gpio.pin) hold %u, %lu\n", pins, BIT(cfg->int_gpio.pin));
//    printk("kx132_gpio_callback - data pins, BIT(data->int_gpio.pin) hold %u, %lu\n", pins, BIT(data->int_gpio.pin));

    if ((pins & BIT(cfg->int_gpio.pin)) == 0U) {
        return;
    }

    gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
    printk("zzz\n");
    printk(DEV_1201_INTERRUPT_STRING);
    printk("zzz\n");

#if defined(CONFIG_KX132_TRIGGER_OWN_THREAD)
    k_sem_give(&kx132->gpio_sem);
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&kx132->work);
#endif /* CONFIG_KX132_TRIGGER_OWN_THREAD */
}

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
static void kx132_thread(struct kx132_device_data *kx132)
{
#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- KX132 triggers - compiling routine kx132_thread()"
#endif
    while (1) {
        k_sem_take(&kx132->gpio_sem, K_FOREVER);
        kx132_handle_interrupt(kx132->dev);
    }
}
#endif /* CONFIG_KX132_TRIGGER_OWN_THREAD */



#ifdef CONFIG_KX132_TRIGGER_GLOBAL_THREAD
static void kx132_work_cb(struct k_work *work)
{
#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- KX132 triggers - compiling routine kx132_work_cb()"
#endif
    struct kx132_device_data *kx132 = CONTAINER_OF(work, struct kx132_device_data, work);

    kx132_handle_interrupt(kx132->dev);

    printk(DEV_1201_INTERRUPT_STRING);
}
#endif /* CONFIG_KX132_TRIGGER_GLOBAL_THREAD */



int kx132_init_interrupt(const struct device *dev)
{
    struct kx132_device_data *kx132_data = dev->data;
    const struct kx132_device_config *cfg = dev->config;
    uint32_t rstatus;

#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "- kx132-triggers.c - compiling routine kx132_init_interrupt()"
#endif

// DEBUG 1124 BEGIN
    printk("- kx132-triggers.c - DMARK 2 - kx132 triggers driver,\n");
    printk("- kx132-triggers.c - sensor device name is '%s'\n", dev->name);



//MESSAGES_INFO_ENABLED

#ifdef DEV__KX132_TRIGGERS__GPIO_PORT_NULL_CHECK
    if ( kx132_data->int_gpio.port == NULL )
    {
        printk("- kx132-triggers.c - WARNING - sensor data->int_gpio.port data structure found null!\n");
    }
    else
    {
        printk("- kx132-triggers.c - DINFO - sensor data->int_gpio.port data structure not null,\n");
    }
#endif

    printk("- kx132-triggers.c - DINFO - data->int_gpio.port->name holds '%s' and int_gpio pin set to pin no %u\n",
      kx132_data->int_gpio.port->name, kx132_data->int_gpio.pin);
    printk("- kx132-triggers.c - DINFO - two to the power of 'pin' holds %lu\n",
      BIT(kx132_data->int_gpio.pin));


// QUESTION:  Are we able to successfully call `device_is_ready()` from here?  ANSWER:  yes
#ifdef DEV__KX132_TRIGGERS__DEVICE_READY_CHECK
    if ( device_is_ready(dev) )
    {
        printk("- kx132-triggers.c - DINFO - Zephyr says KX132-1211 sensor is ready,\n");
    }
    else
    { 
        printk("- kx132-triggers.c - DINFO - Zephyr says KX132-1211 sensor not ready!\n");
    }
#endif


    if ( kx132_data->int_gpio.port->state != NULL )
    {
        printk("- kx132-triggers.c - DINFO - data->int_gpio.port->state not null,\n");
    }

    if ( strlen(kx132_data->int_gpio.port->name) < 3 )
    {
        printk("- kx132-triggers.c - drdy port looks mal-assigned, deferring further port checks for now . . .\n");
        kx132_data->drdy_port_status = DRDY_PORT_MAL_INITIALIZED;
        return ROUTINE_STATUS__GPIO_DRDY_INTERRUPT_LOOKS_MAL_ASSIGNED;
    }


#ifdef DEV__KX132_TRIGGERS__SHOW_PORT_INIT_STATAE
    printk("- kx132-triggers.c - DINFO - data->int_gpio.port->state->initialized holds %u,\n",
      (uint8_t)kx132_data->int_gpio.port->state->initialized);
    printk("- kx132-triggers.c - DINFO - data->int_gpio.port->state->init_res holds %u,\n",
      kx132_data->int_gpio.port->state->init_res);
    printk("- kx132-triggers.c - DMARK 5 -\n");
#endif


// DEBUG 1124 END


// # REF https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/gpio.h#L271
    if (!device_is_ready(kx132_data->int_gpio.port))
    {
        printk("- kx132-triggers.c - DMARK 6 - kx132 triggers, GPIO interrupt port not ready!\n");
        LOG_ERR("%s: device %s is not ready", dev->name, cfg->int_gpio.port->name);
        return -ENODEV;
    }
    else
    {
        kx132_data->drdy_port_status = DRDY_PORT_INITIALIZED;
    }

// QUESTION:  is following a cyclic assignment, where above *kx132 points to dev->data,
//  and now here kx132->dev points to dev?  Is this saying:
//      +--------------+
//      V              |
//     dev -> data -> dev

    printk("- kx132-triggers.c - DMARK 7 - kx132 triggers, pointing sensor->data->dev back to 'sensor',\n");
    kx132_data->dev = dev;

#if defined(CONFIG_KX132_TRIGGER_OWN_THREAD)
    printk("- kx132-triggers.c - DMARK 8 - kx132 setting up dedicated thread which involves semaphore . . .\n");
    k_sem_init(&kx132_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

    k_thread_create(&kx132_data->thread, kx132_data->thread_stack,
                    CONFIG_KX132_THREAD_STACK_SIZE,
                    (k_thread_entry_t)kx132_thread, kx132_data,
                    0, NULL, K_PRIO_COOP(CONFIG_KX132_THREAD_PRIORITY),
                    0, K_NO_WAIT);
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
    printk("- DMARK 8 - kx132 setting up global thread, assigning sensor->data->work.handler 'kx132_work_cb',\n");
    kx132_data->work.handler = kx132_work_cb;
#endif /* CONFIG_KX132_TRIGGER_OWN_THREAD */

    rstatus = gpio_pin_configure_dt(&kx132_data->int_gpio, GPIO_INPUT);
    if ( rstatus < 0 )
    {
        LOG_DBG("KX132:  Could not configure gpio");
        return rstatus;
    }

    gpio_init_callback(&kx132_data->gpio_cb,
                       kx132_gpio_callback,
                       BIT(kx132_data->int_gpio.pin));

    if ( gpio_add_callback(kx132_data->int_gpio.port, &kx132_data->gpio_cb) < 0 )
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

// # REF https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html#c.GPIO_INT_EDGE_TO_ACTIVE
//    printk("- kx132 triggers - configuring GPIO pin with flag(s) GPIO_INT_EDGE_TO_ACTIVE . . .\n");
    printk("- kx132 triggers - configuring GPIO pin with flag(s) GPIO_INT__KX132_SETTING . . .\n");
    rstatus = gpio_pin_interrupt_configure_dt(&kx132_data->int_gpio, GPIO_INT__KX132_SETTING);
    return rstatus;
}

#endif // CONFIG_KX132_TRIGGER_NONE



// --- EOF ---
