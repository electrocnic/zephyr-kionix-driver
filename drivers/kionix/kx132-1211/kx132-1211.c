/**
 * @project Kionix Sensor Drivers
 *
 * @file kx132-1211.c
 *
 * @author Ted Havelka
 *
 * @license Apache 2.0 licensed.
 */

#define DT_DRV_COMPAT kionix_kx132_1211

#include <stdio.h>
#include <math.h>

#include <zephyr/device.h>          // 2022-11-10 was <device.h>
#include <zephyr/drivers/i2c.h>     // 2022-11-10 was <drivers/i2c.h>
#include <zephyr/drivers/sensor.h>  // 2022-11-10 was <drivers/sensor.h>
#include <zephyr/sys/util_macro.h>    // <-- not sure whether this header required by driver - TMH
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>     // 2022-11-10 was <logging/log.h>

LOG_MODULE_REGISTER(KX132, CONFIG_SENSOR_LOG_LEVEL); // <-- NEED to review LOG_MODULE_DECLARE() due to this line

// NEED to review LOG_MODULE_DECLARE() stanzas in other driver sources,
//  to  assure those are not in conflict with this related Zephyr
//  logging macro - TMH

#include "kx132-1211.h"
#include "kx132-registers.h"       // to provide map of KX132 internal registers and bit-wise flags
#include "out-of-tree-drivers.h"
//#include "development-defines.h"
#include "kx132-triggers.h"        // to provide sensor interrupt port reinitialization code



//----------------------------------------------------------------------
// - SECTION - defines
//----------------------------------------------------------------------

#define DEV_TRACE_ATTR_SET_RSTATUS_VALUE



//----------------------------------------------------------------------
// - SECTION - Kionix sensor specific configuration routines
//----------------------------------------------------------------------

/*
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  @brief   Routine to configure KX132-1211 Output Data Rate (ODR).
 *  @param   Zephyr device pointer
 *  @param   Zephyr sensor value
 *
 *  @note    sensor_value.val1 data members holds a value found in
 *           KX132-1211 enumeration named 'kx132_1211_config_setting_e'.
 *           This datum is tested to select which sensor attribute
 *           calling code wants to update.
 *
 *           sensor_value.val2 data members holds a value found in
 *           KX132-1211 enumeration named 'kx132_1211_output_data_rates_e'.
 *           This datum represents the Output Data Rate setting to
 *           apply here to the KX132-1211 sensor.
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 */

#if 0
static int kx132_configure_output_data_rate(const struct device *dev, const struct sensor_value *val)
{
    struct kx132_device_data *data = dev->data;

    uint8_t reg_val_to_read = 0x00U;
    uint8_t *read_buffer = &reg_val_to_read;
#ifdef DEV_1121__KX132_I2C_BURST_WRITES_WORKING 
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
#else
    uint8_t reg_addr_and_value_to_write[] = { KX132_ODCNTL, 0x00U };
    uint8_t *write_buffer = reg_addr_and_value_to_write;
//    uint32_t len = 2;
#endif

    int rstatus = ROUTINE_OK;

    if ((val->val2 < KX132_ODR_0P781_HZ) || (val->val2 > KX132_ODR_25600_HZ))
    {
        return ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION;
    }

    {

        rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, 1);


        rstatus = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, 1);

        if ( rstatus != ROUTINE_OK )
        {
            rstatus = ROUTINE_STATUS__COMM_FAILURE_ON_BUS;
        }
        else
        {
#ifdef DEV_1121__KX132_I2C_BURST_WRITES_WORKING 
            reg_val_to_write = reg_val_to_read;  // save original value - may not be needed, review this - TMH
            reg_val_to_write &= 0xF0;            // mask to erase OSA3:OSA0
            reg_val_to_write |= val->val2;       // write bit pattern to set output data rate

            rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, 1);
#else
            reg_addr_and_value_to_write[1] = reg_val_to_read;  // save original value - may not be needed, review this - TMH
            reg_addr_and_value_to_write[1] &= 0xF0;            // mask to erase OSA3:OSA0
            reg_addr_and_value_to_write[1] |= val->val2;       // write bit pattern to set output data rate

            rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, 2);
#endif
        }
    }

    return rstatus;
}
#endif



//----------------------------------------------------------------------
// - SECTION - routines per Zephyr sensor API
//----------------------------------------------------------------------

static int kx132_1211_attr_get(const struct device *dev,
                               enum sensor_channel chan,
                               enum sensor_attribute attr,
                               struct sensor_value *val)
{
    int rstatus = ROUTINE_OK;

    switch (attr)
    {
// 2022-11-28 Unsure if we'll need it, but first case may be to
//  return a per device stored nodelabel string value.  So far
//  not finding a non-static, run time way to identify which instance
//  of a like sensor or device to which a `const struct device *dev`
//  points . . .     - TMH
//
//        case xxx:
//            break;

// Note also that static device tree macros won't allows us to
// express any C language operators, so while structures can be
// declard statically nothing can be assigned any values.  We may
// need a way, if it comes to it, to pass nodelabels which are
// string values, to this driver, whose conventional Zephyr API
// only provides for sensor_t values to be shared.  That will be
// kludgy at best to convey strings between app and driver code.

        case SENSOR_ATTR_KIONIX__OUTPUT_DATA_RATE_REG_ODCNTL:
            rstatus = kx132_get_attr__output_data_rate(dev, val);
            break;

        default:
            rstatus = ROUTINE_STATUS__UNDEFINED_SENSOR_ATTRIBUTE;
            break;
    }

    return rstatus;
}



//----------------------------------------------------------------------
// @brief  Routine to update KX132 sensor attributes, also called
//        configuration values.  The somewhat misnamed `value` parameter
//        conveys in its first of two integer members a sensor attribute,
//        configuration or state change to effect.  The second integer
//        member optionally conveys a value, when fitting, to which
//        this routine sets given sensor attribute or configuration
//        parameter.
//
//        Some KX132 config related attributes lie outside Zephyr's
//        defined sensor attributes.  These KX132 specific attributes
//        are expressed in Kionix driver enumeration named
//        'kx132_1211_config_setting_e'.
//
//        Note that not all configuration and state changes are simple,
//        numeric values applied to a KX132 register, but that some
//        attribute settings involve more complex sensor side action,
//        self tests and sensor side register updates.  In these cases
//        the second integer part of @param `value` may not convey any
//        value from app side code.  The driver itself directs the
//        sensor to perform self tests or other configuration tasks.
//
//
// @param  *dev, a pointer to a Zephyr form sensor instance, a compound data structure
//
// @param  channel, sensor API construct to describe which sensor readings (channels) to which attribute applies
//
// @param  attribute, most often a sensor setting or configuration value, which app code calls us to update
//
// @param  value, a small Zephyr structure holding two 32-bit integers.
//         The first conveys the KX132 configuration or state to change,
//         and the second integer optionally conveys a value to which
//         this routine sets given attribute, when such value applies.
//
// @return rstatus "routine status", one of:
//
//         *  unsupported sensor configuration
//         *  undefined sensor attribute
//         *  undefined sensor channel
//         *  routine ok (routine succeeded)
//
//
// Example set up and call, sets up readings with sensor actuated hardware
// interrupt, and sets an accelerometer output data rate of 3200 Hz:
//
//      requested_config.val1 = KX132_ENABLE_SYNC_READINGS_WITH_HW_INTERRUPT;
//      requested_config.val2 = KX132_ODR_3200_HZ;
//
//      sensor_api_status = sensor_attr_set(
//       dev_kx132_1,
//       SENSOR_CHAN_ALL,
//       SENSOR_ATTR_PRIV_START,
//       &requested_config
//      );
//
//
// Notes:
//
//   REF https://docs.zephyrproject.org/2.6.0/reference/peripherals/sensor.html#c.sensor_attribute
//     case SENSOR_ATTR_CONFIGURATION:  <- this enumerator available in Zephyr RTOS 2.7.99 but not 2.6.0 - TMH
//
//   +  For compatibility with Zephyr 2.6.0, Zephyr sensor channel
//      enum member `SENSOR_CHAN_ALL` is paired with the more recently
//      available and more fitting sensor attribute enum member
//      `SENSOR_ATTR_CONFIGURATION` in a nested switch construct of
//      this routine.
//----------------------------------------------------------------------

// REF https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html#c.sensor_channel.SENSOR_CHAN_ALL

static int kx132_1211_attr_set(const struct device *dev,
                               enum sensor_channel chan,
                               enum sensor_attribute attr,
                               const struct sensor_value *val)
{
    uint32_t rstatus = ROUTINE_OK;
    uint32_t sensor_config_requested = val->val1;

    switch (attr)
    {

// Note:  Zephyr standard sensor attribute enumeration values will be
//  added here, at top of SWITCH construct.  None so far implemented as
//  of 2022-11-28.  See `zephyr/include/zephyr/drivers/sensor.h` for
//  full enumeration of Zephyr defined sensor attributes.  - TMH

        case SENSOR_ATTR_PRIV_START:
        {
            switch (chan)
            {
                case SENSOR_CHAN_ALL:
                case SENSOR_ATTR_CONFIGURATION:
                {
                    switch (sensor_config_requested)
                    {

                        case KX132_PERMFORM_SOFTWARE_RESET:
// A fetch like routine, updates sensor data shadow registers 'who_am_i' and 'cotr':
                            rstatus = kx132_software_reset(dev);
                            break;

// Single register update:
                        case KX132_ENTER_STANDBY_MODE:
                            kx132_enter_standby_mode(dev);
                            break;

// Single register update:
                        case KX132_DISABLE_SAMPLE_BUFFER:
                            kx132_disable_sample_buffer(dev);
                            break;

                        case KX132_ENABLE_ASYNC_READINGS:
                            kx132_enable_asynchronous_readings(dev);
                            break;

                        case KX132_SET_OUTPUT_DATA_RATE:
                            kx132_update_output_data_rate__odcntl(dev,
                              (const enum kx132_1211_output_data_rates_e)val->val2);
                            break;

// NEED 2023-01-13 to review whether needed parameters are missing here`
                        case KX132_ENABLE_SYNC_READINGS_WITH_HW_INTERRUPT:
                            rstatus = kx132_enable_synchronous_reading_with_hw_interrupt(dev);
                            break;

#ifdef CONFIG_KX132_TRIGGER
                        case KX132_REINITIALIZE_DRDY_GPIO_PORT:
                            rstatus = kx132_reinitialize_interrupt_port(dev, DEFAULT_INTERAL_OPTION_OF_ZERO);
                            break;
#endif
                        case KX132_ENABLE_WATERMARK_INTERRUPT:
                            rstatus = kx132_enable_watermark_interrupt(dev);
                            break;


                        default: // ...action to take when requested config not supported
                            rstatus = ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION;
                            break;

                    } // close scope of switch(val)
                }

                default: // ...action to take with unrecognized sensor channel
                {
                    rstatus = ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL;
                    break;
                }

            } // close scope of switch(chan)
        }

        default: // ...default action to take with unrecognized sensor attribute
        {
            rstatus = ROUTINE_STATUS__UNDEFINED_SENSOR_ATTRIBUTE;
            break;
        }
    } // close scope of switch(attr)

#ifdef DEV_TRACE_ATTR_SET_RSTATUS_VALUE
    printk("- kx132-1211.c - kx132_1211_attr_set() rstatus holds %u, returning . . .\n\n", rstatus);
#endif
    return rstatus;

} // end routine kx132_1211_attr_set()



static int kx132_1211_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
// NEED 2023-01-19 to fully implement helpful return status value 'rstatus', this
// entails defining helpful rstatus values for each routine called
// from SWITCH construct of this routine.

    int rstatus = 0;

    switch (channel)
    {
        case SENSOR_CHAN_KIONIX_MANUFACTURER_ID:  // a four byte value
            kx132_fetch_device_id(dev);
            break;

        case SENSOR_CHAN_KIONIX_PART_ID:          // a two byte value
            kx132_fetch_part_id(dev);
            break;

        case SENSOR_CHAN_ACCEL_X:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_fetch_acceleration_x_axis(dev);
            break;

        case SENSOR_CHAN_ACCEL_Y:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_fetch_acceleration_y_axis(dev);
            break;

        case SENSOR_CHAN_ACCEL_Z:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_fetch_acceleration_z_axis(dev);
            break;

        case SENSOR_CHAN_ACCEL_XYZ:               // read of prior three pairs of registers in sequence
            kx132_fetch_acceleration_xyz_axis(dev);
            break;

        case SENSOR_CHAN_KIONIX_INTERRUPT_LATCH_RELEASE:
            kx132_fetch_interrupt_latch_release(dev);
            break;

        case SENSOR_CHAN_KIONIX_BUF_READ:
            kx132_fetch_readings_from_buf_read(dev);
            break;

        default:
            rstatus = ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL;
    }

    return rstatus;

} // end routine kx132_1211_sample_fetch()




// https://docs.zephyrproject.org/latest/hardware/peripherals/sensor.html#c.sensor_sample_fetch_chan
#if 0
static int kx132_1211_sample_fetch_chan(const struct device *dev, enum sensor_channel channel)
{
// stub function - we'll refactor code in kx132_1211_sample_fetch() to mostly
// or wholly be in this routine - TMH

    int rstatus = 0;

    return rstatus;
}
#endif




// 2021-08-31 Notes:
//  *  routine implementation underway
//  *  must review ncs v1.6.1 ncs/zephyr/include/drivers/sensor.h channel enum for highest value
//  *
// Zephyr defines a structure 'sensor_value' as two 32-bit integers,
// named .val1 and .val2.  We can use these to return smaller data,
// such as a single 32-bit or 16-value.  We need only comment here the
// way in which we are formatting our data to return to calling code.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static int kx132_1211_channel_get(const struct device *dev,
                                  enum sensor_channel chan,
                                  struct sensor_value *val)
{
// 2021-08-31 - function implementation in progress, TMH.

    int routine_status = 0;
    struct kx132_device_data *data = dev->data;

// Clear memory passed to us by calling code:
    memset(val, 0, sizeof(*val));

// Select data to 'get' based on caller's desired sensor channel:
// ( data has already been fetched, thus 'to get' data is primarily a copy operation )
    switch (chan)
    {
        case SENSOR_CHAN_KIONIX_MANUFACTURER_ID:  // a four byte value
//            val->val1 = dev->data->manufacturer_id.as_32_bit_integer;
//                             ^^^^ ...incorrect, this structure member is not named in a way we may access,
//                                     see DEVICE_DEFINE expression end of this source file - TMH
            val->val1 = data->manufacturer_id.as_32_bit_integer;
            val->val2 = 0;
            break;

        case SENSOR_CHAN_KIONIX_PART_ID:          // a two byte value
            val->val1 = data->part_id.as_16_bit_integer;
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_X:                 // a two byte value
            val->val1 = ( ( data->accel_axis_x[1] << 8 ) | ( data->accel_axis_x[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_Y:                 // a two byte value
            val->val1 = ( ( data->accel_axis_y[1] << 8 ) | ( data->accel_axis_y[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_Z:                 // a two byte value
            val->val1 = ( ( data->accel_axis_z[1] << 8 ) | ( data->accel_axis_z[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_XYZ:               // six bytes of data,
                                                  // three in lower res higher speed mode

// Here encode X, Y, Z two-byte accelerometer readings in struct sensor_value as follows:
//
//          sensor_value.val1 <-- [ Y_MSB ][ Y_LSB ][ X_MSB ][ X_LSB ]
//          sensor_value.val2 <-- [   0   ][   0   ][ Z_MSB ][ Z_LSB ]

            val->val1 = ( ( data->accel_axis_x[1] <<  8 ) | ( data->accel_axis_x[0] <<  0 )
                        | ( data->accel_axis_y[1] << 24 ) | ( data->accel_axis_y[0] << 16 ) );

            val->val2 = ( ( data->accel_axis_z[1] <<  8 ) | ( data->accel_axis_z[0] <<  0 ) );
            break;

// NOTE, KX132 sample buffer readings not to be confused with values
// read from XOUT_L, XOUT_H, YOUT_L, YOUT_H, etc:
        case SENSOR_CHAN_KIONIX_BUF_READ:
            val->val1 = ((data->shadow_reg_buf_read[0] <<  0) | (data->shadow_reg_buf_read[1] <<  8) | // x | xlsb, xmsb
                         (data->shadow_reg_buf_read[2] << 16) | (data->shadow_reg_buf_read[3] << 24)   // y | ylsb, ymsb
                        );
            val->val2 = ((data->shadow_reg_buf_read[4] <<  0) | (data->shadow_reg_buf_read[5] <<  8)   // z | zlsb, zmsb
                        );
            break;



        case SENSOR_CHAN_KIONIX_INTERRUPT_LATCH_RELEASE:
            val->val1 = data->shadow_reg_int_rel;
            val->val2 = 0;
            break;

        case SENSOR_CHAN_KIONIX_INS2:
            val->val1 = data->shadow_reg_ins2;
            val->val2 = 0;
            break;


        default:
            routine_status = ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL;
    }

    return routine_status;

} // end routine kx132_1211_channel_get()



//----------------------------------------------------------------------
// - SECTION - instantiation and initialization code
//----------------------------------------------------------------------

// Following design pattern of Zephyr 3.2.0 zephyr/drivers/sensor/iis2dh/iis2dh.c:

static int kx132_init_interface(const struct device *dev)
{
        int rstatus;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
        rstatus = kx132_spi_init(dev);
        if (rstatus) {
                return rstatus;
        }
#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
        rstatus = kx132_i2c_init(dev);
        if (rstatus) {
                return rstatus;
        }
#else
#error "BUS MACRO NOT DEFINED IN DTS"
#endif

        return 0;
}




// FEATURE - initializating function in KX132-1211 driver:
// # REF https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/sensor/iis2dh/iis2dh.c#L253

static int kx132_1211_init(const struct device *dev)
{
#ifdef CONFIG_KX132_TRIGGER
    const struct kx132_device_config *cfg = dev->config;
#endif
// - DEV 1128 - add a member to KX132 data structure to hold GPIO port initialization status, an enum - TMH
    struct kx132_device_data *data = dev->data;
// - DEV 1128 -

    uint32_t rstatus = ROUTINE_OK;


    kx132_init_interface(dev);

// Optionally check chip ID here:
    rstatus = kx132_software_reset(dev);

// Optionally set a block data update mode, if applicable, here:

// Optionally set an operating mode for power savings here:

// Optionally set a default, compile time chosen Output Data Rate (ODR) here:

// Optionally set a default range here:

// - DEV 1128 -
    data->drdy_port_status = DRDY_PORT_NOT_INITIALIZED;
// - DEV 1128 -

#ifdef CONFIG_KX132_TRIGGER
        if (cfg->int_gpio.port) {

            printk("- kx132-1211.c - kx132_1211_init() finds cfg->int_gpio.port not null,\n");

//const struct gpio_dt_spec int_gpio_for_diag = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 });
////const struct gpio_dt_spec int_gpio_for_diag = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, { 0 });
            printk("- kx132-1211.c - kx132_1211_init() interrupt GPIO port name holds '%s',\n",
              cfg->int_gpio.port->name);

            if (kx132_init_interrupt(dev) < 0) {
                LOG_ERR("kx132-1211.c - failed to initialize interrupts");
                return -EIO;
            }
        }
        else
        {
            printk("- MARK 1 b - kx132-1211.c finds cfg->int_gpio.port null!\n");
        }
#endif // CONFIG_KX132_TRIGGER

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
#warning "KX132 driver compiled with dedicated thread support"
#endif

// - DEV 1130 BEGIN -
printk("- DEV 1028 - devicetree API finds drdy-gpios compatible node with path '%s'\n", xstr(DRDY_GPIO_DEVICETREE_PATH));
// - DEV 1130 END -

    return rstatus;
}


// FEATURE - structure defined in Zephyr Project sensor.h header,
//  each member here assigned to point to functions which read and
//  write given sensor attributes, and which fetch and get (copy)
//  sensor readings most recently fetched.  
//  Zephyr's notions of 'fetch', 'get', sensor channels and related
//  terms detailed here:
// https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html

static const struct sensor_driver_api kx132_driver_api =
{
    .attr_get = &kx132_1211_attr_get,
    .attr_set = &kx132_1211_attr_set,
#if CONFIG_KX132_TRIGGER
    .trigger_set = kx132_trigger_set,
#endif
    .sample_fetch = &kx132_1211_sample_fetch,
    .channel_get = &kx132_1211_channel_get
};



// Note:  Zephyr 3.2.0 STMicro IIS2DH driver way, uses devicetree instance numbers:

#define KX132_SPI(inst)                                                                       \
        (.spi = SPI_DT_SPEC_INST_GET(                                                         \
                 0, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0),)

#define KX132_I2C(inst) (.i2c = I2C_DT_SPEC_INST_GET(inst),)



#ifdef CONFIG_KX132_TRIGGER
#warning "- DEV 1125 - assigning Zephyr DT macro value to .int_gpio of kx132_device_config structure,"
#endif


#define KX132_DEFINE(inst)                                                                    \
\
/* make sure KX132 node exists */ \
/* BUILD_ASSERT(DT_NODE_EXISTS(DT_DRV_INST(inst))); */ \
/* make sure KX132 node has drdy-gpios property */ \
/* BUILD_ASSERT(DT_NODE_HAS_PROP(DT_DRV_INST(inst), drdy_gpios)); */ \
/* check that first drdy-gpios pin is on GPIO port 1 (gpio1) */ \
/* BUILD_ASSERT(DT_SAME_NODE(DT_GPIO_CTRL_BY_IDX(DT_DRV_INST(inst), drdy_gpios, 0), DT_NODELABEL(gpio1))) */ \
\
        static struct kx132_device_data kx132_device_data_##inst = {                          \
                IF_ENABLED(CONFIG_KX132_TRIGGER,                                             \
                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),)   \
                          ) };                                                                \
                                                                                              \
        static const struct kx132_device_config kx132_device_config_##inst = {                \
                COND_CODE_1(DT_INST_ON_BUS(inst, i2c), KX132_I2C(inst), ())                   \
                COND_CODE_1(DT_INST_ON_BUS(inst, spi), KX132_SPI(inst), ())                   \
                .pm = CONFIG_KX132_POWER_MODE,                                                \
                                                                                              \
                IF_ENABLED(CONFIG_KX132_TRIGGER,                                             \
                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),)   \
                          )                                                                   \
                                                                                              \
        };                                                                                    \
                                                                                              \
        DEVICE_DT_INST_DEFINE(                                                                \
                              inst,                                                           \
                              kx132_1211_init,                                                \
                              NULL,                                                           \
                              &kx132_device_data_##inst,                                      \
                              &kx132_device_config_##inst,                                    \
                              APPLICATION,                                                    \
                              CONFIG_SENSOR_INIT_PRIORITY,                                    \
                              &kx132_driver_api                                               \
                             );


// - DEV 1125 - trying alternate interrupt bindings types, in multi-line macro about fifteen lines above here:
//                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),))
//                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, { 0 }),))

// Instance number way for compile time assingment to gpio_dt_spec sensor 'data' struct  member:
//                IF_ENABLED(CONFIG_KX132_TRIGGER_GLOBAL_THREAD,                              [backslash] 
//                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),)   [backslash]
//			   (.int_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(kionix_sensor_1), drdy_gpios, 0),)   [backslash]

//
// Instance number way for compile time assingment to gpio_dt_spec sensor 'config' struct member:
//                IF_ENABLED(CONFIG_KX132_TRIGGER_GLOBAL_THREAD,                              [backslash] 
//                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),)   [backslash]
//			   (.int_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(kionix_sensor_1), drdy_gpios, 0),)   [backslash]


/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(KX132_DEFINE)



// --- EOF ---
