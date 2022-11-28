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

#include <math.h>

#include <zephyr/device.h>          // 2022-11-10 was <device.h>
#include <zephyr/drivers/i2c.h>     // 2022-11-10 was <drivers/i2c.h>
#include <zephyr/drivers/sensor.h>  // 2022-11-10 was <drivers/sensor.h>
#include <zephyr/sys/util_macro.h>

#include <zephyr/logging/log.h>     // 2022-11-10 was <logging/log.h>

LOG_MODULE_REGISTER(KX132, CONFIG_SENSOR_LOG_LEVEL); // <-- NEED to review LOG_MODULE_DECLARE() due to this line

// NEED to review LOG_MODULE_DECLARE() stanzas in other driver sources,
//  to  assure those are not in conflict with this related Zephyr
//  logging macro - TMH

#include "kx132-1211.h"
#include "kx132-registers.h"       // to provide map of KX132 internal registers and bit-wise flags
#include "out-of-tree-drivers.h"
//#include "development-defines.h"
#define "kx132-triggers.h"         // to provide sensor interrupt port reinitialization code


#include <stdio.h>



//----------------------------------------------------------------------
// Note:  struct sensor_value is defined in Zephyr's sensor.h header
// file.  In Nordic ncs v1.6.1 SDK this file found in
// nsc/v1.6.1/zephyr/include/drivers
//
// Kionix KX132-1211 register definitions found in KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf.
//----------------------------------------------------------------------



//----------------------------------------------------------------------
// - SECTION - Kionix sensor specific configuration routines
//----------------------------------------------------------------------

#if 0
#endif



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
    uint32_t len = 2;
#endif

    int rstatus = ROUTINE_OK;


    if ((val->val2 < KX132_ODR__0P781_HZ) || (val->val2 > KX132_ODR_25600_HZ))
    {
        rstatus = ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION;
    }
    else
    {
        rstatus = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, 1);

        if ( rstatus != ROUTINE_OK )
        {
            // ROUTINE_STATUS__COMM_FAILURE_ON_BUS
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



//----------------------------------------------------------------------
// - SECTION - routines per Zephyr sensor API
//----------------------------------------------------------------------

static int kx132_1211_attr_get(const struct device *dev,
                               enum sensor_channel chan,
                               enum sensor_attribute attr,
                               struct sensor_value *val)
{
// stub function
    return 0;
}



// REF https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html#c.sensor_channel.SENSOR_CHAN_ALL

static int kx132_1211_attr_set(const struct device *dev,
                               enum sensor_channel chan,
                               enum sensor_attribute attr,
                               const struct sensor_value *val)
{
    int rstatus = ROUTINE_OK;

    int sensor_config_requested = val->val1;

    switch (attr)
    {
// REF https://docs.zephyrproject.org/2.6.0/reference/peripherals/sensor.html#c.sensor_attribute
//        case SENSOR_ATTR_CONFIGURATION:  <- this enumerator available in Zephyr RTOS 2.7.99 but not 2.6.0 - TMH
        case SENSOR_ATTR_PRIV_START:
        {
            switch (chan)
            {
                case SENSOR_CHAN_ALL:
                {
                    switch (sensor_config_requested)
                    {
// When a sensor attribute to set is a configuration value, and it
// applies to some or all readings channels not just one, then calling
// code should call this routine with above two Zephyr enumerated
// case values, and a final value in paramter 'val' which falls in
// the local Kionix driver enumeration named 'kx132_1211_config_setting_e'.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                        case KX132_ENABLE_ASYNC_READINGS:
                        {
                            kx132_enable_asynchronous_readings(dev);
                            break;
                        }

                        case KX132_SET_OUTPUT_DATA_RATE:
                        {
                            kx132_configure_output_data_rate(dev, val);
                            break;
                        }

                        case KX132_ENABLE_SYNC_READINGS_WITH_HW_INTERRUPT:
                        {
                            kx132_enable_synchronous_reading_with_hw_interrupt(dev);
                            break;
                        }

                        case KX132_REINITIALIZE_DRDY_GPIO_PORT:
                        {
                            kx132_reinitialize_interrupt_port(dev, DEFAULT_INTERAL_OPTION_OF_ZERO);
                            break;
                        }

                        default: // ...action to take when requested config not supported
                        {
                            rstatus = ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION;
                            break;
                        }

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

    return rstatus;
}



static int kx132_1211_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
    int rstatus = 0;

    switch (channel)
    {
        case SENSOR_CHAN_KIONIX_MANUFACTURER_ID:  // a four byte value
            kx132_device_id_fetch(dev);
            break;

        case SENSOR_CHAN_KIONIX_PART_ID:          // a two byte value
            kx132_part_id_fetch(dev);
            break;

        case SENSOR_CHAN_ACCEL_X:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_acceleration_x_axis_fetch(dev);
            break;

        case SENSOR_CHAN_ACCEL_Y:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_acceleration_y_axis_fetch(dev);
            break;

        case SENSOR_CHAN_ACCEL_Z:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_acceleration_z_axis_fetch(dev);
            break;

        case SENSOR_CHAN_ACCEL_XYZ:               // read of prior three pairs of registers in sequence
            kx132_acceleration_xyz_axis_fetch(dev);
            break;

        default:
            rstatus = ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL;
    }

    return rstatus;
}




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

        default:
            routine_status = ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL;
    }

    return routine_status;
}



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
    const struct kx132_device_config *cfg = dev->config;
// - DEV 1128 -
    struct kx132_device_data *data = dev->data;
// - DEV 1128 -

    kx132_init_interface(dev);

// Optionally check chip ID here:

// Optionally set a block data update mode, if applicable, here:

// Optionally set an operating mode for power savings here:

// Optionally set a default, compile time chosen Output Data Rate (ODR) here:

// Optionally set a default range here:

// - DEV 1128 -
    data->drdy_port_status = DRDY_PORT_NOT_INITIALIZED;
// - DEV 1128 -

#ifdef CONFIG_KX132_TRIGGER
#warning "zztop"
        if (cfg->int_gpio.port) {

            printk("- MARK 1 a - kx132-1211.c finds cfg->int_gpio.port not null,\n");

//const struct gpio_dt_spec int_gpio_for_diag = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 });
////const struct gpio_dt_spec int_gpio_for_diag = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, { 0 });
//printk("- KX132 driver - interrupt GPIO port name holds '%s',\n", int_gpio_for_diag.port->name);

                if (kx132_init_interrupt(dev) < 0) {
                        LOG_ERR("KX132:  failed to initialize interrupts");
                        return -EIO;
                }
        }
        else
        {
            printk("- MARK 1 b - kx132-1211.c finds cfg->int_gpio.port null!\n");
        }
#endif // CONFIG_KX132_TRIGGER

    return 0;
}


// FEATURE - structure defined in Zephyr Project sensor.h header,
//  each member here assigned to point to functions which read and
//  write given sensor attributes, and which fetch and get (copy)
//  sensor readings most recently fetched.  
//  Zephyr's notions of 'fetch', 'get', sensor channels and related
//  terms detailed here:
// https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html

static const struct sensor_driver_api kx132_driver_api = {
    .attr_get = &kx132_1211_attr_get,
    .attr_set = &kx132_1211_attr_set,
#if CONFIG_KX132_TRIGGER
    .trigger_set = kx132_trigger_set,
#endif
    .sample_fetch = &kx132_1211_sample_fetch,
    .channel_get = &kx132_1211_channel_get
};



// # https://gcc.gnu.org/onlinedocs/gcc-3.4.6/cpp/Stringification.html . . . stringification via C macros
#define xstr(s) str(s)
#define str(s) #s
// Ejemplo:   printk("- DEV 1028 - symbol ST_IIS2DH got assigned '%s'\n", xstr(ST_IIS2DH));



// Note:  Zephyr 3.2.0 STMicro IIS2DH driver way:

#define KX132_SPI(inst)                                                                       \
        (.spi = SPI_DT_SPEC_INST_GET(                                                         \
                 0, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0),)

#define KX132_I2C(inst) (.i2c = I2C_DT_SPEC_INST_GET(inst),)



#ifdef CONFIG_KX132_TRIGGER
#warning "- DEV 1125 - assigning Zephyr DT macro value to .int_gpio of kx132_device_config structure,"
#endif


#define KX132_DEFINE(inst)                                                                    \
        static struct kx132_device_data kx132_device_data_##inst;                                 \
                                                                                              \
        static const struct kx132_device_config kx132_device_config_##inst = {                \
                COND_CODE_1(DT_INST_ON_BUS(inst, i2c), KX132_I2C(inst), ())                   \
                COND_CODE_1(DT_INST_ON_BUS(inst, spi), KX132_SPI(inst), ())                   \
                .pm = CONFIG_KX132_POWER_MODE,                                                \
\
                IF_ENABLED(CONFIG_KX132_TRIGGER_GLOBAL_THREAD,                                \
                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),)   \
                          )                                                                   \
\
        };                                                                                    \
                                                                                              \
        DEVICE_DT_INST_DEFINE(                                                                \
                              inst,                                                           \
                              kx132_1211_init,                                                \
                              NULL,                                                           \
                              &kx132_device_data_##inst,                                        \
                              &kx132_device_config_##inst,                                    \
                              POST_KERNEL,                                                    \
                              CONFIG_SENSOR_INIT_PRIORITY,                                    \
                              &kx132_driver_api                                               \
                             );


// - DEV 1125 - trying alternate interrupt bindings types, in multi-line macro about fifteen lines above here:
//                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),))
//                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, { 0 }),))


/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(KX132_DEFINE)



// --- EOF ---
