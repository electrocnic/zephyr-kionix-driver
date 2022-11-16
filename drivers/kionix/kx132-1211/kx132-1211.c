/**
 * @project Kionix Sensor Drivers
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */

#define DT_DRV_COMPAT kionix_kx132_1211

#include <math.h>

#include <zephyr/device.h>          // 2022-11-10 was <device.h>
#include <zephyr/drivers/i2c.h>     // 2022-11-10 was <drivers/i2c.h>
#include <zephyr/drivers/sensor.h>  // 2022-11-10 was <drivers/sensor.h>

#include "kx132-1211.h"
#include "out-of-tree-drivers.h"
#include "development-defines.h"

#include <zephyr/logging/log.h>     // 2022-11-10 was <logging/log.h>
LOG_MODULE_REGISTER(kx132, CONFIG_SENSOR_LOG_LEVEL);


#define SIZE_MANUFACT_ID_STRING (4)
#define SIZE_PART_ID_STRING (2)

union string_union_type__manufacturer_id
{
    char as_string[SIZE_MANUFACT_ID_STRING];
    uint8_t as_bytes[SIZE_MANUFACT_ID_STRING];
    uint32_t as_32_bit_integer;
};

union string_union_type__part_id
{
    char as_string[SIZE_MANUFACT_ID_STRING];
    uint8_t as_bytes[SIZE_MANUFACT_ID_STRING];
    uint16_t as_16_bit_integer;
};


//----------------------------------------------------------------------
// Note:  struct sensor_value is defined in Zephyr's sensor.h header
// file.  In Nordic ncs v1.6.1 SDK this file found in
// nsc/v1.6.1/zephyr/include/drivers
//
// Kionix KX132-1211 register definitions found in KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf.
//----------------------------------------------------------------------

struct kx132_1211_data
{
    const struct device *i2c_dev;
    union string_union_type__manufacturer_id manufacturer_id;
    union string_union_type__part_id part_id;
// Following three data members are written with LSB, MSB of respective accelerometer readings:
    uint8_t accel_axis_x[BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS];
    uint8_t accel_axis_y[BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS];
    uint8_t accel_axis_z[BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS];
// Following data member written with LSB, MSB of allaccelerometer readings X, Y and Z axes:
    uint8_t accel_axis_xyz[BYTE_COUNT_OF_KX132_ACCELERATION_READING_THREE_AXES];
// QUESTION:  any reason we need to align data on four byte boundary? - TMH
//    uint8_t padding[BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS];
};



//----------------------------------------------------------------------
// - SECTION - Kionix sensor specific configuration routines
//----------------------------------------------------------------------

static int kx132_enable_asynchronous_readings(const struct device *dev)
{
// Note per Sensirion datasheet sensor SHTC3 sleep command is 0xB098,
// given on page 6 of 14.

    int comms_status = 0;
    struct kx132_1211_data *data = dev->data;
    int status = ROUTINE_OK;

// Per AN092-Getting-Started.pdf from Kionix, page 2 of 27:
    uint8_t cmd_cntl_00[] = { 0x1B, 0x00 };

#ifdef _DEV_ENABLE_PRINTK
#ifdef DEV__KX132_SHOW_CONFIG_COMMANDS_BEFORE_SENDING
    printk("writing {%02X, %02X . . .\n", cmd_cntl_00[0], cmd_cntl_00[1]);
#endif
#endif
    comms_status = i2c_write(data->i2c_dev, cmd_cntl_00, sizeof(cmd_cntl_00), DT_INST_REG_ADDR(0));

    if (comms_status != 0)
    {
        LOG_WRN("- ERROR - Unable to write CNTL register, but error:  %i", comms_status);
        return comms_status;
    }

    k_msleep(100);


// Next commands (register writes) are:

// KX132-1211 Register CTRL1:
    uint8_t cmd_odcntl_06[] = { 0x21, 0x06 };
    comms_status = i2c_write(data->i2c_dev, cmd_odcntl_06, sizeof(cmd_odcntl_06), DT_INST_REG_ADDR(0));
    k_msleep(100);

// KX132-1211 Register INC1:
    uint8_t cmd_cntl_c0[] = { 0x1B, 0xC0 };
    comms_status = i2c_write(data->i2c_dev, cmd_cntl_c0, sizeof(cmd_cntl_c0), DT_INST_REG_ADDR(0));
    k_msleep(100);

#ifdef _DEV_ENABLE_PRINTK
    printk("attempt to configure async reading complete.\n");
#endif

    return status;
}



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
    int status = ROUTINE_OK;
    uint8_t cmd[] = { KX132_1211_CONFIG_REGISTER__ODCNTL };
    struct kx132_1211_data *data_struc_ptr = (struct kx132_1211_data *)dev->data;
    uint8_t scratch_pad_byte = 0;
    uint8_t odcntl_as_found = 0;

    if ((val->val2 < KX132_ODR__0P781_HZ) || (val->val2 > KX132_ODR_25600_HZ))
    {
        status = ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION;
    }
    else
    {

// Read present value of KX132-1211 output data rate control register:
        status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), &scratch_pad_byte, sizeof(scratch_pad_byte));

// NEED TO CHECK FOR I2C COMMS ERROR RETURN VALUE
// if ( comms error ) { . . . }
// else

        odcntl_as_found = scratch_pad_byte;  // save original value - may not be needed, review this - TMH
        scratch_pad_byte &= 0xF0;            // mask to erase OSA3:OSA0
        scratch_pad_byte |= val->val2;       // write bit pattern to set output data rate

        uint8_t cmd_odcntl[] = { KX132_1211_CONFIG_REGISTER__ODCNTL, scratch_pad_byte };
        status = i2c_write(data_struc_ptr->i2c_dev, cmd_odcntl, sizeof(cmd_odcntl), DT_INST_REG_ADDR(0));

//        k_msleep(100);
// QUESTION:  how quickly can we write KX132 config registers?  We're
// not doing so often but we are writing several at start up time.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    }

    return status;
}




//----------------------------------------------------------------------
// - SECTION - Kionix sensor specific readings and data fetch routines
//----------------------------------------------------------------------

// Note this routine fetches Kionix KX132-1211 Manufacturer ID string.

static int kx132_device_id_fetch(const struct device *dev)
{
    struct kx132_1211_data *data_struc_ptr = (struct kx132_1211_data *)dev->data;
    uint8_t cmd[] = CMD_KX132_REQUEST_MANUFACTURER_ID;
    uint8_t rx_buf[] = {0, 0, 0, 0};
    int bus_comms_status = 0;
    int i = 0;

// request manufacturer ID string from Kionix KX132-1211 sensor
    bus_comms_status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (bus_comms_status != 0)
    {
        LOG_WRN("Unable to read manufacturer ID string. Err: %i", bus_comms_status);
        return bus_comms_status;
    }

#ifdef _DEV_ENABLE_PRINTK
    for (i = 0; i < SIZE_MANUFACT_ID_STRING; i++)
    {
        if ((rx_buf[i] < 0x20) || (rx_buf[i] > 0x7E))
            { printk("manufacturer id byte %d outside ASCII printable range:  %u\n", i, rx_buf[i]); }
    }
#endif

    for (i = 0; i < SIZE_MANUFACT_ID_STRING; i++)
    {
        data_struc_ptr->manufacturer_id.as_bytes[i] = rx_buf[i];
    }

    return 0;
}



static int kx132_part_id_fetch(const struct device *dev)
{
    int bus_comms_status = 0;
    struct kx132_1211_data *data_struc_ptr = (struct kx132_1211_data *)dev->data;
    uint8_t cmd[] = CMD_KX132_REQUEST_PART_ID;
    uint8_t rx_buf[] = {0, 0};
    int i = 0;

// request part ID string from Kionix KX132-1211 sensor:
    bus_comms_status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (bus_comms_status != 0)
    {
        LOG_WRN("Unable to read numeric part ID . Err: %i", bus_comms_status);
        return bus_comms_status;
    }

    for (i = 0; i < SIZE_PART_ID_STRING; i++)
    {
        data_struc_ptr->part_id.as_bytes[i] = rx_buf[i];
    }

    return 0;
}
 


static int kx132_acceleration_x_axis_fetch(const struct device *dev)
{
    int comms_status = 0;
    struct kx132_1211_data* data_struc_ptr = (struct kx132_1211_data*)dev->data;
    uint8_t cmd[] = { KX132_1211_ACCELEROMETER_XOUT_L };
    uint8_t rx_buf[] = {0, 0};
    int i = 0;

    comms_status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (comms_status != 0)
    {
        LOG_WRN("Unable to read X axis acceleration.  Error: %i", comms_status);
        return comms_status;
    }

    for (i = 0; i < BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS; i++)
    {
        data_struc_ptr->accel_axis_x[i] = rx_buf[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - X axis acceleration is %d   - DEV -\n",
      ( ( data_struc_ptr->accel_axis_x[1] << 8 ) + data_struc_ptr->accel_axis_x[0] ) );
    printk("- DEV - ( requested %d bytes of data starting from sensor internal addr %d ) - DEV -\n",
      sizeof(rx_buf), cmd[0]);
#endif

    return comms_status;
}


static int kx132_acceleration_y_axis_fetch(const struct device *dev)
{
    int comms_status = 0;
    struct kx132_1211_data* data_struc_ptr = (struct kx132_1211_data*)dev->data;
    uint8_t cmd[] = { KX132_1211_ACCELEROMETER_YOUT_L };
    uint8_t rx_buf[] = {0, 0};
    int i = 0;

    comms_status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (comms_status != 0)
    {
        LOG_WRN("Unable to read Y axis acceleration.  Error: %i", comms_status);
        return comms_status;
    }

    for (i = 0; i < BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS; i++)
    {
        data_struc_ptr->accel_axis_y[i] = rx_buf[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - Y axis acceleration is %d   - DEV -\n",
      ( ( data_struc_ptr->accel_axis_y[1] << 8 ) + data_struc_ptr->accel_axis_y[0] ) );
#endif

    return comms_status;
}





static int kx132_acceleration_z_axis_fetch(const struct device *dev)
{
    int comms_status = 0;

    struct kx132_1211_data* data_struc_ptr = (struct kx132_1211_data*)dev->data;
    uint8_t cmd[] = { KX132_1211_ACCELEROMETER_ZOUT_L };
    uint8_t rx_buf[] = {0, 0};
    int i = 0;

    comms_status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (comms_status != 0)
    {
        LOG_WRN("Unable to read Z axis acceleration.  Error: %i", comms_status);
        return comms_status;
    }

    for (i = 0; i < BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS; i++)
    {
        data_struc_ptr->accel_axis_z[i] = rx_buf[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - Z axis acceleration is %d   - DEV -\n",
      ( ( data_struc_ptr->accel_axis_z[1] << 8 ) + data_struc_ptr->accel_axis_z[0] ) );
#endif

    return comms_status;
}





static int kx132_acceleration_xyz_axis_fetch(const struct device *dev)
{
// stub
    int bus_comms_status = 0;
    struct kx132_1211_data* data_struc_ptr = (struct kx132_1211_data*)dev->data;
    uint8_t cmd[] = { 0x08 }; // starting address of X acceleration reading, LSB of two byte value
    uint8_t rx_buf[] = {0, 0,  0, 0,  0, 0};
    int i = 0;

    bus_comms_status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (bus_comms_status != 0)
    {
        LOG_WRN("Unable to read numeric part ID . Err: %i", bus_comms_status);
        return bus_comms_status;
    }

    for (i = 0; i < (BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS + 0); i++)
        { data_struc_ptr->accel_axis_x[i - 0] = rx_buf[i]; }
    for (i = 2; i < (BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS + 2); i++)
        { data_struc_ptr->accel_axis_y[i - 2] = rx_buf[i]; }
    for (i = 4; i < (BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS + 4); i++)
        { data_struc_ptr->accel_axis_z[i - 4] = rx_buf[i]; }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - X, Y and Z accelerations are %d, %d, %d   - DEV -\n",
      ( ( data_struc_ptr->accel_axis_x[1] << 8 ) + data_struc_ptr->accel_axis_x[0] ),
      ( ( data_struc_ptr->accel_axis_y[1] << 8 ) + data_struc_ptr->accel_axis_y[0] ),
      ( ( data_struc_ptr->accel_axis_z[1] << 8 ) + data_struc_ptr->accel_axis_z[0] )
    );
    printk("- DEV - ( requested %d bytes of data starting from sensor internal addr %d ) - DEV -\n",
      sizeof(rx_buf), cmd[0]);
#endif

    return bus_comms_status;
}



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
    int status = ROUTINE_OK;

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

                        default: // ...action to take when requested config not supported
                        {
                            status = ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION;
                            break;
                        }

                    } // close scope of switch(val)
                }

                default: // ...action to take with unrecognized sensor channel
                {
                    status = ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL;
                    break;
                }

            } // close scope of switch(chan)
        }

        default: // ...default action to take with unrecognized sensor attribute
        {
            status = ROUTINE_STATUS__UNDEFINED_SENSOR_ATTRIBUTE;
            break;
        }
    } // close scope of switch(attr)

    return status;
}



static int kx132_1211_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
    int status = 0;
//    struct kx132_1211_data *data = (struct kx132_1211_data *)dev->data;

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
            status = ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL;
    }

    return status;
}




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

// Following design pattern in J Wolff AQW driver for Sensirion SHTC3 sensor,
// begin with a pointer to a data structure that's crafted to support
// our new-to-us Kionix sensor:
    struct kx132_1211_data *data = (struct kx132_1211_data *)dev->data;

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

        case SENSOR_CHAN_ACCEL_X:          // a two byte value
            val->val1 = ( ( data->accel_axis_x[1] << 8 ) | ( data->accel_axis_x[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_Y:          // a two byte value
            val->val1 = ( ( data->accel_axis_y[1] << 8 ) | ( data->accel_axis_y[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_Z:          // a two byte value
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



// FEATURE - initializating function in KX132-1211 driver:

//static int kx132_init(const struct device *dev)
static int kx132_1211_init(const struct device *dev)
{
    struct kx132_1211_data *data = dev->data;

// 2022-11-10 REF https://docs.zephyrproject.org/latest/build/dts/api/api.html#c.DT_INST_BUS_LABEL
//    data->i2c_dev = device_get_binding(DT_INST_BUS_LABEL(0));
//    data->i2c_dev = device_get_binding(DEVICE_DT_GET(DT_INST_BUS(0)));  // expected 'const char *' but argument is of type 'const struct device *'
    data->i2c_dev = DEVICE_DT_GET(DT_INST_BUS(0));

    if (data->i2c_dev == NULL)
    {
        LOG_ERR("Unable to get I2C controller while initializing KX132-1211 device instance.");
        return -EINVAL;
    }

    return 0;
}


// FEATURE - structure defined in Zephyr Project sensor.h header,
//  each member here assigned to point to functions which read and
//  write given sensor attributes, and which fetch and get (copy)
//  sensor readings most recently fetched.  
//  Zephyr's notions of 'fetch', 'get', sensor channels and related
//  terms detailed here:
// https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html

static const struct sensor_driver_api kx132_api = {
    .attr_get = &kx132_1211_attr_get,
    .attr_set = &kx132_1211_attr_set,
    .sample_fetch = &kx132_1211_sample_fetch,
    .channel_get = &kx132_1211_channel_get
};


static struct kx132_1211_data kx132_1211_data;

// NOTE Zephyr documentation says "Use DEVICE_DEFINE() only when device is not allocated from a devicetree node."

#if 0
DEVICE_DEFINE(kx132_1211,                    // dev_id
              DT_INST_LABEL(0),              // name
              kx132_1211_init,               // init function
              NULL,                          // pm - pointer to power management resources
              &kx132_1211_data,              // data - pointer to device' private mutable data
              NULL,                          // config - pointer to device' private constant data
              POST_KERNEL,                   // level
              CONFIG_SENSOR_INIT_PRIORITY,   // priority
              &kx132_api                     // API
);
#endif


// # https://gcc.gnu.org/onlinedocs/gcc-3.4.6/cpp/Stringification.html . . . stringification via C macros
#define xstr(s) str(s)
#define str(s) #s
// Ejemplo:   printk("- DEV 1028 - symbol ST_IIS2DH got assigned '%s'\n", xstr(ST_IIS2DH));


// REF https://docs.zephyrproject.org/latest/kernel/drivers/index.html#c.DEVICE_DT_DEFINE

#if 1

#define macro(x) x

//#if (CONFIG_DEFINE_DEVICE_KX132_VIA_DT_NODELABEL == 0)
#warning "- DEV 1115 - defining device KX132-1211 via Zephyr DEVICE_NODELABEL() macro . . ."

DEVICE_DT_DEFINE(
//                 DT_NODELABEL(kionix_sensor),  // node_id . . . works, but nodelabel is hard-coded

//                 DT_NODELABEL(macro(CONFIG_KX132_NODELABEL_VALUE)),

#define y macro(CONFIG_KX132_NODELABEL_VALUE)
                 DT_NODELABEL(macro(y)),

                 kx132_1211_init,              // init function
                 NULL,                         // pm
                 &kx132_1211_data,             // data
                 NULL,                         // config
                 POST_KERNEL,                  // level
                 CONFIG_SENSOR_INIT_PRIORITY,  // priority
                 &kx132_api                    // API
);
#endif

#if 0
//#if (CONFIG_DEFINE_DEVICE_KX132_VIA_DEVICE_DT_INST_DEFINE == 0)
#warning "- DEV 1115 - defining device KX132-1211 via Zephyr DEVICE_DT_INST_DEFINE() macro . . ."


#define KX132_1211_DEFINE(inst)                                  \
    static struct kx132_1211_data kx132_1211_data_##inst;        \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          kx132_1211_init,                       \
                          NULL,                                  \
                          &kx132_1211_data_##inst,               \
                          NULL,                                  \
                          POST_KERNEL,                           \
                          CONFIG_SENSOR_INIT_PRIORITY,           \
                          &kx132_api);

/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(KX132_1211_DEFINE)
#endif





//----------------------------------------------------------------------
// - SECTION - notes and reference code
//----------------------------------------------------------------------

#if 0
## Sample driver code from Jared Wolff https://github.com/circuitdojo/air-quality-wing-zephyr-drivers/blob/main/drivers/sgp40/sgp40.c:
#define SGP40_DEFINE(inst)                                       \
    static struct sgp40_data sgp40_data_##inst;                  \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          sgp40_init, NULL,                      \
                          &sgp40_data_##inst, NULL, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &sgp40_api);

/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(SGP40_DEFINE)
#endif


#if 0
#define nodelabel_value KX132_NODELABEL_VALUE
//                 DT_NODELABEL(nodelabel_value),  // node_id . . . works, but nodelabel is hard-coded
// [app_workspace]/zephyr/include/zephyr/devicetree.h:190:36: error: 'DT_N_NODELABEL_KX132_NODELABEL_VALUE_FULL_NAME' undeclared here (not in a function)

//                 DT_NODELABEL(xstr(KX132_NODELABEL_VALUE)),  // node_id
// [app_workspace]/zephyr/include/zephyr/devicetree.h:190:36: error: pasting "DT_N_NODELABEL_" and ""KX132_NODELABEL_VALUE"" does not give a valid preprocessing token

//                 DT_NODELABEL(CONFIG_KX132_NODELABEL_VALUE),  // node_id . . . works, but nodelabel is hard-coded
// [app_workspace]/zephyr/include/zephyr/devicetree.h:190:36: error: pasting "DT_N_NODELABEL_" and ""kionix_sensor"" does not give a valid preprocessing token

//                 DT_NODELABEL(str(CONFIG_KX132_NODELABEL_VALUE)),  // node_id . . . works, but nodelabel is hard-coded
// build time error here too, just not noted - TMH

//                 DT_NODELABEL(str(KXNL)),      // node_id . . . works, but nodelabel is hard-coded
// [app_workspace]/kionix-drivers/drivers/kionix/kx132-1211/kx132-1211.c:678:1: error: pasting ""KXNL"" and "_EXISTS" does not give a valid preprocessing token

//                 DT_NODELABEL(xstr(CONFIG_KXNL)),      // node_id . . . works, but nodelabel is hard-coded
// [app_workspace]/kionix-drivers/drivers/kionix/kx132-1211/kx132-1211.c:681:1: error: pasting ""\"kionix_sensor\""" and "_EXISTS" does not give a valid preprocessing token

//                 DT_NODELABEL(CONFIG_KXNL),    // node_id . . . works, but nodelabel is hard-coded
// samples/iis2dh-driver-demo/build/zephyr/include/generated/autoconf.h:71:21: error: pasting ""kionix_sensor"" and "_EXISTS" does not give a valid preprocessing token
#endif



// --- EOF ---
