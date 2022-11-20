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

#include <zephyr/logging/log.h>     // 2022-11-10 was <logging/log.h>

LOG_MODULE_REGISTER(KX132, CONFIG_SENSOR_LOG_LEVEL); // <-- NEED to review LOG_MODULE_DECLARE() due to this line

// NEED to review LOG_MODULE_DECLARE() stanzas in other driver sources,
//  to  assure those are not in conflict with this related Zephyr
//  logging macro - TMH

#include "kx132-1211.h"
#include "kx132-registers.h"
#include "out-of-tree-drivers.h"
#include "development-defines.h"



//----------------------------------------------------------------------
// Note:  struct sensor_value is defined in Zephyr's sensor.h header
// file.  In Nordic ncs v1.6.1 SDK this file found in
// nsc/v1.6.1/zephyr/include/drivers
//
// Kionix KX132-1211 register definitions found in KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf.
//----------------------------------------------------------------------




/**
  * @defgroup  KX132_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t kx132_read_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
  int32_t rstatus;

  rstatus = ctx->read_reg(ctx->handle, reg, data, len);

  return rstatus;
}


/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t kx132_write_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
  int32_t rstatus;

  rstatus = ctx->write_reg(ctx->handle, reg, data, len);

  return rstatus;
}





//----------------------------------------------------------------------
// - SECTION - Kionix sensor specific configuration routines
//----------------------------------------------------------------------

static int kx132_enable_asynchronous_readings(const struct device *dev)
{
// Register sequence this routine chosen per AN092-Getting-Started.pdf
// from Kionix, page 2 of 27:

    struct kx132_1211_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    int rstatus = ROUTINE_OK;

//
// Notes on register read, write wrappers . . .
//
//
//                                      I2C ctrlr, regaddr, data buffer, length data to write
//                                           |       |          |                |
// 112 typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
// 113 typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
//                                           |       |          |                |
//                                      I2C ctrlr, regaddr, data buffer, length data to read
//
// IIS2dh example call
//
//    ret = iis2dh_read_reg(ctx, IIS2DH_WHO_AM_I, buff, 1);

    rstatus = kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, 1);

    if ( rstatus != 0 )
    {
        LOG_WRN("- ERROR - unable to write CNTL register, got bus error:  %i", rstatus);
        return rstatus;
    }

// KX132-1211 Register CTRL1:
    reg_val_to_write = 0x06U;
    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, 1);

// KX132-1211 Register INC1:
//    uint8_t cmd_cntl_c0[] = { 0x1B, 0xC0 };
    reg_val_to_write = 0xC0U;
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, 1);

    return rstatus;
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
    struct kx132_1211_data *data_struc_ptr = (struct kx132_1211_data *)dev->data;
//    struct kx132_1211_data *data_struc_ptr = *dev->data;

    uint8_t reg_val_to_read = 0x00U;
    uint8_t *read_buffer = &reg_val_to_read;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;

    int rstatus = ROUTINE_OK;



    if ((val->val2 < KX132_ODR__0P781_HZ) || (val->val2 > KX132_ODR_25600_HZ))
    {
        rstatus = ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION;
    }
    else
    {

//
//    reg_val_to_write = 0x06U;
//    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, 1);
//

// Read present value of KX132-1211 output data rate control register:
//        rstatus = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
//                         cmd, sizeof(cmd), &scratch_pad_byte, sizeof(scratch_pad_byte));

        rstatus = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, 1);

        if ( rstatus != ROUTINE_OK )
        {
            // ROUTINE_STATUS__COMM_FAILURE_ON_BUS
        }
        else
        {

            write_buffer = read_buffer;  // save original value - may not be needed, review this - TMH
            write_buffer &= 0xF0;            // mask to erase OSA3:OSA0
            write_buffer |= val->val2;       // write bit pattern to set output data rate

            rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, 1);
        }
    }

    return rstatus;
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
        LOG_WRN("Unable to read X,Y,Z accelerometer axes data. Err: %i", bus_comms_status);
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

static int kx132_1211_init(const struct device *dev)
{
//    struct kx132_1211_data *data = dev->data;
//
//    data->i2c_dev = DEVICE_DT_GET(DT_INST_BUS(0));
//
//    if (data->i2c_dev == NULL)
//    {
//        LOG_ERR("Unable to get I2C controller while initializing KX132-1211 device instance.");
//        return -EINVAL;
//    }

    kx132_init_interface(dev);

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


//static struct kx132_1211_data kx132_1211_data;

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
              &kx132_driver_api                     // API
);
#endif


// # https://gcc.gnu.org/onlinedocs/gcc-3.4.6/cpp/Stringification.html . . . stringification via C macros
#define xstr(s) str(s)
#define str(s) #s
// Ejemplo:   printk("- DEV 1028 - symbol ST_IIS2DH got assigned '%s'\n", xstr(ST_IIS2DH));


// REF https://docs.zephyrproject.org/latest/kernel/drivers/index.html#c.DEVICE_DT_DEFINE

#if 0
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// NOTE:  intermediate device definition, which calls DT_NODELABEL to
//   obtain a device tree node identifier.
//
// NOTE:  this way replaced original DT_LABEL() in app way, which contributor
//   Ted first got working.  DT_LABEL() is deprecated as of Zephyr 3.2.0
//   or earlier, but use of DT_NODELABEL() couples device tree overlays
//   with this driver -- not a desirable design choice!

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
                 &kx132_driver_api                    // API
);

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#endif


#if 0
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// NOTE:  following device define follows CircuitDojo way.  Contributor
//   Ted must review whether Jared Wolff's driver supports both I2C and
//   SPI bus communications interfaces:

#define KX132_1211_DEFINE(inst)                                  \
    static struct kx132_1211_data kx132_1211_data_##inst;        \
                                                                 \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          kx132_1211_init,                       \
                          NULL,                                  \
                          &kx132_1211_data_##inst,               \
                          NULL,                                  \
                          POST_KERNEL,                           \
                          CONFIG_SENSOR_INIT_PRIORITY,           \
                          &kx132_driver_api);
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#endif



// Note:  Zephyr 3.2.0 STMicro IIS2DH driver way:

#define KX132_SPI(inst)                                                                       \
        (.spi = SPI_DT_SPEC_INST_GET(                                                         \
                 0, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0),)

#define KX132_I2C(inst) (.i2c = I2C_DT_SPEC_INST_GET(inst),)

#define KX132_DEFINE(inst)                                                                    \
        static struct kx132_1211_data kx132_1211_data_##inst;                                 \
                                                                                              \
        static const struct kx132_device_config kx132_device_config_##inst = {                \
                COND_CODE_1(DT_INST_ON_BUS(inst, i2c), KX132_I2C(inst), ())                   \
                COND_CODE_1(DT_INST_ON_BUS(inst, spi), KX132_SPI(inst), ())                   \
                .pm = CONFIG_KX132_POWER_MODE,                                                \
                IF_ENABLED(CONFIG_KX132_TRIGGER,                                              \
                           (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),))  \
        };                                                                                    \
                                                                                              \
        DEVICE_DT_INST_DEFINE(                                                                \
                              inst,                                                           \
                              kx132_1211_init,                                                \
                              NULL,                                                           \
                              &kx132_1211_data_##inst,                                        \
                              &kx132_device_config_##inst,                                    \
                              POST_KERNEL,                                                    \
                              CONFIG_SENSOR_INIT_PRIORITY,                                    \
                              &kx132_driver_api                                               \
                             );

/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(KX132_DEFINE)


// --- EOF ---
