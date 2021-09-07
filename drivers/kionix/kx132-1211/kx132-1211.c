/**
 * Cornel Pump IIoT, Pulse Stage 1 firmware work
 * @author Ted Havelka (thavelka@cornellpump.com)
 * @copyright Copyright Cornell Pump 2021
 */

#define DT_DRV_COMPAT kionix_kx132_1211

#include <math.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include "kx132-1211.h"

#include <logging/log.h>
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
    struct sensor_value accel_axis_x;
    struct sensor_value accel_axis_y;
    struct sensor_value accel_axis_z;
};



static int kx132_device_id_fetch(const struct device *dev)
{
    int i = 0;
    int bus_comms_status = 0;
    struct kx132_1211_data *data_struc_ptr = (struct kx132_1211_data *)dev->data;

    uint8_t cmd[] = CMD_KX132_REQUEST_MANUFACTURER_ID;
    uint8_t rx_buf[] = {0, 0, 0, 0};


    printk("file kx132-1211.c fetching device ID . . .\n");

    // request manufacturer ID string from Kionix KX132-1211 sensor
    bus_comms_status = i2c_write_read(data_struc_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (bus_comms_status != 0)
    {
        LOG_WRN("Unable to read manufacturer ID string. Err: %i", bus_comms_status);
        return bus_comms_status;
    }

    for (i = 0; i < SIZE_MANUFACT_ID_STRING; i++)
    {
        if ((rx_buf[i] < 0x20) || (rx_buf[i] > 0x7E))
            { printk("manu' id byte %d outside ASCII printable range:  %u\n", i, rx_buf[i]); }
    }

//    printk("sensor manufacturer id: %s\n", rx_buf);

    for (i = 0; i < SIZE_MANUFACT_ID_STRING; i++)
    {
        data_struc_ptr->manufacturer_id.as_bytes[i] = rx_buf[i];
    }

    return 0;
}



static int kx132_1211_attr_get(const struct device *dev,
                          enum sensor_channel chan,
                          enum sensor_attribute attr,
                          struct sensor_value *val)
{
// stub function
    return 0;
}



static int kx132_1211_attr_set(const struct device *dev,
                          enum sensor_channel chan,
                          enum sensor_attribute attr,
                          const struct sensor_value *val)
{
// stub function
    return 0;
}



static int kx132_1211_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
    int routine_status = 0;
    struct kx132_1211_data *data = (struct kx132_1211_data *)dev->data;

    switch (channel)
    {
        case SENSOR_CHAN_KIONIX_MANUFACTURER_ID:  // a four byte value
            kx132_device_id_fetch(dev);
            break;

        default:
            routine_status = UNDEFINED_SENSOR_CHANNEL;
    }

    return routine_status;
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


        default:
            routine_status = UNDEFINED_SENSOR_CHANNEL;
    }



    return routine_status;
}



// FEATURE - initializating function in KX132-1211 driver:

//static int kx132_init(const struct device *dev)
static int kx132_1211_init(const struct device *dev)
{
    struct kx132_1211_data *data = dev->data;

    data->i2c_dev = device_get_binding(DT_INST_BUS_LABEL(0));

    if (data->i2c_dev == NULL)
    {
        LOG_ERR("Unable to get I2C Master.");
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


DEVICE_DEFINE(kx132_1211,
              DT_INST_LABEL(0),
//              kx132_init,
              kx132_1211_init,
              NULL,
              &kx132_1211_data,
              NULL,
              POST_KERNEL,
              CONFIG_SENSOR_INIT_PRIORITY,
              &kx132_api
);

// --- EOF ---
