/**
 * Cornel Pump IIoT, Pulse Stage 1 firmware work
 * @author Ted Havelka (thavelka@cornellpump.com)
 * @copyright Copyright Cornell Pump 2021
 */

#define DT_DRV_COMPAT kionix_kx132

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
};

union string_union_type__part_id
{
    char as_string[SIZE_MANUFACT_ID_STRING];
    uint8_t as_bytes[SIZE_MANUFACT_ID_STRING];
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
    int status_de_comms = 0;
    struct kx132_1211_data *data_str_ptr = (struct kx132_1211_data *)dev->data;

    uint8_t cmd[] = CMD_KX132_REQUEST_MANUFACTURER_ID;
    uint8_t rx_buf[] = {0, 0, 0, 0};


    // request manufacturer ID string from Kionix KX132-1211 sensor
    status_de_comms = i2c_write_read(data_str_ptr->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
    if (status_de_comms != 0)
    {
        LOG_WRN("Unable to read manufacturer ID string. Err: %i", error);
        return error;
    }

    printk("sensor manufacturer id: %s\n", rx_buf);

    int i = 0;
    for (i = 0; i < SIZE_MANUFACT_ID_STRING; i++)
    {
        data_str_ptr->manufacturer_id.as_bytes[i] = rx_buf[i];
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
// stub function   
    return 0;
}


static int kx132_1211_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
// stub function   
    return 0;
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
