/**
 * @project Kionix Sensor Drivers
 * @file kx132-registers.c
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */



//----------------------------------------------------------------------
// - SECTION - includes
//----------------------------------------------------------------------

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

#include "kx132-1211.h"            // to provide KX132 data struct and config struct, couple unions, and enumerated settings
#include "out-of-tree-drivers.h"   // to provide enumerated driver scoped return values



//----------------------------------------------------------------------
// - SECTION - defines
//----------------------------------------------------------------------

// 2022-11-21:  Encountered issue on 11-21, finding that with Kionix
//  accelerometer I2C "burst" read API call returns mostly garbled
//  data.  Tried i2c_write_read_dt() and this routine returns expected
//  data, so switched to this one.  Contributor Ted suspect that I2C
//  burst write API call in Zephyr 3.2.0 also fails with Kionix KX132
//  sensor.  The alternative API calls, however, have a different
//  enough implementation that calling them requires sending the sensor
//  internal register address as part of the data buffer to write,
//  in other words, all calls from this driver to the I2C write
//  routine must be adapted to include sensor internal register address
//  as first byte in the data buffer of data to write.
//
//  Contributor Ted working to find a Zephyr I2C API call which accepts
//  parameters and data buffers in the same way I2C write burst API
//  requires its parameters to be passed.
//
//  Note also per AN092-Getting_Started.pdf, page 3 of 27, KX132
//  "data ready" or drdy interrupt will be cleared automatically when
//  x,y,z acceleration readings registers are read out via an I2C
//  burst read operation.

// #define DEV_1121__KX132_I2C_BURST_WRITES_WORKING



//----------------------------------------------------------------------
// - SECTION - register read and write wrapper functions
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

//**********************************************************************
// Notes on register read, write wrappers . . .
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
//
//**********************************************************************

int kx132_enable_asynchronous_readings(const struct device *dev)
{
// Register sequence this routine chosen per AN092-Getting-Started.pdf
// from Kionix, page 2 of 27:

    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 1;
    int rstatus = ROUTINE_OK;

    rstatus  = kx132_write_reg(data->ctx, KX132_CNTL1, 0x00U, len);

    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, 0x06U, len);

    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, 0xC0U, len);

    return rstatus;
}



int kx132_enable_synchronous_reading_with_hw_interrupt(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 1;
    int rstatus = ROUTINE_OK;

// From AN092-Getting-Started.pdf:
//
// CNTL1  0x00
// INC1   0x30
// INC4   0x10
// ODCNTL 0x06
// CNTL1  0xE0

    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, 0x00U, len);

    rstatus |= kx132_write_reg(data->ctx, KX132_INC1, 0x30U, len);

    rstatus |= kx132_write_reg(data->ctx, KX132_INC4, 0x10U, len);

    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, 0x06U, len);

    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, 0xE0U, len);

    return rstatus;
}



//----------------------------------------------------------------------
// - SECTION - sensor "fetch" routines
//----------------------------------------------------------------------

// Note this routine fetches Kionix KX132-1211 Manufacturer ID string.

int kx132_device_id_fetch(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_MAN_ID, read_buffer, KX132_MAN_ID_SIZE);

    if ( rstatus != 0 )
    {
        LOG_WRN("Unable to read manufacturer ID string. Err: %i", rstatus);
        return rstatus;
    }

    for ( i = 0; i < SIZE_MANUFACT_ID_STRING; i++ )
    {
        data->manufacturer_id.as_bytes[i] = read_buffer[i];
    }

// Diag 1 here - 2022-12-05

    return rstatus;
}



int kx132_part_id_fetch(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_PART_ID, read_buffer, KX132_PART_ID_SIZE);

    if ( rstatus != 0 )
    {
        LOG_WRN("Unable to read numeric part ID . Err: %i", rstatus);
        return rstatus;
    }

    for ( i = 0; i < SIZE_PART_ID_STRING; i++ )
    {
        data->part_id.as_bytes[i] = read_buffer[i];
    }

    return rstatus;
}
 


int kx132_acceleration_x_axis_fetch(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_XOUT_L, read_buffer, 2);

    if ( rstatus != 0 )
    {
        LOG_WRN("Unable to read X axis acceleration.  Error: %i", rstatus);
        return rstatus;
    }

    for ( i = 0; i < BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS; i++ )
    {
        data->accel_axis_x[i] = read_buffer[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - X axis acceleration is %d   - DEV -\n",
      ( ( data->accel_axis_x[1] << 8 ) + data->accel_axis_x[0] ) );
    printk("- DEV - ( requested %d bytes of data starting from sensor internal addr %d ) - DEV -\n",
      sizeof(read_buffer), KX132_XOUT_L);
#endif

    return rstatus;
}



int kx132_acceleration_y_axis_fetch(const struct device *dev)
{
    struct kx132_device_data *data = (struct kx132_device_data*)dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_YOUT_L, read_buffer, 2);

    if ( rstatus != 0 )
    {
        LOG_WRN("Unable to read Y axis acceleration.  Error: %i", rstatus);
        return rstatus;
    }

    for ( i = 0; i < BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS; i++ )
    {
        data->accel_axis_y[i] = read_buffer[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - Y axis acceleration is %d\n",
      ( ( data->accel_axis_y[1] << 8 ) + data->accel_axis_y[0] ) );
#endif

    return rstatus;
}



int kx132_acceleration_z_axis_fetch(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_ZOUT_L, read_buffer, 2);

    if ( rstatus != 0 )
    {
        LOG_WRN("Unable to read Z axis acceleration.  Error: %i", rstatus);
        return rstatus;
    }

    for ( i = 0; i < BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS; i++ )
    {
        data->accel_axis_z[i] = read_buffer[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - Z axis acceleration is %d\n",
      ( ( data_struc_ptr->accel_axis_z[1] << 8 ) + data_struc_ptr->accel_axis_z[0] ) );
#endif

    return rstatus;
}



int kx132_acceleration_xyz_axis_fetch(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;

    int rstatus = 0;


    rstatus = kx132_read_reg(data->ctx, KX132_XOUT_L, read_buffer, 6);

    if ( rstatus != 0 )
    {
        LOG_WRN("Unable to read X,Y,Z accelerometer axes data. Err: %i", rstatus);
        return rstatus;
    }

    for (i = 0; i < (BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS + 0); i++)
        { data->accel_axis_x[i - 0] = read_buffer[i]; }
    for (i = 2; i < (BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS + 2); i++)
        { data->accel_axis_y[i - 2] = read_buffer[i]; }
    for (i = 4; i < (BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS + 4); i++)
        { data->accel_axis_z[i - 4] = read_buffer[i]; }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - X, Y and Z accelerations are %d, %d, %d   - DEV -\n",
      ( ( data->accel_axis_x[1] << 8 ) + data->accel_axis_x[0] ),
      ( ( data->accel_axis_y[1] << 8 ) + data->accel_axis_y[0] ),
      ( ( data->accel_axis_z[1] << 8 ) + data->accel_axis_z[0] )
    );
    printk("- DEV - ( requested %d bytes of data starting from sensor internal addr %d ) - DEV -\n",
      sizeof(read_buffer), KX132_XOUT_L);
#endif

    return rstatus;
}




//----------------------------------------------------------------------
// - SECTION - notes
//----------------------------------------------------------------------

#if 0
    if ( rstatus != 0 )
    {
        LOG_WRN("- ERROR - unable to write CNTL register, got bus error:  %i", rstatus);
        return rstatus;
    }
#endif


#if 0

// Diag 1 here - 2022-12-05
#if 0
char lbuf[240];
snprintf(lbuf, sizeof(lbuf), "- DEV 1120 - in KX132 driver, manufacturer ID '%c %c %c %c'\n",
  read_buffer[0],
  read_buffer[1],
  read_buffer[2],
  read_buffer[3]
  );
printk("%s", lbuf);
#endif

#endif



// --- EOF ---
