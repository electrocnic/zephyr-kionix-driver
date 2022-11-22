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
//#include "kx132-register-interface.h" // to provide kionix_ctx_t type, KX132 "context" data structure
#include "out-of-tree-drivers.h"   // to provide enumerated driver scoped return values



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

int kx132_enable_asynchronous_readings(const struct device *dev)
{
// Register sequence this routine chosen per AN092-Getting-Started.pdf
// from Kionix, page 2 of 27:

    struct kx132_1211_data *data = dev->data;
#ifdef DEV_1121__KX132_I2C_BURST_WRITES_WORKING 
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 1;
#else
    uint8_t reg_addr_and_value_to_write[] = { KX132_CNTL1, 0x00U };
    uint8_t *write_buffer = reg_addr_and_value_to_write;
    uint32_t len = 2;
#endif
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

    rstatus = kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    if ( rstatus != 0 )
    {
        LOG_WRN("- ERROR - unable to write CNTL register, got bus error:  %i", rstatus);
        return rstatus;
    }

// KX132-1211 Register CTRL1:
#ifdef DEV_1121__KX132_I2C_BURST_WRITES_WORKING 
    reg_val_to_write = 0x06U;
#else
    write_buffer[0] = KX132_ODCNTL;
    write_buffer[1] = 0x06U;
#endif
    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);

// KX132-1211 Register INC1:
#ifdef DEV_1121__KX132_I2C_BURST_WRITES_WORKING 
    reg_val_to_write = 0xC0U;
#else
    write_buffer[0] = KX132_CNTL1;
    write_buffer[1] = 0xC0U;
#endif
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    return rstatus;
}



