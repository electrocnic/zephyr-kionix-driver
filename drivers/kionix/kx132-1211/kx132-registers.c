/**
 * @project Kionix Sensor Drivers
 * @file kx132-registers.c
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */



//----------------------------------------------------------------------
// - SECTION - notes
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



//----------------------------------------------------------------------
// - SECTION - includes
//----------------------------------------------------------------------

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

#include "kx132-1211.h"            // to provide KX132 data struct and config struct, a couple
                                   //  unions, and enumerated settings
#include "out-of-tree-drivers.h"   // to provide enumerated driver scoped return values

#include "kx132-registers.h"       //



//----------------------------------------------------------------------
// - SECTION - defines
//----------------------------------------------------------------------

//#define DEV_1121__KX132_I2C_BURST_WRITES_WORKING

//#define DEV_0116

//#define DEV_0118

//#define DEV__SOFTWARE_RESET_DIAG

//#define DEV__ODCNTL_UPDATE_DIAG

//#define KX132_DRIVER__SET_ODR_OF_50_HZ_IN_ENABLE_WATERMARK_SEQUENCE



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

//**********************************************************************
//
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
// - SECTION - KX132 multi-register config routines
//----------------------------------------------------------------------

// Per software reset description in Kionix TN027-Power-On-Procedure.pdf:

int kx132_software_reset(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;

    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
// NEED to review size of read buffer here in KX132 software reset routine:
    uint8_t reg_val_to_read[] = {0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;

    uint32_t len = 2;
    int rstatus = ROUTINE_OK;


    k_msleep(PERIOD_TO_POWER_UP_IN_MS);

    reg_val_to_write = 0x00U;
    rstatus = kx132_write_reg(data->ctx, KX132_UNNAMED_SW_RESET_REG_0x7F, write_buffer, len);

    reg_val_to_write = 0x00U;
    rstatus += kx132_write_reg(data->ctx, KX132_CNTL2, write_buffer, len);

    reg_val_to_write = 0x80U;
    rstatus += kx132_write_reg(data->ctx, KX132_CNTL2, write_buffer, len);

    k_msleep(PERIOD_TO_PERFORM_SW_RESET_IN_MS);


    rstatus += kx132_read_reg(data->ctx, KX132_WHO_AM_I, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_who_am_i = read_buffer[0];
#ifdef DEV__SOFTWARE_RESET_DIAG
    printk("- KX132 driver - WHO_AM_I register holds 0x%02X\n", data->shadow_reg_who_am_i);
#endif

    rstatus += kx132_read_reg(data->ctx, KX132_COTR, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_cotr = read_buffer[0];
#ifdef DEV__SOFTWARE_RESET_DIAG
    printk("- KX132 driver - COTR register holds 0x%02X\n", data->shadow_reg_cotr);
    printk("- KX132 driver - low level reads and write so far return status of %u\n", rstatus);
#endif

// 2023-01-18
// NEED to amend return values to capture bus transaction errors,
// and yet support unexpected 'who am i' and 'cotr' values - TMH

    if ( data->shadow_reg_who_am_i != KX132_WHO_AM_I_EXPECTED_VALUE )
        { rstatus = ROUTINE_STATUS__UNEXPECTED_VALUE_WHO_AM_I; }

    if ( data->shadow_reg_cotr != KX132_COTR_EXPECTED_VALUE )
        { rstatus = ROUTINE_STATUS__UNEXPECTED_VALUE_COTR; }

#ifdef DEV__SOFTWARE_RESET_DIAG
    printk("- KX132 driver - after checks of who_am_i and cotr values, rstatus holds %u\n\n", rstatus);
#endif
    return rstatus;

} // end routine kx132_software_reset()



int kx132_enable_asynchronous_readings(const struct device *dev)
{
// Register sequence this routine chosen per AN092-Getting-Started.pdf
// from Kionix, page 2 of 27:

    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;
    int rstatus = ROUTINE_OK;

    reg_val_to_write = 0x00U;
    rstatus  = kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    reg_val_to_write = 0x06U;
    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);

    reg_val_to_write = 0xC0U;
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    return rstatus;
}



int kx132_enable_synchronous_reading_with_hw_interrupt(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;
    int rstatus = ROUTINE_OK;

// From AN092-Getting-Started.pdf:
//
//   CNTL1  0x00   . . . put sensor into stand-by mode
//   INC1   0x30   . . . reg Interrupt Control 1, enable INT1, set active high, and for latched operation
//   INC4   0x10   . . . reg Interrupt Control 4, set interrupt event to "data ready"
// ( ODCNTL 0x06   . . . reg to control output data rate . . . Optional!  Default rate is 50 Hz )
//   CNTL1  0xE0   . . . put sensor into active readings mode

    reg_val_to_write = 0x00U;
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    reg_val_to_write = 0x30U;
    rstatus |= kx132_write_reg(data->ctx, KX132_INC1, write_buffer, len);

    reg_val_to_write = 0x10U;
    rstatus |= kx132_write_reg(data->ctx, KX132_INC4, write_buffer, len);

// NEED to review setting of ODCNTL register in this routine:   - TMH
    reg_val_to_write = 0x06U;
    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);

    reg_val_to_write = 0xE0U;
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    return rstatus;
}



int kx132_enable_watermark_interrupt(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;
    int rstatus = ROUTINE_OK;

// From AN092-Getting-Started.pdf, section 3.4.2 "Watermark Interrupt (WMI)":
//
//   CNTL1     0x00   . . . put sensor into stand-by mode
// ( ODCNTL    0x06   . . . Optional!  Set Output Data Rate via writes to this register )
//   INC1      0x30   . . . reg Interrupt Control 1, enable INT1, set active high, and for latched operation
//   INC4      0x20   . . . reg Interrupt Control 4, set interrupt event to "watermark / sample threshold reached"
//   BUF_CNTL1 0x2B   . . . buffer control reg 1 to set sample threshold or watermark interrupt "level"
//   BUF_CNTL2 0xE0   . . . buffer control reg 2,
//   CNTL1     0xE0   . . . put sensor into active readings mode


    reg_val_to_write = 0x00U;
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

#ifdef KX132_DRIVER__SET_ODR_OF_50_HZ_IN_ENABLE_WATERMARK_SEQUENCE
// For reg ODCNTL and OSA3..OSA1 bits, see page 26 of 75 of KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf:
    reg_val_to_write = KX132_ODR_50_HZ;
    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);
#endif

    reg_val_to_write = 0x30U;
    rstatus |= kx132_write_reg(data->ctx, KX132_INC1, write_buffer, len);

    reg_val_to_write = 0x20U;
    rstatus |= kx132_write_reg(data->ctx, KX132_INC4, write_buffer, len);

// NEED TO REVIEW whether there is a better way to manage / support watermark
// threshold setting, which following stanzas configure for a sample
// threshold of ten.  That may not be what end application requires . . .
//    reg_val_to_write = 0x0AU;
    reg_val_to_write = data->shadow_reg_buf_cntl1;
    rstatus |= kx132_write_reg(data->ctx, KX132_BUF_CNTL1, write_buffer, len);

    reg_val_to_write = 0xE0U;
    rstatus |= kx132_write_reg(data->ctx, KX132_BUF_CNTL2, write_buffer, len);

    reg_val_to_write = 0xE0U;
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    return rstatus;
}



int kx132_enter_standby_mode(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;

//    uint8_t reg_val_to_write = 0x00U;
//    uint8_t *write_buffer = &reg_val_to_write;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    uint32_t len = 2;
    int rstatus = ROUTINE_OK;

    rstatus = kx132_read_reg(data->ctx, KX132_CNTL1, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_cntl1 = read_buffer[0];
#ifdef DEV_0116
    printk("*\n* DEV 0116 * 'enter standby mode' finds 0x%02x in CNTL1 register.\n", read_buffer[0]);
    printk("* DEV 0116 * 'enter standby mode' about to mask this value with 0x%02x\n", ~(KX132_CNTL1_BIT_FLAG_PC1));
#endif

    data->shadow_reg_cntl1 &= ~(KX132_CNTL1_BIT_FLAG_PC1);
#ifdef DEV_0116
    printk("* DEV 0116 * masked value is 0x%02x\n", data->shadow_reg_cntl1);
#endif

    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, &(data->shadow_reg_cntl1), len);

#ifdef DEV_0116
    rstatus = kx132_read_reg(data->ctx, KX132_CNTL1, read_buffer, SIZE_KX132_REGISTER_VALUE);
    printk("*\n DEV 0116 - 'enter standby mode' wrote 0x%02x to CNTL1 register.\n*\n", read_buffer[0]);
#endif
    return rstatus;
}



int kx132_disable_sample_buffer(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;

//    uint8_t reg_val_to_write = 0x00U;
//    uint8_t *write_buffer = &reg_val_to_write;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    uint32_t len = 2;
    int rstatus = ROUTINE_OK;

    rstatus = kx132_read_reg(data->ctx, KX132_BUF_CNTL2, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_buf_cntl2 = read_buffer[0];
#ifdef DEV_0116
    printk("*\n* DEV 0116 * 'disable sample buffer' finds 0x%02x in BUF_CNTL2 register.\n", read_buffer[0]);
    printk("* DEV 0116 * 'disable sample buffer' about to mask this value with 0x%02x\n", (uint8_t)~(KX132_BUF_CNTL2_BIT_FLAG_BUFE));
#endif

    data->shadow_reg_buf_cntl2 &= (uint8_t)~(KX132_BUF_CNTL2_BIT_FLAG_BUFE);
#ifdef DEV_0116
    printk("* DEV 0116 * masked value is 0x%02x\n", data->shadow_reg_buf_cntl2);
#endif

    rstatus |= kx132_write_reg(data->ctx, KX132_BUF_CNTL2, &(data->shadow_reg_buf_cntl2), len);

#ifdef DEV_0116
    rstatus = kx132_read_reg(data->ctx, KX132_BUF_CNTL2, read_buffer, SIZE_KX132_REGISTER_VALUE);
    printk("- DEV 0116 - 'disable sample buffer' wrote 0x%02x to BUF_CNTL2 register.\n", read_buffer[0]);
#endif
    return rstatus;
}



//----------------------------------------------------------------------
// - SECTION - update register routines
//----------------------------------------------------------------------

/**
 * @brief Routine to update KX132 Output Data Rate control register.
 *
 * @note Kionix KX132 register ODCNTL can only be updated when sensor
 *       is in standby mode, hence the changes to control register
 *       CNTL1 in this routine.
 */

int kx132_update_reg__odcntl__output_data_rate(const struct device *dev,
                                          const enum kx132_1211_output_data_rates_e new_odr)
// NEED to review KX132 datasheet(s) to see whether there are multiple
// output data rates independtly settable by end users, in registers
// beyond KX132_ODCNTL - TMH

// NEED TO LOGICALLY 'OR' THE PASSED OUTPUT DATA RATE (ODR) BITS!!!

// NOTE - NEED to refactor or replace ODR routine near top of kx132-1211.c - TMH
{
    struct kx132_device_data *data = dev->data;

    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    uint32_t len = 2;

    uint8_t as_found_reg_cntl1 = 0;
    uint8_t as_found_reg_odcntl = 0;

    uint32_t rstatus = 0;

#ifdef DEV__ODCNTL_UPDATE_DIAG
    printk("\n- kx132-registers.c - ODCNTL update begin:\n");
    printk("- kx132-registers.c - caller requests ODR setting of %u,\n", new_odr);
#endif
// grab the present CNTL1 value:
    rstatus = kx132_read_reg(data->ctx, KX132_CNTL1, read_buffer, len);
    as_found_reg_cntl1 = read_buffer[0];
#ifdef DEV__ODCNTL_UPDATE_DIAG
    printk("- kx132-registers.c - register CNTL1 as found value is %u,\n", as_found_reg_cntl1);
#endif

// mask out the PC1 (power control?) bit:
    reg_val_to_write = (as_found_reg_cntl1 & ~KX132_CNTL1_BIT_FLAG_PC1);
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

// grab the present ODCNTL value:
    rstatus = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, len);
    as_found_reg_odcntl = read_buffer[0];
#ifdef DEV__ODCNTL_UPDATE_DIAG
    printk("- kx132-registers.c - register ODCNTL as found value is %u,\n", as_found_reg_odcntl);
#endif

// mask, update and write new output data rate to ODCNTL reg:
    reg_val_to_write = (as_found_reg_odcntl & ~KX132_OSA_BITS_MASK);
    reg_val_to_write |= new_odr;
    rstatus |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);
#ifdef DEV__ODCNTL_UPDATE_DIAG
    printk("- kx132-registers.c - updated reg ODCNTL to %u,\n", reg_val_to_write);
#endif

// restore the CNTL1 register value to as found:
    reg_val_to_write = as_found_reg_cntl1;
    rstatus |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);
#ifdef DEV__ODCNTL_UPDATE_DIAG
    printk("- kx132-registers.c - ODCNTL update end.\n\n");
#endif

    if ( rstatus == ROUTINE_OK ) { }
    return rstatus;
}



/**
 * @brief Routine to update KX132 sample threshold value, which sets
 *        count of x,y,z samples needed to trigger a "watermark reached"
 *        interrupt, also called WMI in Kionix technical references.
 */

int kx132_update_reg__sample_threshold_buf_cntl1(const struct device *dev, const uint8_t new_sample_threshold)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = new_sample_threshold;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;

    uint32_t rstatus = kx132_write_reg(data->ctx, KX132_BUF_CNTL1, write_buffer, len);

//    if ( rstatus == ROUTINE_OK ) { }
    return rstatus;
}


/**
 * @brief Routine to write a value to KX132 "clear sample buffer"
 *        register.
 *
 * @note  Kionix technical manual gives no specific value
 *        required to write, only says that any write operation clears
 *        the sample buffer, and also the level based WMI (watermark)
 *        and BFI (buffer full) interrupts.  Those interrupts cannot
 *        be cleared by other means, such as a read of register INT_REL.
 */

int kx132_update_reg__buf_clear(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x01U;  // value of one chosen arbitrarily, tech' manual specs no particular write value necesary.
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;

    uint32_t rstatus = kx132_write_reg(data->ctx, KX132_BUF_CLEAR, write_buffer, len);

    if ( rstatus == ROUTINE_OK ) { }
    return rstatus;
}



/**
 * @brief Routing to allow for app code to set a FIFO sample watermark
 *        or threshold level, which is referenced in the multi-register
 *        config sequence to enable watermark driven interrupt sampling.
 */

int kx132_update_shadow_reg__sample_threshold(const struct device *dev, const uint8_t new_sample_threshold)
{
    struct kx132_device_data *data = dev->data;
    data->shadow_reg_buf_cntl1 = new_sample_threshold;
    return ROUTINE_OK;
}



//----------------------------------------------------------------------
// - SECTION - fetch and get register value routines
//----------------------------------------------------------------------

/**
 * @ note Zephyr RTOS sensor API presents notions of fetching, and
 *   getting sensor readings and register values.  In Zephyr context
 *   to fetch a value means a driver reads a value from a sensor and
 *   stores this value in some variable, such as a "shadow" or copy
 *   register variable, which is itself part of the driver.
 *
 *   In Zephyr context to get a sensor reading or register value means
 *   that the driver returns that value to application code, whether
 *   reading directly from a sensor register or returning the
 *   shadowed copy of the most recent reading.
 *
 *   KX132 driver routines in this section either fetch register
 *   values, or they fetch and return values to application code.
 */

/** 
 * @brief This routine fetches Kionix KX132-1211 Manufacturer ID string.
 */

int kx132_fetch_device_id(const struct device *dev)
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



int kx132_fetch_part_id(const struct device *dev)
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
 


int kx132_get_attr__return_interrupt_statae_2(const struct device *dev, struct sensor_value *val)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};  uint8_t *read_buffer = reg_val_to_read;
    uint32_t rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_INS2, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_ins2 = read_buffer[0];
    val->val1 = data->shadow_reg_ins2;
    val->val2 = 0;

    return rstatus;
}



int kx132_get_attr__output_data_rate(const struct device *dev, struct sensor_value *val)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};  uint8_t *read_buffer = reg_val_to_read;
    uint32_t rstatus = 0;

//    rstatus = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, 2);  // NEED we read two bytes, e.g. SIZE_KX132_REGISTER_VALUE?
    rstatus = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, SIZE_KX132_REGISTER_VALUE);  // NEED we read two bytes, e.g. SIZE_KX132_REGISTER_VALUE?
    data->shadow_reg_odcntl = read_buffer[0];
    val->val1 = (data->shadow_reg_odcntl & KX132_OSA_BITS_MASK);
    val->val2 = 0;

    return rstatus;
}



// IN PROGRESS - routine to return BUF_CNTL1 sample threshold value:

int kx132_get_attr__buf_cntl1__sample_threshold_setting(const struct device *dev, struct sensor_value *val)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};  uint8_t *read_buffer = reg_val_to_read;
    uint32_t rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_BUF_CNTL1, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_buf_cntl1 = read_buffer[0];
    val->val1 = data->shadow_reg_buf_cntl1;
    val->val2 = 0;

    return rstatus;
}



//
// 2023-01-23 - IN PROGRESS routine to return BUF_READ six bytes for
//  latest sample, directly to calling code:
//----------------------------------------------------------------------

/**
 * @brief Routine to read high resolution (16-bit) x,y,z acc sample
 *        triplet and return this sample directly to calling code.
 *
 * @note  This direct value returning is in contrast to Zephyr's sensor
 *        API practice, where Zephyr drivers conventionally "fetch"
 *        a reading from a sensor and store the reading in driver side
 *        memory, and a second routine call to the driver is then
 *        required to "get" the fetched data and return it to
 *        application code.
 */

int kx132_get_attr__buf_read__sample_as_attribute(const struct device *dev, struct sensor_value *val)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_BUF_READ, read_buffer, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);

    if ( rstatus != 0 )
    {
        LOG_WRN("Unable to read acceleration sample buffer BUF_READ.  Error: %i", rstatus);
        return rstatus;
    }

    val->val1 = (
                 ( read_buffer[0] <<  0 ) +  // XOUT_L
                 ( read_buffer[1] <<  8 ) +  // XOUT_H
                 ( read_buffer[2] << 16 ) +  // YOUT_L
                 ( read_buffer[3] << 24 )    // YOUT_H
                );
    val->val2 = (
                 ( read_buffer[4] <<  0 ) +  // ZOUT_L
                 ( read_buffer[5] <<  8 )    // ZOUT_H
                );

    return rstatus;
}



int kx132_fetch_acceleration_x_axis(const struct device *dev)
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

    for ( i = 0; i < KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT; i++ )
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



int kx132_fetch_acceleration_y_axis(const struct device *dev)
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

    for ( i = 0; i < KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT; i++ )
    {
        data->accel_axis_y[i] = read_buffer[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - Y axis acceleration is %d\n",
      ( ( data->accel_axis_y[1] << 8 ) + data->accel_axis_y[0] ) );
#endif

    return rstatus;
}



int kx132_fetch_acceleration_z_axis(const struct device *dev)
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

    for ( i = 0; i < KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT; i++ )
    {
        data->accel_axis_z[i] = read_buffer[i];
    }

#ifdef _DEV_ENABLE_PRINTK
    printk("- DEV - Z axis acceleration is %d\n",
      ( ( data_struc_ptr->accel_axis_z[1] << 8 ) + data_struc_ptr->accel_axis_z[0] ) );
#endif

    return rstatus;
}



int kx132_fetch_acceleration_xyz_axis(const struct device *dev)
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

#if 1
// This block of FOR loops hard to read - TMH
    for (i = 0; i < (KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT + 0); i++)
        { data->accel_axis_x[i - 0] = read_buffer[i]; }
    for (i = 2; i < (KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT + 2); i++)
        { data->accel_axis_y[i - 2] = read_buffer[i]; }
    for (i = 4; i < (KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT + 4); i++)
        { data->accel_axis_z[i - 4] = read_buffer[i]; }
#else
    
#endif

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



int kx132_fetch_interrupt_latch_release(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0};
    uint8_t *read_buffer = reg_val_to_read;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_INT_REL, read_buffer, SIZE_KX132_REGISTER_VALUE);

    if ( rstatus != 0 )
        { LOG_WRN("Unable to read interrupt latch release register.  Err: %i", rstatus); }
    else
        { data->shadow_reg_int_rel = read_buffer[0]; }

    return rstatus;
}



int kx132_fetch_interrupt_source_2(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0};
    uint8_t *read_buffer = reg_val_to_read;
    int rstatus = 0;

    rstatus = kx132_read_reg(data->ctx, KX132_INS2, read_buffer, SIZE_KX132_REGISTER_VALUE);

    if ( rstatus != 0 )
        { LOG_WRN("Unable to read INS2 register.  Err: %i", rstatus); }
    else
        { data->shadow_reg_ins2 = read_buffer[0]; }

    return rstatus;
}




//
// Following routine attempts to read from KX132 readings buffer 'watermark'
// watermark count of bytes, where watermark value is set in
//
// This routine also checks for selected readings resolution, which
// defaults to 16-bit width, and may also be 8-bit width in other
// modes related to low power.
//
//----------------------------------------------------------------------

/**
 * @brief routine to read one x,y,z acceleration reading set from
 *        KX132 sample buffer.
 *
 * @param const struct device *dev A valid Zephyr device pointer to a
 *        Kionix KX132 sensor.
 *
 * @return 0 if successful, negative errno code if failure.
 *
 * @note per Kionix KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf,
 *       page 45 of 75, "To prevent any data loss, data must be read on
 *       a single byte basis or in complete data sets."
 *
 *       High resolution x,y,z data sets reside in six bytes of buffer
 *       space.  Low resolution acceleration data sets take up three
 *       bytes.  This routine reads either six bytes of sample buffer
 *       data or three bytes, based on present readings resolution
 *       setting in the sensor.
 */

int kx132_fetch_readings_from_buf_read(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT];
    uint8_t *read_buffer = reg_val_to_read;
    uint16_t needed_sample_byte_count = 0;
//    enum kx132_readings_resolution_e reading_resolution = KX132_READING_RES_HI_16_BIT;

    int rstatus = 0;


// Clear read buffer, which will be used first to obtain sample resolution setting:

    memset(read_buffer, 0, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);

// Read BUF_CNTL2 to determine sensor readings bit width, "low res" or "high res":

    rstatus = kx132_read_reg(data->ctx, KX132_BUF_CNTL2, read_buffer, SIZE_KX132_REGISTER_VALUE);
#ifdef DEV_0118
// - DIAG BEGIN -
    printk("- KX132 driver - readings resolution bit flag set to %u,\n",
      ( read_buffer[0] & KX132_CNTL2_BIT_FLAG_BRES ? 1 : 0));
// - DIAG END -
#endif

    if ( ( read_buffer[0] & KX132_CNTL2_BIT_FLAG_BRES ) == KX132_CNTL2_BIT_FLAG_BRES )
        { needed_sample_byte_count = 6; }
    else
        { needed_sample_byte_count = 3; }

    memset(reg_val_to_read, 0, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);

    rstatus = kx132_read_reg(data->ctx, KX132_BUF_READ, read_buffer, needed_sample_byte_count);

    memcpy(data->shadow_reg_buf_read, read_buffer, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);

#ifdef DEV_0118
// - DIAG BEGIN -
    printk("- KX132 driver - first six bytes from BUF_READ sample buffer:\n  0x%04X, 0x%04X, 0x%04X,\n\n",
        (read_buffer[0] + (read_buffer[1] << 8 )),
        (read_buffer[2] + (read_buffer[3] << 8 )),
        (read_buffer[4] + (read_buffer[5] << 8 ))
      );
    printk("- KX132 driver - shadowed BUF_READ now holds:\n  0x%04X, 0x%04X, 0x%04X,\n\n",
        (data->shadow_reg_buf_read[0] + (data->shadow_reg_buf_read[1] << 8 )),
        (data->shadow_reg_buf_read[2] + (data->shadow_reg_buf_read[3] << 8 )),
        (data->shadow_reg_buf_read[4] + (data->shadow_reg_buf_read[5] << 8 ))
      );
// - DIAG END -
#endif

    return rstatus;

} // end routine kx132_fetch_readings_from_buf_read()




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
