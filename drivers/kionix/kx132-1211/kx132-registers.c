/**
 * @project Kionix Sensor Drivers
 * @file kx132-registers.c
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */



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



