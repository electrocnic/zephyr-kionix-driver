/**
 * @project Kionix Sensor Drivers
 * @file kx132-registers.h
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */

#ifndef KX132_REGISTERS_H
#define KX132_REGISTERS_H


//----------------------------------------------------------------------
// - SECTION - includes
//----------------------------------------------------------------------

#include "kx132-register-interface.h" // to provide kionix_ctx_t to function prototypes




//----------------------------------------------------------------------
// - SECTION - enums and non-register-address defines
//----------------------------------------------------------------------

// REF Kionix document KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf
// ODCNTL (0x21) control register, pages 25..26 of 75. 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum kx132_1211_output_data_rates_e
{
    KX132_ODR__0P781_HZ,
    KX132_ODR__1P563_HZ,
    KX132_ODR__3P125_HZ,
    KX132_ODR__6P25_HZ,

    KX132_ODR_12P5_HZ,
    KX132_ODR_25_HZ,
    KX132_ODR_50_HZ,
    KX132_ODR_100_HZ,

    KX132_ODR_200_HZ,
    KX132_ODR_400_HZ,
    KX132_ODR_800_HZ,
    KX132_ODR_1600_HZ,

    KX132_ODR_3200_HZ,
    KX132_ODR_6400_HZ,
    KX132_ODR_12800_HZ,
    KX132_ODR_25600_HZ,
};



//----------------------------------------------------------------------
// - SECTION - defines register addresses
//----------------------------------------------------------------------

// For following defines see Kionix document
// KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf

#define KX132_MAN_ID  (0x00)
#define KX132_MAN_ID_SIZE  (4)
#define KX132_PART_ID (0x01)
#define KX132_PART_ID_SIZE (2)

#define KX132_XOUT_L  (0x08)
#define KX132_XOUT_H  (0x09)
#define KX132_YOUT_L  (0x0A)
#define KX132_YOUT_H  (0x0B)
#define KX132_ZOUT_L  (0x0C)
#define KX132_ZOUT_H  (0x0D)

#define KX132_CNTL1  (0x1B)
#define KX132_ODCNTL (0x21)
#define KX132_INC1   (0x22)
#define KX132_INC2   (0x23)
#define KX132_INC3   (0x24)
#define KX132_INC4   (0x25)



//----------------------------------------------------------------------
// - SECTION - prototypes
//----------------------------------------------------------------------

// Important wrapper functions to read registers, write registers:

int32_t kx132_read_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
int32_t kx132_write_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);


int kx132_enable_asynchronous_readings(const struct device *dev);

int kx132_enable_synchronous_reading_with_hw_interrupt(const struct device *dev);


int kx132_fetch_device_id(const struct device *dev);

int kx132_fetch_part_id(const struct device *dev);

int kx132_fetch_acceleration_x_axis(const struct device *dev);

int kx132_fetch_acceleration_y_axis(const struct device *dev);

int kx132_fetch_acceleration_z_axis(const struct device *dev);

int kx132_fetch_acceleration_xyz_axis(const struct device *dev);




#endif // KX132_REGISTERS_H
