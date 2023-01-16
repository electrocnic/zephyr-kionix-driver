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

// This looks like an indirect Zephyr RTOS dependency . . . TMH:
#include "kx132-register-interface.h" // to provide kionix_ctx_t to function prototypes



#define KX132_WHO_AM_I_EXPECTED_VALUE (0x3D)
#define KX134_WHO_AM_I_EXPECTED_VALUE (0x46)
#define KX132_COTR_EXPECTED_VALUE (0x55)

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

enum kx132_readings_resolution_e
{
    KX132_READING_RES_HI_16_BIT,
    KX132_READING_RES_LO_8_BIT,
};



//----------------------------------------------------------------------
// - SECTION - defined register addresses
//----------------------------------------------------------------------

// For following defines see Kionix document
// KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf

#define KX132_MAN_ID    (0x00)
#define KX132_MAN_ID_SIZE  (4)
#define KX132_PART_ID   (0x01)
#define KX132_PART_ID_SIZE (2)

#define KX132_XOUT_L    (0x08)
#define KX132_XOUT_H    (0x09)
#define KX132_YOUT_L    (0x0A)
#define KX132_YOUT_H    (0x0B)
#define KX132_ZOUT_L    (0x0C)
#define KX132_ZOUT_H    (0x0D)

//----------------------------------------------------------------------
// 
//----------------------------------------------------------------------
#define SIZE_KX132_REGISTER_VALUE (1)


#define KX132_COTR      (0x12)
#define KX132_WHO_AM_I  (0x13)


#define KX132_INS1      (0x16)

// Status register to indicate source of or event triggering interrupt:
#define KX132_INS2      (0x17)

#define KX132_INS2_BIT_FLAG_FREE_FALL_STATUS        (1 << 7)
#define KX132_INS2_BIT_FLAG_BUFFER_FULL             (1 << 6)
#define KX132_INS2_BIT_FLAG_WATERMARK_EXCEEDED      (1 << 5)
#define KX132_INS2_BIT_FLAG_DATA_READY              (1 << 4)
#define KX132_INS2_BIT_FLAG_TAP_DOUBLE_TAP_STATUS_1 (1 << 3)
#define KX132_INS2_BIT_FLAG_TAP_DOUBLE_TAP_STATUS_0 (1 << 2)
#define KX132_INS2_BIT_FLAG_TILT_POSITION_STATUS    (1 << 0)

#define KX132_INS3      (0x18)

// KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf p 14 of 75:
// interrtup latch release register INT_REL
#define KX132_INT_REL   (0x1A)


#define KX132_CNTL1     (0x1B)
#define KX132_CNTL1_BIT_FLAG_PC1                    (1 << 7)  // "Controls the operating mode" per tech' reference manual
#define KX132_CNTL1_BIT_FLAG_RES                    (1 << 6)  // acc reading resolution, 8-bit versus 16-bit
#define KX132_CNTL1_BIT_FLAG_DRDYE                  (1 << 5)  // Data Ready Engine enable
#define KX132_CNTL1_BIT_FLAG_GSEL1                  (1 << 4)  // G range selection bit most significant
#define KX132_CNTL1_BIT_FLAG_GSEL0                  (1 << 3)  // G range selection bit least significant
#define KX132_CNTL1_BIT_FLAG_TDTE                   (1 << 2)  // Tap, Double-Tap enable
#define KX132_CNTL1_BIT_FLAG_BIT1_RESERVED          (1 << 1)
#define KX132_CNTL1_BIT_FLAG_TPE                    (1 << 0)  // Tile Position Engine enable

#define KX132_CNTL2     (0x1C)
#define KX132_CNTL2_BIT_FLAG_BRES                   (1 << 6)

#define KX132_ODCNTL    (0x21)
#define KX132_INC1      (0x22)
#define KX132_INC2      (0x23)
#define KX132_INC3      (0x24)
#define KX132_INC4      (0x25)


// Register whose bits express sample count threshold, a.k.a. watermark
// at which to raise interrupt for application to read samples:
#define KX132_BUF_CNTL1 (0x5E)
// bit-wise flags of this register in technical manual are named SMP_TH7..SMP_TH0

#define KX132_BUF_CNTL2 (0x5F)
#define KX132_BUF_CNTL2_BIT_FLAG_BUFE               (1 << 7)  // controls activation of sample buffer at BUF_READ
#define KX132_BUF_CNTL2_BIT_FLAG_BRES               (1 << 6)  // determines resolution of samples collected by buffer
#define KX132_BUF_CNTL2_BIT_FLAG_BFIE               (1 << 5)  // buffer full interrupt enable
#define KX132_BUF_CNTL2_BIT_FLAG_BIT4_RESERVED      (1 << 4)
#define KX132_BUF_CNTL2_BIT_FLAG_BIT3_RESERVED      (1 << 3)
#define KX132_BUF_CNTL2_BIT_FLAG_BIT2_RESERVED      (1 << 2)
#define KX132_BUF_CNTL2_BIT_FLAG_BM1                (1 << 1)  // buffer operating mode bit
#define KX132_BUF_CNTL2_BIT_FLAG_BM0                (1 << 0)  // buffer operating mode bit
// buffer operating modes, KX132 technical reference manual page 43 of 75:
//         BM1  BM0
// FIFO     0    0
// STREAM   0    1
// TRIGGER  1    0

// Per manual latched buffer status info and entire buffer data are
// cleared whenever are data written to this register:
#define KX132_BUF_CLEAR (0x62)

#define KX132_BUF_READ  (0x63)


// See TN027-Power-On-Procedure.pdf for following register use:
#define KX132_UNNAMED_SW_RESET_REG_0x7F (0x7F)



//----------------------------------------------------------------------
// - SECTION - prototypes
//----------------------------------------------------------------------

// Important wrapper functions to read registers, write registers:

int32_t kx132_read_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
int32_t kx132_write_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);


// As of 2023 Q1 software reset routine treated as a fetching routine, as it reads two
// registers as part of final status check:
int kx132_software_reset(const struct device *dev);

int kx132_enable_watermark_interrupt(const struct device *dev);

int kx132_enable_asynchronous_readings(const struct device *dev);

int kx132_enable_synchronous_reading_with_hw_interrupt(const struct device *dev);



int kx132_fetch_device_id(const struct device *dev);

int kx132_fetch_part_id(const struct device *dev);

int kx132_fetch_acceleration_x_axis(const struct device *dev);

int kx132_fetch_acceleration_y_axis(const struct device *dev);

int kx132_fetch_acceleration_z_axis(const struct device *dev);

int kx132_fetch_acceleration_xyz_axis(const struct device *dev);

int kx132_fetch_interrupt_latch_release(const struct device *dev);

int kx132_fetch_readings_from_buf_read(const struct device *dev);

int kx132_fetch_interrupt_source_2(const struct device *dev);



#endif // KX132_REGISTERS_H
