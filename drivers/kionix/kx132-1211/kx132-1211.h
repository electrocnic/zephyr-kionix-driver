#ifndef KX132_1211_H
#define KX132_1211_H

// These includes here, following iis2dh.h example driver header file:
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>

#include "kx132-registers.h"       // to provide register addresses via symbols

#include "kx132-register-interface.h" // to provide kionix_ctx_t sensor context data structure definition




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



/**
 * struct kx132_device_config - Kionix KX132-1211 hardware configuration
 * @spi: SPI bus spec.
 * @i2c: I2C bus spec.
 * @pm: Power mode (to be determined - TMH).
 * @int_gpio: GPIO spec for sensor pin interrupt.
 */

struct kx132_device_config {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#else
#warning  "- WARNING KX132 driver - on SPI bus no device instance found with status 'okay'"
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#else
#warning  "- WARNING KX132 driver - on I2C bus no device instance found with status 'okay'"
#endif

	uint8_t pm;
#ifdef CONFIG_KX132_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_KX132_TRIGGER */
};


/* sensor data */

#if 0 // - following iis2dh driver code commented for reference - TMH

struct iis2dh_data {
	int16_t acc[3];
	uint32_t gain;

        stmdev_ctx_t *ctx;
#ifdef CONFIG_IIS2DH_TRIGGER
        const struct device *dev;
        struct gpio_callback gpio_cb;
        sensor_trigger_handler_t drdy_handler;
#if defined(CONFIG_IIS2DH_TRIGGER_OWN_THREAD)
        K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_IIS2DH_THREAD_STACK_SIZE);
        struct k_thread thread;
        struct k_sem gpio_sem;
#elif defined(CONFIG_IIS2DH_TRIGGER_GLOBAL_THREAD)
        struct k_work work;
#endif /* CONFIG_IIS2DH_TRIGGER_GLOBAL_THREAD */
#endif /* CONFIG_IIS2DH_TRIGGER */
};

#endif // 0


#define BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS (2)
#define BYTE_COUNT_OF_KX132_ACCELERATION_READING_THREE_AXES ((BYTE_COUNT_OF_KX132_ACCELERATION_READING_SINGLE_AXIS) * 3) 

struct kx132_1211_data {
	int16_t acc[3];

// From the original 2021 kionix driver data structure:
// NOTE:  all of these are slated to be removed.  Pointer to
//  I2C controller will be part of kx132_ctx_t data structure,
//  which will support both I2C and SPI buses.  The part ID
//  and accelerometer readings are data which are normally
//  read from the sensor and copied directly to memory to
//  which calling code sends our driver here pointers.  No
//  need to have a copy of those data in this sensor data
//  structure . . .

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


// NOTE:  this "sensor context" data structure holds function pointers to
//  generalized, flexible register_write() and register_read() functions:

        kionix_ctx_t *ctx;

// NOTE:  2022-11-18 following two Kconfig symbols Ted has not yet defined
//  for Kionix driver:

#ifdef CONFIG_KX132_TRIGGER
        const struct device *dev;
        struct gpio_callback gpio_cb;
        sensor_trigger_handler_t drdy_handler;
#if defined(CONFIG_KX132_TRIGGER_OWN_THREAD)
        K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_KX132_THREAD_STACK_SIZE);
        struct k_thread thread;
        struct k_sem gpio_sem;
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
        struct k_work work;
#endif /* CONFIG_KX132_TRIGGER_GLOBAL_THREAD */
#endif /* CONFIG_KX132_TRIGGER */
};


int kx132_i2c_init(const struct device *dev);
int kx132_spi_init(const struct device *dev);

#ifdef CONFIG_KX132_TRIGGER
int kx132_init_interrupt(const struct device *dev);
int kx132_trigger_set(const struct device *dev,
                      const struct sensor_trigger *trig,
                      sensor_trigger_handler_t handler);
#endif /* CONFIG_KX132_TRIGGER */



// KX132-1211 sensor possible I2C addresses:
// ( See KX132-1211-Specifications-Rev-1.0.pdf for details )
#define KX132_I2C_ADDRESS__ADDR_PIN_AT_GND    (0x1E)
#define KX132_I2C_ADDRESS__ADDR_PIN_AT_IO_VDD (0x1F)
#define KX132_I2C_ADDRESS_FLIPPED__ADDR_PIN_AT_GND    (0x1C)
#define KX132_I2C_ADDRESS_FLIPPED__ADDR_PIN_AT_IO_VDD (0x1D)


// Note:  Zephyr Project 2.6.0 provides sensor.h header file in
//  ncs/zephyr/include/drivers/sensor.h.  An important enum given here
//  is named sensor_channel.  Near it's end last two elements are:
//
//    189         SENSOR_CHAN_PRIV_START = SENSOR_CHAN_COMMON_COUNT,
//
//    194         SENSOR_CHAN_MAX = INT16_MAX,
//
//  Until we find better we're going to use "sensor channel private start"
//  enum element to provide some custom channels to Kionix KX132-1211.
//----------------------------------------------------------------------

enum sensor_channels_kionix_specific {
    SENSOR_CHAN_KIONIX_START = (SENSOR_CHAN_PRIV_START + 1),
    SENSOR_CHAN_KIONIX_MANUFACTURER_ID,
    SENSOR_CHAN_KIONIX_PART_ID,
    SENSOR_CHAN_KIONIX_END
};


// REF https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html#c.sensor_attribute
// REF from Kionix AN092-Getting-Stated.pdf

// QUESTION:  how do we keep our and any custom enumerated sensor
//  attributes from colliding with other third party, out-of-tree
//  driver enumerations?
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum kx132_1211_config_setting_e
{
    KX132_CONFIGURATION_SETTING_FIRST,

// From AN092-Getting-Stated.pdf:
    KX132_ENABLE_ASYNC_READINGS,
    KX132_ENABLE_SYNC_READINGS_WITH_HW_INTERRUPT,
    KX132_ENABLE_SYNC_READINGS_WITHOUT_HW_INTERRUPT,
    KX132_ENABLE_ACCELEROMETER_READINGS_BUFFER,
    KX132_ENABLE_WATERMARK_INTERRUPT,
    KX132_SET_TRIGGER_MODE,
    KX132_ENABLE_WAKE_UP,
    KX132_ENABLE_WAKE_UP_AND_BACK_TO_SLEEP,
    KX132_ENABLE_TILT_POSITION_WITH_FACE_DETECT,
    KX132_ENABLE_TAP_DOUBLE_TAP,
    KX132_ENABLE_FREE_FALL_ENGINE,

// From AN109-...-3p0.pdf
    KX132_SET_OUTPUT_DATA_RATE,

    KX132_CONFIGURATION_SETTING_LAST
};


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



#endif // KX132_1211_H
