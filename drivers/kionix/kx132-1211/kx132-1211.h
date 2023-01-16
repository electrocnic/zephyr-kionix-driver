#ifndef KX132_1211_H
#define KX132_1211_H

//----------------------------------------------------------------------
// - SECTION - includes
//----------------------------------------------------------------------

// These includes here, following iis2dh.h example driver header file:
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>

#include "kx132-registers.h"       // to provide register addresses via symbols

// Following include here to provide kionix_ctx_t sensor context data
// structure definition.  This structure entails function pointers
// to act as assignable wrappers to register read, and register write
// functions.  This strategy found in STMicro IIS2DH Zephyr driver code
// permits automatic compile-time selection of bus API calls -- I2C
// versus SPI -- as expressed in device tree source files such as .dts
// .overlay files.  In other words, straight-forward changes to a
// firmware project device tree source result in correct, needed driver
// compilation of the chosen bus on which KX132 sensor resides.

#include "kx132-register-interface.h" 



//----------------------------------------------------------------------
// Note:  following DT_DRV_COMPAT macro needed in order to use important
//   subset of Zephyr Device Tree Macro API:
//----------------------------------------------------------------------

#define DT_DRV_COMPAT kionix_kx132_1211



//----------------------------------------------------------------------
// - SECTION - Kionix KX132 specific
//----------------------------------------------------------------------

#define SIZE_MANUFACT_ID_STRING (4)
#define SIZE_PART_ID_STRING (2)
#define PERIOD_TO_POWER_UP_IN_MS         (50)
#define PERIOD_TO_PERFORM_SW_RESET_IN_MS (20)

// NEED 2023-01-12 to review how byte count here works for higher
//  resolution KX132 readings, 16-bit wide readings, but is
//  incorrect for low resolution readings which are 8 bits wide:

#define KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT (2)

#define BYTE_COUNT_OF_KX132_ACCELERATION_READING_THREE_AXES ((KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT) * 3) 


//
//**********************************************************************
// Following enum likely to remain part of first production release
// of this driver, but as of 2022 Q4 - 2023 Q1 there is debugging work
// underway to find out and correct "data ready" interrupt support in
// this Zephyr driver.  Same `drdy-gpios` based interrupt provisioning
// in IIS2DH Zephyr driver has so far failed to function as expected
// too . . .  TMH

// "Data ready" interrupt port status possibilites:

enum kx132_1211_drdy_port_status_e
{
    DRDY_PORT_NOT_INITIALIZED,
    DRDY_CFG_INT_GPIO_FOUND_NULL,
    DRDY_PORT_FOUND_NULL,
    DRDY_PORT_MAL_INITIALIZED,
    DRDY_PORT_INITIALIZED
};

//**********************************************************************
//




//----------------------------------------------------------------------
// - SECTION - development
//----------------------------------------------------------------------

// Following define is very handy to confirm multi-register config sequences:
#define DEV__KX_DRIVER_DEV_1202__LOW_LEVEL_SPI_WRITE

//#define DEV__KX_DRIVER_DEV_1120__LOW_LEVEL_SPI_READ

//#define DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION



//----------------------------------------------------------------------
// - SECTION - utilities
//----------------------------------------------------------------------

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

union kx132_acc_reading
{
    uint16_t in_hi_res_16_bit_mode;
    uint8_t in_lo_res_8_bit_mode[2];
};



// # https://gcc.gnu.org/onlinedocs/gcc-3.4.6/cpp/Stringification.html . . . stringification via C macros
#define xstr(s) str(s)
#define str(s) #s

// Ejemplo:
// #define ST_IIS2DH DT_INST(0, st_iis2dh)
// printk("- DEV 1028 - symbol ST_IIS2DH got assigned '%s'\n", xstr(ST_IIS2DH));



//----------------------------------------------------------------------
// - SECTION - Zephyr sensor API constructs
//----------------------------------------------------------------------

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
#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "KX132 1211 driver - compiling gpio_dt_spec instance in struct 'kx132_device_config'"
#endif
// # REF https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/gpio.h#L271
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_KX132_TRIGGER */
};


// - DEV 1130 -
// Effort to identify `drdy-gpios` compatible node with okay status,
// though all we've seen and written in our device tree overly is
// a sensor node attached to a bus, and this sensor node contains
// a `drdy-gpios` property to express the sensor's hardware interrupt
// line coming to the MCU.  And our driver based on STMicro IIS2DH
// driver is not picking up a value gpio_dt_spec type port device
// handle, so we're thus far not enabling an interrupt for this
// sensor . . .

#define DRDY_GPIOS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(drdy_gpios)
#define DRDY_GPIO_DEVICETREE_PATH DT_NODE_PATH(DRDY_GPIOS_NODE)



struct kx132_device_data {

// NEED to review whether this array needed, it is at present
// and at first KX132 driver draft copied over from IIS2DH
// driver, as found in Zephyr 3.2.0.  Looks like it might be
// used as part of a dummy read, to clear an interrupt bit in 
// the IIS2DH:
    int16_t acc[3];

// NEED to review this data member, which seems now moot with
// the adding of the sensor context type 'kionix_ctx_t':
//    const struct device *i2c_dev;  . . . 2023-01-13 Ted commenting out


// NOTE:  this "sensor context" data structure holds function pointers to
//  generalized, flexible register_write() and register_read() functions:

    kionix_ctx_t *ctx;

#ifdef CONFIG_KX132_TRIGGER
#ifdef DEV_ANNOUNCE_TRIGGER_CODE_COMPILATION
#warning "compiling KX132 trigger data members into kx132_device_data structure . . ."
#endif
    const struct device *dev;
    struct gpio_callback gpio_cb;
    sensor_trigger_handler_t drdy_handler;

    struct gpio_dt_spec int_gpio;

#if defined(CONFIG_KX132_TRIGGER_OWN_THREAD)
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_KX132_THREAD_STACK_SIZE);
    struct k_thread thread;
    struct k_sem gpio_sem;
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif /* CONFIG_KX132_TRIGGER_GLOBAL_THREAD */
#endif /* CONFIG_KX132_TRIGGER */


// - DEV 1128 - driver introspection effort to address early,
//  seeming mal-assigned drdy "data ready" GPIO port device pointer:
// NEED 2023-01-13 to determine whether good practice to conditionally compile following when CONFIG_KX132_TRIGGER=y:

    uint32_t drdy_port_status;


//----------------------------------------------------------------------
// "shadow" instances of sensor registers:
//----------------------------------------------------------------------

// As of 2013-01-23 a four byte data type:
    union string_union_type__manufacturer_id manufacturer_id;

// As of 2013-01-23 a two byte data type:
    union string_union_type__part_id part_id;


    uint8_t shadow_reg_who_am_i;

    uint8_t shadow_reg_cotr; // KX132 Command Test control Register

// Interrupt latch release register, interrupts cleared when code reads this register:
    uint8_t shadow_reg_int_rel;

    uint8_t shadow_reg_ins2;

    uint8_t shadow_reg_buf_cntl2;


// Following three data members are written with LSB, MSB of respective accelerometer readings:
    uint8_t accel_axis_x[KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT];
    uint8_t accel_axis_y[KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT];
    uint8_t accel_axis_z[KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT];

// Following data member written with LSB, MSB of allaccelerometer readings X, Y and Z axes:
    uint8_t accel_axis_xyz[BYTE_COUNT_OF_KX132_ACCELERATION_READING_THREE_AXES];

// QUESTION:  any reason we need to align data on four byte boundary? - TMH
//    uint8_t padding[KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT];


// From KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf page 43 of 75:
// In 16-bit readings resolution mode readings buffer holds 86 triplets of x,y,z readings,
// In 8-bit readings resolution mode readings buffer holds 171 triplets of x,y,z readings.

// From AN092-Getting-Started.pdf page 7 of 27, Kionix says KX132 read buffer has 258 bytes:
#define KX132_FIFO_CAPACITY_FOR_HI_RES_XYZ_READING_TRIPLETS 43
#define KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT 6
#define KX132_READINGS_TRIPLET_LO_RES_BYTE_COUNT 3
#define KX132_BUF_READ_SIZE (KX132_FIFO_CAPACITY_FOR_HI_RES_XYZ_READING_TRIPLETS * KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT)

// For 16-bit and 8-bit resolution readings buffer, (16 * 129) is same as (8 * 258) byte buffer size:
    union kx132_acc_reading buf_read[KX132_BUF_READ_SIZE / 2];

// A driver side array index to driver's array holding buf_read FIFO readings:
    uint16_t buf_read_index;

// A driver side "register", not in sensor, to support flexible FIFO reading
// during stream, watermark based and related FIFO readings modes:
    uint8_t driver_reg_sample_count_to_read;
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



//
//----------------------------------------------------------------------
//
// Note:  Zephyr Project 2.6.0, and Zephyr 3.2.0 provide sensor.h header
//   file.  An important enum given here is named sensor_channel.  Near
//   it's end last two elements are:
//
//     189         SENSOR_CHAN_PRIV_START = SENSOR_CHAN_COMMON_COUNT,
//
//     194         SENSOR_CHAN_MAX = INT16_MAX,
//
//   Until we find better we're going to use "sensor channel private start"
//   enum element to provide some custom channels to Kionix KX132-1211.
//
//
// 2023-01-12 addendum:
//
//   In more general sense Zephyr's sensor API establishes a notion
//   of two interaction types between driver code and app code.  These
//   interactions are in part referenced by the phrases "sensor channel
//   fetch" and "sensor channel get".
//
//   To "fetch sensor data" causes given driver to communicate with a
//   sensor and read back data.  The driver then stores latest readings
//   or other status, config, or identifying data in memory allocated
//   to the driver.
//
//   To "get sensor data" causes given driver to return the most
//   recently fetched values to application code.
//
//   Zephyr also establishes a set of sensor "channels", in an attempt
//   to make reading units and types uniformly known.  Sensor channels
//   cover many commonly measured parameters from the outside world,
//   parameters such as temperature, pressure, voltage, electrical
//   current, acceleration, and similar.  This KX132 driver uses these
//   channels where they apply.  There are also some configuration
//   and identifying values application code may want to read from a
//   KX132 sensor which do not appear in Zephyr's sensor API channels
//   enumeration.  For this reason this driver implements a few
//   additinal sensor channels in following enumeration, to support
//   returning these data to app code from a KX132 accelerometer:
//----------------------------------------------------------------------
//

enum sensor_channels_kionix_specific {
    SENSOR_CHAN_KIONIX_START = (SENSOR_CHAN_PRIV_START + 1),

    SENSOR_CHAN_KIONIX_MANUFACTURER_ID,
    SENSOR_CHAN_KIONIX_PART_ID,
// KX132_INT_REL, something of a config and a status register:
    SENSOR_CHAN_KIONIX_INTERRUPT_LATCH_RELEASE,
// sensor status register:
    SENSOR_CHAN_KIONIX_INS2,
// programmatic:
    SENSOR_CHAN_KIONIX_SOFTWARE_RESET_STATUS_VALUE,
// sensor readings FIFO:
    SENSOR_CHAN_KIONIX_BUF_READ,

// a general "read configuration value" custom sensor channel:
    SENSOR_CHAN_KIONIX__KX132_CONFIG_REGISTER,

//**********************************************************************
// NOTE:  for safety there is no general "write register"
// Zephyr sensor channel defined in this driver.  Certain register
// bits of KX132 are reserved and technical manual instructs
// sensor users to leave these bits unwritten and not changed.
//**********************************************************************

// a general "read status value" custom sensor channel:
    SENSOR_CHAN_KIONIX__KX132_STATUS_REGISTER,

    SENSOR_CHAN_KIONIX_END
};



// REF https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html#c.sensor_attribute
// REF from Kionix AN092-Getting-Stated.pdf

enum kx132_1211_config_setting_e
{
    KX132_CONFIGURATION_SETTING_FIRST,

// From Kionix document TN027-Power-On-Procedure.pdf:
    KX132_PERMFORM_SOFTWARE_RESET,

// From AN109-...-3p0.pdf
    KX132_SET_OUTPUT_DATA_RATE,

// From Kionix document AN092-Getting-Stated.pdf:
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

// - single-register configurations:
    KX132_ENTER_STANDBY_MODE,
    KX132_DISABLE_SAMPLE_BUFFER,

// - DEV 1128 -
    KX132_REINITIALIZE_DRDY_GPIO_PORT,

// - DEV 0115 -
    KX132_SET_BUF_READ_FETCH_SAMPLES_COUNT,

    KX132_CONFIGURATION_SETTING_LAST
};



#endif // KX132_1211_H
