#ifndef KX132_1211_H
#define KX132_1211_H


// For following defines see Kionix document
// KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf

#define MAN_ID (0x00)
#define PART_ID (0x01)

// KX132-1211 I2C sensor address with ADDR pin connected to IO_VDD:
// 2021-09-01 Updating per KX132-1211-Specifications-Rev-1.0.pdf:
#define KX132_I2C_ADDRESS__ADDR_PIN_AT_GND    (0x1E)
#define KX132_I2C_ADDRESS__ADDR_PIN_AT_IO_VDD (0x1F)
#define KX132_I2C_ADDRESS_FLIPPED__ADDR_PIN_AT_GND    (0x1C)
#define KX132_I2C_ADDRESS_FLIPPED__ADDR_PIN_AT_IO_VDD (0x1D)


#define CMD_KX132_REQUEST_MANUFACTURER_ID { MAN_ID }

#define CMD_KX132_REQUEST_PART_ID { PART_ID }



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
    SENSOR_CHAN_KIONIX_END,
};


#if 0
// NEED to move this enum to a shared include file:
enum kionix_routine_statae {
    ROUTINE_OK,
    ROUTINE_SUCCESS,
    ROUTINE_ERROR_ENCOUNTERED,
    KX132_ROUTINE__UNDEFINED_SENSOR_CHANNEL,
    KX132_ROUTINE__UNDEFINED_SENSOR_ATTRIBUTE,
    KX132_ROUTINE__UNSUPPORTED_SENSOR_CONFIGURATION
}; 
#endif


//----------------------------------------------------------------------
// - SECTION - function prototypes
//----------------------------------------------------------------------


#endif // KX132_1211_H
