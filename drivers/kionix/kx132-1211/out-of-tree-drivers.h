#ifndef _OUT_OF_TREE_DRIVERS_H
#define _OUT_OF_TREE_DRIVERS_H

//----------------------------------------------------------------------
// part of project https://github.com/tedhavelka/zephyr-driver-work-v2,
// an out-of-tree Zephyr RTOS driver for Kionix KX132-1211 accelerometer.
//----------------------------------------------------------------------


// NEED to move this enum to a shared include file:
enum kionix_routine_statae {
    ROUTINE_OK,
    ROUTINE_STATUS__SUCCESS,
    ROUTINE_STATUS__ERROR_ENCOUNTERED,
    ROUTINE_STATUS__UNDEFINED_SENSOR_CHANNEL,
    ROUTINE_STATUS__UNDEFINED_SENSOR_ATTRIBUTE,
    ROUTINE_STATUS__UNSUPPORTED_SENSOR_CONFIGURATION,
    ROUTINE_STATUS__COMM_FAILURE_I2C,
    ROUTINE_STATUS__COMM_FAILURE_SPI,
    COUNT_OF_ROUTINE_STATAE
}; 

#endif // _OUT_OF_TREE_DRIVERS_H
