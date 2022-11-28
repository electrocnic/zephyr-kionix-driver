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
    ROUTINE_STATUS__COMM_FAILURE_ON_BUS,
    ROUTINE_STATUS__GPIO_DRDY_INTERRUPT_LOOKS_MAL_ASSIGNED,
    ROUTINE_STATUS__GPIO_DRDY_INTERRUPT_REINIT_FAIL,
    COUNT_OF_ROUTINE_STATAE
}; 


// Copied from [Zephyr 3.2.0 workspace]/modules/hal/st/sensor/stmemsc/iis2dh_STdC/driver/iis2dh_reg.h:
#define PROPERTY_DISABLE  (0U)
#define PROPERTY_ENABLE   (1U)


#define DEFAULT_INTERAL_OPTION_OF_ZERO (0U)


#endif // _OUT_OF_TREE_DRIVERS_H
