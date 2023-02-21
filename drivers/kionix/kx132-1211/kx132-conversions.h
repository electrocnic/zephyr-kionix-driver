#ifndef KX132_CONVERSIONS_H
#define KX132_CONVERSIONS_H

 /**
  * @project Kionix Sensor Drivers
  * @file kx132-conversions.h
  * @author Ted Havelka
  * @license Apache 2.0 licensed.
  */



//----------------------------------------------------------------------
// - SECTION - conversion
//----------------------------------------------------------------------

// From KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf page 10:

#ifndef KX132_ACCELEROMETER_RANGE_DEFINES
#define KX132_ACCELEROMETER_RANGE_DEFINES

#define KX132_RANGE_RES_HIGH_DECIMAL_COUNTS_MIN -32768
#define KX132_RANGE_RES_HIGH_DECIMAL_COUNTS_MAX  32767

#define KX132_RANGE_RES_HIGH_2G_MIN            -2.00000
#define KX132_RANGE_RES_HIGH_2G_MAX             1.99994

#define KX132_RANGE_RES_HIGH_4G_MIN            -4.00000
#define KX132_RANGE_RES_HIGH_4G_MAX             3.99988

#define KX132_RANGE_RES_HIGH_8G_MIN            -8.00000
#define KX132_RANGE_RES_HIGH_8G_MAX             7.99976

#define KX132_RANGE_RES_HIGH_16G_MIN          -16.00000
#define KX132_RANGE_RES_HIGH_16G_MAX           15.99951

// 12-7 NEED to add low resolution range values (these for 8-bit readings).


enum kx132_acceleration_ranges
{
    KX132_ACCEL_RANGES_BEGIN,
    KX132_RANGE_PLUS_MINUS_2G,
    KX132_RANGE_PLUS_MINUS_4G,
    KX132_RANGE_PLUS_MINUS_8G,
    KX132_RANGE_PLUS_MINUS_16G,
    KX132_ACCEL_RANGES_END
};

enum kx132_acceleration_resolutions
{
    KX132_ACCEL_RESOLUTION_HIGH = 1,
    KX132_ACCEL_RESOLUTION_LOW
};

#endif // KX132_ACCELEROMETER_RANGE_DEFINES


enum acceleration_units_of_measure
{
    ACCELERATION_IN_M_PER_S_SQUARED,
    ACCELERATION_IN_INCHES_PER_S_SQUARED
};

// https://en.wikipedia.org/wiki/Gravitational_acceleration
// https://en.wikipedia.org/wiki/Standard_gravity
#define ACCELERATION_OF_GRAVITY_AT_EARTH_MEAN_SURFACE 9.8067



float reading_in_g(const unsigned int reading_in_dec_counts,
                   const enum kx132_acceleration_resolutions resolution,
                   const enum kx132_acceleration_ranges range,
                   const enum acceleration_units_of_measure desired_units);


#endif // KX132_CONVERSIONS_H
