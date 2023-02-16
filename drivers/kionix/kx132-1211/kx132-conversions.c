

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
    ACCELERATION_IN_G,
    ACCELERATION_IN_M_PER_S_SQUARED
};

// https://en.wikipedia.org/wiki/Gravitational_acceleration
// https://en.wikipedia.org/wiki/Standard_gravity
#define ACCELERATION_OF_GRAVITY_AT_EARTH_MEAN_SURFACE 9.8067


float reading_in_g(const uint32_t reading_in_dec_counts,
                   const enum kx132_acceleration_resolutions resolution,
                   const enum kx132_acceleration_ranges range,
                   const enum acceleration_units_of_measure desired_units)
{
    int32_t decimal_count_max = 0;
    int32_t decimal_count_min = 0;
    float units_of_g_range_max = 0.0;
    float units_of_g_range_min = 0.0;
    float reading = 0.0;

// For 16-bit, high resolution readings:
    {
// NEED to consider bounds check on reading to keep within unsigned 0x0000 to 0xFFFF, 16-bit range of values.

        decimal_count_max = KX132_RANGE_RES_HIGH_DECIMAL_COUNTS_MAX;
        decimal_count_min = KX132_RANGE_RES_HIGH_DECIMAL_COUNTS_MIN;

        switch (range)
        {
            case KX132_RANGE_PLUS_MINUS_2G:
                units_of_g_range_max = KX132_RANGE_RES_HIGH_2G_MAX;
                units_of_g_range_min = KX132_RANGE_RES_HIGH_2G_MIN;
                break;

            case KX132_RANGE_PLUS_MINUS_4G:
                units_of_g_range_max = KX132_RANGE_RES_HIGH_4G_MAX;
                units_of_g_range_min = KX132_RANGE_RES_HIGH_4G_MIN;
                break;

            case KX132_RANGE_PLUS_MINUS_8G:
                units_of_g_range_max = KX132_RANGE_RES_HIGH_8G_MAX;
                units_of_g_range_min = KX132_RANGE_RES_HIGH_8G_MIN;
                break;

            case KX132_RANGE_PLUS_MINUS_16G:
                units_of_g_range_max = KX132_RANGE_RES_HIGH_16G_MAX;
                units_of_g_range_min = KX132_RANGE_RES_HIGH_16G_MIN;
                break;

            default:
                printk("- WARNING - unsupported KX132 ");
                return reading;
        }
    }

// NEED to add low-res conversion code - TMH

// For 8-bit, high resolution readings:
    {
// NEED to consider bounds check on reading to keep within unsigned 0x00 to 0xFF, 8-bit range of values.
    }


// Incoming reading in two's compliment needs to be scaled by a factor of
//
//   units of G range
// --------------------
// decimal counts range

    reading = ( (int16_t)reading_in_dec_counts * ( (units_of_g_range_max - units_of_g_range_min) /
                ((float)decimal_count_max - (float)decimal_count_min)
              ) );

    if ( desired_units == ACCELERATION_IN_M_PER_S_SQUARED )
    {
        reading *= ACCELERATION_OF_GRAVITY_AT_EARTH_MEAN_SURFACE;
    }

    return reading;
}
