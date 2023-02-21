/**
  * @project Kionix Sensor Drivers
  * @file kx132-conversions.c
  * @author Ted Havelka
  * @license Apache 2.0 licensed.
  */

#include <stdio.h>                 // to provide printk()

#include "kx132-conversions.h"

float reading_in_g(const unsigned int reading_in_dec_counts,
                   const enum kx132_acceleration_resolutions resolution,
                   const enum kx132_acceleration_ranges range,
                   const enum acceleration_units_of_measure desired_units)
{
    int decimal_count_max = 0;
    int decimal_count_min = 0;
    float units_of_g_range_max = 0.0;
    float units_of_g_range_min = 0.0;
    float reading = 0.0;

printk("---\n--- KX132 driver:  got raw acceleration reading of %u ---\n---\n");

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
//                printk("- WARNING - unsupported KX132 ");
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

    reading = ( (short int)reading_in_dec_counts * ( (units_of_g_range_max - units_of_g_range_min) /
                ((float)decimal_count_max - (float)decimal_count_min)
              ) );

    if ( desired_units == ACCELERATION_IN_M_PER_S_SQUARED )
    {
        reading *= ACCELERATION_OF_GRAVITY_AT_EARTH_MEAN_SURFACE;
    }

    return reading;
}
