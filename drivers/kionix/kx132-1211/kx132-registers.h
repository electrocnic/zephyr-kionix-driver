/**
 * @project Kionix Sensor Drivers
 * @file kx132-registers.h
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */

#ifndef KX132_REGISTERS_H
#define KX132_REGISTERS_H



//----------------------------------------------------------------------
// - SECTION -
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



//----------------------------------------------------------------------
// - SECTION -
//----------------------------------------------------------------------

int kx132_enable_asynchronous_readings(const struct device *dev);



#endif // KX132_REGISTERS_H
