#ifndef KX132_REGISTERS_H
#define KX132_REGISTERS_H



// For following defines see Kionix document
// KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf

// Note following are KX132-1211 config' register addresses:
#define MAN_ID (0x00)
#define PART_ID (0x01)

#define KX132_1211_ACCELEROMETER_XOUT_L    (0x08)
#define KX132_1211_ACCELEROMETER_XOUT_H    (0x09)
#define KX132_1211_ACCELEROMETER_YOUT_L    (0x0A)
#define KX132_1211_ACCELEROMETER_YOUT_H    (0x0B)
#define KX132_1211_ACCELEROMETER_ZOUT_L    (0x0C)
#define KX132_1211_ACCELEROMETER_ZOUT_H    (0x0D)


// Given Zephyr's lower level I2C API, our sensor query type commands
//  are effectively one byte in size and contain the address of the
//  sensor configuration register from which to read:

#define CMD_KX132_REQUEST_MANUFACTURER_ID { MAN_ID }
#define CMD_KX132_REQUEST_PART_ID { PART_ID }



// - DEV 1120 BEGIN - better naming . . .
#define KX132_CNTL1  (0x1B)
#define KX132_ODCNTL (0x21)

// - DEV 1120 END -


#endif // KX132_REGISTERS_H
