KX132 Zephyr Driver Roadmap
2023-01-16


OVERVIEW
********

This driver written in C designed to work with Zephyr RTOS, as of its releases 2.6.0 and 3.2.0.  Significant parts of this driver follow the design of STMicro's IIS2DH driver, specifically around the support of I2C and SPI bus connections and the use of Zephyr device tree macro API to allow their selection in an application project, nearly fully in device tree overlay files.


FILES AND FEATURE FACTORING
***************************

kx132-1211.h          . . . primary driver header file, defines:

                            *  Zephyr RTOS sensor data and config structures
                            *  extra Zephyr sensor channels to read KX132 specific parameters
                            *  custom enum to support Zephyr's sensor attribute set and get API convention
                            *  KX132 possible I2C device addresses, per TN027-Power-On-Procedure.pdf

kx132-1211.c          . . . implements Zephyr sensor APIS

                            *  _attr_set
                            *  _attr_get
                            *  _sensor_fetch
                            *  _sensor_get

                            and instantiates sensor structure instances for each KX132
                            found in project device tree.

kx132-register-interface.h
                      . . . function pointer type defines for low level register read and write functions

kx132-i2c.[ch]        . . . low level I2C register read and write routines

kx132-spi.[ch]        . . . low level SPI register read and write routines

kx132-registers.h     . . . defines:

                            *  symbol names for sensor registers, some bit-wise flag names
                            *  prototypes for low level register read and write wrappers
                            *  prototypes for defined sensor config sequences
                            *  prototypes for sensor parameter fetch (read) routines

kx132-registers.c     . . . implements:

                            *  implements defined sensor configuration sequences
                            *  implements sensor parameter fetch (read) routines

kx132-triggers.[ch]   . . . 

out-of-tree-drivers.h
                      . . . enumerated driver routine return values



CONTRIBUTORS
************
This driver developed initially by Ted Havelka.
