## 2021-10-06
@Brief:  This project builds, and a couple of KX132-1211 configurations are now working via this code for Kionix' KX132-1211 accelerometer.  Basic XYZ single shot readings are also working, via the demo project code which calls this driver.  Demo at https://github.com/tedhavelka/kionix-driver-demo.

Note that to exercise this driver code one must compile it with the demo application, or another Zephyr app which correctly includes and calls its API.  As with Zephyr out-of-tree drivers this driver is not a stand alone app.

To compile this code it is necessary to clone the demo project, have `west` and other Zephyr command line tools installed, `west init` and `west update` per typical Zephyr RTOS application set up steps.


## 2021-08-24

Zephyr RTOS driver for Kionix sensor, second attempt employing a typical Zephyr driver directory structure but this time with more knowledge about cmake, Kconfig and DTS.  Following directory and file layout as observed in 'Air Quality Wing' driver project by Jared Wolff.  Wolff's project hosted at https://github.com/circuitdojo/air-quality-wing-zephyr-drivers.

