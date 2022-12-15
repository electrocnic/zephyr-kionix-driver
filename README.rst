2022-11-22
==========

Overview
********

This project a Zephyr based sensor driver for Kionix KX132-1211 accelerometer, and follows the design pattern of STMicro's IIS2DH Zephyr driver as found in Zephyr release 3.2.0.  As of this November this driver is a work in progress.  I2C and SPI communications with sensor have been informally tested and observed as working.

A small subset of the KX132's roughly ninety eight config and data registers are configurable by this driver.  Enough registers are configurable to enable x,y,z asynchronous acceleration readings.


Where the driver runs, programmatically
***************************************

As a driver, the code in this repository does not run as a stand-alone app.  There is no "int main" or "void main" routine.  A demo application which builds as a Zephyr RTOS 3.2.0 app can be found at https://github.com/tedhavelka/kionix-driver-demo.  In this repo, cd into `samples/kionix-demo-minimal` and build with `west build -b [name_of_supported_board]` there.  This demo version in the `samples` subdir is just a demo of the driver, really no other features.

Sources in `kionix_driver_demo/src`, the top directory of that just mentioned driver demo project, include some non-demo features from the year 2021.  These will be moved to a subdir or removed in a future code clean up action.  Again for the most straight-forward and easy to understand demo, navigate to, build and use `samples/kionix-demo-minimal` in the Kionix driver demo.
 
KX132 interrupt support in this driver is not yet implemented.  Work underway on this important feature.


Supported boards
****************

NXP's lpcxpresso55s69 presently tested.

Most any board with an MCU for which Zephyr RTOS includes SoC level device tree sources, and in which MXU has at least one I2C and or SPI interface can be used to run this demo.  An appropriate device tree overlay file named `boards/[target_board_name].overlay` at the root of the project will normally suffice to inform Zephyr at compile time of which on-micro-controller peripheral(s) to enable to talk to the KX132 accelerometer.

https://github.com/tedhavelka/zephyr-driver-work-v2/pull/new/sensor-trigger-support


How to build
**************

Driver compilation using a Zephyr toolchain, installed under Ubuntu 20.04 LTS on about 2022 Q2 is tested to produce usable Zephyr based driver demos.  With a Zephyr's needed build tools on hand, compiling the driver demo for this driver is achieved at a Linux shell command line with:

  $ west build -b [board_name]

To change boards, pass along to the build tools the "pristine" or dash p option:

  $ west build -b [board_name] -p

Assuming developer has a physical JLink compatible programmer / debugger, such as Segger JLink or a built-into-board STLink programmer, to flash firmware can be achieved with:

  $ west flash




Commits of Note
***************

* commit 71363c16f8923be80ce9a72c3489adad42c975bb
  This commit shows KX132 trigger callback code responding, a routine in kx132-triggers.c, but seemingly too frequently.  Next step work involves careful review of KX132 datasheet document 'KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf'.
