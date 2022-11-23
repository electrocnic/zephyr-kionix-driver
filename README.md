## 2022-11-22

This project a Zephyr based sensor driver for Kionix KX132-1211 accelerometer.  As of this November this driver is a work in progress.  There is I2C communications with sensor tested and working.  A small subset of the roughly ninety eight config and data registers are configurable by this driver.  Enough registers are configurable to enable x,y,z continuous, asynchronous acceleration readings.

As a driver, the code in this repository does not run as a stand-alone app.  There is no "int main" or "void main" routine.  A demo application which builds as a Zephyr RTOS 3.2.0 app can be found at https://github.com/tedhavelka/kionix-driver-demo.  In this repo, cd into `samples/kionix-demo-minimal` and build with `west build -b [name_of_supported_board]` there.
 
KX132 interrupt support in this driver is not yet implemented.  Work underway on this important feature.


## Supported boards

NXP's lpcxpresso55s69 presently tested.

Most any board with an MXU for which Zephyr RTOS includes SoC level device tree sources, and in which MXU has at least one I2C and or SPI interface can be used to run this demo.  An appropriate device tree overlay file named `boards/[target_board_name].overlay` at the root of the project will normally suffice to inform Zephyr at compile time of which on-micro-controller peripheral(s) to enable to talk to the KX132 accelerometer.

