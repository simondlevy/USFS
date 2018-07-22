<a href="https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution/"><img src="sentral.jpg" width=300></a>

This repository contains Arduino, Raspberry Pi (WiringPi), and Linux i2cdev
libraries and examples for working with the incredible <a
href="https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution/">
EM7180 Ultimate Sensor Fusion Solution</a> from Pesky Products. 

We have tested this library on the following hardware:

* Ladybug STM32L4 board from Tlera Corp

* Teensy 3.2, 3.6

* Raspberry Pi 3

* NVIDIA Jetson TX1

The library provides two C++ classes for working with the EM7180 SENtral sensor hub on this board:

* An <b>EM7180</b> class providing a convenient API for the full functionality of the EM7180 unit

* An <b>EM7180_Master</b> class that runs the EM7180 in master mode, for simple access to the sensor-fusion features

The <b>examples</b> directory contains sketches showing how to use these two classes. As usual, just clone the repo
into your Arduino libraries folder to get started. The class library and
examples were adapted from Kris Winer's [repository](https://github.com/kriswiner/EM7180_SENtral_sensor_hub).
We strongly recommend reading Kris's  [wiki](https://github.com/kriswiner/EM7180_SENtral_sensor_hub/wiki) for
a comprehensive overview of the EM7180 and other sensor-fusion solutions.

RaspberryPi users should download and install [WiringPi](http://wiringpi.com/),
then cd to <b>EM7180/extras/WiringPi/examples</b>, and run <b>make</b>
to build the examples.

Users of NVIDIA Jetson and other Linux-based boards can cd to <b>EM7180/extras/i2cdev/examples</b>, and run <b>make</b>.
