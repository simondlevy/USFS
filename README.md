<a href="https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution/"><img src="sentral.jpg" width=300></a>

This repository contains a little Arduino/Teensy library and examples for working with the incredible
<a href="https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution/">
EM7180 Ultimate Sensor Fusion Solution</a> from Pesky Products. The library provides two C++ classes for
working with the EM7180 SENtral sensor hub on this board:
<ul>
<li> An <b>EM7180</b> class providing a convenient API for the functionality of the EM7180 unit
<li> An <b>EM7180_Passthru</b> class with a single method, <tt>begin</tt>, that puts the SENtral
unit in pass-through mode, allowing you to access its sensors (IMU, barometer) individually using standard
I<sup>2</sup>C calls.
</ul>

The examples directory contains sketches showing how to use these two classes. As usual, just clone or fork the repo
into your Arduino libraries folder to get started.


