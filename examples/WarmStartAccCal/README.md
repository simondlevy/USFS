This folder contains a modified version of the Warm-Start / Accelerometer-Calibration 
[sketch](https://github.com/kriswiner/EM7180_SENtral_sensor_hub/tree/master/WarmStartandAccelCal) from Kris Winer:

<ul>
<p><li>Uses the [EM7180](https://github.com/simondlevy/EM7180/blob/master/src/EM7180.h) class
<p><li>Removed the pass-through code 
<p><li>Runs I<sup>2</sup>C over pins 18/19 instead of 16/17
<p><li>Simplified the output to the bare minimum for performing the calibration
</ul>

You can still follow Greg Tomasch's 
[tutorial](https://github.com/kriswiner/EM7180_SENtral_sensor_hub/wiki/F.--Magnetometer-and-Accelerometer-Calibration) 
to calibrate the SENtral sensor hub with this sketch.

