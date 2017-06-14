Arduino code to control ThermoElectric Cooler (TEC) using PID control based on user input.

Code branches:
* direct
  * controls temperature directly on surface of device
  * uses code for legacy configured 16x2 LCD screen

* 6cm_plate uses updated code for:
    * 16x2 LCD with reduced pins
    * PID equation for measuring temperature of plate _in agaro_
