# Klipper-Raise3D-Pro3-Series
This software is a modified version of the open-source Klipper software, specifically tailored for use with the Raise3D Pro3 series. 
It contains special functions that are designed to work with the unique characteristics of the Raise3D Pro3 series models. 
We advise exercising caution and conducting thorough research to ensure that the software is in line with the relevant specifications before use.

## Official website
* [Raise3D Official Website](https://www.raise3d.com)

## Update History:  

#### V1.7.0:  
* Add power loss recovery feature
* Add hot end control board communication detection
* Add automatic frequency calibration function
* Add M280 command to control the probe deployment
* Add M990 command to detect filament runout sensor status
* Add M208 command to set travel limits
* Add M906 command to set motor currents
* Add M557 command to set bed leveling points
* Add M566 command to set jerk speed
* Add M572 command for extrusion optimization
* Add M301, M303, M304 commands to control PID
* Add M558 command to control probe probing speed
* Add G31 command to set probe offset value

## Quick start:  
=================================================================

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

Klipper is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use Klipper.

To begin using Klipper start by
[installing](https://www.klipper3d.org/Installation.html) it.

Klipper is Free Software. See the [license](COPYING) or read the
[documentation](https://www.klipper3d.org/Overview.html).


### Important Notes:  
* Never change the serial port baudrate settings, this will cause the touchscreen cannot connect to the motion controller board.  
* Never change any serial communication protocol or it will make the touchscreen unresponding.  