# Table of Contents

1. [Basic Build and eCalc Statistics](#1-basic-build-and-ecalc-statistics)
2. [Physical Assembly](#2-physical-assembly)
   - [Flight Controller](#flight-controller)
   - [ESC](#esc)
   - [Raspberry Pi](#raspberry-pi)
3. [Firmware](#3-firmware)
   - [Flight Controller - ArduPilot/Kakute H7](#flight-controller---ardupilotkakute-h7)
   - [ESC](#esc-1)
   - [Raspberry Pi](#raspberry-pi-1)
4. [Initial Setup](#4-initial-setup)
   - [Raspberry Pi Antenna Setup](#raspberry-pi-antenna-setup)
   - [FC Config](#fc-config)

## 1. Basic Build and eCalc Statistics
To confirm the performance of our proposed custom drone build, we use Ecalc Software. Ecalc takes in information about your custom drone, and predicts metrics such as Thrust:Weight and flight time

Basic Drone parts include, Frame, ESC, Motors, Props, and Batteries. Ecalc will also take into account the power draw of any drone components like Flight controller, FPV camera, and antenna. 
To begin we will make an Excel file with all of proposed parts, with summed weights, and power draws (not including motors)...

![My Image](documentation_images/Final_robotics_drone_excel.png)

Note the totaled weights and power draw. Power draw does NOT include motors. 

Next step is to input this information into Ecalc ... 

![My Image](documentation_images/ecalc_performance_stats_robotics_drone.png)

Note we are using "Including Drive" as our weight option. This is because we have chosen all parts and know the actual weight. The other options beside "include drive" will asume your total weight doesn't include particular components and try to add those weights for you. In this case we can ignore the warning related to total weight, because the estimated weight that ecalc thinks this build should weigh is less than our actual parts. But since we went to the trouble of finding the actual total weight of our drone, we can ignore this. 

Take aways:

In order for our drone to be safely fly, Load, Current, Est. Temp, and Thrust-weight all have to be in the green. If these metrics are in yellow and red, drone my not be able to fly, or prone to damage itself. 

Flight time and Specific Thrust can be in Yellow or Red. Since we are going for an indoor drone with reasonably long flight time, our specific thrust is in the yellow. This is okay because specific thrust relates how effienctly our drone is turning wattage into thrust, and does not relate ablitly to drone to fly or possibly overheat. 

With this ecalc calculation we can confirm our drone will operate correctly with this combination of parts. 

## 2. Physical Assembly
### Flight Controller
#### Raspberry Pi
#### Electronic Speed Controller (ESC)
#### First-Person View (FPV) Camera
### ESC
#### Battery Connector
#### Motors
### Raspberry Pi
#### Button

## 3. Firmware
### Flight Controller - ArduPilot/Kakute H7
#### STM32Cube Programmer and ArduCopter ISO
#### Mission Planner
### ESC
#### BLHeli Suite
### Raspberry Pi
#### Raspbian Image Flasher
#### MAVProxy

## 4. Initial Setup
### Raspberry Pi Antenna Setup
#### Flight Controller Settings
#### Raspberry Pi Settings
#### MAVProxy Relay
### Flight Controller Configuration
#### Motor, Frame, Propeller, and Battery
#### Failsafes and Warnings
#### Calibration


