# Table of Contents

1. [Basic Build and eCalc Statistics](#1-basic-build-and-ecalc-statistics)
2. [Physical Assembly](#2-physical-assembly)
   - [Flight Controller](#flight-controller)
   - [ESC](#electronic-speed-controller-esc)
   - [Raspberry Pi](#raspberry-pi)
3. [Firmware](#3-firmware)
   - [Flight Controller - ArduPilot/Kakute H7](#flight-controller---ardupilotkakute-h7)
   - [ESC](#electronic-speed-controller-esc-1)
   - [Raspberry Pi](#raspberry-pi-1)
4. [Initial Setup](#4-initial-setup)
   - [Raspberry Pi Antenna Setup](#raspberry-pi-antenna-setup)
   - [FC Config](#fc-config)

## 1. Basic Build and eCalc Statistics
To confirm the performance of our proposed custom drone build, we use Ecalc Software. Ecalc takes in information about your custom drone, and predicts metrics such as Thrust:Weight and flight time

Basic Drone parts include, Frame, ESC, Motors, Props, and Batteries. Ecalc will also take into account the power draw of any drone components like Flight controller, FPV camera, and antenna. 
To begin we will make an Excel file with all of proposed parts, with summed weights, and power draws (not including motors)...

<img src="documentation_images/Final_robotics_drone_excel.png" width="500" height="300" alt="My Image">

Note the totaled weights and power draw. Power draw does NOT include motors. 

Next step is to input this information into Ecalc ... 

![My Image](documentation_images/ecalc_performance_stats_robotics_drone.png)

Note we are using "Including Drive" as our weight option. This is because we have chosen all parts and know the actual weight. The other options beside "include drive" will asume your total weight doesn't include particular components and try to add those weights for you. In this case we can ignore the warning related to total weight, because the estimated weight that ecalc thinks this build should weigh is more than our actual parts. But since we went to the trouble of finding the actual total weight of our drone, we can ignore this. 

Take aways:

In order for our drone to be safely fly, Load, Current, Est. Temp, and Thrust-weight all have to be in the green. If these metrics are in yellow and red, drone my not be able to fly, or prone to damage itself. 

Flight time and Specific Thrust can be in Yellow or Red. Since we are going for an indoor drone with reasonably long flight time, our specific thrust is in the yellow. This is okay because specific thrust relates how effienctly our drone is turning wattage into thrust, and does not relate ablitly to drone to fly or possibly overheat. 

With this ecalc calculation we can confirm our drone will operate correctly with this combination of parts. 

## 2. Physical Assembly
### Flight Controller

For our flight controller we are using  Kakute h7 mini 1.3s. Compatible with Ardupilot Firmware. 

<img src="documentation_images/kakute_h7_pic.png" width="500" height="300" alt="My Image">


For wiring with PI, we are using Pi only as antenna, so we much make four connections, 5v, Ground, Tx (transmit pin), and Rx (recieve pin). 

![My Image](documentation_images/fc_and_pi.jpg)

For Wiring FPV Camera, our model has it's own antenna, so we only need 5v and GND

![My Image](documentation_images/fc_and_cam.jpg)

### Raspberry Pi
Additionally, we added a button to the raspberry pi in order to properly turn off pi (Pi must be turned off before battery is unplugged, otherwise pi does not shut down correctly). 

![My Image](documentation_images/pi_draw.jpg)

### Electronic Speed Controller (ESC)
Flight controller comes with 6 pin JST connector, but our ESC has a different pin mapping, so the connector has been repinned as such:

![My Image](documentation_images/pi_draw_andFC.jpg)

ESC motor connections in general do not matter, motor will be able to move no matter how the three pins are connected to ESC. The only difference it makes is the direction the motor spins, which you are able to switch in ESC firmware if desired. However for ease of use, we decided to use pin connectors between motor and esc for easy direction swapping...

In addition to motors, a XT30 connector is soldered onto power connections of ESC this connects direclty to 1100mah 2s High Voltage Lipo Battery. 
![My Image](documentation_images/esc_and_motors.jpg)

## 3. Firmware
### Flight Controller - ArduPilot/Kakute H7
For our flight controller, we decided to use ArduCopter firmware. Exact version is included in Firmware folder. 
Or you can download latest version here: https://firmware.ardupilot.org/Copter/
#### STM32Cube Programmer and ArduCopter ISO
Installation guide follows general ArduCopter installation available here: https://ardupilot.org/copter/docs/common-loading-firmware-onto-chibios-only-boards.html

In Short the steps are: 

1.Download arducopter_with_bl.hex (from repo, or latest from: https://firmware.ardupilot.org/Copter/), we want "with_bl" because your esc is using running BLHeli_s...

2.Download and Install STM32CUBEProgrammer software: https://www.st.com/en/development-tools/stm32cubeprog.html

3.While holding down Kakute H7 mini's DFU button, connect to PC via USB

![My Image](documentation_images/kakuteH7miniDFU.png)

4."USB Configuration" hit the refresh button, the port should change from "No DFU detected" to "USB#" # being any number. Hit connect.

![My Image](documentation_images/STM32Cube_connection_ex.png)

Then device information should display at bottom of USB configuration window.

![My Image](documentation_images/STM32Cube_fc_connection_data.png)

5.Now click the "open file" tab, and select the previously downloaded "arducopter_with_bl.hex"

![My Image](documentation_images/STM32Cube_download.png)

After this arducopter will install on flight controller and you can disconnect when you are done...

### Electronic Speed Controller (ESC)
Our esc comes with blheli_s, meaning we will use BLHeliSuite to configure esc settings NOT BLheliSuite32. In our setup the only purpose of using BLHeliSuite is to reverse the direction of motors, without rewiring them. You may not need to use this configuration software you don't need to swap directions without rewiring...

#### BLHeliSuite
1.Download and Launch BLHeliSuite, available here: https://www.mediafire.com/folder/dx6kfaasyo24l/BLHeliSuite

2.Power Flight controller and Esc via battery (FC and ESC are connected via the modified 8-pin JST connector referened in hardware setup for FC). 

3.Once stack is powered by battery, plug flight controller into computer via USB.

4.In BLHeliSuite, click "Select ATMEL/SILABS Interface" drop down, and select "SILABS BLHeli Bootloader (cleanflight)" or the yellow "E". 

![My Image](documentation_images/BLHeliSuite_select.png)

5. Now, in com window, select COM port that FC is currenlty connected under, Hit "connect", then hit "read setup"

![My Image](documentation_images/BLHeliSuite_connect_read.png)

Now you should be connected to ESC settings, and have the ability to switch motor directions of motors, as well as ajust other settings. 






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


