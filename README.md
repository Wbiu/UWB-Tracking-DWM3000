# UWB Tracking DWM3000
This repository contains the firmware for the esp32 + DWM300 Tag and Anchor.

## Description 
This was used in the [WVR Glove](https://github.com/Wbiu/WVR-GLOVE-Firmware) as the alternative tracking solution, since no VR hardware was not used.<br/>
The DWM3000 API from  [Makersfabs](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000) was used and 5 Of the UWM3000 are at least needed.


## Positioning system
The positioning calculation is based on [this](https://www.instructables.com/ESP32-UWB-Indoor-Positioning-Test/) algorithm and Two-way-ranging single-sided (TWR SS) was used.

The calculated positions are transmitted over esp now.
