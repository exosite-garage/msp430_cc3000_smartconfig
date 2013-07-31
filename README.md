About MSP430 FRAM CC3000 Smart Config Sensor Cloud
================================================================================
This project is an embedded software project meant to run on the Texas
Instruments MSP430 FRAM development kit to demonstrate sending sensor data
to the cloud over CC3000 WiFi.

The project functionality includes:

1) Reading from ADC inputs (A0, A1, A2, A4/Thermistor, A5, A7, A12/X-Axis, A13/
   Y-Axis, A14/Z-Axis, A15).

2) Using CC3000 WiFi to write ADC data to Exosite

3) Using CC3000 WiFi to read command/control data from Exosite

4) Turning on and off LED7 based on Cloud-based control data

License is BSD, Copyright 2013, Exosite LLC (see LICENSE file)

Tested with:
* IDE and Toolchain: Code Composer v5 (Version: 5.3) for Windows
* MSP430 Compiler 4.1.2
* FRAM development kit MSP-EXP430FR5739 (5/20/2011 seal date)
* CC3000 patch version 1.11
* LS Research CC3000 module - TiWi-SL Rev 2 or
  muRata CC3000 module - LBWA1ZZVK7 Rev 2.0

================================================================================
Quick Start
================================================================================   
1) Download the repository containing this readme file (msp430_cc3000_smartconfig) 
   and extract to a folder of your choosing

2) Go to the folder flashtool and install the flashtool.msi

3) Attach the FRAM board to your PC via the mini-USB connector, make sure the CC3000 module is on the devkit

4) If you never update the CC3000 driver patch, open Start -> All Programs -> flashtool, then run download_cc3000_patch_programmer_driver

5) If you never update the CC3000 firmware patch, Open Start -> All Programs -> flashtool, then run download_cc3000_patch_programmer_firmware

6) Open Start -> All Programs -> flashtool, then run download_exosite_demo

7) Open a browser and go to https://ti.exosite.com and sign-up if you don't
   already have an account
   
8) In the TI Portal, add a new MSP430 + CC3000 device, enter the MAC address from the sticker on the WiFi module.

9) Press S1 until LED6 starts blinking. Refer http://processors.wiki.ti.com/index.php/CC3000_Wi-Fi_MSP430_FRAM_Getting_Started_Guide_Using_Smart_Config
    to config WiFi AP.

10) Verify LED1, followed by LED2, LED3 and finally LED4 all come on
- LED1 is a "communications with WiFi module is working" indicator
- LED2 is a "WiFi signal with appropriate credentials is associated" indicator
- LED3 is a "WiFi signal has been connected" indicator
- LED4 is "Internet communications are working" indicator

11) Verify data in Exosite Portals is being reported to the data sources.  Data
    is written roughly every 10 seconds to all data sources, while data is read
    every two seconds.
    
12) Verify the kit can receive data from Exosite - go to the /manage/data page 
    and click on the LED7 Control data source and enter a '0' in the Write Data 
    block - LED7 on the MS430 FRAM will turn off.

13) Disconnect from your PC and run anywhere you have a 5v power mini-USB 
    connector available!

14) See the file utils.h for error codes to assist with troubleshooting.  The
    errorhandler will blink LED 7 rapidly for a couple of seconds and then it
    will blink slowly for the following codes:
- 1 blink - not associated with an access point (check SSID/passphrase)
- 2 blinks - cannot connect to the Exosite server (check Internet connection)
- 3 blinks - cannot write (check signal strength / distance from AP)
- 4 blinks - communiations the SPI module experience error, retry
- 5 blinks - rare race condition experienced, retry

================================================================================
Release Info
================================================================================
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Release 2013-07-31
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
--) Add flash tool to update fireware
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Release 2013-07-15
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
--) Update HTTP response parsing in device activation function.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Release 2013-06-10
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
--) Update CC3000 host driver and reduce stack usage in Exosite library.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Release 2013-05-06
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
--) Add WDT and update CC3000 patch program
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Release 2013-03-06
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
--) initial release
