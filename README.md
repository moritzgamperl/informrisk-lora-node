# Informrisk-Lora-Node

This repository hosts the firmware for the Inform@Risk LoRa Nodes. Brief details about the software and hardware used in the project is mentioned below. For further details, visit the following websites:
1. More general information about the project and contact information can be found on the [AlpGeorisk Website](https://www.informrisk.alpgeorisk.com)
2. General Overview of the project and it's implementation can be found on the [Project Website](https://www.bmbf-client.de/projekte/informrisk)
3. All Information on the hardware setup for these nodes can be found on [www.informrisk.alpgeorisk.com](https://www.informrisk.alpgeorisk.com)???? Specific website?

<!---
The software comprises five stages. Although these stages are fixed, multiple parameters can be changed to accommodate for varying on-site requirements. For example, the overall measurement duration as well as the measurement frequency for each sensor can be changed. These parameters can not only be changed when installing the node but also remotely via commands transmitted via LoRa communication. Also, for each sensor it is possible to decide whether median or mean values should be sent to the gateway. For example for the accelerometer, it is best to calculate the median of the measured values since it is less sensitive to outliers from external influences (e.g. vibrations, impacts). For all other values, usually mean values are chosen. The calculation is done by the microprocessor to save power and on-air time during LoRa communication, which uses the most power.
<!---	        
After the measurement, calculations and LoRa-uplink, the device receives an optional downlink from the gateway. This can be one of several predefined commands such as setting the looptime (time taken for measurements and time the device is asleep), changing measurement duration or activating/deactivating individual sensors on the device. After receiving commands, the device makes the changes and goes into a sleep mode, where current consumption is minimized.
-->
<br />

## Firmware Structure

The firmware is structured in 12 files, whose functions are listed briefly in the following table:

**File**                             |  **Function**
-------------------------------------|----------------------------------------------------
LoRaBasis_InformRisk_V1.2_LORA.ino   |   Main Code (loop, measurements, data transmission)
00_general_config.h                  |   Editable Settings -- Basic setup of Inform@Risk measurement node
AD12_config.h                        |   12-Bit Analog to Digital Conveter (AD12) Sensor Configuration
AD24_config.h                        |   24-Bit Analog to Digital Conveter (AD24) Sensor Configuration
arduino_secrets.h                    |   This file contains sensitive infomation (passwords etc.) encrypted on Arduino
BARO_config.h                        |   Barometer & Temperature Sensor Configuration
BATV_config.h                        |   Battery Voltage Sensor Configuration 
extra_functions.h                    |   Programs required for use of Inform@Risk Measurement Node
IMU_config.h                         |   IMU Sensor Configuration 
INCL_config.h                        |   Inclination Sensor Configuration 
setup.h                              |   This file contains the setup for the Inform@Risk PCB 
SMP_config.h                         |   Subsurface Measurement Probe / Low Cost Inclinometer Configuaton Settings

When setting up a node, changes should only be made in the **00_general_config.h** and **arduino_secrets.h** files. All flexible parameters in all other files can be changed using the general config file.
<br />

## Sensor Libraries

The following libraries are needed for the sensors that can be attached to the nodes:

**Library**                | **Hardware**                               | **Function**
---------------------------|--------------------------------------------|----------------------
BMP388_DEV.h               |  Bosch BMP388                              |  BMP388 Sensor Library
I2Cdev.h, ICM20948_WE.h    |  ICM20948                                  |  ICM20948 Sensor Library & Dependencies
SPI.h, SCL3300.h           |  Murata SCL3300                            |  Murata SCL3300 Sensor Library & Dependencies
arduino_bma456.h           |  Seeed Studio Step Counter / Bosch BMA456  | Subsurface Measurement Probe/ Inclinometer Configurations
agr_ads1220.h              | AGR ADS1220 Library (edited by AlpGeorisk) | Texas Instruments ADS1220 

The library **agr_ads1220.h** is an adaptation of the 'adafruit ads1220' library and can be accessed in this repository ?????

<br />

## Controllable Parameters

The parameters which can be controlled in the general_config.h file are listed in the following sections. For further information, refer to the individual files or the hardware file ????

### Sensor State

These parameters control the overall state of the sensors. Here, individual sensors can be turned on/off with one variable (1 = on, 0 = off). If they are set to off, they will not be initialized or measured. 

**Parameter**  | **Description**
---------------|-------------
SET_BATV       |    Switch on or off ON-Board Battery Input Voltage Measurement 
SET_BARO       |    Switch on or off ON-Board Barometer and Thermometer (BMP388)
SET_IMU        |    Switch on or off ON-Board Inertial Measurement Unit IMU (ICM20948)
SET_AD24       |    Switch on or off ON-Board ADC 24bit (ADS1200)
SET_AD12       |    Switch on or off ON-Board ADC 12bit (Arduino)
SET_INCL       |    Switch on or off ON-Board High Accuracy inclination sensor (Murata SCL3300)
SET_SMP        |    Switch on or off External Subsurface Probes OR Low Cost Inclinometer (ONLY inclination sensors)

### Active Power Ports

These parameters define which power ports should be active during measurements (does not apply to the SMP measurements)

**Parameter** | **Description**
--------------|-------------
SET_V12       |  Switch on or off SW12V
SET_SW3V3     |  Switch on or off First (Standard) SW3V3
SET_SW3V3A    |  Switch on or off Second SW3V3, As defined below:
PW_3V3_A      |  Switch on or off Possible Values: AIN0_12 (AIn0 12bit), AIN1_12 (AIn1 12bit), CS_INKL (Port CP of INKL connector, only if INKL not active/connected) 
SET_SW3V3B    |  Switch on or off Third SW3V3, As defined below:
PW_3V3_B      |  Switch on or off Possible Values: AIN0_12 (AIn0 12bit), AIN1_12 (AIn1 12bit), CS_INKL (Port CP of INKL connector, only if INKL not active/connected)


### Timing parameters

Timing parameters for loop intervals during measurement cycles. 

**Parameter**    | **Description**
-----------------|-------------
loopintv         |  Measurement loop interval (ms) -- Defines measurement interval.
sensorstarttime  |   time to wait after sensors are powered up
measlength       | measurement duration (ms) for all sensors -- Defines how long sensors should be active.

### LORA(R) Settings

These parameters define the LORA connection settings for data transmission.

**Parameter**      | **Description**
-------------------|-------------
LORA_BAND          |   LoRa Band: EU868 - Europe,  US915 - USA/Colombia, AS923 - Australia
LORA_ADR           |   LoRa Automatic Data Rate: TRUE - Data rate is controlled by LoRa Server / FALSE - Data rate is set to a fixed value
LORA_DR            |   LoRa Data Rate: 1 ... 10. Values depend on selected LoRa Band. See LoRaWAN Regional Parameters for Details
LORA_CH_DEF        |   LoRa Channel Plan: TRUE - Use default channel plan for selected Band / FALSE - use channel plan as defined in LORA_CH_ACTIVE
LORA_CH_ACT        |   LoRa Active Channels: {1,2,3,4,5,6,7,8}. Custom list of channels to be used. See LoRaWAN Regional Parameters for Details
LORA_TIMEOUT       |   LoRa Connection Timeout in Seconds


### Other Settings?

1. Packet_Designator - Defines Payload Type
2. SET_SER1  - Turns On/Off the Serial1 port
<br />

## Payload
s
Payload definition for PACKET_DESIGNATOR Type 10:

Defines content of Lora package. Each bit stands for a defined value - if TRUE has been transmitted, if FALSE hat not been transmitted

 Bit    | BYTE 0              |  Type    | BYTE 1             |   Type    | BYTE 2               | Type    | BYTE 3               | Type    | Value of Power of 2 associated with position
 -------|---------------------|----------|--------------------|-----------|----------------------|---------|----------------------|---------|:----:
 BIT 0  | ***BATV***          |  U8      | ***AD24-CH0***     |   I24     | ***SMP-I1B***        |  I16 x3 | ***SMP-I3B***        |  I16 x3 | 1
 BIT 1  | ***BARO-T-B***      |  I16,U24 | ***AD24-CH1***     |   I24     | ***SMP-I1B_TEMP***   |  I8     | ***SMP-I3A_TEMP***   |  I8     | 2
 BIT 2  | ***BARO-ALT***      |  U16     | ***AD24-CH2***     |   I24     | ***SMP-I2A***        |  I16 x3 | ***NOT USED***       |         | 4
 BIT 3  | ***IMU-ACC-XYZ***   |  I16 x3  | ***AD24-CH3***     |   I24     | ***SMP-I2A_TEMP***   |  I8     | ***NOT USED***       |         | 6 
 BIT 4  | ***IMU-GYR-XYZ***   |  I16 x3  | ***AD12-CH0***     |   I16     | ***SMP-I2B***        |  I16 x3 | ***NOT USED***       |         | 16
 BIT 5  | ***IMU-MAG-XYZ***   |  I16 x3  | ***AD12-CH1***     |   I16     | ***SMP-I2B_TEMP***   |  I8     | ***NOT USED***       |         | 32
 BIT 6  | ***INCL-XYZ***      |  I24 x3  | ***SMP-I1A***      |   I16 x3  | ***SMP-I3A***        |  I16 x3 | ***NOT USED***       |         | 64 
 BIT 7  | ***INCL-TEMP***     |  I16     | ***SMP-I1A_TEMP*** |   I8      | ***SMP-I3A_TEMP***   |  I8     | ***NOT USED***       |         | 128

## Downlink commands
			
## Future functions
- [ ] OTA Firmware update
- [ ] More downlink commands if needed 
- [ ] Data storage on Arduino?
