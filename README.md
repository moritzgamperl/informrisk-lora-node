# Informrisk-Lora-Node

This repository hosts the firmware for the Inform@Risk LoRa Nodes. Brief details about the software and hardware used in the project is mentioned below. For further details, visit the following websites:
1. More general information about the project and contact information can be found on the [AlpGeorisk Website](https://www.informrisk.alpgeorisk.com)
2. General Overview of the project and it's implementation can be found on the [Project Website](https://www.bmbf-client.de/projekte/informrisk)
3. All Information on the hardware setup for these nodes can be found on [yet to be updated](https://www.informrisk.alpgeorisk.com)
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
general_config.h                     |   Editable Settings -- Basic setup of Inform@Risk measurement node
AD12_config.h                        |   12-Bit Analog to Digital Conveter (AD12) Sensor Configuration
AD24_config.h                        |   24-Bit Analog to Digital Conveter (AD24) Sensor Configuration
arduino_secrets.h                    |   This file contains sensitive infomation (passwords etc.) encrypted on Arduino
BARO_config.h                        |   Barometer & Temperature Sensor Configuration
BATV_config.h                        |   Battery Voltage Sensor Configuration 
extra_functions.h                    |   Programs required for use of Inform@Risk Measurement Node
IMU_config.h                         |   IMU Sensor Configuration 
INCL_config.h                        |   Inclination Sensor Configuration 
setup.h                              |   This file contains the setup for the Inform@Risk PCB 
SMN_config.h                         |   Subsurface Measurement Probe / Low Cost Inclinometer Configuaton Settings

When setting up a node, changes should only be made in the ***general_config.h*** and ***arduino_secrets.h*** files. All flexible parameters in all other files can be changed using the ***general_config.h*** file.
<br />

## Sensor Libraries

The following libraries are needed for the sensors that can be attached to the nodes:

**Library**                | **Hardware**                               | **Function**                                                | Resource
---------------------------|--------------------------------------------|-------------------------------------------------------------|------------------------
SPI.h                      |  ---                                       |  This library allows you to communicate with SPI devices    | This library is bundled with every Arduino platform
I2Cdev.h                   |  ---                                       |  Provides simple and intuitive interfaces to I2C devices    | [Github: I2Cdev.h](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/I2Cdev)
BMP388_DEV.h               |  Bosch BMP388                              |  BMP388 Sensor Library                                      | [Github: BMP388_DEV.h](https://github.com/MartinL1/BMP388_DEV.git)
ICM20948_WE.h              |  ICM20948                                  |  ICM20948 Sensor Library & Dependencies                     | [Github: ICM20948_WE.h](https://github.com/wollewald/ICM20948_WE.git)
SCL3300.h                  |  Murata SCL3300                            |  Murata SCL3300 Sensor Library                              | [Github: SCL3300.h](https://github.com/DavidArmstrong/SCL3300)
arduino_bma456.h           |  Seeed Studio Step Counter / Bosch BMA456  |  Subsurface Measurement Probe/ Inclinometer Configurations  | [Dependencies Folder: arduino_bma456.h](https://github.com/moritzgamperl/informrisk-lora-node/tree/V1.2/Dependencies/AlpGeorisk_BMA456)
agr_ads1220.h              | Texas Instruments ADS1220                  |  AGR ADS1220 Library (edited by AlpGeorisk)                 | [Dependencies Folder: arduino_bma456.h](https://github.com/moritzgamperl/informrisk-lora-node/tree/V1.2/Dependencies/AGR_ADS1220)

The SPI.h is a standard librarie that can be accessed by just including <SPI.h>. The other libraries used are included in the 'Dependencies' folder. 
The library ***agr_ads1220.h*** is an adaptation of the standard 'adafruit ads1220' library. The ***arduino_bma456.h*** is also an adaptation from the [Seeed BMA456 Library](https://github.com/Seeed-Studio/Seeed_BMA456). The modified source codes can be found in the 'Dependencies' folder. 

<br />

## Controllable Parameters

The parameters which can be controlled in the ***general_config.h*** file are listed in the following sections. For further information, refer to the source code or the hardware information file.

### Sensor State

These parameters control the overall state of the sensors. Here, individual sensors can be turned on/off with one variable (1 = on, 0 = off). If they are set to off, they will not be initialized, measured or taken into account for data transfer. 

**Parameter**  | **Description**
---------------|-------------
SET_BATV       |    Switch on or off ON-Board Battery Input Voltage Measurement 
SET_BARO       |    Switch on or off ON-Board Barometer and Thermometer (BMP388)
SET_IMU        |    Switch on or off ON-Board Inertial Measurement Unit IMU (ICM20948)
SET_AD24       |    Switch on or off ON-Board ADC 24bit (ADS1200)
SET_AD12       |    Switch on or off ON-Board ADC 12bit (Arduino)
SET_INCL       |    Switch on or off ON-Board High Accuracy inclination sensor (Murata SCL3300)
SET_SMN        |    Switch on or off External Subsurface Probes OR Low Cost Inclinometer (ONLY inclination sensors)

### Active Power Ports

These parameters define which power ports should be active during measurements (does not apply to the SMN measurements). While the SET_V12 and SET_SW3V3 ports are the regular ports that can be accessed from the connectors on the PCB, the alternative SW3V3_A and SW3V3_B ports are actually the Analog in ports on the Arduino, which can be set to be used as outputs. They are different from the regular SW3V3 and V12 though, as they can supply less current than the latter. 

**Parameter** | **Description**
--------------|-------------
SET_V12       |  Switch on or off SW12V
SET_SW3V3     |  Switch on or off First (Standard) SW3V3
SET_SW3V3A    |  Switch on or off Second SW3V3, As defined below:
PW_3V3_A      |  Switch on or off Possible Values: AIN0_12 (AIn0 12bit), AIN1_12 (AIn1 12bit), CS_INKL (Port CP of INKL connector, only if INKL not active/connected) 
SET_SW3V3B    |  Switch on or off Third SW3V3, As defined below:
PW_3V3_B      |  Switch on or off Possible Values: AIN0_12 (AIn0 12bit), AIN1_12 (AIn1 12bit), CS_INKL (Port CP of INKL connector, only if INKL not active/connected)


### Timing Parameters

Timing parameters for loop intervals during measurement cycles. 

**Parameter**    | **Description**
-----------------|-------------
loopintv         |  Measurement loop interval (ms) -- Defines measurement interval.
sensorstarttime  |  Time to wait after sensors are powered up
measlength       | measurement duration (ms) for all sensors -- Defines how long sensors should be active.

### LoRa(R) Settings

These parameters define the LoRa connection settings for data transmission.

**Parameter**      | **Description**
-------------------|-------------
LORA_BAND          |   LoRa Band: EU868 - Europe,  US915 - USA/Colombia, AS923 - Australia
LORA_ADR           |   LoRa Automatic Data Rate: TRUE - Data rate is controlled by LoRa Server / FALSE - Data rate is set to a fixed value
LORA_DR            |   LoRa Data Rate: 1 ... 10. Values depend on selected LoRa Band. See LoRaWAN Regional Parameters for Details
LORA_CH_DEF        |   LoRa Channel Plan: TRUE - Use default channel plan for selected Band / FALSE - use channel plan as defined in LORA_CH_ACTIVE
LORA_CH_ACT        |   LoRa Active Channels: {1,2,3,4,5,6,7,8}. Custom list of channels to be used. See LoRaWAN Regional Parameters for Details
LORA_TIMEOUT       |   LoRa Connection Timeout in Seconds


### Other Settings

1. Packet_Designator - This defines the payload type, it is automatically updated depending on the sensors that are active
2. SET_SER1  - Default value is 0, then serial data is passed through the USB. If changed to 1, it serial data is sent through the TX/RX pin
3. SleepMode - "deepsleep": most hardware is disabled (lowest power consumption, recommended for deployment) / "sleep": most hardware is disabled, but COM is active / "nil": all hardware stays active.

<br />

## Payload

The payload is defined automatically from the Packet_Designator. Depending on the availabe/functional sensors, the Packet_Designator is defined. This futhermore automatically decides the information to be sent through the LoRa network. The information carried by the Packet_Designator is shown below:
<!---
Todo: Upload a table indicating the information passed through the network. Payload distribution should be indicated. 
-->

## Downlink commands

Downlink commands can be enable to allow changes in the general_config file through the LoRa network. As of now, changes can be made only to the measurement interval. This section can be found under parse commands in the main file.  
1. **commands[i]** - Parameter variable that needs to be changed.
2. **value[i]** - The value to which the parameter passed through command[i] should be changed to.

This section can be further modified to recognize other general parameters and edit them. 
			
