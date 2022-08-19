# informrisk-lora-node

This repository hosts the firmware for the Inform@Risk LoRa Nodes. More on the project on [the AlpGeorisk website](https://www.informrisk.alpgeorisk.com) and [the project website](https://www.bmbf-client.de/projekte/informrisk).

All Information on the hardware setup for these nodes can be found on [www.informrisk.alpgeorisk.com](https://www.informrisk.alpgeorisk.com).

<!---
The software comprises five stages. Although these stages are fixed, multiple parameters can be changed to accommodate for varying on-site requirements. For example, the overall measurement duration as well as the measurement frequency for each sensor can be changed. These parameters can not only be changed when installing the node but also remotely via commands transmitted via LoRa communication. Also, for each sensor it is possible to decide whether median or mean values should be sent to the gateway. For example for the accelerometer, it is best to calculate the median of the measured values since it is less sensitive to outliers from external influences (e.g. vibrations, impacts). For all other values, usually mean values are chosen. The calculation is done by the microprocessor to save power and on-air time during LoRa communication, which uses the most power.
<!---	        
After the measurement, calculations and LoRa-uplink, the device receives an optional downlink from the gateway. This can be one of several predefined commands such as setting the looptime (time taken for measurements and time the device is asleep), changing measurement duration or activating/deactivating individual sensors on the device. After receiving commands, the device makes the changes and goes into a sleep mode, where current consumption is minimized.
-->

## Firmware structure

The firmware is structured in four files, whose functions are listed in the following table.

File | Function
--------------|-------------
informrisk-lora-node.ino  |  main code (loop, measurements, data transmission)
array.h  |   array-function for median
arduino_secrets.h |  App EUI and KEY 
config.h   | configuration file (timing, sensor state, hardware settings)

When setting up a node, changes should only be made in the ''config.h'' and ''arduino_secrets.h'' files. All flexible parameters in the main file can be changed using the config file.

## Sensor libraries

The following libraries are needed for the sensors that can be attached to the nodes. The library **agr_ads1220.h** is an adaptation of the adafruit ads1220 library and can be accessed in this repository.

Library | Function
--------------|-------------
MKRWAN.h; ArduinoLowPower; SPI.h; Wire.h; I2Cdev.h  |  Base libraries for LoRa and com-ports
MPU9250.h; BMP280.h  |   IMU 10DOF
arduino_bma456.h |  Grove Step Counter 
agr_ads1220.h   | Olimex ADS1220 ADC
SCL3300.h | SCL3300 inclinometer

## Controllable parameters

The parameters which can be controlled in the config.h file are listed in the following sections.

### Timing parameters

Parameter | Description
--------------|-------------
loopintv  |  total loop interval in ms
sensorstarttime   |   time to wait after sensors are powered up
measlength   | total measurement duration (all sensors) 
extrafreq | frequency of extra payload being sent instead of base payload 

### Sensor state

These parameters control the overall state of the sensors. Here, individual sensors can be turned on/off with just one variable (1 = on, 0 = off). They will then not be initialized or measured. 

Parameter | Description
--------------|-------------
desig  |  defines type of payload and sensors
SET_IMU   |   turns the IMU on/off
SET_AD24   | turns the ADC on/off
SET_INCL | turns the SCL3300 on/off
SET_SMN | turns the SMN overall on/off (all sensors and measurements)

The designator (desig) defines, which type of LoRa node is used. This has implications for the payload, e.g. a LIN payload does not include StpCtr or groundwater data, while a Subsurface Node usually does not include SCL3300 data. In the future, we may add more desig options, but this has to be implemented on the network side as well.

### Hardware settings

These settings determine which sensor is connected to which port of the circuit board/processor. Using the PCB, this should mostly not have to be changed, except for e.g. the SMN settings.
Every sensor/set of sensors have a "\*_MINT" variable which controls the measurement interval and a "\*_MCTR" variable which is needed to count the measurements per loop cycle. The MINT can be set depending on the supported measurement interval of the sensor.

All variables named "\*_DPORT" and "\*_APORT" determine the digital and analog port of the sensor, respectively.

Information on ADC Settings and Commands will be added here soon.



## Payload

## Downlink commands
			
## Future functions
- [ ] OTA Firmware update
- [ ] More downlink commands if needed 
- [ ] Data storage on Arduino?
