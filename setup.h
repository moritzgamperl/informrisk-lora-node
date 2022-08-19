// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- I@R PCB SETUP
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// DO NOT CHANGE!! This file contains the setup for the I@R PCB. Do not change these parameters.
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// I@R PCB Version 1.0
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- INCLUDE LIBRARIES/FILES 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

#include "arduino_secrets.h"
#include "Wire.h"
#include <MKRWAN.h>               //  Library: MKRWAN from Arduino - Library for Arduino MKR WAN boards
LoRaModem modem;
#include <ArduinoLowPower.h>         //  Library: Arduino Low Power - Library for Low Power mode (requires RTCZero library)
#include <stdio.h>                   //  LIbrary ????

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- DEFINE DIGITAL PORTS & their state during mesurement, and in sleep/off state
// -------------------------------------------------------------------------------------------------------------------------------------------------------

#define PW_VBAT           0
#define PW_VBAT_ON        "OUTPUT,HIGH"            // POWER for VBAT Sensor: IS TURNED ON DIRECTLY AT INITIALIZATION
#define PW_VBAT_OFF       "OUTPUT,LOW"
bool    PW_VBAT_STS =     true;
#define PW1_V12           1
#define PW1_V12_ON        "OUTPUT,LOW"             // POWER for External Sensors 12V  -- DO NOT CHANGE TO "OUTPUT, HIGH" -- ADC should always be initialized first (Relay settings) to make sure 12V output does not destroy ADC in 4-20 mA configuration -- 12V out is turned on after initialization of ADC during measurements
#define PW1_V12_OFF       "OUTPUT,LOW"
bool    PW1_V12_STS =     true;
#define PW_3V3            2
#define PW_3V3_ON         "OUTPUT,LOW"             // POWER for external sensors 3,3V -- DO NOT CHANGE TO "OUTPUT, HIGH" -- ADC should always be initialized first.
#define PW_3V3_OFF        "OUTPUT,LOW"
bool    PW_3V3_STS =      true;
#define PW_IMBA           3
#define PW_IMBA_ON        "OUTPUT,HIGH"            // POWER for IMU/BARO Sensor: IS TURNED ON DIRECTLY AT INITIALIZATION
#define PW_IMBA_OFF       "OUTPUT,HIGH"            // SHOULD BE HIGH, BUT BARO HAS TO BE SET TO LOW POWER MODE
bool    PW_IMBA_STS =     true;
#define DR_ADC24          4
#define DR_ADC24_ON       "OUTPUT,LOW"
#define DR_ADC24_OFF      "OUTPUT,LOW"
bool    DR_ADC24_STS =    true;
#define CS_ADC24          5
#define CS_ADC24_ON       "OUTPUT,LOW"
#define CS_ADC24_OFF      "OUTPUT,LOW"
bool    CS_ADC24_STS =    true;
#define PW2_V12           6
#define PW2_V12_ON        "OUTPUT,LOW"             // DO NOT CHANGE TO "OUTPUT, HIGH" -- ADC should always be initialized first (Relay settings) to make sure 12V output does not destroy ADC in 4-20 mA configuration -- 12V out is turned on after initialization of ADC during measurements
#define PW2_V12_OFF       "OUTPUT,LOW"
bool    PW2_V12_STS =     true;
#define CS_INKL           7
#define CS_INKL_ON        "OUTPUT,LOW"
#define CS_INKL_OFF       "OUTPUT,LOW"             // DEV: UNCLEAR IF THIS SHOULD BE HIGH OR LOW DURING DEEP SLEEP. 
bool    CS_INKL_STS =     true;
#define SPI_MOS           8
#define SPI_MOS_ON        "OUTPUT,LOW"
#define SPI_MOS_OFF       "OUTPUT,LOW"
bool    SPI_MOS_STS =     true;
#define SPI_SCK           9
#define SPI_SCK_ON        "OUTPUT,LOW" 
#define SPI_SCK_OFF       "OUTPUT,LOW"
bool    SPI_SCK_STS =     true;
#define SPI_MIS           10
#define SPI_MIS_ON        "OUTPUT,LOW"    
#define SPI_MIS_OFF       "OUTPUT,LOW"
bool    SPI_MIS_STS =     true;
#define I2C_SDA           11
#define I2C_SDA_ON        "OUTPUT,LOW"   
#define I2C_SDA_OFF       "OUTPUT,LOW"
bool    I2C_SDA_STS =     true;
#define I2C_SCL           12
#define I2C_SCL_ON        "OUTPUT,LOW"   
#define I2C_SCL_OFF       "OUTPUT,LOW"
bool    I2C_SCL_STS =     true;
#define SER_RX            13
#define SER_RX_ON         "OUTPUT,LOW"   
#define SER_RX_OFF        "OUTPUT,LOW"
bool    SER_RX_STS =      true;
#define SER_TX            14
#define SER_TX_ON         "OUTPUT,LOW"  
#define SER_TX_OFF        "OUTPUT,LOW"
bool    SER_TX_STS =      true;

// Combine into arrays
int DPORTS [15]         = {PW_VBAT, PW1_V12, PW_3V3, PW_IMBA, DR_ADC24, CS_ADC24, PW2_V12, CS_INKL, SPI_MOS, SPI_SCK, SPI_MIS, I2C_SDA, I2C_SCL, SER_RX, SER_TX};
String DPORTS_NAME [15] = {"PW_VBAT","PW1_V12","PW_3V3","PW_IMBA","DR_ADC24","CS_ADC24","PW2_V12","CS_INKL", "SPI_MOS", "SPI_SCK", "SPI_MIS", "I2C_SDA", "I2C_SCL", "SER_RX", "SER_TX"};
String DPORTS_ON [15]   = {PW_VBAT_ON, PW1_V12_ON, PW_3V3_ON, PW_IMBA_ON, DR_ADC24_ON, CS_ADC24_ON, PW2_V12_ON, CS_INKL_ON, SPI_MOS_ON, SPI_SCK_ON, SPI_MIS_ON, I2C_SDA_ON, I2C_SCL_ON, SER_RX_ON, SER_TX_ON};
String DPORTS_OFF [15]  = {PW_VBAT_OFF, PW1_V12_OFF, PW_3V3_OFF, PW_IMBA_OFF, DR_ADC24_OFF, CS_ADC24_OFF, PW2_V12_OFF, CS_INKL_OFF, SPI_MOS_OFF, SPI_SCK_OFF, SPI_MIS_OFF, I2C_SDA_OFF, I2C_SCL_OFF, SER_RX_OFF, SER_TX_OFF};


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- DEFINE ANALOG PORTS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------
#define AIN0_12           A0
#define AIN0_12_ON        "INPUT"
#define AIN0_12_OFF       "OUTPUT,LOW"
bool    AIN0_12_STS =     true;
#define AIN1_12           A1
#define AIN1_12_ON        "INPUT"
#define AIN1_12_OFF       "OUTPUT,LOW"
bool    AIN1_12_STS =     true;
#define REL1_A            A2
#define REL1_A_ON         "OUTPUT,LOW"
#define REL1_A_OFF        "INPUT"
bool    REL1_A_STS =      true;
#define REL1_B            A3
#define REL1_B_ON         "OUTPUT,LOW"
#define REL1_B_OFF        "INPUT"
bool    REL1_B_STS =      true;
#define REL2_A            A4
#define REL2_A_ON         "OUTPUT,LOW"
#define REL2_A_OFF        "INPUT"
bool    REL2_A_STS =      true;
#define REL2_B            A5
#define REL2_B_ON         "OUTPUT,LOW"
#define REL2_B_OFF        "INPUT"
bool    REL2_B_STS =      true;
#define AIN_VBAT          A6
#define AIN_VBAT_ON       "INPUT"
#define AIN_VBAT_OFF      "INPUT"
bool    AIN_VBAT_STS =    true;

// Combine into arrays
int APORTS [7]          = {AIN0_12, AIN1_12, REL1_A, REL1_B, REL2_A, REL2_B, AIN_VBAT};
String APORTS_NAME [7]  = {"AIN0_12", "AIN1_12", "REL1_A", "REL1_B", "REL2_A", "REL2_B", "AIN_VBAT"};
String APORTS_ON [7]    = {AIN0_12_ON, AIN1_12_ON, REL1_A_ON, REL1_B_ON, REL2_A_ON, REL2_B_ON, AIN_VBAT_ON};
String APORTS_OFF [7]   = {AIN0_12_OFF, AIN1_12_OFF, REL1_A_OFF, REL1_B_OFF, REL2_A_OFF, REL2_B_OFF, AIN_VBAT_OFF};

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// GENERAL CONFIGURATIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// ------ VARIABLE DEFINTIONS ------ //                
//Timing Variables (all are unsigned long, so overflow of millis() will also cause an overflow of the calculation of the according relative time --> result is OK)
unsigned long loopstart = 0;                 // starttime of loop (ms)
unsigned long looptime = 0;                  // relative time within current loop (ms)
unsigned long measstart = 0;                 // stattime of measurement (ms)
unsigned long meastime = 0;                  // relative time within current measurement (ms)
unsigned long sleeptime = 0;                 // time arduino will sleep until next measurement
unsigned long loopctr = 0;                   // counting the number of loops for extra payload

//PAYLOAD
byte payload_temp[54];                       // MAX PAYLOADSIZE for LORA is 54 byte.
int pctr = 0;                                // Payload position counter
uint8_t vd0 = 0;                             // Value designator BYTE 0
uint8_t vd1 = 0;                             // Value designator BYTE 1
uint8_t vd2 = 0;                             // Value designator BYTE 2
uint8_t vd3 = 0;                             // Value designator BYTE 3

//REPORTS
char report[150];                            // Measurement report for Serial output

//LoRa Connection
int lora_connected = 0;

//LoRa Commands
String commands[10];                                   // Maximum number of commands per message = 10
String values[10];                                     // Maximum number of commands per message = 10
int m = 0;                                             // Command counter
int err;