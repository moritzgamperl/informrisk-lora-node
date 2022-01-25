// -------------------------------------------------------------------------------------------------------------------------------------------------------
// HEADER - INCLUDE LIBRARIES/FILES
// -------------------------------------------------------------------------------------------------------------------------------------------------------

#include <MKRWAN.h>                  //  Library: MKRWAN from Arduino - Library for Arduino MKR WAN boards
LoRaModem modem;
#include <ArduinoLowPower.h>         //  Library: Arduino Low Power - Library for Low Power mode (requires RTCZero library)


// ------ ICM20948 Sensor ------ //
#include "Wire.h"
#include "I2Cdev.h"
#include "ICM20948_WE.h"
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
I2Cdev I2C_M;


// ------ BMP388 Sensor ------ //
#include "BMP388_DEV.h"
BMP388_DEV bmp388;         


// ------ Step Counter ------ //
#include "arduino_bma456.h";


// ------ Olimex ADS1220 Sensor ------ //
#include "agr_ads1220.h";
agr_ads1220 ads1220;


// ------ Define for SAMD processors ------ //
#if defined (ARDUINO_ARCH_SAMD)
//#define SP Serial1USB
#endif
                                                                        // TODO:Setup File - Read from Flash!/Write to Flash
// ------ Custom Header Includes ------ //
#include "arduino_secrets.h"
#include "extra_functions.h"


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// UNIVERSAL CONFIGURATIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// ------ VARIABLE DEFINTIONS ------ //

uint8_t buffer_m[6];                         // For 
float lpot;
float lpotvoltage;

// Security
String appEui = SECRET_APP_EUI;              // Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appKey = SECRET_APP_KEY;              // Please enter your sensitive data in the Secret tab or arduino_secrets.h

//Timing Variables (all are unsigned long, so overflow of millis() will also cause an overflow of the calculation of the according relative time --> result is OK)
unsigned long loopstart = 0;                 // starttime of loop (ms)
unsigned long looptime = 0;                  // relative time within current loop (ms)
unsigned long measstart = 0;                 // stattime of measurement (ms)
unsigned long meastime = 0;                  // relative time within current measurement (ms)
unsigned long sleeptime = 0;                 // time arduino will sleep until next measurement
unsigned long loopctr = 0;                   // counting the number of loops for extra payload


char report[170];                            // Measurement report for Serial output
char report2[170];                           // SMN Measurement report for Serial output

int16_t watertbl = 0;
float watertbl_cal;

int StpCtr1_DPORT = -1;


  
// ------ DEFINE TIMING ------//

#define SP Serial                                                      // Define Serial port to output Serial messages to                                          
                                                                       // LoRa Main Loop Timing; main loop is cycled every 100 ms --> loopintv should be dividable by 100 ms
int loopintv = 300000;                                                 // loop interval (ms)
int sensorstarttime = 500;                                             // time to wait after sensors are powered up
int extrafreq = 100000;                                                // Frequency of extra payload being sent instead of base payload: 10 means that this is sent every 10th time.
                                                                       //Measurement Timing; measurement loop is cycled every 10 ms --> all time intervals should be dividable by 10 ms
int measlength = 5000;                                                 // measurement duration (ms) for all sensors

// ------ SENSOR STATE ------ //

int desig = 1;                                                         // DESIGNATOR defining Payload type and Sensor type -> 1: LoRa Infrastructure Node (LIN), 2: Subsurface Node (SMN or LCI)
int settings_desig = 00001;
bool SET_IMU = false;                                                  // Sensor settings: false to deactivate Sensor in File, true to activate;
bool SET_ADC = false;                                                  // 
bool SET_SCL = false;                                                  // High Accuracy inclination sensor (in most nodes that dont have the IMU)
bool SET_SMN = false;                                                  // Turn additional functions on/off (eg Bodensonde: SET_SMN: "Subsurface Measurement Node")
bool SET_BARO = true;                                        

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// HARDWARE CINFIGURATIONS                                             // Set DPORT to -1 (or any other negative value) to turn off device
// -------------------------------------------------------------------------------------------------------------------------------------------------------

int LED_DPORT = -1;                                                   // Digital port of OnBoard LED; set to "LED_BUILTIN" to activate,
                                                                      //  set to "-1" to deactivate; LED will light up during measurement
// ------ PCB State ------ //
                                                                      // Battery voltage measurement using voltage divider (BATT in; up to XXX Volts; defined by voltage divider resistor values)
int BATT_DPORT = 0;                                                   // Digital port to turn on/off voltage divider; set to -1 to deactivate 
int BATT_APORTin = A6;                                                // AnalogIn port used for batt voltage measurement
int BATT_R1 = 33000;                                                  // Voltage divider resistor 1 in Ohms (used for voltage measurement)
int BATT_R2 = 100000;                                                 // Voltage divider resistor 2 in Ohms
int BATT_MINT = 500;                                                  // measurement interval of the BATT Voltage (ms)
int BATT_MCTR =  1;                                                   // BATT measurement counter -- do not change
int ADC_MINT = 100;                                                   // measurement interval of the ADC (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz
int ADC_MCTR = 1;         
int ADC_CAL = 0;

// IMU Sensor
int IMU_DPORT = 3;                                                    // Digital port to turn on/off Grove IMU 10DOF;  set to -1 to deactivate   // ICM20948
int IMU_MCTR = 1;                                                     // IMU measurement counter -- do not change

// Switched 12V output
int SW12_DPORT = 6;                                                   // Digital port to control 12V output; set to -1 to deactivate

// Switched 3,3V output (turned on during measurements)
int SW33_A_DPORT = 2;                                                 // Digital port to control 3,3V output A; set to -1 to deactivate
int SW33_B_DPORT = -1;                                                // Digital port to control 3,3V output B; set to -1 to deactivate                    


// Buoyancy sensor                                                     //SMN sensor/ICL
int Wtr_DPORT = -1;                                                    // Digital port to turn on/off buoyancy measurement
int Wtr_APORT = A1;                                                    // Digital port of water measurements

// SMN

int SMN_MCTR = 1;
int SMN_MINT = 10;

// Olimex ADS1220                                                      // For the ADC CHIP
#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4

// -- Not sure if they are SCL configs --//
int INKL_MINT = 100;
int INKL_MCTR = 1;

// Not sure which sensor
int MAG_MCTR = 1;         // MAG measurement counter -- do not change
int BARO_MCTR = 1;        // BARO measurement counter -- do not change


//-- IMU CONFIGURATION SETTING, KEPT HERE TEMPORARILY -- //

long meassum[8];                            // Array with summerized measurements // Achtung! Länge = max i + 1, da indizierung mit 0 losgeht
long arr_imu_a_x[100];                       // Array with measurements for imu.a.x Qestion: How to deal with max size of Array? Here, 1000 as default
long arr_imu_a_y[100];
long arr_imu_a_z[100];
long arr_imu_g_x[100];
long arr_imu_g_y[100];
long arr_imu_g_z[100];
long arr_mag_m_x[100];
long arr_mag_m_y[100];
long arr_mag_m_z[100];

float temperature, pressure, altitude;       // Create the temperature, pressure and altitude variables
                                                                                                    
int16_t ax, ay, az;                          // For acceleration values from 
int16_t gx, gy, gz;                          // For  values from 
int16_t mx, my, mz;                          // For  values from 

// Measurement interval and counter are same as for Pololu
int IMU_MINT = 100;       // measurement interval of the IMU (Accelerometer & Gyro) (ms) -- make sure is supported by device; e.g. 100 = 10 Hz
int MAG_MINT = 100;      // measurement interval of the Magnetometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz 
int BARO_MINT = 100;     // measurement interval of the Barometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz


//---------------------------------------------------------------------------------------------------------------------------
//INCLUDE EXTRA CONFIGURATION FILE DEPENDING THE SENSORS ATTACHED
//---------------------------------------------------------------------------------------------------------------------------
#include "SCL_Config.h"
#include "SMN_Config.h"
#include "ADC_Config.h"
