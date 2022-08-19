// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- BAROMETER & TEMPERATURE SENSOR CONFIGURATION -- Bosch BMP388
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- BMP388 SENSOR LIBRARY
#include "BMP388_DEV.h"
BMP388_DEV bmp388;


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BARO MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// Operation Mode during measurement
// 1: NORMAL_MODE - continuous measurements with defined frequency (allows for activation of IIR filter); 2: FORCED_MODE - single measurement (less power consumption)
#define BARO_MODE 2

// -- NORMAL Mode timing
// Sampling period (time between two measurements)
// Valid Parameters: TIME_STANDBY_XXMS, Where XX can be:
// 10, 20, 40, 80, 160, 320, 640, 1280, 2560, 5120, 10240, 20480, 40960, 81920, 163840, 327680, 655360 [in ms]
#define BARO_TIMING TIME_STANDBY_10MS

// -- DATA RETRIEVAL interval
// Interval in [ms] at which data is read from BMP388 (earliest). In NORMAL_MODE has to be > BARO_TIMING, otherwise errors might occur.
int     BARO_MINT = 200;

// Measurement Accuracy (Oversampling)
//  SETTING               |   BAROMETER RESOLUTION    |   THERMOMETER RESOLUTION    |   Data packet size (Transmission)   |   MODE NAME
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// 	OVERSAMPLING_SKIP     |   16 bit / 2.64 Pa        |   16 bit / 0.0050 °C        |   16 bit each                       |   Ultra low power
//  OVERSAMPLING_X2       |   17 bit / 1.32 Pa        |   17 bit / 0.0025 °C        |   24 bit each                       |   Low power 
// 	OVERSAMPLING_X4       |   18 bit / 0.66 Pa        |   18 bit / 0.0012 °C        |   24 bit each                       |   Standard resolution   
//	OVERSAMPLING_X8       |   19 bit / 0.33 Pa        |   19 bit / 0.0006 °C        |   24 bit each                       |   High resolution   
//	OVERSAMPLING_X16      |   20 bit / 0.17 Pa        |   20 bit / 0.0003 °C        |   24 bit each                       |   Ultra high resolution   
//	OVERSAMPLING_X32      |   21 bit / 0.085 Pa       |   21 bit / 0.00015 °C       |   24 bit each                       |   Highest resolution   
#define BARO_OVERSAMPLE OVERSAMPLING_SKIP
#define TEMP_OVERSAMPLE OVERSAMPLING_SKIP

// -- Infinite Impulse Response (IIR) Filter (filters sudden changes in data e.g. caused by slamming a door/window)
//  SETTING               |   Typ. Noise @ Ultra low power    |   Typ. Noise @ Standard    |   Typ. Noise @ Highest resolution
// ---------------------------------------------------------------------------------------------------------------------------------
// 	IIR_FILTER_OFF        |   6.6 Pa                          |   3.2 Pa                   |   1.2 Pa
//  IIR_FILTER_2          |   3.8 Pa                          |   1.8 Pa                   |   0.7 Pa
// 	IIR_FILTER_4          |   2.5 Pa                          |   1.2 Pa                   |   0.4 Pa   
//	IIR_FILTER_8          |   1.7 Pa                          |   0.8 Pa                   |   0.3 Pa   
//	IIR_FILTER_16         |   1.2 Pa                          |   0.6 Pa                   |   0.3 Pa   
//	IIR_FILTER_32         |   0.8 Pa                          |   0.4 Pa                   |   0.1 Pa   
//	IIR_FILTER_64         |   0.6 Pa                          |   0.3 Pa                   |   0.1 Pa 
//	IIR_FILTER_128        |   0.4 Pa                          |   0.2 Pa                   |   <0.1 Pa 
#define BARO_IIR IIR_FILTER_OFF

// -- TRANSMIT DATA
// Transmit Temerature & Air Pressure via LoRa? 0: NO; 1: YES
#define BARO_TRANS 1
// Transmit Altitude via LoRa? 0: NO; 1: YES
#define BARO_TRANS_ALT 1


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BARO GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Measurement Variables
float temperature, pressure, altitude;    // Variables for single measurements
float SUM_BARO = 0;                       // Pressure: SUM of all measurements in one measurement cycle
float SUM_TEMP = 0;                       // Temperature: SUM of all measurements in one measurement cycle
float SUM_ALTI = 0;                       // Altitude: SUM of all measurements in one measurement cycle
uint   MCTR_BARO = 0;                     // BARO measurement counter (theoretical number of performed measurements)
uint   VCTR_BARO = 0;                     // BARO value counter (number of collected values)
float AVG_BARO = 0;                       // Pressure: Average value of all measurements in one measurement cycle
float AVG_TEMP = 0;                       // Temperature: Average value of all measurements in one measurement cycle
float AVG_ALTI = 0;                       // Altitude: Average value of all measurements in one measurement cycle


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BARO FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- BARO INITIALIZE
// Initializes Barometer (BMP388) (if active)
void BARO_Init () {
  SP.print("BARO\t");  
  if (SET_BARO) {
    //Reset Variables    
    SUM_BARO = 0, SUM_TEMP = 0, SUM_ALTI = 0;
    MCTR_BARO = 0, VCTR_BARO = 0;
    AVG_BARO = 0, AVG_BARO = 0, AVG_ALTI = 0;
    //Initialize Hardware
    SP.print("Initializing Barometer & Thermometer.. ");
    if (bmp388.begin()) {                                     //  BMP INITIALIZE --> Default initialisation, places the BMP388 into SLEEP_MODE
      SP.println("OK!");     
    } 
    else SP.println("ERROR! BARO could not be initialized!");                                            
  }
  else {
  SP.println("OFF");
  bmp388.begin();
  bmp388.stopConversion();  
  }   
}

// -- BARO SETUP
// Setup Barometer/Temperature measurements (BMP388) (if active)
void BARO_Setup () {
  if (SET_BARO) {
    //Setup Measurement            
    SP.print("BARO\tSetting Up Barometer & Thermometer Measurement.. ");
    bmp388.setPresOversampling(BARO_OVERSAMPLE);             // Set Pressure Oversampling
    bmp388.setTempOversampling(TEMP_OVERSAMPLE);             // Set Temperature Oversampling
    bmp388.setIIRFilter(BARO_IIR);                           // Set IIR Filter
    if (BARO_MODE == 1) {
      SP.print("Normal Mode.. ");     
      bmp388.setTimeStandby(BARO_TIMING);             // Set the standby time
      bmp388.startNormalConversion();                 // Start BMP388 continuous conversion in NORMAL_MODE
      delay(5);
      SP.println("OK!");
    }    
    else if (BARO_MODE == 2) {
      SP.print("Forced Mode.. ");
      bmp388.startForcedConversion();                 // Start single BMP388 measurement in FORCED_MODE
      delay(5);
      SP.println("OK!");
    }
    else SP.println("BARO\tERROR, unknown measurement mode!");
  }     
}

// -- BARO MEASURE
// Make single measurement
void BARO_Measure () {
  if (SET_BARO) {
    MCTR_BARO++;
    if (BARO_MODE == 2) {
      bmp388.startForcedConversion();                        // Start single BMP388 measurement in FORCED_MODE   
      delay(5);                                              // Required to allow for measurement
    }      
    if (bmp388.getMeasurements(temperature, pressure, altitude)) {             // Read data        
      SUM_TEMP = SUM_TEMP + temperature;                                       // Temperature in °C
      SUM_BARO = SUM_BARO + pressure;                                          // Pressure in hPa
      SUM_ALTI = SUM_ALTI + altitude;                                          // Altitude in m
      VCTR_BARO++;
    } 
    else SP.print("BARO\tERROR during measurement (not complete)!");
  }    
}

// -- BARO FINALIZE
// Shut down sensor and evaluate data (calculate averages)
void BARO_Finalize () {
  if (SET_BARO) {
    //SET Sensor to sleep mode
    bmp388.stopConversion();
    //Calculate and output Averages
    AVG_TEMP = SUM_TEMP / VCTR_BARO;
    AVG_BARO = SUM_BARO / VCTR_BARO;
    AVG_ALTI = SUM_ALTI / VCTR_BARO;
    // Output Results
    sprintf(report, "BARO\tTemperature\t%.2f\t°C\t%d\t%d", AVG_TEMP, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "BARO\tAir pressure\t%.2f\thPa\t%d\t%d", AVG_BARO, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "BARO\tAltitude\t%.2f\tm\t%d\t%d", AVG_ALTI, VCTR_BARO, MCTR_BARO);
    SP.println(report);
  }
}      