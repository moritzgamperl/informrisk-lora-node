// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- INCLINATION SENSOR CONFIGURATION -- Murata SCL3300
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

#include "SPI.h"
#include "SCL3300.h"
SCL3300 inclinometer;

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BARO MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- OPERATION MODE during measurement
// 1: Acceleration Mode +/- 1,2g; Sensitivity 105/°; 40 HZ
// 2: Acceleration Mode +/- 2,4g; Sensitivity  52/°; 70 Hz
// 3: Inclination Mode  +/-  10°; Sensitivity 209/°; 10 Hz
// 4: Inclination Mode  +/-  10°; Sensitivity 209/°; 10 Hz; Reduced Noise; 
#define INCL_MODE 4

// -- TEMPERATURE MEASUREMENT
// 0: do not include temperature measurement
// 1: include temperature measurement
#define INCL_TEMP 1

// -- FAST READ MODE
// determines in SPI channel is kept open in between measurements. May disturb other measurements on the SPI bus. DEFAULT SETTING: 0 -- Use with caution e.g. if AD24 id deactivated.
// 0: fast read mode OFF
// 1: fast read mode ON
#define INCL_FASTREAD 0

// -- DATA RETRIEVAL interval
// Interval in [ms] at which data is read from INKL (earliest). Depends on measurement Mode (see above). Should not be less than 100 ms.
int     INCL_MINT = 100;

// -- TRANSMIT DATA
// Transmit Tilt/Temperature Data via LoRa? 0: NO; 1: YES
#define INCL_TRANS_TILT 1
#define INCL_TRANS_TEMP 1



// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- INCL GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Measurement Variables
float incl_x, incl_y, incl_z, incl_temp;  // Variables for single measurements
float SUM_INCL_X = 0;                     // Inclination: SUM of all measurements in one measurement cycle
float SUM_INCL_Y = 0;
float SUM_INCL_Z = 0;
float SUM_INCL_TEMP = 0;                  // Temperature: SUM of all measurements in one measurement cycle
uint   MCTR_INCL = 0;                     // INCL measurement counter (theoretical number of performed measurements)
uint   VCTR_INCL = 0;                     // INCL value counter (number of collected values)
float AVG_INCL_X = 0;                     // Inclination: Average value of all measurements in one measurement cycle
float AVG_INCL_Y = 0;
float AVG_INCL_Z = 0;
float AVG_INCL_TEMP = 0;                  // Temperature: Average value of all measurements in one measurement cycle
int   INCL_SLEEP = 0;                     // Sensor is in sleep mode?
uint16_t INCL_ERROR;                      // Error Code


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- INCL FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- INCL INITIALIZE
// Initializes Inclinometer (Murata SCL3300) (if active)
void INCL_Init () {
  SP.print("INCL\t");  
  if (SET_INCL) {
    SP.print("Initializing.. ");
    //Reset Variables    
    SUM_INCL_X = 0, SUM_INCL_Y = 0, SUM_INCL_Z = 0, SUM_INCL_TEMP = 0;
    MCTR_INCL = 0, VCTR_INCL = 0;
    AVG_INCL_X = 0, AVG_INCL_Y = 0, AVG_INCL_Z = 0, AVG_INCL_TEMP = 0;
    //Initialize Hardware
    // WAKEUP SENSOR -- PRODUCES AN ERROR, WHICH INDICATES "Reference Voltage Error" and "Digital Power Error" --> Probably because of deepsleep mode?
    /*if (INCL_SLEEP) {
      INCL_ERROR = inclinometer.WakeMeUp();
      if (!INCL_ERROR) {
        SP.println("WAKEUP OK");                  // INCLINOMETER.WakeMeUp issues "0" if everything is OK!
        delay(1);
        INCL_SLEEP = 0;
      }
      else {
        SP.print("ERROR! WAKEUP failed with error code: ");
        SP.println(INCL_ERROR);
      }        
    }*/
    // INIZIALZE MURATA      
    if (inclinometer.begin(CS_INKL)) SP.println("OK");
    else SP.println("ERROR! INKL not connected?");
    // CHECK IF INIT WAS SUCCESSFUL
    SP.print("INCL\tChecking device.. ");
    if (inclinometer.isConnected()) SP.println("OK");
    else SP.println("ERROR");
  }    
  else {
    SP.println("OFF");
    inclinometer.begin(CS_INKL);    
    inclinometer.powerDownMode();
  }
}

// -- INCL SETUP
// Setup Inclinometer (Murata SCL3300) (if active)
void INCL_Setup () {
  if (SET_INCL) {
    //Setup Measurement
    SP.print("INCL\tSetting up inclinometer measurement (MODE ");
    SP.print(INCL_MODE);
    SP.print(").. ");
    if (inclinometer.setMode(INCL_MODE)) SP.println("OK");
    else SP.println("ERROR");
    if (INCL_FASTREAD) {
      SP.print("INCL\tSetting up fast read mode.. ");
      inclinometer.setFastReadMode();
      SP.println("OK");
    }
    SP.print("INCL\tWaiting for first dataset (max. 1s).. ");
    int timezero = millis();
    while (!inclinometer.available() && (millis()-timezero) < 1000);
    if (inclinometer.available()) SP.println("OK");
    else SP.println("ERROR");
  }     
}    

// -- INCL MEASURE
// Make single measurement
void INCL_Measure () {
  if (SET_INCL) {
    MCTR_INCL++;
    if (inclinometer.available()) {
      incl_x = inclinometer.getTiltLevelOffsetAngleX();
      incl_y = inclinometer.getTiltLevelOffsetAngleY();
      incl_z = inclinometer.getTiltLevelOffsetAngleZ();
      SUM_INCL_X = SUM_INCL_X + incl_x;
      SUM_INCL_Y = SUM_INCL_Y + incl_y;
      SUM_INCL_Z = SUM_INCL_Z + incl_z;
      if (INCL_TEMP) {
        incl_temp = inclinometer.getCalculatedTemperatureCelsius();
        SUM_INCL_TEMP = SUM_INCL_TEMP + incl_temp;                         
      }
      VCTR_INCL++;
    }
    else {
      inclinometer.reset();
      delay(10);
    }
  }
}

// -- INCL FINALIZE
// Shut down sensor and evaluate data (calculate averages)
void INCL_Finalize () {
  if (SET_INCL) {
    //Deactivate fast mode (if active)
    if (INCL_FASTREAD) {
      inclinometer.stopFastReadMode();
      SP.println("OK");
    }    
    //SET Sensor to sleep mode
    INCL_ERROR = inclinometer.powerDownMode();
    if (!INCL_ERROR) INCL_SLEEP = 1;                // INCLINOMETER.powerDownMode issues "0" if everything is OK!
    else {
      INCL_SLEEP = 0;
      SP.println("INCL\tERROR! SLEEP failed with error code: "+INCL_ERROR);             
    }
    //Calculate and output Averages
    AVG_INCL_X = SUM_INCL_X / VCTR_INCL;
    AVG_INCL_Y = SUM_INCL_Y / VCTR_INCL;
    AVG_INCL_Z = SUM_INCL_Z / VCTR_INCL;
    if (INCL_TEMP) AVG_INCL_TEMP = SUM_INCL_TEMP / VCTR_INCL;
    // Output Results
    sprintf(report, "INCL\tTilt X  \t%.3f\t°\t%d\t%d", AVG_INCL_X, VCTR_INCL, MCTR_INCL);
    SP.println(report);
    sprintf(report, "INCL\tTilt Y  \t%.3f\t°\t%d\t%d", AVG_INCL_Y, VCTR_INCL, MCTR_INCL);
    SP.println(report);
    sprintf(report, "INCL\tTilt Z  \t%.3f\t°\t%d\t%d", AVG_INCL_Z, VCTR_INCL, MCTR_INCL);
    SP.println(report);
    if (INCL_TEMP) {
      sprintf(report, "INCL\tTemperature\t%.2f\t°C\t%d\t%d", AVG_INCL_TEMP, VCTR_INCL, MCTR_INCL);
      SP.println(report);      
    }
  }
}