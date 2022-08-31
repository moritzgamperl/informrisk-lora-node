// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- SUBSURFACE MEASUREMENT PROBE / LOW COST INCLINOMETER (INCLINOMETER(S) ONLY) (Seeed Studio Step Counter // Bosch BMA456)
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

#include "arduino_bma456.h"
BMA456 inkl_a;
BMA456 inkl_b;


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- SMN MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// IMPORTANT !! THE FOLLOWING SETTINGS ONLY CONCERN THE INCLINATION SENSORS OF THE SUBSURFACE MEASUREMENT / LOW COST INCLINOMETER PROBE. 

// To fully activate the SUBSURFACE MEASUREMENT PROBE / LOW COST INCLINOMETER the following settings / cable connections have to be setup:
// - AD24: activate, and set one of the channels to mode "1" (single channel voltage). Connect ORANGE wire (signal out of water pressure sensor) to the according port.
// - SW3V3: connect RED wire (Power source) the SW3V3 port; Connect BLACK wire to any ground (GND) port; in general_config set SW3V3 to "true" --> Power for Measurement of water sensor.
// - SW3V3A (if you use I2A or I2B sensor): set to either AIN0_12 OR AIN1_12 port (in general_config); set SW3V3A to false in general_config, if not needed for other measurement. Connect the BROWN wire to the according port of the AD12 (4 PIN OUTPUT, NOT THE AD24 8 PIN OUTPUT)
// - I2C: Connect the GREEN and YELLOW wire to SDA and SCL of the I2C port respectively.

// -- DEFINE CONNECTED INCLINATION SENSORS OF SMN/LCI
// Up to 4 inclination sensors can be connected. These are from bottom to top: I1A, I1B, I2A, I2B. It is possible to skip one or more sensors, but please always follow this order!
// 0: Sensor not connected/inactive; 1: Sensor connected.
#define I1A 1
#define I1B 0
#define I2A 0
#define I2B 0
#define I3A 0       //NOT IMPLEMENTED YET
#define I3B 0       //NOT IMPLEMENTED YET

// -- DATA RETRIEVAL interval
// Interval in [ms] at which data is read from BMA456 (earliest).
int     SMN_MINT = 200;

// -- TRANSMIT DATA
// Transmit SMN Data via LoRa? 0: NO; 1: YES
#define SMN_TRANS_I1A       1
#define SMN_TRANS_I1A_TEMP  1
#define SMN_TRANS_I1B       1
#define SMN_TRANS_I1B_TEMP  1
#define SMN_TRANS_I2A       1
#define SMN_TRANS_I2A_TEMP  1
#define SMN_TRANS_I2B       1
#define SMN_TRANS_I2B_TEMP  1
#define SMN_TRANS_I3A       1       //NOT IMPLEMENTED YET
#define SMN_TRANS_I3A_TEMP  1       //NOT IMPLEMENTED YET
#define SMN_TRANS_I3B       1       //NOT IMPLEMENTED YET
#define SMN_TRANS_I3B_TEMP  1       //NOT IMPLEMENTED YET

//DEVELOPMENT: ADD MORE SETTINGS? CURRENT DEFAULT SETTINGS:
//Resolution of BMA456 Accelerometer Data is 16bit --> 1g/16384        , Range is -2 ... 2g (MODE RANGE_2G)
//Resolution of BMA456 temperature sensor is  8bit --> 1° (no decimals), Range is -104 ... 150 °C.


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- SMN GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Measurement Variables
float SMN_raw_x = 0, SMN_raw_y = 0, SMN_raw_z = 0;              // Variables for single measurements
float SMN_x = 0, SMN_y = 0, SMN_z = 0;
int8_t SMN_temp = 0;
long  SUM_SMN_TEMP[6], AVG_SMN_TEMP[6];
float SUM_SMN_X[6], SUM_SMN_Y[6], SUM_SMN_Z[6];                 // Variables (array) for SUM
float AVG_SMN_X[6], AVG_SMN_Y[6], AVG_SMN_Z[6];                 // Variables (array) for AVG
bool  INIT_SMN[6];                                              // Sensor initialized
uint  MCTR_SMN[3];                                              // SMN measurement counter (theoretical number of performed measurements)
uint  VCTR_SMN[6];                                              // SMN value counter (number of collected values)

const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- SMN FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- SMN INITIALIZE
// INIT & MEASURE LINE BY LINE (BMA456) (if active)
void SMN_Init_Measure () {
  if (SET_SMN) {
    // Reset Variables
    MCTR_SMN[0] = 0, MCTR_SMN[1] = 0, MCTR_SMN[2] = 0;
    VCTR_SMN[0] = 0, VCTR_SMN[1] = 0, VCTR_SMN[2] = 0, VCTR_SMN[3] = 0, VCTR_SMN[4] = 0, VCTR_SMN[5] = 0;
    SUM_SMN_X[0] = 0, SUM_SMN_X[1] = 0, SUM_SMN_X[2] = 0, SUM_SMN_X[3] = 0, SUM_SMN_X[4] = 0, SUM_SMN_X[5] = 0;
    SUM_SMN_Y[0] = 0, SUM_SMN_Y[1] = 0, SUM_SMN_Y[2] = 0, SUM_SMN_Y[3] = 0, SUM_SMN_Y[4] = 0, SUM_SMN_Y[5] = 0;
    SUM_SMN_Z[0] = 0, SUM_SMN_Z[1] = 0, SUM_SMN_Z[2] = 0, SUM_SMN_Z[3] = 0, SUM_SMN_Z[4] = 0, SUM_SMN_Z[5] = 0;
    SUM_SMN_TEMP[0] = 0, SUM_SMN_TEMP[1] = 0, SUM_SMN_TEMP[2] = 0, SUM_SMN_TEMP[3] = 0, SUM_SMN_TEMP[4] = 0, SUM_SMN_TEMP[5] = 0;
    AVG_SMN_X[0] = 0, AVG_SMN_X[1] = 0, AVG_SMN_X[2] = 0, AVG_SMN_X[3] = 0, AVG_SMN_X[4] = 0, AVG_SMN_X[5] = 0;
    AVG_SMN_Y[0] = 0, AVG_SMN_Y[1] = 0, AVG_SMN_Y[2] = 0, AVG_SMN_Y[3] = 0, AVG_SMN_Y[4] = 0, AVG_SMN_Y[5] = 0;
    AVG_SMN_Z[0] = 0, AVG_SMN_Z[1] = 0, AVG_SMN_Z[2] = 0, AVG_SMN_Z[3] = 0, AVG_SMN_Z[4] = 0, AVG_SMN_Z[5] = 0;
    AVG_SMN_TEMP[0] = 0, AVG_SMN_TEMP[1] = 0, AVG_SMN_TEMP[2] = 0, AVG_SMN_TEMP[3] = 0, AVG_SMN_TEMP[4] = 0, AVG_SMN_TEMP[5] = 0;
    for (int i = 0; i <= 1; i++) {
      //CHECK IF ANY SENSORS ACTIVE IN CURRENT LINE
      if (((i == 0) && (I1A || I1B)) || ((i == 1) && (I2A || I2B))) {
        //SETUP POWER
        if (i == 0) {
          SP.print("SMN\tPowering up LINE 1.. ");
          digitalWrite(PW_3V3, HIGH);
          digitalWrite(PW_3V3_A, LOW);
          SP.println("OK");
        }           
        else if (i == 1) {
          SP.print("SMN\tPowering up LINE 2.. ");
          digitalWrite(PW_3V3, LOW);
          digitalWrite(PW_3V3_A, HIGH);
          SP.println("OK");   
        }
        delay(10);
        // INITIALIZE SENSORS
        if ((i == 0 && I1A) || (i == 1 && I2A)) {
          SP.print("SMN\tInitializing Sensor ");
          SP.print(i+1);
          SP.print("A.. ");
          INIT_SMN[i*2] = !inkl_a.initialize(BMA4_I2C_ADDR_SECONDARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG);                              
          if (INIT_SMN[i*2]) SP.println("OK");
          else SP.println("ERROR"); 
        }
        if ((i == 0 && I1B) || (i == 1 && I2B)) {
          SP.print("SMN\tInitializing Sensor ");
          SP.print(i+1);
          SP.print("B.. ");
          INIT_SMN[(i*2)+1] = !inkl_b.initialize(BMA4_I2C_ADDR_PRIMARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG);                    
          if (INIT_SMN[(i*2)+1]) SP.println("OK");
          else SP.println("ERROR"); 
        }
        delay(175);        
        // -- START SMN MEASUREMENTS        
        SP.print("SMN\tMEASURING LINE ");
        SP.print(i+1);
        SP.println("..");
        uint measstart_SMN = millis();                                                 //Reset Timing & Counters
        while (millis() - measstart_SMN <= measlength) {
          uint meastime_SMN = millis() - measstart_SMN;
          if (SET_SMN && (meastime_SMN >= SMN_MINT * MCTR_SMN[i])) {
            MCTR_SMN[i]++;
            if ((i == 0 && I1A && INIT_SMN[0]) || (i == 1 && I2A && INIT_SMN[2])) {
              SMN_raw_x = NAN, SMN_raw_y = NAN, SMN_raw_z = NAN, SMN_temp = 0;                                // SET raw variables to NAN
              inkl_a.getAcceleration(&SMN_raw_x, &SMN_raw_y, &SMN_raw_z);
              SMN_temp = inkl_a.getTemperature();
              SMN_x = (atan(SMN_raw_x / sqrt(sq(SMN_raw_y) + sq(SMN_raw_z))))*RAD2DEG;
              SMN_y = (atan(SMN_raw_y / sqrt(sq(SMN_raw_x) + sq(SMN_raw_z))))*RAD2DEG;
              SMN_z = (atan(SMN_raw_z / sqrt(sq(SMN_raw_x) + sq(SMN_raw_y))))*RAD2DEG;          
              if (i == 0 && !isnan(SMN_x) && !isnan(SMN_y) && !isnan(SMN_z) && !isnan(SMN_temp)) {              // only consider values if all are NOT NaN
                SUM_SMN_X[0] = SUM_SMN_X[0] + SMN_x;
                SUM_SMN_Y[0] = SUM_SMN_Y[0] + SMN_y;
                SUM_SMN_Z[0] = SUM_SMN_Z[0] + SMN_z;
                SUM_SMN_TEMP[0] = SUM_SMN_TEMP[0] + SMN_temp;
                VCTR_SMN[0]++;                         
              }
              if (i == 1 && !isnan(SMN_x) && !isnan(SMN_y) && !isnan(SMN_z) && !isnan(SMN_temp)) {              // only consider values if all are NOT NaN
                SUM_SMN_X[2] = SUM_SMN_X[2] + SMN_x;
                SUM_SMN_Y[2] = SUM_SMN_Y[2] + SMN_y;
                SUM_SMN_Z[2] = SUM_SMN_Z[2] + SMN_z;
                SUM_SMN_TEMP[2] = SUM_SMN_TEMP[2] + SMN_temp;
                VCTR_SMN[2]++;                        
              }
            }              
            if ((i == 0 && I1B && INIT_SMN[1]) || (i == 1 && I2B && INIT_SMN[3])) {
              inkl_b.getAcceleration(&SMN_raw_x, &SMN_raw_y, &SMN_raw_z);
              SMN_temp = inkl_b.getTemperature();
              SMN_x = (atan(SMN_raw_x / sqrt(sq(SMN_raw_y) + sq(SMN_raw_z))))*RAD2DEG;
              SMN_y = (atan(SMN_raw_y / sqrt(sq(SMN_raw_x) + sq(SMN_raw_z))))*RAD2DEG;
              SMN_z = (atan(SMN_raw_z / sqrt(sq(SMN_raw_x) + sq(SMN_raw_y))))*RAD2DEG;
              if (i == 0) {
                SUM_SMN_X[1] = SUM_SMN_X[1] + SMN_x;
                SUM_SMN_Y[1] = SUM_SMN_Y[1] + SMN_y;
                SUM_SMN_Z[1] = SUM_SMN_Z[1] + SMN_z;
                SUM_SMN_TEMP[1] = SUM_SMN_TEMP[1] + SMN_temp;
                VCTR_SMN[1]++;                                     
              }
              if (i == 1) {
                SUM_SMN_X[3] = SUM_SMN_X[3] + SMN_x;
                SUM_SMN_Y[3] = SUM_SMN_Y[3] + SMN_y;
                SUM_SMN_Z[3] = SUM_SMN_Z[3] + SMN_z;
                SUM_SMN_TEMP[3] = SUM_SMN_TEMP[3] + SMN_temp;
                VCTR_SMN[3]++;                         
              }
            }            
          }
        }         
      }
    }      
  }
}

// -- SMN FINALIZE
// Shut down sensors and evaluate data (calculate averages)
void SMN_Finalize () {
  if (SET_SMN) {
    //TURN OFF sensors -- DEVELOPMENT: PROBABLY MORE COMPLEX PROCEDURE NECCESSARY FOR MULTIPLE SENSORS!! FIRST MAKE SURE ONLY ONE LINE IS POWERED UP, INITIALIZE, PUT TO SLEEP, THEN SENSORS ON OTHER LINE...
    if (I1A || I1B) {
      // KEEP SW3V3 HIGH and set to POWER DOWN MODE
      digitalWrite(PW_3V3, HIGH);
      if (I1A) inkl_a.accel_enable(AKM_POWER_DOWN_MODE);
      if (I1B) inkl_b.accel_enable(AKM_POWER_DOWN_MODE);
      SP.println("OK");
    }
    if (I2A || I2B) {
      // KEEP SW3V3A HIGH and set to POWER DOWN MODE
      digitalWrite(PW_3V3_A, HIGH);
      if (I1A) inkl_a.accel_enable(AKM_POWER_DOWN_MODE);
      if (I1B) inkl_b.accel_enable(AKM_POWER_DOWN_MODE);
    }
    //Calculate and output Averages
    for (int i = 0; i <= 3; i++) {
      AVG_SMN_X[i] = SUM_SMN_X[i] / VCTR_SMN[i];
      AVG_SMN_Y[i] = SUM_SMN_Y[i] / VCTR_SMN[i];
      AVG_SMN_Z[i] = SUM_SMN_Z[i] / VCTR_SMN[i];
      AVG_SMN_TEMP[i] = SUM_SMN_TEMP[i] / VCTR_SMN[i]; 
    }         
    // Output Results
    if (I1A) {
    sprintf(report, "SMN\tI1A Tilt X\t%.2f\t°\t%u\t%u", AVG_SMN_X[0], VCTR_SMN[0], MCTR_SMN[0]);
    SP.println(report);
    sprintf(report, "SMN\tI1A Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMN_Y[0], VCTR_SMN[0], MCTR_SMN[0]);
    SP.println(report);
    sprintf(report, "SMN\tI1A Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMN_Z[0], VCTR_SMN[0], MCTR_SMN[0]);
    SP.println(report);
    sprintf(report, "SMN\tI1A Temp\t%ld\t°C\t%u\t%u", AVG_SMN_TEMP[0], VCTR_SMN[0], MCTR_SMN[0]);
    SP.println(report);         
    }
    if (I1B) {
    sprintf(report, "SMN\tI1B Tilt X\t%.2f\t°\t%u\t%u", AVG_SMN_X[1], VCTR_SMN[1], MCTR_SMN[1]);
    SP.println(report);
    sprintf(report, "SMN\tI1B Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMN_Y[1], VCTR_SMN[1], MCTR_SMN[1]);
    SP.println(report);
    sprintf(report, "SMN\tI1B Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMN_Z[1], VCTR_SMN[1], MCTR_SMN[1]);
    SP.println(report);
    sprintf(report, "SMN\tI1B Temp\t%ld\t°C\t%u\t%u", AVG_SMN_TEMP[1], VCTR_SMN[1], MCTR_SMN[1]);
    SP.println(report);         
    }
    if (I2A) {
    sprintf(report, "SMN\tI2A Tilt X\t%.2f\t°\t%u\t%u", AVG_SMN_X[2], VCTR_SMN[2], MCTR_SMN[2]);
    SP.println(report);
    sprintf(report, "SMN\tI2A Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMN_Y[2], VCTR_SMN[2], MCTR_SMN[2]);
    SP.println(report);
    sprintf(report, "SMN\tI2A Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMN_Z[2], VCTR_SMN[2], MCTR_SMN[2]);
    SP.println(report);
    sprintf(report, "SMN\tI2A Temp\t%ld\t°C\t%u\t%u", AVG_SMN_TEMP[2], VCTR_SMN[2], MCTR_SMN[2]);
    SP.println(report);         
    }
    if (I2B) {
    sprintf(report, "SMN\tI2B Tilt X\t%.2f\t°\t%u\t%u", AVG_SMN_X[3], VCTR_SMN[3], MCTR_SMN[3]);
    SP.println(report);
    sprintf(report, "SMN\tI2B Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMN_Y[3], VCTR_SMN[3], MCTR_SMN[3]);
    SP.println(report);
    sprintf(report, "SMN\tI2B Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMN_Z[3], VCTR_SMN[3], MCTR_SMN[3]);
    SP.println(report);
    sprintf(report, "SMN\tI2B Temp\t%ld\t°C\t%u\t%u", AVG_SMN_TEMP[3], VCTR_SMN[3], MCTR_SMN[3]);
    SP.println(report);         
    }
  }
}
