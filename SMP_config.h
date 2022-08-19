// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- SUBSURFACE MEASUREMENT PROBE / LOW COST INCLINOMETER (INCLINOMETER(S) ONLY) (Seeed Studio Step Counter // Bosch BMA456)
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

#include "arduino_bma456.h"
BMA456 inkl_a;
BMA456 inkl_b;


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- SMP MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// IMPORTANT !! THE FOLLOWING SETTINGS ONLY CONCERN THE INCLINATION SENSORS OF THE SUBSURFACE MEASUREMENT / LOW COST INCLINOMETER PROBE. 

// To fully activate the SUBSURFACE MEASUREMENT PROBE / LOW COST INCLINOMETER the following settings / cable connections have to be setup:
// - AD24: activate, and set one of the channels to mode "1" (single channel voltage). Connect ORANGE wire (signal out of water pressure sensor) to the according port.
// - SW3V3: connect RED wire (Power source) the SW3V3 port; Connect BLACK wire to any ground (GND) port; in general_config set SW3V3 to "true" --> Power for Measurement of water sensor.
// - SW3V3A (if you use I2A or I2B sensor): set to either AIN0_12 OR AIN1_12 port (in general_config); set SW3V3A to false in general_config, if not needed for other measurement. Connect the BROWN wire to the according port of the AD12 (4 PIN OUTPUT, NOT THE AD24 8 PIN OUTPUT)
// - I2C: Connect the GREEN and YELLOW wire to SDA and SCL of the I2C port respectively.

// -- DEFINE CONNECTED INCLINATION SENSORS OF SMP/LCI
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
int     SMP_MINT = 200;

// -- TRANSMIT DATA
// Transmit SMP Data via LoRa? 0: NO; 1: YES
#define SMP_TRANS_I1A       1
#define SMP_TRANS_I1A_TEMP  1
#define SMP_TRANS_I1B       1
#define SMP_TRANS_I1B_TEMP  1
#define SMP_TRANS_I2A       1
#define SMP_TRANS_I2A_TEMP  1
#define SMP_TRANS_I2B       1
#define SMP_TRANS_I2B_TEMP  1
#define SMP_TRANS_I3A       1       //NOT IMPLEMENTED YET
#define SMP_TRANS_I3A_TEMP  1       //NOT IMPLEMENTED YET
#define SMP_TRANS_I3B       1       //NOT IMPLEMENTED YET
#define SMP_TRANS_I3B_TEMP  1       //NOT IMPLEMENTED YET

//DEVELOPMENT: ADD MORE SETTINGS? CURRENT DEFAULT SETTINGS:
//Resolution of BMA456 Accelerometer Data is 16bit --> 1g/16384        , Range is -2 ... 2g (MODE RANGE_2G)
//Resolution of BMA456 temperature sensor is  8bit --> 1° (no decimals), Range is -104 ... 150 °C.


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- SMP GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Measurement Variables
float smp_raw_x = 0, smp_raw_y = 0, smp_raw_z = 0;              // Variables for single measurements
float smp_x = 0, smp_y = 0, smp_z = 0;
int8_t smp_temp = 0;
long  SUM_SMP_TEMP[6], AVG_SMP_TEMP[6];
float SUM_SMP_X[6], SUM_SMP_Y[6], SUM_SMP_Z[6];                 // Variables (array) for SUM
float AVG_SMP_X[6], AVG_SMP_Y[6], AVG_SMP_Z[6];                 // Variables (array) for AVG
bool  INIT_SMP[6];                                              // Sensor initialized
uint  MCTR_SMP[3];                                              // SMP measurement counter (theoretical number of performed measurements)
uint  VCTR_SMP[6];                                              // SMP value counter (number of collected values)

const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- SMP FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- SMP INITIALIZE
// INIT & MEASURE LINE BY LINE (BMA456) (if active)
void SMP_Init_Measure () {
  if (SET_SMP) {
    // Reset Variables
    MCTR_SMP[0] = 0, MCTR_SMP[1] = 0, MCTR_SMP[2] = 0;
    VCTR_SMP[0] = 0, VCTR_SMP[1] = 0, VCTR_SMP[2] = 0, VCTR_SMP[3] = 0, VCTR_SMP[4] = 0, VCTR_SMP[5] = 0;
    SUM_SMP_X[0] = 0, SUM_SMP_X[1] = 0, SUM_SMP_X[2] = 0, SUM_SMP_X[3] = 0, SUM_SMP_X[4] = 0, SUM_SMP_X[5] = 0;
    SUM_SMP_Y[0] = 0, SUM_SMP_Y[1] = 0, SUM_SMP_Y[2] = 0, SUM_SMP_Y[3] = 0, SUM_SMP_Y[4] = 0, SUM_SMP_Y[5] = 0;
    SUM_SMP_Z[0] = 0, SUM_SMP_Z[1] = 0, SUM_SMP_Z[2] = 0, SUM_SMP_Z[3] = 0, SUM_SMP_Z[4] = 0, SUM_SMP_Z[5] = 0;
    SUM_SMP_TEMP[0] = 0, SUM_SMP_TEMP[1] = 0, SUM_SMP_TEMP[2] = 0, SUM_SMP_TEMP[3] = 0, SUM_SMP_TEMP[4] = 0, SUM_SMP_TEMP[5] = 0;
    AVG_SMP_X[0] = 0, AVG_SMP_X[1] = 0, AVG_SMP_X[2] = 0, AVG_SMP_X[3] = 0, AVG_SMP_X[4] = 0, AVG_SMP_X[5] = 0;
    AVG_SMP_Y[0] = 0, AVG_SMP_Y[1] = 0, AVG_SMP_Y[2] = 0, AVG_SMP_Y[3] = 0, AVG_SMP_Y[4] = 0, AVG_SMP_Y[5] = 0;
    AVG_SMP_Z[0] = 0, AVG_SMP_Z[1] = 0, AVG_SMP_Z[2] = 0, AVG_SMP_Z[3] = 0, AVG_SMP_Z[4] = 0, AVG_SMP_Z[5] = 0;
    AVG_SMP_TEMP[0] = 0, AVG_SMP_TEMP[1] = 0, AVG_SMP_TEMP[2] = 0, AVG_SMP_TEMP[3] = 0, AVG_SMP_TEMP[4] = 0, AVG_SMP_TEMP[5] = 0;
    for (int i = 0; i <= 1; i++) {
      //CHECK IF ANY SENSORS ACTIVE IN CURRENT LINE
      if (((i == 0) && (I1A || I1B)) || ((i == 1) && (I2A || I2B))) {
        //SETUP POWER
        if (i == 0) {
          SP.print("SMP\tPowering up LINE 1.. ");
          digitalWrite(PW_3V3, HIGH);
          digitalWrite(PW_3V3_A, LOW);
          SP.println("OK");
        }           
        else if (i == 1) {
          SP.print("SMP\tPowering up LINE 2.. ");
          digitalWrite(PW_3V3, LOW);
          digitalWrite(PW_3V3_A, HIGH);
          SP.println("OK");   
        }
        delay(10);
        // INITIALIZE SENSORS
        if ((i == 0 && I1A) || (i == 1 && I2A)) {
          SP.print("SMP\tInitializing Sensor ");
          SP.print(i+1);
          SP.print("A.. ");
          INIT_SMP[i*2] = !inkl_a.initialize(BMA4_I2C_ADDR_SECONDARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG);                              
          if (INIT_SMP[i*2]) SP.println("OK");
          else SP.println("ERROR"); 
        }
        if ((i == 0 && I1B) || (i == 1 && I2B)) {
          SP.print("SMP\tInitializing Sensor ");
          SP.print(i+1);
          SP.print("B.. ");
          INIT_SMP[(i*2)+1] = !inkl_b.initialize(BMA4_I2C_ADDR_PRIMARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG);                    
          if (INIT_SMP[(i*2)+1]) SP.println("OK");
          else SP.println("ERROR"); 
        }
        delay(175);        
        // -- START SMP MEASUREMENTS        
        SP.print("SMP\tMEASURING LINE ");
        SP.print(i+1);
        SP.println("..");
        uint measstart_SMP = millis();                                                 //Reset Timing & Counters
        while (millis() - measstart_SMP <= measlength) {
          uint meastime_SMP = millis() - measstart_SMP;
          if (SET_SMP && (meastime_SMP >= SMP_MINT * MCTR_SMP[i])) {
            MCTR_SMP[i]++;
            if ((i == 0 && I1A && INIT_SMP[0]) || (i == 1 && I2A && INIT_SMP[2])) {
              smp_raw_x = NAN, smp_raw_y = NAN, smp_raw_z = NAN, smp_temp = 0;                                // SET raw variables to NAN
              inkl_a.getAcceleration(&smp_raw_x, &smp_raw_y, &smp_raw_z);
              smp_temp = inkl_a.getTemperature();
              smp_x = (atan(smp_raw_x / sqrt(sq(smp_raw_y) + sq(smp_raw_z))))*RAD2DEG;
              smp_y = (atan(smp_raw_y / sqrt(sq(smp_raw_x) + sq(smp_raw_z))))*RAD2DEG;
              smp_z = (atan(smp_raw_z / sqrt(sq(smp_raw_x) + sq(smp_raw_y))))*RAD2DEG;          
              if (i == 0 && !isnan(smp_x) && !isnan(smp_y) && !isnan(smp_z) && !isnan(smp_temp)) {              // only consider values if all are NOT NaN
                SUM_SMP_X[0] = SUM_SMP_X[0] + smp_x;
                SUM_SMP_Y[0] = SUM_SMP_Y[0] + smp_y;
                SUM_SMP_Z[0] = SUM_SMP_Z[0] + smp_z;
                SUM_SMP_TEMP[0] = SUM_SMP_TEMP[0] + smp_temp;
                VCTR_SMP[0]++;                         
              }
              if (i == 1 && !isnan(smp_x) && !isnan(smp_y) && !isnan(smp_z) && !isnan(smp_temp)) {              // only consider values if all are NOT NaN
                SUM_SMP_X[2] = SUM_SMP_X[2] + smp_x;
                SUM_SMP_Y[2] = SUM_SMP_Y[2] + smp_y;
                SUM_SMP_Z[2] = SUM_SMP_Z[2] + smp_z;
                SUM_SMP_TEMP[2] = SUM_SMP_TEMP[2] + smp_temp;
                VCTR_SMP[2]++;                        
              }
            }              
            if ((i == 0 && I1B && INIT_SMP[1]) || (i == 1 && I2B && INIT_SMP[3])) {
              inkl_b.getAcceleration(&smp_raw_x, &smp_raw_y, &smp_raw_z);
              smp_temp = inkl_b.getTemperature();
              smp_x = (atan(smp_raw_x / sqrt(sq(smp_raw_y) + sq(smp_raw_z))))*RAD2DEG;
              smp_y = (atan(smp_raw_y / sqrt(sq(smp_raw_x) + sq(smp_raw_z))))*RAD2DEG;
              smp_z = (atan(smp_raw_z / sqrt(sq(smp_raw_x) + sq(smp_raw_y))))*RAD2DEG;
              if (i == 0) {
                SUM_SMP_X[1] = SUM_SMP_X[1] + smp_x;
                SUM_SMP_Y[1] = SUM_SMP_Y[1] + smp_y;
                SUM_SMP_Z[1] = SUM_SMP_Z[1] + smp_z;
                SUM_SMP_TEMP[1] = SUM_SMP_TEMP[1] + smp_temp;
                VCTR_SMP[1]++;                                     
              }
              if (i == 1) {
                SUM_SMP_X[3] = SUM_SMP_X[3] + smp_x;
                SUM_SMP_Y[3] = SUM_SMP_Y[3] + smp_y;
                SUM_SMP_Z[3] = SUM_SMP_Z[3] + smp_z;
                SUM_SMP_TEMP[3] = SUM_SMP_TEMP[3] + smp_temp;
                VCTR_SMP[3]++;                         
              }
            }            
          }
        }         
      }
    }      
  }
}

// -- SMP FINALIZE
// Shut down sensors and evaluate data (calculate averages)
void SMP_Finalize () {
  if (SET_SMP) {
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
      AVG_SMP_X[i] = SUM_SMP_X[i] / VCTR_SMP[i];
      AVG_SMP_Y[i] = SUM_SMP_Y[i] / VCTR_SMP[i];
      AVG_SMP_Z[i] = SUM_SMP_Z[i] / VCTR_SMP[i];
      AVG_SMP_TEMP[i] = SUM_SMP_TEMP[i] / VCTR_SMP[i]; 
    }         
    // Output Results
    if (I1A) {
    sprintf(report, "SMP\tI1A Tilt X\t%.2f\t°\t%u\t%u", AVG_SMP_X[0], VCTR_SMP[0], MCTR_SMP[0]);
    SP.println(report);
    sprintf(report, "SMP\tI1A Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMP_Y[0], VCTR_SMP[0], MCTR_SMP[0]);
    SP.println(report);
    sprintf(report, "SMP\tI1A Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMP_Z[0], VCTR_SMP[0], MCTR_SMP[0]);
    SP.println(report);
    sprintf(report, "SMP\tI1A Temp\t%ld\t°C\t%u\t%u", AVG_SMP_TEMP[0], VCTR_SMP[0], MCTR_SMP[0]);
    SP.println(report);         
    }
    if (I1B) {
    sprintf(report, "SMP\tI1B Tilt X\t%.2f\t°\t%u\t%u", AVG_SMP_X[1], VCTR_SMP[1], MCTR_SMP[1]);
    SP.println(report);
    sprintf(report, "SMP\tI1B Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMP_Y[1], VCTR_SMP[1], MCTR_SMP[1]);
    SP.println(report);
    sprintf(report, "SMP\tI1B Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMP_Z[1], VCTR_SMP[1], MCTR_SMP[1]);
    SP.println(report);
    sprintf(report, "SMP\tI1B Temp\t%ld\t°C\t%u\t%u", AVG_SMP_TEMP[1], VCTR_SMP[1], MCTR_SMP[1]);
    SP.println(report);         
    }
    if (I2A) {
    sprintf(report, "SMP\tI2A Tilt X\t%.2f\t°\t%u\t%u", AVG_SMP_X[2], VCTR_SMP[2], MCTR_SMP[2]);
    SP.println(report);
    sprintf(report, "SMP\tI2A Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMP_Y[2], VCTR_SMP[2], MCTR_SMP[2]);
    SP.println(report);
    sprintf(report, "SMP\tI2A Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMP_Z[2], VCTR_SMP[2], MCTR_SMP[2]);
    SP.println(report);
    sprintf(report, "SMP\tI2A Temp\t%ld\t°C\t%u\t%u", AVG_SMP_TEMP[2], VCTR_SMP[2], MCTR_SMP[2]);
    SP.println(report);         
    }
    if (I2B) {
    sprintf(report, "SMP\tI2B Tilt X\t%.2f\t°\t%u\t%u", AVG_SMP_X[3], VCTR_SMP[3], MCTR_SMP[3]);
    SP.println(report);
    sprintf(report, "SMP\tI2B Tilt Y\t%.2f\t°\t%u\t%u", AVG_SMP_Y[3], VCTR_SMP[3], MCTR_SMP[3]);
    SP.println(report);
    sprintf(report, "SMP\tI2B Tilt Z\t%.2f\t°\t%u\t%u", AVG_SMP_Z[3], VCTR_SMP[3], MCTR_SMP[3]);
    SP.println(report);
    sprintf(report, "SMP\tI2B Temp\t%ld\t°C\t%u\t%u", AVG_SMP_TEMP[3], VCTR_SMP[3], MCTR_SMP[3]);
    SP.println(report);         
    }
  }
}