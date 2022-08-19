// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- IMU SENSOR CONFIGURATION -- ICM20948
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- ICM20948 Sensor Library & Dependencies
#include "I2Cdev.h"
I2Cdev I2C_M;
#include "ICM20948_WE.h"


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- IMU MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------


// DEVELOPMENT!!! IMU currently not implemented on Inform@Risk PCB


// ICM20948 Hardware Address
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// -- TRANSMIT DATA
// Transmit Accelerometer/Gyroscope/Magnetometer Data via LoRa?
#define IMU_TRANS_ACC 1
#define IMU_TRANS_GYR 1
#define IMU_TRANS_MAG 1

// -- DATA RETRIEVAL interval
// Interval in [ms] at which data is read Arduino Ain Port (earliest).
int     IMU_MINT = 200;

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- IMU GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Measurement Variables
float imumagx, imumagy, imumagz, imuaccx, imuaccy, imuaccz, imugyrx, imugyry, imugyrz;      // Variables for single measurements
float SUM_IMUMAGX = 0;                                                                      // Magnetometer: SUM of all measurements in one measurement cycle
float SUM_IMUMAGY = 0;
float SUM_IMUMAGZ = 0;
float SUM_IMUACCX = 0;                                                                      // Accelerometer: SUM of all measurements in one measurement cycle
float SUM_IMUACCY = 0;
float SUM_IMUACCZ = 0;
float SUM_IMUGYRX = 0;                                                                      // Gyroscope: SUM of all measurements in one measurement cycle
float SUM_IMUGYRY = 0;
float SUM_IMUGYRZ = 0;
uint   MCTR_IMU = 0;                                                                        // IMU measurement counter (theoretical number of performed measurements)
uint   VCTR_IMU = 0;                                                                        // IMU value counter (number of collected values)
float AVG_IMUMAGX = 0;                                                                      // Magnetometer: SUM of all measurements in one measurement cycle
float AVG_IMUMAGY = 0;
float AVG_IMUMAGZ = 0;
float AVG_IMUACCX = 0;                                                                      // Accelerometer: SUM of all measurements in one measurement cycle
float AVG_IMUACCY = 0;
float AVG_IMUACCZ = 0;
float AVG_IMUGYRX = 0;                                                                      // Gyroscope: SUM of all measurements in one measurement cycle
float AVG_IMUGYRY = 0;
float AVG_IMUGYRZ = 0;


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- IMU FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- IMU INITIALIZE
// Initializes IMU (ICM20948) (if active)
void IMU_Init () {
  SP.print("IMU\t");  
  if (SET_IMU) {
    //Reset Variables    
    SUM_IMUMAGX = 0, SUM_IMUMAGY = 0, SUM_IMUMAGZ = 0;
    SUM_IMUACCX = 0, SUM_IMUACCY = 0, SUM_IMUACCZ = 0;
    SUM_IMUGYRX = 0, SUM_IMUGYRY = 0, SUM_IMUGYRZ = 0;
    MCTR_IMU = 0, VCTR_IMU = 0;
    AVG_IMUMAGX = 0, AVG_IMUMAGY = 0, AVG_IMUMAGZ = 0;
    AVG_IMUACCX = 0, AVG_IMUACCY = 0, AVG_IMUACCZ = 0;
    AVG_IMUGYRX = 0, AVG_IMUGYRY = 0, AVG_IMUGYRZ = 0;
    //Initialize Hardware IMU (Accelerometer & Gyroscope)    
    SP.print("Initializing IMU (Accelerometer & Gyroscope).. ");
    if(myIMU.init()) {
      SP.println("OK");
      }
    else {SP.println("ERROR! Could not be initialized!");}
    //Initialize Hardware IMU (Magnetometer)
    SP.print("IMU\t");    
    SP.print("Initializing IMU (Magnetometer).. ");
    if(myIMU.initMagnetometer()) {
      SP.println("OK");
      }
    else {SP.println("ERROR! Could not be initialized!");}                                         
  }
  else SP.println("OFF");
  // DEV: SET SENSOR TO SLEEP MODE??
}

// -- IMU SETUP
// Setup IMU (ICM20948) measurements (if active)
void IMU_Setup () {
  if (SET_IMU) {
    //Setup IMU measurements
    SP.print("IMU\tSetting Up IMU measurements.. ");    
    myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
    myIMU.setAccDLPF(ICM20948_DLPF_6);
    myIMU.setAccSampleRateDivider(10);
    myIMU.setGyrDLPF(ICM20948_DLPF_6);
    myIMU.setGyrSampleRateDivider(10);
    myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
    SP.println("OK");
  }    
}

// -- IMU MEASUREMENT
// Make single IMU (ICM20948) measurement (if active)
void IMU_Measure () {
  if (SET_IMU) {
    MCTR_IMU++;
    //DEVELOPMENT: PERFORM IMU MEASUREMENTS    
    SP.println("IMU\tWARNING: IMU measurements not implemented!");
    SUM_IMUMAGX = SUM_IMUMAGX + imumagx;
    SUM_IMUMAGY = SUM_IMUMAGY + imumagy;
    SUM_IMUMAGZ = SUM_IMUMAGZ + imumagz;
    SUM_IMUACCX = SUM_IMUACCX + imuaccx;
    SUM_IMUACCY = SUM_IMUACCY + imuaccy;
    SUM_IMUACCZ = SUM_IMUACCZ + imuaccz;
    SUM_IMUGYRX = SUM_IMUGYRX + imugyrx;
    SUM_IMUGYRY = SUM_IMUGYRY + imugyry;
    SUM_IMUGYRZ = SUM_IMUGYRZ + imugyrz;        
    //IF MEASUREMENT SUCCESSFUL (Values added to SUMs)
    VCTR_IMU++;
  }        
}

// -- IMU FINALIZE
// Make single IMU (ICM20948) measurement (if active)
void IMU_Finalize () {
  if (SET_IMU) {
    //DEVELOPMENT: SET Sensor to sleep mode
    //Calculate and output Averages
    AVG_IMUMAGX = SUM_IMUMAGX / VCTR_IMU;
    AVG_IMUMAGY = SUM_IMUMAGY / VCTR_IMU;
    AVG_IMUMAGZ = SUM_IMUMAGZ / VCTR_IMU;
    AVG_IMUACCX = SUM_IMUACCX / VCTR_IMU;
    AVG_IMUACCY = SUM_IMUACCY / VCTR_IMU;
    AVG_IMUACCZ = SUM_IMUACCZ / VCTR_IMU;
    AVG_IMUGYRX = SUM_IMUGYRX / VCTR_IMU;
    AVG_IMUGYRY = SUM_IMUGYRY / VCTR_IMU;
    AVG_IMUGYRZ = SUM_IMUGYRZ / VCTR_IMU;
    // Output Results
    sprintf(report, "IMU\tMagnetometer X\t%.2f\t?\t%d\t%d", AVG_IMUMAGX, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tMagnetometer Y\t%.2f\t?\t%d\t%d", AVG_IMUMAGY, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tMagnetometer Z\t%.2f\t?\t%d\t%d", AVG_IMUMAGZ, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tAccelerometer X\t%.2f\t?\t%d\t%d", AVG_IMUACCX, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tAccelerometer Y\t%.2f\t?\t%d\t%d", AVG_IMUACCY, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tAccelerometer Z\t%.2f\t?\t%d\t%d", AVG_IMUACCZ, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tGyroscope X\t%.2f\t?\t%d\t%d", AVG_IMUGYRX, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tGyroscope Y\t%.2f\t?\t%d\t%d", AVG_IMUGYRY, VCTR_BARO, MCTR_BARO);
    SP.println(report);
    sprintf(report, "IMU\tGyroscope Z\t%.2f\t?\t%d\t%d", AVG_IMUGYRZ, VCTR_BARO, MCTR_BARO);
    SP.println(report);
  }
}  

    