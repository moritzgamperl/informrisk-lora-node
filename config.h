// -------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINE TIMING
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// LoRa Main Loop Timing; main loop is cycled every 100 ms --> loopintv should be dividable by 100 ms
int loopintv = 60000;        // loop interval (ms)
int sensorstarttime = 500;   // time to wait after sensors are powered up

int extrafreq = 10;          // Frequency of extra payload being sent instead of base payload: 10 means that this is sent every 10th time.

//Measurement Timing; measurement loop is cycled every 10 ms --> all time intervals should be dividable by 10 ms
int measlength = 3000;    // measurement duration (ms) for all sensors

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SENSOR STATE
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// DESIGNATOR defining Payload type and Sensor type:
// 1: LoRa Infrastructure Node (LIN)
// 2: Subsurface Node (SMN or LCI)
int desig = 2;

// Sensor settings: 0 to deactivate Sensor in File, 1 to activate;

int SET_IMU = 1; 
int SET_ADC = 0;
int SET_SCL = 1;

// Turn additional functions on/off (eg Bodensonde: SET_SMN: "Subsurface Measurement Node")
int SET_SMN = 0;



// -------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINE HARDWARE
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// Set DPORT to -1 (or any other negative value) to turn off device

// OnBoard LED
int LED_DPORT = -1;       // Digital port of OnBoard LED; set to "LED_BUILTIN" to activate, set to "-1" to deactivate; LED will light up during measurement

// Battery voltage measurement using voltage divider (BATT in; up to XXX Volts; defined by voltage divider resistor values)
int BATT_DPORT = 0;       // Digital port to turn on/off voltage divider; set to -1 to deactivate 
int BATT_APORTin = A6;    // AnalogIn port used for batt voltage measurement
int BATT_R1 = 33000;      // Voltage divider resistor 1 in Ohms (used for voltage measurement)
int BATT_R2 = 100000;     // Voltage divider resistor 2 in Ohms
int BATT_MINT = 500;     // measurement interval of the BATT Voltage (ms)
int BATT_MCTR =  1;       // BATT measurement counter -- do not change

// Switched 12V output
int SW12_DPORT = 1;       // Digital port to control 12V output; set to -1 to deactivate

// Switched 3,3V output (turned on during measurements)
int SW33_A_DPORT = -1;     // Digital port to control 3,3V output A; set to -1 to deactivate
int SW33_B_DPORT = -1;     // Digital port to control 3,3V output B; set to -1 to deactivate

// Grove 10DOF_V2
int Grove10DOF_DPORT = 3;  // Digital port to turn on/off Grove IMU 10DOF;  set to -1 to deactivate
// Measurement interval and counter are same as for Pololu
int IMU_MINT = 100;       // measurement interval of the IMU (Accelerometer & Gyro) (ms) -- make sure is supported by device; e.g. 100 = 10 Hz
int MAG_MINT = 100;      // measurement interval of the Magnetometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz 
int BARO_MINT = 100;     // measurement interval of the Barometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz
int IMU_MCTR = 1;         // IMU measurement counter -- do not change
int MAG_MCTR = 1;         // MAG measurement counter -- do not change
int BARO_MCTR = 1;        // BARO measurement counter -- do not change


// Grove Step Counter
int StpCtr1_DPORT = 1;  // Digital port to turn on/off Step Counter;  set to -1 to deactivate
//int StpCtr2_DPORT = 4;  // Digital port to turn on/off Step Counter;  set to -1 to deactivate
//int StpCtr3_DPORT = 5;  // Digital port to turn on/off Step Counter;  set to -1 to deactivate
int SMN_MCTR = 1;
int SMN_MINT = 10;

// Buoyancy sensor
int Wtr_DPORT = 2; // Digital port to turn on/off buoyancy measurement
int Wtr_APORT = A1; // Digital port of water measurements

int ADC_MINT = 100;      // measurement interval of the ADC (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz


// Olimex ADS1220
#define PGA          128               // Programmable Gain = 1
#define VREF         3.3               // Internal reference of 2.048V
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1)
#define ADS1220_CS_PIN    2
#define ADS1220_DRDY_PIN  3
int ADC_MCTR = 1;
#define SENSOR_POWERUP    4


//Setup SCL3300
//#define SCL3300_Power_PIN 4
#define SCL3300_CS_PIN    7
int scl3300_sspin = SCL3300_CS_PIN;
int scl3300_mode = 4;
int INKL_MINT = 100;
int INKL_MCTR = 1;
