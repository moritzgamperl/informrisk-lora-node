// -------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINE TIMING
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// LoRa Main Loop Timing; main loop is cycled every 100 ms --> loopintv should be dividable by 100 ms
int loopintv = 300000;        // loop interval (ms)
int sensorstarttime = 500;   // time to wait after sensors are powered up

int extrafreq = 100000;          // Frequency of extra payload being sent instead of base payload: 10 means that this is sent every 10th time.

//Measurement Timing; measurement loop is cycled every 10 ms --> all time intervals should be dividable by 10 ms
int measlength = 5000;    // measurement duration (ms) for all sensors

//Development Messages
#define SP Serial         // Define Serial port to output Serial messages to          

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SENSOR STATE
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// DESIGNATOR defining Payload type and Sensor type:
// 1: LoRa Infrastructure Node (LIN)
// 2: Subsurface Node (SMN or LCI)
int desig = 1;

int settings_desig = 00001;

// Sensor settings: 0 to deactivate Sensor in File, 1 to activate;

bool SET_IMU = true; 
bool SET_ADC = false;
bool SET_SCL = false;

// Turn additional functions on/off (eg Bodensonde: SET_SMN: "Subsurface Measurement Node")
int SET_SMN = true;



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
int BATT_MINT = 500;      // measurement interval of the BATT Voltage (ms)
int BATT_MCTR =  1;       // BATT measurement counter -- do not change


// Switched 12V output
int SW12_DPORT = 6;       // Digital port to control 12V output; set to -1 to deactivate

// Switched 3,3V output (turned on during measurements)
int SW33_A_DPORT = 2;     // Digital port to control 3,3V output A; set to -1 to deactivate
int SW33_B_DPORT = -1;     // Digital port to control 3,3V output B; set to -1 to deactivate

// ICM20948
int IMU_DPORT = 3;  // Digital port to turn on/off Grove IMU 10DOF;  set to -1 to deactivate
// Measurement interval and counter are same as for Pololu
int IMU_MINT = 100;       // measurement interval of the IMU (Accelerometer & Gyro) (ms) -- make sure is supported by device; e.g. 100 = 10 Hz
int MAG_MINT = 100;      // measurement interval of the Magnetometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz 
int BARO_MINT = 100;     // measurement interval of the Barometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz
int IMU_MCTR = 1;         // IMU measurement counter -- do not change
int MAG_MCTR = 1;         // MAG measurement counter -- do not change
int BARO_MCTR = 1;        // BARO measurement counter -- do not change


// Grove Step Counter
int I1_PORT = A0;  // Digital port to turn on/off Step Counter 1;  set to -1 to deactivate
int I2_PORT = -1;
bool I1A = true;
bool I2A = false;
bool I1B = false;
bool I2B = false;

int SMN_MCTR = 1;
int SMN_MINT = 10;

// Buoyancy sensor
int Wtr_DPORT = -1; // Digital port to turn on/off buoyancy measurement
int Wtr_APORT = A1; // Digital port of water measurements

int ADC_MINT = 100;      // measurement interval of the ADC (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz
int ADC_MCTR = 1; 
int ADC_CAL = 0;

// Olimex ADS1220
#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4

// SET ADS1220 channels; 0 = off; 1 = on
int ADS_C0 = 1;
int ADS_C1 = 1;
int ADS_C2 = 0;
int ADS_C3 = 0;
bool ads1220debug = true;

//Setup SCL3300
//#define SCL3300_Power_PIN 4
#define SCL3300_CS_PIN -1
int scl3300_sspin = SCL3300_CS_PIN;
int scl3300_mode = 4;
int INKL_MINT = 100;
int INKL_MCTR = 1;
