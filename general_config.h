// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- GENERAL CONFIGURATION
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// EDITABLE SETTINGS -- Basic setup of I@R measurement node
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// Define Serial Port for Staus output
#define SP Serial                                                      //Serial: USB Serial Port; Serial1: TTL Serial Port on Inform@Risk PCB (activate SET_SER!)
bool SET_SER1 = 0;                                                     //Turns On/Off the Serial1 port.

// Define active sensors                                               // set all of following to "true" if used/active and set to "false" if not used/active                                      
bool SET_BATV =   1;                                                   // ON-Board Battery Input Voltage Measurement        --> See BATV_Config.h for detailed settings
bool SET_BARO =   1;                                                   // ON-Board Barometer and Thermometer (BMP388)       --> See BARO_Config.h for detailed settings
bool SET_IMU  =   0;                                                   // ON-Board Inertial Measurement Unit IMU (ICM20948) --> See IMU_config.h for detailed settings    --- CURRENTLY NOT AVAILIABLE!! (Not implemented on PCB) --- 
bool SET_AD24 =   0;                                                   // ON-Board ADC 24bit (ADS1200)                      --> See AD24_config.h for detailed settings
bool SET_AD12 =   0;                                                   // ON-Board ADC 12bit (Arduino)                      --> See AD12_config.h for detailed settings
bool SET_INCL =   1;                                                   // ON-Board High Accuracy inclination sensor (Murata SCL3300) (optional) --> See INCL_config.h for detailed settings
bool SET_SMN  =   0;                                                   // EXTERNAL Subsurface Probes OR Low Cost Inclinometer (ONLY inclination sensors) --> See SMN_config.h for detailed settings

// Define active power ports during main measurement cycle             // Define which power ports should be active during measurements // This does not apply to SMN measurements
bool SET_V12     =  0;                                                 // SW12V
bool SET_SW3V3   =  0;                                                 // First (Standard) SW3V3
bool SET_SW3V3A  =  0;                                                 // Second SW3V3, As defined below:
int  PW_3V3_A    =  AIN0_12;                                           // Possible Values: AIN0_12 (AIn0 12bit), AIN1_12 (AIn1 12bit), CS_INKL (Port CP of INKL connector, only if INKL not active/connected) 
bool SET_SW3V3B  =  0;                                                 // Third SW3V3, As defined below:
int  PW_3V3_B    =  AIN1_12;                                           // Possible Values: AIN0_12 (AIn0 12bit), AIN1_12 (AIn1 12bit), CS_INKL (Port CP of INKL connector, only if INKL not active/connected)

// Setup LORA(R) Connection
#define LORA_BAND    US915                                             //LoRa Band: EU868 - Europe; US915 - USA/Colombia; AS923 - Australia;                                                             For I@R Colombia: US915
#define LORA_ADR     1                                                 //LoRa Automatic Data Rate: true - Data rate is controlled by LoRa Server; false - Data rate is set to a fixed value;             For I@R Colombia: 1
#define LORA_DR      5                                                 //LoRa Data Rate: 1 ... 10; Valdid values depend on selected LoRa Band; See LoRaWAN Regional Parameters for Details               For I@R Colombia: 5
#define LORA_CH_DEF  0                                                 //LoRa Channel Plan: true: use default channel plan for selected Band; false: use channel plan as defined in LORA_CH_ACTIVE       For I@R Colombia: 0
#define LORA_CH_ACT  {8,9,10,11,12,13,14,15,64}                        //LoRa Active Channels: {1,2,3,4,5,6,7,8} Custom list of channels to be used; See LoRaWAN Regional Parameters for Details;        For I@R Colombia: {8,9,10,11,12,13,14,15,64}
#define LORA_TIMEOUT 120                                               //LoRa Connection Timeout in Seconds
String appEui = SECRET_APP_EUI;                                        //LoRa Secret APP EUI: DO NOT CHANGE! Sensitive data is defined in arduino_secrets.h.
String appKey = SECRET_APP_KEY;                                        //LoRa Secret APP KEY: DO NOT CHANGE! Sensitive data is defined in arduino_secrets.h.

// Timing                                                              // IMPORTANT! Main loop is cycled every 10 ms --> all of the following values should be dividable by 10 ms!
uint loopintv = 1800000;                                               // Measurement loop interval (ms) -- Defines measurement interval.
uint measlength = 5000;                                                // measurement duration (ms) for all sensors -- Defines how long sensors should be active. A Median or Average value is generated from the acquired data in this timespan
uint sensorstarttime = 500;                                            // time to wait after sensors are powered up

// Sleep mode                                                          // defines mode of operation in between measurements
#define SleepMode "deepsleep"                                          // "deepsleep": most hardware is disabled (lowest power consumption, recommended for deployment); "sleep": most hw is disabled, but COM is active; "": all hardware stays active.

// Data packet designator
const uint16_t PACKET_DESIGNATOR = 10;                                 // DESIGNATOR defining Payload type -- DO NOT CHANGE!
