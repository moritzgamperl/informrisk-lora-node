/*  Inform@Risk LoRa Node
    (c) AlpGeorisk & TUM
    Authors: Moritz Gamperl & John Singer

    LoRaBasis_InformRisk
    Version: 0.3
    Date: 04.02.2021

    Changelog from version 0.2:
    - Check functionality on breadboard
    - implement ADC: different modes
    - Implemented designator for Measurement types
    - Implemented Control for frequency of extra data (Variable name?)
    - Moved time controls to config file

    Supported Hardware:
    BOARDS:
    - Arduino MKR WAN 1310

    SENSORS:
    - Waveshare - IMU 10DOF (MPU9250, BMP280)
    - Watertable measurement with swimming sensor
    - Watertable measurement with Float switch
    
    PERIFERALS:
    - Inclination Sensor BMA


*/


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// HEADER
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// include Libraries, Files

// for addional Arduino functions
#include <MKRWAN.h>           //  Library: MKRWAN from Arduino - Library for Arduino MKR WAN boards
LoRaModem modem;
#include <ArduinoLowPower.h>  //  Library: Arduino Low Power - Library for Low Power mode (requires RTCZero library)


// For Waveshare 10DOF
#include <MPU9255.h>
MPU9255 mpu;

// for Grove IMU 10DOF
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP280.h"

MPU9250 accelgyro;
I2Cdev I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

BMP280 bmp280;

// For Step Counter
#include "arduino_bma456.h";

// For Olimex ADS1220
#include "AlpGeorisk_ADS1220.h"
#include <SPI.h>
AlpGeorisk_ADS1220 ads1220;
int32_t adc_data;


// For SCL3300
#include "SPI.h"
#include "SCL3300.h"
SCL3300 inclinometer;


// Need the following define for SAMD processors
#if defined (ARDUINO_ARCH_SAMD)
//#define Serial1 Serial1USB
#endif

// Setup File: Read from Flash!/Write to Flash

// Files
#include "arduino_secrets.h"
#include "config.h"
#include "array.h"


// VARIABLES (Do not change standard values)

// Security
String appEui = SECRET_APP_EUI;    // Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appKey = SECRET_APP_KEY;    // Please enter your sensitive data in the Secret tab or arduino_secrets.h
//Timing Variables (all are unsigned long, so overflow of millis() will also cause an overflow of the calculation of the according relative time --> result is OK)
unsigned long loopstart = 0;       // starttime of loop (ms)
unsigned long looptime = 0;        // relative time within current loop (ms)
unsigned long measstart = 0;       // stattime of measurement (ms)
unsigned long meastime = 0;        // relative time within current measurement (ms)
unsigned long sleeptime = 0;       // time arduino will sleep until next measurement
unsigned long loopctr = 0;         // counting the number of loops for extra payload

//Measurement Variables
// muss alles float sein
long meassum[22];         // Array with summerized measurements // Achtung! Länge = max i + 1, da indizierung mit 0 losgeht
long arr_imu_a_x[100];          // Array with measurements for imu.a.x Qestion: How to deal with max size of Array? Here, 1000 as default
long arr_imu_a_y[100];
long arr_imu_a_z[100];
long arr_imu_g_x[100];
long arr_imu_g_y[100];
long arr_imu_g_z[100];
long arr_mag_m_x[100];
long arr_mag_m_y[100];
long arr_mag_m_z[100];
char report[170];         // Measurement report for Serial1 output
char report2[170];        // SMN Measurement report for Serial1 output

//SMN Measurement Variables
float smn_x = 0, smn_y = 0, smn_z = 0;
int32_t smn_temp = 0;

int16_t watertbl = 0;
float watertbl_cal;





// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {




  // OPEN Serial1 PORT -- used for development purposes
  Serial1.begin(115200);
  // Wait 5 secs for Serial1 connection otherwise continue anyway...
  while (!Serial1 && millis() < 5000);
  Wire.begin();
  // For Waveshare IMU
  //  TWBR = 24;


  // SET PINS TO -1 IF DEACTIVATED; ADC comes later
  if (SET_IMU == 0) Grove10DOF_DPORT = -1;
  //if (SET_ADC = 0) {
  if (SET_SMN == 0) StpCtr1_DPORT = -1;
  

  // SETUP DIGITAL PINS
  // initialize digital pins
  // Set measurement pins to output, set unused pins to input_pullup
  Serial1.println("Digital ports setup:");
  for (int i = 0; i <= 7; i++) {
    Serial1.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == Grove10DOF_DPORT || i == StpCtr1_DPORT) {
      pinMode(i, OUTPUT);
      Serial1.println(": Output");
    }
    else {
      // SET LED port to output low, all other unused ports to INPUT_PULLUP to save energy
      if (i == LED_BUILTIN) {
        pinMode(i, INPUT);
        Serial1.println(": Input (LED Off)");
      }
      else {
        pinMode(i, INPUT_PULLUP);
        Serial1.println(": INPUT_PULLUP");
      }
    }
  }


  // SETUP ANALOG INPUT
  analogReadResolution(12);

  // ACTIVATE SENSORS
  if (BATT_DPORT >= 0) digitalWrite(BATT_DPORT, HIGH);
  if (LED_DPORT >= 0) digitalWrite(LED_DPORT, HIGH);
  if (SW33_A_DPORT >= 0) digitalWrite(SW33_A_DPORT, HIGH);
  if (SW33_B_DPORT >= 0) digitalWrite(SW33_B_DPORT, HIGH);
  if (SW12_DPORT >= 0) digitalWrite(SW12_DPORT, HIGH);
  if (Grove10DOF_DPORT >= 0) digitalWrite(Grove10DOF_DPORT, HIGH);

  // ACTIVATE SMN SENSORS
  if (StpCtr1_DPORT >= 0) digitalWrite(StpCtr1_DPORT, HIGH);

      


    // ACTIVATE ADC
    if (SET_ADC == 1) {
    //Initialize ADS1220
    ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN);
    ads1220.select_mux_channels(MUX_AIN1_AIN2);
    ads1220.PGA_ON();
    ads1220.set_pga_gain(PGA_GAIN_128);
    ads1220.set_data_rate(DR_20SPS);
    ads1220.set_op_mode(OPMODE_DUTY_CYCLE);
    ads1220.set_vref(VREF_REFP1_REFN1);
    ads1220.set_fir_filter(FIR_filter_50_60);
    ads1220.set_PWS_AUTO();
    ads1220.set_conv_mode_single_shot();
    }

  delay(sensorstarttime); // wait for sensors to startup

    if (SET_SCL == 1) {
// Setup PINS MURATASC3300
// pinMode(SCL3300_Power_PIN, OUTPUT);
   pinMode(SCL3300_CS_PIN, OUTPUT);
    }

     
  // CONNECT TO LORA NETWORK
  // change this to your regional band (eg. US915, AS923, ...) For Colombia, use "US915"! For Europe, use "EU868".
  if (!modem.begin(EU868))
  {
    Serial1.println("Failed to start module");
    while (1) {}
  };
  String ArdVers = modem.version();
  String DevEUI = modem.deviceEUI();
  Serial1.print("Your module version is: ");
  Serial1.println(ArdVers);
  Serial1.print("Your device EUI is: ");
  Serial1.println(DevEUI);

  // attemt to join LoRaWAN network using Over-The-Air-Activation (OTAA)
  int connected = modem.joinOTAA(appEui, appKey);
delay(100);
  if (connected)
  {
    // ----- Setup Signal strength ------
    //      change SF by choosing data rate (bool dataRate(uint8_t dr)), can be between 0 (SF 12) and 6 (SF 7)
    //      ADR=Adaptive Data Rate allows modulation of SW and BW depending on signal strength from gateway. Is on by default
    //      https://github.com/arduino/mkrwan1300-fw/issues/3
    //try: modem.setADR(true);
    //modem.dataRate(5);

    // Set poll interval to x seconds
    modem.minPollInterval(10);
    delay(100);                    // because ... more stable
  }
  else
  {
    Serial1.println("Something went wrong; are you indoors? Move near a window and retry");
    while (1) {}
  }


   

  Serial1.println("connected");

    // See datasheet
    //bmp280.setSampling(BMP280::MODE_NORMAL,  // mode
    //              BMP280::SAMPLING_X2,  // temperature, not more than x2
    //              BMP280::SAMPLING_X16, // pressure                  
    //              BMP280::FILTER_X16,   // filter
    //              BMP280::STANDBY_MS_500); // standby

//   bmp_temp->printSensorDetails(); 

}

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {

  // PAYLOADS
short payload1size = 35;
byte payload1[payload1size];

short payload2size = 35;
byte payload2[payload2size];

short extrapayload1size = 35;
byte extrapayload1[extrapayload1size];

short extrapayload2size = 35;
byte extrapayload2[extrapayload2size];
  

  loopctr++;

  //MEASUREMENT LOOP
  loopstart = millis();
  Serial1.println();
  Serial1.print("New Measurement, Starttime: ");
  Serial1.println(loopstart);
  //Wire.begin();

  // ACTIVATE SENSORS
  if (BATT_DPORT >= 0) digitalWrite(BATT_DPORT, HIGH);
  if (LED_DPORT >= 0) digitalWrite(LED_DPORT, HIGH);
  if (SW33_A_DPORT >= 0) digitalWrite(SW33_A_DPORT, HIGH);
  if (SW33_B_DPORT >= 0) digitalWrite(SW33_B_DPORT, HIGH);
  if (SW12_DPORT >= 0) digitalWrite(SW12_DPORT, HIGH);
  if (Grove10DOF_DPORT >= 0) digitalWrite(Grove10DOF_DPORT, HIGH);
  if (StpCtr1_DPORT >= 0) digitalWrite(StpCtr1_DPORT, HIGH);
  

  delay(sensorstarttime); // wait for sensors to startup
  Serial1.println("BREAK: sensors should be started");

  // -------------------------------------------------------------------------------------------------------------------------------------------------------
  //INITIALIZE SENSORS AFTER POWERUP

 // IMU INITIALIZE
     if (Grove10DOF_DPORT >= 0) {
      Serial1.println("Initializing 10DOF");
    accelgyro.initialize();
    bmp280.init();
    /*if (accelgyro.testConnection()){}
    else
    {
      Serial1.println("Failed to detect and initialize Grove IMU!");
      //while (1);
    }*/
     }


 // SMN INITIALIZE
if (SET_SMN == 1)
  {
    bma456.initialize();
  }

if (SET_ADC == 1){
    ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN);
    ads1220.select_mux_channels(MUX_AIN1_AIN2);
    ads1220.PGA_ON();
    ads1220.set_pga_gain(PGA_GAIN_128);
    ads1220.set_data_rate(DR_20SPS);
    ads1220.set_op_mode(OPMODE_DUTY_CYCLE);
    ads1220.set_vref(VREF_REFP1_REFN1);
    ads1220.set_fir_filter(FIR_filter_50_60);
    ads1220.set_PWS_AUTO();
    ads1220.set_conv_mode_single_shot();
    
    pinMode(SENSOR_POWERUP, OUTPUT);
    digitalWrite(SENSOR_POWERUP, HIGH);
}

if (SET_SCL == 1) {
// Turn on Murata
      inclinometer.WakeMeUp();
//    digitalWrite(SCL3300_Power_PIN, HIGH);
      digitalWrite(SCL3300_CS_PIN, LOW);
      delay(10);

  // SETUP MURATA
    if (inclinometer.begin(scl3300_sspin) == false) {
      Serial1.println("Murata SCL3300 inclinometer not connected.");
      while(1); //Freeze
    }
    inclinometer.setMode(scl3300_mode);
}


  // -------------------------------------------------------------------------------------------------------------------------------------------------------
  //MEASUREMENTS
  //Reset Timing & Counters
  measstart = millis();
  BATT_MCTR = 1;
  IMU_MCTR = 1;
  MAG_MCTR = 1;
  BARO_MCTR = 1;
  SMN_MCTR = 1;
  INKL_MCTR = 1;
  Serial1.println("BREAK: Measurement started, counters set to 1");


  //Reset Measurent Variables
  for (int i = 0; i <= 21; i++)
  {
    meassum[i] = 0;
  }



  

  while (millis() - measstart <= measlength)
  {
    meastime = millis() - measstart;

    // BATT VOLTAGE MEASUREMENT
    if (BATT_DPORT >= 0) {
      if (meastime >= BATT_MINT * BATT_MCTR) {
        BATT_MCTR++;
        // Read Analog Port
        int batt = analogRead(BATT_APORTin);
        meassum[0] = meassum[0] + batt;
        Serial1.print("BREAK: Battery voltage finished:");
        Serial1.println(batt);
      }
    }

    // IMU MEASUREMENT
        if (Grove10DOF_DPORT >= 0) {
      if (meastime >= IMU_MINT * IMU_MCTR) {
        IMU_MCTR++;
        // ----- Get Grove 10DOF data -----
        accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        meassum[1] = meassum[1] + ax;
        meassum[2] = meassum[2] + ay;
        meassum[3] = meassum[3] + az;
        meassum[4] = meassum[4] + gx;
        meassum[5] = meassum[5] + gy;
        meassum[6] = meassum[6] + gz;
        meassum[7] = meassum[7] + mx;
        meassum[8] = meassum[8] + my;
        meassum[9] = meassum[9] + mz;
        // Median:
        arr_imu_a_x[IMU_MCTR - 1] = ax;
        arr_imu_a_y[IMU_MCTR - 1] = ay;
        arr_imu_a_z[IMU_MCTR - 1] = az;
        arr_imu_g_x[IMU_MCTR - 1] = gx;
        arr_imu_g_y[IMU_MCTR - 1] = gy;
        arr_imu_g_z[IMU_MCTR - 1] = gz;
        arr_mag_m_x[MAG_MCTR - 1] = mx;
        arr_mag_m_y[MAG_MCTR - 1] = my;
        arr_mag_m_z[MAG_MCTR - 1] = mz;
      }

      // BARO MEASUREMENT
      if (meastime >= BARO_MINT * BARO_MCTR) {

        BARO_MCTR++;
        // From bmp280:
        meassum[10] = meassum[10] + round(bmp280.getTemperature() * 10);
        meassum[11] = meassum[11] + round(bmp280.getPressure() / 100); // Pa div. 100 equ. hPa = mBar

      }
    }

    

    // ADC MEASUREMENT

      if (SET_ADC == 1)
      {
        if (meastime >= ADC_MINT * ADC_MCTR) 
        {
          ADC_MCTR++;
          adc_data=ads1220.Read_SingleShot_WaitForData();
          float Vout = (float)((adc_data*VFSR*1000)/FULL_SCALE);     //In  mV
     meassum[12] = meassum[12] + round(Vout*1000);
    Serial1.println("ADC: ");
    Serial1.println(Vout);
    delay(10);
      }
      }

      // SCL inclinometer measurement
    if (SET_SCL == 1){
     if (meastime >= INKL_MINT * INKL_MCTR) {
      INKL_MCTR++;
      if (inclinometer.available()) {
        float incl_raw_x = (inclinometer.getTiltLevelOffsetAngleX());
        float incl_raw_y = (inclinometer.getTiltLevelOffsetAngleY());
        float incl_raw_z = (inclinometer.getTiltLevelOffsetAngleZ());
        meassum[13] = meassum[13] + round(incl_raw_x*1000);
        meassum[14] = meassum[14] + round(incl_raw_y*1000);
        meassum[15] = meassum[15] + round(incl_raw_z*1000);
      } 
      else inclinometer.reset();
      
    }
    }

      // SMN MEASUREMENT (for only one SMN)

      if (SET_SMN == 1) {                    
         if (meastime >= SMN_MINT * SMN_MCTR) {
          SMN_MCTR++;
        bma456.getAcceleration(&smn_x, &smn_y, &smn_z);
        smn_temp = bma456.getTemperature();

        meassum[16] = meassum[16] + smn_x;
        meassum[17] = meassum[17] + smn_y;
        meassum[18] = meassum[18] + smn_z;
        meassum[19] = meassum[19] + smn_temp;
      }
      }
    
    // DELAY after every measurement cycle
    delay(50);
  }

Serial1.println("MEASUREMENT END");
  // -------------------------------------------------------------------------------------------------------------------------------------------------------
  // MEASUREMENT END
  // DEACTIVATE SENSORS
  if (BATT_DPORT >= 0) digitalWrite(BATT_DPORT, LOW);
  if (LED_DPORT >= 0) digitalWrite(LED_DPORT, LOW);
  if (SW33_A_DPORT >= 0) digitalWrite(SW33_A_DPORT, LOW);
  if (SW33_B_DPORT >= 0) digitalWrite(SW33_B_DPORT, LOW);
  if (SW12_DPORT >= 0) digitalWrite(SW12_DPORT, LOW);
  if (SET_ADC == 1) ads1220.powerdown();

  if (SET_SMN == 1){
// Turn off Murata
//    digitalWrite(SCL3300_Power_PIN, LOW);
      digitalWrite(SCL3300_CS_PIN, HIGH);
      inclinometer.powerDownMode();
  }

    pinMode(SENSOR_POWERUP, INPUT);
    digitalWrite(SENSOR_POWERUP, LOW);
    
   if (SET_SMN == 1) 
  {
    if (StpCtr1_DPORT >= 0) digitalWrite(StpCtr1_DPORT, LOW); 
  }


  //Display measurement duration
  Serial1.print("Measurement duration inkl. initialization (sensors on): ");
  Serial1.println(millis() - loopstart);

  //Calculate averages
  int16_t battv = round(meassum[0] / (BATT_MCTR - 1));
  Serial1.println("BATTV: ");
  Serial1.println(battv);
  int16_t imuax = round(meassum[1] / (IMU_MCTR - 1));
  int16_t imuay = round(meassum[2] / (IMU_MCTR - 1));
  int16_t imuaz = round(meassum[3] / (IMU_MCTR - 1));
  int16_t imugx = round(meassum[4] / (IMU_MCTR - 1));
  int16_t imugy = round(meassum[5] / (IMU_MCTR - 1));
  int16_t imugz = round(meassum[6] / (IMU_MCTR - 1));
  int16_t magmx = round(meassum[7] / (IMU_MCTR - 1));
  int16_t magmy = round(meassum[8] / (IMU_MCTR - 1));
  Serial1.println(magmx);
  int16_t magmz = round(meassum[9] / (IMU_MCTR - 1));
  int16_t temp = round(meassum[10] / (BARO_MCTR - 1));
  int16_t prsr = round(meassum[11] / (BARO_MCTR - 1));
  int16_t adc = round(meassum[12] / (ADC_MCTR - 1));
  int16_t inkl_x = round(meassum[13] / (INKL_MCTR - 1));
  int16_t inkl_y = round(meassum[14] / (INKL_MCTR - 1));
  int16_t inkl_z = round(meassum[15] / (INKL_MCTR - 1));
  int16_t smn1_x = round(meassum[16] / (SMN_MCTR - 1));
  int16_t smn1_y = round(meassum[17] / (SMN_MCTR - 1));
  int16_t smn1_z = round(meassum[18] / (SMN_MCTR - 1));
  int16_t smn1_temp = round(meassum[19] / (SMN_MCTR - 1));
  
  //Calculate medians

  int16_t med_imu_a_x = int(Array_median(arr_imu_a_x, IMU_MCTR));
  int16_t med_imu_a_y = int(Array_median(arr_imu_a_y, IMU_MCTR));
  int16_t med_imu_a_z = int(Array_median(arr_imu_a_z, IMU_MCTR));
  int16_t med_imu_g_x = int(Array_median(arr_imu_g_x, IMU_MCTR));
  int16_t med_imu_g_y = int(Array_median(arr_imu_g_y, IMU_MCTR));
  int16_t med_imu_g_z = int(Array_median(arr_imu_g_z, IMU_MCTR));
  int16_t med_imu_m_x = int(Array_median(arr_mag_m_x, IMU_MCTR));
  int16_t med_imu_m_y = int(Array_median(arr_mag_m_y, IMU_MCTR));
  int16_t med_imu_m_z = int(Array_median(arr_mag_m_z, IMU_MCTR));

  Serial1.println("BREAK: Medians, Averages calculated");
  
  //Calculate battvolt
  int8_t battery = 10 * battv * 3.3 / 4095 * (BATT_R1 + BATT_R2) / (BATT_R1); // is 10 times the battery voltage in volts
  Serial1.println(battery);
  // Create byte array to send (see https://www.thethingsnetwork.org/docs/devices/bytes.html for byte encoding examples)

  payload1[0] = highByte(desig);
  payload1[1] = lowByte(desig);
  payload1[2] = battery;
  payload1[3] = highByte(imuax);
  payload1[4] = lowByte(imuax);
  payload1[5] = highByte(imuay);
  payload1[6] = lowByte(imuay);
  payload1[7] = highByte(imuaz);
  payload1[8] = lowByte(imuaz);
  payload1[9] = highByte(imugx);
  payload1[10] = lowByte(imugx);
  payload1[11] = highByte(imugy);
  payload1[12] = lowByte(imugy);
  payload1[13] = highByte(imugz);
  payload1[14] = lowByte(imugz);
  payload1[15] = highByte(magmx);
  payload1[16] = lowByte(magmx);
  payload1[17] = highByte(magmy);
  payload1[18] = lowByte(magmy);
  payload1[19] = highByte(magmz);
  payload1[20] = lowByte(magmz);
  payload1[21] = highByte(prsr);
  payload1[22] = lowByte(prsr);
  payload1[23] = highByte(temp);
  payload1[24] = lowByte(temp);
  payload1[25] = highByte(adc);
  payload1[26] = lowByte(adc);
  payload1[27] = highByte(inkl_x);
  payload1[28] = lowByte(inkl_x);
  payload1[29] = highByte(inkl_y);
  payload1[30] = lowByte(inkl_y);
  payload1[31] = highByte(inkl_z);
  payload1[32] = lowByte(inkl_z);
  payload1[33] = highByte(smn1_temp);
  payload1[34] = lowByte(smn1_temp);
  
  //EXTRA PAYLOAD1
  extrapayload1[3] = highByte(imuay);
  extrapayload1[4] = lowByte(imuay);
  extrapayload1[7] = highByte(imugx);
  extrapayload1[8] = lowByte(imugx);
  extrapayload1[9] = highByte(imugy);
  extrapayload1[10] = lowByte(imugy);
  extrapayload1[11] = highByte(imugz);
  extrapayload1[12] = lowByte(imugz);
  extrapayload1[13] = highByte(magmx);
  extrapayload1[14] = lowByte(magmx);
  extrapayload1[15] = highByte(magmy);
  extrapayload1[16] = lowByte(magmy);
  extrapayload1[17] = highByte(magmz);
  extrapayload1[18] = lowByte(magmz);
  
  
  
  payload2[0] = highByte(desig);
  payload2[1] = lowByte(desig);
  payload2[2] = battery;
  payload2[3] = highByte(imuax);
  payload2[4] = lowByte(imuax);
  payload2[5] = highByte(imuay);
  payload2[6] = lowByte(imuay);
  payload2[7] = highByte(imuaz);
  payload2[8] = lowByte(imuaz);
  payload2[9] = highByte(imugx);
  payload2[10] = lowByte(imugx);
  payload2[11] = highByte(imugy);
  payload2[12] = lowByte(imugy);
  payload2[13] = highByte(imugz);
  payload2[14] = lowByte(imugz);
  payload2[15] = highByte(magmx);
  payload2[16] = lowByte(magmx);
  payload2[17] = highByte(magmy);
  payload2[18] = lowByte(magmy);
  payload2[19] = highByte(magmz);
  payload2[20] = lowByte(magmz);
  payload2[21] = highByte(prsr);
  payload2[22] = lowByte(prsr);
  payload2[23] = highByte(temp);
  payload2[24] = lowByte(temp);
  payload2[25] = highByte(adc);
  payload2[26] = lowByte(adc);
  payload2[27] = highByte(smn1_x);
  payload2[28] = lowByte(smn1_x);
  payload2[29] = highByte(smn1_y);
  payload2[30] = lowByte(smn1_y);
  payload2[31] = highByte(smn1_z);
  payload2[32] = lowByte(smn1_z);
  payload2[33] = highByte(smn1_temp);
  payload2[34] = lowByte(smn1_temp);
 
  //EXTRA PAYLOAD
  extrapayload2[3] = highByte(imuay);
  extrapayload2[4] = lowByte(imuay);
  extrapayload2[7] = highByte(imugx);
  extrapayload2[8] = lowByte(imugx);
  extrapayload2[9] = highByte(imugy);
  extrapayload2[10] = lowByte(imugy);
  extrapayload2[11] = highByte(imugz);
  extrapayload2[12] = lowByte(imugz);
  extrapayload2[13] = highByte(magmx);
  extrapayload2[14] = lowByte(magmx);
  extrapayload2[15] = highByte(magmy);
  extrapayload2[16] = lowByte(magmy);
  extrapayload2[17] = highByte(magmz);
  extrapayload2[18] = lowByte(magmz);

  Serial1.println(magmx);
  
  // Display data on Serial1 port
  sprintf(report, "Batt: %6d; IMU: A: %6d %6d %6d; G: %6d %6d %6d; M: %6d %6d %6d; INCL: %6d %6d %6d; T: %6d; P: %6d",
          battery,
          imuax, imuay, imuaz,
          imugx, imugy, imugz,
          magmx, magmy, magmz,
          inkl_x, inkl_y, inkl_z,
          temp, prsr);

  Serial1.println(report);
  Serial1.println();

  if (SET_SMN == 1)
  {
  sprintf(report2, "INCL: %6d; TEMP: %6d",
          smn1_x, smn1_temp);
  Serial1.println(report2);
  Serial1.println();
  }

  // Display raw data on Serial1 port
   //Serial1.write(payload,payloadsize);
   //Serial1.println();

  // SEND DATA VIA LoRa
  // Reset commands every loop!
  String commands[10]; // Maximum number of commands per message = 10
  String values[10];   // Maximum number of commands per message = 10
  int m = 0;           // Command counter
  int err;

  // Set dataRate manually - usually deactivated to allow for automatic data rate adaptation depending on signal to noise ratio
  //modem.dataRate(5);             // 5: switch to SF7
  delay(100);                    // because ... more stable
  // modem.beginPacket startet das Packet das mit LoRa versendet wird
  modem.beginPacket();

  // Determining payload to be sent to modem         
  if (loopctr < extrafreq)
    {        
  if (desig == 1) 
  {
    Serial1.println("LoRa Infrastructure Node");
    modem.write(extrapayload1, extrapayload1size);
  }
  if (desig == 2) 
  {
    Serial1.println("Subsurface Measurement Node");
    modem.write(extrapayload2, extrapayload2size);
  }
    }
    else
  {
  if (desig == 1) 
  {
    Serial1.println("LoRa Infrastructure Node: Extra Payload");
    modem.write(extrapayload1, extrapayload1size);
  }
  if (desig == 2)
  {
    Serial1.println("Subsurface Measurement Node: Extra Payload");
    modem.write(extrapayload2, extrapayload2size);
  }
    loopctr = 0;
  }

  //wenn modem.endPacket(true), dann confirmed data up!
  err = modem.endPacket(true);
  if (err > 0)
  {
    Serial1.println("Message sent correctly!");
  }
  else
  {
    Serial1.println("Error sending message :(");
    Serial1.println("(you may send a limited amount of messages per minute, depending on the signal strength");
    Serial1.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
  }

  // ----- CHECK if data upload was recieved -----
  if (!modem.available())
  {
    Serial1.println("No downlink message received at this time.");
  }
  else
  {
    char rcv[64];
    int i = 0;
    while (modem.available())
    {
      rcv[i++] = (char)modem.read();
    }

    // PRINT DATA
    Serial1.print("Message Received: ");
    // PRINT AS HEX CHAR by CHAR
    // for (unsigned int j = 0; j < i; j++) {
    // Serial1.print(rcv[j] >> 4, HEX);
    // Serial1.print(rcv[j] & 0xF, HEX);
    // Serial1.print(" ");
    // }

    // PRINT AS TXT
    String RcvStr = "";
    for (unsigned int j = 0; j < i; j++) {
      RcvStr = RcvStr + rcv[j];
    }
    Serial1.println(RcvStr);
    Serial1.println();

    // ANALYSE DATA
    Serial1.println("Checking for commands..."); // Commands have following stucture: COMMAND:VALUE;COMMAND:VALUE;COMMAND:VALUE;
    int k = RcvStr.length();
    int l = 0;
    while (k > 0) {
      k = RcvStr.indexOf(';');
      if (k > 0) {
        String commandtxt = RcvStr.substring(0, k);
        RcvStr = RcvStr.substring(k + 1);
        l = commandtxt.indexOf(':');
        String command = commandtxt.substring(0, l);
        String value = commandtxt.substring(l + 1);
        if (command != "") {
          Serial1.println("Command: " + command + ", Value: " + value);
          // Serial1.println("Remaining String: " + RcvStr);
          commands[m] = command;
          values[m] = value;
          m = m + 1;
        }
        else {
          Serial1.println("ERROR! Empty command");
        }
      }
      else {
        Serial1.println("No (more) commands found!");
      }
      Serial1.println();
    }
  }


  // PARSE COMMANDS
  for (int i = 0; i < m; i++) {
    Serial1.println("Processing Command: " + commands[i] + " ,Value: " + values[i]);

    // COMMAND MInt
    if (commands[i] == "MInt") {
      if ((values[i].toInt() >= 1) && (values[i].toInt() <= 31536000)) {
        loopintv = (values[i].toInt() * 1000);
        Serial1.println("Measurement interval has been set to " + values[i] + " s");
      }
      else {
        Serial1.println("Error! Wrong measurement interval value: " + values[i]);
      }
    }

    // COMMAND Not Recognized
    else {
      Serial1.println("Command was not recognized!");
    }
    Serial1.println();
  }

  // -------------------------------------------------------------------------------------------------------------------------------------------------------
  // END of LOOP: send Arduino to low power state
  delay(100);
  looptime = millis() - loopstart;
  Serial1.write("Loop duration: ");
  Serial1.print(looptime);
  Serial1.println(" ms");
  sleeptime = loopintv - looptime;
  Serial1.write("Sleep time: ");
  Serial1.print(sleeptime);
  Serial1.println(" ms");
  if (sleeptime < 10000) {
    sleeptime = 10000;
    Serial1.println("Warning! Sleeptime is too low - was set to 10000 ms");
  }

  
  
  // SET DIGITAL PORTS FOR SLEEP
  Serial1.println("Setting digital ports for sleep:");
  for (int i = 0; i <= 7; i++) {
    Serial1.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == Grove10DOF_DPORT || i == StpCtr1_DPORT) {
      pinMode(i, INPUT);
      Serial1.println(": Input");
    }
    else {
      Serial1.println(": not changed");
    }
  }

  Serial1.println("Sleeping..");
  delay(sleeptime);
  //LowPower.deepSleep(sleeptime);

  // SET DIGITAL PORTS FOR WAKEUP
  Serial1.println("Setting digital ports for wakeup:");
  for (int i = 0; i <= 7; i++) {
    Serial1.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == Grove10DOF_DPORT || i == StpCtr1_DPORT) {
      pinMode(i, OUTPUT);
      Serial1.println(": Output");
    }
    else {
      Serial1.println(": not changed");
    }
  }

}
