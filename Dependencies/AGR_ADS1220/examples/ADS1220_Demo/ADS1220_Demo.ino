#include "agr_ads1220.h"
agr_ads1220 ads1220;

#include <SPI.h>

#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4

void setup() {
  //INITIALIZE SERIAL PORT
    Serial.begin(115200);
    Serial.println("TEST");
    delay(2000);

    pinMode(16,OUTPUT);
    pinMode(17,OUTPUT);
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);

  //TURN ON/OFF DEBUG MESSAGES
  ads1220.ads1220debug = 1;

  //SET ADS1220 Reference voltages (if not set, 3,3 V is default value in all cases)
  ads1220.avdd = 3.3;
  //ads1220.vref0 = 3.3;
  //ads1220.vref1 = 3.3;

  //INITIALIZE ADS1220
  Serial.println("ADS1220 Initialization...");
  if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {Serial.println("ADS1220 initialized successfully!");}
  else {Serial.println("ADS1220 was NOT initialized successfully!");}
  Serial.println();

    digitalWrite(16,HIGH);
  digitalWrite(17,HIGH);
  digitalWrite(18,HIGH);
  digitalWrite(19,HIGH);

  Serial.println("Pulses output");

  delay(2000);

    
  //CALIBRATE ADS1220
  Serial.println("ADS1220 Calibration (this may take a few seconds)...");
  ads1220.calibrate();
  Serial.println();
  //DEVELOPMENT: RUN CALIBRATION IN CONTINUOUS MODE --> MUCH FASTER?




  //CHECK CALIBRATION
  //Rerun Calibration test with calibration offset activee --> New calculated average offset should ideally be 0.
  Serial.println("ADS1220 Testing Calibration...");
  ads1220.config_backup();
    if (ads1220.ads1220debug) {
    ads1220.config_cur_print();
    ads1220.config_act_print();
    ads1220.config_tmp_print();
    }
  uint8_t config_reg0 = 0xE0;   //AINp and AINn shorted to (AVDD + AVSS) / 2; Gain 1; PGA enabled
  uint8_t config_reg1 = 0x00;   //DR=20 SPS, Mode=Normal, Conv mode=Single_Shot, Temp Sensor disabled, Current Source off
  uint8_t config_reg2 = 0x10;   //Default settings: Vref internal, 50/60Hz rejection, power open, IDAC off
  uint8_t config_reg3 = 0x00;   //Default settings: IDAC1 disabled, IDAC2 disabled, DRDY pin only
  ads1220.write_reg0(config_reg0);
  ads1220.write_reg1(config_reg1);
  ads1220.write_reg2(config_reg2);
  ads1220.write_reg3(config_reg3);
  if (ads1220.ads1220debug) {
    ads1220.config_cur_print();
    ads1220.config_act_print();
    ads1220.config_tmp_print();
    }
  delay(100);
  int itot = 50;
  int32_t ssdatasum = 0;
  int32_t result = 0;
  int j = 0;
  for (int i = 0; i < itot; i++) {
    int32_t ssdata = ads1220.read_single_shot();
    if (ads1220.ads1220debug) {Serial.print(ssdata);}
    if (ads1220.ads1220debug) {Serial.println("; ");}
    ssdatasum = ssdatasum + ssdata;
    j = j+1;
  }
  result = ssdatasum / itot;
  Serial.print("Mean (n = ");
  Serial.print(j);
  Serial.print(") Offset (should be near 0): ");  
  Serial.println(result);
  Serial.print(ads1220.dac2mv(result),3); // Output as MilliVolt
  Serial.println(" mV");
  ads1220.config_restore();
    if (ads1220.ads1220debug) {
    ads1220.config_cur_print();
    ads1220.config_act_print();
    ads1220.config_tmp_print();
    }
  Serial.println();
  
  //PERFORM MEASUREMENTS (DIFFERENTIAL CH0-1) SINGLE-SHOT-MODE
  Serial.println("ADS1220 Differential Measurement Channel 0-1; Internal Voltage Reference (2V max.); PGA bypassed, gain 1; Normal Mode; Single-Shot Mode:");
  if (ads1220.ads1220debug) {Serial.println("ADS1220 - Configuration");}
  ads1220.config_mux(DIF_CH0_1);
  ads1220.config_gain(PGA_GAIN_1);
  ads1220.config_pga(PGA_OFF);
  ads1220.config_datarate(DR_20SPS);
  ads1220.config_opmode(OM_NORMAL);
  ads1220.config_conmode(CM_SINGLE_SHOT);
  ads1220.config_tempmode(TEMP_OFF);
  ads1220.config_burnout(BO_OFF);
  ads1220.config_vref(VREF_AVDD);
  ads1220.config_fir(FIR_50_60);
  
  if (ads1220.ads1220debug) {
    ads1220.config_cur_print();
    ads1220.config_act_print();
    ads1220.config_tmp_print();
    }

  itot = 50;
  ssdatasum = 0;
  result = 0;
  j = 0;
  for (int i = 0; i < itot; i++) {
    int32_t ssdata = ads1220.read_single_shot();
    if (ads1220.ads1220debug) {Serial.print(ssdata);}
    if (ads1220.ads1220debug) {Serial.println("; ");}
    ssdatasum = ssdatasum + ssdata;
    j = j+1;
  }
  result = ssdatasum / itot;
  Serial.print("Mean value of ");
  Serial.print(j);
  Serial.print(" measurements: ");
  Serial.println(result);
  Serial.print(ads1220.dac2mv(result),3); // Output as MilliVolt
  Serial.println(" mV");
  Serial.println();

  //PERFORM MEASUREMENTS (DIFFERENTIAL CH0-1) CONTINUOUS-MODE
  Serial.println("ADS1220 Differential Measurement Channel 0-1; Internal Voltage Reference (2V max.); PGA bypassed, gain 1; Normal Mode; Continuous Mode:");
  if (ads1220.ads1220debug) {Serial.println("ADS1220 - Starting Continuous Mode");}
  ads1220.meas_start();
  
  if (ads1220.ads1220debug) {
    ads1220.config_cur_print();
    ads1220.config_act_print();
    ads1220.config_tmp_print();
    }

  itot = 50;
  ssdatasum = 0;
  result = 0;
  j = 0;
  for (int i = 0; i < itot; i++) {
    int32_t ssdata = ads1220.get_data();
    if (ads1220.ads1220debug) {Serial.print(ssdata);}
    if (ads1220.ads1220debug) {Serial.println("; ");}
    ssdatasum = ssdatasum + ssdata;
    j = j+1;
  }
  ads1220.meas_stop();
  result = ssdatasum / itot;
  Serial.print("Mean value of ");
  Serial.print(j);
  Serial.print(" measurements: ");
  Serial.println(result);
  Serial.print(ads1220.dac2mv(result),3); // Output as MilliVolt
  Serial.println(" mV");
  Serial.println();
  
  //CHECK SENSOR STATUS ON CURRENT CHANNEL
  Serial.println("ADS1220 Sensor health check (checking for shorted or open circuit on current channel)");
  int senscheck = ads1220.sensorcheck(); //if sensor is OK: 0; if sensor is shorted: 1; if sensor is open: 2;
  if (senscheck == 1) {Serial.println("Open circuit detected!");}
  else if (senscheck == 2) {Serial.println("Shorted circuit detected!");}
  else {Serial.println("Sensor OK!");}
  Serial.println();

  //PERFORM TEMPERATURE MEASUREMENTS - SINGLE-SHOT-MODE
  Serial.println("ADS1220 Temperature measurement: Single Shot mode");
  ads1220.config_tempmode(TEMP_ON);
  if (ads1220.ads1220debug) {
    ads1220.config_cur_print();
    ads1220.config_act_print();
    ads1220.config_tmp_print();
    }
  itot = 10;
  float tempdatasum = 0;
  float tempresult = 0;
  for (int i = 0; i < itot; i++) {
    float tempdata = ads1220.dac2temp(ads1220.read_single_shot());
    if (ads1220.ads1220debug) {Serial.print(tempdata);}
    if (ads1220.ads1220debug) {Serial.println("°C; ");}
    tempdatasum = tempdatasum + tempdata;
  }
  tempresult = tempdatasum / itot;
  Serial.print("Mean value of 10 measurements: ");
  Serial.print(tempresult,2);
  Serial.println("°C");
  Serial.println();
  
  ads1220.config_tempmode(TEMP_OFF);
  if (ads1220.ads1220debug) {
    ads1220.config_cur_print();
    ads1220.config_act_print();
    ads1220.config_tmp_print();
    }

  //TURN ON LOW SIDE POWER SWITCH
  Serial.println("ADS1220 Closing Low Side Power Switch on CHANNEL AIN3 for 10 s");
  ads1220.config_lspsw(LSPSW_CLOSE);
  delay(10000);
  Serial.println("ADS1220 Opening Low Side Power Swirch");
  ads1220.config_lspsw(LSPSW_OPEN);
  Serial.println();

  //TURING ON IDAC CURRENT OUTPUT ON CHANNEL REFP0
  Serial.println("ADS1220 Turning on IDAC Current on CHANNELS REFP0 and REFN0 for 10 s each");
  ads1220.config_idac_cur(IDAC_CUR_500);
  Serial.println("ADS1220 Turning on REFP0...");
  ads1220.config_idac1(IDAC1_REFP0);
  delay(10000);
  Serial.println("ADS1220 Turning off REFP0...");
  ads1220.config_idac1(IDAC1_OFF);
  Serial.println("ADS1220 Turning on REFN0...");
  ads1220.config_idac2(IDAC2_REFN0);
  delay(10000);
  Serial.println("ADS1220 Turning off REFN0...");
  ads1220.config_idac2(IDAC2_OFF);
  Serial.println("ADS1220 Turning on REFP0 AND REFN0...");
  ads1220.config_idac1(IDAC1_REFP0);
  ads1220.config_idac2(IDAC2_REFN0);
  delay(10000);
  Serial.println("ADS1220 Turning off REFP0 AND REFN0...");
  ads1220.config_idac1(IDAC1_OFF);
  ads1220.config_idac2(IDAC2_OFF);
  Serial.println();
  
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("Done!");
delay (60000);
}
