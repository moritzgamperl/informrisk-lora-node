#include <agr_ads1220.h>
agr_ads1220 ads1220;
#include <MKRWAN.h>           //  Library: MKRWAN from Arduino - Library for Arduino MKR WAN boards

#include <SPI.h>

#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4

float lpot;
float lpotvoltage;


void setup() {
  //INITIALIZE SERIAL PORT
  Serial.begin(115200);

  delay(1000);
  Serial.write("ads1220 test.");
  /*
pinMode(5, OUTPUT);
digitalWrite(5, HIGH);
*/
  //TURN ON/OFF DEBUG MESSAGES
  ads1220.ads1220debug = 0;

  //SET ADS1220 Reference voltages (if not set, 3,3 V is default value in all cases)
  ads1220.avdd = 3.3;
  //ads1220.vref0 = 3.3;
  //ads1220.vref1 = 3.3;

  //INITIALIZE ADS1220
  Serial.println("ADS1220 Initialization...");
  if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {Serial.println("ADS1220 initialized successfully!");}
  else {Serial.println("ADS1220 was NOT initialized successfully!");}
  Serial.println();
    
  //CALIBRATE ADS1220
  Serial.println("ADS1220 Calibration (this may take a few seconds)...");
  ads1220.calibrate();
  Serial.println();

  //SETUP & MEASURE LINEARPOTI on CHANNEL 2
  ads1220.config_mux(SE_CH3);
  ads1220.config_gain(PGA_GAIN_1);
  ads1220.config_pga(PGA_OFF);
  ads1220.config_datarate(DR_20SPS);
  ads1220.config_opmode(OM_NORMAL);
  ads1220.config_conmode(CM_SINGLE_SHOT);
  ads1220.config_tempmode(TEMP_OFF);
  ads1220.config_burnout(BO_OFF);
  ads1220.config_vref(VREF_INT);
  ads1220.config_fir(FIR_NONE);
  ads1220.config_idac_cur(IDAC_CUR_10);
  ads1220.config_idac2(IDAC2_OFF);
  ads1220.config_idac1(IDAC1_REFN0);
  delay(200); //Delay to powerup IDAC
  //GET INITIAL VALUES
  int32_t lpotdata = ads1220.read_single_shot();
  lpotvoltage = ads1220.dac2mv(lpotdata);
  lpot = lpotvoltage * 0.9572;
}

  void loop() {
  //MEASURE ADS1220
  int32_t lpotdata = ads1220.read_single_shot();

  //CONVERT LINEARPOTI DATA
  lpotvoltage = ads1220.dac2mv(lpotdata);
  lpot = (0.5 * lpot) + (0.5 * lpotvoltage * 0.9572); // ROUGH CALIBRATION LINEARPOTI; reduces noise (0,5 old value + 0,5 new value)
  Serial.print("Linear potentiometer: voltage: ");
  Serial.print(lpotvoltage);
  Serial.print(" mV -- position: ");
  Serial.print(lpot);
  Serial.println(" mm");

  delay (100);
}
