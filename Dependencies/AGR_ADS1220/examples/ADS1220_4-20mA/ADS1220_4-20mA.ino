#include <agr_ads1220.h>
agr_ads1220 ads1220;

#include <SPI.h>

#define ADS1220_CS_PIN    1
#define ADS1220_DRDY_PIN  0

int RELAY_PIN = 4;


void setup() {
  //INITIALIZE SERIAL PORT
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);

  //TURN ON/OFF DEBUG MESSAGES
  ads1220.ads1220debug = 0;

  //SET ADS1220 Reference voltages (if not set, 3,3 V is default value in all cases)
  ads1220.avdd = 3.3;
  //ads1220.vref0 = 3.3; 
  //ads1220.vref1 = 3.3;
  
  digitalWrite(RELAY_PIN, HIGH);
  delay(500);
    
  //INITIALIZE ADS1220
  Serial.println("ADS1220 Initialization...");
  if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {Serial.println("ADS1220 initialized successfully!");}
  else {Serial.println("ADS1220 was NOT initialized successfully!");}
  Serial.println();
    
  //CALIBRATE ADS1220
  Serial.println("ADS1220 Calibration (this may take a few seconds)...");
  ads1220.calibrate();
  Serial.println();

  //SETUP MEASUREMENT OF 4-20mA on CHANNEL 1
  ads1220.config_datarate(DR_20SPS);
  ads1220.config_opmode(OM_NORMAL);
  ads1220.config_conmode(CM_SINGLE_SHOT);
  ads1220.config_tempmode(TEMP_OFF);
  ads1220.config_burnout(BO_OFF);
  ads1220.config_vref(VREF_AVDD);
  ads1220.config_fir(FIR_50_60);
  ads1220.config_idac_cur(IDAC_CUR_500);
  ads1220.config_idac1(IDAC1_REFN0);
  delay(200); // delay to allow IDAC to powerup

    digitalWrite(RELAY_PIN, LOW);

}

void loop() {
  // ADC Measurement
     //INITIALIZE ADS1220
  Serial.println("ADS1220 Initialization...");
  if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {Serial.println("ADS1220 initialized successfully!");}
  else {Serial.println("ADS1220 was NOT initialized successfully!");}
  Serial.println();
      
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);
    ads1220.config_mux(SE_CH1);
    ads1220.config_gain(PGA_GAIN_1);
    ads1220.config_pga(PGA_OFF);
    int32_t ssdata = ads1220.read_single_shot();
    float voltage = ads1220.dac2mv(ssdata);
    float current = voltage/150;
    float mbar = 800 + (260/16*(current-4)); 
    Serial.print("voltage: ");
    Serial.print(voltage);
    Serial.print(" mV -- current: ");
    Serial.print(current);
    Serial.print(" mA -- pressure: ");
    Serial.print(mbar);
    Serial.println(" mbar");
    delay (1000);
}
