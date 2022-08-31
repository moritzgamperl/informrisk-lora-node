#include <agr_ads1220.h>
agr_ads1220 ads1220;

#include <SPI.h>

#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4


int RELAY_PIN = 4;
void setup() {
  //INITIALIZE SERIAL PORT
  Serial.begin(115200);

      pinMode(RELAY_PIN, OUTPUT);
pinMode(1, OUTPUT);

  //TURN ON/OFF DEBUG MESSAGES
  ads1220.ads1220debug = 0;

  //SET ADS1220 Reference voltages (if not set, 3,3 V is default value in all cases)
  ads1220.avdd = 3.3;
  //ads1220.vref0 = 3.3;
  //ads1220.vref1 = 3.3;
  //  digitalWrite(RELAY_PIN, HIGH);
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


}

void loop() {
  //MEASURE ADS1220
  //SETUP & MEASURE 4-20mA on CHANNEL 0
  digitalWrite(1, HIGH);
  delay(200);
  ads1220.config_mux(SE_CH1);
  ads1220.config_gain(PGA_GAIN_1);
  ads1220.config_pga(PGA_OFF);
  ads1220.config_datarate(DR_20SPS);
  ads1220.config_opmode(OM_NORMAL);
  ads1220.config_conmode(CM_SINGLE_SHOT);
  ads1220.config_tempmode(TEMP_OFF);
  ads1220.config_burnout(BO_OFF);
  ads1220.config_vref(VREF_AVDD); // <-- better to use internal 2V Reference if possible (measurement resistor has to be about 80 OHM)
  ads1220.config_fir(FIR_50_60);
  ads1220.config_idac_cur(IDAC_CUR_1500);
  ads1220.config_idac2(IDAC2_OFF);
  ads1220.config_idac1(IDAC1_REFP0);
  delay(200); //Delay to powerup IDAC
  int32_t barodata = ads1220.read_single_shot();
  //SETUP & MEASURE LINEARPOTI on CHANNEL 2
  ads1220.config_mux(SE_CH2);
  ads1220.config_gain(PGA_GAIN_1);
  ads1220.config_pga(PGA_OFF);
  ads1220.config_datarate(DR_20SPS);
  ads1220.config_opmode(OM_NORMAL);
  ads1220.config_conmode(CM_SINGLE_SHOT);
  ads1220.config_tempmode(TEMP_OFF);
  ads1220.config_burnout(BO_OFF);
  ads1220.config_vref(VREF_AVDD);
  ads1220.config_fir(FIR_50_60);
  ads1220.config_idac_cur(IDAC_CUR_100);
  ads1220.config_idac2(IDAC2_REFN0);
  ads1220.config_idac1(IDAC1_OFF);
  delay(200); //Delay to powerup IDAC
  int32_t lpotdata = ads1220.read_single_shot();
  //SETUP & MEASURE TEMPERATURE
  ads1220.config_tempmode(TEMP_ON);
  ads1220.config_idac2(IDAC2_OFF);
  ads1220.config_idac1(IDAC1_OFF);
  delay(50);
  int32_t tempdata = ads1220.read_single_shot();
    //POWER DOWN ADS1220
  ads1220.powerdown();
  //CONVERT BAROMETER DATA  
  float barovoltage = ads1220.dac2mv(barodata);
  float barocurrent = barovoltage/82;
  float baro = 800 + (260/16*(barocurrent-4)); 
  Serial.print("Barometer measurement: voltage: ");
  Serial.print(barovoltage);
  Serial.print(" mV -- current: ");
  Serial.print(barocurrent);
  Serial.print(" mA -- pressure: ");
  Serial.print(baro);
  Serial.println(" mbar");
  //CONVERT LINEARPOTI DATA
  float lpotvoltage = ads1220.dac2mv(lpotdata);
  float lpot = lpotvoltage * 0.9572; // ROUGH CALIBRATION LINEARPOTI
  Serial.print("Linear potentiometer: voltage: ");
  Serial.print(lpotvoltage);
  Serial.print(" mV -- position: ");
  Serial.print(lpot);
  Serial.println(" mm");
  //CONVERT TEMPERATURE DATA
  float temp = ads1220.dac2temp(tempdata);
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");
  delay (5000);
  //WAKE UP ADS1220
  ads1220.start_conv();
}
