/*  Inform@Risk LoRa Node
    (c) AlpGeorisk & TUM
    Authors: Moritz Gamperl & John Singer

    LoRaBasis_InformRisk
    Version: 0.5
    Date: 16.09.2021

    Supported Hardware:
    BOARDS:
    - Arduino MKR WAN 1310

    SENSORS:
    - ICM 20948
    - ADS1220 ADC
    - BMP388
*/

#include "Uni_Config.h"            // -- Includes necessary File Includes and Variable Declarations -- //

void setup(){

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

  // ------ VARIABLE DEFINTION DURING RUNTIME ------ //
  
  SP.begin(9600);                              // OPEN Serial PORT -- used for development purposes
  while (!SP && millis() < 5000);              // Wait 5 secs for Serial connection otherwise continue anyway...
  Wire.begin();
  
  if (SET_IMU == false) IMU_DPORT = -1;            // SET PINS TO -1 IF DEACTIVATED; ADC comes later
  //if (SET_ADC = false) {
  if (SET_SMN == false) StpCtr1_DPORT = -1;

  //TODO : ADC SETTINGS  ???

  ads1220.ads1220debug = 1;                      //TURN ON/OFF DEBUG MESSAGES
  ads1220.avdd = 3.3;                            //SET ADS1220 Reference voltages (if not set, 3,3 V is default value in all cases)
  //ads1220.vref0 = 3.3;
  //ads1220.vref1 = 3.3;
  

  // ----- SETUP DIGITAL PINS ------ // : Set measurement pins to output, set unused pins to input_pullup
  
  SP.println("Digital ports setup:");
  for (int i = 0; i <= 7; i++) {
    SP.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == IMU_DPORT || i == StpCtr1_DPORT) {
      pinMode(i, OUTPUT);
      SP.println(": Output");
    }
    else {
      if (i == LED_BUILTIN) {                     // SET LED port to output low, all other unused ports to INPUT_PULLUP to save energy
        pinMode(i, INPUT);
        SP.println(": Input (LED Off)");
      }
      else {
        pinMode(i, INPUT_PULLUP);
        SP.println(": INPUT_PULLUP");
      }
    }
  }

  // ------ RELAIS SETTINGS ------ //
  
  pinMode(A2, OUTPUT);                                               // Relais K1
  pinMode(A3, OUTPUT);
  K1_AllOff ();
  
  pinMode(A4, OUTPUT);                                               // Relais K2
  pinMode(A5, OUTPUT);
  K2_AllOff ();

  analogReadResolution(12);                                          // SETUP ANALOG INPUT
  
  delay(sensorstarttime);                                            // wait for sensors to startup

  if (SET_SCL == true) {
  // TODO: Setup PINS MURATASC3300
  // pinMode(SCL3300_Power_PIN, OUTPUT);
  //pinMode(SCL3300_CS_PIN, OUTPUT);
  }

     
  // ------ CONNECT TO LORA NETWORK ------ //
  if (!modem.begin(EU868))                               // Change this to your Regional Band (eg. US915, AS923, ...). For Colombia, use "US915"!. For Europe, use "EU868".
  {
    SP.println("Failed to start module");
    while (1) {}
  };
  
  String ArdVers = modem.version();
  String DevEUI = modem.deviceEUI();
  SP.print("Your module version is: ");
  SP.println(ArdVers);
  SP.print("Your device EUI is: ");
  SP.println(DevEUI);

  int connected = modem.joinOTAA(appEui, appKey);        // Attempt to Join LoRaWAN Network using Over-The-Air-Activation (OTAA)
  delay(100);
  if (connected)
  {
    // ----- Setup Signal strength ------
    //      change SF by choosing data rate (bool dataRate(uint8_t dr)), can be between 0 (SF 12) and 6 (SF 7)
    //      ADR=Adaptive Data Rate allows modulation of SW and BW depending on signal strength from gateway. Is on by default
    //      https://github.com/arduino/mkrwan1300-fw/issues/3
    //try: modem.setADR(true);
    //modem.dataRate(5);
    
    modem.minPollInterval(10);     // Set poll interval to x seconds
    delay(100);                    // For more Stable Connection
  }
  else
  {
    SP.println("Something went wrong; Are you indoors? Move near a Window and Retry");
    while (1) {}
  }

  SP.println("Connected");

}


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{

  // ------ PAYLOAD DECLARATIONS ------ //


  short payload1size = 6;
  
  if(SET_IMU == true){payload1size += 18;}      // If loops to estimate payload size - implemented only for the 3 sensors discussed
  if(SET_ADC == true){payload1size += 12;}      
  if(SET_SMN == true){payload1size += 8;} 

  byte payload1[payload1size];
  
  //TODO: Same type of if loop for extra payload // 
  short extrapayload1size = 35;
  byte extrapayload1[extrapayload1size];
  
  loopctr++;                                   // Loop Counter
  
  loopstart = millis();                        //MEASUREMENT LOOP
  SP.println();
  SP.print("New Measurement, Starttime: ");
  SP.println(loopstart);
  //Wire.begin();

  // ------- ACTIVATE SENSORS ------- //
  if (BATT_DPORT >= 0) digitalWrite(BATT_DPORT, HIGH);            
  if (LED_DPORT >= 0) digitalWrite(LED_DPORT, HIGH);
  if (SW33_A_DPORT >= 0) digitalWrite(SW33_A_DPORT, HIGH);
  if (SW33_B_DPORT >= 0) digitalWrite(SW33_B_DPORT, HIGH);
  if (SW12_DPORT >= 0) digitalWrite(SW12_DPORT, HIGH);
  if (IMU_DPORT >= 0) digitalWrite(IMU_DPORT, HIGH);
  if (StpCtr1_DPORT >= 0) digitalWrite(StpCtr1_DPORT, HIGH);

  // ------ INITIALIZE ADS1220 ------ //
  if (SET_ADC == true){
  SP.println("ADS1220 Initialization...");
  if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {SP.println("ADS1220 initialized successfully!");}
  else {SP.println("ADS1220 was NOT initialized successfully!");}
  SP.println();

  // ------ CALIBRATE ADS1220 ------ //
  if (ADC_CAL == true){
  SP.println("ADS1220 Calibration (this may take a few seconds)...");
  ads1220.calibrate();
  SP.println();
  }
  ads1220.config_datarate(DR_20SPS);
  ads1220.config_opmode(OM_NORMAL);
  ads1220.config_conmode(CM_SINGLE_SHOT);
  ads1220.config_tempmode(TEMP_OFF);
  ads1220.config_burnout(BO_OFF);
  ads1220.config_vref(VREF_AVDD);
  ads1220.config_fir(FIR_50_60);
  ads1220.config_idac_cur(IDAC_CUR_500);
  ads1220.config_idac1(IDAC1_REFN0);
  //ads1220.config_idac2(IDAC2_AIN3);
  //delay(200);                                          //Delay to powerup IDAC - removed because of sensorstarttime
  //GET INITIAL VALUES
  //int32_t lpotdata = ads1220.read_single_shot();
  //lpotvoltage = ads1220.dac2mv(lpotdata);
  //lpot = lpotvoltage * 0.9572;
  }
  else ads1220.powerdown();

  delay(sensorstarttime);                                // Wait for Sensors to Startup
  SP.println("BREAK: sensors should be started");

  
  // ------ INITIALIZE SENSORS AFTER POWERUP ------ //
  
  if (IMU_DPORT == HIGH ) {                                  // IMU INITIALIZE
    if(!myIMU.init()){
      Serial.println("ICM20948 does not respond");
    }
    else{
      Serial.println("ICM20948 is connected");
    }
    if(!myIMU.initMagnetometer()){
      Serial.println("Magnetometer does not respond");
    }
    else{
      Serial.println("Magnetometer is connected");
    }

    myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
    myIMU.setAccDLPF(ICM20948_DLPF_6);    
    myIMU.setAccSampleRateDivider(10);
    myIMU.setGyrDLPF(ICM20948_DLPF_6);  
    myIMU.setGyrSampleRateDivider(10);
    myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
  }


  bmp388.begin();                                            //  BMP INITIALIZE   // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp388.setTimeStandby(TIME_STANDBY_1280MS);                                     // Set the standby time to 1.3 seconds
  bmp388.startNormalConversion();                                                 // Start BMP388 continuous conversion in NORMAL_MODE  
  

  if (SET_SMN == true){                                      // SMN INITIALIZE
    bma456.initialize();
  }


  if (SET_SCL == true) {
    inclinometer.WakeMeUp();                                 // Turn on Murata
    //digitalWrite(SCL3300_Power_PIN, HIGH);
    delay(10);

    if (inclinometer.begin(scl3300_sspin) == false) {                  // SETUP MURATA
      SP.println("Murata SCL3300 inclinometer not connected.");
      while(1);                                                        //Freeze
    }
    inclinometer.setMode(scl3300_mode);
  }


  // ------ MEASUREMENTS ------ //  
  measstart = millis();                                                 //Reset Timing & Counters
  BATT_MCTR = 1;
  IMU_MCTR = 1;
  MAG_MCTR = 1;
  BARO_MCTR = 1;
  SMN_MCTR = 1;
  INKL_MCTR = 1;
  ADC_MCTR = 1;
  SP.println("BREAK: Measurement started, counters set to 1");

  for (int i = 0; i <= 21; i++){
    meassum[i] = 0;                                                    //Reset Measurent Variables
  }

  while (millis() - measstart <= measlength)
  {
    meastime = millis() - measstart;
    
    if (BATT_DPORT >= 0 && meastime >= BATT_MINT * BATT_MCTR) {        // BATT VOLTAGE MEASUREMENT
      BATT_MCTR++;
      int batt = analogRead(BATT_APORTin);                             // Read Analog Port
      meassum[0] = meassum[0] + batt;
      SP.print("BREAK: Battery voltage finished:");
      SP.println(batt);
    }

    if (IMU_DPORT >= 0 && meastime >= IMU_MINT * IMU_MCTR) {           // IMU MEASUREMENT
      IMU_MCTR++;
      SP.println("Measuring IMU....");
      
      myIMU.readSensor();                                              // Get IMU Sensor Data
      xyzFloat gValue = myIMU.getGValues();
      xyzFloat gyr = myIMU.getGyrValues();
      xyzFloat magValue = myIMU.getMagValues();
      float imu_temp = myIMU.getTemperature();
      float resultantG = myIMU.getResultantG(gValue);
    
      SP.print(gValue.x);
      
      meassum[1] = meassum[1] + gValue.x;                              // Adding Measured Values to the Measurement Array Variable
      meassum[2] = meassum[2] + gValue.y; 
      meassum[3] = meassum[3] + gValue.z;
      meassum[4] = meassum[4] + gyr.x;
      meassum[5] = meassum[5] + gyr.y;
      meassum[6] = meassum[6] + gyr.z;
      meassum[7] = meassum[7] + magValue.x;
      meassum[8] = meassum[8] + magValue.y;
      meassum[9] = meassum[9] + magValue.z;
      
      arr_imu_a_x[IMU_MCTR - 1] = gValue.x;                            // To calculate the Median??
      arr_imu_a_y[IMU_MCTR - 1] = gValue.y;
      arr_imu_a_z[IMU_MCTR - 1] = gValue.z;
      arr_imu_g_x[IMU_MCTR - 1] = gyr.x;
      arr_imu_g_y[IMU_MCTR - 1] = gyr.y;
      arr_imu_g_z[IMU_MCTR - 1] = gyr.z;
      arr_mag_m_x[MAG_MCTR - 1] = magValue.x;
      arr_mag_m_y[MAG_MCTR - 1] = magValue.y;
      arr_mag_m_z[MAG_MCTR - 1] = magValue.z;
    }  
    
    if (IMU_DPORT >= 0 && meastime >= BARO_MINT * BARO_MCTR) {                         // BARO MEASUREMENT
      BARO_MCTR++;
      if (bmp388.getMeasurements(temperature, pressure, altitude))   // Check if the measurement is complete
      {
        Serial.print(temperature);                                   // Display the results    
        Serial.print(F("*C   "));
        Serial.print(pressure);    
        Serial.print(F("hPa   "));
        Serial.print(altitude);
        Serial.println(F("m"));  
       }
       meassum[10] = meassum[10] + round(temperature*10);
       meassum[11] = meassum[11] + round(pressure / 100);            // Pa div. 100 equ. hPa = mBar
    }

    if (SET_SCL == true && meastime >= INKL_MINT * INKL_MCTR){            // SCL Inclinometer Measurement
      INKL_MCTR++;
      if (inclinometer.begin(scl3300_sspin) == false) {
        SP.println("Murata SCL3300 inclinometer not connected.");
        while(1);                                                      //Freeze
      }
      inclinometer.setMode(scl3300_mode);
      delay(1);
      if (inclinometer.available()) {
        float incl_raw_x = (inclinometer.getTiltLevelOffsetAngleX());     // Inline these functions to save memory.
        float incl_raw_y = (inclinometer.getTiltLevelOffsetAngleY());
        float incl_raw_z = (inclinometer.getTiltLevelOffsetAngleZ());
        meassum[13] = meassum[13] + round(incl_raw_x*1000);
        meassum[14] = meassum[14] + round(incl_raw_y*1000);
        meassum[15] = meassum[15] + round(incl_raw_z*1000);
      } 
      else inclinometer.reset();
      delay(10);
    }

    if (SET_SMN == true && meastime >= SMN_MINT * SMN_MCTR){               // SMN MEASUREMENT (for only one SMN)
      SMN_MCTR++;
      bma456.getAcceleration(&smn_x, &smn_y, &smn_z);
      smn_temp = bma456.getTemperature();

      meassum[16] = meassum[16] + smn_x;
      meassum[17] = meassum[17] + smn_y;
      meassum[18] = meassum[18] + smn_z;
      meassum[19] = meassum[19] + smn_temp;
      
    }
    delay(50);                                                          // DELAY after every measurement cycle
  }

  if (SET_SCL == true){                                                 // Turn off Murata
    //digitalWrite(SCL3300_Power_PIN, LOW);
    digitalWrite(SCL3300_CS_PIN, HIGH);
    inclinometer.powerDownMode();
  }

  while(ADC_MCTR < 10){                                                 // ADC MEASUREMENT
    ADC_MCTR++;
    if (SET_ADC == true){
      if (ADS_C1 == 1){                                                 //CHannel 1 measurement (4-20 mA):
        SP.println("ADS1220 Initialization...");
        if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {SP.println("ADS1220 initialized successfully!");}
        else {SP.println(("ADS1220 was NOT initialized successfully!"));}
      SP.println();
          
      delay(10);                                                        
      ads1220.config_mux(SE_CH0);
      ads1220.config_gain(PGA_GAIN_1);
      ads1220.config_pga(PGA_OFF);
      int32_t ssdata = ads1220.read_single_shot();
      float voltage = ads1220.dac2mv(ssdata);
      float current = voltage/150;
      float mbar = 800 + (260/16*(current-4)); 
      SP.print("SSDATA: ");
      SP.print(ssdata);
      SP.print("C1 voltage: ");
      SP.print(voltage);
      SP.print("C1 mV -- current: ");
      SP.print(current);
      SP.println(" mbar");  
      }  
      if (ADS_C2 == 1){                                                 // Channel 2 measurement (Potentiometer):      
      ads1220.config_mux(SE_CH2);
      ads1220.config_gain(PGA_GAIN_4);
      ads1220.config_pga(PGA_ON);
      int32_t lpotdata = ads1220.read_single_shot();
      lpotvoltage = ads1220.dac2mv(lpotdata);
      SP.print(lpotvoltage);
      SP.println(" C2 ipotvoltage");  
      }
    }
  }

  // ------ MEASUREMENT END AND DEACTIVATING SENSORS ------ //
  
  SP.println("MEASUREMENT END");
  if (BATT_DPORT >= 0) digitalWrite(BATT_DPORT, LOW);
  if (LED_DPORT >= 0) digitalWrite(LED_DPORT, LOW);
  if (SW33_A_DPORT >= 0) digitalWrite(SW33_A_DPORT, LOW);
  if (SW33_B_DPORT >= 0) digitalWrite(SW33_B_DPORT, LOW);
  if (SW12_DPORT >= 0) digitalWrite(SW12_DPORT, LOW);
  if (SET_ADC == true) ads1220.powerdown();
  if (IMU_DPORT >= 0) digitalWrite(IMU_DPORT, LOW);
    
   if (SET_SMN == true) {
    if (StpCtr1_DPORT >= 0) digitalWrite(StpCtr1_DPORT, LOW); 
  }


  SP.print("Measurement duration inkl. initialization (sensors on): ");         //Display measurement duration
  SP.println(millis() - loopstart);


  int16_t battv = round(meassum[0] / (BATT_MCTR - 1));                          //Calculate averages
  int16_t imuax = round(meassum[1] / (IMU_MCTR - 1));
  int16_t imuay = round(meassum[2] / (IMU_MCTR - 1));
  int16_t imuaz = round(meassum[3] / (IMU_MCTR - 1));
  int16_t imugx = round(meassum[4] / (IMU_MCTR - 1));
  int16_t imugy = round(meassum[5] / (IMU_MCTR - 1));
  int16_t imugz = round(meassum[6] / (IMU_MCTR - 1));
  int16_t magmx = round(meassum[7] / (IMU_MCTR - 1));
  int16_t magmy = round(meassum[8] / (IMU_MCTR - 1));
  int16_t magmz = round(meassum[9] / (IMU_MCTR - 1));
  int16_t temp = round(meassum[10] / (BARO_MCTR - 1));
  int16_t prsr = round(meassum[11] / (BARO_MCTR - 1));
  int16_t adc = round(meassum[12] / (ADC_MCTR - 1));
  int16_t inkl_x = round(meassum[13] / (INKL_MCTR - 1));           // SCL Values x,y,z
  int16_t inkl_y = round(meassum[14] / (INKL_MCTR - 1));
  int16_t inkl_z = round(meassum[15] / (INKL_MCTR - 1));
  int16_t smn1_x = round(meassum[16] / (SMN_MCTR - 1));
  int16_t smn1_y = round(meassum[17] / (SMN_MCTR - 1));
  int16_t smn1_z = round(meassum[18] / (SMN_MCTR - 1));
  int16_t smn1_temp = round(meassum[19] / (SMN_MCTR - 1));

  int16_t med_imu_a_x = int(Array_median(arr_imu_a_x, IMU_MCTR));                //Calculate medians
  int16_t med_imu_a_y = int(Array_median(arr_imu_a_y, IMU_MCTR));
  int16_t med_imu_a_z = int(Array_median(arr_imu_a_z, IMU_MCTR));
  int16_t med_imu_g_x = int(Array_median(arr_imu_g_x, IMU_MCTR));
  int16_t med_imu_g_y = int(Array_median(arr_imu_g_y, IMU_MCTR));
  int16_t med_imu_g_z = int(Array_median(arr_imu_g_z, IMU_MCTR));
  int16_t med_imu_m_x = int(Array_median(arr_mag_m_x, IMU_MCTR));
  int16_t med_imu_m_y = int(Array_median(arr_mag_m_y, IMU_MCTR));
  int16_t med_imu_m_z = int(Array_median(arr_mag_m_z, IMU_MCTR));

  SP.println("BREAK: Medians, Averages calculated");
                                                                                 
  int8_t battery = 10 * battv * 3.3 / 4095 * (BATT_R1 + BATT_R2) / (BATT_R1);    // Calculate battvolt - 10 times the battery voltage in volts
                                                                                 
  payload1[0] = payload1size;                                                    // Create byte array to send (see https://www.thethingsnetwork.org/docs/devices/bytes.html for byte encoding examples)
  payload1[1] = highByte(desig);
  payload1[2] = lowByte(desig);
  payload1[3] = highByte(settings_desig);
  payload1[4] = lowByte(settings_desig);
  payload1[5] = battery;
  payload1[6] = temp >> 16;
  payload1[7] = temp >> 8;
  payload1[8] = temp;
  payload1[6] = prsr >> 16;
  payload1[7] = prsr >> 8;
  payload1[8] = prsr;
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
  
  extrapayload1[3] = highByte(imuay);                                               //EXTRA PAYLOAD1
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
  
  sprintf(report, "Batt: %6d; IMU: A: %6d %6d %6d; G: %6d %6d %6d; M: %6d %6d %6d; INCL: %6d %6d %6d; T: %6d; P: %6d",            // Display data on SP port
          battery,
          imuax, imuay, imuaz,
          imugx, imugy, imugz,
          magmx, magmy, magmz,
          inkl_x, inkl_y, inkl_z,
          temp, prsr);

  SP.println(report);
  SP.println();

  if (SET_SMN == true)
  {
  sprintf(report2, "INCL: %6d; TEMP: %6d",
          smn1_x, smn1_temp);
  SP.println(report2);
  SP.println();
  }

  //SP.write(payload,payloadsize);                       // Display raw data on SP port
  //SP.println();


  // ------ SEND DATA VIA LoRa ------ //                 //Reset commands every loop
  String commands[10];                                   // Maximum number of commands per message = 10
  String values[10];                                     // Maximum number of commands per message = 10
  int m = 0;                                             // Command counter
  int err;
  
  //modem.dataRate(5);                                   // Set dataRate manually: Usually deactivated to allow for automatic data rate adaptation depending on signal to Noise Ratio, 5: switch to SF7
  delay(100);                                            // For more Stable Connection
  modem.beginPacket();                                   // modem.beginPacket startet das Packet das mit LoRa versendet wird
         
  if (loopctr < extrafreq){                               // Determining payload to be sent to modem          
    if (desig == 1) {
      SP.println("LoRa Infrastructure Node");
      modem.write(payload1, payload1size);
    }
    /*if (desig == 2){
      SP.println("Subsurface Measurement Node");
      modem.write(payload2, payload2size);
    } */
  }
  err = modem.endPacket(true);                            //When modem.endPacket(true), then confirmed data sent!
  if (err > 0){
    SP.println("Message sent correctly!");
  }
  else{
    SP.println("Error sending message :(");
    SP.println("(you may send a limited amount of messages per minute, depending on the signal strength");
    SP.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
  }


  // ------ CHECK DATA UPLOAD ------  //
  if (!modem.available()){                                
    SP.println("No downlink message received at this time.");
  }
  else{
    char rcv[64];
    int i = 0;
    while (modem.available()){
      rcv[i++] = (char)modem.read();
    }
    SP.print("Message Received: ");                        // PRINT DATA

    /*
    // PRINT AS HEX CHAR by CHAR
    for (unsigned int j = 0; j < i; j++) {
    SP.print(rcv[j] >> 4, HEX);
    SP.print(rcv[j] & 0xF, HEX);
    SP.print(" ");
    }
    */
 
    String RcvStr = "";                                    // PRINT AS TXT
    for (unsigned int j = 0; j < i; j++) {
      RcvStr = RcvStr + rcv[j];
    }
    SP.println(RcvStr);
    SP.println();


    // ------ ANALYSE DATA ------ //
    SP.println("Checking for commands...");                // Commands have following stucture: COMMAND:VALUE;COMMAND:VALUE;COMMAND:VALUE;
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
          SP.println("Command: " + command + ", Value: " + value);
          // SP.println("Remaining String: " + RcvStr);
          commands[m] = command;
          values[m] = value;
          m = m + 1;
        }
        else {
          SP.println("ERROR! Empty command");
        }
      }
      else {
        SP.println("No (more) commands found!");
      }
      SP.println();
    }
  }


  // ------ PARSE COMMANDS ------ //
  for (int i = 0; i < m; i++) {
    SP.println("Processing Command: " + commands[i] + " ,Value: " + values[i]);
    if (commands[i] == "MInt") {                                                     // COMMAND MInt
      if ((values[i].toInt() >= 1) && (values[i].toInt() <= 31536000)) {
        loopintv = (values[i].toInt() * 1000);
        SP.println("Measurement interval has been set to " + values[i] + " s");
      }
      else {
        SP.println("Error! Wrong measurement interval value: " + values[i]);
      }
    }
    else {                                                                           // COMMAND Not Recognized
      SP.println("Command was not recognized!");
    }
    SP.println();
  }


  // ------ END of LOOP: Set Arduino to Low Power State ------ //
  delay(100);
  looptime = millis() - loopstart;
  SP.write("Loop duration: ");
  SP.print(looptime);
  SP.println(" ms");
  sleeptime = loopintv - looptime;
  SP.write("Sleep time: ");
  SP.print(sleeptime);
  SP.println(" ms");
  if (sleeptime < 10000) {
    sleeptime = 10000;
    SP.println("Warning! Sleeptime is too low - was set to 10000 ms");
  }

  
  // ------ SET DIGITAL PORTS FOR SLEEP ------ //
  SP.println("Setting digital ports for sleep:");
  for (int i = 0; i <= 7; i++) {
    SP.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == IMU_DPORT || i == StpCtr1_DPORT) {
      pinMode(i, INPUT);
      SP.println(": Input");
    }
    else {
      SP.println(": not changed");
    }
  }

  SP.println("Sleeping..");
  //delay(sleeptime);
  LowPower.deepSleep(sleeptime);

  // ------ SET DIGITAL PORTS FOR WAKEUP ------ //
  SP.println("Setting digital ports for wakeup:");
  for (int i = 0; i <= 7; i++) {
    SP.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == IMU_DPORT || i == StpCtr1_DPORT) {
      pinMode(i, OUTPUT);
      SP.println(": Output");
    }
    else {
      SP.println(": not changed");
    }
  }

}

void K1_AllOff () {             // turn on pullup resistors
  digitalWrite(A2, LOW);        
  digitalWrite(A3, LOW);
}

void K1_TurnA () {              // turn on pullup resistors
  digitalWrite(A2, HIGH);       
  digitalWrite(A3, LOW);      
}

void K1_TurnB () {              // turn on pullup resistors
  digitalWrite(A2, LOW);        
  digitalWrite(A3, HIGH);      
}

void K2_AllOff () {             // turn on pullup resistors
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
}

void K2_TurnA () {              // turn on pullup resistors
  digitalWrite(A4, HIGH);       
  digitalWrite(A5, LOW);       
}

void K2_TurnB () {              // turn on pullup resistors
  digitalWrite(A4, LOW);       
  digitalWrite(A5, HIGH);       
}
