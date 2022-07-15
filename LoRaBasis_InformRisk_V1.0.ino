/*  Inform@Risk LoRa Node
    (c) AlpGeorisk & TUM
    Authors: Ashwanth Ramesh, Moritz Gamperl & John Singer

    LoRaBasis_InformRisk
    Version: 1.0
    Date: 26.03.2022

    Supported Hardware:
    BOARDS:
    - Arduino MKR WAN 1310

    SENSORS:
    - ICM 20948
    - ADS1220 ADC
    - BMP388
*/

#include "Uni_Config.h"

void setup(){

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// ------ VARIABLE DEFINTION DURING RUNTIME ------ //

  SP.println("Starting Arduino...., Inside Setup");

  SP.begin(115200);                                        // OPEN Serial PORT -- used for development purposes
  while (!SP && millis() < 5000);                        // Wait 5 secs for Serial connection otherwise continue anyway...
  Wire.begin();

  if (SET_SMN == false) I1_PORT = -1;
  if (SET_SMN == false) I2_PORT = -1;


// ----- SETUP DIGITAL PINS ------ // : Set measurement pins to output, set unused pins to input_pullup

  SP.println("Setting up Digital ports:");
  for (int i = 0; i <= 7; i++) {
    SP.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == IMU_DPORT || i == I1_PORT)         
    {        
      pinMode(i, OUTPUT);
      SP.print(": Output");
      SP.print(" \t");
    }
    else 
    {
      if (i == LED_BUILTIN)                                         // SET LED port to output low, all other unused ports to INPUT_PULLUP to save energy
      {                    
        pinMode(i, INPUT);
        SP.print(": Input (LED Off)");
        SP.print(" \t");
      }
      else 
      {
        pinMode(i, INPUT_PULLUP);
        SP.print(": INPUT_PULLUP");
        SP.print(" \t");
      }
    }
  }
  
  SP.println("Digital Port Setup Successful");

// ------ RELAIS SETTINGS ------ //

  pinMode(A2, OUTPUT);                                               // Relais K1
  pinMode(A3, OUTPUT);
  K1_AllOff ();

  pinMode(A4, OUTPUT);                                               // Relais K2
  pinMode(A5, OUTPUT);
  K2_AllOff ();

  delay(100);

  if (ADS_C1 == 2) 
  {
    SP.println("Turning on Relais for Channel 1");
    K1_TurnA ();               // turn on pullup resistors
  }
  if (ADS_C2 == 2) 
  {
    SP.println("Turning on Relais for Channel 2");
    K2_TurnA ();
  }

  analogReadResolution(12);                                          // SETUP ANALOG INPUT

  SP.println("Relay Setup Successful");

  if(SET_SMN == true) desig = 2;
  if(SET_LCI == true) desig = 3;
  

  delay(sensorstarttime);                                            // wait for sensors to startup
  
  // ------ CONNECT TO LORA NETWORK ------ //

  SP.println("Connecting to the Lora Network");
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

  // Print default channels configuration
  /*Serial.print("- Default mask: ");
  Serial.println(modem.getChannelMask());

  Serial.println("- Disabling all channels...");  
  for (unsigned int i = 0; i < 72; i++) {
  modem.disableChannel(i);
  }

  // Print current channels configuration
  Serial.print("- Current mask: ");
  Serial.println(modem.getChannelMask());

  // Enable US915-928 channels
  // LoRaWAN® Regional Parameters and TTN specification: channels 8 to 15 plus 65 
  Serial.println("- Enabling channels 8 to 15 plus 65...");  
  for (unsigned int i = 8; i <= 15; i++) {
    modem.enableChannel(i);
  }
  modem.enableChannel(65);

   // Print current channels configuration
  Serial.print("- Current mask: ");
  Serial.println(modem.getChannelMask());*/

  connection:
  Serial.println("- Joining Server - This may take a while... Timeout @ 150 sec");
  Connected = modem.joinOTAA(appEui, appKey, "", 150000);        // Attempt to Join LoRaWAN Network using Over-The-Air-Activation (OTAA)
  int x = modem.connected();
  SP.println(x);
  delay(100);
  if (Connected)
  {
    // ----- Setup Signal strength ------
    //      change SF by choosing data rate (bool dataRate(uint8_t dr)), can be between 0 (SF 12) and 6 (SF 7)
    //      ADR=Adaptive Data Rate allows modulation of SW and BW depending on signal strength from gateway. Is on by default
    //      https://github.com/arduino/mkrwan1300-fw/issues/3
    //try: modem.setADR(true);
    //modem.dataRate(5);

    SP.println("Connection is Successful");
    modem.minPollInterval(10);     // Set poll interval to x seconds
    delay(100);                    // For more Stable Connection
  }
  else
  {
    while(1)
    {
    SP.println("Something went wrong; Are you indoors? Move near a Window and Retry");
    delay(1);
    networktry = networktry + 1;
    if(networktry>1) break;
    goto connection;
    }
  }
  
  if(!Connected)
  {
    SP.println("Connection failed after three tries");
  }

  SP.println("Setup Concluded....");
}


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{

// ------ CHECKING NETWORK CONNECTION ------ //
  SP.println("Inside Loop....");
  SP.println("Checking Network Connection....");
  int x = modem.connected();
  if(x == 0)
  {
    restart:    
    for (int i = 0; i <=1; i++)
    {
      Connected = modem.joinOTAA(appEui, appKey, "", 150000);        // Attempt to Join LoRaWAN Network using Over-The-Air-Activation (OTAA)
      if(Connected == 0)
      {
        SP.println("Something went wrong; Are you indoors? Move near a Window and Retry");
        delay(1);
      }
      else 
      {
        SP.println("Connected Successfully");
        continue;
      }
    }
    if(Connected == 0)
    {
      SP.println("Connection failed after two tries, going to Deep Sleep Mode");
      DeepSleepMode();
      Wire.begin();
      goto restart;
    }
  
  }
  
  else SP.println("Connection Stable");
  
// ------ PAYLOAD DECLARATIONS ------ //

  short datasize = 6;

  if(SET_BARO == true){datasize += 4;}                       // If loop for barometer
  if(SET_SCL == true){datasize += 11;}                        // If loops to estimate payload size - implemented only for the 3 sensors discussed
  if(SET_ADC == true){datasize += 3;}
  if(SET_SMN == true){datasize += 8;}
  if(SET_LCI == true){datasize += 24;}                          
  
  long meassum[datasize];                                    // Array with summerized measurements // Achtung! Länge = max i + 1, da indizierung mit 0 losgeht
  byte payload1[datasize];

  SP.print("Payload Size: ");
  SP.println(datasize);


  loopctr++;                                                 // Loop Counter

  loopstart = millis();                                      //MEASUREMENT LOOP
  SP.println();
  SP.print("New Cycle for New Measurement...., Starttime: ");
  SP.println(loopstart);
  //Wire.begin();

// ------- ACTIVATE SENSORS ------- //

  SP.println("Activating Sensors....");
  if (BATT_DPORT >= 0) digitalWrite(BATT_DPORT, HIGH);
  if (LED_DPORT >= 0) digitalWrite(LED_DPORT, HIGH);
  if (SW33_A_DPORT >= 0) digitalWrite(SW33_A_DPORT, HIGH);
  if (SW33_B_DPORT >= 0) digitalWrite(SW33_B_DPORT, HIGH);
  if (SW12_DPORT >= 0) digitalWrite(SW12_DPORT, HIGH);
  if (IMU_DPORT >= 0) digitalWrite(IMU_DPORT, HIGH);                      // IMU_DPORT HERE
  if (I1_PORT >= 0) digitalWrite(I1_PORT, HIGH);


  delay(sensorstarttime);                                // Wait for Sensors to Startup
  SP.println("BREAK: sensors should be started");


// ------ INITIALIZE SENSORS AFTER POWERUP ------ //

  if (SET_IMU == HIGH )                                   // IMU INITIALIZE
  {                                 
    if(!myIMU.init()) {SP.println("ICM20948 does not respond");}
    else {SP.println("ICM20948 is connected");}
    
    if(!myIMU.initMagnetometer()) {SP.println("Magnetometer does not respond");}
    else {SP.println("Magnetometer is connected");}

    myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
    myIMU.setAccDLPF(ICM20948_DLPF_6);
    myIMU.setAccSampleRateDivider(10);
    myIMU.setGyrDLPF(ICM20948_DLPF_6);
    myIMU.setGyrSampleRateDivider(10);
    myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
  }

  // Barometer Initialization
  if (SET_BARO == true)
  {
    SP.println("Barometer is setup");
    bmp388.begin();                                            //  BMP INITIALIZE   // Default initialisation, place the BMP388 into SLEEP_MODE
    bmp388.setTimeStandby(TIME_STANDBY_1280MS);                                     // Set the standby time to 1.3 seconds
    bmp388.startNormalConversion();                                                 // Start BMP388 continuous conversion in NORMAL_MODE
  }

  if (SET_SMN == true)                                         // SMN INITIALIZE
  {                                        
    if(I1A == true)
    {
      digitalWrite(I1_PORT, HIGH);
      delay (100);
      if (inkl_a.initialize(BMA4_I2C_ADDR_SECONDARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) 
      {
        SP.println("INKL_A: Successfully connected to BMA456");
        inkl_a_init = 1;
      }
      else 
      {
        SP.println("INKL_A: Connection error - BMA could not be initialized!");
        inkl_a_init = 0;
      }
    }
    else SP.println("I1A not activated");
  }

   if (SET_LCI == true)                                         // SMN INITIALIZE
  {                                        
    if(I1B == true)
    {
      if (inkl_b.initialize(BMA4_I2C_ADDR_PRIMARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) 
      {
        SP.println("INKL_B: Successfully connected to BMA456");
        inkl_b_init = 1;
      }
      else 
      {
        SP.println("INKL_B: Connection error - BMA could not be initialized!");
        inkl_b_init = 0;
      }
    }
    else SP.println("I1B not activated");
  }


  if (SET_SCL == true)
  {
    //pinMode(SCL3300_CS_PIN, INPUT_PULLUP);
    //digitalWrite(SCL3300_CS_PIN, HIGH);
    //uint16_t errors = inclinometer.WakeMeUp();                                 // Turn on Murata
    delay(100);
    SP.println("Inclinometer woken up.");
    if (inclinometer.begin(scl3300_sspin) == false)                              // SETUP MURATA
    {                  
      SP.println("Murata SCL3300 inclinometer not connected.");
      while(1);                                                                  //Freeze
    }
    //inclinometer.setMode(mode);
  }

 /* if (SET_ADC == true){
      SP.println("ADS1220 Initialization...");
      if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {SP.println("ADS1220 initialized successfully!");}
      else {SP.println(("ADS1220 was NOT initialized successfully!"));}
      SP.println();
      SP.println("ADS1220 Calibration (this may take a few seconds)...");
      ads1220.calibrate();
      SP.println();
  } */

  // ------ INITIALIZE ADS1220 ------ //

  if (SET_ADC)
  {
  SP.println("ADS1220 Initialization...");
  if (ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN)) {SP.println("ADS1220 initialized successfully!");}
  else 
  {
    ads1220.powerdown();
    SP.println("ADS1220 was NOT initialized successfully!");
  }
  SP.println();

// ------ CALIBRATE ADS1220 ------ //
  if (ADC_CAL == true)
  {
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
  //ads1220.config_idac2(IDAC2_AIN0);
  //delay(200);                                          //Delay to powerup IDAC - removed because of sensorstarttime
  //GET INITIAL VALUES
  //int32_t lpotdata = ads1220.read_single_shot();
  //lpotvoltage = ads1220.dac2mv(lpotdata);

  //else ads1220.powerdown();
  }

  // ------ MEASUREMENTS ------ //
  measstart = millis();                                                 //Reset Timing & Counters
  BATT_MCTR = 1;
  IMU_MCTR = 1;
  MAG_MCTR = 1;
  BARO_MCTR = 1;
  BARO_VCTR = 1;
  SMN_MCTR = 1;
  INKL_MCTR = 1;
  INKL_VCTR = 1;
  SMN_VCTR = 1;
  ADC_MCTR = 1;
  ADC_VCTR = 1;
  SP.println("BREAK: Measurement started...., counters set to 1 \n");

  for (int i = 0; i <= datasize; i++){
    meassum[i] = 0;                                                    //Reset Measurent Variables
  }

  while (millis() - measstart <= measlength)
  {
    meastime = millis() - measstart;
    if (BATT_DPORT >= 0 && meastime >= BATT_MINT * BATT_MCTR) {        // BATT VOLTAGE MEASUREMENT
      BATT_MCTR++;
      int batt = analogRead(BATT_APORTin);                             // Read Analog Port
      meassum[0] = meassum[0] + batt;
      SP.print("BREAK: Battery voltage finished: ");
      SP.println(batt);
    }

    if (IMU_DPORT >= 0)
    {
      if (SET_BARO == true && meastime >= BARO_MINT * BARO_MCTR)       // BARO MEASUREMENT
      {                         
        BARO_MCTR++;
        bmp388.startForcedConversion();                 // Start a forced conversion (if in SLEEP_MODE)
        if (bmp388.getMeasurements(temperature, pressure, altitude))   // Check if the measurement is complete
        {
          SP.println(report);
          SP.print(temperature);                                      // Display the results
          SP.print(F("*C   "));
          SP.print(pressure);
          SP.print(F("hPa   "));
          SP.print(altitude);
          SP.println(F("m"));
          meassum[1] = meassum[1] + round(temperature*100);
          meassum[2] = meassum[2] + round(pressure*10);            // Pa div. 100 equ. hPa = mBar
          BARO_VCTR++;
        }
      }
    }

    if (SET_SCL == true && meastime >= INKL_MINT * INKL_MCTR)
    {            // SCL Inclinometer Measurement
      INKL_MCTR++;
      if (inclinometer.begin(scl3300_sspin) == false) 
      {
        SP.println("Murata SCL3300 inclinometer not connected.");
        while(1);                                                      //Freeze
      }
      delay(10);
      //inclinometer.setMode(mode);
      if (inclinometer.available())
      {
        meassum[3] = meassum[3] + round((inclinometer.getTiltLevelOffsetAngleX()*1000));
        meassum[4] = meassum[4] + round((inclinometer.getTiltLevelOffsetAngleY()*1000));
        meassum[5] = meassum[5] + round((inclinometer.getTiltLevelOffsetAngleZ()*1000));
        INKL_VCTR++;
      }
      else 
      {
        inclinometer.reset();
        SP.println("Inclinometer not working");
        delay(10);
      }
    }

    if (SET_SMN == true && meastime >= SMN_MINT * SMN_MCTR)         // SMN MEASUREMENT (for only one SMN)
    {               
      SMN_MCTR++;
      if (inkl_a_init) 
      {
        inkl_a.getAcceleration(&x_a, &y_a, &z_a);
        temp_a = inkl_a.getTemperature();
        meassum[6] = meassum[6] + (x_a*100);                        // Adjust according to precision of BMA Sensor
        meassum[7] = meassum[7] + (y_a*100);
        meassum[8] = meassum[8] + (z_a*100);
        meassum[9] = meassum[9] + temp_a;
        SMN_VCTR++;
      }
      if (inkl_b_init)
      {
        inkl_b.getAcceleration(&x_b, &y_b, &z_b);
        temp_b = inkl_b.getTemperature();
        meassum[12] = meassum[12] + (x_b*100);                        // Adjust according to precision of BMA Sensor
        meassum[13] = meassum[13] + (y_b*100);
        meassum[14] = meassum[14] + (z_b*100);
        meassum[15] = meassum[15] + temp_b;
      }
    }

     while(SET_ADC == true && meastime >= ADC_MINT * ADC_MCTR)              // ADC MEASUREMENT
  {                                                 
    ADC_MCTR++;
      if (ADS_C0 == 1)                                        // Channel 0 measurement (Potentiometer):
      {                                      
      ads1220.config_mux(SE_CH0);
      ads1220.config_gain(PGA_GAIN_1);
      ads1220.config_pga(PGA_OFF);
      ads1220.config_datarate(DR_20SPS);
      ads1220.config_opmode(OM_NORMAL);
      ads1220.config_conmode(CM_SINGLE_SHOT);
      ads1220.config_tempmode(TEMP_OFF);
      ads1220.config_burnout(BO_OFF);
      ads1220.config_vref(VREF_AVDD);
      ads1220.config_fir(FIR_50_60);
      delay(100);
      int32_t lpotdata = ads1220.read_single_shot();
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      meassum[10] = meassum[10] + ads1220.dac2mv(lpotdata);
      SP.print(lpotdata);
      SP.println(" C0 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C0 ipotvoltage");
      }

      if (ADS_C0 == 2)                                        // Channel 0 measurement (Potentiometer 4-20mA):
      {
      digitalWrite(SW12_DPORT, HIGH);                                                   // Turn on 12V OUT
      delay(100);                                                 
      ads1220.config_mux(SE_CH0);
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
      delay(100); //Delay to powerup IDAC
      int32_t lpotdata = ads1220.read_single_shot();
      digitalWrite(SW12_DPORT, LOW);                                                   // Turn off 12V OUT
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      float current = lpotvoltage/150;
      float mbar = 800 + (260/16*(current-4));
      meassum[10] = meassum[10] + lpotdata;
      SP.print(lpotdata);
      SP.println(" C0 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C0 ipotvoltage 4-20 mA");
      }

      if (ADS_C1 == 1)                                     // Channel 1 measurement (Potentiometer)
            {                                      
      ads1220.config_mux(SE_CH1);
      ads1220.config_gain(PGA_GAIN_1);
      ads1220.config_pga(PGA_OFF);
      ads1220.config_datarate(DR_20SPS);
      ads1220.config_opmode(OM_NORMAL);
      ads1220.config_conmode(CM_SINGLE_SHOT);
      ads1220.config_tempmode(TEMP_OFF);
      ads1220.config_burnout(BO_OFF);
      ads1220.config_vref(VREF_AVDD);
      ads1220.config_fir(FIR_50_60);
      delay(100);
      int32_t lpotdata = ads1220.read_single_shot();
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      meassum[24] = meassum[24] + ads1220.dac2mv(lpotdata);
      SP.print(lpotdata);
      SP.println(" C1 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C1 ipotvoltage");
      }

      
     if (ADS_C1 == 2)                                        // Channel 1 measurement (Potentiometer 4-20mA):
      {    
      digitalWrite(SW12_DPORT, HIGH);                                                   // Turn on 12V OUT
      delay(100);                                                                                        
      ads1220.config_mux(SE_CH1);
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
      delay(100); //Delay to powerup IDAC
      int32_t lpotdata = ads1220.read_single_shot();
      digitalWrite(SW12_DPORT, LOW);                                                   // Turn off 12V OUT
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      float current = lpotvoltage/150;
      float mbar = 800 + (260/16*(current-4));
      meassum[24] = meassum[24] + lpotdata;
       SP.print(lpotdata);
      SP.println(" C1 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C1 ipotvoltage 4-20 mA");
      }
      
      if (ADS_C2 == 1)                                    //  Channel 2 measurement (Potentiometer)
            {                                      
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
      delay(100);
      int32_t lpotdata = ads1220.read_single_shot();
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      meassum[25] = meassum[25] + ads1220.dac2mv(lpotdata);
      SP.print(lpotdata);
      SP.println(" C2 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C2 ipotvoltage");
      }

      
     if (ADS_C2 == 2)                                        // Channel 2 measurement (Potentiometer 4-20mA):
      {
        digitalWrite(SW12_DPORT, HIGH);                                                   // Turn on 12V OUT
      delay(100);                                           
      ads1220.config_mux(SE_CH2);
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
      delay(100); //Delay to powerup IDAC
      int32_t lpotdata = ads1220.read_single_shot();
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      digitalWrite(SW12_DPORT, LOW);                                                   // Turn off 12V OUT
      float current = lpotvoltage/150;
      float mbar = 800 + (260/16*(current-4));
      meassum[25] = meassum[25] + lpotdata;
      SP.print(lpotdata);
      SP.println(" C2 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C2 ipotvoltage 4-20 mA");
      }

      if (ADS_C3 == 1)                                     // Channel 3 measurement (Potentiometer)
            {                                      
      ads1220.config_mux(SE_CH3);
      ads1220.config_gain(PGA_GAIN_1);
      ads1220.config_pga(PGA_OFF);
      ads1220.config_datarate(DR_20SPS);
      ads1220.config_opmode(OM_NORMAL);
      ads1220.config_conmode(CM_SINGLE_SHOT);
      ads1220.config_tempmode(TEMP_OFF);
      ads1220.config_burnout(BO_OFF);
      ads1220.config_vref(VREF_AVDD);
      ads1220.config_fir(FIR_50_60);
      delay(100);
      int32_t lpotdata = ads1220.read_single_shot();
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      meassum[26] = meassum[26] + ads1220.dac2mv(lpotdata);
      SP.print(lpotdata);
      SP.println(" C3 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C3 ipotvoltage 4-20 mA");
      }

      
     if (ADS_C3 == 2)                                        // Channel 3 measurement (Potentiometer 4-20mA):
      {
        digitalWrite(SW12_DPORT, HIGH);                                                   // Turn on 12V OUT
      delay(100);                                           
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
      delay(100); //Delay to powerup IDAC
      int32_t lpotdata = ads1220.read_single_shot();
      float lpotvoltage = ads1220.dac2mv(lpotdata);
      digitalWrite(SW12_DPORT, LOW);                                                   // Turn off 12V OUT
      float current = lpotvoltage/150;
      float mbar = 800 + (260/16*(current-4));
      meassum[26] = meassum[26] + lpotdata;
      SP.print(lpotdata);
      SP.println(" C3 ipotdata");
      SP.print(lpotvoltage);
      SP.println(" C3 ipotvoltage 4-20 mA");
      }

      
      
    }
    
    delay(50);                                                          // DELAY after every measurement cycle
  }

 if (SET_SCL == true)                                                   // Turn off Murata
 {                                                 
    //digitalWrite(SCL3300_Power_PIN, LOW);
    digitalWrite(SCL3300_CS_PIN, HIGH);
    inclinometer.powerDownMode();
  }

  if (SET_LCI == true)
  {
    digitalWrite(I2_PORT, LOW);
    delay(100);
     if(I1A == true) // ACTIVATE I1A
     {
      digitalWrite(I1_PORT, HIGH);
      delay (100);
      if (inkl_a.initialize(BMA4_I2C_ADDR_SECONDARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) 
      {
        SP.println("LCI INKL_1A: Successfully connected to BMA456");
        inkl_a_init = 1;
      }
      else 
      {
        SP.println("LCI INKL_A: Connection error - BMA could not be initialized!");
        inkl_a_init = 0;
      }
    }
    else SP.println("LCI I1A not activated");

     if(I1B == true) // ACTIVATE I1B
    {
      digitalWrite(I1_PORT, HIGH);
      delay (100);
      if (inkl_b.initialize(BMA4_I2C_ADDR_PRIMARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) 
      {
        SP.println("LCI INKL_B: Successfully connected to BMA456");
        inkl_b_init = 1;
      }
      else 
      {
        SP.println("LCI INKL_B: Connection error - BMA could not be initialized!");
        inkl_b_init = 0;
      }
    }
    else SP.println("I1B not activated");

        
        for (int i = 0; i < 5; i++) {
    SP.println("LCI: Measuring line 1");           
    if (inkl_a_init) {
      inkl_a.getAcceleration(&x_a, &y_a, &z_a);
      temp_a = inkl_a.getTemperature();
        meassum[16] = meassum[16] + (x_a*100);                        // Adjust according to precision of BMA Sensor
        meassum[17] = meassum[17] + (y_a*100);
        meassum[18] = meassum[18] + (z_a*100);
        meassum[19] = meassum[19] + temp_a;
        }
    if (inkl_b_init) { 
      inkl_b.getAcceleration(&x_b, &y_b, &z_b);
      temp_b = inkl_b.getTemperature();
      meassum[20] = meassum[20] + (x_b*100);                        // Adjust according to precision of BMA Sensor
      meassum[21] = meassum[21] + (y_b*100);
      meassum[22] = meassum[22] + (z_b*100);
      meassum[23] = meassum[23] + temp_b;
      }
    delay(150); // Adapt to measurement frequency selected above
    }

    
    digitalWrite(I1_PORT, LOW);
    delay (100);         
                                   
    if(I2A == true) // ACTIVATE I2A
     {
      digitalWrite(I2_PORT, HIGH);
      delay (100);
      if (inkl_a.initialize(BMA4_I2C_ADDR_SECONDARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) 
      {
        SP.println("INKL_A: Successfully connected to BMA456");
        inkl_a_init = 1;
      }
      else 
      {
        SP.println("INKL_A: Connection error - BMA could not be initialized!");
        inkl_a_init = 0;
      }
    }
    else SP.println("I2A not activated");

    
      if(I2B == true) // ACTIVATE I2B
    {
      digitalWrite(I2_PORT, HIGH);
      delay (100);
      if (inkl_b.initialize(BMA4_I2C_ADDR_PRIMARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) 
      {
        SP.println("INKL_B: Successfully connected to BMA456");
        inkl_b_init = 1;
      }
      else 
      {
        SP.println("INKL_B: Connection error - BMA could not be initialized!");
        inkl_b_init = 0;
      }
    }
    else SP.println("I2B not activated");

    // Line 2 LCI MEASUREMENTS
    
        for (int i = 0; i < 5; i++) {
        SP.println("LCI: Measuring line 2");           

    if (inkl_a_init) {
      inkl_a.getAcceleration(&x_a, &y_a, &z_a);
      temp_a = inkl_a.getTemperature();
        meassum[16] = meassum[16] + (x_a*100);                        // Adjust according to precision of BMA Sensor
        meassum[17] = meassum[17] + (y_a*100);
        meassum[18] = meassum[18] + (z_a*100);
        meassum[19] = meassum[19] + temp_a;
        }
    if (inkl_b_init) { 
      inkl_b.getAcceleration(&x_b, &y_b, &z_b);
      temp_b = inkl_b.getTemperature();
      meassum[20] = meassum[20] + (x_b*100);                        // Adjust according to precision of BMA Sensor
      meassum[21] = meassum[21] + (y_b*100);
      meassum[22] = meassum[22] + (z_b*100);
      meassum[23] = meassum[23] + temp_b;
      }
    delay(150); // Adapt to measurement frequency selected above
    }
      }

 
  

  // ------ MEASUREMENT END AND DEACTIVATING SENSORS ------ //

  SP.println("MEASUREMENT END");
  if (BATT_DPORT >= 0) digitalWrite(BATT_DPORT, LOW);
  if (LED_DPORT >= 0) digitalWrite(LED_DPORT, LOW);
  if (SW33_A_DPORT >= 0) digitalWrite(SW33_A_DPORT, LOW);
  if (SW33_B_DPORT >= 0) digitalWrite(SW33_B_DPORT, LOW);
  if (SW12_DPORT >= 0) digitalWrite(SW12_DPORT, LOW);
  //if (SET_ADC) ads1220.powerdown();
  if (IMU_DPORT >= 0) digitalWrite(IMU_DPORT, LOW);

   if (SET_SMN == true)
   {
    SP.println("Deactivating SMN Sensors...");
    if(I1A == true)
    {
     // inkl_a.accel_enable(AKM_POWER_DOWN_MODE);
    }
    //inkl_b.accel_enable(AKM_POWER_DOWN_MODE);
    if (I1_PORT >= 0) {digitalWrite(I1_PORT, LOW);}
  }
  if (SET_LCI == true)
  {
    SP.println("Deactivating LCI...");
    if(I2A == true)
    {
      inkl_a.accel_enable(AKM_POWER_DOWN_MODE);
    }
    //inkl_b.accel_enable(AKM_POWER_DOWN_MODE);
    if (I2_PORT >= 0) {digitalWrite(I1_PORT, LOW);}
  }


  SP.print("Measurement duration inkl. initialization (sensors on): ");         //Display measurement duration
  SP.println(millis() - loopstart);

  int16_t battv = round(meassum[0] / (BATT_MCTR - 1));                          //Calculate averages
  SP.print("Meassum Batt: ");         //Display measurement duration
  SP.println(meassum[0]);
  SP.print("Battv: ");         //Display measurement duration
  SP.println(battv);

  if (battv > 2207)
  {
    battery = round(10*((0.0008*battv*battv) - (3.3705*battv) + 3678.6));
    sprintf(report, "Inside first one %6d",
          battery);
    SP.println(report);
  }
  else
  {
    battery = round(10*((0.0033*battv) + 0.2608));
    sprintf(report, "Battery voltage high %6d",
          battery);
    SP.println(report);
  }

  sprintf(report, "Battery Value: %d",
          battery);
  SP.println(report);

  payload1[0] = datasize;                                                    // Create byte array to send (see https://www.thethingsnetwork.org/docs/devices/bytes.html for byte encoding examples)
  payload1[1] = highByte(desig);
  payload1[2] = lowByte(desig);
  payload1[3] = highByte(settings_desig);
  payload1[4] = lowByte(settings_desig);
  payload1[5] = battery;

  int i = 5;

  if (SET_BARO == true)
    {
    payload1[i+1] = highByte((int16_t)(round(meassum[1] / (BARO_VCTR - 1))));            //Temperature
    payload1[i+2] = lowByte((int16_t)(round(meassum[1] / (BARO_VCTR - 1))));
    payload1[i+3] = highByte((int16_t)(round(meassum[2] / (BARO_VCTR - 1))));            //Pressure
    payload1[i+4] = lowByte((int16_t)(round(meassum[2] / (BARO_VCTR - 1))));
    i = i+4;
    }

  if(SET_SCL == true)
    {
    payload1[i+3] = (int32_t)(round(meassum[3] / (INKL_VCTR - 1)));
    payload1[i+2] = (((int32_t)(round(meassum[3] / (INKL_VCTR - 1))))>>8);
    payload1[i+1] = (((int32_t)(round(meassum[3] / (INKL_VCTR - 1))))>>16);
    payload1[i+6] = (int32_t)(round(meassum[4] / (INKL_VCTR - 1)));
    payload1[i+5] = (((int32_t)(round(meassum[4] / (INKL_VCTR - 1))))>>8);
    payload1[i+4] = (((int32_t)(round(meassum[4] / (INKL_VCTR - 1))))>>16);
    payload1[i+9] = (int32_t)(round(meassum[5] / (INKL_VCTR - 1)));
    payload1[i+8] = (((int32_t)(round(meassum[5] / (INKL_VCTR - 1))))>>8);
    payload1[i+7] = (((int32_t)(round(meassum[5] / (INKL_VCTR - 1))))>>16);
    i = i+11;     // For now 11 because of missing temperature measurement
    }

  if(SET_ADC == true)
    {
    payload1[i+3] = (int32_t)(round(meassum[10] / (ADC_VCTR - 1)));
    payload1[i+2] = (((int32_t)(round(meassum[10] / (ADC_VCTR - 1))))>>8);
    payload1[i+1] = (((int32_t)(round(meassum[10] / (ADC_VCTR - 1))))>>16);
    i = i+3;
    }

  if(SET_ADC == true && SET_SMN == false)
    {
    payload1[i+3] = (int32_t)(round(meassum[24] / (ADC_VCTR - 1)));
    payload1[i+2] = (((int32_t)(round(meassum[24] / (ADC_VCTR - 1))))>>8);
    payload1[i+1] = (((int32_t)(round(meassum[24] / (ADC_VCTR - 1))))>>16);
    payload1[i+6] = (int32_t)(round(meassum[25] / (ADC_VCTR - 1)));
    payload1[i+5] = (((int32_t)(round(meassum[25] / (ADC_VCTR - 1))))>>8);
    payload1[i+4] = (((int32_t)(round(meassum[25] / (ADC_VCTR - 1))))>>16);
    payload1[i+9] = (int32_t)(round(meassum[26] / (ADC_VCTR - 1)));
    payload1[i+8] = (((int32_t)(round(meassum[26] / (ADC_VCTR - 1))))>>8);
    payload1[i+7] = (((int32_t)(round(meassum[26] / (ADC_VCTR - 1))))>>16);
    i = i+9;    
    }
    
  if(SET_SMN == true)
    {
    payload1[i+1] = highByte((int16_t)(round(meassum[6] / (SMN_VCTR - 1))));             //smn1_x
    payload1[i+2] = lowByte((int16_t)(round(meassum[6] / (SMN_VCTR - 1))));
    payload1[i+3] = highByte((int16_t)(round(meassum[7] / (SMN_VCTR - 1))));             //smn1_y
    payload1[i+4] = lowByte((int16_t)(round(meassum[7] / (SMN_VCTR - 1))));
    payload1[i+5] = highByte((int16_t)(round(meassum[8] / (SMN_VCTR - 1))));             //smn1_z
    payload1[i+6] = lowByte((int16_t)(round(meassum[8] / (SMN_VCTR - 1))));
    payload1[i+7] = highByte((int16_t)(round(meassum[9] / (SMN_VCTR - 1))));             //smn1_temp
    payload1[i+8] = lowByte((int16_t)(round(meassum[9] / (SMN_VCTR - 1))));
    i = i+8;
    }

  if(SET_LCI == true)
    {
    payload1[i+1] = highByte((int16_t)(round(meassum[12] / (SMN_VCTR - 1))));             //I1B_x
    payload1[i+2] = lowByte((int16_t)(round(meassum[12] / (SMN_VCTR - 1))));
    payload1[i+3] = highByte((int16_t)(round(meassum[13] / (SMN_VCTR - 1))));             //I1B_y
    payload1[i+4] = lowByte((int16_t)(round(meassum[13] / (SMN_VCTR - 1))));
    payload1[i+5] = highByte((int16_t)(round(meassum[14] / (SMN_VCTR - 1))));             //I1B_z
    payload1[i+6] = lowByte((int16_t)(round(meassum[14] / (SMN_VCTR - 1))));
    payload1[i+7] = highByte((int16_t)(round(meassum[15] / (SMN_VCTR - 1))));             //I1B_temp
    payload1[i+8] = lowByte((int16_t)(round(meassum[15] / (SMN_VCTR - 1))));

    payload1[i+9] = highByte((int16_t)(round(meassum[16] / (5))));             //I2A_x
    payload1[i+10] = lowByte((int16_t)(round(meassum[16] / (5))));
    payload1[i+11] = highByte((int16_t)(round(meassum[17] / (5))));             //I2A_y
    payload1[i+12] = lowByte((int16_t)(round(meassum[17] / (5))));
    payload1[i+13] = highByte((int16_t)(round(meassum[18] / (5))));             //I2A_z
    payload1[i+14] = lowByte((int16_t)(round(meassum[18] / (5))));
    payload1[i+15] = highByte((int16_t)(round(meassum[19] / (5))));             //I2A_temp
    payload1[i+16] = lowByte((int16_t)(round(meassum[19] / (5))));

    payload1[i+17] = highByte((int16_t)(round(meassum[20] / (5))));             //I2B_x
    payload1[i+18] = lowByte((int16_t)(round(meassum[20] / (5))));
    payload1[i+19] = highByte((int16_t)(round(meassum[21] / (5))));             //I2B_y
    payload1[i+20] = lowByte((int16_t)(round(meassum[21] / (5))));
    payload1[i+21] = highByte((int16_t)(round(meassum[22] / (5))));             //I2B_z
    payload1[i+22] = lowByte((int16_t)(round(meassum[22] / (5))));
    payload1[i+23] = highByte((int16_t)(round(meassum[23] / (5))));             //I2B_temp
    payload1[i+24] = lowByte((int16_t)(round(meassum[23] / (5))));

    i = i+24;
    }    

  long temp = (round(meassum[1] / (BARO_VCTR - 1)));
  sprintf(report, "Average Temperature %6d", temp);
  SP.println(report);
  sprintf(report, "Temperature Highbyte %x Lowbyte %x ",
          payload1[6], payload1[7]);
  SP.println(report);

  long pressure = (round(meassum[2] / (BARO_VCTR - 1)));
  sprintf(report, "Average Pressure %6d", pressure);
  SP.println(report);
  sprintf(report, "Pressure Highbyte %x Lowbyte %x ",
          payload1[8], payload1[9]);
  SP.println(report);

  long inkx = (round(meassum[3] / (INKL_VCTR - 1)));
  long inky = (round(meassum[4] / (INKL_VCTR - 1)));
  long inkz = (round(meassum[5] / (INKL_VCTR - 1)));

  sprintf(report, "Average Inclination X: %6d, Y: %6d, Z: %6d",
          inkx, inky, inkz);
  SP.println(report);
  
  sprintf(report, "Inclination X Highbyte %x Highbyte %x Lowbyte %x ",
          payload1[10], payload1[11], payload1[12]);
  SP.println(report);
  sprintf(report, "Inclination Y Highbyte %x Highbyte %x Lowbyte %x ",
          payload1[13], payload1[14], payload1[15]);
  SP.println(report);
  sprintf(report, "Inclination Z Highbyte %x Highbyte %x Lowbyte %x ",
          payload1[16], payload1[17], payload1[18]);
  SP.println(report);

  if (SET_SMN == true)
  {
  long SMNx = (round(meassum[6] / (SMN_VCTR - 1)));
  long SMNy = (round(meassum[7] / (SMN_VCTR - 1)));
  long SMNz = (round(meassum[8] / (SMN_VCTR - 1)));
  long SMNt = (round(meassum[9] / (SMN_VCTR - 1)));
  sprintf(report, "Average SMN Inclination X: %6d, Y: %6d, Z: %6d, Temp: %6d",
          SMNx, SMNy, SMNz, SMNt);
  SP.println(report);
  }
  
  sprintf(report, "SMN X Highbyte %x Lowbyte %x ",
          payload1[22], payload1[23]);
  SP.println(report);
    sprintf(report, "SMN Y Highbyte %x Lowbyte %x ",
          payload1[24], payload1[25]);
  SP.println(report);
  sprintf(report, "SMN Z Highbyte %x Lowbyte %x ",
          payload1[26], payload1[27]);
  SP.println(report);
  sprintf(report, "SMN TEMP Highbyte %x Lowbyte %x ",
          payload1[28], payload1[29]);
  SP.println(report);

  long lpotd = (round(meassum[10] / (ADC_VCTR - 1)));
  sprintf(report, "Average Lpotdata %6d", lpotd);
  SP.println(report);
  sprintf(report, "lpotdata Highbyte %x Highbyte %x Lowbyte %x ",
          payload1[19], payload1[20], payload1[21]);
  SP.println(report);
 

  SP.println();

  // ------ SEND DATA VIA LoRa ------ //                 //Reset commands every loop
  String commands[10];                                   // Maximum number of commands per message = 10
  String values[10];                                     // Maximum number of commands per message = 10
  int m = 0;                                             // Command counter
  int err;

  //modem.dataRate(5);                                   // Set dataRate manually: Usually deactivated to allow for automatic data rate adaptation depending on signal to Noise Ratio, 5: switch to SF7
  delay(100);                                            // For more Stable Connection
  modem.beginPacket();                                   // modem.beginPacket startet das Packet das mit LoRa versendet wird

  if (desig == 1) {
    SP.println("LoRa Infrastructure Node");
  }

  modem.write(payload1, datasize);
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
  DeepSleepMode();
  
  // ------ SET DIGITAL PORTS FOR WAKEUP ------ //
  
  SensorWakeUp();
  
}
