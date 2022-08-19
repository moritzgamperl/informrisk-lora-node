/*  Inform@Risk LoRa Node
    (c) AlpGeorisk & TUM
    Authors: Ashwanth Ramesh, Moritz Gamperl & John Singer

    LoRaBasis_InformRisk
    Version: 1.2
    Date: 13.08.2022

    Supported Hardware:
    BOARDS:
    - Arduino MKR WAN 1310

    SENSORS:
    - ICM 20948
    - ADS1220 ADC
    - BMP388
    - BMA456
*/

#include "setup.h"
#include "00_general_config.h"
#include "BATV_config.h"
#include "BARO_config.h"
#include "IMU_config.h"
#include "AD24_config.h"
#include "AD12_config.h"
#include "INCL_config.h"
#include "SMP_config.h"
#include "extra_functions.h"

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() 
{
  // -- SERIAL PORT FOR STATUS OUTPUT
  SP.begin(115200);                                      // OPEN Serial PORT -- used for development purposes
  while (!SP && millis() < 10000);                       // Wait 10 secs for Serial connection otherwise continue anyway...
  delay(3000);                                           // Wait  3 secs after Serial connection        
  SP.println("INFORM@RISK MEASUREMENT NODE");
  SP.println("INITIALIZING...");
  SP.println("");
  
  // -- SETUP PORTS
  PortsActivate();
  SP.println("");
  
  // -- TEST BATV SENSOR
  if (SET_BATV) {
    SP.println("BATV\tBATTERY VOLTAGE SENSOR: ON");
    BATV_Init();
    BATV_Measure();
    BATV_Finalize();
  }
  else SP.println("BATV\tBATTERY VOLTAGE SENSOR: OFF");
  SP.println("");
  
  // -- TEST BARO SENSOR
  if (SET_BARO) {
    SP.println("BARO\tBAROMETER & THERMOMETER: ON");
    BARO_Init();
    BARO_Setup();
    BARO_Measure();
    BARO_Finalize();
  }    
  else SP.println("BARO\tBAROMETER & THERMOMETER: OFF");
  SP.println("");

  // -- TEST IMU SENSOR
  if (SET_IMU) {
    SP.println("IMU\tINERTIAL MEASUREMENT UNIT (IMU): ON");
    IMU_Init();
    IMU_Setup();
    IMU_Measure();
    IMU_Finalize();    
  }
  else SP.println("IMU\tINERTIAL MEASUREMENT UNIT (IMU): OFF");
  SP.println("");

  // -- TEST INCL SENSOR
  if (SET_INCL) {
    SP.println("INCL\tINCLINOMETER (INCL): ON");
    INCL_Init();
    INCL_Setup();
    INCL_Measure();
    INCL_Finalize();    
  }
  else SP.println("IMU\tINCLINOMETER (INCL): OFF");
  SP.println("");

  // -- TEST AD24 SENSOR
  if (SET_AD24) {
    SP.println("AD24\t24 BIT ANALOG TO DIGITAL CONVERTER (AD24): ON");
    AD24_Init();
    AD24_Setup();
    SW12V_ON();
    SW3V3_ON();
    delay(sensorstarttime);
    AD24_Measure();
    AD24_Finalize();
    SW12V_OFF();
    SW3V3_OFF();
  }
  else SP.println("AD24\t24 BIT ANALOG TO DIGITAL CONVERTER (AD24): OFF");
  SP.println("");  

  // -- TEST AD12 SENSOR
  if (SET_AD12) {
    SP.println("AD12\t12 BIT ANALOG TO DIGITAL CONVERTER (AD12): ON");
    AD12_Init();
    AD12_Setup();
    SW12V_ON();
    SW3V3_ON();
    delay(sensorstarttime);
    AD12_Measure();
    AD12_Finalize();
    SW12V_OFF();
    SW3V3_OFF();
  }
  else SP.println("AD12\t24 BIT ANALOG TO DIGITAL CONVERTER (AD12): OFF");
  SP.println("");

// -- TEST SMP SENSOR
  if (SET_SMP) {
    SP.println("SMP\tSUBSURFACE MEASUREMENT PROBE / LOW COST INCLINOMETER (INCLINATION SENSORS): ON");
    SMP_Init_Measure ();
    SMP_Finalize();    
  }
  else SP.println("IMU\tSUBSURFACE MEASUREMENT PROBE / LOW COST INCLINOMETER (INCLINATION SENSORS): OFF");
  SP.println(""); 

  // -- CONNECT TO LORA NETWORK
  IRloramodemSetup();

  SP.println("INFORM@RISK MEASUREMENT NODE");
  SP.println("SYSTEM\tINITIALIZATION COMPLETE!");
  SP.println("");
  SP.println("");
  SP.println("");
  SP.println("");
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// MEASUREMENT LOOP
// -------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{ 
  // -- START MEASUREMENT CYCLE
  loopctr++;
  loopstart = millis();   
  SP.println("INFORM@RISK MEASUREMENT NODE");
  SP.print("SYSTEM\tSTART OF MEASUREMENT CYCLE #");
  SP.print(loopctr);
  SP.println(" -- Cycle time 0 ms");
  SP.println("SYSTEM\tINITIALIZATION & SETUP");   
  SP.println("");
  
  // -- ACTIVATE PORTS
  PortsActivate();

  // -- CHECKING LORA CONNECTION
  SP.print("LORA\tChecking connection... ");
  if(modem.connected())
  {
    SP.println("OK!");
  }
  else
  {
    SP.println("ERROR! Not connected! Reinitiazing LoRa Connection...");
    IRloramodemSetup();
    if(!modem.connected()) 
    {
    goto MEAS_ABORT;                                           //Skips all measurements and goes directly back to sleep
    }
  } 
  SP.println("");

// -- INITIALIZE SENSORS
  // BATV Initialization
  BATV_Init();
  // BARO Initialization & Setup
  BARO_Init();
  BARO_Setup();
  // IMU Initialization & Setup
  IMU_Init();
  IMU_Setup();
  // INCL Initialization & Setup
  INCL_Init();
  INCL_Setup();
  // AD24 Initialization & Setup
  AD24_Init ();
  AD24_Setup ();
  // AD12 Initialization & Setup
  AD12_Init ();
  AD12_Setup ();     
  // Activate External Power Sources
  SW12V_ON();
  SW3V3_ON();
  delay(sensorstarttime);
  SP.println("");
  
  // -- START MEASUREMENTS
  SP.print("SYSTEM\tMEASUREMENT START");
  looptime = millis() - loopstart;
  SP.print(" -- Cycle time ");  
  SP.print(looptime);
  SP.println(" ms"); 
  measstart = millis();                                                 //Reset Timing & Counters

  while (millis() - measstart <= measlength)
  {
    meastime = millis() - measstart;
    // BATTV MEASUREMENT    
    if (SET_BATV && meastime >= BATV_MINT * MCTR_BATV) BATV_Measure();      
    // BARO MEASUREMENT
    if (SET_BARO && meastime >= (BARO_MINT * MCTR_BARO)) BARO_Measure();
    // IMU MEASUREMENT
    if (SET_IMU && meastime >= (IMU_MINT * MCTR_IMU)) IMU_Measure();
    // INCL MEASUREMENT
    if (SET_INCL && meastime >= (INCL_MINT * MCTR_INCL)) INCL_Measure();            
    // AD24 MEASUREMENTS
    if (SET_AD24 && meastime >= (AD24_MINT * MCTR_AD24)) AD24_Measure();
    // AD12 MEASUREMENTS
    if (SET_AD12 && meastime >= (AD12_MINT * MCTR_AD12)) AD12_Measure();
  }

  // -- MEASUREMENT END AND DEACTIVATING SENSORS
  // Deactivate External Power Sources
  SW12V_OFF();
  SW3V3_OFF();  

  // Deactivate / Shutdown Sensors and Calculate and Report Results  
  SP.print("SYSTEM\tMEASUREMENT ENDED");   
  looptime = millis() - loopstart;
  SP.print(" -- Cycle time ");  
  SP.print(looptime);
  SP.println(" ms");  
  SP.println ("");
  SP.println("MEASUREMENT RESULTS:");
  SP.println("--------------------------------------------------------------------------------------");
  SP.println("SENSOR\tDATASET\t\tRESULT\tDIM\tVALUES#\tMEAS#");
  SP.println("--------------------------------------------------------------------------------------");
  BATV_Finalize();
  BARO_Finalize();
  IMU_Finalize();
  INCL_Finalize();
  AD24_Finalize();
  AD12_Finalize();
  SP.println("--------------------------------------------------------------------------------------");
  SP.println ("");
  
 // -- MEASUREMENT OF SMP
  if (SET_SMP) {
    SP.print("SMP\tSMP MEASUREMENTS START");
    looptime = millis() - loopstart;
    SP.print(" -- Cycle time ");  
    SP.print(looptime);
    SP.println(" ms"); 
    // Perform SMP Measurements
    SMP_Init_Measure ();
    // Deactivate / Shutdown Sensors and Calculate and Report Results  
    SP.print("SMP\tSMP MEASUREMENTS ENDED");
    looptime = millis() - loopstart;
    SP.print(" -- Cycle time ");  
    SP.print(looptime);
    SP.println(" ms");  
    SP.println ("");
    SP.println("SMP MEASUREMENT RESULTS:");
    SP.println("--------------------------------------------------------------------------------------");
    SP.println("SENSOR\tDATASET\t\tRESULT\tDIM\tVALUES#\tMEAS#");
    SP.println("--------------------------------------------------------------------------------------");  
    SMP_Finalize ();
    SP.println("--------------------------------------------------------------------------------------");
    SP.println (""); 
  }  

  // -- PREPARE DATA PACKAGE(S) FOR TRANSMISSION
  SP.print("SYSTEM\tPREPARING DATA PACKAGE FOR TRANSMISSION..");
  // -- CREATE VALUE DESIGNATOR FOR PACKET_DESIGNATOR -- TYPE 10
  // Defines content of Lora package. Each bit stands for a defined value - if TRUE has been transmitted, if FALSE hat not been transmitted
  //        | BYTE 0          Type    | BYTE 1          Type    | BYTE 2          Type    | BYTE 3          Type    | Power of 2 value associated with position
  // --------------------------------------------------------------------------------------------------------------------------------------
  // BIT 0  | BATV            U8      | AD24-CH0        I24     | SMP-I1B          I16 x3 | SMP-I3B          I16 x3 | 1
  // BIT 1  | BARO-T-B        I16,U24 | AD24-CH1        I24     | SMP-I1B_TEMP     I8     | SMP-I3A_TEMP     I8     | 2
  // BIT 2  | BARO-ALT        U16     | AD24-CH2        I24     | SMP-I2A          I16 x3 | NOT USED                | 4
  // BIT 3  | IMU-ACC-XYZ     I16 x3  | AD24-CH3        I24     | SMP-I2A_TEMP     I8     | NOT USED                | 6 
  // BIT 4  | IMU-GYR-XYZ     I16 x3  | AD12-CH0        I16     | SMP-I2B          I16 x3 | NOT USED                | 16
  // BIT 5  | IMU-MAG-XYZ     I16 x3  | AD12-CH1        I16     | SMP-I2B_TEMP     I8     | NOT USED                | 32
  // BIT 6  | INCL-XYZ        I24 x3  | SMP-I1A         I16 x3  | SMP-I3A          I16 x3 | NOT USED                | 64 
  // BIT 7  | INCL-TEMP       I16     | SMP-I1A_TEMP    I8      | SMP-I3A_TEMP     I8     | NOT USED                | 128

  // -- CREATE PAYLOAD
  // BYTE 0:    U8    Packet Size in Byte
  // BYTE 1/2:  U16   Packet Designator
  // BYTE 3:    U8    VALUE DESIGNATOR BYTE 0
  // BYTE 4:    U8    VALUE DESIGNATOR BYTE 1
  // BYTE 5:    U8    VALUE DESIGNATOR BYTE 2
  // BYTE 6:    U8    VALUE DESIGNATOR BYTE 3 
  // BYTE 7-54: DATA AS DEFINED IN VALUE DESIGNATOR

  // Create byte array to send (see https://www.thethingsnetwork.org/docs/devices/bytes.html for byte encoding examples)
  
  Packet10_B0_Prepare();
  Packet10_B1_Prepare();
  Packet10_B2_Prepare();
  Packet10_B3_Prepare();  
  Packet_Finalize();

  // -- SEND DATA VIA LoRa
  SP.println ("");
  SP.println("LORA\tSENDING DATA PACKET VIA LORA..");

  modem.beginPacket();                                                // modem.beginPacket startet das Packet das mit LoRa versendet wird
  modem.write(payload_temp, pctr);
  err = modem.endPacket(true);                                        //When modem.endPacket(true), then confirmed data sent!
  if (err > 0){
    SP.println("LORA\tMessage sent correctly!");
  }
  else{
    SP.println("LORA\tError sending message :(");
    SP.println("LORA\t(you may send a limited amount of messages per minute, depending on the signal strength");
    SP.println("LORA\tit may vary from 1 message every couple of seconds to 1 message every minute)");
  }


  // ------ CHECK DATA UPLOAD ------  //
  if (!modem.available()){
    SP.println("LORA\tNo downlink message received at this time.");
  }
  else{
    char rcv[64];
    uint i = 0;
    while (modem.available()){
      rcv[i++] = (char)modem.read();
    }
    SP.print("LORA\tMessage Received: ");                        // PRINT DATA

    /*
    // PRINT AS HEX CHAR by CHAR
    for (unsigned int j = 0; j < i; j++) {
    SP.print(rcv[j] >> 4, HEX);
    SP.print(rcv[j] & 0xF, HEX);
    SP.print(" ");
    }
    */

    String RcvStr = "";                                    // PRINT AS TXT
    for (uint j = 0; j < i; j++) {
      RcvStr = RcvStr + rcv[j];
    }
    SP.println(RcvStr);
    SP.println();


    // ------ ANALYSE DATA ------ //
    SP.println("LORA\tChecking for commands...");                // Commands have following stucture: COMMAND:VALUE;COMMAND:VALUE;COMMAND:VALUE;
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
          SP.println("LORA\tCommand: " + command + ", Value: " + value);
          // SP.println("Remaining String: " + RcvStr);
          commands[m] = command;
          values[m] = value;
          m = m + 1;
        }
        else {
          SP.println("LORA\tERROR! Empty command");
        }
      }
      else {
        SP.println("LORA\tNo (more) commands found!");
      }
      SP.println();
    }
  }


  // ------ PARSE COMMANDS ------ //
  for (int i = 0; i < m; i++) {
    SP.println("LORA\tProcessing Command: " + commands[i] + " ,Value: " + values[i]);
    if (commands[i] == "MInt") {                                                     // COMMAND MInt
      if ((values[i].toInt() >= 1) && (values[i].toInt() <= 31536000)) {
        loopintv = (values[i].toInt() * 1000);
        SP.println("LORA\tMeasurement interval has been set to " + values[i] + " s");
      }
      else {
        SP.println("LORA\tError! Wrong measurement interval value: " + values[i]);
      }
    }
    else {                                                                           // COMMAND Not Recognized
      SP.println("LORA\tCommand was not recognized!");
    }
    SP.println();
  }


  // -- DEACTIVATION PORTS
  // PortsDeactivate() -- DO NOT ACTIVATE... MUCH HIGHER POWER CONSUMPTION IN DEEP SLEEP WHEN ACTIVE... WHY?
  MEAS_ABORT:
    
  // -- END OF MEASUREMENT CYCLE
  SP.println ("");      
  SP.print("SYSTEM\tEND OF MEASUREMENT CYCLE #"); 
  SP.print(loopctr);
  looptime = millis() - loopstart;
  SP.print(" -- Cycle time ");  
  SP.print(looptime);
  SP.println(" ms");
  
  // -- SLEEP
  DeepSleepMode();
  SP.println("");
  SP.println("");
  SP.println("");
  SP.println("");
}
