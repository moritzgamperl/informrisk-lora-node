// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- 24bit ANALOG TO DIGITAL CONVERTER (AD24) SENSOR CONFIGURATION -- Texas Instruments ADS1220
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- AGR ADS1220 Library (edited by AlpGeorisk)
#include "agr_ads1220.h"
agr_ads1220 ads1220;


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- AD24 MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- AD24 Channel Modes
// Define which AD24 channels (Ain0, Ain1, Ain2, Ain3) are active and in which mode they should operate
// 0: Channel Off
// 1: Single Ended Voltage Measurement with System 3,3V Reference (use e.p. for potentiometric measruements)
// 2: Single Ended Current Measurement (on Channel 1 and 2 the required 82 OHM measurement resistor is connected automatically; on channel 0 and 3 the user has to add high accuracy 82 OHM resistor between Channel and Ground)
// 3: Differential Voltage Measurement between this channel and +1 channel (e.g. if set on channel 0, then diff. measurement is performed between channels 0 and 1; Set +1 Channel to 0 (OFF) to avoid Errors)
// 4: Custom Mode --- NOT IMPLEMENTED YET

#define AD24_AIN0     1
#define AD24_AIN1     0    //THIS CHANNEL NATIVELY SUPPORTS 4-20 mA measurements @ 12V - 82 OHM Resistor is added automatically if set to Mode "2"
#define AD24_AIN2     0    //THIS CHANNEL NATIVELY SUPPORTS 4-20 mA measurements @ 12V - 82 OHM Resistor is added automatically if set to Mode "2"
#define AD24_AIN3     0

// -- AD24 Timing
// Interval in [ms] at which data is read from AD24 (earliest). Allow at least about 150 ms per active channel.
int     AD24_MINT =   200;     

// -- AD24 Debug Messages
// 0: Turn off debug messages; 1: Turn on debug messages
#define AD24_DEBUG    0

// -- AD24 Auto Calibrate during initialization
// 0: Turn off calibration; 1: Turn on calibration
#define AD24_CALIB    1

// -- TRANSMIT DATA
// Transmit ADC Data via LoRa? 0: NO; 1: YES
#define AD24_TRANS_AIN0     1
#define AD24_TRANS_AIN1     1
#define AD24_TRANS_AIN2     1
#define AD24_TRANS_AIN3     1


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- AD24 GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Channel Definition Array
int AD24_CH_SETUP[4] = {AD24_AIN0, AD24_AIN1, AD24_AIN2, AD24_AIN3}; // Array of settings

// -- Measurement Variables
float SUM_AD24_AIN0 = 0;
float SUM_AD24_AIN1 = 0;
float SUM_AD24_AIN2 = 0;
float SUM_AD24_AIN3 = 0;
uint MCTR_AD24 = 0;
uint VCTR_AD24 = 0;      
float AVG_AD24_AIN0 = 0;
float AVG_AD24_AIN1 = 0;
float AVG_AD24_AIN2 = 0;
float AVG_AD24_AIN3 = 0;

long  ad24_raw;
float ad24_result;

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- AD24 FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- MEASUREMENT RELAY SWITCH FUNCTIONS (4x)
//functions to turn on/off measurement relays (for 4-20mA measurements) on ADC24 Channel 1 and 2 

void R1_TurnON () {              // turn on measurement resistor for ADS channel 1 
  digitalWrite(REL1_A, HIGH);       
  digitalWrite(REL1_B, LOW);  
  delay(50);
  digitalWrite(REL1_A, LOW);    
}

void R1_TurnOFF () {              // turn off measurement resistor for ADS channel 1
  digitalWrite(REL1_A, LOW);        
  digitalWrite(REL1_B, HIGH);
  delay(50);
  digitalWrite(REL1_B, LOW);      
}

void R2_TurnON () {              // turn on measurement resistor for ADS channel 2
  digitalWrite(REL2_A, HIGH);       
  digitalWrite(REL2_B, LOW); 
    delay(50);
  digitalWrite(REL2_A, LOW);       
}

void R2_TurnOFF () {              // turn off measurement resistor for ADS channel 2
  digitalWrite(REL2_A, LOW);       
  digitalWrite(REL2_B, HIGH); 
  delay(50);
  digitalWrite(REL2_B, LOW);
}

// -- AD24 RELAY INITIALIZE
// Initializes the relays (optional addition of measurement relay on channel 1 and 2)
void AD24_RELAY_Init () {
  SP.print("AD24\tMeasurement Resistors: Initializing relays..");
  if (SET_AD24) {
    // Setup Debug Messages
    ads1220.ads1220debug = AD24_DEBUG;       //0: Debug Messages OFF; 1: Debug Messages ON;     
    
    // SETUP AD24 RELAY CHANNEL 1
    if (AD24_AIN1 == 2) {
      R1_TurnON ();
      SP.print(" Ain1: ON;");
    }      
    else {
      R1_TurnOFF ();
      SP.print(" Ain1: OFF;");
    }
    // SETUP AD24 RELAY CHANNEL 2
    if (AD24_AIN2 == 2) {
      R2_TurnON ();
      SP.println(" Ain2: ON.");
    }      
    else {
      R2_TurnOFF ();
      SP.println(" Ain2: OFF.");
    }
  }
  else {
    R1_TurnOFF ();
    R2_TurnOFF ();
    SP.println(" Ain1/Ain2: OFF.");
    // DEV: SET SENSOR TO SLEEP MODE??
  }        
}

// -- AD24 INITIALIZE
// Initializes AD24 (ADS1202) (if active)
void AD24_Init () {
  AD24_RELAY_Init ();
  SP.print("AD24\t");  
  if (SET_AD24) {
    SP.print("Initializing.. ");    
    //Reset Variables    
    SUM_AD24_AIN0 = 0, SUM_AD24_AIN1 = 0, SUM_AD24_AIN2 = 0, SUM_AD24_AIN3 = 0;
    MCTR_AD24 = 0, VCTR_AD24 = 0;      
    AVG_AD24_AIN0 = 0, AVG_AD24_AIN1 = 0, AVG_AD24_AIN2 = 0, AVG_AD24_AIN3 = 0;
    // Setup Debug Messages
    ads1220.ads1220debug = AD24_DEBUG;       //0: Debug Messages OFF; 1: Debug Messages ON;
    // Initialize Hardware
    if (ads1220.begin(CS_ADC24,DR_ADC24)) {SP.println("OK!");}
    else 
    {
      ads1220.powerdown();
      SP.println("ERROR: ADS1220 was NOT initialized successfully!");
    }

    // ------ CALIBRATE ADS1220 ------ //
    if (AD24_CALIB)
    {
      SP.print("AD24\tCalibration (this may take a few seconds)..");
      ads1220.calibrate();
      SP.println(" OK!");
    }
  }
  else {
    ads1220.begin(CS_ADC24,DR_ADC24);
    ads1220.powerdown();
    SP.println("OFF");
  }
}

// -- AD24 SETUP
// Setup AD24 measurements (ADS1202) (if active), Channel specific settings are done during measurement
void AD24_Setup () {
  if (SET_AD24) {  
    SP.print("AD24\tSetting up (general settings).. ");
    //SETUP MEASUREMENT (GENERAL SETTINGS, VALID FOR ALL MEASUREMENT TYPES)
    ads1220.config_gain(PGA_GAIN_1);
    ads1220.config_pga(PGA_OFF);
    ads1220.config_idac1(IDAC1_OFF);
    ads1220.config_idac2(IDAC2_OFF);
    ads1220.config_fir(FIR_NONE);        
    ads1220.config_tempmode(TEMP_OFF);
    ads1220.config_burnout(BO_OFF);
    ads1220.config_datarate(DR_20SPS);
    ads1220.config_opmode(OM_NORMAL);
    ads1220.config_conmode(CM_SINGLE_SHOT);
    SP.println(" OK!");
    for (int i = 0; i <= 3; i++) {
      char m_setting[100];
      sprintf(m_setting, "AD24\tChannel Ain%d: ", i);
      if      (AD24_CH_SETUP[i] == 0) strcat(m_setting, "OFF");
      else if (AD24_CH_SETUP[i] == 1) strcat(m_setting, "Single ended voltage measurement");
      else if (AD24_CH_SETUP[i] == 2) strcat(m_setting, "Single ended current measurement");
      else if (AD24_CH_SETUP[i] == 3) {
        char m_setting2[2];
        sprintf(m_setting2,"%d",i+1);          
        strcat(m_setting, "Differential voltage measurement with channel Ain");
        strcat(m_setting, m_setting2);
        }      
      else strcat(m_setting, "Unknown Setup");
      SP.println(m_setting);
    }    
  }                                                                                                                                                                                                        
}

// -- AD24 MEASURE
// Setup AD24 measurements (ADS1202) & perform measurement CHANNEL BY CHANNEL
void AD24_Measure () {
  if (SET_AD24) {                                    
    MCTR_AD24++;
    VCTR_AD24++;
    for (int i = 0; i <= 3; i++) {      
      //SETUP ADS1220 CHANNEL
      if      (i == 0) {
        if (AD24_AIN0 == 1 || AD24_AIN0 == 2) ads1220.config_mux(SE_CH0);
        else if (AD24_AIN0 == 3) ads1220.config_mux(DIF_CH0_1);
        else goto SKIP_MEASUREMENT;
      }
      else if (i == 1) {
        if (AD24_AIN1 == 1 || AD24_AIN1 == 2) ads1220.config_mux(SE_CH1);
        else if (AD24_AIN1 == 3) ads1220.config_mux(DIF_CH1_2);
        else goto SKIP_MEASUREMENT;
      }
      else if (i == 2) {  
        if (AD24_AIN2 == 1 || AD24_AIN2 == 2) ads1220.config_mux(SE_CH2);
        else if (AD24_AIN2 == 3) ads1220.config_mux(DIF_CH2_3);
        else goto SKIP_MEASUREMENT;
      }
      else if (i == 3) {
        if (AD24_AIN3 == 1 || AD24_AIN3 == 2) ads1220.config_mux(SE_CH3);
        else goto SKIP_MEASUREMENT;
      }
      else goto SKIP_MEASUREMENT;

      //SETUP REFERENCE
      if (AD24_CH_SETUP[i] == 1) ads1220.config_vref(VREF_AVDD);
      else if (AD24_CH_SETUP[i] == 2) ads1220.config_vref(VREF_INT);
      else if (AD24_CH_SETUP[i] == 3) ads1220.config_vref(VREF_AVDD);   
      else goto SKIP_MEASUREMENT;     

      //PERFORM MEASUREMENT                                  
      ad24_raw = ads1220.read_single_shot();
      ad24_result = ads1220.dac2mv(ad24_raw);
      if (AD24_CH_SETUP[i] == 2) ad24_result = ad24_result/82;
      else ad24_result = ad24_result / 1000;                            //Convert to V (to have about same Range of values a mA) --> better for transmission

      //SUMMERIZE DATA
      if      (i == 0) SUM_AD24_AIN0 = SUM_AD24_AIN0 + ad24_result;
      else if (i == 1) SUM_AD24_AIN1 = SUM_AD24_AIN1 + ad24_result;
      else if (i == 2) SUM_AD24_AIN2 = SUM_AD24_AIN2 + ad24_result;
      else if (i == 3) SUM_AD24_AIN3 = SUM_AD24_AIN3 + ad24_result;
                  
      SKIP_MEASUREMENT:
      SP.print("");                  
    }      
  }
}  

// -- AD24 FINALIZE
// Shut down sensor and evaluate data (calculate averages)
void AD24_Finalize () {
  if (SET_AD24) {
    //SET Sensor to sleep mode
    ads1220.powerdown();
    //Calculate and output Averages
    AVG_AD24_AIN0 = SUM_AD24_AIN0 / VCTR_AD24;
    AVG_AD24_AIN1 = SUM_AD24_AIN1 / VCTR_AD24;
    AVG_AD24_AIN2 = SUM_AD24_AIN2 / VCTR_AD24;
    AVG_AD24_AIN3 = SUM_AD24_AIN3 / VCTR_AD24;

    //Report results    
    float AD24_RESULT_ARRAY[4] = {AVG_AD24_AIN0, AVG_AD24_AIN1, AVG_AD24_AIN2, AVG_AD24_AIN3};
    for (int i = 0; i <= 3; i++) {
      if (!AD24_CH_SETUP[i] == 0) {
        char m_name[15] = "";
        char m_dim[5]   = "";      
        if (AD24_CH_SETUP[i] == 2) {
          sprintf(m_name, "Ain%d Current", i);
          strcpy(m_dim, "mA");      
        }
        else if (AD24_CH_SETUP[i] == 1 || AD24_CH_SETUP[i] == 3) {
          sprintf(m_name, "Ain%d Voltage", i);
          strcpy(m_dim, "V");      
        }
        else {
          sprintf(m_name, "Ain%d ???", i);
          strcpy(m_dim, "?");        
        }      
        sprintf(report, "AD24\t%s\t%.3f\t%s\t%d\t%d", m_name, AD24_RESULT_ARRAY[i], m_dim, VCTR_AD24, MCTR_AD24);
        SP.println(report);
      }     
     
    }
  }
}