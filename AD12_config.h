// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- 12bit ANALOG TO DIGITAL CONVERTER (AD12) SENSOR CONFIGURATION -- Arduino MKR WAN 1310
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- AD12 MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- AD12 Channel Modes
// Define which AD12 channels (Ain0, Ain1) are active and in which mode they should operate
// 0: Channel Off
// 1: Single Ended Voltage Measurement with System 3,3V Reference (use e.p. for potentiometric measruements)

int     AD12_AIN0 =   1;
int     AD12_AIN1 =   0;    

// -- AD12 Timing
// Interval in [ms] at which data is read from AD24 (earliest). Allow at least about 50 ms per active channel.
int     AD12_MINT =   200;

// -- TRANSMIT DATA
// Transmit ADC Data via LoRa? 0: NO; 1: YES
#define AD12_TRANS_AIN0     1
#define AD12_TRANS_AIN1     1


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- AD12 GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Channel Definition Array
int AD12_CH_SETUP[4] = {AD12_AIN0, AD12_AIN1}; // Array of settings

// -- Measurement Variables
float SUM_AD12_AIN0 = 0;
float SUM_AD12_AIN1 = 0;
uint MCTR_AD12 = 0;
uint VCTR_AD12 = 0;      
float AVG_AD12_AIN0 = 0;
float AVG_AD12_AIN1 = 0;

int   ad12_raw;
float ad12_result;


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- AD12 FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- AD12 INITIALIZE
// Initializes AD12 (On Board Arduino MKR WAN 1310) (if active)
void AD12_Init () {
  SP.print("AD12\t");  
  if (SET_AD12) {
    SP.print("Initializing.. ");    
    //Reset Variables    
    SUM_AD12_AIN0 = 0, SUM_AD12_AIN1 = 0;
    MCTR_AD12 = 0, VCTR_AD12 = 0;      
    AVG_AD12_AIN0 = 0, AVG_AD12_AIN1 = 0;
    SP.println("OK!");
    // Check if conflict with power out settings
    if (((SET_SW3V3A && PW_3V3_A == AIN0_12) || (SET_SW3V3B && PW_3V3_B == AIN0_12)) && AD12_AIN0 == 1) {
      AD12_AIN0 = 0;  
      SP.println("AD12\tERROR! AD12 Ain0 is already configured as voltage output! AD12 Ain0 measurement is deactivated.");
    }
    if (((SET_SW3V3A && PW_3V3_A == AIN1_12) || (SET_SW3V3B && PW_3V3_B == AIN1_12)) && AD12_AIN1 == 1) {
      AD12_AIN1 = 0;  
      SP.println("AD12\tERROR! AD12 Ain1 is already configured as voltage output! AD12 Ain1 measurement is deactivated.");
    }
    // Check if both channels are 
    if (AD12_AIN0 == 0 && AD12_AIN1 == 0) SET_AD12 = false;      
  }
  else {
    SP.println("OFF");
  }
}

void AD12_Setup () {
  if (SET_AD12) {
    // -- SETUP ANALOG INPUT RESOLUTION
    SP.print("AD12\tSetting resolution to 12 bit...");
    analogReadResolution(12);  
    SP.println(" OK!");
  }  
}

void AD12_Measure () {
  if (SET_AD12) {
    MCTR_AD12++;
    //PERFORM MEASUREMENT CHANNEL AIN0
    if (AD12_AIN0) {
      ad12_raw = analogRead(AIN0_12);      
      ad12_result = ((float)ad12_raw)/4096*3.3;      
      SUM_AD12_AIN0 = SUM_AD12_AIN0 + ad12_result;
    }
    if (AD12_AIN1) {
      ad12_raw = analogRead(AIN1_12);
      ad12_result = ((float)ad12_raw)/4096*3.3;
      SUM_AD12_AIN1 = SUM_AD12_AIN1 + ad12_result;
    }
    VCTR_AD12++;            
  }   
}

void AD12_Finalize () {
  if (SET_AD12) {
    //Calculate and output Averages & Report Results   
    if (AD12_AIN0) {
      AVG_AD12_AIN0 = SUM_AD12_AIN0 / VCTR_AD12;
      sprintf(report, "AD12\tAin0 Voltage\t%.3f\tV\t%d\t%d", AVG_AD12_AIN0, VCTR_AD12, MCTR_AD12);
      SP.println(report);
    }      
    if (AD12_AIN1) {
      AVG_AD12_AIN1 = SUM_AD12_AIN1 / VCTR_AD12;
      sprintf(report, "AD12\tAin1 Voltage\t%.3f\tV\t%d\t%d", AVG_AD12_AIN1, VCTR_AD12, MCTR_AD12);
      SP.println(report);
    }
  }
}
