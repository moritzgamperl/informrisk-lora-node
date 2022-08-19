// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- BATTERY VOLTAGE SENSOR CONFIGURATION
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SETTINGS, VARIABLES AND FUNCTIONS
// -------------------------------------------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BATV MEASUREMENT SETTINGS 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- DATA RETRIEVAL interval
// Interval in [ms] at which data is read Arduino Ain Port (earliest).
int     BATV_MINT = 200;

// -- TRANSMIT DATA
// Transmit Battery Voltage via LoRa?
#define BATV_TRANS 1

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BATV MEASUREMENT CONSTANTS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------
#define BATT_R1 33000                                                  // Voltage divider resistor 1 in Ohms (used for voltage measurement)
#define BATT_R2 100000                                                 // Voltage divider resistor 2 in Ohms

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BATV GLOBAL VARIABLES (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -- Measurement Variables
long batv_raw;                           // Variable for single measurement (raw)
float batv;                              // Variable for single measurement 
float SUM_BATV = 0;                      // SUM of all measurements in one measurement cycle
uint  MCTR_BATV = 0;                     // Measurement counter (theoretical number of performed measurements)
uint  VCTR_BATV = 0;                     // Value counter (number of collected values)
float AVG_BATV = 0;                      // Average value of all measurements in one measurement cycle

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// -- BATV FUNCTIONS (DO NOT CHANGE!) 
// -------------------------------------------------------------------------------------------------------------------------------------------------------

void BATV_Init () {
  SP.print("BATV\t");
  if (SET_BATV) {
    SP.print("Initializing.. ");
    //Reset Variables
    SUM_BATV = 0;
    MCTR_BATV = 0, VCTR_BATV = 0;
    AVG_BATV = 0;
    analogReadResolution(12);
    SP.println("OK");    
  }
  else SP.println("OFF");  
}  

void BATV_Measure () {
  if (SET_BATV) {
    MCTR_BATV++;
    VCTR_BATV++;
    //read Analog IN (Arduino)
    batv_raw = analogRead(AIN_VBAT);
    //Transform raw value to Battery Voltage using 2-part calibration curve
    if (batv_raw > 2207) batv = ((0.0008*batv_raw*batv_raw) - (3.3705*batv_raw) + 3678.6);   
    else batv = ((0.0033*batv_raw) + 0.2608);
    //Summarize data for averaging
    SUM_BATV = SUM_BATV + batv;
  }      
}

void BATV_Finalize () {
  if (SET_BATV) {
    //Turn off measurement
    digitalWrite(PW_VBAT, LOW);    
    //Calculate and output Averages
    AVG_BATV = SUM_BATV / VCTR_BATV;
    // Output Results
    sprintf(report, "BATV\tBattery Voltage\t%.2f\tV\t%d\t%d", AVG_BATV, VCTR_BATV, MCTR_BATV);
    SP.println(report);
  }
}
