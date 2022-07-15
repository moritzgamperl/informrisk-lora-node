// -------------------------------------------------------------------------------------------------------------------------------------------------------
// ADC CONFIGURATION SETTINGS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// Olimex ADS1220                                                      // For the ADC CHIP
#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4

// SET ADS1220 channels; 0 = off; 1 = single-ended measurement; 2 = single-ended measurement with 12V (4-20 mA)
int ADS_C0 = 0;
int ADS_C1 = 2;
int ADS_C2 = 2;
int ADS_C3 = 0;

//ads1220.ads1220debug = 1;                      //TURN ON/OFF DEBUG MESSAGES

float lpot;
float lpotvoltage;
