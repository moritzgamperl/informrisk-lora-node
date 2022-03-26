// -------------------------------------------------------------------------------------------------------------------------------------------------------
// ADC CONFIGURATION SETTINGS
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// Olimex ADS1220                                                      // For the ADC CHIP
#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4

// SET ADS1220 channels; 0 = off; 1 = on // For the ADC CHIP
bool ADS_C0 = true;
bool ADS_C1 = false;
bool ADS_C2 = false;
bool ADS_C3 = false;

//ads1220.ads1220debug = 1;                      //TURN ON/OFF DEBUG MESSAGES

float lpot;
float lpotvoltage;
