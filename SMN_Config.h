// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SMN CONFIGURATION SETTINGS
// -------------------------------------------------------------------------------------------------------------------------------------------------------


float smn_x = 0, smn_y = 0, smn_z = 0;       //SMN Measurement Variables - velocity?
int32_t smn_temp = 0;                        //SMN Measurement Variables - Temperature

int I1_PORT = A0;                            // Digital port to turn on/off Step Counter 1;  set to -1 to deactivate
int I2_PORT = -1;

bool I1A = true;
bool I2A = false;
bool I1B = false;
bool I2B = false;
