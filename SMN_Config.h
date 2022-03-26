// -------------------------------------------------------------------------------------------------------------------------------------------------------
// SMN CONFIGURATION SETTINGS
// -------------------------------------------------------------------------------------------------------------------------------------------------------
#include "arduino_bma456.h"
BMA456 inkl_a;
BMA456 inkl_b;

float x_a = 0, y_a = 0, z_a = 0, x_b = 0, y_b = 0, z_b = 0;
int temp_a = 0, temp_b = 0;
bool inkl_a_init, inkl_b_init;


int I1_PORT = 2;                            // Digital port to turn on/off Step Counter 1;  set to -1 to deactivate
int I2_PORT = -1;

bool I1A = true;
bool I2A = false;
bool I1B = false;
bool I2B = false;
