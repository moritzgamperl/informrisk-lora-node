// -------------------------------------------------------------------------------------------------------------------------------------------------------
// IMU CONFIGURATION SETTINGS
// -------------------------------------------------------------------------------------------------------------------------------------------------------


long meassum[22];                            // Array with summerized measurements // Achtung! Länge = max i + 1, da indizierung mit 0 losgeht
long arr_imu_a_x[100];                       // Array with measurements for imu.a.x Qestion: How to deal with max size of Array? Here, 1000 as default
long arr_imu_a_y[100];
long arr_imu_a_z[100];
long arr_imu_g_x[100];
long arr_imu_g_y[100];
long arr_imu_g_z[100];
long arr_mag_m_x[100];
long arr_mag_m_y[100];
long arr_mag_m_z[100];

float temperature, pressure, altitude;       // Create the temperature, pressure and altitude variables
                                                                                                    
int16_t ax, ay, az;                          // For acceleration values from 
int16_t gx, gy, gz;                          // For  values from 
int16_t mx, my, mz;                          // For  values from 

// Measurement interval and counter are same as for Pololu
int IMU_MINT = 100;       // measurement interval of the IMU (Accelerometer & Gyro) (ms) -- make sure is supported by device; e.g. 100 = 10 Hz
int MAG_MINT = 100;      // measurement interval of the Magnetometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz 
int BARO_MINT = 100;     // measurement interval of the Barometer (ms) -- make sure is supported by device; e.g. 1000 = 1 Hz
