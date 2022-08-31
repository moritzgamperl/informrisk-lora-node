
#include "arduino_bma456.h"
BMA456 inkl_a;
BMA456 inkl_b;

float x_a = 0, y_a = 0, z_a = 0, x_b = 0, y_b = 0, z_b = 0;
int temp_a = 0, temp_b = 0;
bool inkl_a_init, inkl_b_init;

int powerpin = 7;

void setup(void) {
	  pinMode(powerpin, OUTPUT);
    
    Serial.begin(115200);
}

void loop(void) {

    // POWERUP SENSORS
    digitalWrite(powerpin, HIGH);
    delay (100);
    
    //SETUP INKLINOMETERS A and B
    //Set I2C Address: BMA4_I2C_ADDR_PRIMARY (0x18) // BMA4_I2C_ADDR_PRIMARY (0x19)
    //Set Range: RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G
    //Set Measurement frequency: ODR_0_78_HZ, ODR_1_5_HZ, ODR_3_1_HZ, ODR_6_25_HZ, ODR_12_5_HZ, ODR_25_HZ, ODR_50_HZ, ODR_100_HZ, ODR_200_HZ, ODR_400_HZ, ODR_800_HZ, ODR_1600_HZ
    //Set Filter Bandwidth / Averageing: OSR4_AVG1, OSR2_AVG2, NORMAL_AVG4, CIC_AVG8, RES_AVG16, RES_AVG32, RES_AVG64, RES_AVG128
    //Set Performance Mode: CIC_AVG, CONTINUOUS

    //INITIALIZE inkl1
    Serial.println("Initializing INKL_A (0x19)");
    if (inkl_a.initialize(BMA4_I2C_ADDR_SECONDARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) {
      Serial.println("INKL_A: Successfully connected to BMA456");
      inkl_a_init = 1;
    }
    else {
      Serial.println("INKL_A: Connection error - BMA could not be initialized!");
      inkl_a_init = 0;
      delay(1000);
    }

    //INITIALIZE inkl2
    Serial.println("Initializing INKL_B (0x18)");
    if (inkl_b.initialize(BMA4_I2C_ADDR_PRIMARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) {
      Serial.println("INKL_B: Successfully connected to BMA456");
      inkl_b_init = 1;
    }
    else {
      Serial.println("INKL_B: Connection error - BMA could not be initialized!");
      inkl_b_init = 0;
      delay(1000);
    }

    // Allow enough time for initialization
    delay(100);

    // PERFORM 10 MEASUREMENTS
    for (int i = 0; i <= 10; i++) {
              
    if (inkl_a_init) {
      inkl_a.getAcceleration(&x_a, &y_a, &z_a);
      temp_a = inkl_a.getTemperature();
      Serial.print("A: X:");
      Serial.print(x_a, 3);
      Serial.print(", Y: ");
      Serial.print(y_a, 3);
      Serial.print(", Z: ");
      Serial.print(z_a, 3);
      Serial.print(", T: ");
      Serial.print(temp_a);
      }
    if (inkl_b_init) { 
      inkl_b.getAcceleration(&x_b, &y_b, &z_b);
      temp_b = inkl_b.getTemperature();
      Serial.print(" -- B: X: ");
      Serial.print(x_b, 3);
      Serial.print(", Y: ");
      Serial.print(y_b, 3);
      Serial.print(", Z: ");
      Serial.print(z_b, 3);
      Serial.print(", T: ");
      Serial.println(temp_b);
      }
    delay(150); // Adapt to measurement frequency selected above
    }
    
    // DEACTIVATE SENSORS (Saves power)
    Serial.println("Deactivating Sensors...");
    inkl_a.accel_enable(AKM_POWER_DOWN_MODE);
    inkl_b.accel_enable(AKM_POWER_DOWN_MODE);
    delay(1000);

    //POWER DOWN SENSORS
    Serial.println("Powering Down Sensors...");
    Serial.println("");
    digitalWrite(powerpin, LOW);

    delay(5000);
}