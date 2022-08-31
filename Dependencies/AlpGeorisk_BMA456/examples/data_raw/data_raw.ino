
#include "arduino_bma456.h"
BMA456 inkl_a;

float x_a = 0, y_a = 0, z_a = 0;
int temp_a = 0;
bool inkl_a_init;

int powerpin = 7;

void setup(void) {
	pinMode(powerpin, OUTPUT);
    digitalWrite(powerpin, HIGH);

    Serial.begin(115200);
  
    //Set I2C Address: BMA4_I2C_ADDR_PRIMARY (0x18) // BMA4_I2C_ADDR_PRIMARY (0x19)
    //Set Range: RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G
    //Set Measurement frequency: ODR_0_78_HZ, ODR_1_5_HZ, ODR_3_1_HZ, ODR_6_25_HZ, ODR_12_5_HZ, ODR_25_HZ, ODR_50_HZ, ODR_100_HZ, ODR_200_HZ, ODR_400_HZ, ODR_800_HZ, ODR_1600_HZ
    //Set Filter Bandwidth / Averageing: OSR4_AVG1, OSR2_AVG2, NORMAL_AVG4, CIC_AVG8, RES_AVG16, RES_AVG32, RES_AVG64, RES_AVG128
    //Set Performance Mode: CIC_AVG, CONTINUOUS

    //INITIALIZE inkl
    Serial.println("Initializing INKL_A");
    if (inkl_a.initialize(BMA4_I2C_ADDR_SECONDARY, RANGE_2G, ODR_6_25_HZ, NORMAL_AVG4, CIC_AVG) == 0) {
      Serial.println("INKL_A: Successfully connected to BMA456");
      inkl_a_init = 1;
    }
    else {
      Serial.println("INKL_A: Connection error - BMA could not be initialized!");
      inkl_a_init = 0;
      delay(1000);
    }
}

void loop(void) {
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
      Serial.println(temp_a);
	}

    delay(150);
}