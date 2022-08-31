/* Fast Read Sensor Mode for Murata SCL3300 Inclinometer
 * Version 3.2.0 - September 3, 2021
 * Example6_FastReadMode
 * Warning: Using Fast Read Mode in the library works by keeping the
 *          SPI connection continuously open.  This may or may not affect
 *          the behavior of other hardware interactions, depending on the
 *          sketch design.  Fast Read Mode is considered an advanced use case,
 *          and not recommended for the beginner.
*/

#include <SPI.h>
#include <SCL3300.h>

SCL3300 inclinometer;
//Default SPI chip/slave select pin is D10

// Need the following define for SAMD processors
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

int16_t AngX, AngY, AngZ, AccX, AccY, AccZ, STO, TEMP;
uint16_t StatusSum, WHOAMI;
long int n = 0;
long int startmillis = 0;

void setup() {
  Serial.begin(9600);
  delay(2000); //SAMD boards may need a long time to init SerialUSB
  Serial.println("Reading Raw register values from SCL3300 Inclinometer");
  Serial.println("We use FastReadMode in 1 second bursts.");
  if (inclinometer.begin() == false) {
    Serial.println("Murata SCL3300 inclinometer not connected.");
    while(1); //Freeze
  }
}

void loop() {
  n = 0;
  inclinometer.setFastReadMode();
  startmillis = millis();
  while (millis() - startmillis < 1000) {
    if (inclinometer.available()) { //Get next block of data from sensor
      n++;
	  // Read raw binary data
      AngX = inclinometer.sclData.AngX;
      AngY = inclinometer.sclData.AngY;
      AngZ = inclinometer.sclData.AngZ;
      AccX = inclinometer.sclData.AccX;
      AccY = inclinometer.sclData.AccY;
      AccZ = inclinometer.sclData.AccZ;
      STO = inclinometer.sclData.STO;
      WHOAMI = inclinometer.sclData.WHOAMI;
      TEMP = inclinometer.sclData.TEMP;
      StatusSum = inclinometer.sclData.StatusSum;
    } else inclinometer.reset();
  }
  inclinometer.stopFastReadMode();
  Serial.print("Iterations per second: ");
  Serial.println(n);
}