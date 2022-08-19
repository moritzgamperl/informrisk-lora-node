// -------------------------------------------------------------------------------------------------------------------------------------------------------
// INFORM@RISK MEASUREMENT NODE -- SUBPROGRAMS
// -------------------------------------------------------------------------------------------------------------------------------------------------------
// DO NOT CHANGE!! Programs required for use of I@R Measurement Node
// -------------------------------------------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// GENERAL FUNCTIONS

//SORT ARRAY ASCENDING 
//function to sort the array in ascending order
int Array_median(long *array , int n)
{ 
    // declare some local variables
    int i=0 , j=0 , temp=0;

    for(i=0 ; i<n ; i++)
    {
        for(j=0 ; j<n-1 ; j++)
        {
            if(array[j]>array[j+1])
            {
                temp        = array[j];
                array[j]    = array[j+1];
                array[j+1]  = temp;
            }
        }
    }
    /*
    Serial.println("\nThe array after sorting is..\n");
    for(i=0 ; i<n ; i++)
    {
        Serial.println("\narray_1[%d] : %d",i,array[i]);
    } */

    long median=0;
    
    // if number of elements are even
    if(n%2 == 0)
        median = (array[(n-1)/2] + array[n/2])/2.0;
    // if number of elements are odd
    else
        median = array[n/2];
    
    return median;
    //Serial.print("The median is ");
    //Serial.println(median);
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------
// HARDWARE FUNCTIONS - GENERAL

// -- DELAY/SLEEP/DEEPSLEEP 
//Set hardware into delay, sleep or deepsleep mode
void DeepSleepMode() 
{
  looptime = millis() - loopstart;
  //check if loop took to long
  if(looptime>loopintv) sleeptime = 10000;
  else sleeptime = loopintv - looptime;
  //check if min sleeptime of 10s is met  
  if (sleeptime <= 10000) {    
    sleeptime = 10000;
    SP.println("SLEEP\tWarning! Sleeptime is too low - was set to 10000 ms");
  }
  
  if (strcmp(SleepMode, "sleep") == 0) {
    SP.print("SYSTEM\tSLEEP duration: ");
    SP.print(sleeptime);
    SP.println(" ms");    
    LowPower.sleep(sleeptime);
  }
  else if (strcmp(SleepMode, "deepsleep") == 0) {
    SP.print("SYSTEM\tDEEP SLEEP duration: ");
    SP.print(sleeptime);
    SP.println(" ms"); 
    LowPower.deepSleep(sleeptime);
  }
  else {
    SP.print("SYSTEM\tDELAY duration: ");
    SP.print(sleeptime);
    SP.println(" ms"); 
    delay(sleeptime);
  }
  Wire.end();
}

// -- PORTS ACTIVATE
//Sets up ports for measurements
void PortsActivate() 
{ 
  //Set port status according to selected sensors (turn off unused ports)
  if (!SET_BATV) PW_VBAT_STS = false;
  if (!SET_IMU && !SET_BARO) PW_IMBA_STS = false;
  if (!SET_AD24) {
    DR_ADC24_STS = false;
    CS_ADC24_STS = false;
  }              
  if (!SET_INCL) CS_INKL_STS = false;
  if (!SET_SW3V3) PW_3V3_STS = false;
  if (!SET_V12) {
    PW1_V12_STS = false;
    PW2_V12_STS = false;
  }
  if (!SET_SER1) {
    SER_RX_STS = false;
    SER_TX_STS = false;
  }
  if (!SET_AD12) {
    AIN0_12_STS = false;
    AIN1_12_STS = false; 
    }   

  //Create port-status arrays
  bool DPORTS_STS [15] = {PW_VBAT_STS, PW1_V12_STS, PW_3V3_STS, PW_IMBA_STS, DR_ADC24_STS, CS_ADC24_STS, PW2_V12_STS, CS_INKL_STS, SPI_MOS_STS, SPI_SCK_STS, SPI_MIS_STS, I2C_SDA_STS, I2C_SCL_STS, SER_RX_STS, SER_TX_STS};
  bool APORTS_STS [7] = {AIN0_12_STS, AIN1_12_STS, REL1_A_STS, REL1_B_STS, REL2_A_STS, REL2_B_STS, AIN_VBAT_STS};

  //Set Additional 3V3ports
  //During initialization set to OUTPUT, LOW. Is set HIGH with Command "SW33_ON"
  if (SET_SW3V3A) {
    if (PW_3V3_A >= 15) {                                           //Analog ports: 16 - 21
      APORTS_STS [PW_3V3_A - 15] = true;
      APORTS_ON [PW_3V3_A - 15] = "OUTPUT,LOW";  
    }
    else {                                                          //Digital ports: 0 - 15
      DPORTS_STS [PW_3V3_A] = true;
      DPORTS_ON [PW_3V3_A] = "OUTPUT,LOW";            
    }
  }

  if (SET_SW3V3B) {
    if (PW_3V3_B >= 15) {                                           //Analog ports: 16 - 21
      APORTS_STS [PW_3V3_B - 15] = true;
      APORTS_ON [PW_3V3_B - 15] = "OUTPUT,LOW";  
    }
    else {                                                          //Digital ports: 0 - 15
      DPORTS_STS [PW_3V3_B] = true;
      DPORTS_ON [PW_3V3_B] = "OUTPUT,LOW";            
    }
  }
  
  //Setup digital ports
  SP.println("Setting up digital ports: ");
  for (int i = 0; i <= 14; i++) {
    SP.print(DPORTS_NAME[i]);
    SP.print("(");
    SP.print(DPORTS[i]);
    SP.print("): ");
    String CURSTATE = "";    
    if (DPORTS_STS[i]) CURSTATE = DPORTS_ON[i];
    else CURSTATE = DPORTS_OFF[i];
    if (CURSTATE == "OUTPUT,HIGH") {
    SP.print("OUTPUT-HIGH\t\t");
    pinMode(DPORTS[i], OUTPUT);
    digitalWrite(DPORTS[i], HIGH);
    }
    else if (CURSTATE == "OUTPUT,LOW") {
    SP.print("OUTPUT-LOW\t\t");
    pinMode(DPORTS[i], OUTPUT);
    digitalWrite(DPORTS[i], LOW);      
    }
    else if (CURSTATE == "INPUT") {
    SP.print("INPUT\t\t");
    pinMode(DPORTS[i], INPUT);
    } 
    else if (CURSTATE == "INPUT,PULLUP") {
    SP.print("INPUT-PULUP\t\t");
    pinMode(DPORTS[i], INPUT_PULLUP);
    }
    if ((i == 4) || (i == 9) || (i == 14)) SP.println("");           
  }
  SP.println("");

  //Setup analog ports
  SP.println("Setting up analog ports: ");
  for (int i = 0; i <= 6; i++) {
    SP.print(APORTS_NAME[i]);
    SP.print("(");
    SP.print(APORTS[i]-15);
    SP.print("): ");
    String CURSTATE = "";    
    if (APORTS_STS[i]) CURSTATE = APORTS_ON[i];
    else CURSTATE = APORTS_OFF[i];
    if (CURSTATE == "OUTPUT,HIGH") {
    SP.print("OUTPUT-HIGH\t\t");
    pinMode(APORTS[i], OUTPUT);
    digitalWrite(APORTS[i], HIGH);
    }
    else if (CURSTATE == "OUTPUT,LOW") {
    SP.print("OUTPUT-LOW\t\t");
    pinMode(APORTS[i], OUTPUT);
    digitalWrite(APORTS[i], LOW);      
    }
    else if (CURSTATE == "INPUT") {
    SP.print("INPUT\t\t");
    pinMode(APORTS[i], INPUT);
    } 
    else if (CURSTATE == "INPUT,PULLUP") {
    SP.print("INPUT-PULUP\t\t");
    pinMode(APORTS[i], INPUT_PULLUP);
    }
    if ((i == 4) || (i == 6)) SP.println("");           
  }
  SP.println("");

  // Activate I2C communication
  SP.print("I2C\tActivating... ");
  Wire.begin();
  SP.println("OK!");
  // Activate SPI communication
  SP.print("SPI\tActivating... ");
  SPI.begin();
  SP.println("OK!");
  
  // Wait for power to balance
  delay(sensorstarttime);
}

// -- PORTS DEACTIVATE
// sets up ports for sleep
void PortsDeactivate() { 
  // Deactivate I2C communication
  SP.print("I2C\tDeactivating... ");
  //Wire.end();
  SP.println("OK!");
  // Activate SPI communication
  SP.print("SPI\tDeactivating... ");
  //SPI.end();
  SP.println("OK!");
  SP.println("");
  
  //Shut down digital ports
  SP.println("Shutting down digital ports: ");
  for (int i = 0; i <= 14; i++) {
    SP.print(DPORTS_NAME[i]);
    SP.print("(");
    SP.print(DPORTS[i]);
    SP.print("): ");
    if (DPORTS_OFF[i] == "OUTPUT,HIGH") {
      SP.print("OUTPUT-HIGH\t\t");
      pinMode(DPORTS[i], OUTPUT);
      digitalWrite(DPORTS[i], HIGH);
    }
    else if (DPORTS_OFF[i] == "OUTPUT,LOW") {
      SP.print("OUTPUT-LOW\t\t");
      pinMode(DPORTS[i], OUTPUT);
      digitalWrite(DPORTS[i], LOW);      
    }
    else if (DPORTS_OFF[i] == "INPUT") {
      SP.print("INPUT\t\t");
      pinMode(DPORTS[i], INPUT);
    } 
    else if (DPORTS_OFF[i] == "INPUT,PULLUP") {
      SP.print("INPUT-PULUP\t\t");
      pinMode(DPORTS[i], INPUT_PULLUP);
    }
    if ((i == 4) || (i == 9) || (i == 14)) SP.println("");           
  }
  SP.println("");

  //Shut down analog ports
  SP.println("Shutting down analog ports: ");
  for (int i = 0; i <= 6; i++) {
    SP.print(APORTS_NAME[i]);
    SP.print("(");
    SP.print(APORTS[i]-15);
    SP.print("): ");
    if (APORTS_OFF[i] == "OUTPUT,HIGH") {
      SP.print("OUTPUT-HIGH\t\t");
      pinMode(APORTS[i], OUTPUT);
      digitalWrite(APORTS[i], HIGH);
    }
      else if (APORTS_OFF[i] == "OUTPUT,LOW") {
      SP.print("OUTPUT-LOW\t\t");
      pinMode(APORTS[i], OUTPUT);
      digitalWrite(APORTS[i], LOW);      
    }
      else if (APORTS_OFF[i] == "INPUT") {
      SP.print("INPUT\t\t");
      pinMode(APORTS[i], INPUT);
    } 
      else if (APORTS_OFF[i] == "INPUT,PULLUP") {
      SP.print("INPUT-PULUP\t\t");
      pinMode(APORTS[i], INPUT_PULLUP);
    }
    if ((i == 4) || (i == 6)) SP.println("");           
  }
  SP.println("");
}

//TURN ON SWITCHED 12V
void SW12V_ON () {
  if (SET_V12) {
    digitalWrite(PW1_V12, HIGH);
    digitalWrite(PW2_V12, HIGH);
    SP.println("SW12V\tON");   
  }
}

void SW12V_OFF () {
  if (SET_V12) {
    digitalWrite(PW1_V12, LOW);
    digitalWrite(PW2_V12, LOW);
    SP.println("SW12V\tOFF");
  }
}

//TURN ON SWITCHED 3,3V
void SW3V3_ON () {
  //CHANNEL 1
  if (SET_SW3V3) {
    digitalWrite(PW_3V3, HIGH);
    SP.println("SW3V3\tON");
    }
  if (SET_SW3V3A) {
    digitalWrite(PW_3V3_A, HIGH);
    SP.println("SW3V3A\tON");
  } 
  if (SET_SW3V3B) {
    digitalWrite(PW_3V3_B, HIGH);
    SP.println("SW3V3B\tON");
  }
}

//TURN OFF SWITCHED 3,3V
void SW3V3_OFF () {
  //CHANNEL 1
  if (SET_SW3V3) {
    digitalWrite(PW_3V3, LOW);
    SP.println("SW3V3\tOFF");
    }
  if (SET_SW3V3A) {
    digitalWrite(PW_3V3_A, LOW);
    SP.println("SW3V3A\tOFF");
  } 
  if (SET_SW3V3B) {
    digitalWrite(PW_3V3_B, LOW);
    SP.println("SW3V3B\tOFF");
  }
}

// PAYLOAD ENCODE U32
void Payload_U32 (uint32_t u32) {
  payload_temp[pctr] = (byte) ((u32 & 0xFF000000) >> 24 );
  pctr++;
  payload_temp[pctr] = (byte) ((u32 & 0x00FF0000) >> 16 );
  pctr++;
  payload_temp[pctr] = (byte) ((u32 & 0x0000FF00) >> 8  );
  pctr++;
  payload_temp[pctr] = (byte) ((u32 & 0X000000FF)       );
  pctr++;
}

// PAYLOAD ENCODE I32
void Payload_I32 (int32_t i32) {
  payload_temp[pctr] = (byte) ((i32 & 0xFF000000) >> 24 );
  pctr++;
  payload_temp[pctr] = (byte) ((i32 & 0x00FF0000) >> 16 );
  pctr++;
  payload_temp[pctr] = (byte) ((i32 & 0x0000FF00) >> 8  );
  pctr++;
  payload_temp[pctr] = (byte) ((i32 & 0X000000FF)       );
  pctr++;
}

// PAYLOAD ENCODE U24
void Payload_U24 (uint32_t u24) {
  payload_temp[pctr] = (byte) ((u24 & 0x00FF0000) >> 16 );
  pctr++;
  payload_temp[pctr] = (byte) ((u24 & 0x0000FF00) >> 8  );
  pctr++;
  payload_temp[pctr] = (byte) ((u24 & 0X000000FF)       );
  pctr++;
}

// PAYLOAD ENCODE I24
void Payload_I24 (int32_t i24) {
  payload_temp[pctr] = (byte) ((i24 & 0x00FF0000) >> 16 );
  pctr++;
  payload_temp[pctr] = (byte) ((i24 & 0x0000FF00) >> 8  );
  pctr++;
  payload_temp[pctr] = (byte) ((i24 & 0X000000FF)       );
  pctr++;
}

// PAYLOAD ENCODE U16
void Payload_U16 (uint16_t u16) {
  payload_temp[pctr] = highByte(u16);
  pctr++;
  payload_temp[pctr] = lowByte(u16);
  pctr++;
}

// PAYLOAD ENCODE I16
void Payload_I16 (int16_t i16) {
  payload_temp[pctr] = highByte(i16);
  pctr++;
  payload_temp[pctr] = lowByte(i16);
  pctr++;
}

// PAYLOAD ENCODE U8
void Payload_U8 (uint8_t u8) {
  payload_temp[pctr] = u8;
  pctr++;
}

// PAYLOAD ENCODE I8
void Payload_I8 (int8_t i8) {
  payload_temp[pctr] = i8;
  pctr++;
}  

//DATA PACKET PREPARE B0
void Packet10_B0_Prepare() {
  // -- CREATE VALUE DESIGNATOR FOR PACKET_DESIGNATOR -- TYPE 10
  // Defines content of Lora package. Each bit stands for a defined value - if TRUE has been transmitted, if FALSE hat not been transmitted
  //        | BYTE 0          Type    | BYTE 1          Type    | BYTE 2          Type    | BYTE 3          Type    | Power of 2 value associated with position
  // --------------------------------------------------------------------------------------------------------------------------------------
  // BIT 0  | BATV            U8      | AD24-CH0        I24     | SMP-I1B          I16 x3 | SMP-I3B          I16 x3 | 1
  // BIT 1  | BARO-T-B        I16,U24 | AD24-CH1        I24     | SMP-I1B_TEMP     I8     | SMP-I3A_TEMP     I8     | 2
  // BIT 2  | BARO-ALT        U16     | AD24-CH2        I24     | SMP-I2A          I16 x3 | NOT USED                | 4
  // BIT 3  | IMU-ACC-XYZ     I16 x3  | AD24-CH3        I24     | SMP-I2A_TEMP     I8     | NOT USED                | 6 
  // BIT 4  | IMU-GYR-XYZ     I16 x3  | AD12-CH0        I16     | SMP-I2B          I16 x3 | NOT USED                | 16
  // BIT 5  | IMU-MAG-XYZ     I16 x3  | AD12-CH1        I16     | SMP-I2B_TEMP     I8     | NOT USED                | 32
  // BIT 6  | INCL-XYZ        I24 x3  | SMP-I1A         I16 x3  | SMP-I3A          I16 x3 | NOT USED                | 64 
  // BIT 7  | INCL-TEMP       I16     | SMP-I1A_TEMP    I8      | SMP-I3A_TEMP     I8     | NOT USED                | 128

  // -- CREATE PAYLOAD
  // BYTE 0:    U8    Packet Size in Byte
  // BYTE 1/2:  U16   Packet Designator
  // BYTE 3:    U8    VALUE DESIGNATOR BYTE 0
  // BYTE 4:    U8    VALUE DESIGNATOR BYTE 1
  // BYTE 5:    U8    VALUE DESIGNATOR BYTE 2
  // BYTE 6:    U8    VALUE DESIGNATOR BYTE 3 
  // BYTE 7-54: DATA AS DEFINED IN VALUE DESIGNATOR 

  pctr = 7;
  vd0 = 0;

  // -- DATA FROM VALUE DESIGNATOR BYTE 0
  if (SET_BATV && BATV_TRANS) {
    //Add Battery Voltage U8 *10 0,0 .. 25,5
    Payload_U8((uint8_t)(round(AVG_BATV*10)));
    vd0 = vd0+1;    
  }
  if (SET_BARO && BARO_TRANS) {
    //Add Temperature I16 *100 -327,68 .. 327,67
    Payload_I16((int16_t)(round(AVG_TEMP*100)));
    //Add Air Pressure U24 *10000 0 .. 1677,7216
    Payload_U24((uint32_t)(round(AVG_BARO*10000)));
    vd0 = vd0+2;
  }
  if (SET_BARO && BARO_TRANS_ALT) {
    //Add Altitude I24 *100 -83886,08 .. 83886,07
    Payload_I24((int32_t)(round(AVG_ALTI*100)));    
    vd0 = vd0+4;
  }
  if (SET_IMU && IMU_TRANS_ACC) {
    //Add Acceleration X I24 *100 -83886,08 .. 83886,07 (milli-g)
    Payload_I24((int32_t)(round(AVG_IMUACCX*100)));
    Payload_I24((int32_t)(round(AVG_IMUACCY*100)));
    Payload_I24((int32_t)(round(AVG_IMUACCZ*100)));
    vd0 = vd0+8;
  }
  if (SET_IMU && IMU_TRANS_GYR) {
    //Add Gyro X I24 *1000 -8388,608 .. 8388,607 (dps)
    Payload_I24((int32_t)(round(AVG_IMUGYRX*1000)));
    Payload_I24((int32_t)(round(AVG_IMUGYRY*1000)));
    Payload_I24((int32_t)(round(AVG_IMUGYRZ*1000)));
    vd0 = vd0+16;
  }
  if (SET_IMU && IMU_TRANS_MAG) {
    //Add Magnetometer X I24 *1000 -8388,608 .. 8388,607 (µT)
    Payload_I24((int32_t)(round(AVG_IMUMAGX*1000)));
    Payload_I24((int32_t)(round(AVG_IMUMAGY*1000)));
    Payload_I24((int32_t)(round(AVG_IMUMAGZ*1000)));
    vd0 = vd0+32;
  }
  if (SET_INCL && INCL_TRANS_TILT) {
    //Add TILT X I24 *10000 -838,8608 .. 838,8607  
    Payload_I24((int32_t)(round(AVG_INCL_X*10000)));
    Payload_I24((int32_t)(round(AVG_INCL_Y*10000)));
    Payload_I24((int32_t)(round(AVG_INCL_Z*10000)));
    vd0 = vd0+64;
  }
  if (SET_INCL && INCL_TRANS_TEMP) {
    //Add Temperature I16 *100 -327,68 .. 327,67
    Payload_I16((int16_t)(round(AVG_INCL_TEMP*100)));
    vd0 = vd0+128;
  }
  //Write value designator byte 0
  payload_temp[3] = vd0;  
}

//DATA PACKET PREPARE B1
void Packet10_B1_Prepare() {
  // -- CREATE VALUE DESIGNATOR FOR PACKET_DESIGNATOR_TYPE 10 -- BYTE 1
  vd1 = 0; 

  // -- DATA FROM VALUE DESIGNATOR BYTE 1
  if (SET_AD24 && AD24_AIN0 && AD24_TRANS_AIN0) {
    //Add AD24 CHANNEL 0 DATA I24 *100000 -83,88608 .. 83,88607
    Payload_I24((int32_t)(round(AVG_AD24_AIN0*100000)));
    vd1 = vd1+1;
  }
  if (SET_AD24 && AD24_AIN1 && AD24_TRANS_AIN1) {
    //Add AD24 CHANNEL 1 DATA I24 *100000 -83,88608 .. 83,88607
    Payload_I24((int32_t)(round(AVG_AD24_AIN1*100000)));
    vd1 = vd1+2;
  }
  if (SET_AD24 && AD24_AIN2 && AD24_TRANS_AIN2) {
    //Add AD24 CHANNEL 2 DATA I24 *100000 -83,88608 .. 83,88607
    Payload_I24((int32_t)(round(AVG_AD24_AIN2*100000)));
    vd1 = vd1+4;
  }    
  if (SET_AD24 && AD24_AIN3 && AD24_TRANS_AIN3) {
    //Add AD24 CHANNEL 3 DATA I24 *100000 -83,88608 .. 83,88607
    Payload_I24((int32_t)(round(AVG_AD24_AIN3*100000)));
    vd1 = vd1+8;
  }
  if (SET_AD12 && AD12_AIN0 && AD12_TRANS_AIN0) {
    //Add AD12 CHANNEL 0 DATA I16 *1000 -32,768 .. 32,767
    Payload_I16((int16_t)(round(AVG_AD12_AIN0*1000)));
    vd1 = vd1+16;
  }
  if (SET_AD12 && AD12_AIN1 && AD12_TRANS_AIN1) {
    //Add AD12 CHANNEL 0 DATA I16 *1000 -32,768 .. 32,767
    Payload_I16((int16_t)(round(AVG_AD12_AIN1*1000)));
    vd1 = vd1+32;
  }
  if (SET_SMP && I1A && SMP_TRANS_I1A) {
    //Add SMP-I1A TILT DATA I16 *100 -327,68 .. 327,67
    Payload_I16((int16_t)(round(AVG_SMP_X[0]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Y[0]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Z[0]*100)));
    vd1 = vd1+64;
  }
  if (SET_SMP && I1A && SMP_TRANS_I1A_TEMP) {
    //Add SMP-I1A TEMPERATURE DATA I8 *1 -128 .. 127
    Payload_I8((int8_t)(AVG_SMP_TEMP[0]));    
    vd1 = vd1+128;
  }
  //Write value designator byte 0
  payload_temp[4] = vd1;  
}  

//DATA PACKET PREPARE B2
void Packet10_B2_Prepare() {
  // -- CREATE VALUE DESIGNATOR FOR PACKET_DESIGNATOR_TYPE 10 -- BYTE 2
  vd2 = 0;

  // -- DATA FROM VALUE DESIGNATOR BYTE 2  
  if (SET_SMP && I1B && SMP_TRANS_I1B) {
    //Add SMP-I1B TILT DATA I16 *100 -327,68 .. 327,67    
    Payload_I16((int16_t)(round(AVG_SMP_X[1]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Y[1]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Z[1]*100)));
    vd2 = vd2+1;
  }
  if (SET_SMP && I1B && SMP_TRANS_I1B_TEMP) {
    //Add SMP-I1B TEMPERATURE DATA I8 *1 -128 .. 127
    Payload_I8((int8_t)(AVG_SMP_TEMP[1]));
    vd2 = vd2+2;
  }
  if (SET_SMP && I2A && SMP_TRANS_I1B) {
    //Add SMP-I2A TILT DATA I16 *100 -327,68 .. 327,67
    Payload_I16((int16_t)(round(AVG_SMP_X[2]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Y[2]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Z[2]*100)));
    vd2 = vd2+4;
  }
  if (SET_SMP && I2A && SMP_TRANS_I1B_TEMP) {
    //Add SMP-I2A TEMPERATURE DATA I8 *1 -128 .. 127
    Payload_I8((int8_t)(AVG_SMP_TEMP[2]));
    vd2 = vd2+8;
  }
    if (SET_SMP && I2B && SMP_TRANS_I1B) {
    //Add SMP-I2B TILT DATA I16 *100 -327,68 .. 327,67
    Payload_I16((int16_t)(round(AVG_SMP_X[3]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Y[3]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Z[3]*100)));
    vd2 = vd2+16;
  }
  if (SET_SMP && I2B && SMP_TRANS_I2B_TEMP) {
    //Add SMP-I2B TEMPERATURE DATA I8 *1 -128 .. 127
    Payload_I8((int8_t)(AVG_SMP_TEMP[3]));
    vd2 = vd2+32;
  }
  if (SET_SMP && I3A && SMP_TRANS_I3A) {
    //NOT IMPLEMENTED YET -- SMP-I3A TILT DATA I16 *100 -327,68 .. 327,67
    Payload_I16((int16_t)(round(AVG_SMP_X[4]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Y[4]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Z[4]*100)));
    vd2 = vd2+64;
  }
  if (SET_SMP && I3A && SMP_TRANS_I3A_TEMP) {
    //NOT IMPLEMENTED YET -- Add SMP-I3A TEMPERATURE DATA I8 *1 -128 .. 127
    Payload_I8((int8_t)(AVG_SMP_TEMP[4]));
    vd2 = vd2+128;
  }
  //Write value designator byte 0
  payload_temp[5] = vd2;
}

//DATA PACKET PREPARE B3
void Packet10_B3_Prepare() {
  // -- CREATE VALUE DESIGNATOR FOR PACKET_DESIGNATOR_TYPE 10 -- BYTE 3
  vd3 = 0;
  if (SET_SMP && I3B && SMP_TRANS_I3B) {
    //NOT IMPLEMENTED YET -- SMP-I3B TILT DATA I16 *100 -327,68 .. 327,67
    Payload_I16((int16_t)(round(AVG_SMP_X[5]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Y[5]*100)));
    Payload_I16((int16_t)(round(AVG_SMP_Z[5]*100)));
    vd3 = vd3+1;
  }
  if (SET_SMP && I3B && SMP_TRANS_I3B_TEMP) {
    //NOT IMPLEMENTED YET -- Add SMP-I3B TEMPERATURE DATA I8 *1 -128 .. 127
    Payload_I8((int8_t)(AVG_SMP_TEMP[5]));
    vd3 = vd3+2;
  }
  payload_temp[6] = vd3;
}  

//DATA PACKET FINALIZE
void Packet_Finalize() {
  // -- Create leading bytes
  payload_temp[0] = ((uint8_t)(pctr));
  payload_temp[1] = highByte((int16_t)(PACKET_DESIGNATOR));
  payload_temp[2] = lowByte((int16_t)(PACKET_DESIGNATOR));
  // -- Print data
  SP.println("");
  sprintf(report, "SYSTEM\tValue Designators:\t%d\t%d\t%d\t%d", vd0, vd1, vd2, vd3);
  SP.println(report);
  SP.print("SYSTEM\tData Package: "); 
  for (int i = 0; i < pctr; i++) {
    sprintf(report, "%02x", payload_temp[i]);
    SP.print(report);
  }
  SP.println("");
  SP.print("SYSTEM\tData Package Size: ");
  SP.println(pctr);
  if (pctr > 54) SP.println("SYSTEM\tWARNING! DATA PACKAGE IS LARGER THAN 54 Bytes!");
}  

//LORA COMMUNICATION INITIALIZE
//function to setup the lora modem
void IRloramodemSetup () {
  SP.println("LORA\tINITIALIZING LORA COMMUNICATION...");
  int lora_connect_cnt = 1;
  lora_connect:
  SP.print("LORA\tNetwork Connection: Band used: ");
  if (LORA_BAND == EU868) SP.println("EU868");
  else if (LORA_BAND == US915) SP.println("US915");
  else if (LORA_BAND == AS923) SP.println("AS923");

  SP.print("LORA\tLoRa Network Connection: Initializing LoRa Module... ");
  if (!modem.begin(LORA_BAND)) SP.println("Failed to start module");
  else 
  {
    SP.println("OK");
    String ArdVers = modem.version();
    String DevEUI = modem.deviceEUI();
    SP.print("LORA\tYour module version is: ");
    SP.println(ArdVers);
    SP.print("LORA\tYour device EUI is: ");
    SP.println(DevEUI);

    SP.print("LORA\tLoRa Network Connection: Use default channel plan: ");
    SP.print((bool)LORA_CH_DEF);
    // Print default channels configuration
    SP.print(" (Default channel mask: ");
    SP.print(modem.getChannelMask());
    SP.println(")");

    if (!LORA_CH_DEF)
    {
      SP.println("LORA\tLora Network Connection: Setting up custom channel plan...");
      SP.println("LORA\tDisabling all channels... ");  
      for (unsigned int i = 0; i < 72; i++) 
      {
      modem.disableChannel(i);
      }

      // Print enmpty channels configuration
      SP.print("LORA\tChannel mask: ");
      SP.println(modem.getChannelMask());

      SP.print("LORA\tEnabling custiom channels... ");  
      int x[] = LORA_CH_ACT;
      for (int i : x)
      {
        modem.enableChannel(i);
        SP.print(i);
        SP.print(", ");
      }
      // Print new channels configuration
      SP.print("LORA\tChannel mask: ");
      SP.println(modem.getChannelMask());
    }

    modem.minPollInterval(10);
    SP.print("LORA\tLoRa Network Connection: ADR active: ");
    SP.println((bool)LORA_ADR);
    modem.setADR(LORA_ADR);
    if (LORA_ADR == false) 
    {
      SP.print("LORA\tLoRa Network Connection: DR level: ");
      SP.println((int)LORA_DR);
      modem.dataRate(LORA_DR);
    }    
    
    SP.print("LORA\tLoRa Network Connection: Joining Server using OTAA (This may take a while)... ");
    lora_connected = modem.joinOTAA(appEui, appKey, "", LORA_TIMEOUT*1000);                                             // Attempt to Join LoRaWAN Network using Over-The-Air-Activation (OTAA)
    if (lora_connected)
    {
      SP.println("Connection successful!");
      goto lora_connect_end;
    }
    else
    {
      SP.print("ERROR! Connection failed (repetion ");
      SP.print(lora_connect_cnt);
      SP.print(" of 3)");
      if (lora_connect_cnt >= 3) 
      {
        SP.println("");
        goto lora_connect_end;
      }
      else 
      {
        SP.println("; Trying to connect again in 10 seconds...");
        lora_connect_cnt++;
        delay (10000);
        goto lora_connect;
      }
    }
  }
  lora_connect_end:
  SP.println("");
}

