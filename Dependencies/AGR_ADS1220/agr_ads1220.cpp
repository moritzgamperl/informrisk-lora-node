/////////////////////////////////////////////////////////////////////////////////////////
//
// Arduino Library for the ADS1220 breakout board (Olimex, Protocentral etc.)
//
// Based on the Code from Ashwin Whitchurch, Copyright (c) 2018 ProtoCentral (https://github.com/Protocentral/Protocentral_ADS1220/)
//
// Edited by: John Singer, AlpGeorisk
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "agr_ads1220.h"
#include <Arduino.h>
#include <SPI.h>

//--------------------------------------------------------------------------------
// INITIALIZATION
//-------------------------------------------------------------------------------- 
// BEGIN / INITIALIZE ADS1220
//--------------------------------------------------------------------------------
bool agr_ads1220::begin(uint8_t cs_pin, uint8_t drdy_pin)
{
  //Define variables
    bool init = false;
    tmp_config_reg0 = 0;   
    tmp_config_reg1 = 0;   
    tmp_config_reg2 = 0;   
    tmp_config_reg3 = 0;
  
	//Define digital pins
  if (ads1220debug) {Serial1.println("ADS1220 - Initialization: Defining digital pins...");}
    def_drdy_pin=drdy_pin;
    def_cs_pin=cs_pin;

    pinMode(def_cs_pin, OUTPUT);
    pinMode(def_drdy_pin, INPUT);
	
	//Setup SPI communication
  if (ads1220debug) {Serial1.println("ADS1220 - Initialization: Setting up SPI communication...");}
	SPI.begin();
    delay(100);
    reset();
    delay(100);

  //Write Registers with current values (initialized with default values)
  if (ads1220debug) {
    Serial1.println("ADS1220 - Initialization: Writing default configuration to device...");
    config_cur_print();
    }
    write_reg0(cur_config_reg0);
    write_reg1(cur_config_reg1);
    write_reg2(cur_config_reg2);
    write_reg3(cur_config_reg3);
    delay(100);

  //Read back registers
  if (ads1220debug) {
    Serial1.println("ADS1220 - Initialization: Reading configuration from device...");
    config_act_print();
    }
    tmp_config_reg0 = readRegister(CONFIG_REG0_ADDRESS);
    tmp_config_reg1 = readRegister(CONFIG_REG1_ADDRESS);
    tmp_config_reg2 = readRegister(CONFIG_REG2_ADDRESS);
    tmp_config_reg3 = readRegister(CONFIG_REG3_ADDRESS);

  //Compare write/read to check if communication is successful
	  if ((cur_config_reg0 == tmp_config_reg0) && (cur_config_reg1 == tmp_config_reg1) && (cur_config_reg2 == tmp_config_reg2) && (cur_config_reg3 == tmp_config_reg3)) {
		  init = true;
      if (ads1220debug) {Serial1.println("ADS1220 - Initialization: Configuration write check OK!");} 
	    }
    else {
		  init = false;
      if (ads1220debug) {Serial1.println("ADS1220 - Initialization: Configuration write check ERROR!");}
	    }

  //Reset temporary variables
  tmp_config_reg0 = 0;
  tmp_config_reg1 = 0;
  tmp_config_reg2 = 0;
  tmp_config_reg3 = 0;

	return init;
}

//CALIBRATE ADS1220 (Offset)
//--------------------------------------------------------------------------------
void agr_ads1220::calibrate(void)
{
  //Reset any previous calibration
  if (ads1220debug) {Serial1.println("ADS1220 - Calibration: Resetting calibration");}
    ads1220_offset = 0;

  //BACKUP CURRENT SETTINGS
  config_backup();
  if (ads1220debug) {
    config_cur_print();
    config_act_print();
    config_tmp_print();
    }
    
    
  //SETUP DEVICE FOR CALIBRATION
  if (ads1220debug) {Serial1.println("ADS1220 - Calibration: Configuring device for calibration");}
    write_reg0(0xE0);    //AINp and AINn shorted to (AVDD + AVSS) / 2; Gain 1; PGA enabled
    write_reg1(0x00);    //DR=20 SPS, Mode=Normal, Conv mode=Single_Shot, Temp Sensor disabled, Current Source off
    write_reg2(0x10);    //Default settings: Vref internal, 50/60Hz rejection, power open, IDAC off
    write_reg3(0x00);    //Default settings: IDAC1 disabled, IDAC2 disabled, DRDY pin only

  if (ads1220debug) {
    config_cur_print();
    config_act_print();
    config_tmp_print();
    }
    delay(100);
  
  //RUN CALIBRATION
  if (ads1220debug) {Serial1.println("ADS1220 - Calibration: Reading calibration data..");}
  int itot = 100;
  int32_t ssdatasum = 0;
  for (int i = 0; i < itot; i++) {
    int32_t ssdata = read_single_shot();
    if (ads1220debug) {
      Serial1.print(ssdata);
      Serial1.println("; ");
      }
    ssdatasum = ssdatasum + ssdata;
  }
  ads1220_offset = ssdatasum / itot;
  if (ads1220debug) {
    Serial1.print("Offset: ");
    Serial1.println(ads1220_offset);
    Serial1.print(dac2mv(ads1220_offset),3); // Output as MilliVolt
    Serial1.println(" mV");
    }
  
  //Restore previous settings
  if (ads1220debug) {Serial1.println("ADS1220 - Calibration: Restoring previous device configuration");}
  config_restore();
  if (ads1220debug) {
    config_cur_print();
    config_act_print();
    config_tmp_print();
    }
}

//--------------------------------------------------------------------------------
// CONFIGURATION
//-------------------------------------------------------------------------------- 
// WRITE COMPLETE REGISTER0 (use this command to make sure settings storage (cur_config_reg0; cur_gain) is updated)
//--------------------------------------------------------------------------------
void agr_ads1220::write_reg0(uint8_t reg0)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuring Register 0");}
  cur_config_reg0 = reg0;
  writeRegister(CONFIG_REG0_ADDRESS,cur_config_reg0);
  //read gain setting & set cur_gain
  uint8_t gain = (cur_config_reg0 & REG0_GAIN_MASK);
  if (gain == 0x00) {cur_gain = 1;}
  else if (gain == 0x02) {cur_gain = 2;}
  else if (gain == 0x04) {cur_gain = 4;}
  else if (gain == 0x06) {cur_gain = 8;}
  else if (gain == 0x08) {cur_gain = 16;}
  else if (gain == 0x0A) {cur_gain = 32;}
  else if (gain == 0x0C) {cur_gain = 64;}
  else if (gain == 0x0E) {cur_gain = 128;}
}

// WRITE COMPLETE REGISTER1 (use this command to make sure settings storage (cur_config_reg0) is updated)
//--------------------------------------------------------------------------------
void agr_ads1220::write_reg1(uint8_t reg1)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuring Register 1");}
  cur_config_reg1 = reg1;
  writeRegister(CONFIG_REG1_ADDRESS,cur_config_reg1);
}

// WRITE COMPLETE REGISTER2 (use this command to make sure settings storage (cur_config_reg0; cur_vref) is updated)
//--------------------------------------------------------------------------------
void agr_ads1220::write_reg2(uint8_t reg2)
{ 
  if (ads1220debug) {Serial1.println("ADS1220 - Configuring Register 2");}
  cur_config_reg2 = reg2;
  writeRegister(CONFIG_REG2_ADDRESS,cur_config_reg2);
  //read vref setting & set cur_vref
  uint8_t vref = (cur_config_reg2 & REG2_VREF_MASK);
  if (vref == 0x00) {cur_vref = 2.048;}
  else if (vref == 0x40) {cur_vref = vref0;}
  else if (vref == 0x80) {cur_vref = vref1;}
  else if (vref == 0xC0) {cur_vref = avdd;}  
}

// WRITE COMPLETE REGISTER3 (use this command to make sure settings storage (cur_config_reg0) is updated)
//--------------------------------------------------------------------------------
void agr_ads1220::write_reg3(uint8_t reg3)
{
   if (ads1220debug) {Serial1.println("ADS1220 - Configuring Register 3");}
  cur_config_reg3 = reg3;
  writeRegister(CONFIG_REG3_ADDRESS,cur_config_reg3);
}

//RESET TO DEFAULT SETTINGS
//--------------------------------------------------------------------------------
void agr_ads1220::reset_default(void)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Resetting device (--> default configuration)");}
  reset();
  delay(100);
  write_reg0(REG0_DEFAULT);
  write_reg1(REG1_DEFAULT);
  write_reg2(REG2_DEFAULT);
  write_reg3(REG3_DEFAULT); 
}

//BACKUP SETTINGS (temporarily)
//--------------------------------------------------------------------------------
void agr_ads1220::config_backup(void)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Backing up configuration");}
  tmp_config_reg0 = readRegister(CONFIG_REG0_ADDRESS);
  tmp_config_reg1 = readRegister(CONFIG_REG1_ADDRESS);
  tmp_config_reg2 = readRegister(CONFIG_REG2_ADDRESS);
  tmp_config_reg3 = readRegister(CONFIG_REG3_ADDRESS);  
}

//Restore Settings (from temporary storage)
//--------------------------------------------------------------------------------
void agr_ads1220::config_restore(void)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Restoring configuration");}
  write_reg0(tmp_config_reg0);
  write_reg1(tmp_config_reg1);
  write_reg2(tmp_config_reg2);
  write_reg3(tmp_config_reg3); 
}

// Configure MUX Channels
//--------------------------------------------------------------------------------
void agr_ads1220::config_mux(int conf_mux)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: MUX");}
  cur_config_reg0 &= ~REG0_MUX_MASK;
  cur_config_reg0 |= conf_mux;
  writeRegister(CONFIG_REG0_ADDRESS,cur_config_reg0);
}

// Configure PGA gain
//--------------------------------------------------------------------------------
void agr_ads1220::config_gain(int conf_gain)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Gain");}
  cur_config_reg0 &= ~REG0_GAIN_MASK;
  cur_config_reg0 |= conf_gain;
  writeRegister(CONFIG_REG0_ADDRESS,cur_config_reg0);
  if (conf_gain == 0x00) {cur_gain = 1;}
  else if (conf_gain == 0x02) {cur_gain = 2;}
  else if (conf_gain == 0x04) {cur_gain = 4;}
  else if (conf_gain == 0x06) {cur_gain = 8;}
  else if (conf_gain == 0x08) {cur_gain = 16;}
  else if (conf_gain == 0x0A) {cur_gain = 32;}
  else if (conf_gain == 0x0C) {cur_gain = 64;}
  else if (conf_gain == 0x0E) {cur_gain = 128;}
}

// Configure PGA ON/OFF
//--------------------------------------------------------------------------------
void agr_ads1220::config_pga(int conf_pga)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: PGA ON/OFF");}
  cur_config_reg0 &= ~REG0_PGA_MASK;
  cur_config_reg0 |= conf_pga;
  writeRegister(CONFIG_REG0_ADDRESS,cur_config_reg0);
}

// Configure DATA RATE
//--------------------------------------------------------------------------------
void agr_ads1220::config_datarate(int conf_dr)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Data Rate");}
  cur_config_reg1 &= ~REG1_DR_MASK;
  cur_config_reg1 |= conf_dr;
  writeRegister(CONFIG_REG1_ADDRESS,cur_config_reg1);
}

// Configure Conversion Mode
//--------------------------------------------------------------------------------
void agr_ads1220::config_conmode(int conf_cm)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Conversion Mode");}
  cur_config_reg1 &= ~REG1_CM_MASK;
  cur_config_reg1 |= conf_cm;
  writeRegister(CONFIG_REG1_ADDRESS,cur_config_reg1);
}

// Configure Operating Mode
//--------------------------------------------------------------------------------
void agr_ads1220::config_opmode(int conf_op)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Operating Mode");}
  cur_config_reg1 &= ~REG1_OP_MASK;
  cur_config_reg1 |= conf_op;
  writeRegister(CONFIG_REG1_ADDRESS,cur_config_reg1);
}

// Configure Temperature Mode
//--------------------------------------------------------------------------------
void agr_ads1220::config_tempmode(int conf_temp)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Temperature Mode");}
  cur_config_reg1 &= ~REG1_TEMP_MASK;
  cur_config_reg1 |= conf_temp;
  writeRegister(CONFIG_REG1_ADDRESS,cur_config_reg1);
}

// Configure Burnout Current Sources
//--------------------------------------------------------------------------------
void agr_ads1220::config_burnout(int conf_bo)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Burnout Current Source");}
  cur_config_reg1 &= ~REG1_BO_MASK;
  cur_config_reg1 |= conf_bo;
  writeRegister(CONFIG_REG1_ADDRESS,cur_config_reg1);
}

// Configure Voltage Reference
//--------------------------------------------------------------------------------
void agr_ads1220::config_vref(int conf_vref)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Voltage Reference");}
  cur_config_reg2 &= ~REG2_VREF_MASK;
  cur_config_reg2 |= conf_vref;
  writeRegister(CONFIG_REG2_ADDRESS,cur_config_reg2);
  if (conf_vref == 0x00) {cur_vref = 2.048;}
  else if (conf_vref == 0x40) {cur_vref = vref0;}
  else if (conf_vref == 0x80) {cur_vref = vref1;}
  else if (conf_vref == 0xC0) {cur_vref = avdd;}
}

// Configure FIR filter
//--------------------------------------------------------------------------------
void agr_ads1220::config_fir(int conf_fir)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: FIR Filter");}
  cur_config_reg2 &= ~REG2_FIR_MASK;
  cur_config_reg2 |= conf_fir;
  writeRegister(CONFIG_REG2_ADDRESS,cur_config_reg2);
}

// Configure low side power switch
//--------------------------------------------------------------------------------
void agr_ads1220::config_lspsw(int conf_lspsw)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: Low side power switch");}
  cur_config_reg2 &= ~REG2_LSPSW_MASK;
  cur_config_reg2 |= conf_lspsw;
  writeRegister(CONFIG_REG2_ADDRESS,cur_config_reg2);
}

// Configure IDAC Current
//--------------------------------------------------------------------------------
void agr_ads1220::config_idac_cur(int conf_idaccur)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: IDAC Current");}
  cur_config_reg2 &= ~REG2_IDACCUR_MASK;
  cur_config_reg2 |= conf_idaccur;
  writeRegister(CONFIG_REG2_ADDRESS,cur_config_reg2);
}

// Configure IDAC1 Channel
//--------------------------------------------------------------------------------
void agr_ads1220::config_idac1(int conf_idac1)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: IDAC1 Channel");}
  cur_config_reg3 &= ~REG3_IDAC1_MASK;
  cur_config_reg3 |= conf_idac1;
  writeRegister(CONFIG_REG3_ADDRESS,cur_config_reg3);
}

// Configure IDAC2 Channel
//--------------------------------------------------------------------------------
void agr_ads1220::config_idac2(int conf_idac2)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: IDAC2 Channel");}
  cur_config_reg3 &= ~REG3_IDAC2_MASK;
  cur_config_reg3 |= conf_idac2;
  writeRegister(CONFIG_REG3_ADDRESS,cur_config_reg3);
}

// Configure DRDY Mode
//--------------------------------------------------------------------------------
void agr_ads1220::config_drdy(int conf_drdy)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Configuration: DataReady PIN Mode");}
  cur_config_reg3 &= ~REG3_DRDY_MASK;
  cur_config_reg3 |= conf_drdy;
  writeRegister(CONFIG_REG3_ADDRESS,cur_config_reg3);
}


//--------------------------------------------------------------------------------
// READ DATA
//-------------------------------------------------------------------------------- 
// READ SINGLE SHOT
//--------------------------------------------------------------------------------
int32_t agr_ads1220::read_single_shot(void)
{ 
  if (ads1220debug) {Serial1.println("ADS1220 - Single Shot Measurement");}
  //Initialize measurement
  start_conv();
  delay(SINGLE_SHOT_WAKEUP); 
  
  return get_data();
}

// START CONTINUOUS MEASUREMENT MODE (for high frequency measurements)
//--------------------------------------------------------------------------------
void agr_ads1220::meas_start(void)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Starting continuous mode");}
  config_conmode(CM_CONTINUOUS);
  start_conv();
  delay(SINGLE_SHOT_WAKEUP);
}

// STOP CONTINUOUS MEASUREMENT MODE (go back to single shot mode)
//--------------------------------------------------------------------------------
void agr_ads1220::meas_stop(void)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Stopping continuous mode");}
  config_conmode(CM_SINGLE_SHOT);
}

// READ CONTINUOUS MODE (Wait for new data & read)
//--------------------------------------------------------------------------------
int32_t agr_ads1220::get_data()
{ 
  if (ads1220debug) {Serial1.println("ADS1220 - Reading data");}
  static byte SPI_Buff[3];
  int32_t mResult32=0;
  long int bit24;
  
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

  //Wait for DRDY to transition low
  while((digitalRead(def_drdy_pin)) == HIGH) {} 
  
  //activate SPI-COM
  digitalWrite(def_cs_pin,LOW);  //Take CS low
  delay(SPI_TIME_START);
  //Read data
  for (int i = 0; i < 3; i++)
  {
    SPI_Buff[i] = SPI.transfer(SPI_MASTER_DUMMY);
  }

  //deactivate SPI-COM
  delay(SPI_TIME_END);
  digitalWrite(def_cs_pin,HIGH);  //  Clear CS to high
  
  SPI.endTransaction();

  //Convert data
  // Converting 3 bytes to a 24 bit int
  bit24 = SPI_Buff[0];
  bit24 = (bit24 << 8) | SPI_Buff[1];
  bit24 = (bit24 << 8) | SPI_Buff[2];  
  
  // Converting 24 bit two's complement to 32 bit two's complement
  bit24= ( bit24 << 8 );
  mResult32 = ( bit24 >> 8 );  

  //Apply calibration
  mResult32 = mResult32 - ads1220_offset;
    
  return mResult32;
}

// CONVERT DATA TO MILLIVOLT
//--------------------------------------------------------------------------------
float agr_ads1220::dac2mv(int32_t mResult32)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Data Conversion to Voltage [mV]");}
  float mVout = (float)((mResult32*cur_vref*1000/cur_gain)/FULL_SCALE);

  return mVout;
}

// CONVERT DATA TO TEMPERATURE
//--------------------------------------------------------------------------------
float agr_ads1220::dac2temp(int32_t mResult32)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Data Conversion to temperature [°C]");}
  float temp = 0;
  mResult32 = (mResult32 >> 10);
  if (bitRead(mResult32,13) == 0){temp = mResult32 * 0.03125;}
  else 
    {
    mResult32 = mResult32 - 1;
    mResult32 = (mResult32 << 18);
    mResult32 = ~mResult32;
    mResult32 = (mResult32 >> 18);
    temp = mResult32 * -0.03125;
    }
  
  return temp;
}

// CHECK SENSOR ON CURRENT CHANNEL
//--------------------------------------------------------------------------------
int agr_ads1220::sensorcheck(void)
{
  if (ads1220debug) {Serial1.println("ADS1220 - Sensor Test");}
  //Define variables
  int result = -1;
  
  //Backup current settings
  config_backup();
  if (ads1220debug) {
    config_cur_print();
    config_act_print();
    config_tmp_print();
    }

  //Setup device for sensor-test
  if (ads1220debug) {Serial1.println("ADS1220 - Sensor Test: Configuring test");}
  config_burnout(BO_OFF);
  config_conmode(CM_SINGLE_SHOT);
  config_vref(VREF_AVDD);

  //Setup device for sensor-test
  if (ads1220debug) {Serial1.println("ADS1220 - Sensor Test: Running test");}
  float meas = dac2mv(read_single_shot());
  if (meas > (0.9*cur_vref)) {result = 1;}
  else if (meas < (0.1*cur_vref)) {result = 2;}
  else {result = 0;}

  //Restore previous settings
  config_restore();
  if (ads1220debug) {
    config_cur_print();
    config_act_print();
    config_tmp_print();
    }
  
  //Output result
  if (ads1220debug) {
    Serial1.print("ADS1220 - Sensor Test result: ");
    if (result == 1) {Serial1.println("Open circuit detected!");}
    else if (result == 2) {Serial1.println("Shorted circuit detected!");}
    else {Serial1.println("Sensor OK!");}
    }

  return result;
}

//--------------------------------------------------------------------------------
// BASIC COM FUNCTIONS
//--------------------------------------------------------------------------------
// READ REGISTER from ADS1220
//--------------------------------------------------------------------------------
uint8_t agr_ads1220::readRegister(uint8_t address)
{
  uint8_t data;
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(def_cs_pin,LOW);
  delay(SPI_TIME_START);
  SPI.transfer(RREG|(address<<2));
  data = SPI.transfer(SPI_MASTER_DUMMY);
  delay(SPI_TIME_END);
  digitalWrite(def_cs_pin,HIGH);
  SPI.endTransaction();
  

  return data;
}

// WRITE REGISTER to ADS1220
//--------------------------------------------------------------------------------
void agr_ads1220::writeRegister(uint8_t address, uint8_t value)
{
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(def_cs_pin,LOW);
  delay(SPI_TIME_START);
  SPI.transfer(WREG|(address<<2));
  SPI.transfer(value);
  delay(SPI_TIME_END);
  digitalWrite(def_cs_pin,HIGH);
  SPI.endTransaction();
}

// WRITE COMMAND to ADS1220
//--------------------------------------------------------------------------------
void agr_ads1220::SPI_Command(unsigned char data_in)
{
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(def_cs_pin,LOW);
  delay(SPI_TIME_START);
  SPI.transfer(data_in);
  delay(SPI_TIME_END);
  digitalWrite(def_cs_pin,HIGH);
  SPI.endTransaction();
}

//--------------------------------------------------------------------------------
// BASIC COMMANDS
//-------------------------------------------------------------------------------- 
// RESET
//--------------------------------------------------------------------------------
void agr_ads1220::reset()
{
  SPI_Command(RESET);
}

// START CONVERSION
//--------------------------------------------------------------------------------
void agr_ads1220::start_conv()
{
  SPI_Command(START);
}

// RDATA
//--------------------------------------------------------------------------------
void agr_ads1220::rdata()
{
  SPI_Command(RDATA);
}

// POWER DOWN
//--------------------------------------------------------------------------------
void agr_ads1220::powerdown()
{
  SPI_Command(POWERDOWN);
}

//--------------------------------------------------------------------------------
// DEBUG
//-------------------------------------------------------------------------------- 
// READ and Display ACTUAL DEVICE configuration on Serial1
//--------------------------------------------------------------------------------
void agr_ads1220::config_act_print(void) 
{
    uint8_t a_config_reg0 = readRegister(CONFIG_REG0_ADDRESS);
    uint8_t a_config_reg1 = readRegister(CONFIG_REG1_ADDRESS);
    uint8_t a_config_reg2 = readRegister(CONFIG_REG2_ADDRESS);
    uint8_t a_config_reg3 = readRegister(CONFIG_REG3_ADDRESS);
    Serial1.print("ADS1220 - Actual Device Settings: Reg0: ");
    Serial1.print(a_config_reg0,HEX);
    Serial1.print("; Reg1: ");
    Serial1.print(a_config_reg1,HEX);
    Serial1.print("; Reg2: ");
    Serial1.print(a_config_reg2,HEX);
    Serial1.print("; Reg3: ");
    Serial1.println(a_config_reg3,HEX);
}

// Display configuration settings on Serial1
//--------------------------------------------------------------------------------
void agr_ads1220::config_cur_print(void) 
{
    Serial1.print("ADS1220 - Config Device Settings: Reg0: ");
    Serial1.print(cur_config_reg0,HEX);
    Serial1.print("; Reg1: ");
    Serial1.print(cur_config_reg1,HEX);
    Serial1.print("; Reg2: ");
    Serial1.print(cur_config_reg2,HEX);
    Serial1.print("; Reg3: ");
    Serial1.println(cur_config_reg3,HEX);
}

// Display backup settings on Serial1
//--------------------------------------------------------------------------------
void agr_ads1220::config_tmp_print(void) 
{
    Serial1.print("ADS1220 - Backup Device Settings: Reg0: ");
    Serial1.print(tmp_config_reg0,HEX);
    Serial1.print("; Reg1: ");
    Serial1.print(tmp_config_reg1,HEX);
    Serial1.print("; Reg2: ");
    Serial1.print(tmp_config_reg2,HEX);
    Serial1.print("; Reg3: ");
    Serial1.println(tmp_config_reg3,HEX);
}
