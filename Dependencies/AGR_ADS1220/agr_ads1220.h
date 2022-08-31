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

#ifndef agr_ads1220		//makes sure library is only added once
#include <Arduino.h>
#include <SPI.h>

//ADS1220 SPI Paramters
#define ADS1220_SPI_SPEED		2000000
#define ADS1220_SPI_DATAORDER 	MSBFIRST
#define ADS1220_SPI_MODE		SPI_MODE1


//ADS1220 Timings
#define SPI_TIME_START 10        //Delay time after CS has been set to low (at beginning of SPI communication) -- MIN 50 ns.
#define SPI_TIME_END 10          //Delay time before CS is set to high (at end of SPI communication) -- MIN 25 ns.
#define SINGLE_SHOT_WAKEUP 50   //Delay to allow the internal oscillator to power up (manual p. 28 - Output data rate): 50 ms in normal mode; 25 ms in turbo mode; can be set to 0 if external clock is used.

//ADS1220 SPI commands
#define SPI_MASTER_DUMMY    0xFF
#define RESET               0x06		//Send the RESET command (06h) to make sure the ADS1220 is properly reset after power-up
#define START               0x08		//Send the START/SYNC command (08h) to start converting in continuous conversion mode
#define RDATA               0x10
#define POWERDOWN           0x02   
#define WREG                0x40
#define RREG                0x20

//Config registers
#define CONFIG_REG0_ADDRESS 0x00
#define CONFIG_REG1_ADDRESS 0x01
#define CONFIG_REG2_ADDRESS 0x02
#define CONFIG_REG3_ADDRESS 0x03

//Config register MASKS
#define REG0_MUX_MASK     0xF0
#define REG0_GAIN_MASK    0x0E
#define REG0_PGA_MASK     0x01
#define REG1_DR_MASK      0xE0
#define REG1_CM_MASK      0x04
#define REG1_OP_MASK      0x18
#define REG1_TEMP_MASK    0x02
#define REG1_BO_MASK      0x01
#define REG2_VREF_MASK    0xC0
#define REG2_FIR_MASK     0x30
#define REG2_LSPSW_MASK   0x08
#define REG2_IDACCUR_MASK 0x07
#define REG3_IDAC1_MASK   0xE0
#define REG3_IDAC2_MASK   0x1C  
#define REG3_DRDY_MASK    0x02

//Config MUX settings (ALL)
#define MUX_AIN0_AIN1   0x00
#define MUX_AIN0_AIN2   0x10
#define MUX_AIN0_AIN3   0x20
#define MUX_AIN1_AIN2   0x30
#define MUX_AIN1_AIN3   0x40
#define MUX_AIN2_AIN3   0x50
#define MUX_AIN1_AIN0   0x60
#define MUX_AIN3_AIN2   0x70
#define MUX_AIN0_AVSS   0x80
#define MUX_AIN1_AVSS   0x90
#define MUX_AIN2_AVSS   0xA0
#define MUX_AIN3_AVSS   0xB0
#define MUX_SYSMON_VREF4    0xC0
#define MUX_SYSMON_AVDD4    0xD0
#define MUX_SYSMON_AVDD2    0xE0

//Config MUX settings (USED)
#define SE_CH0      0x80
#define SE_CH1      0x90
#define SE_CH2      0xA0
#define SE_CH3      0xB0
#define DIF_CH0_1   0x00
#define DIF_CH1_2	0x30
#define DIF_CH2_3   0x50

//Config PGA Gain
//When PGA is disabled, max gain is 4. See manual page 24.
#define PGA_GAIN_1   0x00
#define PGA_GAIN_2   0x02
#define PGA_GAIN_4   0x04
#define PGA_GAIN_8   0x06
#define PGA_GAIN_16  0x08
#define PGA_GAIN_32  0x0A
#define PGA_GAIN_64  0x0C
#define PGA_GAIN_128 0x0E

//Config PGA ON/OFF (BYPASS OFF/BYPASS ON)
//When using SE-Channels PGA is disabled independently of below settings (max gain: 4). See manual page 24.
#define PGA_ON    0x00
#define PGA_OFF   0x01

//Config DATA RATE
//Given numbers based on internal clock, normal mode; See manual page 28.
#define DR_20SPS    0x00
#define DR_45SPS    0x20
#define DR_90SPS    0x40
#define DR_175SPS   0x60
#define DR_330SPS   0x80
#define DR_600SPS   0xA0
#define DR_1000SPS  0xC0

//Config CONVERSION MODES
#define CM_CONTINUOUS    0x04
#define CM_SINGLE_SHOT   0x00

//Config OPERATION MODES
#define OM_NORMAL     0x00
#define OM_DUTY_CYCLE 0x08
#define OM_TURBO      0x10

//Config BURNOUT CURRENT SOURCES
#define BO_ON       0x01
#define BO_OFF      0x00

//Config TEMPERATURE MODES
#define TEMP_ON       0x02
#define TEMP_OFF      0x00

//Config Voltage Reference
#define VREF_INT         0x00
#define VREF_REFP0_REFN0 0x40
#define VREF_REFP1_REFN1 0x80
#define VREF_AVDD        0xC0

//Config FIR filter
//ONLY USE FIR filters in combination with data rate setting "000" in NORMAL and DUTY-CYCLE MODE (20 SPS / 5 SPS). 
//In all other settings turn off FIR filter
#define FIR_NONE    0x00
#define FIR_50_60   0x10
#define FIR_50      0x20
#define FIR_60      0x30

//Config Low-side power switch configuration (between AIN3/REFN1 and AVSS)
#define LSPSW_OPEN    0x00
#define LSPSW_CLOSE   0x08

//Config IDAC Current
#define IDAC_CUR_OFF  0x00
#define IDAC_CUR_10   0x01
#define IDAC_CUR_50   0x02
#define IDAC_CUR_100  0x03
#define IDAC_CUR_250  0x04
#define IDAC_CUR_500  0x05
#define IDAC_CUR_1000 0x06  
#define IDAC_CUR_1500 0x07

//Config IDAC1 Channel
#define IDAC1_OFF     0x00
#define IDAC1_AIN0    0x20
#define IDAC1_AIN1    0x40
#define IDAC1_AIN2    0x60
#define IDAC1_AIN3    0x80
#define IDAC1_REFP0   0xA0
#define IDAC1_REFN0   0xC0

//Config IDAC2 Channel
#define IDAC2_OFF     0x00
#define IDAC2_AIN0    0x04
#define IDAC2_AIN1    0x08
#define IDAC2_AIN2    0x0C
#define IDAC2_AIN3    0x10
#define IDAC2_REFP0   0x14
#define IDAC2_REFN0   0x18

//Config DRDY Mode
#define DRDY_STD    0x00
#define DRDY_DOUT   0x02
     

//DEFAULT SETTINGS
#define REG0_DEFAULT 0x00   //Default settings: AINP=AIN0, AINN=AIN1, Gain 1, PGA enabled
#define REG1_DEFAULT 0x04   //Default settings: DR=20 SPS, Mode=Normal, Conv mode=continuous, Temp Sensor disabled, Current Source off
#define REG2_DEFAULT 0x10   //Default settings: Vref internal, 50/60Hz rejection, power open, IDAC off
#define REG3_DEFAULT 0x00   //Default settings: IDAC1 disabled, IDAC2 disabled, DRDY pin only

//FULL SCALE (24bit)
#define FULL_SCALE   (((long int)1<<23)-1) //required for converting data

class agr_ads1220 {
	private:
	//Current CONFIG SETTINGS (MEMORY) -- WARNING SHOULD ALWAY BE THE SAME AS ACTUAL SETTINGS ON DEVICE
      uint8_t cur_config_reg0 = REG0_DEFAULT;   
      uint8_t cur_config_reg1 = REG1_DEFAULT;   
      uint8_t cur_config_reg2 = REG2_DEFAULT;   
      uint8_t cur_config_reg3 = REG3_DEFAULT;

  //Backup CONFIG SETTINGS -- Used when settings have to be changed temporarily
      uint8_t tmp_config_reg0 = 0;   
      uint8_t tmp_config_reg1 = 0;   
      uint8_t tmp_config_reg2 = 0;   
      uint8_t tmp_config_reg3 = 0;

  //Current reference voltage and PGA gain settings (used for calculation of voltage)
      float cur_vref = 2.048;
      int cur_gain = 1;
      
	//Digital pins (defined during initialization)
      uint8_t def_drdy_pin = 4;		//if not defined during initialization, this port is used
      uint8_t def_cs_pin = 5;		//if not defined during initialization, this port is used

  //Calibration (set during calibration)
      int32_t ads1220_offset = 0;

  //Functions
      uint8_t readRegister(uint8_t address);
      void writeRegister(uint8_t address, uint8_t value);
      void SPI_Command(unsigned char data_in);
      void reset(void);   
  	  
	public:
  //Turn Debug Information on Serial1 on/off
      int ads1220debug = 0;

  //Reference Voltages
      float avdd = 3.3;
      float vref0 = 3.3;
      float vref1 = 3.3;
  
  //Functions
      bool begin(uint8_t cs_pin, uint8_t drdy_pin);
      void calibrate(void);
      void reset_default(void);
      void config_backup(void);   //reads (from device) and backups current configuration to temporary storage
      void config_restore(void);  //reads temporary storage and sets configuration (to device)
      void config_mux(int conf_mux);
      void config_gain(int conf_gain);
      void config_pga(int conf_pga);
      void config_datarate(int conf_dr);
      void config_opmode(int conf_op);
      void config_conmode(int conf_cm);
      void config_tempmode(int conf_temp);
      void config_burnout(int conf_bo);
      void config_vref(int conf_vref);
      void config_fir(int conf_fir);
      void config_lspsw(int conf_lspsw);
      void config_idac_cur(int conf_idaccur);
      void config_idac1(int conf_idac1);
      void config_idac2(int conf_idac2);
      void config_drdy(int conf_drdy);
      
      void config_act_print(void);
      void config_cur_print(void);
      void config_tmp_print(void);
      void meas_start(void); //start measurements in continuous mode
      void meas_stop(void);  //end measurements in continuous mode (goes back to single-shot-mode)
      int32_t get_data(void); //reads data from the register
      int32_t read_single_shot(void);
      float dac2mv(int32_t);
      float dac2temp(int32_t);
      int sensorcheck(void);
      void powerdown(void);
      void start_conv(void);
      void write_reg0(uint8_t);
      void write_reg1(uint8_t);
      void write_reg2(uint8_t);
      void write_reg3(uint8_t);  
      void rdata(void);  
};


#endif
