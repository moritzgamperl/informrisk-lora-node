//////////////////////////////////////////////////////////////////////////////////////////
//
//    Demo code for the ADS1220 24-bit ADC breakout board
//
//    Author: Ashwin Whitchurch // EDITED BY JOHN SINGER
//    Copyright (c) 2018 ProtoCentral
//
//    This example sequentially reads all 4 channels in continuous conversion mode
//
//    Arduino connections:
//
//  |ADS1220 pin label| Pin Function         |Arduino Connection|
//  |-----------------|:--------------------:|-----------------:|
//  | DRDY            | Data ready Output pin|  D5              |
//  | MISO            | Slave Out            |  D12             |
//  | MOSI            | Slave In             |  D11             |
//  | SCLK            | Serial Clock         |  D13             |
//  | CS              | Chip Select          |  D4              |
//  | DVDD            | Digital VDD          |  +5V             |
//  | DGND            | Digital Gnd          |  Gnd             |
//  | AN0-AN3         | Analog Input         |  Analog Input    |
//  | AVDD            | Analog VDD           |  -               |
//  | AGND            | Analog Gnd           |  -               |
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/Protocentral/Protocentral_ADS1220
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "AlpGeorisk_ADS1220.h"
#include <SPI.h>

#define PGA          128               // Programmable Gain = 1
#define VREF         3.3               // Internal reference of 2.048V
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1)

#define ADS1220_CS_PIN    4
#define ADS1220_DRDY_PIN  5

AlpGeorisk_ADS1220 ads1220;
int32_t adc_data;

void setup()
{
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    Serial1.begin(9600);

    ads1220.begin(ADS1220_CS_PIN,ADS1220_DRDY_PIN);

    uint8_t Config_Reg0 = ads1220.readRegister(CONFIG_REG0_ADDRESS);
    uint8_t Config_Reg1 = ads1220.readRegister(CONFIG_REG1_ADDRESS);
    uint8_t Config_Reg2 = ads1220.readRegister(CONFIG_REG2_ADDRESS);
    uint8_t Config_Reg3 = ads1220.readRegister(CONFIG_REG3_ADDRESS);

    Serial1.println("Config_Reg : ");
    Serial1.println(Config_Reg0,HEX);
    Serial1.println(Config_Reg1,HEX);
    Serial1.println(Config_Reg2,HEX);
    Serial1.println(Config_Reg3,HEX);
    Serial1.println(" ");

    ads1220.select_mux_channels(MUX_AIN1_AIN2);
    ads1220.PGA_ON();
    ads1220.set_pga_gain(PGA_GAIN_128);
    ads1220.set_data_rate(DR_20SPS);
    ads1220.set_op_mode(OPMODE_DUTY_CYCLE);
    ads1220.set_vref(VREF_REFP1_REFN1);
    ads1220.set_fir_filter(FIR_filter_50_60);
    ads1220.set_PWS_AUTO();
    ads1220.set_conv_mode_single_shot();

    Config_Reg0 = ads1220.readRegister(CONFIG_REG0_ADDRESS);
    Config_Reg1 = ads1220.readRegister(CONFIG_REG1_ADDRESS);
    Config_Reg2 = ads1220.readRegister(CONFIG_REG2_ADDRESS);
    Config_Reg3 = ads1220.readRegister(CONFIG_REG3_ADDRESS);

    Serial1.println("Config_Reg : ");
    Serial1.println(Config_Reg0,HEX);
    Serial1.println(Config_Reg1,HEX);
    Serial1.println(Config_Reg2,HEX);
    Serial1.println(Config_Reg3,HEX);
    Serial1.println(" ");

    delay(5000);
}

void loop()
{
    adc_data=ads1220.Read_SingleShot_WaitForData();
    float Vout = (float)((adc_data*VFSR*1000)/FULL_SCALE);     //In  mV
    Serial1.println(Vout);
    delay(5000);
    ads1220.powerdown();
    delay(5000);
}
