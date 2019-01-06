#include "./mcp3008/mcp3008Spi.h"

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "ArduiPi_OLED_lib.h"
#include "Adafruit_GFX.h"
#include "ArduiPi_OLED.h"

using namespace std::literals::chrono_literals;

volatile sig_atomic_t stop;
void inthand(int signum)
{
    stop = 1;
}

const int LOGO16_GLCD_HEIGHT = 16;
const int LOGO16_GLCD_WIDTH  = 16; 
static unsigned char logo16_glcd_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

// Instantiate the display
ArduiPi_OLED display;

/***********************************************************************
 * This function returns one value from the provided ADC channel.
 * Opens the spidev0.0 device with SPI_MODE_0 (MODE 0) (defined in 
 * linux/spi/spidev.h), speed = 1MHz & bitsPerWord=8.
 *
 * i.e. transmit ->  byte1 = 0b00000001 (start bit)
 *                   byte2 = 0b1000000  (SGL/DIF = 1, D2=D1=D0=0)
 *                   byte3 = 0b00000000  (Don't care)
 *      receive  ->  byte1 = junk
 *                   byte2 = junk + b8 + b9
 *                   byte3 = b7 - b0
 *
 * after conversion must merge data[1] and data[2] to get final result
 *
 * *********************************************************************/

int get_a2d_val(int Channel)
{
    mcp3008Spi a2d("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
    int a2dVal = 0;
    unsigned char data[3];

    data[0] = 1;  //  first byte transmitted -> start bit
    data[1] = 0b10000000 |( ((Channel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
    data[2] = 0; // third byte transmitted....don't care
    
    a2d.spiWriteRead(data, sizeof(data) );
    
    a2dVal = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
    a2dVal |=  (data[2] & 0xff);

    return a2dVal;
}

/***********************************************************************
 * This function returns degrees Celsius as measured by the Amphenol
 * TK95F232V NTC thermistor in the following voltage divider circuit:
 *  - 3.3V reference voltage
 *  - 1.1 kOhm, 5% tolerance resistor tied to Vref
 *  - thermistor attached to ground
 *
 *  More infomation on the conversion equation here:
 *  https://en.wikipedia.org/wiki/Thermistor#Steinhart%E2%80%93Hart_equation
 * 
 * *********************************************************************/

float convert_a2d_to_temp(int a2dVal, const char units='C', std::string solver="S-H")
{
    float temp_K = 0;
    const float T_K_25 = 298.15; // temp in Kelvin at 25 deg C
    const float T_K_0 = 273.15; // temp in Kelvin at 0 deg C
    
    const float V_REF = 3.3; // voltage divider and logic reference
    float v_t = 0;
    float r_t = 0;
    double r_norm = 0;
    const float R_f = 1100; // resistance of voltage divider resistor

    // resistance at 25 deg C taken from:
    // https://www.amphenol-sensors.com/en/component/edocman/328-thermometrics-ntc-thermistors-epoxy-interchangeable-type-95-datasheet/viewdocument
    const float R_25 = 2252;

    v_t = a2dVal * V_REF / 1023; // input voltage at temp
    r_t = R_f * v_t / (V_REF - v_t); // thermistor resistance at temp
    r_norm = log(r_t/R_25);

    if (solver == "S-H")
    {
        // Steinhart–Hart equation coefficients taken from material Type F, resistances 3.274 to 0.36036 Ohms:
        // https://www.amphenol-sensors.com/en/component/edocman/292-thermometrics-temperature-resistance-curves-reference-guide/viewdocument
        const double A = 3.3540154E-03;
        const double B = 2.5627725E-04;
        const double C = 2.0829210E-06;
        const double D = 7.3003206E-08;
        
        temp_K = 1 / (A + B * r_norm + C * pow(r_norm, 2) + D * pow(r_norm, 3));
    }
    else if (solver == "Beta")
    {
        // Beta value taking from material Type F, temperature range 0 to 50 C:
        // https://www.amphenol-sensors.com/en/component/edocman/292-thermometrics-temperature-resistance-curves-reference-guide/viewdocument
        const float beta = 3895;

        temp_K = 1 / ( (r_norm/beta) + (1/T_K_25));
    }

    if (units == 'C')      { return (temp_K - T_K_0); }
    else if (units == 'F') { return ((temp_K - T_K_0) * 9.0 / 5.0 + 32); }
    else                   { return temp_K; }
}

/***********************************************************************
 * Display text.
 * 
 * *********************************************************************/

int display_text(const char * string)
{
    const char * text = string;

    // clear last display
    display.clearDisplay();
    display.display();

    // text display tests
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print(text);
    display.display();

    return 0;
}

int main(void)
{
    signal(SIGINT, inthand);

    const int OLED_TYPE = 0; // OLED_ADAFRUIT_SPI_128x32
    const int OLED_SPI_DC = 6; // RPI GPIO 6 pin 31
    const int OLED_SPI_RESET = 26; // RPI GPIO 26 pin 37
    const int OLED_SPI_CS = 1; // BCM2835_SPI_CS1

    if ( !display.init(OLED_SPI_DC, OLED_SPI_RESET, OLED_SPI_CS, OLED_TYPE) )
         exit(EXIT_FAILURE);

    display.begin();

    int channel = 0;
    int a2d_val = 0;
    int temp_F = 0;

    // temp conversion options
    char units_F = 'F';
    // std::string solver_Beta = "Beta";

    while (!stop)
    {
        a2d_val = get_a2d_val(channel);
        temp_F = static_cast<int>(convert_a2d_to_temp(a2d_val, units_F));
        std::cout << "Temp0: " << temp_F << " F" << std::endl;

        std::string text = "Temp: " + std::to_string(temp_F) + (char)247 + "F";
        display_text(text.c_str());

	std::this_thread::sleep_for(5s);
    }

    display.close();

    return 0;
}
