#include "./mcp3008/mcp3008Spi.h"
#include "./rpiPWM1/rpiPWM1.h"
#include "./GPIOClass/GPIOClass.h"
#include "./pid/pid.h"

#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

#include "ArduiPi_OLED_lib.h"
#include "Adafruit_GFX.h"
#include "ArduiPi_OLED.h"

using namespace std::literals::chrono_literals;

// Instantiate the display
ArduiPi_OLED display;

// Control cycle worker variables
bool main_ready;
bool worker_ready;
std::mutex m;
std::condition_variable cv;
std::chrono::system_clock::time_point start_time;
std::chrono::duration<double> diff;

double control_cycle = 0.500; //seconds
volatile sig_atomic_t stop;
const int TEMP0 = 0;
enum Mode
{
  HEAT = 1, 
  COOL = 2 
};

void inthand(int signum)
{
    stop = 1;
}

/***********************************************************************
 * This function returns one value from the provided ADC channel.
 * Opens the spidev0.0 device with SPI_MODE_0 (MODE 0) (defined in 
 * linux/spi/spidev.h), speed = 1MHz & bitsPerWord=8.
 * (After conversion must merge data[1] and data[2] to get final result)
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
 * This function returns degrees Celsius as measured from the Amphenol
 * TK95F232V NTC thermistor.
 *
 * Returns:
 *  temp - float temperature in Celsius 
 * *********************************************************************/
float get_temp(void)
{
    float temp_K, v_t, r_t;
    double r_norm;

    // Resistance at 25 deg C taken from:
    // https://www.amphenol-sensors.com/en/component/edocman/328-thermometrics-ntc-thermistors-epoxy-interchangeable-type-95-datasheet/viewdocument
    const float R_25 = 2252;
    const float V_REF = 3.3; // voltage divider and logic reference
    const float R_f = 1100; // resistance of voltage divider resistor
    const float T_K_0 = 273.15; // temp in Kelvin at 0 deg C

    // Steinhart–Hart equation coefficients taken from material Type F, resistances 3.274 to 0.36036 Ohms:
    // https://www.amphenol-sensors.com/en/component/edocman/292-thermometrics-temperature-resistance-curves-reference-guide/viewdocument
    const double A = 3.3540154E-03;
    const double B = 2.5627725E-04;
    const double C = 2.0829210E-06;
    const double D = 7.3003206E-08;

    // Get reading from ADC
    int a2dVal = get_a2d_val(TEMP0);

    v_t = a2dVal * V_REF / 1023; // input voltage at temp
    r_t = R_f * v_t / (V_REF - v_t); // thermistor resistance at temp
    r_norm = log(r_t/R_25);

    // Temperature in Kelvin
    temp_K = 1 / (A + B * r_norm + C * pow(r_norm, 2) + D * pow(r_norm, 3));

    // Convert temp to Celsius
    float temp = temp_K - T_K_0;

    return temp;
}

/***********************************************************************
 * Convert temperature from Celsius to Fahrenheit
 * *********************************************************************/
float convert_to_fahrenheit(float temp_C)
{
    return (temp_C * 9.0 / 5.0 + 32);
}

/***********************************************************************
 * Display text passed to function on OLED.
 * *********************************************************************/
int display_text(const char * string)
{
    const char * text = string;

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

/***********************************************************************
 * Initialize display  
 * *********************************************************************/
int display_init(void)
{
    // Configure display model and pins
    const int OLED_TYPE = 0; // OLED_ADAFRUIT_SPI_128x32
    const int OLED_SPI_DC = 6; // RPI GPIO 6 pin 31
    const int OLED_SPI_RESET = 26; // RPI GPIO 26 pin 37
    const int OLED_SPI_CS = 1; // BCM2835_SPI_CS1

    // Initialize display
    if ( !display.init(OLED_SPI_DC, OLED_SPI_RESET, OLED_SPI_CS, OLED_TYPE) )
         exit(EXIT_FAILURE);
    display.begin();

    return 0;
}

/***********************************************************************
 * Enforce the control cycle time  
 * *********************************************************************/
void dt_control()
{
    // Wait until main() sends data
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, []{return main_ready;});

    diff = std::chrono::system_clock::now() - start_time;

    while (diff.count() < control_cycle){
        diff = std::chrono::system_clock::now() - start_time;
    }
 
    // Send data back to main()
    worker_ready = true;
    // Manual unlocking is done before notifying, to avoid waking up
    // the waiting thread only to block again (see notify_one for details)
    lk.unlock();
    cv.notify_one();
}

int main(void)
{
    std::string mode;
    int temp, loop_count = 0;
    double temp_target = 23;

    // Initialize display
    display_init();

    // Initialize PWM
    float duty_cycle = 10.0;
    rpiPWM1 pwm(1000.0, 256, duty_cycle, rpiPWM1::MSMODE);

    // Initialize H-Bridge direction control pin
    GPIOClass* gpio23 = new GPIOClass("23");
    gpio23->export_gpio();
    gpio23->setdir_gpio("out");

    // Initialize PID controller
    PID pid = PID(control_cycle, 99.9, -99.9, 2.0, 0.1, 0.3);

    // Handle CTRL-C interrupt from keyboard
    signal(SIGINT, inthand);

    while (!stop)
    {
	main_ready = false;
	worker_ready = false;

        std::thread worker(dt_control);
 
	start_time = std::chrono::system_clock::now();
        // send data to the worker thread
        {
            std::lock_guard<std::mutex> lk(m);
            main_ready = true;
        }
        cv.notify_one();


	// Update temperature
        temp = static_cast<int>(get_temp());

	// Display and print temperature at reduced loop rate
	if (loop_count == 0 or loop_count % 10 == 0){
            std::string text = "Temp: " + std::to_string(temp) + (char)247 + "C";
            display_text(text.c_str());
	    std::cout << text << " Target: " << temp_target << "C" << std::endl;
	    std::cout << "Duty cycle: " << duty_cycle << " Mode: " << mode << std::endl;
	}

	// Set new control state
        duty_cycle = pid.calculate(temp_target, temp);

	if (duty_cycle > 0){
            gpio23->setval_gpio("1");
	    mode = "HEAT";
	} else if (duty_cycle < 0){
            gpio23->setval_gpio("0");
	    mode = "COOL";
	} else {
            duty_cycle = 1; // Duty Cycle low but non-zero
	}

	duty_cycle = std::abs(duty_cycle);
        pwm.setDutyCycleCount(duty_cycle);
	    
        // Wait for end of control cycle
        {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk, []{return worker_ready;});
        }
        worker.join();

        loop_count++;
    }

    pwm.setDutyCycleCount(0); // Duty Cycle to 0%
    display.close();

    return 0;
}
