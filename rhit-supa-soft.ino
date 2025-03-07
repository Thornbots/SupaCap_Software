/*
rhit-supa-soft.ino

By: Alex Stedman for RHIT Thornbots

This code is for the control board for the team's supercapacitor bank. It runs on a Raspberry Pi Pico and interfaces with an INA219 sensor, a STM32 board (Type C), and PWM peripherals
This board controls the output of the capacitor bank to meet an input specification from the Type C, and uses a PI controller to regulate this. The output is complementary PWM.
Everything is divided into sections, with the individual components broken up at the top. Setup and Loop are at the bottom

THIS IS UNTESTED I HAVE NO IDEA IF THIS WORKS. ALSO CHATGPT HELPED WHICH MAKES IT EVEN WORSE
*/

//imports ===============================================================
#include <Adafruit_INA219.h>
#include "hardware/pwm.h"

//physical cap bank properties =========================================
#define CAPACITANCE 16.67     //series capacitance (F)
#define OUTPUT_VOLTAGE 24.0   //output voltage (V)
#define CAP_MAX_VOLTAGE 15.0  //maximum voltage of cap bank (V)
#define CAP_MIN_VOLTAGE 5.0   //minimum voltage of cap bank (V)
#define CAP_MAX_CURRENT 3.0   //maximum power cap bank can output (W)

//Current sesor I2c on the pico =========================================
#define SENSOR_SDA_PIN 19
#define SENSOR_SCL_PIN 18

//current sensor variable
Adafruit_INA219 ina219;
TwoWire sensorWire{ SENSOR_SDA_PIN, SENSOR_SCL_PIN };

//function to read all values from cap bank with minimum calls
void readFromCapBank(float* outputCurrent, float* bankVoltage, float* bankEnergy) {

  //mA to A conversion.
  float bankCurrent = ina219.getCurrent_mA() / 1000.0f;

  //add up the voltages this could be wrong idk
  *bankVoltage = ina219.getBusVoltage_V() + ina219.getShuntVoltage_mV() / 1000;

  //gets the current after the boost converter
  //temp transform with a div by 0 check. The 0.9 is to account for boost ineffecieny
  *outputCurrent = *bankVoltage > 0.01f ? (bankCurrent * OUTPUT_VOLTAGE / *bankVoltage / 0.9f) : 0;

  // 1/2 cv^2 our favorite
  *bankEnergy = 0.5f * CAPACITANCE * pow(*bankVoltage, 2);
}

//PWM pins and parameters =============================================
#define PWM_TOP_PIN 16
#define PWM_BOT_PIN 17
#define PWM_FREQ_HZ 100000  //100 khz
#define PWM_DEADTIME 63  //in ticks for 504 nS deadtime
#define SYS_CLOCK_HZ 125000000  // Default 125 MHz for RP2040
const int SLICE_NUM = pwm_gpio_to_slice_num(PWM_TOP_PIN);
const uint32_t TOP_PWM = SYS_CLOCK_HZ / (PWM_FREQ_HZ * 2) - 1;  // Correct calculation for center-aligned mode


//pwm variables
uint32_t levelA, levelB;


//function
void outputPWM(float dutyCycle) {
  // Set duty cycle with dead time
  levelA = dutyCycle * TOP_PWM;
  levelB = min(levelA + PWM_DEADTIME, TOP_PWM);

  //writes
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_A, levelA);
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_B, levelB);
}

//PI controller ============================================
#define K_P 1.0
#define K_I 0.0
#define MAX_ISUM 10.0
#define MAX_DT 0.1

//pi controller variable
float errorIntegral = 0;
long lastTimeMicros = 0;

//PI control function
void calculatePIController(float error, float* controlEffort) {
  //max is important to deal with starting up transient
  float dt = min((micros() - lastTimeMicros) / 1e6f, MAX_DT);
  lastTimeMicros = micros();

  errorIntegral = max(min(errorIntegral + error * dt, MAX_ISUM), -MAX_ISUM);

  *controlEffort = K_P * error + K_I * errorIntegral;
}

//TYPE C LOGIC AND PINS ===============================================
#define TYPE_C_SDA_PIN 0
#define TYPE_C_SCL_PIN 1
#define TYPE_C_I2C_ADDR 0x50  // FIND WHAT THIS ACTUALLY IS

//I2c variables
TwoWire typeCWire{ TYPE_C_SDA_PIN, TYPE_C_SCL_PIN };
uint16_t energyMJ, powerMW, currentMA;

//send the energy to the type C board
void writeToTypeC(float energy) {
  energyMJ = (uint16_t)(energy * 1000); //convert to milliJoules to send as a uint16

  typeCWire.beginTransmission(TYPE_C_I2C_ADDR);

  typeCWire.write(0x01);                    // ChatGPT suggested this
  typeCWire.write((energyMJ >> 8) & 0xFF);  // High byte
  typeCWire.write(energyMJ & 0xFF);         // Low byte

  typeCWire.endTransmission();
}

//read values from the type C
void readFromTypeC(float* powerLimit, float* requestedCurrent) {

  typeCWire.beginTransmission(TYPE_C_I2C_ADDR);
  typeCWire.write(0x02);  // ChatGPT suggested this
  typeCWire.endTransmission(false);

  typeCWire.requestFrom(TYPE_C_I2C_ADDR, 2);
  if (typeCWire.available() == 4) {
    powerMW = (typeCWire.read() << 8) | typeCWire.read();   //first read power limit
    currentMA = (typeCWire.read() << 8) | typeCWire.read(); //second read current requested
  } else {
    Serial.println("Type-C read failed.");
    //TODO add failure mode behavior
  }

  *powerLimit = powerMW / 1000.0f;          // Extract power limit
  *requestedCurrent = currentMA / 1000.0f;  // Extract requested current
    
}

//setup function =================================================================
void setup() {
  Serial.begin(115200);

  //initialize the current sensor
  if (!ina219.begin(&sensorWire)) {
    Serial.println("Failed to find INA219 chip");
  }

  //type C i2c init stuff
  typeCWire.begin();
  Serial.println("Type C I2c initialized");

  // Configure GPIOs for PWM
  gpio_set_function(PWM_TOP_PIN, GPIO_FUNC_PWM);
  gpio_set_function(PWM_BOT_PIN, GPIO_FUNC_PWM);

  // Set PWM frequency
  pwm_set_wrap(SLICE_NUM, TOP_PWM);

  // Enable center-aligned mode and invert channel B
  pwm_hw->slice[SLICE_NUM].csr |= (1 << PWM_CH0_CSR_PH_CORRECT_LSB);  // Center-aligned
  pwm_hw->slice[SLICE_NUM].csr |= (1 << PWM_CH0_CSR_B_INV_LSB);       // Invert B

  pwm_set_enabled(SLICE_NUM, true);


  lastTimeMicros = micros();
}

//main loop variables==============================================================================
float voltage = 0, current = 0, energy = 0; //volts, amps, Joules
float x, y; //loop values
float pLimit = 0, rCurrent = 0; //watts, and amps

//main loop for real trust
void loop() {

  //read from the ina219 the important values yay
  readFromCapBank(&current, &voltage, &energy);

  //get the values from the type C todo
  readFromTypeC(&pLimit, &rCurrent);

  //send current bank energy to the type C 
  writeToTypeC(energy);

  //calculate what needs to be supplied by the supacap
  x = min(CAP_MAX_CURRENT, rCurrent) - pLimit / OUTPUT_VOLTAGE;

  //do not output if the voltage is too low and we want to draw
  if (voltage < CAP_MIN_VOLTAGE && x > 0) y = 0;

  //if we have energy in the bank we can use, or we are charging, run the controller
  else calculatePIController(x - current, &y);

  //finally, send output to the boost converter
  outputPWM(y);
}
