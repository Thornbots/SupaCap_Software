/*
rhit-supa-soft.ino

By: Alex Stedman for RHIT Thornbots

This code is for the control board for the team's supercapacitor bank. It runs on a Raspberry Pi Pico and interfaces with an INA219 sensor, a STM32 board (Type C), and PWM peripherals
This board controls the output of the capacitor bank to meet an input specification from the Type C, and uses a PI controller to regulate this. The output is complementary PWM.
Everything is divided into sections, with the individual components broken up at the top. Setup and Loop are at the bottom

THIS IS UNTESTED I HAVE NO IDEA IF THIS WORKS. ALSO CHATGPT HELPED WHICH MAKES IT EVEN WORSE

One cores should handle I2c and one cores hould handle PWM
Ensure that we use all 3 current sensors. We currently have one
Create a seperate setup and void setup1 and loop1 to have second core
physical test to ensure tat both PWM signals are never both high
if vbat < 10 or so, needs stop
should double check I2C values
I2C current sensors, current and voltage from battery, current to the robot and current to from the cap current we use is not measured.

Check PWM signal generator current voltage
*/

//imports ===============================================================
#include <Adafruit_INA219.h>
#include "hardware/pwm.h"
#include <Wire.h>

//physical cap bank properties =========================================
#define CAPACITANCE 16.67f     //series capacitance (F)
#define OUTPUT_VOLTAGE 24.0f   //output voltage (V)
#define CAP_MAX_VOLTAGE 15.0f  //maximum voltage of cap bank (V)
#define CAP_MIN_VOLTAGE 5.0f   //minimum voltage of cap bank (V)
#define CAP_MAX_CURRENT 3.0f   //maximum power cap bank can output (W)

//Current sesor I2c on the pico =========================================
#define SENSOR_SDA_PIN 19
#define SENSOR_SCL_PIN 18

//current sensor variable
Adafruit_INA219 battery_INA219;
Adafruit_INA219 robot_INA219;
Adafruit_INA219 cap_INA219;
//TwoWire sensorWire{ SENSOR_SDA_PIN, SENSOR_SCL_PIN };
// TwoWire sensorWire = TwoWire(0);

//function to read all values from cap bank with minimum calls
void readFromCapBank(float* outputCurrent, float* bankVoltage, float* bankEnergy) {

  //mA to A conversion.
  float bankCurrent = cap_INA219.getCurrent_mA() / 1000.0f;

  //add up the voltages this could be wrong idk
  *bankVoltage = cap_INA219.getBusVoltage_V() + cap_INA219.getShuntVoltage_mV() / 1000;

  //gets the current after the boost converter
  //temp transform with a div by 0 check. The 0.9 is to account for boost ineffecieny
  *outputCurrent = *bankVoltage > 0.01f ? (bankCurrent * OUTPUT_VOLTAGE / *bankVoltage / 0.9f) : 0;

  // 1/2 cv^2 our favorite
  *bankEnergy = 0.5f * CAPACITANCE * pow(*bankVoltage, 2);
}

//PWM pins and parameters =============================================
#define PWM_TOP_PIN 16
#define PWM_BOT_PIN 17
#define PWM_FREQ_HZ 100000      //100 khz
#define PWM_DEADTIME 63         //in ticks for 504 nS deadtime
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
#define K_P 1.0f
#define K_I 0.0f
#define MAX_ISUM 10.0f
#define MAX_DT 0.1f

//pi controller variable
float errorIntegral = 0;
long lastTimeMicros = 0;

//PI control function
void calculatePIController(float error, float* controlEffort) {
  //max is important to deal with starting up transient
  float dt = min((micros() - lastTimeMicros) / 1e6f, MAX_DT); //What is the purpose of this MAX_DT
  lastTimeMicros = micros();

  errorIntegral = max(min(errorIntegral + error * dt, MAX_ISUM), -MAX_ISUM);

  *controlEffort = K_P * error + K_I * errorIntegral;
}

//TYPE C LOGIC AND PINS ===============================================
#define TYPE_C_SDA_PIN 0
#define TYPE_C_SCL_PIN 1
#define I2C_SLAVE_ADDR 0x50  // FIND WHAT THIS ACTUALLY IS

//I2c variables
// TwoWire typeCWire = TwoWire(1);//{ TYPE_C_SDA_PIN, TYPE_C_SCL_PIN };

//these store the values for the i2cdata so we can know what actually is used in I2C and what is not. 
struct I2CData{
  uint16_t bankEnergyJ = 0; 
  uint16_t powerLimitW = 0; 
  uint16_t reqCurrentMA = 0;
} dataI2C;

//send the energy to the type C board
void writeToTypeC() {
  //writes on the line. Since it is a 16 bit need 2 bytes
  Wire.write((dataI2C.bankEnergyJ >> 8) & 0xFF);  // High byte
  Wire.write(dataI2C.bankEnergyJ & 0xFF);         // Low byte
}

//read values from the type C
void readFromTypeC(int byteCount) {

  while (Wire.available() && (byteCount -= 4) >= 0) {
    // Handle incoming data from Type C. its 4 bytes.
    dataI2C.powerLimitW = Wire.read() << 8 | Wire.read();    // Low byte
    dataI2C.reqCurrentMA = Wire.read() << 8 | Wire.read();  // Low byte
  }
}

//setup for PWM core =================================================================
void setup() {
  Serial.begin(115200);
  Wire1.setSCL(SENSOR_SCL_PIN);
  Wire1.setSDA(SENSOR_SDA_PIN);
  Wire1.begin();
  //initialize the current sensor
  if (!cap_INA219.begin(&Wire1)) {
    Serial.println("Failed to find capacitor INA219 chip");
  }else if (!battery_INA219.begin(&Wire1)) {
    Serial.println("Failed to find battery INA219 chip");
  }else if (!robot_INA219.begin(&Wire1)) {
    Serial.println("Failed to find robot INA219 chip");
  }

  // Configure GPIOs for PWM
  gpio_set_function(PWM_TOP_PIN, GPIO_FUNC_PWM);
  gpio_set_function(PWM_BOT_PIN, GPIO_FUNC_PWM);

  // Set PWM frequency
  pwm_set_wrap(SLICE_NUM, TOP_PWM);

  // Enable center-aligned mode and invert channel B
  pwm_hw->slice[SLICE_NUM].csr |= (1 << PWM_CH0_CSR_PH_CORRECT_LSB);  // Center-aligned
  pwm_hw->slice[SLICE_NUM].csr |= (1 << PWM_CH0_CSR_B_INV_LSB);       // Invert B

  //start pwm 
  pwm_set_enabled(SLICE_NUM, true);

  //log last time to prevent transient spike
  lastTimeMicros = micros();
}

//main loop variables==============================================================================
float bankVoltage = 0, boostedCurrent = 0, bankEnergy = 0;  //volts, amps, Joules
float targetCurrent, dutyCycle;                                  //loop values 

//main loop for PWM Core
void loop() {
  //read from the ina219 the important values yay
  readFromCapBank(&boostedCurrent, &bankVoltage, &bankEnergy);

  //TODO read from  battery and confirm voltage is high enough.
  float batteryVoltage = battery_INA219.getBusVoltage_V() + battery_INA219.getShuntVoltage_mV() / 1000;
  if(batteryVoltage < 10){
    Serial.println("Battery Voltage Too low < 10V \n");
  }
  //send current bank energy to the type C
  //dataI2C.bankEnergyJ = bankEnergy;

  //calculate what needs to be supplied by the supacap
  //make sure current is reasonable. Then account for power limit
  targetCurrent = min(CAP_MAX_CURRENT, (dataI2C.reqCurrentMA / 1000.0f) - (dataI2C.powerLimitW / OUTPUT_VOLTAGE));

  //do not output if the voltage is too low and we want to draw
  if (bankVoltage < CAP_MIN_VOLTAGE && targetCurrent > 0) dutyCycle = 0;

  //if we have energy in the bank we can use, or we are charging, run the controller
  else calculatePIController(targetCurrent - boostedCurrent, &dutyCycle);

  //finally, send output to the boost converter
  outputPWM(dutyCycle);
}

//setup function for USB-C Core =================================================================
void setup1() {
  Serial.begin(115200);

  //type C i2c init stuff
  Wire.setSCL(TYPE_C_SCL_PIN);
  Wire.setSDA(TYPE_C_SDA_PIN);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(writeToTypeC);
  Wire.onReceive(readFromTypeC);

  //log last time to prevent transient spike
  lastTimeMicros = micros();
}

//main loop for USB-C Core
void loop1() {

  //read from the ina219 the important values yay
  //readFromCapBank(&boostedCurrent, &bankVoltage, &bankEnergy);

  //send current bank energy to the type C
  dataI2C.bankEnergyJ = bankEnergy;

  //calculate what needs to be supplied by the supacap
  //make sure current is reasonable. Then account for power limit
  //targetCurrent = min(CAP_MAX_CURRENT, (dataI2C.reqCurrentMA / 1000.0f) - (dataI2C.powerLimitW / OUTPUT_VOLTAGE));

  //do not output if the voltage is too low and we want to draw
  // if (bankVoltage < CAP_MIN_VOLTAGE && targetCurrent > 0) dutyCycle = 0;

  //if we have energy in the bank we can use, or we are charging, run the controller
  // else calculatePIController(targetCurrent - boostedCurrent, &dutyCycle);

  //finally, send output to the boost converter
  // outputPWM(dutyCycle);
}