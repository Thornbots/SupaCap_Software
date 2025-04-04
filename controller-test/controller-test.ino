#include <Wire.h>
#include <Adafruit_INA219.h>

TwoWire CurrentBus(18, 19);
Adafruit_INA219 ina219_CAPBANK(0x40);
Adafruit_INA219 ina219_PMM(0x41);
Adafruit_INA219 ina219_MOTORS(0x44);

#include "hardware/pwm.h"

#define PWM_TOP_PIN 16
#define PWM_BOT_PIN 17
#define PWM_FREQ_HZ 100000      //100 khz
#define PWM_DEADTIME 12         //in ticks for 504 nS deadtime
#define SYS_CLOCK_HZ 125000000  // Default 125 MHz for RP2040
const int SLICE_NUM = pwm_gpio_to_slice_num(PWM_TOP_PIN);
const uint32_t TOP_PWM = SYS_CLOCK_HZ / (PWM_FREQ_HZ * 2) - 1;  // Correct calculation for center-aligned mode

uint32_t levelA, levelB;

#define STOP_VOLTAGE 11

void outputPWM(float dutyCycle) {
  // Set duty cycle with dead time
  levelA = dutyCycle * TOP_PWM;
  levelB = min(levelA + PWM_DEADTIME, TOP_PWM);

  //writes
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_A, levelA);
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_B, levelB);
}

//voltage to duty cycle
#define CURRENT_DEADZONE 0.0  //1 //A
#define SUPPLY_VOLTAGE 24
#define MAX_BUCK_DUTY 0.6
#define MAX_BOOST_DUTY 0.9

float voltageToDutyCycle(float targetCurrent, float targetVoltage) {
  float dutyCycle;
  if (targetCurrent > CURRENT_DEADZONE) {                            //we are discharging the cap (boost)
    dutyCycle = min(1 - (1 / targetVoltage), MAX_BOOST_DUTY);          //equation with limit
  } else {                                                           //} if(targetCurrent <= -CURRENT_DEADZONE){ //we are charging the cap (buck)
    dutyCycle = min(targetVoltage / SUPPLY_VOLTAGE, MAX_BUCK_DUTY);  //equatio with limit
    // } else {  //we are letting the cap hold voltage so we do nothing
    //   //todo cut off both ports
  }
  return max(0.001, dutyCycle); //dont allow 0 duty cycle 
}

//P controller + voltage and constant feedforward
#define K_P 0.05      //this is negative because more negative current is higher voltage, and more positive current is lower voltage
#define K_I 0.0
#define K_F 1.0           //scale measured voltage
#define K_C -0.15        //add to measured voltage
#define MAX_EFFORT 0.2  //V
// #define BUFFER_LENGTH 10

// float errorBuffer[BUFFER_LENGTH] = {0};  // buffer to store past errors
// int bufferIndex = 0;
// float errorSum = 0;

float currentToVoltage(float measuredCurrent, float targetCurrent, float measuredVoltage) {

  // errorSum -= errorBuffer[bufferIndex];      // remove the oldest error
  // errorBuffer[bufferIndex] = - targetCurrent + measuredCurrent;          // add the new error
  // errorSum += - targetCurrent + measuredCurrent;                         // update the sum
  // bufferIndex = (bufferIndex + 1) % BUFFER_LENGTH;

  float controlEffort = K_P * (- targetCurrent + measuredCurrent); // + K_I *(errorSum/BUFFER_LENGTH);  //pi controller

  controlEffort = min(max(-MAX_EFFORT, controlEffort), MAX_EFFORT);  // control effort limiter

  return controlEffort + max(0, K_F * measuredVoltage + K_C);  //add in open loop terms to get voltage target
}


void configINA219(uint8_t addr) {
  uint16_t config = 0;


    uint16_t configValue = 
        INA219_CONFIG_BVOLTAGERANGE_32V |  // Set Bus Voltage Range to 32V
        INA219_CONFIG_GAIN_8_320MV |      // Set Gain to 8 (320mV range)
        INA219_CONFIG_BADCRES_12BIT |     // Set Bus ADC Resolution to 12-bit
        INA219_CONFIG_SADCRES_12BIT_128S_69MS | // Set Shunt ADC Resolution to 128x 12-bit samples
        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; // Continuous mode

    // Write to INA219's configuration register
    CurrentBus.beginTransmission(addr); //capbank
    CurrentBus.write(INA219_REG_CONFIG); // Point to Configuration Register
    CurrentBus.write(configValue >> 8);  // Send high byte
    CurrentBus.write(configValue & 0xFF); // Send low byte
    CurrentBus.endTransmission();




  Serial.println("INA219 configured with 128 sample averaging.");
}

float voltageArray[10], currentArray[10];

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // will pause until serial console opens DO NOT INCLUDE ON FINAL CODE
    delay(1);
  }
  Serial.println("beginning");

  // configINA219(0x40);

  Serial.println("ee");
  while(!ina219_CAPBANK.begin(&CurrentBus)) {
    Serial.println("Failed to find Cap Bank chip");
    delay(1000);
  }
  configINA219(0x40);

  while (!ina219_PMM.begin(&CurrentBus)) {
    Serial.println("Failed to find PMM chip");
    delay(1000);
  }
  configINA219(0x41);

  while (!ina219_MOTORS.begin(&CurrentBus)) {
    Serial.println("Failed to find Motor Output chip");
    delay(1000);
  }
  configINA219(0x44);


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

  for(int i = 0; i < 10; i++){
    voltageArray[i] = ina219_CAPBANK.getBusVoltage_V();
  }
}

void loop() {
  float measuredVoltage, measuredCurrent, dutyCycle, targetCurrent = -0.25, targetVoltage;
  
  for(int i = 0; i < 9; i++){
    voltageArray[i] = voltageArray[i+1];
    measuredVoltage += voltageArray[i];
  }
  voltageArray[9] = ina219_CAPBANK.getBusVoltage_V();
  measuredVoltage = (measuredVoltage + voltageArray[9])/10.0f;


  for(int i = 0; i < 9; i++){
    currentArray[i] = currentArray[i+1];
    measuredCurrent += currentArray[i];
  }
  currentArray[9] = ina219_CAPBANK.getShuntVoltage_mV()/10.0f;
  measuredCurrent = (measuredCurrent + currentArray[9])/10.0f;



  if (measuredVoltage >= STOP_VOLTAGE) {
    while (1) {
      pinMode(16, OUTPUT);
      digitalWrite(16, LOW);
      pinMode(17, OUTPUT);
      digitalWrite(17, LOW);
      Serial.println("OVERVOLTAGE ERROR");
    }
  }

  targetVoltage = currentToVoltage(measuredCurrent, targetCurrent, measuredVoltage);

  dutyCycle = voltageToDutyCycle(targetCurrent, targetVoltage);

  char buffer[120];  // Allocate enough space for formatted output
  snprintf(buffer, sizeof(buffer), "TargetVoltage:%.3f,MeasuredVoltage:%.3f,TargetCurrent:%.3f,MeasuredCurrent:%.3f,DutyCycle:%.3f",
           targetVoltage, measuredVoltage, targetCurrent, measuredCurrent, dutyCycle);

  Serial.println(buffer);
  //finally, send output to the boost converter
  outputPWM(dutyCycle);

  delay(10);
}
