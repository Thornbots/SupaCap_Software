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
#define PWM_DEADTIME 63         //in ticks for 504 nS deadtime
#define SYS_CLOCK_HZ 125000000  // Default 125 MHz for RP2040
const int SLICE_NUM = pwm_gpio_to_slice_num(PWM_TOP_PIN);
const uint32_t TOP_PWM = SYS_CLOCK_HZ / (PWM_FREQ_HZ * 2) - 1;  // Correct calculation for center-aligned mode

uint32_t levelA, levelB;
int dutyCycle;

#define TARGET_CURRENT -0.1f
#define STOP_VOLTAGE 12

void outputPWM(float dutyCycle) {
  // Set duty cycle with dead time
  levelA = dutyCycle * TOP_PWM;
  levelB = min(levelA + PWM_DEADTIME, TOP_PWM);

  //writes
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_A, levelA);
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_B, levelB);
}

//PI Controller ============================================
#define K_P 0.001f
#define K_I 0.0f
#define MAX_ISUM 10.0f
#define MAX_DT 0.1f

//pi controller variable
// float errorIntegral = 0;
long lastTimeMicros = 0;

// //PI control function
// void calculatePIController(float error, float* controlEffort) {
//   //max is important to deal with starting up transient
//   float dt = min((micros() - lastTimeMicros) / 1e6f, MAX_DT); //What is the purpose of this MAX_DT
//   lastTimeMicros = micros();

//   errorIntegral = max(min(errorIntegral + error * dt, MAX_ISUM), -MAX_ISUM);

//   *controlEffort = max(min(K_P * error + K_I * errorIntegral, 0.5), -0.5);
// }

//finds duty cycle to achieve a current over the inductor
float INDUCTANCE = 10*10e-5;
float openLoopController(float desiredCurrent, float measuredCurrent, float measuredVoltage){

  float dt = (micros() - lastTimeMicros)/1000000.0f;
  lastTimeMicros = micros();

  float di = desiredCurrent - measuredCurrent;
  // lastCurrent = desiredCurrent;

  Serial.print(-INDUCTANCE * (di/dt));
  // Serial.print(" ");
  // Serial.print(measuredVoltage);
  // Serial.print(" ");
  // Serial.print(desiredCurrent);
  Serial.print(" ");
  Serial.print(dt);
  Serial.print(" ");
 
  float outputVoltage = - INDUCTANCE * (di / dt) + measuredVoltage;
  Serial.println(outputVoltage);
  return outputVoltage / 24; //convert to duty cycle

}
//PI Controller ============================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {
      // will pause until serial console opens DO NOT INCLUDE ON FINAL CODE 
      delay(1);
  }

  if (! ina219_CAPBANK.begin(&CurrentBus)) {
    Serial.println("Failed to find Cap Bank chip");
    while (1) { delay(10); }
  }

  if (! ina219_PMM.begin(&CurrentBus)) {
    Serial.println("Failed to find PMM chip");
    while (1) { delay(10); }
  }

  if (! ina219_MOTORS.begin(&CurrentBus)) {
    Serial.println("Failed to find Motor Output chip");
    while (1) { delay(10); }
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
  dutyCycle = 0;

}

void loop() {
  float busvoltage = 0;
  float mycurrent = 0;
  float dutyCycle;
  float targetCurrent = TARGET_CURRENT;

  busvoltage = ina219_CAPBANK.getBusVoltage_V();
  mycurrent = ina219_CAPBANK.getShuntVoltage_mV()/10.0;
  // busvoltage = 5.0;
  // mycurrent = 0.5;

  // Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  // Serial.print("my current:       "); Serial.print(mycurrent); Serial.println(" A");
  
  if(busvoltage >= STOP_VOLTAGE){
    while(1){
      pinMode(16, OUTPUT);
      digitalWrite(16, LOW);
      pinMode(17, OUTPUT);
      digitalWrite(17, LOW);
    }
  }

  dutyCycle = openLoopController(TARGET_CURRENT, mycurrent, busvoltage);
  // Serial.print("Duty Cycle:       "); Serial.println(dutyCycle);

  //finally, send output to the boost converter
  outputPWM(dutyCycle);
  delay(50);
}
