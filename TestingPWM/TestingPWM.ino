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


#include <Wire.h>
#include <Adafruit_INA219.h>

TwoWire CurrentBus(18, 19);
Adafruit_INA219 ina219_CAPBANK(0x40);
Adafruit_INA219 ina219_PMM(0x41);
Adafruit_INA219 ina219_MOTORS(0x44);

void outputPWM(float dutyCycle) {
  // Set duty cycle with dead time
  levelA = dutyCycle * TOP_PWM;
  levelB = min(levelA + PWM_DEADTIME, TOP_PWM);

  //writes
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_A, levelA);
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_B, levelB);
}

void setup() {
  // put your setup code here, to run once:
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
  float current_mA = 0;

  busvoltage = ina219_CAPBANK.getBusVoltage_V();
  current_mA = ina219_CAPBANK.getCurrent_mA();
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");



  float dutyCycle_float  = 0.4;
  outputPWM(dutyCycle_float);
  delay(150);
}
