#include <Arduino.h>
#include "hardware/pwm.h"
#include "PicoEncoder.h"
#include <pico/multicore.h>

#include <Wire.h>
#include <hardware/i2c.h>
#include <Adafruit_INA219.h>
//#include <I2CSlave.h>

//
enum BoardStatus {
  INIT,
  STOPPED,
  CHARGING,
  DISCHARGING,
  NUM_STATES
};

//PWM DECLARATIONS
#define PWM_TOP_PIN 16
#define PWM_BOT_PIN 17
#define PWM_FREQ_HZ 100000      //100 khz
#define PWM_DEADTIME 12         //in ticks for 504 nS deadtime
#define SYS_CLOCK_HZ 125000000  // Default 125 MHz for RP2040
const int SLICE_NUM = pwm_gpio_to_slice_num(PWM_TOP_PIN);
const uint32_t TOP_PWM = SYS_CLOCK_HZ / (PWM_FREQ_HZ * 2) - 1;  // Correct calculation for center-aligned mode
const uint32_t MAX_DUTY_CYCLE = 0.475; //higher than this results in overvolting the cap bank
const uint32_t MIN_DUTY_CYCLE = 0.100;

uint32_t levelA, levelB;

#define TARGET_CURRENT -0.1f
#define STOP_VOLTAGE 13

//I2C Declarations
static const uint I2C_AS_PERIPHERAL_SDA_GPIO = 16;   //board pin 21
static const uint I2C_AS_PERIPHERAL_SCL_GPIO = 17;   //board pin 22
static const uint I2C_AS_PERIPHERAL_ADDRESS = 0x45;  //nice

static const uint I2C_AS_CONTROLLER_SDA_GPIO = 18;  //board pin 24
static const uint I2C_AS_CONTROLLER_SCL_GPIO = 19;  //board pin 25

//initializes both i2c0 and i2c1

//I2C0
TwoWire DataBus(I2C_AS_PERIPHERAL_SDA_GPIO, I2C_AS_PERIPHERAL_SCL_GPIO);
//I2C1
TwoWire CurrentBus(I2C_AS_CONTROLLER_SDA_GPIO, I2C_AS_CONTROLLER_SCL_GPIO);
Adafruit_INA219 ina219_CAPBANK(0x40);
Adafruit_INA219 ina219_PMM(0x41);
Adafruit_INA219 ina219_MOTORS(0x44);

//MIGHT NEED TO BE VOLATILE?
//Instructions, values etc received by pico as peripheral
struct __attribute__((packed)) receivedFrame {
  byte currentInstruction;
  float targetCurrent = TARGET_CURRENT;
} incomingData;

//MIGHT NEED TO BE VOLATILE
//State, data, readout values, current time, etc, sent by pico as peripheral
struct __attribute__((packed)) CoreStatusDataFrame {
  BoardStatus STATE = INIT;
  float busVoltage = 0.0;
  float myCurrent = 0.0;
  float dutyCycle = 0.01;  //tbd if needed, could stop weird transient effects of slamming to 0, bc to stop we really want both pwm 0, not 0 duty cycle
  float targetCurrent = 0;
  long currentTimeMillis = 0;
  volatile bool statusDataReady = false;
};
CoreStatusDataFrame StatusData;

void outputPWM(float dutyCycle) {
  // Set duty cycle with dead time
  dutyCycle = min(MAX_DUTY_CYCLE, dutyCycle); //sets upper bound
  dutyCycle = max(dutyCycle, MIN_DUTY_CYCLE); //sets lower bound
  
  levelA = dutyCycle * TOP_PWM;
  levelB = min(levelA + PWM_DEADTIME, TOP_PWM);

  //writes
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_A, levelA);
  pwm_set_chan_level(SLICE_NUM, PWM_CHAN_B, levelB);
}

//sets both pwm outputs low then disables PWM.
void stopPWM() {
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);
  pwm_set_enabled(SLICE_NUM, false);
}

void DataBusReceiveData(int numBytes) {
  if (numBytes == 1) {  //here is where a control byte would be parsed
    switch (DataBus.peek()) {
      default:
        break;
    }
  }

  if (numBytes == sizeof(incomingData)) {
    byte recvdata[sizeof(incomingData)];
    for (int i = 0; i < numBytes; i++) {
      recvdata[i] = DataBus.read();
    }
    memcpy(&incomingData, recvdata, sizeof(recvdata));  //gotta be same order or it'll geek
    StatusData.statusDataReady = true;
  }
}  // cook here

void DataBusOnRequest() {
  uint8_t sendBuffer[sizeof(StatusData)];
  memcpy(sendBuffer, &StatusData, sizeof(StatusData));  //casting to bytes
  DataBus.write(sendBuffer, sizeof(sendBuffer));
}  // cook here


void setup() {
  StatusData.currentTimeMillis = millis();
  Serial.begin(115200);
  while (!Serial) {
    // will pause until serial console opens DO NOT INCLUDE ON FINAL CODE
    delay(1);
  }
  //i2c0 begining
  DataBus.begin(I2C_AS_PERIPHERAL_ADDRESS);
  DataBus.onReceive(DataBusReceiveData);
  DataBus.onRequest(DataBusOnRequest);

  //i2c1 begining
  if (!ina219_CAPBANK.begin(&CurrentBus)) {
    Serial.println("Failed to find Cap Bank chip");
    while (1) { delay(10); }
  }

  if (!ina219_PMM.begin(&CurrentBus)) {
    Serial.println("Failed to find PMM chip");
    while (1) { delay(10); }
  }

  if (!ina219_MOTORS.begin(&CurrentBus)) {
    Serial.println("Failed to find Motor Output chip");
    while (1) { delay(10); }
  }

  StatusData.currentTimeMillis = millis();
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
  //lastVoltage = ina219_CAPBANK.getBusVoltage_V();
}





void loop() {

  if (StatusData.statusDataReady) {
    //maybe parse inst here??
    StatusData.targetCurrent = incomingData.targetCurrent;
    StatusData.statusDataReady = false;  //clear queued message
  }

  //blocking i2c read every cycle????
  StatusData.busVoltage = ina219_CAPBANK.getBusVoltage_V();
  StatusData.myCurrent = ina219_CAPBANK.getShuntVoltage_mV() / 10.0;

  //HARD STOP ON CHARGING IF TOO HIGH
  if (StatusData.busVoltage >= STOP_VOLTAGE) {
    stopPWM();
    StatusData.STATE = STOPPED;
  }

  float busVoltages[100];

  switch (StatusData.STATE) {
    case INIT:                                                     //charge
      StatusData.dutyCycle = (0.02 + StatusData.busVoltage) / 24;  //derived* for ~0.5A charging
      break;
    case STOPPED:  //discharge according to i2c
      stopPWM();
      StatusData.STATE = STOPPED;
      break;

    case CHARGING:  //end of match discharge (do nothing b/c implemented in HW)
      stopPWM();
      StatusData.STATE = STOPPED;
      break;

    case DISCHARGING:
      stopPWM();
      break;

    default:  //do nothing
      stopPWM();
      StatusData.STATE = STOPPED;
      break;
  }

  outputPWM(StatusData.dutyCycle);
}
