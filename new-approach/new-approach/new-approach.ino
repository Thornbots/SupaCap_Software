#include <Arduino.h>
#include "hardware/pwm.h"

#define PWM_TOP_PIN 16
#define PWM_BOT_PIN 17
#define PWM_FREQ_HZ 100000      //100 khz
#define PWM_DEADTIME 12         //in ticks for 504 nS deadtime
#define SYS_CLOCK_HZ 125000000  // Default 125 MHz for RP2040
const int SLICE_NUM = pwm_gpio_to_slice_num(PWM_TOP_PIN);
const uint32_t TOP_PWM = SYS_CLOCK_HZ / (PWM_FREQ_HZ * 2) - 1;  // Correct calculation for center-aligned mode

uint32_t levelA, levelB;

#define TARGET_CURRENT -0.1f
#define STOP_VOLTAGE 13

void outputPWM(float dutyCycle) {
  // Set duty cycle with dead time
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

#include <Wire.h>
#include <hardware/i2c.h>
#include <Adafruit_INA219.h>
//#include <I2CSlave.h>
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
  }
}  // cook here



//MIGHT NEED TO BE VOLATILE
//State, data, readout values, current time, etc, sent by pico as peripheral
struct __attribute__((packed)) statusDataFrame {
  int STATE = 0;
  float busVoltage = 0.0;
  float myCurrent = 0.0;
  float dutyCycle = 0.01;  //tbd if needed, could stop weird transient effects of slamming to 0, bc to stop we really want both pwm 0, not 0 duty cycle
  long currentTimeMillis = 0;
} statusData;

void DataBusOnRequest() {
  uint8_t sendBuffer[sizeof(statusData)];
  memcpy(sendBuffer, &statusData, sizeof(statusData));  //casting to bytes
  DataBus.write(sendBuffer, sizeof(sendBuffer));
}  // cook here



void setup() {
  statusData.currentTimeMillis = millis();
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
  // if (! ina219_CAPBANK.begin(&CurrentBus)) {
  //   Serial.println("Failed to find Cap Bank chip");
  //   while (1) { delay(10); }
  // }

  // if (! ina219_PMM.begin(&CurrentBus)) {
  //   Serial.println("Failed to find PMM chip");
  //   while (1) { delay(10); }
  // }

  // if (! ina219_MOTORS.begin(&CurrentBus)) {
  //   Serial.println("Failed to find Motor Output chip");
  //   while (1) { delay(10); }
  // }



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
  statusData.dutyCycle = 0;
  // lastVoltage = ina219_CAPBANK.getBusVoltage_V();
}




float busVoltages[100];

void loop() {



  //HARD STOP ON CHARGING IF TOO HIGH
  if (statusData.busVoltage >= STOP_VOLTAGE) {
    stopPWM();
  }

  //busvoltage = ina219_CAPBANK.getBusVoltage_V();
  //mycurrent = ina219_CAPBANK.getShuntVoltage_mV()/10.0;




  //TODO: Change state transitions to be a result of i2c, and make target current controlled by i2c aswell
  float testCurrent = 0;  //replace with targetcurrent
  if (statusData.busVoltage >= 12) {
    statusData.STATE = 1;
    testCurrent = 0.1f;
  }

  switch (statusData.STATE) {
    case 0:                                                        //charge
      statusData.dutyCycle = (0.02 + statusData.busVoltage) / 24;  //derived* for ~0.5A charging
      break;

    case 1:  //discharge according to i2c
      stopPWM();
      break;

    case 2:  //end of match discharge (do nothing b/c implemented in HW)
      stopPWM();
      break;

    default:  //do nothing
      stopPWM();
      break;
  }

  {
    long millisBetweenSerial = 100;
    long startTime = 0;
    statusData.currentTimeMillis = millis();
    if (statusData.currentTimeMillis - startTime > millisBetweenSerial) {
      startTime = statusData.currentTimeMillis;
      Serial.print("Duty Cycle:   ");
      Serial.print(statusData.dutyCycle);
      Serial.print(" % ");
      Serial.print("Bus Voltage:  ");
      Serial.print(statusData.busVoltage);
      Serial.print(" V ");
      Serial.print("my current:   ");
      Serial.print(statusData.myCurrent);
      Serial.print(" A ");
      Serial.print("TIME Micros:  ");
      Serial.print(statusData.currentTimeMillis);
      Serial.println(" us");
    }
  }



  outputPWM(statusData.dutyCycle);
  // if (millis() > 5000) {
  //   STATE = 1;
  // }
}


// void setup1(){
// }

// void loop1(){
// }