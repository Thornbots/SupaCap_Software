#include <Arduino.h>
#include "hardware/pwm.h"
#include "PicoEncoder.h"
#include <pico/multicore.h>

#include <Wire.h>
#include <hardware/i2c.h>
#include <Adafruit_INA219.h>


//SERIAL SETTINGS
static const bool serialEnabled = true; //this should probably be done with C macros but oh well
static const bool printErrorValues = true;
static const bool printOtherCurrentValues = true;
static const long millisBetweenSerial = 250;
static float       printableError = -1.0;
static float       printableFeedForward = -1.0;




//CONTROL SETTINGS
//max should be roughly 0.5, but maybe slightly over to increase speed of charging? (tbd)
static const float MAX_DUTY_CYCLE = 0.475;
//24V down to 12 V at 50% dutycycle
static const float STEADY_STATE_VOLTAGE_DUTY_CYCLE_FACTOR = (1.0 / 24.0);
//NEEDS TO BE TUNED
static const float K_P = 1;
//limits current swings
static const float MAX_ERROR = 0.05;
//(Amps)
static const float MAX_REQUESTABLE_CURRENT = 3.0;
//Voltage at which caps will try to stop accepting more current
static const float MAX_RATED_VOLTAGE = 12.0;
//voltage at which system will emergency shut off
static const float MAX_EMERGENCY_SHUTOFF_VOLTAGE = 13.0;
//magically determined by ben, (should be overwritten before first outputPWM)
static const float STARTUP_DUTY_CYCLE = 0.25;
//tbd if needed, could stop weird transient effects of slamming to 0, bc to stop we really want both pwm 0, not 0 duty cycle
static const float STARTUP_CURRENT = -0.1;

//I2C Declarations
//PERIPEHERAL Pins
//board pin 21
static const uint I2C_AS_PERIPHERAL_SDA_GPIO = 16;
//board pin 22
static const uint I2C_AS_PERIPHERAL_SCL_GPIO = 17;
//nice
static const uint I2C_AS_PERIPHERAL_ADDRESS = 0x45;

//I2C Pins to Sensors:
//board pin 24
static const uint I2C_AS_CONTROLLER_SDA_GPIO = 18;
//board pin 25
static const uint I2C_AS_CONTROLLER_SCL_GPIO = 19;

//initializes both i2c0 and i2c1

//I2C0
TwoWire DataBus(I2C_AS_PERIPHERAL_SDA_GPIO, I2C_AS_PERIPHERAL_SCL_GPIO);
//I2C1
TwoWire CurrentBus(I2C_AS_CONTROLLER_SDA_GPIO, I2C_AS_CONTROLLER_SCL_GPIO);
Adafruit_INA219 ina219_CAPBANK(0x40);
Adafruit_INA219 ina219_PMM(0x41);
Adafruit_INA219 ina219_MOTORS(0x44);
//determined from resistors on board, we use a different resist
static const float SHUNT_VOLTAGE_RESISTOR_MILI_OHM = 10.0;

static int historyArrayIndex = 0;
static const int NUM_READINGS_AVERAGED = 10;
float voltageHistory[NUM_READINGS_AVERAGED], currentHistory[NUM_READINGS_AVERAGED];

enum BoardStatus {
  INIT,
  STOPPED,
  ACTIVE
};

enum i2cInstruction {
  INITIALIZE,
  STOP,
  NO_CHANGE
};

//MIGHT NEED TO BE VOLATILE?
//Instructions, values etc received by pico as peripheral
struct __attribute__((packed)) receivedFrame {
  i2cInstruction currentInstruction;
  float targetCurrent = STARTUP_CURRENT;
} incomingData;

//State, data, readout values, current time, etc, sent by pico as peripheral
struct __attribute__((packed)) CoreStatusDataFrame {
  BoardStatus STATE = INIT;
  float busVoltage = 0.0;
  float measuredCurrent = 0.0;
  float dutyCycle = STARTUP_DUTY_CYCLE;  //tbd if needed, see definition
  float targetCurrent = STARTUP_CURRENT;
  long currentTimeMillis = 0;
  volatile bool isI2CDataReceived = false;
} StatusData;


//When Type-C controller writes to the pico
void DataBusReceiveData(int numBytes) {
  if (numBytes == sizeof(incomingData)) {
    byte recvdata[sizeof(incomingData)];
    for (int i = 0; i < numBytes; i++) {
      recvdata[i] = DataBus.read();
    }
    memcpy(&incomingData, recvdata, sizeof(recvdata));  //gotta be same order or it'll geek
    StatusData.isI2CDataReceived = true;
  }
}  // cook here

//when the Type-C Requests data from the pico
void DataBusOnRequest() {
  static uint8_t sendBuffer[sizeof(StatusData)];
  memcpy(sendBuffer, &StatusData, sizeof(StatusData));  //casting to bytes
  DataBus.write(sendBuffer, sizeof(sendBuffer));
}  // cook here


//PWM DECLARATIONS
static const int PWM_TOP_PIN = 16;
static const int PWM_BOT_PIN = 17;
//100 khz
static const int PWM_FREQ_HZ = 100000;
//in ticks for 504 nS deadtime
static const int PWM_DEADTIME = 12;
// Default 125 MHz for RP2040
static const int SYS_CLOCK_HZ = 125000000;
static const int SLICE_NUM = pwm_gpio_to_slice_num(PWM_TOP_PIN);
// Correct calculation for center-aligned mode
static const uint32_t TOP_PWM = SYS_CLOCK_HZ / (PWM_FREQ_HZ * 2) - 1;
//higher than this results in overvolting the cap bank

void outputPWM(float dutyCycle) {
  // Set duty cycle with dead time
  dutyCycle = min(MAX_DUTY_CYCLE, dutyCycle);  //sets upper bound

  uint32_t levelA = dutyCycle * TOP_PWM;
  uint32_t levelB = min(levelA + PWM_DEADTIME, TOP_PWM);

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


void setup() {
  StatusData.currentTimeMillis = millis();
  if (serialEnabled) {
    Serial.begin(115200);
    while (!Serial) {
      // will pause until serial console opens DO NOT INCLUDE ON FINAL CODE
      delay(1);
    }
    Serial.println("Starting Up");
  }
  //i2c0 begining
  DataBus.begin(I2C_AS_PERIPHERAL_ADDRESS);
  DataBus.onReceive(DataBusReceiveData);
  DataBus.onRequest(DataBusOnRequest);

  //i2c1 begining
  if (!ina219_CAPBANK.begin(&CurrentBus)) {
    if (serialEnabled) Serial.println("Failed to find Cap Bank chip");
    while (1) { delay(10); }
  }


  if (!ina219_PMM.begin(&CurrentBus)) {
    if (serialEnabled) Serial.println("Failed to find PMM chip");
    while (1) { delay(10); }
  }

  if (!ina219_MOTORS.begin(&CurrentBus)) {
    if (serialEnabled) Serial.println("Failed to find Motor Output chip");
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


  for (int i = 1; i < NUM_READINGS_AVERAGED; i++) {  //technically we don't ever use the 0th value here bc its immediately overwritten in main loop
    voltageHistory[i] = ina219_CAPBANK.getBusVoltage_V();
    currentHistory[i] = ina219_CAPBANK.getShuntVoltage_mV() / SHUNT_VOLTAGE_RESISTOR_MILI_OHM;
  }

  if (serialEnabled) Serial.println("Entering Main Loop");
}


void loop() {
  if (StatusData.isI2CDataReceived) {
    //maybe parse inst here??
    switch (incomingData.currentInstruction) {  //parse instruction from i2c controller
      case INITIALIZE:
        StatusData.STATE = INIT;
        break;
      case STOP:
        StatusData.STATE = STOPPED;
        break;
      case NO_CHANGE:
        break;
      default:
        StatusData.STATE = STOPPED;
        break;
    }
    StatusData.targetCurrent = incomingData.targetCurrent;
    StatusData.isI2CDataReceived = false;  //clear queued message
  }

  {  //update voltages
    //blocking i2c read every cycle????
    StatusData.busVoltage = ina219_CAPBANK.getBusVoltage_V();
    StatusData.measuredCurrent = ina219_CAPBANK.getShuntVoltage_mV() / SHUNT_VOLTAGE_RESISTOR_MILI_OHM;
    voltageHistory[historyArrayIndex] = StatusData.busVoltage;
    currentHistory[historyArrayIndex] = StatusData.measuredCurrent;
    historyArrayIndex = (historyArrayIndex + 1) % NUM_READINGS_AVERAGED;
  }

  //HARD STOP ON CHARGING IF TOO HIGH
  if (StatusData.busVoltage >= MAX_EMERGENCY_SHUTOFF_VOLTAGE) {
    stopPWM();
    StatusData.STATE = STOPPED;
  }

  switch (StatusData.STATE) {
    case INIT:  //charge
      //StatusData.dutyCycle = (0.02 + StatusData.busVoltage) / 24;  //derived* for ~0.5A charging
      StatusData.STATE = ACTIVE;
      break;

    case STOPPED:  //discharge according to i2c
      stopPWM();
      StatusData.STATE = STOPPED;
      break;

    case ACTIVE:  //end of match discharge (do nothing b/c implemented in HW)
      {           // error calculation

        float limitedTargetCurrent = max(-MAX_REQUESTABLE_CURRENT, StatusData.targetCurrent);
        limitedTargetCurrent = min(StatusData.targetCurrent, MAX_REQUESTABLE_CURRENT);

        if (StatusData.busVoltage > MAX_RATED_VOLTAGE) {
          limitedTargetCurrent = max(limitedTargetCurrent, 0);  //forces attempt to cap voltage of caps at 12V (negative current is into caps)
        }

        float error = StatusData.targetCurrent- StatusData.measuredCurrent;
        error = max(-MAX_ERROR, error);  //lower limit (negative)
        error = min(error, MAX_ERROR);   //upper limit

        float feedForward = (StatusData.busVoltage * STEADY_STATE_VOLTAGE_DUTY_CYCLE_FACTOR);
        StatusData.dutyCycle = feedForward + (K_P * error);  //one term is voltage the other is current??
        if(printErrorValues){
          printableError = error;
          printableFeedForward = feedForward;
        }
      }

      outputPWM(StatusData.dutyCycle);
      break;

    default:  //do nothing
      stopPWM();
      StatusData.STATE = STOPPED;
      break;
  }


  if (serialEnabled) {
    static long startTime = 0;
    StatusData.currentTimeMillis = millis();
    if (StatusData.currentTimeMillis - startTime > millisBetweenSerial) {
      startTime = StatusData.currentTimeMillis;
      Serial.print("Duty Cycle:   ");
      Serial.print(StatusData.dutyCycle);
      Serial.print(" % ");
      Serial.print("Bus Voltage:  ");
      Serial.print(StatusData.busVoltage);
      Serial.print(" V ");
      Serial.print("Measured Current:   ");
      Serial.print(StatusData.measuredCurrent);
      Serial.print(" A ");
      Serial.print("Target Current:   ");
      Serial.print(StatusData.targetCurrent);
      Serial.print(" A ");
      if(printErrorValues){
        Serial.print("Computed Error:   ");
        Serial.print(printableError);
        Serial.print(" A ");
        Serial.print("Computed FeedForward:   ");
        Serial.print(printableFeedForward);
        Serial.print(" % ");
      }
      if(printOtherCurrentValues){
        Serial.print("SecondCurrent:   ");
        Serial.print(ina219_MOTORS.getShuntVoltage_mV() / SHUNT_VOLTAGE_RESISTOR_MILI_OHM);
        Serial.print(" A ");
      }
      Serial.print("Time Milis:  ");
      Serial.print(StatusData.currentTimeMillis);
      Serial.println(" ms");
    }
  }
}
