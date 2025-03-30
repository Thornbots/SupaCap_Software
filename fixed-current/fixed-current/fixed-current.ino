#include <Wire.h>
#include <Adafruit_INA219.h>

TwoWire CurrentBus(18, 19);
Adafruit_INA219 ina219_CAPBANK(0x40);
Adafruit_INA219 ina219_PMM(0x41);
Adafruit_INA219 ina219_MOTORS(0x44);


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

}

void loop() {
  float busvoltage = 0;
  float current_mA = 0;
  float mycurrent = 0;

  busvoltage = ina219_MOTORS.getBusVoltage_V();
  mycurrent = ina219_MOTORS.getShuntVoltage_mV()/0.01;
  current_mA = ina219_MOTORS.getCurrent_mA();

  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("ina current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("my current:       "); Serial.print(mycurrent); Serial.println(" mA");
  delay(2000);
}
