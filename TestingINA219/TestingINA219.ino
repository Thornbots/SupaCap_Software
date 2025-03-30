#include <Wire.h>
#include <Adafruit_INA219.h>

TwoWire CurrentBus(18, 19);
Adafruit_INA219 ina219_CAPBANK(0x40);
Adafruit_INA219 ina219_PMM(0x41);
Adafruit_INA219 ina219_MOTORS(0x44);

void setup(){
  CurrentBus.begin();

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

  //TODO: add configuration for each chip (I think we will need to write this ourselves cause the library is shit)
  // if(!ina219_MOTORS.setMaxCurrentShunt(10, 0.01)){
  //   Serial.println("Failed Calibration on Motor INA");
  //   while (1) { delay(10); }
  // }

  Serial.println("Measuring voltage and current with ina219_PMM ...");
}

void loop(){
  // float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  // float loadvoltage = 0;
  // float power_mW = 0;

  // shuntvoltage = ina219_PMM.getShuntVoltage_mV();
  busvoltage = ina219_CAPBANK.getBusVoltage_V();
  current_mA = ina219_CAPBANK.getCurrent_mA();
  // power_mW = ina219_PMM.getPower_mW();
  // loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  // Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  // Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  // Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");

  delay(2000);
}
