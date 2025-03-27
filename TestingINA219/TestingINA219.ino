#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219_PMM(0x41);


void setup(void) 
{
  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
  // Adafruit_INA219 ina219 = new Adafruit_INA219(1);
  Serial.println("Hello!");
  Serial.println("Hello2!");

  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  Wire.setSCL(19);
  Wire.setSDA(18);
  Serial.println("Hello3!");
  if (! ina219_PMM.begin()) {
    Serial.println("Failed to find ina219_PMM chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219_PMM.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219_PMM.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with ina219_PMM ...");
}

void loop(void) 
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219_PMM.getShuntVoltage_mV();
  busvoltage = ina219_PMM.getBusVoltage_V();
  current_mA = ina219_PMM.getCurrent_mA();
  power_mW = ina219_PMM.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");

  delay(2000);
}
