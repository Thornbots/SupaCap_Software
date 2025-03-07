# SupaCap Software

This code is for the control board for the team's supercapacitor bank. It runs on a Raspberry Pi Pico and interfaces with an INA219 sensor, a STM32 board (Type C), and PWM peripherals
This board controls the output of the capacitor bank to meet an input specification from the Type C, and uses a PI controller to regulate this. The output is complementary PWM.
Everything is divided into sections, with the individual components broken up at the top. Setup and Loop are at the bottom

THIS IS UNTESTED I HAVE NO IDEA IF THIS WORKS. ALSO CHATGPT HELPED WHICH MAKES IT EVEN WORSE YAY

## Setup and Compilation Instructions

### 1. Get Arduino IDE and install Required Libraries/Boards

Before building the project, ensure you have the necessary tools installed:  
 - Library: Adafruit INA219
 - Board: Arduino Mbed OS RP2040 Boards

This should be all you need to get the code to compile

### 2. Flashing to the Raspberry Pi Pico  
1. Hold the **BOOTSEL** button on your Pico and connect it to your computer via USB.  
2. Upload using Arduino IDE
3. The Pico will automatically reboot and start running the program. 🚀  
