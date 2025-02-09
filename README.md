# SupaCap Software

## Setup and Compilation Instructions

### 1. Install Dependencies  
Before building the project, ensure you have the necessary tools installed:  

```bash
sudo apt update && sudo apt install -y cmake python3 build-essential \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

### 2. Build the Project  
Run the following commands in your project directory:  

```bash
mkdir -p build
cd build
cmake ..
make
cd ..
```

### 3. Flashing to the Raspberry Pi Pico  
1. Hold the **BOOTSEL** button on your Pico and connect it to your computer via USB.  
2. Drag and drop the compiled `.uf2` file from the `build` directory onto the **RPI-RP2** drive:  

   ```bash
   cp build/*.uf2 /media/$USER/RPI-RP2/
   ```

3. The Pico will automatically reboot and start running the program. 🚀  
