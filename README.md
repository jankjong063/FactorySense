
# FactorySense

**FactorySense** is an Arduino-based power monitoring system designed to sense and track electricity usage in industrial environments. The project leverages current and voltage sensors to monitor real-time power consumption, providing valuable insights into factory energy usage.

## Features
- **Real-time Monitoring**: Measures both current and voltage to calculate real-time power consumption in watts.
- **Current and Voltage Sensors**: Utilizes SCT-013 current sensor and ZMPT101B voltage sensor for accurate measurements.
- **LCD Display**: Displays current, voltage, and power consumption directly on an LCD screen or through the Serial Monitor.
- **Energy Usage Insights**: Helps factories monitor and optimize their energy usage, contributing to reduced costs and improved efficiency.
- **Portable and Expandable**: Easily adaptable to different factory sizes and power setups.

## Hardware Requirements
- Arduino (e.g., Uno, Mega)
- SCT-013 Current Sensor
- ZMPT101B Voltage Sensor Module
- 16x2 LCD Display (Optional)
- Resistors, Jumpers, and Breadboard

## Getting Started
1. Clone this repository.
2. Wire up the components according to the schematic provided.
3. Upload the code to your Arduino.
4. Monitor the real-time power consumption via the Serial Monitor or LCD display.

## How It Works
FactorySense reads the current and voltage data from the sensors, computes the power consumption using the formula:

```
Power (W) = Voltage (V) * Current (A)
```

It then outputs this data to an LCD or through the Arduino Serial Monitor, providing factory managers with live energy usage data.

## Post-Build Automation
This project includes a post-build script (`postbuild.bat`) that automatically copies the compiled `.elf` file from the Arduino temporary directory to your project folder for easy access.

## License
This project is licensed under the MIT License - see the LICENSE file for details.

