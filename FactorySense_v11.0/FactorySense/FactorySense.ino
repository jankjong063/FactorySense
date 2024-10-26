// Libraries for LCD and voltage sensor
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>

// Define pins for current and voltage sensors
#define CURRENT_SENSOR_PIN A0
#define VOLTAGE_SENSOR_PIN A1

// Set up the LCD pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Calibration constants for the sensors
const float VOLTAGE_CALIBRATION = 234.26;  // Adjust based on the sensor
const float CURRENT_CALIBRATION = 30.0;    // Adjust based on the sensor

void inner_setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);  // Initialize LCD (16x2 size)
  lcd.print("Power Meter");
  delay(2000);
  lcd.clear();
}

unsigned long previousMillis = 0;  // stores the last time display was updated
const long interval = 2000;        // interval at which to update display (2 seconds)
bool showPower = true;             // toggle between showing power and voltage/current

void inner_loop() {
  // Read current from SCT-013
  float current = readCurrent(CURRENT_SENSOR_PIN);

  // Read voltage from ZMPT101B
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);

  // Calculate power consumption (P = V * I)
  float power = voltage * current;

  // Get the current time
  unsigned long currentMillis = millis();

  // If 2 seconds have passed, update the display
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    lcd.clear();

    if (showPower) {
      // Display power on the LCD
      lcd.setCursor(0, 0);
      lcd.print("Power:");
      lcd.setCursor(0, 1);
      lcd.print(power);
      lcd.print(" W");
    } else {
      // Display voltage and current on the LCD
      lcd.setCursor(0, 0);
      lcd.print("Voltage:");
      lcd.print(voltage);
      lcd.print(" V");
      lcd.setCursor(0, 1);
      lcd.print("Current:");
      lcd.print(current);
      lcd.print(" A");
    }

    // Toggle the display flag
    showPower = !showPower;
  }

  // Print to Serial Monitor
  Serial.print("Current: ");
  Serial.print(current);
  Serial.print(" A, Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Power: ");
  Serial.print(power);
  Serial.println(" W");
}

float readCurrent(int pin) {
  int sensorValue = analogRead(pin);
  float current = (sensorValue - 512) * (5.0 / 1024.0) / CURRENT_CALIBRATION;
  return abs(current);  // Return the absolute current value
}

float readVoltage(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5.0 / 1024.0) * VOLTAGE_CALIBRATION;
  return voltage;
}



// SD card chip select pin
const int chipSelect = 4;  // Adjust based on your SD card module

void inner3_setup() {
  inner_setup();
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Power Meter");
  delay(2000);
  lcd.clear();

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    lcd.print("SD Init Failed");
    return;
  }
  Serial.println("SD card initialized.");
  lcd.print("SD Init Success");
  delay(2000);
  lcd.clear();
}

void logDataToSD(float voltage, float current, float power) {
  // Open the file for appending
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // If the file is available, write the data
  if (dataFile) {
    dataFile.print("Voltage: ");
    dataFile.print(voltage);
    dataFile.print(" V, Current: ");
    dataFile.print(current);
    dataFile.print(" A, Power: ");
    dataFile.print(power);
    dataFile.println(" W");
    dataFile.close();  // Close the file
    Serial.println("Data logged to SD card.");
  } else {
    Serial.println("Error opening datalog.txt");
  }
}

void inner3_loop() {
  inner_loop();
  // Read current from SCT-013
  float current = readCurrent(CURRENT_SENSOR_PIN);

  // Read voltage from ZMPT101B
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);

  // Calculate power consumption (P = V * I)
  float power = voltage * current;

  // Log data to SD card
  logDataToSD(voltage, current, power);

  // Existing alternating display code (from version 2.0)
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    lcd.clear();

    if (showPower) {
      lcd.setCursor(0, 0);
      lcd.print("Power:");
      lcd.setCursor(0, 1);
      lcd.print(power);
      lcd.print(" W");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Voltage:");
      lcd.print(voltage);
      lcd.print(" V");
      lcd.setCursor(0, 1);
      lcd.print("Current:");
      lcd.print(current);
      lcd.print(" A");
    }

    showPower = !showPower;
  }

  // Print to Serial Monitor
  Serial.print("Current: ");
  Serial.print(current);
  Serial.print(" A, Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Power: ");
  Serial.print(power);
  Serial.println(" W");
}

#include <SoftwareSerial.h>

// Bluetooth module pins
SoftwareSerial bluetooth(10, 11);  // RX, TX pins for Bluetooth module

void inner4_setup() {
  // Call the existing setup function for LCD and SD card
  inner3_setup();

  // Initialize Bluetooth serial communication
  bluetooth.begin(9600);
  Serial.println("Bluetooth initialized.");
}

void sendDataOverBluetooth(float voltage, float current, float power) {
  bluetooth.print("Voltage: ");
  bluetooth.print(voltage);
  bluetooth.print(" V, Current: ");
  bluetooth.print(current);
  bluetooth.print(" A, Power: ");
  bluetooth.print(power);
  bluetooth.println(" W");

  Serial.println("Data sent over Bluetooth.");
}

void inner4_loop() {
  // Call the existing loop function for data logging and display
  inner3_loop();

  // Read current from SCT-013
  float current = readCurrent(CURRENT_SENSOR_PIN);

  // Read voltage from ZMPT101B
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);

  // Calculate power consumption (P = V * I)
  float power = voltage * current;

  // Send data over Bluetooth
  sendDataOverBluetooth(voltage, current, power);
}

void inner5_setup() {
  // Call the existing setup function for LCD and SD card
  inner4_setup();
}

// New Feature: Real-time power consumption graphing

void plotPowerGraph(float power) {
  // This function will print the power value in a way that can be used by the Arduino Serial Plotter
  Serial.print("Power: ");
  Serial.println(power);  // Power value is plotted on the graph
}

void inner5_loop() {
  // Call the existing loop function for logging and Bluetooth transmission
  inner4_loop();

  // Read current from SCT-013
  float current = readCurrent(CURRENT_SENSOR_PIN);

  // Read voltage from ZMPT101B
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);

  // Calculate power consumption (P = V * I)
  float power = voltage * current;

  // Plot the power graph
  plotPowerGraph(power);

  // Send data over Bluetooth
  sendDataOverBluetooth(voltage, current, power);
}

void inner6_setup(){
  inner5_setup();
}

// New Feature: Calibration for current and voltage sensors

float calibrateCurrentSensor(float currentReading) {
  // Apply calibration factor to current sensor reading
  float calibratedCurrent = currentReading * CURRENT_CALIBRATION;
  return calibratedCurrent;
}

float calibrateVoltageSensor(float voltageReading) {
  // Apply calibration factor to voltage sensor reading
  float calibratedVoltage = voltageReading * VOLTAGE_CALIBRATION;
  return calibratedVoltage;
}

void inner6_loop() {
  // Call the existing loop function for logging and Bluetooth transmission
  inner5_loop();

  // Read current from SCT-013 and apply calibration
  float rawCurrent = readCurrent(CURRENT_SENSOR_PIN);
  float current = calibrateCurrentSensor(rawCurrent);

  // Read voltage from ZMPT101B and apply calibration
  float rawVoltage = readVoltage(VOLTAGE_SENSOR_PIN);
  float voltage = calibrateVoltageSensor(rawVoltage);

  // Calculate power consumption (P = V * I)
  float power = voltage * current;

  // Plot the power graph
  plotPowerGraph(power);

  // Send data over Bluetooth
  sendDataOverBluetooth(voltage, current, power);
}


// New Feature: Power Threshold Alert

const float POWER_THRESHOLD = 100.0;  // Example threshold (in watts)
const int ALERT_PIN = 13;             // Pin connected to the alert system (e.g., LED)

void inner7_setup() {
  inner6_setup();  // Call version 6.0 setup
  
  // Set up the alert pin
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);  // Turn off alert at start

  Serial.println("Version 7.0 Setup Complete");
}

void inner7_loop() {
  inner6_loop();  // Call version 6.0 loop

  // Check if power exceeds threshold
  float current = readCurrent(CURRENT_SENSOR_PIN);
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);
  float power = voltage * current;

  if (power > POWER_THRESHOLD) {
    digitalWrite(ALERT_PIN, HIGH);  // Turn on alert (e.g., LED on)
    Serial.println("Power threshold exceeded! Alert triggered.");
  } else {
    digitalWrite(ALERT_PIN, LOW);   // Turn off alert (e.g., LED off)
  }

  Serial.println("Version 7.0 Loop Running");
}


// New Feature: Timestamping for Readings

void logDataWithTimestamp(float voltage, float current, float power) {
  // Get the current timestamp
  unsigned long timestamp = millis();  // Use millis for simplicity; can be replaced with a real RTC module
  
  // Log the data with the timestamp to the Serial Monitor
  Serial.print("Timestamp: ");
  Serial.print(timestamp);
  Serial.print(", Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Current: ");
  Serial.print(current);
  Serial.print(" A, Power: ");
  Serial.print(power);
  Serial.println(" W");

  // If needed, log the timestamp to the SD card as well
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Timestamp: ");
    dataFile.print(timestamp);
    dataFile.print(", Voltage: ");
    dataFile.print(voltage);
    dataFile.print(" V, Current: ");
    dataFile.print(current);
    dataFile.print(" A, Power: ");
    dataFile.println(power);
    dataFile.close();
  } else {
    Serial.println("Error opening datalog.txt");
  }
}

void inner8_setup(){
  inner7_setup();
}

void inner8_loop() {
  inner7_loop();  // Call version 7.0 loop

  // Read current from SCT-013
  float current = readCurrent(CURRENT_SENSOR_PIN);

  // Read voltage from ZMPT101B
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);

  // Calculate power consumption (P = V * I)
  float power = voltage * current;

  // Log data with timestamp
  logDataWithTimestamp(voltage, current, power);

  // Check if power exceeds threshold (from v7.0)
  if (power > POWER_THRESHOLD) {
    digitalWrite(ALERT_PIN, HIGH);  // Turn on alert (e.g., LED on)
    Serial.println("Power threshold exceeded! Alert triggered.");
  } else {
    digitalWrite(ALERT_PIN, LOW);   // Turn off alert (e.g., LED off)
  }

  Serial.println("Version 8.0 Loop Running");
}


// New Feature: Energy Cost Estimation

const float COST_PER_KWH = 0.12;  // Cost per kilowatt-hour in USD

void inner9_setup() {
  inner8_setup();  // Call version 8.0 setup
  
  Serial.println("Version 9.0 Setup Complete with Energy Cost Estimation");
}

void inner9_loop() {
  inner8_loop();  // Call version 8.0 loop

  // Calculate power consumption (P = V * I)
  float current = readCurrent(CURRENT_SENSOR_PIN);
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);
  float power = voltage * current;  // Power in watts

  // Estimate energy cost based on power consumption
  float energyConsumedKWh = power / 1000.0;  // Convert power from watts to kilowatts
  float estimatedCost = energyConsumedKWh * COST_PER_KWH;

  // Print the estimated cost
  Serial.print("Estimated Cost: $");
  Serial.println(estimatedCost);

  Serial.println("Version 9.0 Loop Running");
}


// New Feature: Settings Menu on LCD

float newPowerThreshold = POWER_THRESHOLD;
float newCostPerKWH = COST_PER_KWH;

void displaySettingsMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1.Power Threshold");
  lcd.setCursor(0, 1);
  lcd.print("2.Cost/kWh");
  delay(3000);
}

void updateSettings(int option) {
  switch (option) {
    case 1:
      // Simulate adjusting power threshold
      newPowerThreshold = newPowerThreshold + 10.0;
      lcd.clear();
      lcd.print("Threshold: ");
      lcd.print(newPowerThreshold);
      delay(2000);
      break;
    case 2:
      // Simulate adjusting cost per kWh
      newCostPerKWH = newCostPerKWH + 0.02;
      lcd.clear();
      lcd.print("Cost/kWh: $");
      lcd.print(newCostPerKWH);
      delay(2000);
      break;
    default:
      lcd.clear();
      lcd.print("Invalid Option");
      delay(2000);
      break;
  }
}

void inner10_setup() {
  inner9_setup();  // Call version 9.0 setup
  
  displaySettingsMenu();
  int selectedOption = 1;  // For now, we simulate option selection. Adjust to your input method.
  updateSettings(selectedOption);

  Serial.println("Version 10.0 Setup Complete with Settings Menu");
}

void inner10_loop() {
  inner9_loop();  // Call version 9.0 loop

  // Simulate adjusting settings via a menu in the loop
  int selectedOption = 2;  // For now, we simulate option selection. Adjust to your input method.
  updateSettings(selectedOption);

  Serial.println("Version 10.0 Loop Running");
}


// New Feature: Cloud Integration for Remote Monitoring

void uploadDataToCloud(float voltage, float current, float power) {
  // Simulate cloud upload (this would use actual cloud API calls in a real system)
  Serial.print("Uploading to cloud: Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Current: ");
  Serial.print(current);
  Serial.print(" A, Power: ");
  Serial.println(power);
}

void setup() {
  inner10_setup();  // Call version 10.0 setup
  
  Serial.println("Version 11.0 Setup Complete with Cloud Integration");
}

void loop() {
  inner10_loop();  // Call version 10.0 loop

  // Simulate uploading data to the cloud
  float current = readCurrent(CURRENT_SENSOR_PIN);
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);
  float power = voltage * current;

  uploadDataToCloud(voltage, current, power);

  Serial.println("Version 11.0 Loop Running");
}
