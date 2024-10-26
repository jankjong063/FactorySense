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

void setup() {
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

void loop() {
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
