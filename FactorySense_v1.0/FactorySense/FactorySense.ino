// Libraries for LCD and voltage sensor
#include <LiquidCrystal.h>

// Define pins for current and voltage sensors
#define CURRENT_SENSOR_PIN A0
#define VOLTAGE_SENSOR_PIN A1

// Set up the LCD pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Calibration constants for the sensors
const float VOLTAGE_CALIBRATION = 234.26;  // Adjust based on the sensor
const float CURRENT_CALIBRATION = 30.0;    // Adjust based on the sensor

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);  // Initialize LCD (16x2 size)
  lcd.print("Power Meter");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Read current from SCT-013
  float current = readCurrent(CURRENT_SENSOR_PIN);

  // Read voltage from ZMPT101B
  float voltage = readVoltage(VOLTAGE_SENSOR_PIN);

  // Calculate power consumption (P = V * I)
  float power = voltage * current;

  // Print to Serial Monitor
  Serial.print("Current: ");
  Serial.print(current);
  Serial.print(" A, Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Power: ");
  Serial.print(power);
  Serial.println(" W");

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("I: ");
  lcd.print(current);
  lcd.print(" A");
  lcd.setCursor(0, 1);
  lcd.print("P: ");
  lcd.print(power);
  lcd.print(" W");

  delay(1000);  // Wait 1 second before next reading
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

