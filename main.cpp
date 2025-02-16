#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize I2C LCD (adjust the address 0x22 if necessary)
LiquidCrystal_I2C lcd(0x22, 16, 2);

// Pin Definitions
#define TEMP_PIN A0        // Temperature sensor (TMP36)
#define GAS_PIN A1         // Gas/Air quality sensor (MQ series)
#define HUMIDITY_PIN A2    // Humidity sensor
#define PRESSURE_PIN A3    // Pressure sensor
#define RAIN_SENSOR_PIN 6  // Rain sensor
#define BUZZER_PIN 4       // Piezo buzzer
#define BUTTON_PIN 8       // Push button for mode switch

// Variables for button debounce
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200;  // milliseconds
bool lastButtonState = HIGH;        // the previous state of the button
bool buttonState = HIGH;            // current state of the button

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("Weather Station Starting...");

  // Initialize LCD
  lcd.init();
  lcd.backlight();  // Turn on the backlight initially
  lcd.print("Weather Station");
  delay(2000);

  // Configure Pins
  pinMode(TEMP_PIN, INPUT);
  pinMode(GAS_PIN, INPUT);
  pinMode(HUMIDITY_PIN, INPUT);
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Set button pin with pull-up resistor
}

void loop() {
  // Read button state (with debounce)
  bool reading = digitalRead(BUTTON_PIN);
  
  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // Reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Update the button state if it has changed
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {  // Button is pressed (switch is ON)
        lcd.backlight();        // Turn on the LCD backlight
      } else {                   // Button is not pressed (switch is OFF)
        lcd.noBacklight();      // Turn off the LCD backlight immediately
        clearLCD();             // Clear LCD immediately when the button is released
      }
    }
  }

  // Save the current button state for next loop
  lastButtonState = reading;

  // If the switch is ON, display the data
  if (buttonState == LOW) {  // When button is pressed (switch ON)
    displayWeatherData();   // Show weather data
  }

  // Delay for a moment to avoid fast button bouncing
  delay(50);
}

// Function to Display Weather Data on LCD
void displayWeatherData() {
  // Read Sensors
  float temperature = readTemperature();
  float airQuality = readAirQuality();
  int humidity = readHumidity();
  int pressure = readPressure();
  bool rainDetected = isRainDetected();

  // Determine Weather Condition
  String weatherCondition = determineWeatherCondition(temperature);

  // Determine Air Quality Condition
  String airQualityCondition = determineAirQualityCondition(airQuality);

  // Display Data on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C ");
  lcd.print(weatherCondition);
  lcd.setCursor(0, 1);
  lcd.print("AQ:");
  lcd.print(airQualityCondition);
 
  // Ensure the pressure value fits within the LCD space
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("P:");
  lcd.print(pressure);
  lcd.print("hPa");
  lcd.setCursor(0, 1);
  if (rainDetected) {
    lcd.print("Rain: Yes");
  } else {
    lcd.print("Rain: No");
  }
  lcd.print("  H:");
  lcd.print(humidity);
  lcd.print("% ");
  
  delay(3000); 

  // Analyze conditions and trigger buzzer if necessary
  analyzeConditions(temperature, airQuality, rainDetected);
}

// Function to Clear the LCD
void clearLCD() {
  lcd.clear();  // Clears the LCD display
}

// Function to Read Temperature from TMP36
float readTemperature() {
  int rawTemp = analogRead(TEMP_PIN);
  float voltage = rawTemp * (5.0 / 1023.0);  // Convert analog reading to voltage
  float temperature = (voltage - 0.5) / 0.01;  // Convert voltage to Celsius
  return temperature;
}

// Function to Read Air Quality from MQ Sensor
float readAirQuality() {
  int rawGas = analogRead(GAS_PIN);
  float voltage = rawGas * (5.0 / 1023.0);  // Convert analog reading to voltage
  float ppm = voltage * 1000;  // Convert voltage to ppm (based on sensor characteristics)
  return ppm;
}

// Function to Read Humidity
int readHumidity() {
  int rawHumidity = analogRead(HUMIDITY_PIN);
  return map(rawHumidity, 0, 1023, 0, 100);  // Map the reading to 0-100%
}

// Function to Read Pressure
int readPressure() {
  int rawPressure = analogRead(PRESSURE_PIN);
  // Map the raw pressure reading to the desired range (950 - 1050 hPa)
  return map(rawPressure, 0, 1023, 950, 1050);  // Range set from 950 to 1050 hPa
}

// Function to Detect Rain
bool isRainDetected() {
  return digitalRead(RAIN_SENSOR_PIN) == HIGH;  // HIGH if rain is detected
}

// Function to Determine Weather Condition Based on Temperature
String determineWeatherCondition(float temperature) {
  if (temperature > 40) return "Very Hot";
  if (temperature > 25) return "Hot";
  if (temperature > 15) return "Warm";
  if (temperature > 5) return "Cool";
  return "Cold";
}

// Function to Determine Air Quality Condition
String determineAirQualityCondition(float airQuality) {
  if (airQuality < 300) return "Good";
  if (airQuality < 1000) return "Moderate";
  return "Poor";
}

// Function to Analyze Conditions and Trigger Buzzer
void analyzeConditions(float temperature, float airQuality, bool rainDetected) {
  bool tempCritical = (temperature < 5 || temperature > 40);  // Critical temperature range
  bool airQualityPoor = (airQuality > 1014); 
  // Poor air quality threshold
  if (tempCritical || airQualityPoor || rainDetected) {
    lcd.clear();
    lcd.setCursor(0, 0);
    if (tempCritical) {
      lcd.print("Temp Critical!");
    }
    if (airQualityPoor) {
      lcd.setCursor(0, 1);
      lcd.print("Air Quality Bad!");
    }
    if (rainDetected) {
      lcd.setCursor(0, 1);
      lcd.print("Rain Detected!");
    }
    soundBuzzer();  // Trigger Buzzer Alert
  }
}

// Function to Sound Buzzer
void soundBuzzer() {
  tone(BUZZER_PIN, 1000, 500);  // Sound buzzer at 1000Hz for 500ms
  delay(500);                  // Pause before next buzz
}
