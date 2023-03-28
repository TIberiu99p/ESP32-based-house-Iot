#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

//  network credentials
const char* ssid = "SSID";
const char* password = "PASSWORD";

// Telegram token and chat ID
#define BOT_TOKEN "BOT_TOKEN"
#define TELEGRAM_CHAT_ID "CHAT_ID"

// Initialize the BME680 sensor
Adafruit_BME680 bme;

// Pin numbers for the rain sensor, PIR sensor, and LEDs
const int RAIN_SENSOR_PIN = 16;
const int PIR_SENSOR_PIN = 17;
const int LED_GREEN_PIN = 2;
const int LED_RED_PIN = 4;

// Variables for storing sensor data and state
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
float gasResistance = 0.0;
bool motionDetected = false;
bool rainDetected = false;
bool previousRainDetected = false;

// Define the WiFi and Telegram client objects
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

void setup() {
  Serial.begin(115200);

  // Initialize the BME680 sensor
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  // Set up the rain sensor pin as an input
  pinMode(RAIN_SENSOR_PIN, INPUT);

  // Set up the PIR sensor pin as an input
  pinMode(PIR_SENSOR_PIN, INPUT);

  // Set up the green LED pin as an output
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);

  // Set up the red LED pin as an output
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);
}

void loop() {
  // Read sensor data
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0;
  gasResistance = bme.readGasResistance() / 1000.0;

  // Check for rain
  rainDetected = digitalRead(RAIN_SENSOR_PIN);

  // Check for motion
  motionDetected = digitalRead(PIR_SENSOR_PIN);

  // Send Telegram notifications based on sensor data and state
  if (temperature >= 30.0) {
    String message = "High temperature alert! The temperature is " + String(temperature) + "C";
    bot.sendMessage(TELEGRAM_CHAT_ID, message);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_RED_PIN, LOW);
  }

  if (humidity >= 70.0) {
    String message = "High humidity alert! The humidity is " + String(humidity) + "%";
    bot.sendMessage(TELEGRAM_CHAT_ID, message);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_RED_PIN, LOW);
  }

  if (pressure <= 990.0) {
    String message = "Low pressure alert! The pressure is " + String(pressure) + "hPa";
    bot.sendMessage(TELEGRAM_CHAT_ID, message);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_RED_PIN, LOW);
  }

  if (gasResistance >= 100000.0) {
    String message = "High air quality alert! The gas resistance is " + String(gasResistance) + "kOhms";
    bot.sendMessage(TELEGRAM_CHAT_ID, message);
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(500);
    digitalWrite(LED_GREEN_PIN, LOW);
  }

  if (rainDetected && !previousRainDetected) {
    String message = "Rain detected! It's currently raining.";
    bot.sendMessage(TELEGRAM_CHAT_ID, message);
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(500);
    digitalWrite(LED_GREEN_PIN, LOW);
  }

  if (motionDetected) {
    String message = "Motion detected! There's someone in the room.";
    bot.sendMessage(TELEGRAM_CHAT_ID, message);
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(500);
    digitalWrite(LED_GREEN_PIN, LOW);
  }

  // Store the current rain state for comparison in the next loop
  previousRainDetected = rainDetected;

  // Wait for 1 second before reading sensor data again
  delay(1000);
}

