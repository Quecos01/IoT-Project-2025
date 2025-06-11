#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define listenerport 2222
#define DEVICE "ESP32Robot"

// WiFi
const char* ssid = "bbox4-71c2";
const char* password = "13620158";

// MQTT
const char* mqtt_server = "quecos.local";
const char* mqtt_user = "linepilot";
const char* mqtt_pass = "stayinline";
WiFiClient espClient;
PubSubClient client(espClient);

// InfluxDB
#define INFLUXDB_URL "http://192.168.0.113:8086"
#define INFLUXDB_TOKEN "EpSkGXMgIK67uM9Moe9TmuXjv_h5ZIqg57Pz4R_PSKna_LqKcnVVzrhsZ6Rw7nyR9mGn5AuJ-mTN4d-Ee7v9Qw=="
#define INFLUXDB_ORG "102b7832fd343794"
#define INFLUXDB_BUCKET "linerobotbucket"

InfluxDBClient influxClient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
Point sensor("robot_data");

// Pins
const int motorLeftFwd = 5;
const int motorLeftBwd = 18;
const int motorRightFwd = 19;
const int motorRightBwd = 21;
const int irLeft = 34;
const int irCenter = 32;
const int irRight = 35;
const int batteryPin = 36;
const int trigPin = 25;
const int echoPin = 26;
const int buttonPin = 27;

// LCD
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// Battery voltage divider
const float R1 = 30000.0;
const float R2 = 7500.0;
const float ADC_MAX = 4095.0;
const float ADC_VREF = 3.3;

// Timing
unsigned long lastMsg = 0;
const long interval = 1000;

// Button debounce
Bounce debouncer = Bounce();
bool stopped = false;
bool manualOverride = false;
unsigned long stopStartTime = 0;
String currentMovement = "stopped";

void setup() {
  Serial.begin(115200);
  Serial.println("Setup starting...");

  pinMode(motorLeftFwd, OUTPUT);
  pinMode(motorLeftBwd, OUTPUT);
  pinMode(motorRightFwd, OUTPUT);
  pinMode(motorRightBwd, OUTPUT);
  pinMode(irLeft, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  debouncer.attach(buttonPin);
  debouncer.interval(25);

  lcd.init();
  lcd.backlight();

  setup_wifi();
  client.setServer(mqtt_server, listenerport);
  client.setCallback(callback);

  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());

  Serial.println("Setup complete.");
}

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi:C M:X");
  lcd.setCursor(0, 1);
  lcd.print("Bat:--.--V");
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Robot", mqtt_user, mqtt_pass)) {
      client.subscribe("robot/control");
      Serial.println("MQTT connected.");
      lcd.setCursor(7, 0);
      lcd.print("MQTT:C");
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.println(client.state());
      delay(1000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("MQTT message: ");
  Serial.println(msg);

  if (msg == "STOP") {
    stopped = true;
    manualOverride = true;
    stopStartTime = millis();
    stopMotors();
    updateMovement("stopped");
  } else if (msg == "START") {
    stopped = false;
    manualOverride = false;
  }
}

float readBatteryVoltage() {
  int adcValue = analogRead(batteryPin);
  float vOut = adcValue * ADC_VREF / ADC_MAX;
  float voltage = vOut / (R2 / (R1 + R2));
  return voltage;
}

float readDistanceCM() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void logToInflux(float voltage, float distance, int buttonState, const char* movement) {
  if (WiFi.status() != WL_CONNECTED) return;

  sensor.clearFields();
  sensor.addField("voltage", voltage);
  sensor.addField("button", buttonState);
  sensor.addField("wifi", WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
  sensor.addField("mqtt", client.connected() ? "connected" : "disconnected");
  sensor.addField("distance", distance);
  sensor.addField("movement", movement);

  Serial.print("Writing: ");
  Serial.println(sensor.toLineProtocol());

  if (!influxClient.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(influxClient.getLastErrorMessage());
  }
}

void updateMovement(String newMovement) {
  if (currentMovement != newMovement) {
    currentMovement = newMovement;
    logToInflux(readBatteryVoltage(), readDistanceCM(), digitalRead(buttonPin) == LOW ? 1 : 0, currentMovement.c_str());
  }
}

void moveForward() {
  analogWrite(motorLeftFwd, 150);
  analogWrite(motorLeftBwd, 0);
  analogWrite(motorRightFwd, 150);
  analogWrite(motorRightBwd, 0);
}

void turnLeft() {
  analogWrite(motorLeftFwd, 0);
  analogWrite(motorLeftBwd, 100);
  analogWrite(motorRightFwd, 100);
  analogWrite(motorRightBwd, 0);
}

void turnRight() {
  analogWrite(motorLeftFwd, 100);
  analogWrite(motorLeftBwd, 0);
  analogWrite(motorRightFwd, 0);
  analogWrite(motorRightBwd, 100);
}

void stopMotors() {
  analogWrite(motorLeftFwd, 0);
  analogWrite(motorLeftBwd, 0);
  analogWrite(motorRightFwd, 0);
  analogWrite(motorRightBwd, 0);
}

void followLine() {
  float distance = readDistanceCM();
  if (distance < 10) {
    stopMotors();
    updateMovement("stopped");
    client.publish("robot/status", "Obstacle Detected");
    delay(500);
    turnLeft(); updateMovement("turnLeft"); delay(500);
    moveForward(); updateMovement("forward"); delay(1000);
    turnRight(); updateMovement("turnRight"); delay(700);
    moveForward(); updateMovement("forward"); delay(1000);
    turnLeft(); updateMovement("turnLeft"); delay(100);
    return;
  }

  int left = digitalRead(irLeft);
  int center = digitalRead(irCenter);
  int right = digitalRead(irRight);

  if (center && !left && !right) { moveForward(); updateMovement("forward"); }
  else if (left && !center && !right) { turnLeft(); updateMovement("turnLeft"); }
  else if (right && !center && !left) { turnRight(); updateMovement("turnRight"); }
  else if (center && left) { turnLeft(); updateMovement("turnLeft"); }
  else if (center && right) { turnRight(); updateMovement("turnRight"); }
  else if (left && center && right) { moveForward(); updateMovement("forward"); }
  else { stopMotors(); updateMovement("stopped"); }
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  debouncer.update();
  if (debouncer.fell() && !manualOverride) { // Allow button only if not overridden
    stopped = !stopped;
    stopStartTime = millis();
    Serial.print("Button press. Stopped: ");
    Serial.println(stopped);
    if (stopped) {
      stopMotors();
      updateMovement("stopped");
    }
  }

  if (stopped && millis() - stopStartTime >= 30000) return;
  if (!stopped) followLine();

  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;
    float batteryVoltage = readBatteryVoltage();
    float distance = readDistanceCM();
    int buttonState = digitalRead(buttonPin) == LOW ? 1 : 0;

    char statusMsg[64];
    snprintf(statusMsg, sizeof(statusMsg), "Bat:%.2fV Dist:%.2fcm", batteryVoltage, distance);
    client.publish("robot/status", statusMsg);
    if (batteryVoltage < 6.6) client.publish("robot/warning", "Battery < 30%");
    if (batteryVoltage < 5.0) client.publish("robot/warning", "Battery < 10%");

    lcd.setCursor(0, 0);
    lcd.print("WiFi:");
    lcd.print(WiFi.status() == WL_CONNECTED ? "C" : "X");
    lcd.print(" M:");
    lcd.print(client.connected() ? "C" : "X");

    lcd.setCursor(0, 1);
    lcd.print("Bat:");
    lcd.print(batteryVoltage, 2);
    lcd.print("V   ");

    logToInflux(batteryVoltage, distance, buttonState, currentMovement.c_str());
  }
}
