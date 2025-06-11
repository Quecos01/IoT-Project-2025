#include <WiFi.h>
#include <PubSubClient.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define DEVICE "ESP32SimBot"

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

// Battery voltage divider
const float R1 = 30000.0;
const float R2 = 7500.0;
const float ADC_MAX = 4095.0;
const float ADC_VREF = 3.3;

const char* movements[] = {"forward", "turnLeft", "turnRight", "stopped"};

const long interval = 1000;
unsigned long lastMsg = 0;
bool robotShutdown = false;

int button = 0;
bool buttonChanged = false;

void setup_wifi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32SimBot", mqtt_user, mqtt_pass)) {
      client.subscribe("robot/control");
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(1000);
    }
  }
}

void sendConnectivityStatus() {
  Point connStatus("robot_data");
  connStatus.addTag("device", DEVICE);
  connStatus.addTag("SSID", WiFi.SSID());
  connStatus.addField("wifi_state", (WiFi.status() == WL_CONNECTED) ? 1 : 0);
  connStatus.addField("mqtt_state", client.connected() ? 1 : 0);
  influxClient.writePoint(connStatus);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("MQTT message: ");
  Serial.println(msg);

  if (msg == "STOP") {
    button = 0;
    buttonChanged = true;
  } else if (msg == "START") {
    button = 1;
    buttonChanged = true;
  } else if (msg == "R MQTT") {
    Serial.println("Reconnecting MQTT...");
    Point mqttDown("robot_data");
    mqttDown.addTag("device", DEVICE);
    mqttDown.addTag("SSID", WiFi.SSID());
    mqttDown.addField("mqtt_state", 0);
    influxClient.writePoint(mqttDown);
    client.disconnect();
    delay(2000);
    reconnect();
    sendConnectivityStatus();
  } else if (msg == "R WIFI") {
    Serial.println("Reconnecting WiFi...");
    Point wifiDown("robot_data");
    wifiDown.addTag("device", DEVICE);
    wifiDown.addTag("SSID", WiFi.SSID());
    wifiDown.addField("wifi_state", 0);
    influxClient.writePoint(wifiDown);
    WiFi.disconnect();
    delay(2000);
    setup_wifi();
    sendConnectivityStatus();
  }
}

float readBatteryVoltage() {
  int adcValue = analogRead(batteryPin);
  float vOut = adcValue * ADC_VREF / ADC_MAX;
  float voltage = vOut / (R2 / (R1 + R2));
  return voltage;
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));

  setup_wifi();

  client.setServer(mqtt_server, 2222);
  client.setCallback(callback);

  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());

  Serial.println("Simulation ready.");
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  if (now - lastMsg > interval || buttonChanged) {
    lastMsg = now;

    float voltage = readBatteryVoltage();

    if (voltage < 1.0) {
      if (!robotShutdown) {
        client.publish("robot/warning", "Battery < 10% - Shutting down robot");
        robotShutdown = true;
      }
    } else if (robotShutdown && voltage >= 1.0) {
      client.publish("robot/status", "Battery > 10% - Resuming robot operations");
      robotShutdown = false;
    }

    if (!robotShutdown) {
      int moveIndex = random(0, 4);
      float distance = random(0, 5000) / 100.0;

      int wifiState = (WiFi.status() == WL_CONNECTED) ? 1 : 0;
      int mqttState = client.connected() ? 1 : 0;

      char statusMsg[64];
      snprintf(statusMsg, sizeof(statusMsg), "Bat:%.2fV Dist:%.2fcm", voltage, distance);
      client.publish("robot/status", statusMsg);

      if (voltage < 2.8) client.publish("robot/warning", "Battery < 30%");

      sensor.clearFields();
      sensor.addField("voltage", voltage);
      sensor.addField("distance", distance);
      sensor.addField("button", button);
      sensor.addField("wifi_state", wifiState);
      sensor.addField("mqtt_state", mqttState);
      sensor.addField("movement_code", moveIndex);

      Serial.println(sensor.toLineProtocol());
      if (!influxClient.writePoint(sensor)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(influxClient.getLastErrorMessage());
      }
    }

    buttonChanged = false;
  }
}
