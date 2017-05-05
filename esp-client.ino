/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the BME280 Breakout board
  ----> http://www.adafruit.com/products/2650
  This sketch only supports the I2C bus for connection.
 ***************************************************************************/

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include "cactus_io_BME280_I2C.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


#define WLAN_SSID        "*******"
#define WLAN_PASS        "*******"
#define MQTT_SERVER      "192.168.0.24"
#define MQTT_SERVERPORT  1883                   // use 8883 for SSL
#define MQTT_USERNAME    "testuser"
#define MQTT_PASSWORD    "123456"
#define MAX_TRIES        20
#define SENSOR_NAME      "bme280"
#define USE_DEFAULTS     0
#define SLEEP_TIME       300


WiFiClient client;
ESP8266WebServer server(80);
struct eeprom_data_t {
  char wlanSsid[16];
  char wlanPassword[16];
  char mqttServer[64];
  short unsigned int mqttPort;
  char mqttUsername[16];
  char mqttPassword[16];
  char sensorName[64];
} eeprom_data;

// Create the BME280 object
BME280_I2C bme(0x76);//NOTE: this address can differ from chip to chip

Adafruit_MQTT_Client * mqtt;

Adafruit_MQTT_Publish * sensor;


void setup() {
  int currentTry = 1;
  char temperature_buffer[10];
  char humidity_buffer[10];
  char pressure_buffer[20];
  char buffer[150];

  Serial.begin(115200);
  Wire.begin(2, 0);

  readSettingsFromEEPROM();

  mqtt = new Adafruit_MQTT_Client(&client, eeprom_data.mqttServer, eeprom_data.mqttPort, eeprom_data.mqttUsername, eeprom_data.mqttPassword);
  char * sensorName = (char *) malloc(80);
  strcpy(sensorName, "sensors/");
  strcat(sensorName, eeprom_data.sensorName);
  sensor = new Adafruit_MQTT_Publish(mqtt, sensorName);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  bme.setTempCal(-5);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(eeprom_data.wlanSsid);

  WiFi.begin(eeprom_data.wlanSsid, eeprom_data.wlanPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
    currentTry++;
    if (currentTry > MAX_TRIES) {
      Serial.println("Switching to access point mode");
      WiFi.softAP("esp8266");
      server.on("/", HTTP_GET, []() {
        Serial.println("GET request");
        drawSettingsPage();
      });
      server.on("/", HTTP_POST, []() {
        Serial.println("POST request");
        writeSettingsToEEPROM(
          server.arg("wlanSsid").c_str(),
          server.arg("wlanPassword").c_str(),
          server.arg("mqttServer").c_str(),
          server.arg("mqttPort").toInt(),
          server.arg("mqttUsername").c_str(),
          server.arg("mqttPassword").c_str(),
          server.arg("sensorName").c_str()
        );
        server.send(200, "text/plain", "OK");
        ESP.reset();
      });
      server.begin();
      while (1) {
        server.handleClient();
        delay(1);
      }
    }
  }
  Serial.println("WiFi connected");

  bme.readSensor();
  MQTT_connect();

  dtostrf(bme.getTemperature_C(), 5, 2, temperature_buffer);
  dtostrf(bme.getHumidity(), 5, 2, humidity_buffer);
  dtostrf(bme.getPressure_MB(), 8, 2, pressure_buffer);

  sprintf(buffer, "{\"parameters\": [{\"type\": \"temperature\", \"value\": %s}, {\"type\": \"humidity\", \"value\": %s}, {\"type\": \"pressure\", \"value\": %s}]}", temperature_buffer, humidity_buffer, pressure_buffer);
  Serial.print("Sending JSON: ");
  Serial.println(buffer);
  if (! sensor->publish(buffer)) {
    Serial.println("Failed");
  } else {
    Serial.println("OK!");
  }

  ESP.deepSleep(SLEEP_TIME * 1000000);
}

void loop() {
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt->connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt->connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt->connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt->disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void drawSettingsPage() {
  String s = "<h1>Sensor settings</h1>";
  s += "<form method='POST'>";
  s += "<h2>Wifi</h2>";
  s += "SSID:<input type='text' name='wlanSsid' length='16' value='" + String(eeprom_data.wlanSsid) + "'><br/>";
  s += "Password:<input type='password' name='wlanPassword' length='16' value='" + String(eeprom_data.wlanPassword) + "'><br/>";
  s += "<h2>MQTT</h2>";
  s += "Server:<input type='text' name='mqttServer' length='64' value='" + String(eeprom_data.mqttServer) + "'><br/>";
  s += "Port:<input type='text' name='mqttPort' length='5' value='" + String(eeprom_data.mqttPort) + "'><br/>";
  s += "Username:<input type='text' name='mqttUsername' length='16' value='" + String(eeprom_data.mqttUsername) + "'><br/>";
  s += "Password:<input type='password' name='mqttPassword' length='16' value='" + String(eeprom_data.mqttPassword) + "'><br/>";
  s += "<h2>Sensor</h2>";
  s += "Name:<input type='text' name='sensorName' length='16' value='" + String(eeprom_data.sensorName) + "'><br/>";
  s += "<input type='submit' value='Save!'></form>";
  server.send(200, "text/html", s);
}

void readSettingsFromEEPROM() {
  int i;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  if (USE_DEFAULTS) {
    Serial.println("Using defaults");
    strncpy(eeprom_data.wlanSsid, WLAN_SSID, sizeof(WLAN_SSID));
    strncpy(eeprom_data.wlanPassword, WLAN_PASS, sizeof(WLAN_PASS));
    strncpy(eeprom_data.mqttServer, MQTT_SERVER, sizeof(MQTT_SERVER));
    eeprom_data.mqttPort = MQTT_SERVERPORT;
    strncpy(eeprom_data.mqttUsername, MQTT_USERNAME, sizeof(MQTT_USERNAME));
    strncpy(eeprom_data.mqttPassword, MQTT_PASSWORD, sizeof(MQTT_PASSWORD));
    strncpy(eeprom_data.sensorName, SENSOR_NAME, sizeof(SENSOR_NAME));
  } else {
    Serial.println("Reading config from EEPROM");

    EEPROM.begin(sizeof(eeprom_data));
    for (int i = 0; i < sizeof(eeprom_data); ++i) {
      eeprom_data_tmp[i] = EEPROM.read(i);
    }
    memcpy(&eeprom_data, eeprom_data_tmp,  sizeof(eeprom_data));

    //setting default values
    if (String(eeprom_data.wlanSsid).equals("")) {
      strncpy(eeprom_data.wlanSsid, WLAN_SSID, sizeof(WLAN_SSID));
    }
    if (String(eeprom_data.wlanPassword).equals("")) {
      strncpy(eeprom_data.wlanPassword, WLAN_PASS, sizeof(WLAN_PASS));
    }
    if (String(eeprom_data.mqttServer).equals("")) {
      strncpy(eeprom_data.mqttServer, MQTT_SERVER, sizeof(MQTT_SERVER));
    }
    if (eeprom_data.mqttPort == 0) {
      eeprom_data.mqttPort = MQTT_SERVERPORT;
    }
    if (String(eeprom_data.mqttUsername).equals("")) {
      strncpy(eeprom_data.mqttUsername, MQTT_USERNAME, sizeof(MQTT_USERNAME));
    }
    if (String(eeprom_data.mqttPassword).equals("")) {
      strncpy(eeprom_data.mqttPassword, MQTT_PASSWORD, sizeof(MQTT_PASSWORD));
    }
    if (String(eeprom_data.sensorName).equals("")) {
      strncpy(eeprom_data.sensorName, SENSOR_NAME, sizeof(SENSOR_NAME));
    }
    Serial.println("Finished reading config from EEPROM");
  }
  Serial.println(eeprom_data.wlanSsid);
  Serial.println(eeprom_data.wlanPassword);
  Serial.println(eeprom_data.mqttServer);
  Serial.println(eeprom_data.mqttPort);
  Serial.println(eeprom_data.mqttUsername);
  Serial.println(eeprom_data.mqttPassword);
  Serial.println(eeprom_data.sensorName);
}

void writeSettingsToEEPROM(
  const char * newWlanSsid,
  const char * newWlanPassword,
  const char * newMqttServer,
  unsigned short int newMqttPort,
  const char * newMqttUsername,
  const char * newMqttPassword,
  const char * newSensorName) {
  int i;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  strncpy(eeprom_data.wlanSsid, newWlanSsid, sizeof(eeprom_data.wlanSsid));
  strncpy(eeprom_data.wlanPassword, newWlanPassword, sizeof(eeprom_data.wlanPassword));
  strncpy(eeprom_data.mqttServer, newMqttServer, sizeof(eeprom_data.mqttServer));
  strncpy(eeprom_data.mqttUsername, newMqttUsername, sizeof(eeprom_data.mqttUsername));
  strncpy(eeprom_data.mqttPassword, newMqttPassword, sizeof(eeprom_data.mqttPassword));
  eeprom_data.mqttPort = newMqttPort;
  strncpy(eeprom_data.sensorName, newSensorName, sizeof(eeprom_data.sensorName));

  Serial.println(eeprom_data.wlanSsid);
  Serial.println(eeprom_data.wlanPassword);
  Serial.println(eeprom_data.mqttServer);
  Serial.println(eeprom_data.mqttPort);
  Serial.println(eeprom_data.mqttUsername);
  Serial.println(eeprom_data.mqttPassword);
  Serial.println(eeprom_data.sensorName);

  memcpy(eeprom_data_tmp, &eeprom_data, sizeof(eeprom_data));
  EEPROM.begin(sizeof(eeprom_data));
  for (int i = 0; i < sizeof(eeprom_data); ++i) {
    EEPROM.write(i, eeprom_data_tmp[i]);
  }
  EEPROM.commit();

  Serial.println("Finished writing config to EEPROM");

}

