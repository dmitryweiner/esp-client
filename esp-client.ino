/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the BME280 Breakout board
  ----> http://www.adafruit.com/products/2650
  This sketch only supports the I2C bus for connection.
 ***************************************************************************/

#include <Wire.h>
#include <ESP8266WiFi.h>
#include "cactus_io_BME280_I2C.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


#define WLAN_SSID        "*******"
#define WLAN_PASS        "*******"
#define MQTT_SERVER      "192.168.0.24"
#define MQTT_SERVERPORT  1883                   // use 8883 for SSL
#define MQTT_USERNAME    "testuser"
#define MQTT_PASSWORD    "123456"

WiFiClient client;

// Create the BME280 object
BME280_I2C bme(0x76);

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

Adafruit_MQTT_Publish sensor = Adafruit_MQTT_Publish(&mqtt, "sensors/temperature");


void setup() {
  Serial.begin(115200);

  Wire.begin(2, 0);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  bme.setTempCal(-1);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");

}

void loop() {

  bme.readSensor();

  Serial.print("P="); Serial.print(bme.getPressure_MB()); Serial.print("mB\t\t");    // Pressure in millibars
  Serial.print("H="); Serial.print(bme.getHumidity()); Serial.print("%\t\t");
  Serial.print("T="); Serial.print(bme.getTemperature_C()); Serial.print(" *C\n");

  MQTT_connect();
  Serial.print("Sending sensor value ");
  if (! sensor.publish(bme.getTemperature_C())) {
    Serial.println("Failed");
  } else {
    Serial.println("OK!");
  }

  delay(2000);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
