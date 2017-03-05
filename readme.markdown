# Main features
* Solar power + accumulator
* Saving power between sending data
  ESP.deepSleep(60000000, WAKE_RF_DEFAULT);
* Setup WiFi AP and MQTT brocker url at the begining

# Connecting sensor

How to connect BMP280 sensor to ESP8266 (NodeMCU in my case).

Firstly I tried to use standard Adafruit library https://github.com/adafruit/Adafruit_BME280_Library , but I failed.

Afterwards I checked basic I2C functions and determined sersor's address (it differs from default, 0x76 instead of 0x77).

Finally I've used this library http://cactus.io/hookups/sensors/barometric/bme280/hookup-arduino-to-bme280-barometric-pressure-sensor

It worked pretty good.

# Installing MQTT

https://www.raspberrypi.org/forums/viewtopic.php?f=29&t=95952

$ wget http://repo.mosquitto.org/debian/mosquitto-repo.gpg.key
$ sudo apt-key add mosquitto-repo.gpg.key
$ cd /etc/apt/sources.list.d/
$ sudo wget http://repo.mosquitto.org/debian/mosquitto-wheezy.list
$ sudo apt-get update
$ sudo apt-get install mosquitto

Don't forget to setup security:
https://www.digitalocean.com/community/tutorials/how-to-install-and-secure-the-mosquitto-mqtt-messaging-broker-on-ubuntu-16-04


