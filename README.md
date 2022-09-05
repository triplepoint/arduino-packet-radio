## Arduino Packet Radio Environmental Sensor
## Introduction
This is a simple envionmental sensor which reads from multiple sensors over I2C.  The measurements are relayed from the sensor to
a base station over a packet radio link, and the base station emits the payloads it receives over the serial USB port.

A [Python script companion project](https://github.com/triplepoint/ansible-radio-bridge/blob/main/files/serial_bridge.py) listens to the
serial port, formats the payloads as JSON, and relays them to an MQTT broker.  After that, various downstream subscribers listen for the
data updates act on them.

The important hardware involved here are:
- 2x [Adafruit Feather M0 RFM69HCW Packet Radio](https://www.adafruit.com/product/3176?gclid=CjwKCAjwvNaYBhA3EiwACgndgpZLvB8s0Ifjs6VuMn-oV2VZxTpuvf8ojY43N4Rm22mZQmp4r0QAeBoCYLAQAvD_BwE) boards
- [Adafruit BME280 I2C or SPI Temperature Humidity Pressure Sensor](https://www.adafruit.com/product/2652)
- [Adafruit PMSA003I Air Quality Breakout](https://www.adafruit.com/product/4632)
- [Adafruit SCD-41 - True CO2 Temperature and Humidity Sensor](https://www.adafruit.com/product/5190)

## Setup
### Libraries
- https://github.com/adafruit/RadioHead/archive/master.zip - unpacked into /libraries
- https://github.com/Sensirion/arduino-i2c-scd4x/tree/6b9835ea64435409ce169a29cde67cfbc2143711 - The Sensiron SCD4x arduino lib
- SleepyDog
- TODO - fill out the rest of these links and versions
