// -*- mode: C++ -*-
// Temperature, Pressure, and Humidity sensor
// built around the Bosch BME280 sensor.
//

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SleepyDog.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <SPI.h>
#include <Wire.h>

// Are we doing debug printing to Serial?
// #define DEBUG_PRINT 1

// Radio frequency, must match the frequency of other radios.
#define RF69_FREQ 915.0

// Identifier on this channel for this radio.
#define MY_ADDRESS     2

// Identifier of the base station radio.
#define DEST_ADDRESS   1

// Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13
#define VBATPIN       A9

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

// I2C BME280 device
Adafruit_BME280 bme;


// Set up the debug print macros
#ifdef DEBUG_PRINT
#define SERIAL_PRINT( ... )       Serial.print( __VA_ARGS__ );
#define SERIAL_PRINTLN( ... )     Serial.println( __VA_ARGS__ );
#else
#define SERIAL_PRINT( ... )
#define SERIAL_PRINTLN( ... )
#endif


void setup() {
    #ifdef DEBUG_PRINT
    Serial.begin(115200);
    while(!Serial) {
        delay(1);
    }
    #endif

    SERIAL_PRINTLN("*INFO: Sensor startup");

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    setup_radio();

    setup_sensor();
}

inline void setup_radio() {
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if(! rf69_manager.init()) {
        SERIAL_PRINTLN("*WARNING: RFM69 radio init failed");
        while (1);
    }

    SERIAL_PRINTLN("*INFO: RFM69 radio init OK");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if(! rf69.setFrequency(RF69_FREQ)) {
        SERIAL_PRINTLN("*WARNING: setFrequency failed");
        while (1);
    }

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be shared betweeen all radios
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);

    SERIAL_PRINT("*INFO: RFM69 radio @");
    SERIAL_PRINT((unsigned int)RF69_FREQ);
    SERIAL_PRINTLN(" MHz");
}

inline void setup_sensor() {
    if (! bme.begin()) {
        SERIAL_PRINTLN("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        SERIAL_PRINT("SensorID was: 0x");
        SERIAL_PRINTLN(bme.sensorID(), 16);
        SERIAL_PRINT("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        SERIAL_PRINT("   ID of 0x56-0x58 represents a BMP 280,\n");
        SERIAL_PRINT("        ID of 0x60 represents a BME 280.\n");
        SERIAL_PRINT("        ID of 0x61 represents a BME 680.\n");
        while (1);
    }

    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
}

void loop() {
    // Read from the battery voltage
    float vbat = read_battery_voltage();
    SERIAL_PRINT("*INFO: VBat (V): ");
    SERIAL_PRINTLN(vbat);

    // Read from sensor
    bme.takeForcedMeasurement();

    // Assemble the sensor and battery data into a payload string
    String radiopacket = String("VB ");
    radiopacket += vbat;
    radiopacket += String(" T ");
    radiopacket += bme.readTemperature();  // Deg C
    radiopacket += String(" P ");
    radiopacket += bme.readPressure();     // Pascal
    radiopacket += String(" H ");
    radiopacket += bme.readHumidity();     // % RH
    SERIAL_PRINTLN((String("*INFO: ") + radiopacket).c_str());

    // Send the payload
    if(! rf69_manager.sendtoWait((uint8_t*)radiopacket.c_str(), radiopacket.length(), DEST_ADDRESS)) {
        SERIAL_PRINTLN("*WARNING: Sending failed (no ack)");
        blink(LED, 40, 5);
    }

    // Put the radio to sleep, and then sleep the processor until
    // it's time for the next read.
    #if 1
    rf69.sleep();
    hack_sleep(60000);
    # else
    delay(60000);
    # endif
}

float read_battery_voltage() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2 in the voltage divider circuit, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert ADC count to voltage
    return measuredvbat;
}

void blink(byte PIN, byte DELAY_MS, byte loops) {
    for(byte i=0; i<loops; i++)  {
        digitalWrite(PIN, HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN, LOW);
        delay(DELAY_MS);
    }
}

void hack_sleep(uint16_t delay) {
    while( delay > 8000) {
        Watchdog.sleep(8000);
        delay -= 8000;
    }
    if( delay > 0) {
        Watchdog.sleep(delay);
    }
}
