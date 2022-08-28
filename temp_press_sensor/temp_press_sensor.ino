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
// Note that if we don't do warm sleep below with this,
// the serial connection will drop after the first cycle
// and not come back.
#define DEBUG_PRINT 1

// Should we do a regular sleep that spends more power?
// Or should we do a full watchdown timer sleep.
#define DEBUG_WARM_SLEEP 1

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

// A buffer into which we'll write messages
// from sending radios
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

// The transmitter power, calculated from
// the server's response containing it's RSSI.
int8_t power;

// The target RSSI which the radio will adjust its power
// to try to reach.
#define TARGET_RSSI     -60

// How many milliseconds should we sleep between samples?
#define SAMPLE_PERIOD 60000

// define the dBm low and high limit settings, for the radio
// transmitter
#define RADIO_POWER_LIMIT_LOW -2
#define RADIO_POWER_LIMIT_HIGH 20

// Set up the debug print macros
#ifdef DEBUG_PRINT
#define SERIAL_PRINT( ... )       Serial.print( __VA_ARGS__ );
#define SERIAL_PRINTLN( ... )     Serial.println( __VA_ARGS__ );
#else
#define SERIAL_PRINT( ... )
#define SERIAL_PRINTLN( ... )
#endif

inline void setup_radio() {
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if(! rf69_manager.init()) {
        SERIAL_PRINTLN("*W: radio init failed");
        while (1);
    }

    SERIAL_PRINTLN("*I: radio init OK");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if(! rf69.setFrequency(RF69_FREQ)) {
        SERIAL_PRINTLN("*W: setFrequency failed");
        while (1);
    }

    // On setup, initialize the transmitter power to max, to start with
    power = RADIO_POWER_LIMIT_HIGH;

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(power, true);  // range from -2 to 20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be shared betweeen all radios
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);

    SERIAL_PRINT("*I: RFM69 radio @");
    SERIAL_PRINT((unsigned int)RF69_FREQ);
    SERIAL_PRINTLN(" MHz");
}

inline void setup_bme280_sensor() {
    if (! bme.begin()) {
        SERIAL_PRINTLN("BME280 init failed");

        // ID of 0xFF probably means a bad address, a BMP 180 or BMP 085.
        // ID of 0x56-0x58 represents a BMP 280.
        // ID of 0x60 represents a BME 280.
        // ID of 0x61 represents a BME 680.
        SERIAL_PRINT("SensorID: 0x");
        SERIAL_PRINTLN(bme.sensorID(), 16);
        while (1);
    }

    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
}

inline void increase_power() {
    SERIAL_PRINT("*I: Inc power from ");
    SERIAL_PRINT(power);
    SERIAL_PRINT(" dBm ");

    power++;

    // Bound the power within the allowed range
    if(power < RADIO_POWER_LIMIT_LOW ) {
        power = RADIO_POWER_LIMIT_LOW;
    } else if(power > RADIO_POWER_LIMIT_HIGH) {
        power = RADIO_POWER_LIMIT_HIGH;
    }

    SERIAL_PRINT("to ");
    SERIAL_PRINT(power);
    SERIAL_PRINTLN(" dBm");
}

inline void decrease_power() {
    SERIAL_PRINT("*I: Dec power from ");
    SERIAL_PRINT(power);
    SERIAL_PRINT(" dBm ");

    power--;

    // Bound the power within the allowed range
    if(power < RADIO_POWER_LIMIT_LOW ) {
        power = RADIO_POWER_LIMIT_LOW;
    } else if(power > RADIO_POWER_LIMIT_HIGH) {
        power = RADIO_POWER_LIMIT_HIGH;
    }

    SERIAL_PRINT("to ");
    SERIAL_PRINT(power);
    SERIAL_PRINTLN(" dBm");
}

// Blink the given LED pin, at a given toggle period, for a give nnumberof loops
inline void blink(byte PIN, byte DELAY_MS, byte loops) {
    for(byte i=0; i<loops; i++)  {
        digitalWrite(PIN, HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN, LOW);
        delay(DELAY_MS);
    }
}

// Given a string, pass it over the radio to the base station, and then handle any
// radio power adjustment, given the RSSI that the base station responds with.
// Put the radio back to sleep when done.
inline void send_payload_to_base_station(String radiopacket) {
    if(rf69_manager.sendtoWait((uint8_t*)radiopacket.c_str(), radiopacket.length(), DEST_ADDRESS)) {

        // Now wait for the RSSI feedback response from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
            SERIAL_PRINT("*I: Rec. : 0x");
            SERIAL_PRINT(from, HEX);
            SERIAL_PRINT(": ");
            SERIAL_PRINTLN((char*)buf);

            long rssi = String((char*)buf).substring(4).toInt();

            if(rssi > TARGET_RSSI) {
                decrease_power();
            } else if( rssi < TARGET_RSSI) {
                increase_power();
            }

            // Set the transmitter power, for the next transmission
            rf69.setTxPower(power, true);  // range from -2 to 20 for power, 2nd arg must be true for 69HCW

        } else {
            SERIAL_PRINTLN("*W: No RSSI from server");
        }
    } else {
        SERIAL_PRINTLN("*W: Send failed (no ack), increasing power");
        increase_power();
        blink(LED, 40, 5);
    }

    rf69.sleep();
}

// Read the battery voltage pin
inline float read_battery_voltage() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2 in the voltage divider circuit, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert ADC count to voltage
    SERIAL_PRINT("*I: VBat (V): ");
    SERIAL_PRINTLN(measuredvbat);
    return measuredvbat;
}

// Sleep in 8 second chunks, and then sleep for any remainder
inline void hack_sleep(uint16_t delay) {
    while( delay > 8000) {
        Watchdog.sleep(8000);
        delay -= 8000;
    }
    if( delay > 0) {
        Watchdog.sleep(delay);
    }
}

void setup() {
    #ifdef DEBUG_PRINT
    Serial.begin(115200);
    // while(!Serial) {
    //     delay(1);
    // }
    #endif

    SERIAL_PRINTLN("*I: startup");

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    setup_radio();

    setup_bme280_sensor();
}

void loop() {
    float vbat = read_battery_voltage();

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
    SERIAL_PRINTLN((String("*I: ") + radiopacket).c_str());

    send_payload_to_base_station(radiopacket);

    #ifndef DEBUG_WARM_SLEEP
    hack_sleep(SAMPLE_PERIOD);
    # else
    delay(SAMPLE_PERIOD);
    # endif
}
