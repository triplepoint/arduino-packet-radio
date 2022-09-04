// -*- mode: C++ -*-
// Temperature, Pressure, and Humidity sensor
// built around the Bosch BME280 sensor.
//

#include <Arduino.h>
#include <Adafruit_PM25AQI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SleepyDog.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <SensirionI2CScd4x.h>
#include <SPI.h>
#include <Wire.h>


// Are we doing debug printing to Serial?
#define DEBUG_PRINT 1

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

// Sensiron SCD41 device
SensirionI2CScd4x scd4x;

// Plantower PMSA003i sensor
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

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

inline void setup_scd41_sensor() {
    scd4x.begin(Wire);

    // stop potentially previously started measurement
    scd4x.stopPeriodicMeasurement();

    // Read in the serial number
    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    scd4x.getSerialNumber(serial0, serial1, serial2);

    // Start Measurement
    scd4x.startPeriodicMeasurement();
}

inline void setup_pm_sensor() {
    aqi.begin_I2C();
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
inline void send_payload_to_base_station(String &radiopacket) {
    SERIAL_PRINTLN((String("*I: ") + radiopacket).c_str());
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
}

inline void append_bme280_measurement(String &radiopacket) {
    radiopacket += String(" T ");
    radiopacket += bme.readTemperature();  // Deg C
    radiopacket += String(" P ");
    radiopacket += bme.readPressure();     // Pascal
    radiopacket += String(" H ");
    radiopacket += bme.readHumidity();     // % RH
}

inline void append_scd41_measurement(String &radiopacket, uint16_t co2, float temperature, float humidity) {
    // Read Measurement
    if (co2 == 0) {
        SERIAL_PRINTLN("*W: Invalid c02 sample, skipping");
        return;
    }
    radiopacket += String(" CO2 ");
    radiopacket += co2;             // ppm
    radiopacket += String(" T2 ");
    radiopacket += temperature;     // Deg2
    radiopacket += String(" H2 ");
    radiopacket += humidity;        // % RH
}

inline void append_pm_measurement_1_4(String &radiopacket, PM25_AQI_Data &data) {
    // Concentration Units (standard)
    radiopacket += String(" STD_PM1_0 ");    // PM 1.0
    radiopacket += data.pm10_standard;
    radiopacket += String(" STD_PM2_5 ");    // PM 2.5
    radiopacket += data.pm25_standard;
    radiopacket += String(" STD_PM10 ");    // PM 10.0
    radiopacket += data.pm100_standard;
}

inline void append_pm_measurement_2_4(String &radiopacket, PM25_AQI_Data &data) {
    // Concentration Units (Environmental)
    radiopacket += String(" ENV_PM1_0 ");    // PM 1.0
    radiopacket += data.pm10_env;
    radiopacket += String(" ENV_PM2_5 ");    // PM 2.5
    radiopacket += data.pm25_env;
    radiopacket += String(" ENV_PM10 ");    // PM 10.0
    radiopacket += data.pm100_env;
}

inline void append_pm_measurement_3_4(String &radiopacket, PM25_AQI_Data &data) {
    // Particle Density
    radiopacket += String(" PRTCL_0_3UM ");    // Particles > 0.3um / 0.1L air
    radiopacket += data.particles_03um;
    radiopacket += String(" PRTCL_0_5UM ");    // Particles > 0.5um / 0.1L air
    radiopacket += data.particles_05um;
    radiopacket += String(" PRTCL_1_0UM ");    // Particles > 1.0um / 0.1L air
    radiopacket += data.particles_10um;
}

inline void append_pm_measurement_4_4(String &radiopacket, PM25_AQI_Data &data) {
    // Particle Density
    radiopacket += String(" PRTCL_2_5UM ");    // Particles > 2.5um / 0.1L air
    radiopacket += data.particles_25um;
    radiopacket += String(" PRTCL_5_0UM ");    // Particles > 5.0um / 0.1L air
    radiopacket += data.particles_50um;
    radiopacket += String(" PRTCL_10_0UM ");    // Particles > 10um / 0.1L air
    radiopacket += data.particles_100um;
}

// Read the battery voltage pin
inline float read_battery_voltage() {
    float measuredvbat = analogRead(VBATPIN);
    // we divided by 2 in the voltage divider circuit, so multiply back
    // Multiply by 3.3V, our reference voltage
    // Divide by the ADC count per V
    measuredvbat *= 2 * 3.3 / 1024;
    SERIAL_PRINT("*I: VBat (V): ");
    SERIAL_PRINTLN(measuredvbat);
    return measuredvbat;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    SERIAL_PRINTLN("*I: startup");

    Wire.begin();

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    setup_radio();
    setup_bme280_sensor();
    setup_scd41_sensor();
    setup_pm_sensor();
}

void loop() {
    // # Measure the power supply voltage
    float vbat = read_battery_voltage();

    // # Measure the BME280
    bool success_bme = true;
    if (! bme.takeForcedMeasurement()) {
        SERIAL_PRINTLN("*W: BME280 read fail");
        success_bme = false;
    }

    // # Measure the SCD41
    uint16_t co2_error;
    uint16_t scd41_co2;
    float scd41_temperature;
    float scd41_humidity;
    co2_error = scd4x.readMeasurement(scd41_co2, scd41_temperature, scd41_humidity);
    if (co2_error) {
        SERIAL_PRINT("*W: CO2 read fail: ");
        SERIAL_PRINTLN(co2_error)
    } else if (scd41_co2 == 0) {
        SERIAL_PRINTLN("W: CO2 == 0 fail");
    }

    // # Measure the PM sensor
    bool success_aqi = true;
    PM25_AQI_Data pm_data;
    if (! aqi.read(&pm_data)) {
        SERIAL_PRINTLN("*W: PM read fail");
        success_aqi = false;
    }

    String radiopacket = String("");

    // # If the BME280 sensor successfully read, send it's data
    if( success_bme ) {
        radiopacket = String("ST 1");
        radiopacket += String(" VB ");
        radiopacket += vbat;
        append_bme280_measurement(radiopacket);
        send_payload_to_base_station(radiopacket);
    }

    // # If the CO2 sensor successfully read, send it's data
    if (! co2_error && scd41_co2 != 0) {
        radiopacket = String("ST 2");
        append_scd41_measurement(radiopacket, scd41_co2, scd41_temperature, scd41_humidity);
        send_payload_to_base_station(radiopacket);
    }

    // # If the PM sensor successfully read, send out it's multiple data packets
    if( success_aqi ) {
        radiopacket = String("ST 3");
        append_pm_measurement_1_4(radiopacket, pm_data);
        send_payload_to_base_station(radiopacket);

        radiopacket = String("ST 4");
        append_pm_measurement_2_4(radiopacket, pm_data);
        send_payload_to_base_station(radiopacket);

        radiopacket = String("ST 5");
        append_pm_measurement_3_4(radiopacket, pm_data);
        send_payload_to_base_station(radiopacket);

        radiopacket = String("ST 6");
        append_pm_measurement_4_4(radiopacket, pm_data);
        send_payload_to_base_station(radiopacket);
    }

    // # Sleep the radio and wait
    rf69.sleep();
    delay(SAMPLE_PERIOD);
}
