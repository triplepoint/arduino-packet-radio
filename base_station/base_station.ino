// -*- mode: C++ -*-
// The base station, which receives messages and repeats
// them onto the serial bus for relaying to a message
// queue.

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

// Radio frequency, must match the frequency of other radios.
#define RF69_FREQ 915.0

// Identifier on this channel for this radio.
#define MY_ADDRESS     1

// Adafruit Feather 32u4 w/Radio pin defines
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

// A buffer into which we'll write messages
// from sending radios
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];


void setup() {
    Serial.begin(115200);
    while(!Serial) {
        delay(1);
    }

    Serial.println("*INFO: Base station bridge startup");

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    setup_radio();
}

inline void setup_radio() {
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if(! rf69_manager.init()) {
        Serial.println("*WARNING: RFM69 radio init failed");
        while (1);
    }

    Serial.println("*INFO: RFM69 radio init OK");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if(! rf69.setFrequency(RF69_FREQ)) {
        Serial.println("*WARNING: setFrequency failed");
        while (1);
    }

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be shared betweeen all radios
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);

    Serial.print("*INFO: RFM69 radio @");
    Serial.print((unsigned int)RF69_FREQ);
    Serial.println(" MHz");
}

void loop() {
    if(rf69_manager.available()) {
        // The length of the received message
        uint8_t len = sizeof(buf);

        // The identifier of the sending radio
        uint8_t from;

        if(rf69_manager.recvfromAck(buf, &len, &from)) {
            // zero out remaining string
            buf[len] = 0;

            // Print the message payload
            Serial.print("{");
            print_json_val("_sender_id", from);
            Serial.print(",");
            print_json_val("_rssi", rf69.lastRssi());
            Serial.print(",");
            print_json_val("msg", (char*)buf);

            Serial.println("}");

            // Blink LED 3 times, 40ms between blinks
            blink(LED, 40, 3);
        }
    }
}

template<typename T>
void print_json_val(const char* name, T value) {
    Serial.print("\"");
    Serial.print(name);
    Serial.print("\": \"");
    Serial.print(value);
    Serial.print("\"");
}

void blink(byte PIN, byte DELAY_MS, byte loops) {
    for(byte i=0; i<loops; i++)  {
        digitalWrite(PIN, HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN, LOW);
        delay(DELAY_MS);
    }
}
