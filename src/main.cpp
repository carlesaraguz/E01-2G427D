#include <Arduino.h>
#include <printf.h>
#include <SPI.h>
#include <RF24.h>
// #include "test_data.h"

typedef enum tx_state_t {
    TX_SENDING_HANDSHAKE,
    TX_WAIT_HANDSHAKE_ACK,
    TX_SENDING_DATA,
    TX_DATA_COMPLETE,
    TX_ERROR
} tx_state_t;
typedef enum rx_state_t {
    RX_WAITING_HANDSHAKE,
    RX_HANDSHAKE_ACK,
    RX_RECEIVING_DATA,
    RX_DATA_COMPLETE,
    RX_ERROR
} rx_state_t;
typedef struct connection_t {
    uint8_t pkt_id;                 // ID of the handshake packet.
    uint8_t frame_size;             // Size of frames in bytes (màx 32).
    uint8_t pkt_count;              // Total number of data packets to send.
    uint32_t stream_repeat;         // Iterations.
    rf24_crclength_e hw_crc;        // Config for the hardware CRC implemented in the nRF24L01.
    bool sw_crc_enabled;            // True when frames's CRC should be checked.
    bool auto_ack_enabled;          // Whether to enable automatic ACK's.
    uint8_t padding;                // Padding applied to control packets.
} connection_t;

uint16_t computeCRC16(const uint8_t *data, size_t length);
void makePayload(uint8_t * buf, const uint8_t len, const uint8_t pkt_id, const uint8_t pkt_data, bool zero_padding = false);
void makeRandomPacket(uint8_t * buf, const uint8_t len, const uint8_t pkt_id, const uint8_t randMin, const uint8_t randMax);
void makeHandshake(uint8_t * buf, const uint8_t len, const connection_t * conn);
bool decodeHandshake(const uint8_t * buf, const uint8_t len, connection_t * conn);
void resetRadio(bool power_reset = true);
void printPacket(const uint8_t * buf, const uint8_t len);

/* Set which endpoint are you loading to code to: (automatically set by Platform IO env.) */
// #define ESP32_ENDPOINT
// #define ARDUNANO_ENDPOINT

/* Config params: */
#define NRF24_CHANNEL   0x4C    // 76
#define NRF24_BUFSIZE   32      // Max. is 32 bytes
#define CRC16_POLY      0x1021  // Polynomial for CRC-16-CCITT
#define CRC16_INIT      0xFFFF  // Initial CRC value
#define HANDSHAKE_CMD   0xAA    // Handshake command
#define HSACK_CMD       0x55    // Handshake ACK command
#define HANDSHAKE_MIN_LEN 13    // Minimum length for the CRC packet.
/* Timeout times (in miliseconds): */
#define TIMEOUT_RX_SINGLE           100         // Adaptive timeout base.
#define TIMEOUT_HANDSHAKE_RESPONSE  2000ULL     // Handshake ACK reception.
#define TIMEOUT_RX_START            5000ULL     // First packet of data.
#define TIMEOUT_RX                  1000ULL     // On-going data reception.
#define TIMEOUT_RX_REPEAT           10          // Repeats wait on TIMEOUT_RX_TIME.
/* Stream configuration: */
#define STREAM_PKT_COUNT    256LL   // Packets to send from test data (min 1, max 256).
#define STREAM_REPEAT       32LL    // Copies of the test data that will be transferred.
/* Endpoint pinout settings */
#ifdef ESP32_ENDPOINT
#define LED     D3          // LED indicator
#define CE_PIN  D0          // CE pin for NRF24L01 (LNA enable)
#define CSN_PIN D2          // CSN pin for NRF24L01 (SPI chip select)
#define IRQ_PIN D1          // IRQ pin for NRF24L01
#elif defined(ARDUNANO_ENDPOINT)
#define LED     9           // LED indicator
#define CE_PIN  A0          // CE pin for NRF24L01 (LNA enable)
#define CSN_PIN 10          // CSN pin for NRF24L01 (SPI chip select)
#define IRQ_PIN 2           // IRQ pin for NRF24L01
#endif

/* Global control variables: */
RF24 radio(CE_PIN, CSN_PIN);
uint8_t rf_buf[NRF24_BUFSIZE + 1];  // +1 for null-terminator
uint8_t addr[][6] = {
    "00001",   // Endpoint ESP32
    "00002",   // Endpoint Arduino Nano
};
tx_state_t tx_state = TX_SENDING_HANDSHAKE;
rx_state_t rx_state = RX_WAITING_HANDSHAKE;
uint32_t timestamp = 0;
connection_t rx_connection;
connection_t tx_connection;
rf24_pa_dbm_e pa_setting = RF24_PA_MIN;
rf24_datarate_e dr_setting = RF24_250KBPS;
uint8_t channel = NRF24_CHANNEL;
uint32_t rx_pkt_count_expected = 0;
uint32_t rx_data_timeout = 0;
bool is_tx_endpoint = false;
bool is_rx_endpoint = false;
uint8_t addr_selector_txpipe = 0;
uint8_t addr_selector_rxpipe = 1;
uint16_t delay_repeat = 10;

int queryNumber(void) {
    int retval = -1;
    while(true) {
        if(Serial.available()) {
            String str = Serial.readStringUntil('\n');
            str.trim();
            if(str[0] >= '0' && str[0] <= '9') {
                retval = atoi(str.c_str());
                Serial.println("");
            } else {
                Serial.println("");
                Serial.println(F("=== Default option selected."));
            }
            break;
        }
    }
    return retval;
}
void queryEndpoint(void) {
    Serial.println(F("=== Endpoint settings:"));
    Serial.println(F("=== 0  : RX (default)."));
    Serial.println(F("=== 1  : TX."));
    Serial.print(F(">>> Enter endpoint setting: "));
    int q = queryNumber();
    if(q >= 1) {
        is_tx_endpoint = true;
        is_rx_endpoint = false;
        addr_selector_txpipe = 1;
        addr_selector_rxpipe = 0;
        Serial.println(F("=== Endpoint is TX"));
    } else {
        is_tx_endpoint = false;
        is_rx_endpoint = true;
        addr_selector_txpipe = 0;
        addr_selector_rxpipe = 1;
        Serial.println(F("=== Endpoint is RX"));
    }
    if(is_tx_endpoint) {
        Serial.println(F("=== Stream repeat delay:"));
        Serial.println(F("===  0 : No delay performed, user is requested to confirm."));
        Serial.println(F("=== 10 : Ten seconds of felay, automatic repeat (default)."));
        Serial.println(F("===    : Enter a different positive integer to configure the delay in seconds."));
        Serial.print(F(">>> Enter repeat delay: "));
        int q = queryNumber();
        if(q > 0) {
            delay_repeat = q;
            Serial.print(F("=== Automatic repetition after "));
            Serial.print(delay_repeat);
            Serial.println(F(" seconds."));
        } else if(q <= 0) {
            delay_repeat = 0;
            Serial.println(F("=== Endpoint will be locked and request user to confirm."));
        }
    }
}
void queryChannel(void) {
    Serial.println(F("=== Channel limits: 2.4 GHz -- 2.525 GHz"));
    Serial.print(F("=== Channel default: "));
    Serial.print((2400 + NRF24_CHANNEL) / 1000.0, 3);
    Serial.println(F(" GHz"));
    Serial.println(F("=== Channel settings: 2.4 GHz + (<channel> MHz) - Min: 0, Max: 125"));
    Serial.print(F(">>> Enter channel: "));
    uint8_t ch_usr = NRF24_CHANNEL;
    int query = queryNumber();
    if(query >= 0) {
        ch_usr = query;
    }
    Serial.print(F("=== Channel set to: "));
    Serial.print((2400 + ch_usr) / 1000.0, 3);
    Serial.println(F(" GHz"));
    radio.setChannel(ch_usr);
}

void queryRadioConfig(void) {
    Serial.println(F("=== nRF24L01+ power output settings:"));
    Serial.println(F("=== 0 : RF24_PA_MIN  : -18 dBm (default)"));
    Serial.println(F("=== 1 : RF24_PA_LOW  : -12 dBm"));
    Serial.println(F("=== 2 : RF24_PA_HIGH :  -6 dBm"));
    Serial.println(F("=== 3 : RF24_PA_MAX  :   0 dBm"));
    Serial.println(F("=== External PA from E01-2G4M27D may still always be +20 ~ +27 dBm."));
    Serial.print(F(">>> Enter PA setting: "));
    switch(queryNumber()) {
    case 1: 
        pa_setting = RF24_PA_LOW; 
        Serial.println(F("=== PA level set to -12 dBm."));
        break;
    case 2: 
        pa_setting = RF24_PA_HIGH; 
        Serial.println(F("=== PA level set to -6 dBm."));
        break;
    case 3: 
        pa_setting = RF24_PA_MAX; 
        Serial.println(F("=== PA level set to 0 dBm."));
        break;
    default: 
        pa_setting = RF24_PA_MIN; 
        Serial.println(F("=== PA level set to -18 dBm."));
        break;
    }

    Serial.println(F("=== nRF24L01+ datarate settings:"));
    Serial.println(F("=== 0 : RF24_250KPS : 250 kbps (default)"));
    Serial.println(F("=== 1 : RF24_1MBPS  : 1 Mbps"));
    Serial.println(F("=== 2 : RF24_2MBPS  : 2 Mbps"));
    Serial.print(F(">>> Enter datarate setting: "));
    switch(queryNumber()) {
    case 1: 
        dr_setting = RF24_1MBPS; 
        Serial.println(F("=== Datarate set to 1 Mbps."));
        break;
    case 2: 
        dr_setting = RF24_2MBPS; 
        Serial.println(F("=== Datarate set to 2 Mbps."));
        break;
    default: 
        dr_setting = RF24_250KBPS; 
        Serial.println(F("=== Datarate set to 250 kbps."));
        break;
    }

    Serial.println(F("=== nRF24L01+ CRC settings:"));
    Serial.println(F("=== 0  : Hardware CRC checksum disabled."));
    Serial.println(F("=== 8  : 8-bit hardware CRC checksum."));
    Serial.println(F("=== 16 : 16-bit hardware CRC checksum (default)."));
    Serial.print(F(">>> Enter CRC setting: "));
    switch(queryNumber()) {
    case 0: 
        tx_connection.hw_crc = RF24_CRC_DISABLED;
        Serial.println(F("=== Hardware CRC disabled."));
        break;
    case 8: 
        tx_connection.hw_crc = RF24_CRC_8;
        Serial.println(F("=== 8-bit CRC checksum."));
        break;
    default: 
        tx_connection.hw_crc = RF24_CRC_16;
        Serial.println(F("=== 16-bit CRC checksum."));
        break;
    }

    Serial.print(F(">>> Enable packet-level CRC? (0:no, *1:yes): "));
    switch(queryNumber()) {
    case 0: 
        tx_connection.sw_crc_enabled = false;
        Serial.println(F("=== Packet-level CRC disabled."));
        break;
    default: 
        tx_connection.sw_crc_enabled = true;
        Serial.println(F("=== Packet-level CRC enabled."));
        break;
    }

    Serial.print(F(">>> Enable auto ACK? (*0:no, 1:yes): "));
    switch(queryNumber()) {
    case 1: 
        tx_connection.auto_ack_enabled = true;
        Serial.println(F("=== Auto ACK enabled."));
        break;
    default: 
        tx_connection.auto_ack_enabled = false;
        Serial.println(F("=== Auto ACK disabled."));
        break;
    }

    radio.setPALevel(pa_setting);
    radio.setDataRate(dr_setting);
    radio.setCRCLength(tx_connection.hw_crc);
    radio.setPayloadSize(NRF24_BUFSIZE);
    radio.setAutoAck(tx_connection.auto_ack_enabled);
}

void queryStreamConfig(void) {
    tx_connection.frame_size = NRF24_BUFSIZE;
    Serial.println(F("=== Stream frames/packets set to 32 bytes."));
    Serial.print(F(">>> Enter packet count setting (min:1, max/default:256): "));
    int query = queryNumber();
    if(query > 0 && query < 256) {
        tx_connection.pkt_count = query - 1;
    } else {
        tx_connection.pkt_count = 255;
    }
    Serial.print(F("=== Packet count set to "));
    Serial.println(tx_connection.pkt_count);
    Serial.print(F(">>> Enter stream repeat setting (min:1, default:32): "));
    query = queryNumber();
    if(query > 0) {
        tx_connection.stream_repeat = query;
    } else {
        tx_connection.stream_repeat = STREAM_REPEAT;
    }
    Serial.print(F("=== Stream repeat set to "));
    Serial.println(tx_connection.stream_repeat);
}

void setup() {
    Serial.begin(115200);
    printf_begin();
    randomSeed(analogRead(A0));
    delay(1000);
    
    pinMode(LED, OUTPUT);

    if(!radio.begin()) {
        while(true) {
            Serial.println(F("=== NRF24L01 initialization failed."));
            delay(1000);
        }
    }
    Serial.println(F("=== NRF24L01 initialization OK."));
    if(radio.isPVariant()) {
        Serial.println(F("=== NRF24L01+ detected."));
    } else {
        Serial.println(F("=== NRF24L01 detected."));
    }
    if(radio.isChipConnected()) {
        Serial.println(F("=== NRF24L01 chip connected."));
    } else {
        Serial.println(F("=== NRF24L01 chip NOT connected."));
    }

    queryEndpoint();
    queryChannel();
    queryRadioConfig();
    
    radio.openWritingPipe(addr[addr_selector_txpipe]);
    radio.openReadingPipe(1, addr[addr_selector_rxpipe]);
    
    if(is_rx_endpoint) {
        radio.startListening(); // Put radio in RX mode.
    } else {
        queryStreamConfig();
        radio.stopListening();  // Put radio in TX mode.
    }

#if 0
for(int i = 0; i < 256; i++) {
    uint8_t random_data[NRF24_BUFSIZE - 3];
    for(int j = 0; j < NRF24_BUFSIZE - 3; j++) {
        random_data[j] = random(0, 256);
    }
    uint16_t crc = computeCRC16(random_data, NRF24_BUFSIZE - 3);
    uint8_t crc_lsb = crc & 0xFF;
    uint8_t crc_msb = (crc >> 8) & 0xFF;
    // Print the array:
    Serial.print(F("{ "));
    for(int j = 0; j < NRF24_BUFSIZE - 3; j++) {
        uint8_t val = random_data[j];
        Serial.print((val < 0x10) ? "0x0" : "0x");
        Serial.print(random_data[j], HEX);
        Serial.print(F(", "));
    }
    Serial.print(F(" /* CRC-16 */ "));
    Serial.print(crc_msb < 0x10 ? "0x0" : "0x");
    Serial.print(crc_msb, HEX);
    Serial.print(F(", "));
    Serial.print(crc_lsb < 0x10 ? "0x0" : "0x");
    Serial.print(crc_lsb, HEX);
    Serial.println(" },  // " + String(i));
}
#endif
}

void loop() {
    if(is_tx_endpoint && !is_rx_endpoint) {
        switch(tx_state) {

        case TX_SENDING_HANDSHAKE:
            // Send handshake to the RX endpoint:
            Serial.println(F("=== Sending handshake to RX endpoint..."));
            digitalWrite(LED, HIGH);
            radio.flush_tx();

            uint8_t handshake[NRF24_BUFSIZE];
            tx_connection.frame_size = NRF24_BUFSIZE;
            tx_connection.pkt_id = 0;
            tx_connection.padding = HANDSHAKE_CMD;
            makeHandshake(handshake, NRF24_BUFSIZE, &tx_connection); // Creates the handshake packet

            if(!radio.write(handshake, NRF24_BUFSIZE)) {
                digitalWrite(LED, LOW);
                if(radio.failureDetected) {
                    Serial.println(F("=== TX ERROR: sending handshake failed. Retrying..."));
                } else {
                    Serial.println(F("=== TX ERROR: unknown error sending handshake. Retrying..."));
                    tx_state = TX_ERROR;
                }
                delay(random(1000, 2000));
            } else {
                digitalWrite(LED, LOW);
                Serial.println(F("=== Handshake sent."));
                tx_state = TX_WAIT_HANDSHAKE_ACK;
                radio.startListening();
                timestamp = millis();
            }
            break;

        case TX_WAIT_HANDSHAKE_ACK:
            // Waiting for handshake ACK from the RX endpoint:
            if(radio.available()) {
                radio.read(rf_buf, NRF24_BUFSIZE);
                if(rf_buf[0] == 0x00 && rf_buf[1] == HSACK_CMD) {
                    Serial.println(F("=== Handshake ACK received. Starting data transmission..."));
                    tx_state = TX_SENDING_DATA;
                    radio.stopListening();
                } else {
                    Serial.println(F("=== TX ERROR: invalid handshake ACK."));
                    tx_state = TX_SENDING_HANDSHAKE;
                    radio.stopListening();
                }
            } else if(millis() - timestamp > TIMEOUT_HANDSHAKE_RESPONSE) {
                // Handshake response timed out. Returning to initial state.
                Serial.print(F("=== Handshake ACK response timed out after "));
                Serial.print((double)TIMEOUT_HANDSHAKE_RESPONSE / 1000.0, 1);
                Serial.println(F(" seconds."));
                tx_state = TX_SENDING_HANDSHAKE;
                radio.stopListening();
            }
            break;

        case TX_SENDING_DATA:
            {
                // Send data to the RX endpoint:
                radio.flush_tx();
                delay(1000);
                Serial.println(F("=== Sending data..."));
                timestamp = micros();
                uint32_t pkt_error = 0;
                uint32_t pkt_sent = 0;
                uint16_t pkt_count = tx_connection.pkt_count;
                int8_t progress_helper = -1;
                const uint32_t stream_size = pkt_count * tx_connection.stream_repeat;
                for(size_t stream_id = 0; stream_id < tx_connection.stream_repeat; stream_id++) {
                    for(size_t pkt_id = 0; pkt_id < pkt_count; pkt_id++) {
                        digitalWrite(LED, HIGH);
                        // rf_buf[0] = pkt_id;
                        // memcpy(rf_buf + 1, data[pkt_id], NRF24_BUFSIZE - 1);
                        makeRandomPacket(rf_buf, NRF24_BUFSIZE, pkt_id, 65, 90);
                        // printPacket(rf_buf, NRF24_BUFSIZE + 1);
                        if(!radio.writeFast(rf_buf, NRF24_BUFSIZE)) {
                            digitalWrite(LED, LOW);
                            Serial.print(F("=== TX ERROR: sending data failed (PKT ID: "));
                            Serial.print(pkt_id);
                            Serial.println(F(")."));
                            pkt_error++;
                        } else {
                            digitalWrite(LED, LOW);
                            pkt_sent++;

                            if(pkt_sent % 250 == 0) {
                                int8_t progress = 100 * pkt_sent / stream_size;
                                Serial.print(F("=== Sending packets... "));
                                Serial.print(progress);
                                Serial.print(F("% ("));
                                Serial.print(pkt_sent);
                                Serial.print(F(" of "));
                                Serial.print(stream_size);
                                Serial.println(F(")"));
                                progress_helper = progress;
                            }
                        }
                    }
                }
                Serial.println(F("=== Sending packets... 100%. Completed."));
                uint32_t t = micros() - timestamp;
                Serial.print(F("===  - Time elapsed: "));
                Serial.print((double)t / 1000.0);
                Serial.println(F(" ms"));
                Serial.print(F("===  - Packets (stream size): "));
                Serial.println(stream_size);
                Serial.print(F("===  - Packets (failed to send): "));
                Serial.println(pkt_error);
                Serial.print(F("===  - Packets (sent): "));
                Serial.println(pkt_sent);
                Serial.print(F("===  - Size: "));
                Serial.print(((uint32_t)pkt_sent * NRF24_BUFSIZE) / 1024);
                Serial.println(F(" KiB"));
                Serial.print(F("===  - Datarate (actual): "));
                Serial.print(((double)pkt_sent * NRF24_BUFSIZE * 8.0) / (double)t);
                Serial.println(F(" Mbps"));
                tx_state = TX_DATA_COMPLETE;
            }
            break;

        case TX_DATA_COMPLETE: 
            {
                Serial.println(F("=== Restarting operations."));
                resetRadio();
                radio.stopListening();  // Put radio in TX mode.

                if(delay_repeat > 0) {
                    Serial.print(F("=== Waiting for "));
                    Serial.print(delay_repeat);
                    Serial.println(F(" seconds..."));
                    delay(delay_repeat * 1000);
                } else {
                    Serial.print(F(">>> Repeat transfer? (yes/default:1, no:0) : "));
                    int query = queryNumber();
                    if(query == 0) {
                        Serial.print(F(">>> Change radio config.? (yes:1, no/default:0) : "));
                        query = queryNumber();
                        if(query == 1) {
                            queryEndpoint();
                            queryRadioConfig();
                            if(is_tx_endpoint) {
                                queryStreamConfig();
                            }
                        }
                    }
                }
                tx_state = TX_SENDING_HANDSHAKE;
            }
            break;

        case TX_ERROR:
            // Error handling.
            Serial.println(F("=== UNEXPECTED ERROR detected. Printing radio config:"));
            radio.printPrettyDetails();
            /* Current version doesn't change anything here for now, but it could be a nice feature to 
               reset the radio (radio.powerDown + radioPowerUp) and to re-initialise the library with
               radio.begin() + all the preconfigured options.
            */
            resetRadio();
            radio.stopListening();
            tx_state = TX_SENDING_HANDSHAKE;
            break;
        }
    } else if(!is_tx_endpoint && is_rx_endpoint) {
        switch(rx_state) {
        case RX_WAITING_HANDSHAKE:
            // Waiting for handshake CMD from the TX endpoint:
            // Serial.println(F("=== Waiting handshake command from TX endpoint..."));
            if(radio.available()) {
                radio.read(rf_buf, NRF24_BUFSIZE);
                if(decodeHandshake(rf_buf, NRF24_BUFSIZE, &rx_connection)) {
                    Serial.println(F("=== Handshake received and decoded:"));
                    Serial.print(F("===  - Expecting "));
                    Serial.print(rx_connection.stream_repeat);
                    Serial.print(F(" streams of "));
                    Serial.print(rx_connection.pkt_count);
                    Serial.print(F(" packets ("));
                    Serial.print(rx_connection.frame_size);
                    Serial.println(F(" bytes each)."));

                    rx_pkt_count_expected = rx_connection.pkt_count * rx_connection.stream_repeat;
                    rx_data_timeout = rx_pkt_count_expected * TIMEOUT_RX_SINGLE;
                    Serial.print(F("===  - Total packets: "));
                    Serial.println(rx_pkt_count_expected);
                    Serial.print(F("===  - RX timeout after: "));
                    Serial.print((double)rx_data_timeout / 1000.0, 1);
                    Serial.println(F(" seconds."));
                    Serial.print(F("===  - RF24 CRC:  "));
                    switch (rx_connection.hw_crc) {
                        case RF24_CRC_16: Serial.println(F("16-bit.")); break;
                        case RF24_CRC_8: Serial.println(F("8-bit.")); break;
                        case RF24_CRC_DISABLED: Serial.println(F(" disabled.")); break;
                    }
                    Serial.print(F("===  - Frame CRC: "));
                    Serial.println(rx_connection.sw_crc_enabled ? "enabled." : "disabled.");

                    // Transition into the next state:
                    rx_state = RX_HANDSHAKE_ACK;
                } else {
                    Serial.print(F("=== RX ERROR: Packet received. Invalid Handshake CMD [0x"));
                    Serial.print(rf_buf[0], HEX);
                    Serial.println(F("]"));
                }
            } else if(millis() - timestamp > 5000) {
                Serial.println(F("=== Waiting for handshake..."));
                timestamp = millis();
                // radio.printPrettyDetails();
            }
            break;

        case RX_HANDSHAKE_ACK:
            // Sending the Handshake ACK packet
            radio.stopListening(); // Sets TX mode
            radio.flush_tx();
            uint8_t handshake_ack[NRF24_BUFSIZE];
            makePayload(handshake_ack, NRF24_BUFSIZE, 0, HSACK_CMD);    // Creates the HS ACK packet
            if(!radio.write(handshake_ack, NRF24_BUFSIZE)) {
                if(radio.failureDetected) {
                    Serial.println(F("=== RX ERROR: sending handshake ACK failed. Retrying..."));
                } else {
                    Serial.println(F("=== RX ERROR: unknown error sending handshake ACK. Retrying..."));
                    rx_state = RX_ERROR;
                }
                delay(random(1000, 2000));
            } else {
                radio.startListening(); // Sets RX mode soon after sending ACK.
                Serial.println(F("=== Handshake ACK sent."));
                rx_state = RX_RECEIVING_DATA;
            }
            break;
            
            case RX_RECEIVING_DATA:
            {
                uint32_t pkt_rcv = 0;
                uint32_t pkt_error = 0;
                uint16_t pkt_count = 0;
                bool rx_started = false;
                uint8_t prev_pkt_id = 0;
                timestamp = micros();
                uint32_t rx_transfer_time = micros(); // microseconds

                Serial.print(F("=== Expecting a stream of "));
                Serial.print(rx_pkt_count_expected);
                Serial.print(F(" packets of "));
                Serial.print(rx_connection.frame_size);
                Serial.println(F(" bytes each."));
                
                while(!rx_started) {
                    if(radio.available()) {
                        radio.read(rf_buf, NRF24_BUFSIZE);
                        uint16_t crc_rx = computeCRC16(rf_buf, NRF24_BUFSIZE - 2);
                        uint8_t crc_msb = rf_buf[30];
                        uint8_t crc_lsb = rf_buf[31];
                        uint16_t crc_pkt = (crc_msb << 8) + crc_lsb;
                        if(crc_pkt == crc_rx && rf_buf[0] != 0) {
                            // First successful reception of a packet.
                            pkt_rcv++;
                            pkt_count++;
                            prev_pkt_id = rf_buf[0];
                            Serial.print(F("=== Data transfer stream started with Packet ID: "));
                            Serial.println(prev_pkt_id);
                        } else if(crc_pkt != crc_rx) {
                            pkt_error++;
                            pkt_count++;
                            Serial.println(F("=== RX ERROR: Data transfer stream started with wrong CRC."));
                        } else {
                            // rf_buf == 0:
                            Serial.println(F("=== Data transfer not started. Control packet received (skipping)."));
                            Serial.println(F("=== RX ERROR: data transfer start timed out."));
                            rx_state = RX_WAITING_HANDSHAKE;
                            break;
                        }
                        rx_started = true;
                        timestamp = micros();
                    } else if(micros() - timestamp > TIMEOUT_RX_START * 1000) {
                        Serial.println(F("=== RX ERROR: data transfer start timed out."));
                        rx_state = RX_WAITING_HANDSHAKE;
                        break;
                    }
                }
                bool no_timeout = true;
                uint8_t timeout_counter = 0;
                bool pkt_count_trigger_print = false;
                while(rx_started && no_timeout) {
                    // Receiving a stream of packets:
                    if(pkt_rcv >= rx_pkt_count_expected) break;
                    if(radio.available()) {
                        radio.read(rf_buf, NRF24_BUFSIZE);
                        uint16_t crc_rx = computeCRC16(rf_buf, NRF24_BUFSIZE - 2);
                        uint16_t crc_pkt = (rf_buf[30] << 8) + rf_buf[31];
                        if(crc_pkt == crc_rx) {
                            pkt_rcv++;
                        } else {
                            pkt_error++;
                        }
                        pkt_count++;
                        pkt_count_trigger_print = (pkt_count % 250 == 0);
                        uint32_t remaining = rx_pkt_count_expected - pkt_count;
                        rx_data_timeout = remaining * TIMEOUT_RX_SINGLE;
                        timestamp = micros();
                        timeout_counter = 0;
                    }
                    if((micros() - timestamp) > rx_data_timeout) {
                        timeout_counter++;
                        timestamp = micros();
                        radio.startListening();
                    }
                    no_timeout = (timeout_counter < TIMEOUT_RX_REPEAT);
                    uint8_t progress = 100.0 * (double)pkt_count / (double)rx_pkt_count_expected; // Estimated.
                    if(pkt_count_trigger_print) {
                        digitalWrite(LED, HIGH);
                        double mbps = ((double)pkt_rcv * rx_connection.frame_size * 8.0) / (double)(micros() - rx_transfer_time);
                        char str[150];
                        sprintf(str, 
                            "=== Receiving packets... %3u%% (%5u of %5lu | ", 
                            progress, 
                            pkt_count, 
                            rx_pkt_count_expected
                        );
                        Serial.print(str);
                        Serial.print(mbps);
                        Serial.println(F(" Mbps)"));
                        pkt_count_trigger_print = false;
                        digitalWrite(LED, LOW);
                    }
                }
                if(no_timeout) {
                    rx_transfer_time = micros() - rx_transfer_time;
                    Serial.println(F("=== Transfer finished. Generating report:"));
                } else {
                    rx_transfer_time = micros() - rx_transfer_time - (rx_data_timeout * 1000);
                    Serial.println(F("=== Transfer timedout. Generating report:"));
                }
                Serial.print(F("=== Total transfer time: "));
                Serial.print((double)rx_transfer_time / 1000.0, 2);
                Serial.println(F(" ms."));
                Serial.print(F("===  - Packets (stream size): "));
                Serial.println(rx_pkt_count_expected);
                Serial.print(F("===  - Packets (CRC error): "));
                Serial.println(pkt_error);
                Serial.print(F("===  - Packets (received):  "));
                Serial.println(pkt_rcv);
                Serial.print(F("===  - Packets (lost):      "));
                Serial.println(rx_pkt_count_expected - pkt_rcv);
                Serial.print(F("===  - Packet Error Rate (software): "));
                Serial.print(100.0 * (pkt_error) / (double)rx_pkt_count_expected, 2);
                Serial.println(F("%"));
                Serial.print(F("===  - Packet Error Rate (total):    "));
                Serial.print(100.0 * (double)(rx_pkt_count_expected - pkt_rcv) / (double)rx_pkt_count_expected, 2);
                Serial.println(F("%"));
                Serial.print(F("===  - Size: "));
                Serial.print(((uint32_t)pkt_rcv * NRF24_BUFSIZE) / 1024);
                Serial.println(F(" KiB"));
                Serial.print(F("===  - Datarate (actual): "));
                Serial.print(((double)pkt_rcv * NRF24_BUFSIZE * 8.0) / (double)rx_transfer_time);
                Serial.println(F(" Mbps"));

                if(pkt_count == 0 || pkt_error >= rx_pkt_count_expected * 0.1) {
                    Serial.println(F("=== UNEXPECTED stream results."));
                    if(radio.failureDetected) {
                        Serial.println(F("=== Radio failure detected."));
                    }
                    radio.printPrettyDetails();
                }
                rx_state = RX_DATA_COMPLETE;
            }
            break;

        case RX_DATA_COMPLETE:
            // Resets the radio and starts over:
            resetRadio();
            radio.startListening();
            rx_state = RX_WAITING_HANDSHAKE;
            break;

        case RX_ERROR:
            // Error handling.
            Serial.println(F("=== UNEXPECTED ERROR detected."));
            radio.printPrettyDetails();
            /* Current version doesn't change anything here for now, but it could be a nice feature to 
                reset the radio (radio.powerDown + radioPowerUp) and to re-initialise the library with
                radio.begin() + all the preconfigured options.
            */
            resetRadio();
            radio.startListening();
            rx_state = RX_WAITING_HANDSHAKE;
            break;
        }

    } else {
        Serial.println(F("=== Misconfiguration error RX/TX endpoint."));
    }
}

uint16_t computeCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = CRC16_INIT;

    for(size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x8000U) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void makePayload(uint8_t * buf, const uint8_t len, const uint8_t pkt_id, const uint8_t pkt_data, bool zero_padding) {
    if(len < 4) {
        return;
    }
    for(uint8_t i = 2; i < len - 2; i++) {
        buf[i] = zero_padding ? 0x00 : pkt_data;
    }
    buf[0] = pkt_id;
    buf[1] = pkt_data;
    buf[len - 1] = 0x00; // No CRC
    buf[len - 2] = 0x00; // No CRC
}

void makeRandomPacket(uint8_t * buf, const uint8_t len, const uint8_t pkt_id, const uint8_t randMin, const uint8_t randMax) {
    if(len < 4) {
        return;
    }
    buf[0] = pkt_id;
    for(uint8_t i = 1; i < len - 2; i++) {
        buf[i] = random(randMin, randMax);
    }
    uint16_t crc_word = computeCRC16(buf, len - 2);
    buf[len - 2] = (crc_word >> 8) & 0xFF;  // 32 - 2 = 30
    buf[len - 1] = (crc_word >> 0) & 0xFF;  // 32 - 1 = 31
}

void makeHandshake(uint8_t * buf, const uint8_t len, const connection_t * conn) {
    if(len < HANDSHAKE_MIN_LEN) return;

    buf[0] = conn->pkt_id;
    buf[1] = HANDSHAKE_CMD;
    buf[2] = conn->pkt_count;
    buf[3] = conn->frame_size;
    buf[4] = (conn->stream_repeat >> 24) & 0xFF; // MSB
    buf[5] = (conn->stream_repeat >> 16) & 0xFF;
    buf[6] = (conn->stream_repeat >>  8) & 0xFF;
    buf[7] = (conn->stream_repeat >>  0) & 0xFF; // LSB
    if(conn->hw_crc == RF24_CRC_16) {
        buf[8] = 16;
    } else if(conn->hw_crc == RF24_CRC_8) {
        buf[8] = 8;
    } else {
        buf[8] = 0;
    }
    buf[9] = conn->sw_crc_enabled;
    buf[10] = conn->auto_ack_enabled;
    // Padding
    if(len > HANDSHAKE_MIN_LEN) {
        for(uint8_t b = 11; b < len - 2; ++b) {
            buf[b] = conn->padding;
        }
    }
    uint16_t crc_hs = computeCRC16(buf, len - 2);
    buf[len - 2] = (crc_hs >> 8) & 0xFF;
    buf[len - 1] = (crc_hs >> 0) & 0xFF;
}

bool decodeHandshake(const uint8_t * buf, const uint8_t len, connection_t * conn) {
    if(len < HANDSHAKE_MIN_LEN) return false;
    conn->pkt_id = buf[0];
    if(conn->pkt_id != 0 || buf[1] != HANDSHAKE_CMD) return false;

    uint16_t crc = computeCRC16(buf, len - 2);
    uint8_t crc_msb = (crc >> 8) & 0xFF;
    uint8_t crc_lsb = (crc >> 0) & 0xFF;

    if(buf[len - 2] == crc_msb && buf[len - 1] == crc_lsb) {
        conn->pkt_count = buf[2];
        conn->frame_size = buf[3];
        conn->stream_repeat = ((uint32_t)buf[4] << 24) + ((uint32_t)buf[5] << 16) + ((uint32_t)buf[6] << 8) + buf[7];

        if(buf[8] == 16) {
            conn->hw_crc = RF24_CRC_16;
        } else if(buf[8] == 8) {
            conn->hw_crc = RF24_CRC_8;
        } else {
            conn->hw_crc = RF24_CRC_DISABLED;
        }
        conn->sw_crc_enabled = buf[9];
        conn->auto_ack_enabled = buf[10];
        return true;

    } else {
        return false;
    }
}


void resetRadio(bool power_reset) {
    Serial.println(F("=== Resetting nRF24L01 radio..."));

    if(power_reset) {
        // Power down and up
        radio.powerDown();
        delay(100);
        radio.powerUp();
    }

    // Reinitialize the radio
    if (!radio.begin()) {
        Serial.println(F("=== NRF24L01 initialization failed."));
        while(true) {
            delay(1000); // Halt execution if initialization fails
        }
    }

    // Reapply configurations
    radio.setPALevel(pa_setting);
    radio.setChannel(channel);
    radio.setDataRate(dr_setting);
    radio.setCRCLength(tx_connection.hw_crc);
    radio.setPayloadSize(NRF24_BUFSIZE);
    radio.setAutoAck(tx_connection.auto_ack_enabled);
    radio.openWritingPipe(addr[addr_selector_txpipe]);
    radio.openReadingPipe(1, addr[addr_selector_rxpipe]);

    // Flush buffers
    radio.flush_tx();
    radio.flush_rx();

    Serial.println(F("=== nRF24L01 radio reset complete."));
}

void printPacket(const uint8_t * buf, const uint8_t len) {
    Serial.print(buf[0], HEX);
    Serial.print(F(" : "));
    for(uint8_t i = 1; i < len - 3; i++) {
        Serial.print((char)(buf[i]));
    }
    Serial.print(buf[len - 3], HEX); // CRC MSB
    Serial.print(F(","));
    Serial.print(buf[len - 2], HEX); // CRC LSB
    Serial.print(F(","));
    Serial.println(buf[len - 1], HEX); // null
}