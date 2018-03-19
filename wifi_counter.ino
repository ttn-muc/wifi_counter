#include <lmic.h>
#include <hal/hal.h>
#include <CayenneLPP.h>
#include <WiFi.h>
#include <U8x8lib.h>

#include "config.h"


// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300;

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

CayenneLPP lpp(51);

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            u8x8.drawString(0, 7, "EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            u8x8.drawString(0, 7, "EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            u8x8.drawString(0, 7, "EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            u8x8.drawString(0, 7, "EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            u8x8.drawString(0, 7, "EV_JOINING   ");
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            u8x8.drawString(0, 7, "EV_JOINED    ");
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            u8x8.drawString(0, 7, "EV_RFUI");
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            u8x8.drawString(0, 7, "EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            u8x8.drawString(0, 7, "EV_REJOIN_FAILED");
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            u8x8.drawString(0, 7, "EV_TXCOMPLETE");
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            /*Serial.print(F("Frequency: "));
            Serial.println(LMIC.freq);
            Serial.print(F("RSSI: "));
            Serial.println(LMIC.rssi);
            Serial.print(F("SNR: "));
            Serial.println(LMIC.snr);
            Serial.print(F("txpow: "));
            Serial.println(LMIC.txpow);*/
            u8x8.setCursor(0, 2);
            u8x8.printf("adrTxPow: %i", LMIC.adrTxPow);
            u8x8.setCursor(0, 3);
            u8x8.printf("txChnl: %i", LMIC.txChnl);
            
            Serial.print(F("adrTxPow: "));
            Serial.println(LMIC.adrTxPow);
            Serial.print(F("txChnl: "));            
            Serial.println(LMIC.txChnl);            
            Serial.println();
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        u8x8.drawString(0, 0, "DataCollection");
        u8x8.drawString(0, 1, "WiFiScan...");
        int n = WiFi.scanNetworks();
        u8x8.setCursor(0, 1);
        u8x8.printf("%i WiFi APs", n);
        
        lpp.reset();
        lpp.addDigitalInput(1, n);

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Packet queued"));
        u8x8.drawString(0, 7, "PACKET QUEUED");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting TTN Muc WiFi Counter"));

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Let LMIC compensate for +/- 1% clock error
    //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
