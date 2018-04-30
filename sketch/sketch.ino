#include <OneWire.h>
#include <U8x8lib.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/* include your TheThingsNetwork device configuration */
#include "TTNConfig.h"

/* include Cayenne Low Payload Profile */
#include <CayenneLPP.h>
CayenneLPP lpp(51);

/* 1-Wire Pin for DS18B20 */
const byte BROCHE_ONEWIRE = 2;

/* Analog pin for FC28 */
int analogpin_FC28 = 13;

/* FC28 minimal value = 0% */
int maxValue = 5980;

/* FC28 maximal value = 100% */
int minValue = 1600;


/* Return codes for the getTemperature() function */
enum DS18B20_RCODES {
  READ_OK,
  NO_SENSOR_FOUND,
  INVALID_ADDRESS, 
  INVALID_SENSOR
};

/* OneWire object creation */
OneWire ds(BROCHE_ONEWIRE);

/* OLED Screen */
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 300;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

void setup() {
  
  Serial.begin(115200);

  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 1, "Mon Composteur");
  u8x8.drawString(0, 3, "Temperature:");
  u8x8.drawString(0, 4, "Humidite:");

  SPI.begin(5, 19, 27);

  // LMIC init
  os_init();
  
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  #if defined(CFG_eu868)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  #elif defined(CFG_us915)
    LMIC_selectSubBand(1);
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  float temperature;
  char buffer[20];

  /* Lit la température ambiante à ~1Hz */
  if (getTemperature(&temperature, true) != READ_OK) {
    Serial.println(F("Erreur de lecture du capteur"));
    return;
  }

  sprintf(buffer, "%3.1f", temperature); // conversion float en char
  u8x8.drawString(10, 3, strcat(buffer,"C"));

  //Lecture de la valeur
  int sensorValue = analogRead(analogpin_FC28);
  sensorValue = constrain(sensorValue, minValue, maxValue);  
  // Calcule en pourcentage
  int soil = map(sensorValue, minValue, maxValue, 100, 0);
  sprintf(buffer, "%0d", soil); // conversion float en char
  u8x8.drawString(11, 4, strcat(buffer, "%"));


  lpp.reset();
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(2, soil);

  os_runloop_once();

  delay(5000);
}

/**
 * Temperature reading from a DS18B20 sensor
 */
byte getTemperature(float *temperature, byte reset_search) {
  byte data[9], addr[8];
  // data[] : data read from scratchpad
  // addr[] : detected 1-Wire module address

  /* Reset 1-Wire bus if needed (mandatory for first sensor reading) */
  if (reset_search) {
    ds.reset_search();
  }

  /* Search next  1-Wire sensor available */
  if (!ds.search(addr)) {
    // No sensor
    return NO_SENSOR_FOUND;
  }

  /* Check that right address has been transmitted */
  if (OneWire::crc8(addr, 7) != addr[7]) {
    // Invalid address
    return INVALID_ADDRESS;
  }

  /* Check that sensor is a DS18B20 */
  if (addr[0] != 0x28) {
    // Wrong sensor type
    return INVALID_SENSOR;
  }

  /* Reset 1-Wire bus and select sensor */
  ds.reset();
  ds.select(addr);

  /* Lance une prise de mesure de température et attend la fin de la mesure */
  ds.write(0x44, 1);
  delay(800);

  /* Reset 1-Wire bus, select sensor and send scratchpad reading request */
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);

 /* Scratchpad reading */
  for (byte i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  /* Temperature value compute in Celsius */
  *temperature = (int16_t) ((data[1] << 8) | data[0]) * 0.0625;
  // No error
  return READ_OK;
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    u8x8.setCursor(0, 5);
    u8x8.printf("TIME %lu", os_getTime());
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
            u8x8.drawString(0, 7, "EV_JOINING");
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            u8x8.drawString(0, 7, "EV_JOINED ");
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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            u8x8.drawString(0, 7, "EV_TXCOMPLETE");
            digitalWrite(BUILTIN_LED, LOW);
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
              u8x8.drawString(0, 7, "Received ACK");
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              u8x8.drawString(0, 6, "RX ");
              Serial.println(LMIC.dataLen);
              u8x8.setCursor(4, 6);
              u8x8.printf("%i bytes", LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              u8x8.setCursor(0, 7);
              u8x8.printf("RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            u8x8.drawString(0, 7, "EV_LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            u8x8.drawString(0, 7, "EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            u8x8.drawString(0, 7, "EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            u8x8.drawString(0, 7, "EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            u8x8.drawString(0, 7, "EV_LINK_ALIVE");
            break;
         default:
            Serial.println(F("Unknown event"));
            u8x8.setCursor(0, 7);
            u8x8.printf("UNKNOWN EVENT %d", ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Packet queued"));
        u8x8.drawString(0, 7, "PACKET QUEUED");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

