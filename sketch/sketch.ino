#include <DHTesp.h>
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
const byte ONEWIRE_PIN = 2;

/** Pin number for DHT11 data pin */
int dhtPin = 17;
DHTesp dht;

/* Analog pin for FC28 */
int analogpin_FC28 = 36;

/* FC28 minimal value = 0% */
int maxValue = 1023;

/* FC28 maximal value = 100% */
int minValue = 18;


/* Return codes for the getTemperature() function */
enum DS18B20_RCODES {
  READ_OK,
  NO_SENSOR_FOUND,
  INVALID_ADDRESS, 
  INVALID_SENSOR
};

/* OneWire object creation */
OneWire ds(ONEWIRE_PIN);

/* OLED Screen */
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

void setup() {
  
  Serial.begin(115200);

  dht.setup(dhtPin, DHTesp::DHT11);

  pinMode(analogpin_FC28, INPUT);

  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 0, "CompostGuard 1.0");
  
  u8x8.drawString(0, 2, "      Soil  Air");
  u8x8.drawString(0, 3, "T C   --.-  --.-");
  u8x8.drawString(0, 4, "H %   ---   ---");

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
  
  TempAndHumidity newValues = dht.getTempAndHumidity();
  
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
  }
  
  sprintf(buffer, "%3.1f", newValues.temperature);
  u8x8.drawString(12, 3, buffer);

  sprintf(buffer, "%3.0f", newValues.humidity);
  u8x8.drawString(12, 4, buffer);

  // Read Soil temperature
  if (getTemperature(&temperature, true) != READ_OK) {
    Serial.println(F("Erreur de lecture du capteur"));
    return;
  }

  sprintf(buffer, "%3.1f", temperature); // conversion float en char
  u8x8.drawString(6, 3, buffer);

  //Lecture de la valeur
  analogReadResolution(10);
  int sensorValue = constrain(analogRead(analogpin_FC28), minValue, maxValue);  
  // Map to % value
  int soil = map(sensorValue, minValue, maxValue, 100, 0);
  sprintf(buffer, "%3d", soil);
  u8x8.drawString(6, 4, buffer);


  // Prepare payload
  lpp.reset();

  // Soil
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(2, soil);

  // Air
  lpp.addTemperature(3, newValues.temperature);
  lpp.addRelativeHumidity(4, newValues.humidity);
  
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
    u8x8.setCursor(0, 6);
    u8x8.printf("TIME %lu", os_getTime());
    Serial.print(": ");
    u8x8.setCursor(0, 7);
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            u8x8.print("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            u8x8.print("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            u8x8.print("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            u8x8.print("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            u8x8.print("EV_JOINING");
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            u8x8.print("EV_JOINED ");
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            u8x8.print("EV_RFUI");
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            u8x8.print("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            u8x8.print("EV_REJOIN_FAILED");
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            u8x8.print("EV_TXCOMPLETE");
            digitalWrite(BUILTIN_LED, LOW);
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
              u8x8.setCursor(0, 7);
              u8x8.print("Received ACK");
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
            u8x8.print("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            u8x8.print("EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            u8x8.print("EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            u8x8.print("EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            u8x8.print("EV_LINK_ALIVE");
            break;
         default:
            Serial.println(F("Unknown event"));
            u8x8.setCursor(0, 7);
            u8x8.printf("UNKNOWN EVENT %d", ev);
            break;
    }
}

void do_send(osjob_t* j) {
    u8x8.setCursor(0, 7);
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        u8x8.print("OP_TXRXPEND, not sent");
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Packet queued"));
        u8x8.print("PACKET QUEUED");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

