/**************************************************************
 *
 * For this example, you need to install PubSubClient library:
 *   https://github.com/knolleary/pubsubclient
 *   or from http://librarymanager/all#PubSubClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more MQTT examples, see PubSubClient library
 *
 **************************************************************
 * This example connects to HiveMQ's showcase broker.
 *
 * You can quickly test sending and receiving messages from the HiveMQ webclient
 * available at http://www.hivemq.com/demos/websocket-client/.
 *
 * Subscribe to the topic GsmClientTest/ledStatus
 * Publish "toggle" to the topic GsmClientTest/led and the LED on your board
 * should toggle and you should see a new message published to
 * GsmClientTest/ledStatus with the newest LED status.
 *
 **************************************************************/

// Select your modem:
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
#define SerialAT Serial1

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
// #define TINY_GSM_DEBUG SerialMon

#define UART_BAUD    115200
#define BOARD_MODEM_DTR_PIN                 25
#define BOARD_MODEM_TX_PIN                  26
#define BOARD_MODEM_RX_PIN                  27
#define BOARD_MODEM_PWR_PIN                 4
#define BOARD_ADC_PIN                       35
#define BOARD_POWER_ON_PIN                  12
#define BOARD_MODEM_RI_PIN                  33
#define BOARD_RST_PIN                       5
#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13

// Define how you're planning to connect to the internet.
// This is only needed for this example, not in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#define DISCONNECT_PIN 32

// set GSM PIN, if any
#define GSM_PIN ""

#include "credentials.h"

// Your GPRS credentials, if any
const char apn[]      = "YourAPN";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = WIFI_SSID;
const char wifiPass[] = WIFI_PASSWD;

// MQTT details
const char* broker = MQTT_ADDRESS;

const char* topicLed       = "GsmClientTest/led";
const char* topicInit      = "GsmClientTest/init";
const char* topicLedStatus = "GsmClientTest/ledStatus";

#include <esp_adc_cal.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

#include <SdFat.h>
#include "DataManager.h"


#include "TimeLib.h"


#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
SdFat SD; 
DataManager dataManager(SD, &mqtt, "mqttest", "gps_log.txt", "offline_log.txt");

JsonDocument message;
JsonDocument configMessage;

// int good_connection = 1;

#define LED_PIN 13
int ledStatus = LOW;

int vref = 1100;

uint32_t lastReconnectAttempt = 0;
uint32_t lastGPSUpdate = 20000;
uint32_t lastFileUpdate = 10000;

char buf[300];

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicLed) {
    ledStatus = !ledStatus;
    digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  }
}

void setupAdcVref(){
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);    //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      Serial.printf("eFuse Vref:%u mV\n", adc_chars.vref);
      vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
      Serial.println("Default Vref: 1100mV");
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  // boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientName", MQTT_USERNAME, MQTT_PASSWD);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  mqtt.publish(topicInit, "GsmClientTest started");
  // mqtt.subscribe(topicLed);
  return mqtt.connected();
}

void initSDCard(){
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SD_SCK_MHZ(1))) {
      SerialMon.println("SDCard MOUNT FAIL");
  } else {
      SerialMon.println("Success --- SDCard");
      dataManager.sdCardAvailable = true;
      csd_t csd;
      if (!SD.card()->readCSD(&csd)) { 
        Serial.print(F("readInfo failed\n"));
      }
      SerialMon.printf("cardSize: %f  MB (MB = 1,000,000 bytes)\n",0.000512 * csd.capacity());
  }
}


void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  pinMode(LED_PIN, OUTPUT);

  pinMode(BOARD_POWER_ON_PIN, OUTPUT);
  digitalWrite(BOARD_POWER_ON_PIN, HIGH);

  pinMode(BOARD_RST_PIN, OUTPUT);
  digitalWrite(BOARD_RST_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_RST_PIN, HIGH);
  delay(3000);
  digitalWrite(BOARD_RST_PIN, LOW);

  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
    digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
    delay(1000);
    digitalWrite(BOARD_MODEM_PWR_PIN, LOW);

  SerialMon.println("Wait...");

  setupAdcVref();

  // pinMode(DISCONNECT_PIN, INPUT_PULLUP);
  // initSDCard();

  // Set GSM module baud rate
  // TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, BOARD_MODEM_RX_PIN, BOARD_MODEM_TX_PIN);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  // modem.restart();
  modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }

  // Disable gnss
  modem.sendAT("+CGNSSPWR=0");
  modem.waitResponse(10000L);

  //Enable gnss
  modem.sendAT("+CGNSSPWR=1");
  modem.waitResponse(10000L);

  //Wait gnss start.
  SerialMon.print("\tWait GPS ready.");
  while (modem.waitResponse(1000UL, "+CGNSSPWR: READY!") != 1) {
      SerialMon.print(".");
  }
  SerialMon.println();

  //Set gnss mode use GPS.
  modem.sendAT("+CGNSSMODE=1");
  modem.waitResponse(10000L);

  // MQTT Broker setup
  mqtt.setServer(broker, 8417);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }

#if TINY_GSM_USE_GPRS
    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) { SerialMon.println("GPRS reconnected"); }
    }
#endif
  }

  uint32_t t = millis();
  if (
      !mqtt.connected() 
      // && good_connection
  ) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
  }

  // if(!digitalRead(DISCONNECT_PIN)){
  //   good_connection = !good_connection;
  //   digitalWrite(LED_PIN,!good_connection);
  //   if(!good_connection){
  //     mqtt.disconnect();
  //     Serial.println("DISCONNECTING");
  //   }
  //   delay(500);
  // }

  if(t - lastGPSUpdate > 20000){
    lastGPSUpdate = t;
    char buf[300];
      modem.sendAT(GF("+CGNSSINFO"));
      if (modem.waitResponse(GF(GSM_NL "+CGNSSINFO:")) == 1) {
        // digitalWrite(LED_PIN, 1);
        uint16_t fixMode = getIntFromStreamBefore(',');
        if (fixMode > 0 && fixMode < 4){
            String gpsData = modem.stream.readStringUntil('\n');
            gpsData.trim();
            /*
            sats: 0-1
            lat: 4-5
            latNS: 5-6
            lng: 6-7
            */
            // gpsData
            float battery_voltage = ((float)analogRead(BOARD_ADC_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
            message.clear();
            message["gps"] = gpsData.c_str();
            message["ID"] = modem.getIMEI().c_str();
            message["VBat"] = battery_voltage;
            // sprintf(buf, "{\"GPS\":\"%s\",\"ID\":\"%s\",\"VBat\":%f}", gpsData.c_str(), modem.getIMEI().c_str(), battery_voltage);
            // mqtt.publish("gsm/gps_raw", buf);
            dataManager.sendMessage(message);
        } else {
            // Serial.print(millis());
            Serial.printf("%ld -> fix %d\n", millis(), fixMode);
        }
      } else {
        // digitalWrite(LED_PIN, 0);
        Serial.printf("No modem response\n");
      }
  }
  if(t - lastFileUpdate > 10000L){
    lastFileUpdate = t;
    dataManager.loop();
  }
  mqtt.loop();
}


// String manipulation functions
int16_t getIntFromStreamBefore(char separator){
  char   buf[7];
  size_t bytesRead = modem.stream.readBytesUntil(
        separator, buf, static_cast<size_t>(7));
  if(bytesRead && bytesRead < 16){
    buf[bytesRead] = 0;
    int16_t res = atoi(buf);
    return res;
  }
  return -9999;
}