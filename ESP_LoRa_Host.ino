// ESP-LoRa Control Center
// An RF LoRa Control Center to dictate commands and receive data from a sensor node.
// 
// This is based on Tanmoy Dutta's "Adafruit Feather Huzzah ESP8266 LoRa receiver for data relay to Raspberry Pi IoT Gateway"
// Augmented by: HyperChiicken

#define CAYENNE_PRINT Serial
#include <CayenneMQTTESP8266.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>

// WiFi network info.
char ssid[] = "insert_your_wifi_id_here";
char wifiPassword[] = "insert_your_password_here";

WiFiClient client;

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "insert_cayenne_username_here";
char password[] = "insert_password_here";
char clientID[] = "insert_client_id_here";

#define RFM95_CS 16
#define RFM95_RST 0
#define RFM95_INT 15

// Blinky on receipt
#define LED 5

// Set radion frequency
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


/*----------------------------------------------------------------------------
  Function : setup()
  Description :
  ------------------------------------------------------------------------------*/
void setup() {
  pinMode(2, OUTPUT);

  Serial.begin(115200);
  Cayenne.begin(username, password, clientID, ssid, wifiPassword);
  _initLoRa();
}


/*----------------------------------------------------------------------------
  Function : loop()
  Description : Main program loop
  ------------------------------------------------------------------------------*/
void loop() {
  // Set local variables
  char buf_txt[32] = " ",
       data[8] = " ",
       devID[2] = " ",
       CMD[2] = " ",
       N1_data[8] = " ",
       N2_data[8] = " ";
  uint16_t full, ir;
  int TEST;
  long lastConnectionTime = 0;
  boolean lastConnected = false,
          isReady_N1 = false,
          isReady_N2 = false;
  int FAILED = 0,
      runNum = 0;

  Cayenne.loop();

  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED, HIGH);
      delay(10);

      // Copy buffer data into char array
      memcpy(buf_txt, buf, len);
      Serial.println(buf_txt);

      // Get Device ID      
      devID[0] = buf_txt[4];
      devID[1] = buf_txt[5];
      Serial.println(devID[1]);
      
      // Get Command ID
      CMD[0] = buf_txt[7];
      CMD[1] = buf_txt[8];

      // Get Data
      TEST = atoi((char*)CMD);
      Serial.println(TEST);
      uint8_t x = TEST;
      Serial.println(x);

      int i = 0;
      while (buf_txt[i] != ' ') {
        data[i] = buf_txt[9 + i];;
        i++;
      }
      uint8_t len2 = sizeof(data);

      Serial.println(data);
      Serial.println(x);

      // Check signal strength
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      delay(10);
          
      // Store data & send reply according to device ID
      switch (devID[1]) {
        case '1': {
            memcpy(N1_data, data, len2);

            ///////////////////////MQTT Code
            Cayenne.virtualWrite(x, N1_data);
            delay(500);
            //////////////////////////////////

            isReady_N1 = true;

            // Send a reply
            uint8_t outgoingData[] = "Node01:OK";
            rf95.send(outgoingData, sizeof(outgoingData));
            rf95.waitPacketSent();
            digitalWrite(LED, LOW);
          } break;
        case '2': {
            memcpy(N2_data, data, len2);
            Serial.println(x);
            
            ///////////////////////MQTT Code
            Cayenne.virtualWrite(x, N2_data);
            delay(500);
            //////////////////////////////////

            isReady_N2 = true;

            // Send a reply
            uint8_t outgoingData[] = "Node02:OK";
            rf95.send(outgoingData, sizeof(outgoingData));
            rf95.waitPacketSent();
            digitalWrite(LED, LOW);
          } break;
        default: {
            Serial.println("FAILED");
            FAILED++;
          } break;
      }
    }
    else
    {
      Serial.println("Receive failed");
      FAILED++;
    }
  }
  else {
    //displayOnOLED("Receive failed");
  }
}

void donothing() {
}

/*----------------------------------------------------------------------------
  Function : _initLoRa()
  Description : Connect to WiFi access point
  ------------------------------------------------------------------------------*/
void _initLoRa() {
  attachInterrupt(digitalPinToInterrupt(RFM95_INT), donothing, CHANGE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //  displayOnOLED("LoRa RX WiFi Repeater");
  delay(1000);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    //    displayOnOLED("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  //  displayOnOLED("LoRa radio init OK!");
  delay(1000);

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    //    displayOnOLED("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(5, false);
  Serial.println("LoRa Listening...");
  //  displayOnOLED("LoRa Listening...");
  delay(1000);
}

/*----------------------------------------------------------------------------
  Cayenne Functions
  Description : Get inputs from Cayenne to send out command
  ------------------------------------------------------------------------------*/
CAYENNE_IN_DEFAULT()
{
  CAYENNE_LOG("CAYENNE_IN_DEFAULT(%u) - %s, %s", request.channel, getValue.getId(), getValue.asString());
  //Process message here. If there is an error set an error message using getValue.setError(), e.g getValue.setError("Error message");
}

// Send out command #1 to sensor node
CAYENNE_IN(21)
{
  CAYENNE_LOG("CAYENNE_IN_DEFAULT(%u) - %s, %s", request.channel, getValue.getId(), getValue.asString());
  //Process message here. If there is an error set an error message using getValue.setError(), e.g getValue.setError("Error message");

  int i = getValue.asInt();
  digitalWrite(2, i);
  uint8_t reply[] = "Node02:1";
  rf95.send(reply, sizeof(reply));
  rf95.waitPacketSent();
  Serial.println("Sent a reply");
  digitalWrite(LED, LOW);
  Serial.println(" ");
}

// Send out command #2 to sensor nide
CAYENNE_IN(22)
{
  CAYENNE_LOG("CAYENNE_IN_DEFAULT(%u) - %s, %s", request.channel, getValue.getId(), getValue.asString());
  //Process message here. If there is an error set an error message using getValue.setError(), e.g getValue.setError("Error message");

  int i = getValue.asInt();
  digitalWrite(2, i);
  uint8_t reply[] = "Node02:2";
  rf95.send(reply, sizeof(reply));
  rf95.waitPacketSent();
  Serial.println("Sent a reply");
  digitalWrite(LED, LOW);
  Serial.println(" ");
}
