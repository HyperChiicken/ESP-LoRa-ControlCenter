// ESP-LoRa Sensor Node aka Client
// This code is based on Adafruit's Feather9x_TX simple messaging client (trnamitter)
// Augmented by : HyperChiicken

#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

/* Set debug mode ON or OFF*/
//#define DEBUG

bool ACK = 0;
/* for feather32u4
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 7
*/

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


/* for shield
  #define RFM95_CS 10
  #define RFM95_RST 9
  #define RFM95_INT 7
*/


/* for ESP w/featherwing
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"
*/

/* Feather 32u4 w/wing
  #define RFM95_RST     11   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Feather m0 w/wing
  #define RFM95_RST     11   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     6    // "D"
*/

/* Teensy 3.x w/wing
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Create TSL2561 instance
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  // tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
}

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial.println("Starting Adafruit TSL2561 Test!");

  /* Initialise the sensor */
  if (!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

  /* We're ready to go! */
  Serial.println("");
  delay(10);
}

void loop()
{
  // Set local variables
  char buf_txt[32] = " ", 
       data[7] = " ", 
       N1_data[7] = " ",
       N2_data[7] = " ", 
       devID[2] = " ",
       CMD[2] = "0",
       databuf[11] = " ",
       radiopacket_end[2] = " ";
  char test[2] = " "; 
  uint8_t dev_ID[1], cmd[1], FAILED;
  float vbat, lux, dataOUT;

  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("Got: ");

      // Copy buffer data into char array
      memcpy(buf_txt, buf, len);
      Serial.println(buf_txt);

      // Get Device ID
      dev_ID[0] = buf_txt[4];
      dev_ID[1] = buf_txt[5];

      Serial.print("Device_ID: ");
      Serial.println(dev_ID[1]);

      devID[0] = buf_txt[4];
      devID[1] = buf_txt[5];
       
      // Get command
      cmd[1] = buf_txt[7];
      CMD[1] = buf_txt[7];
      //char cmd_str[2] = " ";
     
      Serial.println(cmd[1]);
      //Serial.println(cmd_str);
      // Check if the message is addressed to this device
      if (dev_ID[1] == 50) {
        switch (cmd[1]) {
          // Send battery voltage
          case 49: {
              vbat = checkBattery();
              dataOUT = vbat;
              dtostrf(vbat, 4, 2, databuf);
            } break;

          // Send TSL data
          case 50: {
              lux = getLux();
              lux += getLux();
              lux += getLux();
              lux += getLux();
              lux += getLux();
              Serial.println(lux);
              lux = lux/5;
              Serial.println(lux);
              dataOUT = lux;
              dtostrf(lux, 4, 2, databuf);
            } break;
          default: {
              Serial.println("FAILED");
              FAILED++;
            } break;
        }

        Serial.println(databuf);

        char radiopacket[20] = "Node02:";
        
        strcat(radiopacket, (char*)CMD);
        strcat(radiopacket, databuf);
        Serial.println(radiopacket_end[1]);
        strcat(radiopacket, radiopacket_end);
        Serial.print("Sending "); Serial.println(radiopacket);
       
        Serial.print("Sending..."); delay(10);
        // Wait for acknowledgment
        while (ACK == 0) {
          rf95.send((uint8_t*)radiopacket, sizeof(radiopacket));
          rf95.waitPacketSent();
          getACK();
        }
        ACK = 0; // reset variable

      }
    }
    else {
      //rf95.sleep();
    }
  }
}

/*
   This routine checks for acknowledgment reply from server
*/
void getACK(void) {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  char buf_txt[32], data[7];
  int dev_ID[1];

  if (rf95.waitAvailableTimeout(3000)) {
    if (rf95.recv(buf, &len)) {
      // Copy buffer data into char array
      memcpy(buf_txt, buf, len);

      // Get Device ID
      dev_ID[0] = buf_txt[4];
      dev_ID[1] = buf_txt[5];

      // Get Data
      int i = 0;
      while (buf_txt[i] != ' ') {
        data[i] = buf_txt[13 + i];;
        i++;
      }
      uint8_t len2 = sizeof(data);
      delay(10);

      // Check if the message is addressed to this device
      if (dev_ID[1] == 50) {
        ACK = 1;
        Serial.println("OK"); delay(10);
      }
      else {
        ACK = 0;
        Serial.println("FAILED"); delay(10);
      }
    }
  }
}

/*
   This routine checks the voltage on the battery (if it exists) and returns that value.  Should be in the 3.2-4.2 range depending upon the battery used
*/
float checkBattery() {
  //This returns the current voltage of the battery on a Feather 32u4.
  float measuredvbat = analogRead(9);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return measuredvbat;
}

/*
   This routine acquires the TSL2561 sensor data
*/
float getLux() {
  float measuredlux;
  uint16_t full, ir;

  tsl.getLuminosity(&full, &ir);
  measuredlux = tsl.calculateLux(full, ir);

  return measuredlux;
}

/*
   This simulates the dtostrf function from avr-libc
*/
char* dtostrf (float val, signed char width, unsigned char prec, char *sout) {
  asm(".global _printf_float");

  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
