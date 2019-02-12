# ESP-LoRa-ControlCenter
This uses an ESP8266 as centralized hub for RF Long-Range devices. The devices are controlled using the Cayenne app

This project is a prototype to receive sensor node data and battery life using the Cayenne app. I wanted to aggregate my data collection to a centralized hub using SparkFun's ESP8266 Thing Dev Board. This makes it easier for me to check the battery life and ambient light data from several sensor nodes without the need to go into the field. The data can be displayed on a graph within the app itself. Hopefully this can help jump start someone else's prototyping.

Here are the following steps I took:

1. Setup and get familiar with Cayenne myDevices app
https://www.theengineeringprojects.com/2016/09/getting-started-with-cayenne-arduino.html

2. Hook up SparkFun's ESP8266 Thing Dev Board to the Adafruit RFM Radio

3. Upload the "Host" code to the Thing Dev Board

4. Wire the TSL2561 Sensor to Adafruit's Feather M0 RFM95

5. Upload the "Client" code to the Feather M0 RFM95

6. Open the Cayenne app and test!

One of the things that I had to play around with to make it work are the button numbers set on both the code and the Cayenne app.
