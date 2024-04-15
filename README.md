# ESP32-CAM based door bell camera

ESP32-CAM based project of a doorbell camera, the project is pretty straight forward and was an idea to help my mother with answering the door to people she knows and wants to answer.

## Basic functionality

This project is connected to the door bell of your home, which requires some tweaking with connecting to the bell voltage regulator(CAUTION!).
The ESP32-CAM is set to be in deep sleep all the time, that is the reason it does not need a loop function, when the bell is rung, the configured pin will trigger the system to wake up which will start the setup of the ESP. While in setup, we configure the camera, connect to the wifi network and connect to the Telegram API, the camera then takes a picture, builds it and sends it to the chat bot(that you need to generate in Telegram through BotFather), after the sending is done the board will go back to sleep until the next door bell is rung.

## Required hardware

- AI THINKER ESP32-CAM (or any other ESP32-CAM module)
- A 220 volts AC to 5 volts DC converter (or 110 volts, depends on your location)
- A 10Kohm resistor
- A 3.3Kohm resistor
- Encloser to hang on the door
- Wiring to connect to the door bell circuitry 

## Connection scheme



## GPIOs

Function | GPIO ESP32-CAM | Mode
-------- | -------- | --------
Power Down | GPIO32 | Input
Reset | GPIO-1 | Input
XCLK | GPIO0 | Input
SIOD | GPIO26 | Input
SIOC | GPIO27 | Input
Y9 | GPIO35 | Output
Y8 | GPIO34 | Output
Y7 | GPIO39 | Output
Y6 | GPIO36 | Output
Y5 | GPIO21 | Output
Y4 | GPIO19 | Output
Y3 | GPIO18 | Output
Y2 | GPIO5 | Output
Vertical Sync | GPIO25 | Output
Horizontal Ref | GPIO23 | Output
PCLK | GPIO22 | Input
LED | GPIO4 | Output
Wake up | GPIO13 | Input

## Reducing power consumption

In order to reduce the power consumption of the ESP32-CAM the following points can be done:

- Leave `void loop()` empty, even if code is implemented there the PC won't get to the main loop.
- Reducing CPU frequency after WiFi stuff is completed (crystal is 240MHz): `setCpuFrequencyMhz(40); //reduce to 40MHz`, this step is not quite necessary because after the WiFi connection it sends the picture and goes back to sleep.
