## Sun Type 6 Keyboard Adapter

This software is a Sun Type 6 UART Keyboard to USB adapter. It lets you connect a sun type 6 non-usb keyboard to your PC in a USB port
using an Arduino Micro. The Arduino will present itself as a real Sun Type 6 USB keyboard.

## Connecting the keyboard

Connect your Sun keyboard as follows:

See this page for a description of the DIN connector

https://github.com/benr/SunType5_ArduinoAdapter

Then connect From KDB to pin 9 on the Arduino, and To KBD to pin 8 on the Arduino.
Hook up power and ground.

## Building

You need to download and install (!https://github.com/abcminiuser/lufa)[LUFA].

Then issue the following command:

`$ make`

Make sure an Arduino Micro/Leonardo (or any other AVR 32u4 board is attached)

`$ make flash`


