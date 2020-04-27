
/*
 * USB Adapter for Sun Type 5 Keyboard
 * Ben Rockwood <benr@cuddletech.com> 1/1/17
 * Jaap Geurts 04/22/2020
 *
 * Developed on Arduino Micro
 * MiniDIN 8a Adapter P/N: MD-80PL100
 * Wiring:
 *    Keyboard Pin 2 (White): GND
 *    Keyboard Pin 8 (Red): +5V
 *    Keyboard Pin 6 (Yellow): Arduino Pin D10 (Serial RX)
 *    Keyboard Pin 5 (Green): Aruidno Pin D11 (Serial TX)
 */

#include <HID-Project.h>
#include <HID-Settings.h>

#include <SoftwareSerial.h>
#include "sunkeytable.h"

#define KEY 7
#define TX 8
#define RX 9

//Software serial for Sun KBD
SoftwareSerial sunSerial(RX, TX, true);

boolean NUM_LOCK_ON = false;  // led bitfield xxx1 (1)
boolean CAPS_LOCK_ON = false; // led bitfield 1xxx (8)

byte led_cmd[2] = {0x0E, 0x01}; // Used for sending the 2 byte commands to the keyboard

byte modifierMask = 0x00;

void setup()
{

    pinMode(KEY, INPUT);

    Serial.begin(1200);    // Normal serial for Serial Monitor Debugging
    sunSerial.begin(1200); // Serial Connection to Sun Keyboard

    Keyboard.begin(); // Initialize USB Keyboard Interface

    sunSerial.write(led_cmd, 2); // Enable Number Lock by Default
}

void loop()
{
    // do block startup unless keypress to allow reprogramming
    if (digitalRead(KEY) == LOW)
    { // user pushed. block
        delay(50);
        while (digitalRead(KEY) == LOW)
            ; // wait until release
        delay(50);
        while (digitalRead(KEY) == HIGH) // wait while not pusing again
            delay(100);
    }

    if (sunSerial.available())
    {
        byte c = sunSerial.read();
        byte key = c & 0x7f;
        if (key == 0x7f)
        {
            Serial.println("All Keys Released");
            //     Keyboard.releaseAll();
        }
        else
        {
            uint16_t translated = keytable[key];
            bool isReleased = (c & 0x80) >> 7;
            if (translated & 0xe0)
            {
                uint8_t mask = (1 << (translated & 0x07));
                if (isReleased)
                    modifierMask &= ~mask;
                else
                    modifierMask |= mask;
            }
            if (!isReleased)
            {
                Serial.print("Key Down: ");
                Serial.println(keytable[key]);
               

                Keyboard.press(KeyboardKeycode(translated | (modifierMask << 8)));
            }
            else
            {
                Serial.print("Key Up: ");
                Serial.println(KeyboardKeycode(keytable[key]));

                Keyboard.release(KeyboardKeycode(translated | (modifierMask << 8)));
            }
        }
    }
}
