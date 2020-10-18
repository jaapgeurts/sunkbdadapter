/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license),
please contact mla_licensing@microchip.com
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <string.h>

#include "system.h"
#include "usb/usb.h"
#include "usb/usb_device_hid.h"

#include "uart.h"
#include "sunkeytable.h"


#if defined(__XC8)
#define PACKED
#else
#define PACKED __attribute__((packed))
#endif

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************

#define LED_NUMLOCK 0x01
#define LED_COMPOSE 0x02
#define LED_SCROLLLOCK 0x04
#define LED_CAPSLOCK 0x08

#define CMD_RESET     0x01
#define CMD_BELL_ON   0x02
#define CMD_BELL_OFF  0x03
#define CMD_CLICK_ON  0x0A
#define CMD_CLICK_OFF 0x0B
#define CMD_LED       0x0E
#define CMD_LAYOUT    0x0F

//Class specific descriptor - HID Keyboard

const struct
{
  uint8_t report[HID_RPT01_SIZE];
} hid_rpt01 = {
  { 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x06, // USAGE (Keyboard)
    0xa1, 0x01, // COLLECTION (Application)
    0x05, 0x07, //   USAGE_PAGE (Keyboard)
    0x19, 0xe0, //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7, //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x25, 0x01, //   LOGICAL_MAXIMUM (1)
    0x75, 0x01, //   REPORT_SIZE (1)
    0x95, 0x08, //   REPORT_COUNT (8)
    0x81, 0x02, //   INPUT (Data,Var,Abs)
    0x95, 0x01, //   REPORT_COUNT (1)
    0x75, 0x08, //   REPORT_SIZE (8)
    0x81, 0x03, //   INPUT (Cnst,Var,Abs)
    0x95, 0x05, //   REPORT_COUNT (5)
    0x75, 0x01, //   REPORT_SIZE (1)
    0x05, 0x08, //   USAGE_PAGE (LEDs)
    0x19, 0x01, //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05, //   USAGE_MAXIMUM (Kana)
    0x91, 0x02, //   OUTPUT (Data,Var,Abs)
    0x95, 0x01, //   REPORT_COUNT (1)
    0x75, 0x03, //   REPORT_SIZE (3)
    0x91, 0x03, //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06, //   REPORT_COUNT (6)
    0x75, 0x08, //   REPORT_SIZE (8)
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x25, 0xff, // 0x65, //   LOGICAL_MAXIMUM (101)
    0x05, 0x07, //   USAGE_PAGE (Keyboard)
    0x19, 0x00, //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0xff, // 0x65, //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00, //   INPUT (Data,Ary,Abs)
    0xc0} // End Collection
};


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Data Types
// *****************************************************************************
// *****************************************************************************

/* This typedef defines the only INPUT report found in the HID report
 * descriptor and gives an easy way to create the OUTPUT report. */
typedef struct PACKED
{

  /* The union below represents the first byte of the INPUT report.  It is
   * formed by the following HID report items:
   *
   *  0x19, 0xe0, //   USAGE_MINIMUM (Keyboard LeftControl)
   *  0x29, 0xe7, //   USAGE_MAXIMUM (Keyboard Right GUI)
   *  0x15, 0x00, //   LOGICAL_MINIMUM (0)
   *  0x25, 0x01, //   LOGICAL_MAXIMUM (1)
   *  0x75, 0x01, //   REPORT_SIZE (1)
   *  0x95, 0x08, //   REPORT_COUNT (8)
   *  0x81, 0x02, //   INPUT (Data,Var,Abs)
   *
   * The report size is 1 specifying 1 bit per entry.
   * The report count is 8 specifying there are 8 entries.
   * These entries represent the Usage items between Left Control (the usage
   * minimum) and Right GUI (the usage maximum).
   */
  union PACKED
  {
    uint8_t value;

    struct PACKED
    {
      unsigned leftControl : 1;
      unsigned leftShift : 1;
      unsigned leftAlt : 1;
      unsigned leftGUI : 1;
      unsigned rightControl : 1;
      unsigned rightShift : 1;
      unsigned rightAlt : 1;
      unsigned rightGUI : 1;
    } bits;
  } modifiers;

  /* There is one byte of constant data/padding that is specified in the
   * input report:
   *
   *  0x95, 0x01,                    //   REPORT_COUNT (1)
   *  0x75, 0x08,                    //   REPORT_SIZE (8)
   *  0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
   */
  unsigned : 8;

  /* The last INPUT item in the INPUT report is an array type.  This array
   * contains an entry for each of the keys that are currently pressed until
   * the array limit, in this case 6 concurent key presses.
   *
   *  0x95, 0x06,                    //   REPORT_COUNT (6)
   *  0x75, 0x08,                    //   REPORT_SIZE (8)
   *  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
   *  0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
   *  0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
   *  0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
   *  0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
   *
   * Report count is 6 indicating that the array has 6 total entries.
   * Report size is 8 indicating each entry in the array is one byte.
   * The usage minimum indicates the lowest key value (Reserved/no event)
   * The usage maximum indicates the highest key value (Application button)
   * The logical minimum indicates the remapped value for the usage minimum:
   *   No Event has a logical value of 0.
   * The logical maximum indicates the remapped value for the usage maximum:
   *   Application button has a logical value of 101.
   *
   * In this case the logical min/max match the usage min/max so the logical
   * remapping doesn't actually change the values.
   *
   * To send a report with the 'a' key pressed (usage value of 0x04, logical
   * value in this example of 0x04 as well), then the array input would be the
   * following:
   *
   * LSB [0x04][0x00][0x00][0x00][0x00][0x00] MSB
   *
   * If the 'b' button was then pressed with the 'a' button still held down,
   * the report would then look like this:
   *
   * LSB [0x04][0x05][0x00][0x00][0x00][0x00] MSB
   *
   * If the 'a' button was then released with the 'b' button still held down,
   * the resulting array would be the following:
   *
   * LSB [0x05][0x00][0x00][0x00][0x00][0x00] MSB
   *
   * The 'a' key was removed from the array and all other items in the array
   * were shifted down. */
  uint8_t keys[6];
} KEYBOARD_INPUT_REPORT;

/* This typedef defines the only OUTPUT report found in the HID report
 * descriptor and gives an easy way to parse the OUTPUT report. */
typedef union PACKED
{
  /* The OUTPUT report is comprised of only one byte of data. */
  uint8_t value;

  struct
  {
    /* There are two report items that form the one byte of OUTPUT report
     * data.  The first report item defines 5 LED indicators:
     *
     *  0x95, 0x05,                    //   REPORT_COUNT (5)
     *  0x75, 0x01,                    //   REPORT_SIZE (1)
     *  0x05, 0x08,                    //   USAGE_PAGE (LEDs)
     *  0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
     *  0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
     *  0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
     *
     * The report count indicates there are 5 entries.
     * The report size is 1 indicating each entry is just one bit.
     * These items are located on the LED usage page
     * These items are all of the usages between Num Lock (the usage
     * minimum) and Kana (the usage maximum).
     */
    unsigned numLock : 1;
    unsigned capsLock : 1;
    unsigned scrollLock : 1;
    unsigned compose : 1;
    unsigned kana : 1;

    /* The second OUTPUT report item defines 3 bits of constant data
     * (padding) used to make a complete byte:
     *
     *  0x95, 0x01,                    //   REPORT_COUNT (1)
     *  0x75, 0x03,                    //   REPORT_SIZE (3)
     *  0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
     *
     * Report count of 1 indicates that there is one entry
     * Report size of 3 indicates the entry is 3 bits long. */
    unsigned : 3;
  } leds;
} KEYBOARD_OUTPUT_REPORT;

/* This creates a storage type for all of the information required to track the
 * current state of the keyboard. */
typedef struct
{
  USB_HANDLE lastINTransmission;
  USB_HANDLE lastOUTTransmission;
  uint8_t keys[6];
  uint8_t modifiers;
  uint8_t index;
} KEYBOARD;

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Variables
// *****************************************************************************
// *****************************************************************************
static KEYBOARD keyboard;

#if !defined(KEYBOARD_INPUT_REPORT_DATA_BUFFER_ADDRESS_TAG)
#define KEYBOARD_INPUT_REPORT_DATA_BUFFER_ADDRESS_TAG
#endif
static KEYBOARD_INPUT_REPORT inputReport KEYBOARD_INPUT_REPORT_DATA_BUFFER_ADDRESS_TAG;

#if !defined(KEYBOARD_OUTPUT_REPORT_DATA_BUFFER_ADDRESS_TAG)
#define KEYBOARD_OUTPUT_REPORT_DATA_BUFFER_ADDRESS_TAG
#endif
static volatile KEYBOARD_OUTPUT_REPORT outputReport KEYBOARD_OUTPUT_REPORT_DATA_BUFFER_ADDRESS_TAG;


// *****************************************************************************
// *****************************************************************************
// Section: Private Prototypes
// *****************************************************************************
// *****************************************************************************
static void APP_KeyboardProcessOutputReport(void);

// *****************************************************************************
// *****************************************************************************
// Section: Macros or Functions
// *****************************************************************************
// *****************************************************************************

void APP_KeyboardInit(void)
{
  //initialize the variable holding the handle for the last
  // transmission
  keyboard.lastINTransmission = 0;

  for (uint8_t i = 0; i < 6; i++) {
    keyboard.keys[i] = 0;
  }
  keyboard.index = 0;

  //enable the HID endpoint
  USBEnableEndpoint(HID_EP, USB_IN_ENABLED | USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);

  //Arm OUT endpoint so we can receive caps lock, num lock, etc. info from host
  keyboard.lastOUTTransmission = HIDRxPacket(HID_EP, (uint8_t*) & outputReport, sizeof (outputReport));
}

void APP_KeyboardTasks(void)
{

  /* If the USB device isn't configured yet, we can't really do anything
   * else since we don't have a host to talk to.  So jump back to the
   * top of the while loop. */
  if (USBGetDeviceState() < CONFIGURED_STATE) {
    return;
  }

  /* If we are currently suspended, then we need to see if we need to
   * issue a remote wakeup.  In either case, we shouldn't process any
   * keyboard commands since we aren't currently communicating to the host
   * thus just continue back to the start of the while loop. */
  if (USBIsDeviceSuspended() == true) {
    //Check if we should assert a remote wakeup request to the USB host,
    //when the user presses the pushbutton.
    if (UART_Data_Ready()) {
      //Add code here to issue a resume signal.
    }
    return;
  }

  /* Check if the IN endpoint is busy, and if it isn't check if we want to send
   * keystroke data to the host. */
  if (HIDTxHandleBusy(keyboard.lastINTransmission) == false) {
    /* Clear the INPUT report buffer.  Set to all zeros. */

    // read uart and process if any
    if (UART_Data_Ready()) {
      memset(&inputReport, 0, sizeof (inputReport));

      uint8_t scanCode;
      UART_Read(&scanCode);
      uint8_t sunKeyCode = scanCode & 0x7f;
      if (sunKeyCode != 0x7f) { // ignore idle messages
        uint8_t usbKeyCode = keytable[sunKeyCode];
        bool isPressed = (scanCode & 0x80) == 0x00;
        bool isModifier = (usbKeyCode & 0xe0) == 0xe0; // check if 0xE0 <= usbcode <= 0xE7 // was 0xf8
        if (isModifier) {
          uint8_t mask = (1 << (usbKeyCode & 0x07));

          if (isPressed)
            keyboard.modifiers |= mask;
          else
            keyboard.modifiers &= ~mask;

        }
        else { // it's  a normal key
          if (isPressed) {
            if (keyboard.index < 6) {
              keyboard.keys[keyboard.index] = usbKeyCode;
              keyboard.index++;
            }
            //keyboard.keys[0] = usbKeyCode;
          }
          else { // it's a key release
            // NKRO handling
            // remove the key if already in the list

            // keyboard.keys[0] = 0;
            uint8_t i;

            for (i = 0; i < keyboard.index; i++) {
              if (keyboard.keys[i] == usbKeyCode)
                break;
            }
            keyboard.keys[i] = 0;
            for (uint8_t j = i; j < keyboard.index; j++) {
              keyboard.keys[j] = keyboard.keys[j + 1];
            }
            keyboard.index--;
          }
        }

        /* Set the important data, the key press data. */
        for (uint8_t i = 0; i < 6; i++)
          inputReport.keys[i] = keyboard.keys[i];
        inputReport.modifiers.value = keyboard.modifiers;

        keyboard.lastINTransmission = HIDTxPacket(HID_EP, (uint8_t*) & inputReport, sizeof (inputReport));
      }
    }

  }//if(HIDTxHandleBusy(keyboard.lastINTransmission) == false)


  /* Check if any data was sent from the PC to the keyboard device.  Report
   * descriptor allows host to send 1 byte of data.  Bits 0-4 are LED states,
   * bits 5-7 are unused pad bits.  The host can potentially send this OUT
   * report data through the HID OUT endpoint (EP1 OUT), or, alternatively,
   * the host may try to send LED state information by sending a SET_REPORT
   * control transfer on EP0.  See the USBHIDCBSetReportHandler() function. */
  if (HIDRxHandleBusy(keyboard.lastOUTTransmission) == false) {
    APP_KeyboardProcessOutputReport();

    keyboard.lastOUTTransmission = HIDRxPacket(HID_EP, (uint8_t*) & outputReport, sizeof (outputReport));
  }

  return;
}

static void APP_KeyboardProcessOutputReport(void)
{
  uint8_t LEDMask = 0x00;
  if (outputReport.leds.capsLock) {
    LEDMask |= LED_CAPSLOCK;
  }
  if (outputReport.leds.numLock) {
    LEDMask |= LED_NUMLOCK;
  }
  if (outputReport.leds.scrollLock) {
    LEDMask |= LED_SCROLLLOCK;
  }
  if (outputReport.leds.compose) {
    LEDMask |= LED_COMPOSE;
  }
  uint8_t ledCmd[2] = {CMD_LED, LEDMask};
  UART_Write_Buf(ledCmd, 2);
}

static void USBHIDCBSetReportComplete(void)
{
  /* 1 byte of LED state data should now be in the CtrlTrfData buffer.  Copy
   * it to the OUTPUT report buffer for processing */
  outputReport.value = CtrlTrfData[0];

  /* Process the OUTPUT report. */
  APP_KeyboardProcessOutputReport();
}

void USBHIDCBSetReportHandler(void)
{
  /* Prepare to receive the keyboard LED state data through a SET_REPORT
   * control transfer on endpoint 0.  The host should only send 1 byte,
   * since this is all that the report descriptor allows it to send. */
  USBEP0Receive((uint8_t*) & CtrlTrfData, USB_EP0_BUFF_SIZE, USBHIDCBSetReportComplete);
}

/*******************************************************************************
 End of File
 */
