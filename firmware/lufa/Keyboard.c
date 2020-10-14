/*
             LUFA Library
     Copyright (C) Dean Camera, 2019.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2019  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Keyboard demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */
#include "SoftwareSerial.h"
#include "Keyboard.h"
#include "sunkeytable.h"

#define KEY 7

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

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

static sserial_t *ssSerial;

static uint8_t modifierMask = 0x00;
uint8_t lastKey = 0x00;

void ResetKbd(void);
void BeepKbd(void);

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber = INTERFACE_ID_Keyboard,
				.ReportINEndpoint =
					{
						.Address = KEYBOARD_EPADDR,
						.Size = KEYBOARD_EPSIZE,
						.Banks = 1,
					},
				.PrevReportINBuffer = PrevKeyboardHIDReportBuffer,
				.PrevReportINBufferSize = sizeof(PrevKeyboardHIDReportBuffer),
			},
};

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{

	ssSerial = ss_create(true);

	SetupHardware();

	GlobalInterruptEnable();

	ss_begin(ssSerial, 1200);

	//while (! ss_available(ssSerial)) ; // wait for boot sequence

	// swallow boot sequence
	ss_read(ssSerial);
	ss_read(ssSerial);
	ss_read(ssSerial);

	for (;;)
	{
		HID_Device_USBTask(&Keyboard_HID_Interface);
		USB_USBTask();
	}

	ss_free(ssSerial);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware()
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	USB_Init();
}

void ResetKbd(void)
{
	modifierMask = 0;
	lastKey = 0;

	// RESET the keyboard
	ss_write(ssSerial, CMD_RESET);
	// swallow the response
	ss_read(ssSerial);
	ss_read(ssSerial);
	ss_read(ssSerial);
}

void BeepKbd(void)
{
	// TODO: send led state to keyboard
	ss_write(ssSerial, CMD_BELL_ON);
	_delay_ms(100);
	ss_write(ssSerial, CMD_BELL_OFF);
	_delay_ms(400);
	ss_write(ssSerial, CMD_BELL_ON);
	_delay_ms(100);
	ss_write(ssSerial, CMD_BELL_OFF);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	// // reset the keyboard
	//ResetKbd();
	BeepKbd();
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{

	// // reset the keyboard
	//ResetKbd();
	BeepKbd();
}

void EVENT_USB_Device_Reset(void)
{
	// // reset the keyboard
	//ResetKbd();
	// better not reset the keyboard. This make it lose all the state.
}

void EVENT_USB_UIDChange(void)
{
	//ResetKbd();
	BeepKbd();
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);

	USB_Device_EnableSOFEvents();

	// // reset the keyboard
//	ResetKbd();
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

bool HandleModifier(USB_KeyboardReport_Data_t *KeyboardReport, uint8_t usbKeyCode, bool isReleased)
{

	if ((usbKeyCode & 0xf8) != 0xe0) // check if 0xe0 <= usbcode <= 0xe7
		return false;

	uint8_t mask = (1 << (usbKeyCode & 0x07));

	if (isReleased)
		modifierMask &= ~mask;
	else
		modifierMask |= mask;

	return true;
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo,
										 uint8_t *const ReportID,
										 const uint8_t ReportType,
										 void *ReportData,
										 uint16_t *const ReportSize)
{
	USB_KeyboardReport_Data_t *KeyboardReport = (USB_KeyboardReport_Data_t *)ReportData;

	// send nothing
	*ReportSize = 0;

	//	unsigned char byte1, byte2, byte3;
	if (ss_available(ssSerial))
	{
		uint8_t scanCode = ss_read(ssSerial);
		uint8_t sunKeyCode = scanCode & 0x7f;
		// They keyboard is telling us it's not idle
		if (sunKeyCode != 0x7f)
		{
			bool isReleased = (scanCode & 0x80) == 0x80;
			uint8_t usbKeyCode = keytable[sunKeyCode];

			bool wasMod = HandleModifier(KeyboardReport, usbKeyCode, isReleased);

			KeyboardReport->Modifier = modifierMask;

			if (!wasMod)
				lastKey = isReleased ? 0x00 : usbKeyCode;

			KeyboardReport->KeyCode[0] = lastKey;

			// Set the report size, so the data will be sent
			*ReportSize = sizeof(USB_KeyboardReport_Data_t);
		}
	}

	return true;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo,
										  const uint8_t ReportID,
										  const uint8_t ReportType,
										  const void *ReportData,
										  const uint16_t ReportSize)
{
	uint8_t LEDMask = 0x00;
	uint8_t *LEDReport = (uint8_t *)ReportData;

	if (*LEDReport & HID_KEYBOARD_LED_NUMLOCK)
		LEDMask |= LED_NUMLOCK;

	if (*LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
		LEDMask |= LED_CAPSLOCK;

	if (*LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
		LEDMask |= LED_SCROLLLOCK;

	if (*LEDReport & HID_KEYBOARD_LED_COMPOSE)
		LEDMask |= LED_COMPOSE;

	uint8_t ledCmd[2] = {CMD_LED, LEDMask};
	// TODO: send led state to keyboard
	ss_write_bytes(ssSerial, ledCmd, 2);
}
