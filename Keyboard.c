/*
 * File:   Keyboard.c
 * Version 1.1.0
 * Copyright 2013 Tool Labs
 *
 * Change History
 *
 * Version 1.1.0
 *    - Supported multiple characters
 *
 * Version 1.0.0
 *    - Send preset character with modifier key code
 */

#ifndef KEYBOARD_C
#define KEYBOARD_C

//*********************
// Initialization Part
//*********************

// Include files
#include <xc.h>
#include "HardwareProfile.h"
#include "./USB/usb.h"
#include "./USB/usb_function_hid.h"
#include "Keyboard.h"

// Set configuration bits
// Automatically generated with MPLAB IDE
// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection Bits (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = ON    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection Bit (NO CPU system divide)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 4x     // PLL Multipler Selection Bit (4x Output Frequency Selected)
#pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// Prepare IN/OUT buffer for USB end point
#define IN_DATA_BUFFER_ADDRESS 0x2050
#define OUT_DATA_BUFFER_ADDRESS (IN_DATA_BUFFER_ADDRESS + HID_INT_IN_EP_SIZE)
#define IN_DATA_BUFFER_ADDRESS_TAG @IN_DATA_BUFFER_ADDRESS
#define OUT_DATA_BUFFER_ADDRESS_TAG @OUT_DATA_BUFFER_ADDRESS

// Allocate report in and out buffer
unsigned char   hid_report_in[HID_INT_IN_EP_SIZE] IN_DATA_BUFFER_ADDRESS_TAG;
unsigned char   hid_report_out[HID_INT_OUT_EP_SIZE] OUT_DATA_BUFFER_ADDRESS_TAG;

// Variables to handle multiple characters input
unsigned char   remainedSendChar;
unsigned char   currentSwitchNumber;
BOOL            needToSendNullKey;

// Default key code assign
// Please refer to page 53 to 59 of the following documentation for keyboard code
// http://www.usb.org/developers/devclass_docs/Hut1_11.pdf
// Followings are key modifier code
//    0x01: control key
//    0x02: shift key
//    0x04: option key
//    0x08: command key

// Number of switches
#define NumberOfSwitches 7

// Number of characters for each switch
unsigned char   numberOfChar[NumberOfSwitches]    = {1, 1, 1, 1, 1, 12, 13}; // number of characters for switch 1, 2, ... , 7

// Keyboard code for each switch
// Note that maxmum length is 30 bytes for each switch
unsigned char   keyCode[NumberOfSwitches][30]     = {   {0x1a}, // Switch 1 (w)
                                                        {0x2c}, // Switch 2 (space)
                                                        {0x2c}, // Switch 3 (space)
                                                        {0x19}, // Switch 4 (v)
                                                        {0x16}, // Switch 5 (s)
                                                        {0x0b, 0x08, 0x0f, 0x0f, 0x12, 0x2c, 0x1a, 0x12, 0x15, 0x0f, 0x07, 0x1e}, // Switch 6 ("Hello world!")
                                                        {0x09, 0x12, 0x12, 0x17, 0x2c, 0x0e, 0x08, 0x1c, 0x05, 0x12, 0x04, 0x15, 0x07}}; //Switch 7 ("Foot Keyboard")

// Modifier key code for each switche
unsigned char   keyModifier[NumberOfSwitches][30] = {   {0x08}, // Switch 1 (command)
                                                        {0x00}, // Switch 2 (normal)
                                                        {0x02}, // Switch 3 (shift)
                                                        {0x00}, // Switch 4 (normal)
                                                        {0x00}, // Switch 5 (normal)
                                                        {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}, // Switch 6
                                                        {0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; // Switch 7

// Variables to store each switch status
BYTE            old_sw1, old_sw2,old_sw3, old_sw4, old_sw5, old_sw6, old_sw7;

// Handles for usb data transmission status
USB_HANDLE      lastINTransmission;
USB_HANDLE      lastOUTTransmission;


// USB device controll prototype definitions
static void InitializeSystem(void);
void ProcessIO(void);
void Keyboard(void);
void USBCBSendResume(void);
void USBHIDCBSetReportComplete(void);
void SetSwitchData(void);

// Switch status confirmation prototype definitions
BOOL Switch1IsPressed(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
BOOL Switch4IsPressed(void);
BOOL Switch5IsPressed(void);
BOOL Switch6IsPressed(void);
BOOL Switch7IsPressed(void);

// USB callback prototype definitions
void USBCBSuspend(void);
void USBCBWakeFromSuspend(void);
void USBCB_SOF_Handler(void);
void USBCBErrorHandler(void);
void USBCBCheckOtherReq(void);
void USBCBStdSetDscHandler(void);
void USBCBInitEP(void);
void USBCBSendResume(void);



//*********************
//     Coding Part
//*********************
//
// main function
//
int main(void)
{
    // First initialize the system
    InitializeSystem();

    // Then do the following infinite loop
    while(1) {
        // Call USBDeviceTasks() to respond qurery from host and so on
        USBDeviceTasks();
    				  
        // Call ProcessIO() to process keyboard behavior
        ProcessIO();        
    }
}



//
// InitializeSystem function
//
static void InitializeSystem(void)
{
    // Set all pins to digital mode
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    // Set switch connected pins as input pins and others as output pins
    TRISA  = 0x00;
    TRISB  = 0xC0;
    TRISC  = 0xF0;

    // Set oscillator parameters
    OSCTUNE = 0;
    OSCCON = 0x3C;              // PLL enabled, 3x clock, 16MHz internal oscillator, SCS external
    OSCCONbits.SPLLMULT = 0;    // 4x
    ACTCON = 0x00;              // Clock recovery off, Clock Recovery enabled; SOF packet

    // Turn off the LED
    mLED_1_Off()

    //Initialize all of the push buttons
    old_sw1 = switch1;
    old_sw2 = switch2;
    old_sw3 = switch3;
    old_sw4 = switch4;
    old_sw5 = switch5;
    old_sw6 = switch6;
    old_sw7 = switch7;

    // Initialize the variable holding the handle for the last transmission
    lastINTransmission  = 0;
    lastOUTTransmission = 0;

    // Initialize USB module
    USBDeviceInit();

    // Initialize parameters for multiple characters
    remainedSendChar = 0;
    needToSendNullKey = FALSE;

    // Turn on the LED so that we can confirm the InitializeSystem() reached here
    mLED_1_On()
}


//
// ProcessIO function
//
void ProcessIO(void)
{
    // Call USBCBSendResume when switch5 is pressed
    // Ignore other keys
    if(switch5 == 0) {
        USBCBSendResume();
    }

    // Perform keyboard tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) {
        return;
    }

    Keyboard();
}


//
// Keyboard function
//
void Keyboard(void)
{
    // If USB transmitting handle is not busy, process the keyboard action
    if(!HIDTxHandleBusy(lastINTransmission)) {
        // Process unsent characters
        if(remainedSendChar) {
            if(needToSendNullKey) {
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                needToSendNullKey = FALSE;
            } else {
                hid_report_in[0] = keyModifier[currentSwitchNumber][numberOfChar[currentSwitchNumber] - remainedSendChar];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[currentSwitchNumber][numberOfChar[currentSwitchNumber] - remainedSendChar];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar--;
                needToSendNullKey = TRUE;
            }
        } else {
            // If no unsent character remained, check switch status and send corresponding charater to the host
            if(Switch1IsPressed()) {
                hid_report_in[0] = keyModifier[0][0];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[0][0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission  = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar    = numberOfChar[0] - 1;
                currentSwitchNumber = 0;
                needToSendNullKey   = TRUE;
            } else if(Switch2IsPressed()) {
                hid_report_in[0] = keyModifier[1][0];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[1][0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission  = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar    = numberOfChar[1] - 1;
                currentSwitchNumber = 1;
                needToSendNullKey   = TRUE;
            } else if(Switch3IsPressed()) {
                hid_report_in[0] = keyModifier[2][0];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[2][0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission  = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar    = numberOfChar[2] - 1;
                currentSwitchNumber = 2;
                needToSendNullKey   = TRUE;
            } else if(Switch4IsPressed()) {
                hid_report_in[0] = keyModifier[3][0];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[3][0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission  = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar    = numberOfChar[3] - 1;
                currentSwitchNumber = 3;
                needToSendNullKey   = TRUE;
            } else if(Switch5IsPressed()) {
                hid_report_in[0] = keyModifier[4][0];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[4][0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission  = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar    = numberOfChar[4] - 1;
                currentSwitchNumber = 4;
                needToSendNullKey   = TRUE;
            } else if(Switch6IsPressed()) {
                hid_report_in[0] = keyModifier[5][0];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[5][0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission  = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar    = numberOfChar[5] - 1;
                currentSwitchNumber = 5;
                needToSendNullKey   = TRUE;
            } else if(Switch7IsPressed()) {
                hid_report_in[0] = keyModifier[6][0];
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode[6][0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission  = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                remainedSendChar    = numberOfChar[6] - 1;
                currentSwitchNumber = 6;
                needToSendNullKey   = TRUE;
            } else {
                // No keyboard action
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            }
        }
    }

    // If USB receiving handle is not busy, process the data reading action
    if(!HIDRxHandleBusy(lastOUTTransmission)) {
        switch(hid_report_out[0]) {
            case 0x01:
                SetSwitchData();
                break;
            case 0x10:
                mLED_1_Toggle()
                break;
            default:
                break;
        }
//        if(hid_report_out[0] == 0x02) {
//            mLED_1_Toggle()
//        }
        lastOUTTransmission = HIDRxPacket(HID_EP,(BYTE*)&hid_report_out,HID_INT_OUT_EP_SIZE);
    }
    
    return;		
}


//
// Switch status check function for switch1 through switch5
//
BOOL Switch1IsPressed(void)
{
    if(switch1 != old_sw1) {
        old_sw1 = switch1;
        if(switch1 == 0) {
            return TRUE;
        }
    }
    return FALSE;
}


BOOL Switch2IsPressed(void)
{
    if(switch2 != old_sw2) {
        old_sw2 = switch2;
        if(switch2 == 0) {
            return TRUE;
        }
    }
    return FALSE;
}

BOOL Switch3IsPressed(void)
{
    if(switch3 != old_sw3) {
        old_sw3 = switch3;
        if(switch3 == 0) {
            return TRUE;
        }
    }
    return FALSE;
}

BOOL Switch4IsPressed(void)
{
    if(switch4 != old_sw4) {
        old_sw4 = switch4;
        if(switch4 == 0) {
            return TRUE;
        }
    }
    return FALSE;
}

BOOL Switch5IsPressed(void)
{
    if(switch5 != old_sw5) {
        old_sw5 = switch5;
        if(switch5 == 0) {
            return TRUE;
        }
    }
    return FALSE;
}

BOOL Switch6IsPressed(void)
{
    if(switch6 != old_sw6) {
        old_sw6 = switch6;
        if(switch6 == 0) {
            return TRUE;
        }
    }
    return FALSE;
}

BOOL Switch7IsPressed(void)
{
    if(switch7 != old_sw7) {
        old_sw7 = switch7;
        if(switch7 == 0) {
            return TRUE;
        }
    }
    return FALSE;
}

//
// Set key code data for specified siwtch
//
// Contents of hid_report_out[]
//   0: command 0x01
//   1: switch number
//   2: data length
//   3: character set position
//   4, 6: key code
//   5, 7: modifier code
void SetSwitchData(void)
{
    unsigned char switchNumber;
    unsigned char charLocation;

    // Get switch number
    switchNumber = hid_report_out[1];

    // Get modifying location
    charLocation = hid_report_out[3];

    // Store number of character for the specified switch
    numberOfChar[switchNumber] = hid_report_out[2];

    // Store the key code and modifier code
    keyCode[switchNumber][charLocation] = hid_report_out[4];
    keyModifier[switchNumber][charLocation] = hid_report_out[6];
    // If some character code is set to the second character i.e. hid_report_out[5]
    // set the second character
    if(hid_report_out[5]){
        keyCode[switchNumber][charLocation+1] = hid_report_out[5];
        keyModifier[switchNumber][charLocation+1] = hid_report_out[7];
    }
}



//
// USB Callback handler
//
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch( event ) {
        case EVENT_TRANSFER:
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}


//
// USB Callback functions
//
void USBCBSuspend(void)
{
    // Do nothing
}

void USBCBWakeFromSuspend(void)
{
    // Do nothing
}

void USBCB_SOF_Handler(void)
{
    // Do nothing
}

void USBCBErrorHandler(void)
{
    // Do nothing
}

void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}

void USBCBStdSetDscHandler(void)
{
    // Do nothing
}

void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Arm OUT endpoint so we can receive caps lock, num lock, etc. info from host
    lastOUTTransmission = HIDRxPacket(HID_EP,(BYTE*)&hid_report_out,HID_INT_OUT_EP_SIZE);
}

void USBCBSendResume(void)
{
    static WORD delay_count;

    if(USBGetRemoteWakeupStatus() == TRUE) {
        if(USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();
            
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;
            delay_count = 3600U;        

            do {
                delay_count--;
            } while(delay_count);
            
            USBResumeControl = 1;
            delay_count = 1800U;

            do {
                delay_count--;
            } while(delay_count);

            USBResumeControl = 0;
            USBUnmaskInterrupts();
        }
    }
}


//
// USBHIDCBSetReportHandler
//
void USBHIDCBSetReportHandler(void)
{
    USBEP0Receive((BYTE*)&CtrlTrfData, USB_EP0_BUFF_SIZE, USBHIDCBSetReportComplete);
}

void USBHIDCBSetReportComplete(void)
{
    switch(CtrlTrfData[0]) {
        case 0x01:
            SetSwitchData();
            break;
        case 0x10:
            mLED_1_Toggle()
            break;
        default:
            break;
    }
//	if(CtrlTrfData[0] == 0x02) {
//          mLED_1_Toggle()
//	}
}

#endif
