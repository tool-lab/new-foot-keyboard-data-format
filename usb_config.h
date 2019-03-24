/*
 * File:   usb_config.h
 * Copyright 2013 Tool Labs
 */

#ifndef USBCFG_H
#define USBCFG_H

// Endpoint
#define USB_EP0_BUFF_SIZE       8
#define USB_MAX_NUM_INT         1
#define USB_MAX_EP_NUMBER       1

// USB ping pong mode
#define USB_PING_PONG_MODE USB_PING_PONG__FULL_PING_PONG

// Use USB Polling
#define USB_POLLING

// Use USB Pullup
#define USB_PULLUP_OPTION USB_PULLUP_ENABLE

// Use USB internal tranceiver
#define USB_TRANSCEIVER_OPTION USB_INTERNAL_TRANSCEIVER

// Set Full speed
#define USB_SPEED_OPTION USB_FULL_SPEED

// Define Vendor ID and Product ID
#define MY_VID 0x04D8
#define MY_PID 0x0055

// Timeout settings
#define USB_ENABLE_STATUS_STAGE_TIMEOUTS
#define USB_STATUS_STAGE_TIMEOUT            (BYTE)45

// Define USB_SUPPORT_DEVICE
#define USB_SUPPORT_DEVICE

// Define number of descriptors
#define USB_NUM_STRING_DESCRIPTORS 3

// Define device class as Human Interface Device
#define USB_USE_HID

// HID definition
#define HID_INTF_ID             0x00
#define HID_EP                  1
#define HID_INT_OUT_EP_SIZE     8
#define HID_INT_IN_EP_SIZE      8
#define HID_NUM_OF_DSC          1
#define HID_RPT01_SIZE          63
#define USER_SET_REPORT_HANDLER USBHIDCBSetReportHandler	

#endif //USBCFG_H
