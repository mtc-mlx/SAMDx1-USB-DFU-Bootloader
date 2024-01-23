/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _USB_H_
#define _USB_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "utils.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  USB_GET_STATUS        = 0,
  USB_CLEAR_FEATURE     = 1,
  USB_SET_FEATURE       = 3,
  USB_SET_ADDRESS       = 5,
  USB_GET_DESCRIPTOR    = 6,
  USB_SET_DESCRIPTOR    = 7,
  USB_GET_CONFIGURATION = 8,
  USB_SET_CONFIGURATION = 9,
  USB_GET_INTERFACE     = 10,
  USB_SET_INTERFACE     = 11,
  USB_SYNCH_FRAME       = 12,
};

enum
{
  DFU_REQUEST_DETACH            = 0,
  DFU_REQUEST_DNLOAD            = 1,
  DFU_REQUEST_UPLOAD            = 2,
  DFU_REQUEST_GETSTATUS         = 3,
  DFU_REQUEST_CLRSTATUS         = 4,
  DFU_REQUEST_GETSTATE          = 5,
  DFU_REQUEST_ABORT             = 6,
};

enum
{
  DFU_STATE_APP_IDLE                      = 0,
  DFU_STATE_APP_DETACH                    = 1,
  DFU_STATE_DFU_IDLE                      = 2,
  DFU_STATE_DFU_DNLOAD_SYNC               = 3,
  DFU_STATE_DFU_DNLOAD_BUSY               = 4,
  DFU_STATE_DFU_DNLOAD_IDLE               = 5,
  DFU_STATE_DFU_MANIFEST_SYNC             = 6,
  DFU_STATE_DFU_MANIFEST                  = 7,
  DFU_STATE_DFU_MANIFEST_WAIT_RESET       = 8,
  DFU_STATE_DFU_UPLOAD_IDLE               = 9,
  DFU_STATE_DFU_ERROR                     = 10,
  DFU_STATE_DFU_DETACH                    = 15,
};

enum
{
  DFU_STATUS_OK                           = 0,
  DFU_STATUS_ERR_TARGET                   = 1,
  DFU_STATUS_ERR_FILE                     = 2,
  DFU_STATUS_ERR_WRITE                    = 3,
  DFU_STATUS_ERR_ERASE                    = 4,
  DFU_STATUS_ERR_CHECK_ERASED             = 5,
  DFU_STATUS_ERR_PROG                     = 6,
  DFU_STATUS_ERR_VERIFY                   = 7,
  DFU_STATUS_ERR_ADDRESS                  = 8,
  DFU_STATUS_ERR_NOTDONE                  = 9,
  DFU_STATUS_ERR_FIRMWARE                 = 10,
  DFU_STATUS_ERR_VENDOR                   = 11,
  DFU_STATUS_ERR_USBR                     = 12,
  DFU_STATUS_ERR_POR                      = 13,
  DFU_STATUS_ERR_UNKNOWN                  = 14,
  DFU_STATUS_ERR_STALLEDPKT               = 15,
};

enum
{
  USB_DEVICE_DESCRIPTOR                    = 1,
  USB_CONFIGURATION_DESCRIPTOR             = 2,
  USB_STRING_DESCRIPTOR                    = 3,
  USB_INTERFACE_DESCRIPTOR                 = 4,
  USB_ENDPOINT_DESCRIPTOR                  = 5,
  USB_DEVICE_QUALIFIER_DESCRIPTOR          = 6,
  USB_OTHER_SPEED_CONFIGURATION_DESCRIPTOR = 7,
  USB_INTERFACE_POWER_DESCRIPTOR           = 8,
  USB_OTG_DESCRIPTOR                       = 9,
  USB_DEBUG_DESCRIPTOR                     = 10,
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR     = 11,
  USB_BINARY_OBJECT_STORE_DESCRIPTOR       = 15,
  USB_DEVICE_CAPABILITY_DESCRIPTOR         = 16,
};

enum
{
  USB_DEVICE_RECIPIENT     = 0,
  USB_INTERFACE_RECIPIENT  = 1,
  USB_ENDPOINT_RECIPIENT   = 2,
  USB_OTHER_RECIPIENT      = 3,
};

enum
{
  USB_STANDARD_REQUEST     = 0,
  USB_CLASS_REQUEST        = 1,
  USB_VENDOR_REQUEST       = 2,
};

enum
{
  USB_OUT_TRANSFER         = 0,
  USB_IN_TRANSFER          = 1,
};

enum
{
  USB_IN_ENDPOINT          = 0x80,
  USB_OUT_ENDPOINT         = 0x00,
  USB_INDEX_MASK           = 0x7f,
  USB_DIRECTION_MASK       = 0x80,
};

enum
{
  USB_CONTROL_ENDPOINT     = 0 << 0,
  USB_ISOCHRONOUS_ENDPOINT = 1 << 0,
  USB_BULK_ENDPOINT        = 2 << 0,
  USB_INTERRUPT_ENDPOINT   = 3 << 0,

  USB_NO_SYNCHRONIZATION   = 0 << 2,
  USB_ASYNCHRONOUS         = 1 << 2,
  USB_ADAPTIVE             = 2 << 2,
  USB_SYNCHRONOUS          = 3 << 2,

  USB_DATA_ENDPOINT        = 0 << 4,
  USB_FEEDBACK_ENDPOINT    = 1 << 4,
  USB_IMP_FB_DATA_ENDPOINT = 2 << 4,
};

/*- Types -------------------------------------------------------------------*/
typedef union
{
  struct PACK
  {
    uint8_t   bmRequestType;
    uint8_t   bRequest;
    uint16_t  wValue;
    uint16_t  wIndex;
    uint16_t  wLength;
  };
  struct PACK
  {
    uint16_t    wRequestTypeAndRequest;
    uint32_t    dwValueAndIndex;
  };
  uint32_t dwRequestTypeAndRequestAndValue;
} usb_request_t;

typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  bcdUSB;
  uint8_t   bDeviceClass;
  uint8_t   bDeviceSubClass;
  uint8_t   bDeviceProtocol;
  uint8_t   bMaxPacketSize0;
  uint16_t  idVendor;
  uint16_t  idProduct;
  uint16_t  bcdDevice;
  uint8_t   iManufacturer;
  uint8_t   iProduct;
  uint8_t   iSerialNumber;
  uint8_t   bNumConfigurations;
} usb_device_descriptor_t;

typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  wTotalLength;
  uint8_t   bNumInterfaces;
  uint8_t   bConfigurationValue;
  uint8_t   iConfiguration;
  uint8_t   bmAttributes;
  uint8_t   bMaxPower;
} usb_configuration_descriptor_t;

typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bInterfaceNumber;
  uint8_t   bAlternateSetting;
  uint8_t   bNumEndpoints;
  uint8_t   bInterfaceClass;
  uint8_t   bInterfaceSubClass;
  uint8_t   bInterfaceProtocol;
  uint8_t   iInterface;
} usb_interface_descriptor_t;

typedef union
{
  struct PACK
  {
    uint32_t  bStatus       : 8;
    uint32_t  bwPollTimeout : 24;
    uint8_t   bState;
    uint8_t   iString;
  };
  struct
  {
    uint32_t dwStatusAndPollTimeout;
    uint32_t dwStateAndString;
  };
} dfu_getstatus_response_t;


/*- Prototypes --------------------------------------------------------------*/

#endif // _USB_H_
