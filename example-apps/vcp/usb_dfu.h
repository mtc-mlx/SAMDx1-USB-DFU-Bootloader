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

#ifndef _USB_DFU_H_
#define _USB_DFU_H_

/*- Includes ----------------------------------------------------------------*/
#include "usb.h"
#include "utils.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  USB_DFU_REQUEST_DETACH            = 0,
  USB_DFU_REQUEST_DNLOAD            = 1,
  USB_DFU_REQUEST_UPLOAD            = 2,
  USB_DFU_REQUEST_GETSTATUS         = 3,
  USB_DFU_REQUEST_CLRSTATUS         = 4,
  USB_DFU_REQUEST_GETSTATE          = 5,
  USB_DFU_REQUEST_ABORT             = 6,
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

/*- Types -------------------------------------------------------------------*/
typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bmAttributes;
  uint16_t  wDetachTimeout;
  uint16_t  wTransferSize;
  uint16_t  bcdDFUVersion;
} usb_dfu_functional_descriptor_t;

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
bool usb_dfu_class_handle_request(usb_request_t *request);

#endif // _USB_DFU_H_

