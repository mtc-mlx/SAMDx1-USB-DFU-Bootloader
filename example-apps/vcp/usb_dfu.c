/*
 * Copyright (c) 2017, Alex Taradov <alex@taradov.com>
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

/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include "samd11.h"
#include "utils.h"
#include "usb.h"
#include "usb_std.h"
#include "usb_dfu.h"

/*- Prototypes --------------------------------------------------------------*/

/*- Variables ---------------------------------------------------------------*/
static volatile dfu_getstatus_response_t dfu_getstatus_response = {
  .bStatus = DFU_STATUS_OK,
  .bwPollTimeout = 0,
  .bState = DFU_STATE_APP_IDLE,
  .iString = 0
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
bool usb_dfu_class_handle_request(usb_request_t *request)
{
  int length = request->wLength;

  switch ((request->bRequest << 8) | request->bmRequestType)
  {
    case USB_CMD(OUT, INTERFACE, CLASS, DFU_REQUEST_DETACH):
      usb_control_send_zlp();
      /* Assume bl_info.rcause_mask includes SYST bit */
      NVIC_SystemReset();
      break;
    case USB_CMD(IN, INTERFACE, CLASS, DFU_REQUEST_GETSTATUS):
      usb_control_send((uint8_t *)&dfu_getstatus_response, LIMIT(length, sizeof(dfu_getstatus_response)));
      break;
    default:
      return false;
  }
  return true;
}
