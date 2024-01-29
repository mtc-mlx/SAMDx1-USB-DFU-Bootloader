/*
 * 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
 * Copyright (c) 2018-2020, Peter Lawrence
 * Copyright (c) 2024 Melexis Inc.
 * derived from https://github.com/ataradov/vcp Copyright (c) 2016, Alex Taradov <alex@taradov.com>
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

/* 
NOTES:
- anything pointed to by udc_mem[*].*.ADDR.reg *MUST* BE IN RAM and be 32-bit aligned... no exceptions
*/


/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <sam.h>
#include "usb.h"
#include "nvm_data.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/

/* Support double-tap /RESET bootloader entry. The EXT bit will be set in
 * bl_info.rcause_mask after the first external reset. The application can
 * clear the bit after a timer. If the bit is still set and the device
 * receives another external reset, the bootloader will begin DFU operations.
 */
#define USE_DOUBLE_TAP

/* Enables entering the bootloader by grounding a GPIO (PA15 by default).
 * Uncomment to enable.
 */
//#define USE_GPIO

/* Enables handling the watchdog, in case it is enabled via the NVM user row.
 * Only non-window operation is supported.
 */
//#define HANDLE_WATCHDOG

/* Enable extra DFU commands and the get-interface request. These are minor
 * features required for strict compliance and don't fit in 1k of flash,
 * so they are only here for completeness.
 */
//#define ENABLE_OPTIONAL_COMMANDS

/* Utility macro for standard USB device requests */
#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    UsbDeviceDescBank  out;
    UsbDeviceDescBank  in;
} udc_mem_t;

typedef struct
{
  uint32_t reserved2;
  uint32_t reserved1;
  uint32_t reserved0;

  /* This value is ANDed with the RCAUSE register. If it matches (ignoring
   * POR, BOD12, and BOD33 bits), we begin DFU operations. By default,
   * a software reset (SYST) or a watchdog reset (WDT) are enabled and external
   * reset (EXT) is masked.
   *
   * If USE_DOUBLE_TAP is defined, EXT is unmasked after the first external
   * reset. The application is then free to mask it again after some delay.
   * If EXT is still unmasked and the user presses the reset button again,
   * the bootloader will begin DFU operations.
   */
  uint32_t rcause_mask;
} bl_info_t;

/*- Variables ---------------------------------------------------------------*/
static uint32_t usb_config = 0;
#ifdef ENABLE_OPTIONAL_COMMANDS
static uint32_t usb_interface = 0;
#endif
static dfu_getstatus_response_t dfu_getstatus_response;
static udc_mem_t udc_mem[USB_EPT_NUM];
static uint32_t udc_control_out_buf[16];
static uint32_t dfu_block_num;
static uint32_t expected_block_length;

/* Start address for the next flash write */
static uint32_t *nvm_tail_ptr;

static volatile bl_info_t __attribute__((section(".bl_info"))) bl_info;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) udc_control_send(const uint32_t *data, uint32_t size)
{
  /* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
  udc_mem[0].in.ADDR.reg = (uint32_t)data;

  udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(size) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

  /* Signal ready to respond to any IN packets; we can't stall OUT tokens
   * since they are used for the status response.
   */
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;
}

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) udc_control_send_positive_status(void)
{
  /* Since this is the status transaction, there is no reason the host should
   * send an OUT token. Stall that endpoint to make sure.
   */
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
  udc_control_send(NULL, 0); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}

static void __attribute__((noinline)) dfu_stall_invalid_request(void)
{
  dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_ERROR;
  dfu_getstatus_response.dwStatusAndPollTimeout = DFU_STATUS_ERR_STALLEDPKT;
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0 | USB_DEVICE_EPSTATUSSET_STALLRQ1;
}

//-----------------------------------------------------------------------------
static inline void __attribute__((always_inline)) USB_Service(void)
{
  uint32_t flags;

  /* Check and clear device interrupt flags */
  flags = USB->DEVICE.INTFLAG.reg;
  USB->DEVICE.INTFLAG.reg = flags;

  if (flags & USB_DEVICE_INTFLAG_EORST) /* End Of Reset */
  {
    /* Technically, the DFU spec requires resetting into the application
     * firmware when receiving a USB reset. (Almost) no one does this because
     * most OSes send one or more resets during enumeration. Instead,
     * we implement the DFU_DETACH command; see below.
     */

    /* No need to initialize IN descriptor; it will be initialized in
     * udc_control_send() and not fetched until STALLRQ1 is cleared and BK1RDY
     * is set and an IN token is received.
     */
    udc_mem[0].out.ADDR.reg = (uint32_t)udc_control_out_buf;
    udc_mem[0].out.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(64) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1 /*CONTROL*/) | USB_DEVICE_EPCFG_EPTYPE1(1 /*CONTROL*/);
    /* Return STALL until we receive a valid SETUP transaction. No need to
     * initialize BK0RDY/BK1RDY; they are ignored for and reset by the STATUS
     * transaction.
     */
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0 | USB_DEVICE_EPSTATUSSET_STALLRQ1;
  }

  /* Check and clear endpoint interrupt flags
   *
   * NOTE: clearing the RXSTP bit allows another SETUP transaction to be
   * received. That transaction could unexpectedly overwrite the data received
   * during a control write operation. This is only a problem if the host
   * misbehaves and does not wait for the ZLP status-stage response.
   */
  flags = USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg;
  USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = flags;

  if (flags & USB_DEVICE_EPINTFLAG_TRCPT0) /* Transmit Complete 0 */
  {
    /* An OUT transaction forms the status stage of a control read.
     * If the host's ACK was corrupted on the final packet of the data stage,
     * we won't get the TRCPT1 interrupt although the transfer did actually
     * complete. So if BK1RDY is still set, we set STALLRQ1 to block non-SETUP
     * transactions.
     *
     * NOTE: clearing BK1RDY is not really necessary, since STALLRQ1 takes
     * precedence and BK1RDY will be cleared by receipt of a SETUP transaction
     * anyway.
     */
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg >> 2;

    if (dfu_getstatus_response.dwStateAndString == DFU_STATE_DFU_DNLOAD_SYNC)
    {
      if (udc_mem[0].out.PCKSIZE.bit.BYTE_COUNT != expected_block_length)
      {
        /* Host didn't send the same number of bytes that it promised */
        dfu_stall_invalid_request();
      }
      else
      {
        uint32_t *buf_ptr = (uint32_t *)&udc_control_out_buf[0];
        for (uint32_t i = 0; i < expected_block_length; i += 4)
        {
          /* If nvm_tail_ptr points to the first word in the 256-byte row,
           * erase the row first.
           */
          if (!((uint32_t)nvm_tail_ptr << 24))
          {
            NVMCTRL->ADDR.reg = (uint32_t)nvm_tail_ptr >> 1;
            NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
          }
          /* Wait for row erase to finish */
          while (!NVMCTRL->INTFLAG.reg);

          *nvm_tail_ptr++ = *buf_ptr++;
        }
        /* Wait for page write to finish */
        while (!NVMCTRL->INTFLAG.reg);

        udc_control_send_positive_status();
      }
    }
  }

  if (flags & USB_DEVICE_EPINTFLAG_TRCPT1)
  {
    /* Transmit was complete and we signaled it with a packet smaller than
     * bMaxPacketSize0 (always true since our descriptors are <64 bytes.)
     * Any further IN tokens are an error.
     */
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
    USB->DEVICE.DADD.reg |= USB_DEVICE_DADD_ADDEN;

    if (dfu_getstatus_response.dwStateAndString == DFU_STATE_DFU_DETACH)
    {
      /* Request system reset */
      SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
                    SCB_AIRCR_SYSRESETREQ_Msk);
    }
  }

  if (flags & USB_DEVICE_EPINTFLAG_RXSTP) /* Received Setup */
  {
    /* Subsequent OUT and IN tokens will be NAK'd until we're ready,
     * since BK0RDY and BK1RDY were automatically set and cleared, respectively.
     */

    usb_request_t *request = (usb_request_t *)udc_control_out_buf;
    uint16_t length = request->wLength;
    uint16_t value = request->wValue;
    uint32_t request_type_and_request_and_value = request->dwRequestTypeAndRequestAndValue;

    switch (request_type_and_request_and_value)
    {
      case USB_CMD(IN, DEVICE, STANDARD) | (USB_GET_DESCRIPTOR << 8) | (USB_DEVICE_DESCRIPTOR << 24) | (0 /* Descriptor index */ << 16):
        udc_control_send((uint32_t *)&usb_device_descriptor, length);
        break;
      case USB_CMD(IN, DEVICE, STANDARD) | (USB_GET_DESCRIPTOR << 8) | (USB_CONFIGURATION_DESCRIPTOR << 24) | (0 /* Descriptor index */ << 16):
        udc_control_send((uint32_t *)&usb_configuration_hierarchy, length);
        break;
      case USB_CMD(IN, DEVICE, STANDARD) | (USB_GET_CONFIGURATION << 8):
        udc_control_send(&usb_config, 1);
        break;
#ifdef ENABLE_OPTIONAL_COMMANDS
      case USB_CMD(IN, INTERFACE, STANDARD) | (USB_GET_INTERFACE << 8):
        udc_control_send(&usb_interface, 1);
        break;
      case USB_CMD(IN, INTERFACE, CLASS) | (DFU_REQUEST_GETSTATE << 8):
        udc_control_send((uint32_t *)&dfu_getstatus_response.dwStateAndString, 1);
        break;
#endif
      case USB_CMD(IN, INTERFACE, CLASS) | (DFU_REQUEST_GETSTATUS << 8):
        switch (dfu_getstatus_response.dwStateAndString)
        {
          case DFU_STATE_DFU_DNLOAD_SYNC:
            dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_DNLOAD_IDLE;
            break;
          case DFU_STATE_DFU_MANIFEST_SYNC:
            dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_MANIFEST;
            break;
          case DFU_STATE_DFU_MANIFEST:
            dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_IDLE;
            break;
          default:
            break;
        }
        udc_control_send((uint32_t *)&dfu_getstatus_response, 6);
        break;
      case USB_CMD(OUT, INTERFACE, CLASS) | (DFU_REQUEST_CLRSTATUS << 8):
        if (dfu_getstatus_response.dwStateAndString == DFU_STATE_DFU_ERROR)
        {
          dfu_getstatus_response.dwStatusAndPollTimeout = DFU_STATUS_OK;
          dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_IDLE;
          udc_control_send_positive_status();
        }
        else
        {
          dfu_stall_invalid_request();
        }
        break;
#ifdef ENABLE_OPTIONAL_COMMANDS
      case USB_CMD(OUT, INTERFACE, CLASS) | (DFU_REQUEST_ABORT << 8):
        if (dfu_getstatus_response.dwStateAndString == DFU_STATE_DFU_DNLOAD_IDLE || dfu_getstatus_response.dwStateAndString == DFU_STATE_DFU_IDLE)
        {
          dfu_getstatus_response.dwStatusAndPollTimeout = DFU_STATUS_OK;
          dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_IDLE;
          udc_control_send_positive_status();
        }
        else
        {
          dfu_stall_invalid_request();
        }
        break;
#endif
      case USB_CMD(OUT, DEVICE, STANDARD) | (USB_SET_CONFIGURATION << 8) | (0 /* Configuration index */ << 16):
      case USB_CMD(OUT, DEVICE, STANDARD) | (USB_SET_CONFIGURATION << 8) | (1 /* Configuration index */ << 16):
        usb_config = value;
        udc_control_send_positive_status();
        break;
      default:
        request_type_and_request_and_value <<= 16;
        switch (request_type_and_request_and_value)
        {
          /* Technically, the detach request is only intended to be implemented
           * in the application and not by the bootloader. However, strictly
           * following the DFU specification doesn't provide any good way for
           * the host to request the bootloader to restart into the application.
           * (This is supposed to be done on any USB reset from the host---
           * impractical since OSes can send any number of resets during
           * enumeration.) Implementing the detach request follows the example
           * of OpenMoko and dfu-util without breaking support for other tools.
           *
           * Some DFU host implementations (e.g. dfu-util) send bogus timeout
           * values for the detach command even though the spec says the
           * timeout should be no greater than the wDetachTimeout in our
           * DFU functional descriptor (zero in our case). Therefore, we
           * ignore wValue and match the command here. The timeout is
           * meaningless anyway when bitWillDetach=1.
           */
          case (USB_CMD(OUT, INTERFACE, CLASS) | (DFU_REQUEST_DETACH << 8)) << 16:
            udc_control_send_positive_status();
            /* Don't reset until after the status is sent */
            dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_DETACH;
            break;
          case (USB_CMD(OUT, DEVICE, STANDARD) | (USB_SET_ADDRESS << 8)) << 16:
            udc_control_send_positive_status();
            /* Set the address but don't enable it until the status response
             * is sent.
             */
            USB->DEVICE.DADD.reg = USB_DEVICE_DADD_DADD(value);
            break;
          case (USB_CMD(OUT, INTERFACE, CLASS) | (DFU_REQUEST_DNLOAD << 8)) << 16:
            switch (dfu_getstatus_response.dwStateAndString)
            {
              case DFU_STATE_DFU_IDLE:
                /* First request; initialize the tail pointer. */
                nvm_tail_ptr = (uint32_t *)(APP_ORIGIN);
                dfu_block_num = 0;
                /* Technically, a zero-length download request in the DFU_IDLE
                 * state should be considered an error. However, treating it
                 * the same as in the DFU_DNLOAD_IDLE state is harmless.
                 */
                /* fall through */
              case DFU_STATE_DFU_DNLOAD_IDLE:
                if (dfu_block_num != value || length & 0x3)
                {
                  /* Block counter doesn't match, or we're not receiving full words */
                  dfu_stall_invalid_request();
                  break;
                }
                ++dfu_block_num;

                if (!length)
                {
                  dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_MANIFEST_SYNC;

                  /* Flush page buffer only if the address is valid */
                  if (NVMCTRL->ADDR.reg)
                    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
                  while (!NVMCTRL->INTFLAG.reg);

                  udc_control_send_positive_status();
                  break;
                }

                expected_block_length = length;
                dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_DNLOAD_SYNC;
                break;
              default:
                /* Download request is not valid in any other state */
                dfu_stall_invalid_request();
            }
            break;
          default:
            /* Unrecognized command; stall the pipe */
            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0 | USB_DEVICE_EPSTATUSSET_STALLRQ1;
            break;
        }
        break;
    }

    /* Signal we are ready to receive more data. This could be either
     * (1) The data stage of a control write
     * (2) The host-acknowledgement for a control read.
     * BK0RDY is ignored for SETUP transactions.
     */
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;
  }
}

/* App origin passed from startup to reduce code size */
void bootloader(uint32_t app_origin)
{
#ifdef USE_GPIO
  /* configure PA15 (bootloader entry pin used by SAM-BA) as input pull-up */
  PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
  PORT->Group[0].OUTSET.reg = (1UL << 15);
#endif
  
  /*
  configure oscillator for crystal-free USB operation (USBCRM / USB Clock Recovery Mode)
  */

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905

  /* This is the same step size used by MPLAB Harmony; seems reasonable since fstep=10 yields a typical settling time
   * of 200us with an input clock of 32kHz according to datasheet. Presumably we'd divide fstep
   * by 32 when reducing the input clock to 1kHz, but that would make it zero...
   */
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000) | SYSCTRL_DFLLMUL_CSTEP(1) | SYSCTRL_DFLLMUL_FSTEP(1);
  
  /* Since the DFLL is used in closed-loop, there is no need for a fine calibration from the fuses.
   * (It will be overwritten anyway). Using the middle value (512) follows the example in MPLAB
   * Harmony and saves 8 bytes of shift and mask instructions.
   */
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL) ) | SYSCTRL_DFLLVAL_FINE(512);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS;

  /* No need to wait for DFLL ready signal; GCLK handles the wait in hardware
   * and won't switch until the DFLL signals to the GCLK that it's ready.
   */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;

  PAC1->WPCLR.reg = 2; /* clear DSU */

  DSU->ADDR.reg = app_origin; /* start CRC check at beginning of user app */
  DSU->LENGTH.reg = *(volatile uint32_t *)(app_origin + 0x10); /* use length encoded into unused vector address in user app */

  /* ask DSU to compute CRC */
  DSU->DATA.reg = 0xFFFFFFFF;
  DSU->CTRL.reg = DSU_CTRL_CRC; /* Strobe bits; no need for read-modify-write */
  while (!DSU->STATUSA.bit.DONE);

  dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_IDLE;
  if (DSU->DATA.reg)
  {
    dfu_getstatus_response.dwStatusAndPollTimeout = DFU_STATUS_ERR_FIRMWARE;
    dfu_getstatus_response.dwStateAndString = DFU_STATE_DFU_ERROR;
    goto run_bootloader; /* CRC failed, so run bootloader */
  }

#ifdef USE_GPIO
  if (!(PORT->Group[0].IN.reg & (1UL << 15)))
    goto run_bootloader; /* pin grounded, so run bootloader */

  return; /* we've checked everything and there is no reason to run the bootloader */
#endif

  register uint32_t tmp_rcause = PM->RCAUSE.reg;
  if ((tmp_rcause & bl_info.rcause_mask) > (PM_RCAUSE_POR | PM_RCAUSE_BOD12 | PM_RCAUSE_BOD33))
  {
    /* Matched SYST, EXT, or WDT reset. Do not match SYST reset next time
     * because that is how we exit the bootloader after DFU.
     */
    bl_info.rcause_mask = PM_RCAUSE_WDT;

    /* Report watchdog timeout as a DFU_STATUS_ERR_TARGET */
    dfu_getstatus_response.dwStatusAndPollTimeout = (tmp_rcause >> PM_RCAUSE_WDT_Pos) & 0x1;
    goto run_bootloader;
  }

#ifdef USE_DOUBLE_TAP
  /* If last reset was an external reset, add external reset to the mask
   * so we stay in the bootloader if the user presses the reset button again.
   * The application can clear this bit after a delay to implement a time-based
   * double-tap behavior.
   */
  tmp_rcause = (tmp_rcause & PM_RCAUSE_EXT) | (PM_RCAUSE_WDT | PM_RCAUSE_SYST | PM_RCAUSE_EXT);
  bl_info.rcause_mask = tmp_rcause;
#else
  /* Initialize the mask for the next soft-reset */
  bl_info.rcause_mask = PM_RCAUSE_WDT | PM_RCAUSE_SYST;
#endif

  return;

run_bootloader:
  /*
  initialize USB
  */

  PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_G_Val) | PORT_WRCONFIG_PINMASK(0x0300);

  PM->APBBMASK.reg = PM_APBBMASK_USB | PM_APBBMASK_PORT | PM_APBBMASK_NVMCTRL | PM_APBBMASK_DSU;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) | GCLK_CLKCTRL_GEN(0);

  USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN( NVM_READ_CAL(NVM_USB_TRANSN) ) | USB_PADCAL_TRANSP( NVM_READ_CAL(NVM_USB_TRANSP) ) | USB_PADCAL_TRIM( NVM_READ_CAL(NVM_USB_TRIM) );

  USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;
  USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY | USB_CTRLA_ENABLE;

  /*
  service USB
  */

  while (1)
  {
#ifdef HANDLE_WATCHDOG
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
#endif
    USB_Service();
  }
}
