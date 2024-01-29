/*
  This provides USB mouse functionality similar to "mouseplay" in the "example-apps" directory of:
  https://github.com/majbthrd/PIC16F1-USB-DFU-Bootloader
*/

/*
 * Copyright (c) 2018, Peter Lawrence
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
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd11.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_hid.h"

/*- Definitions -------------------------------------------------------------*/

/* Types --------------------------------------------------------------------*/
typedef struct
{
  uint32_t reserved2;
  uint32_t reserved1;
  uint32_t reserved0;
  uint32_t rcause_mask;
} bl_info_t;

/*- Variables ---------------------------------------------------------------*/
static alignas(4) uint8_t app_hid_report[64];
static bool app_hid_report_free = true;
static unsigned tick_count;

static volatile bl_info_t __attribute__((section(".bl_info"))) bl_info;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  uint32_t sn = 0;

  /* Bootloader performs clock init */

  sn ^= *(volatile uint32_t *)0x0080a00c;
  sn ^= *(volatile uint32_t *)0x0080a040;
  sn ^= *(volatile uint32_t *)0x0080a044;
  sn ^= *(volatile uint32_t *)0x0080a048;

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[9] = 0;
}

//-----------------------------------------------------------------------------
void usb_hid_send_callback(void)
{
  app_hid_report_free = true;
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_hid_report_free = false;

  usb_hid_send(app_hid_report, 3);
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  (void)config;
  usb_hid_send_callback();
}

//-----------------------------------------------------------------------------
void usb_sof_callback(void)
{
  tick_count++;
}

//-----------------------------------------------------------------------------
static void hid_task(void)
{
  static unsigned table_index = 0;

  /* arbitrary mouse movement pattern to play back */
  const int8_t move_table[]=
  {
          /* 
          X, Y, (at time 0)
          X, Y, (at time 1)
          X, Y, (at time 2)
          ...
          */
          6, -2,
          2, -6,
          -2, -6,
          -6, -2,
          -6, 2,
          -2, 6,
          2, 6,
          6, 2,
  };

  if (tick_count < 64)
    return;

  if (!app_hid_report_free)
    return;

  /* table_index modulus 16 *AND* make table_index an even number */
  table_index &= 0xE;

  app_hid_report[0] = 0;
  app_hid_report[1] = move_table[table_index++];
  app_hid_report[2] = move_table[table_index++];

  send_buffer();
  tick_count = 0;
}

//-----------------------------------------------------------------------------
int main(void)
{
  /* Implement double-tap-/RESET bootloader entry */
  volatile int wait = 65536; while (wait--);
  /* If we didn't get an external reset by now, the window for double-tapping
   * the reset button has closed.
   */
  bl_info.rcause_mask &= ~PM_RCAUSE_EXT;

  sys_init();
  usb_init();
  usb_hid_init();

  while (1)
  {
    usb_task();
    hid_task();
  }

  return 0;
}
