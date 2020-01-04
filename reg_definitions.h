/*
 * Copyright (c) 2020 Dimitar Kunchev
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
 
#include <Arduino.h>
#ifndef __REG_DEFINITIONS__
#define __REG_DEFINITIONS__

// #define DAP_TRANSFER_AP = 1 << 0,
// #define DAP_TRANSFER_READ = 1 << 1,

enum {
  SWD_DP_REG_IDCODE = 0x00,
  SWD_DP_REG_ABORT = 0x00,
  SWD_DP_REG_CTRL_STAT = 0x04, // When CTRLSEL == 0
  SWD_DP_REG_WCR = 0x04,       // When CTRLSEL == 1
  SWD_DP_REG_RESEND = 0x08,
  SWD_DP_REG_SELECT = 0x08,
  SWD_DP_REG_RDBUFF = 0x0c,
};

enum {
  SWD_AP_REG_CSW = 0x00, // | DAP_TRANSFER_AP,
  SWD_AP_REG_TAR = 0x04, // | DAP_TRANSFER_AP,
  SWD_AP_REG_DRW = 0x0c, // | DAP_TRANSFER_AP,

  SWD_AP_REG_DB0 = 0x00, // | DAP_TRANSFER_AP, // 0x10
  SWD_AP_REG_DB1 = 0x04, // | DAP_TRANSFER_AP, // 0x14
  SWD_AP_REG_DB2 = 0x08, // | DAP_TRANSFER_AP, // 0x18
  SWD_AP_REG_DB3 = 0x0c, // | DAP_TRANSFER_AP, // 0x1c

  SWD_AP_REG_CFG = 0x04, // | DAP_TRANSFER_AP,  // 0xf4
  SWD_AP_REG_BASE = 0x08, // | DAP_TRANSFER_AP, // 0xf8
  SWD_AP_REG_IDR = 0x0c, // | DAP_TRANSFER_AP,  // 0xfc
};

// SAMDx5 specific


#define DHCSR 0xe000edf0      /// Debug Halting Control and Status Register
// #define DCRSR 0xE000EDF4      /// Debug Core Register Selector Register
#define DEMCR 0xe000edfc      /// Debug Exception and Monitor Control Register
// #define DCRDR 0xE000EDF8      /// Debug Core Register Data Register
#define AIRCR 0xe000ed0c      /// Application Interrupt and Reset Control Register

// As per chapter 12.9 of the SAMD51 datasheet: DSU registers are mirrored at offset 0x100 from the base 0x41002000 address for security reasons. External probes have to access it there
#define DAP_DSU_BASE        0x41002100
#define DAP_DSU_CTRL_STATUS (DAP_DSU_BASE + 0)
#define DAP_DSU_ADDR        (DAP_DSU_BASE + 0x04)
#define DAP_DSU_LENGTH      (DAP_DSU_BASE + 0x08);
#define DAP_DSU_DATA        (DAP_DSU_BASE + 0x0C)

#define DAP_DSU_DID         (DAP_DSU_BASE + 0x18)

//

#define USER_ROW_ADDR 0x00804000
#define USER_ROW_SIZE 32

#endif // __REG_DEFINITIONS__
