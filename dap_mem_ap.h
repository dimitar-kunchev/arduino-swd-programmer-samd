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

 /**
  * Here are some wrapping functions to send requests to a MEM AP of the target.
  * They implement ARM Debug Interface version 5 (ADIv5) and are tested
  * against Cortex M4
  * 
  * These are intended only to simplify further development and reducing risk of
  * errors when accessing registers.
  * 
  * What bugs me is I have to read RDBUF every time when I try to read DRW or CSW. That makes no sense...
  */

#include <Arduino.h>
#include "dap.h"
#include "reg_definitions.h"

#ifndef __DAP_MEM_AP__
#define __DAP_MEM_AP__

#ifdef __cplusplus
extern "C" {
#endif

// Write the CSW AP register
inline bool dap_write_csw(uint32_t val) {
  // Serial.print("Write AP CSW 0x"); Serial.println(val, HEX);
  return dap_write_reg(SWD_AP_REG_CSW, true, val);
}

// Read the CSW AP register
inline uint32_t dap_read_csw() {
  uint32_t val;
  if (dap_read_reg(SWD_AP_REG_CSW, true, &val)) {
    val = dap_read_read_buf();
  }
  // Serial.print("Read CSW: 0x"); Serial.println(val, HEX);
  return val;
}

// Read the DRW AP register
inline bool dap_read_drw(uint32_t * res) {
   // return dap_read_reg(SWD_AP_REG_DRW, true, res);
   if (dap_read_reg(SWD_AP_REG_DRW, true, res)) {
    *res = dap_read_read_buf();
    return true;
   }
   return false;
}

// Write the DRW AP register
inline bool dap_write_drw(uint32_t val) {
   return dap_write_reg(SWD_AP_REG_DRW, true, val);
}

// Write the TAR AP register
inline bool dap_write_tar(uint32_t address) {
  return dap_write_reg(SWD_AP_REG_TAR, true, address);
}

////

// Read a memory word: write an address to TAR and read the contents of that address via DRW
inline bool dap_read_word(uint32_t addr, uint32_t * res) {
  return (dap_write_tar(addr) && dap_read_drw(res));
}

// Write a memory word: write the address to TAR and the contents to DRW
inline bool dap_write_word(uint32_t addr, uint32_t data) {
  return dap_write_tar(addr) && dap_write_drw(data);
}

///

// Read a block from memory. Use sizes, multiples of 4 (as we read it in 32 bit words)
bool dap_read_mem_block(uint32_t address, uint8_t * res, int size);

#ifdef __cplusplus
};
#endif 

#endif //  __DAP_MEM_AP__
