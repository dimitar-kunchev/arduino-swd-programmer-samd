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

#include "samd51_dap.h"

bool samd_power_up_debug_domains() {
  dap_write_ctrl_stat(SWD_REG_CTRL_STAT_CSYSPWRUPREQ | SWD_REG_CTRL_STAT_CDBGPWRUPREQ);
  dap_read_read_buf();  // should return 0
  uint32_t ctrl_stat = dap_read_ctrl_stat();
  for (int i = 0; i < 100; i ++){
    if ((ctrl_stat & SWD_REG_CTRL_STAT_CSYSPWRUPACK) && (ctrl_stat & SWD_REG_CTRL_STAT_CDBGPWRUPACK)) {
      return true;
    } else {
      delay(1);
    }
  }
  return false;
}
 
void samd_stop_core() {
  dap_write_word(DHCSR, 0xa05f0003);
  dap_read_read_buf();
  dap_write_word(DEMCR, 0x00100501); // ICE writes 1000001 to DEMCR?
  dap_read_read_buf();
  dap_write_word(AIRCR, 0x05fa0004);
}

uint32_t samd_read_dsu_did() {
  // Serial.print("Read DSU DID: ");
  uint32_t tmp;
  if (dap_read_word(DAP_DSU_DID, &tmp)) {
    uint32_t did = dap_read_read_buf();
    // Serial.print("0x"); Serial.println(did, HEX);
    return did;
  }
  return 0;
}

uint32_t samd_read_dsu_ctrl_status() {
  uint32_t res = 0;
  dap_read_word(DAP_DSU_CTRL_STATUS, &res);
  res = dap_read_read_buf();
  return res;
}

uint32_t samd_write_dsu_ctrl_status(uint32_t val) {
  return dap_write_word(DAP_DSU_CTRL_STATUS, val);
}

bool samd_read_user_mem(uint8_t * user_bytes) {
  //Serial.println("Read user mem");
  //memset(user_bytes, 0, 512);
  return dap_read_mem_block(USER_ROW_ADDR, user_bytes, 512);
}

bool samd_read_fuses(uint8_t * fuse_bytes) {
  //memset(fuse_bytes, 0, 32);
  return dap_read_mem_block(USER_ROW_ADDR, fuse_bytes, 32);
}

bool samd_chip_erase() {
  uint32_t ctrl_status = samd_read_dsu_ctrl_status();
  dap_write_word(DAP_DSU_CTRL_STATUS, 0x00001f00); // Clear flags
  dap_write_word(DAP_DSU_CTRL_STATUS, 0x00000010); // Chip erase
  delay(100);
  int retries = 600;
  while (retries > 0) {
    ctrl_status = samd_read_dsu_ctrl_status();
    if ((ctrl_status & 0x00000100) != 0) { // check the DONE flag
      break;
    }
    retries --;
    delay(100);
  };
  
  return ((ctrl_status & 0x00000100) != 0);
}

bool samd_read_serial_number(uint8_t * res) {
  uint32_t tmp;
  if (!dap_read_word(0x008061FC, &tmp)) {
    return false;
  }
  memcpy(res, &tmp, 4);
  if (!dap_read_word(0x00806010, &tmp)) {
    return false;
  }
  memcpy(&res[4], &tmp, 4);
  if (!dap_read_word(0x00806014, &tmp)) {
    return false;
  }
  memcpy(&res[8], &tmp, 4);
  if (!dap_read_word(0x00806018, &tmp)) {
    return false;
  }
  memcpy(&res[12], &tmp, 4);
  return true;
}


bool samd_perform_mbist(uint32_t mem_start_address, uint32_t mem_length) {
  // write address start to DSU ADDR
  dap_write_word(DAP_DSU_ADDR, mem_start_address << 2); // AMOD=0: exit on error
  // write memory length to DSU LENGTH
  dap_write_word(DAP_DSU_LENGTH, mem_length);
  // write 1 to DSU MBIST
  uint32_t ctrl_status = samd_read_dsu_ctrl_status();
  ctrl_status |= 1 << 3;
  Serial.print("Set DSU CTRL/STATUS: 0x"); Serial.println(ctrl_status, HEX);
  samd_write_dsu_ctrl_status(ctrl_status);

  while (true) {
    //delay(1);
    ctrl_status = samd_read_dsu_ctrl_status();
    Serial.print("CST: 0x"); Serial.println(ctrl_status, HEX);
    if (ctrl_status & (1 << 8)) { // check the DONE bit
      Serial.println("Done");
      uint32_t tmp;
      dap_read_word(DAP_DSU_ADDR, &tmp);
      tmp = dap_read_read_buf();
      Serial.print("ADDR reg contents: 0x"); Serial.println(tmp, HEX);
      dap_read_word(DAP_DSU_DATA, &tmp);
      tmp = dap_read_read_buf();
      Serial.print("DATA reg contents: 0x"); Serial.println(tmp, HEX);
      if (ctrl_status & ( 1 << 11)) {
        Serial.println("MBIST detected errors");
        return false;
      } else {
        Serial.println("MBIST found no problems");
        if (ctrl_status & (1 << 10)) {
          Serial.println("However there is DSU.BERR (bus error) flag raised");
        }
        return true;
      }
    }
  }

  
}
