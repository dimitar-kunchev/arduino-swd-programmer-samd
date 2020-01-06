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
  dap_write_word(DEMCR, 0x00100501); // ICE writes 1000001 to DEMCR?
  dap_write_word(AIRCR, 0x05fa0004);
}

uint32_t samd_read_dsu_did() {
  // Serial.print("Read DSU DID: ");
  uint32_t tmp;
  dap_read_word(DAP_DSU_DID, &tmp);
  return tmp;
}

uint32_t samd_read_dsu_ctrl_status() {
  uint32_t res = 0;
  dap_read_word(DAP_DSU_CTRL_STATUS, &res);
  return res;
}

uint32_t samd_write_dsu_ctrl_status(uint32_t val) {
  return dap_write_word(DAP_DSU_CTRL_STATUS, val);
}


/*
 * NVMCTRL FUNCTIONS
 */

bool samd_read_user_mem(uint8_t * user_bytes) {
  //Serial.println("Read user mem");
  //memset(user_bytes, 0, 512);
  return dap_read_mem_block(USER_ROW_ADDR, user_bytes, 512);
}

bool samd_read_fuses(uint8_t * fuse_bytes) {
  //memset(fuse_bytes, 0, 32);
  return dap_read_mem_block(USER_ROW_ADDR, fuse_bytes, 32);
}

static bool samd_nvmctrl_get_status_ready() {
  uint32_t tmp;
  if (!dap_read_word(DAP_NVMCTRL_INTFLAG, &tmp)) {
    return false;
  }
  tmp >>= 16;
  // Serial.print("DAP STATUS: 0x"); Serial.println(tmp, HEX);
  return tmp & 0x01;
}

static bool samd_nvmctrl_get_intflag_done() {
  uint32_t tmp;
  if (!dap_read_word(DAP_NVMCTRL_INTFLAG, &tmp)) {
    return false;
  }
  tmp &= 0xffff;
  // Serial.print("DAP INTFLAG: 0x"); Serial.println(tmp, HEX);
  return tmp & 0x01;
}

static bool samd_nvmctrl_wait_status_ready(int cnt) {
  bool ready_bit_set = false;
  for (int i = 0; i <cnt & !ready_bit_set; i ++) {
    ready_bit_set = samd_nvmctrl_get_status_ready();
    if (!ready_bit_set) {
      delay(1);
    }
  }
  return ready_bit_set;
}

static bool samd_nvmctrl_wait_intflag_done(int cnt) {
   bool done_bit_set = false;
  for (int i = 0; i <cnt & !done_bit_set; i ++) {
    done_bit_set = samd_nvmctrl_get_intflag_done();
    if (!done_bit_set) {
      delay(1);
    }
  }
  return done_bit_set;
}

static bool samd_nvmctrl_ctrla_clr_wmode_and_set_autows() {
  uint32_t tmp;
  if (!dap_read_word(DAP_NVMCTRL_CTRLA, &tmp)) {
    return false;
  }
  // Serial.print("DAP CTRLA: 0x"); Serial.println(tmp, HEX);

  // Serial.println("Clr wmode, set autows");
  tmp = (tmp & (~DAP_NVMCTRL_CTRLA_WMODE_Msk));
  tmp |= DAP_NVMCTRL_CTRLA_AUTOWS;
  
  if (!dap_write_word(DAP_NVMCTRL_CTRLA, tmp)) {
    return false;
  }
  
  // verify
  uint32_t tmp2 = 0;
  if (!dap_read_word(DAP_NVMCTRL_CTRLA, &tmp2)) {
    return false;
  }
  // tmp2 = dap_read_read_buf();
  // Serial.print("DAP CTRLA: 0x"); Serial.println(tmp, HEX);
  return tmp == tmp2;
  
}

bool samd_write_user_mem(uint32_t address_offset, uint8_t * data, uint32_t length) {
  // Wait for NVMCTRL.CTRLA.READY bit
  uint32_t tmp;
  samd_nvmctrl_wait_status_ready(100);
  
  // Set NVMCTRL.CTRLA.WMODE to zero for manual write
  if (!samd_nvmctrl_ctrla_clr_wmode_and_set_autows()) {
    // Serial.println("Failed");
    return false;
  }

  samd_nvmctrl_wait_status_ready(100);
  
  // Serial.println("Clear page buffer");
  // Clear the page buffer with the PBC command
  dap_write_word(DAP_NVMCTRL_CTRLB, DAP_NVMCTRL_CTRLB_CMD_PBC | DAP_NVMCTRL_CTRLB_CMDEX_KEY);

  // Wait for NVMCTRL.CTRLA.READY bit
  samd_nvmctrl_wait_status_ready(100);
  samd_nvmctrl_wait_intflag_done(100);

  // Write the actual data in memory. Quad-word at a time
  uint32_t target_address = USER_ROW_ADDR+address_offset;
  tmp = 0;
  for (int j = 0; j < length/16; j ++) {
    // Set the ADDR register
    uint32_t quad_word_start_address = target_address;
    dap_write_word(DAP_NVMCTRL_ADDR, quad_word_start_address);
    
    for (int i = 0; i < 16; i+= 4) {
      memcpy(&tmp, &(data[i+j*16]), 4);
      // Serial.print("Write 0x"); Serial.print(tmp, HEX); Serial.print(" to 0x"); Serial.println(target_address, HEX);
      dap_write_word(target_address, tmp);
      target_address += 4;
    }
  
    // Write the WQW command in CTRLB
    dap_write_word(DAP_NVMCTRL_CTRLB, DAP_NVMCTRL_CTRLB_CMDEX_KEY | DAP_NVMCTRL_CTRLB_CMD_WQW);
    
    // Wait for STATUS.READY and INTFLAG.DONE
    samd_nvmctrl_wait_status_ready(100);
    samd_nvmctrl_wait_intflag_done(1000);
  }
  return true;
}

bool samd_erase_user_mem() {
  samd_nvmctrl_wait_status_ready(100);
  samd_nvmctrl_ctrla_clr_wmode_and_set_autows();
  dap_write_word(DAP_NVMCTRL_ADDR, USER_ROW_ADDR);
  dap_write_word(DAP_NVMCTRL_CTRLB, DAP_NVMCTRL_CTRLB_CMDEX_KEY | DAP_NVMCTRL_CTRLB_CMDEX_EP);
  samd_nvmctrl_wait_status_ready(100);
  samd_nvmctrl_wait_intflag_done(1000);
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
  // tmp = dap_read_read_buf();
  tmp=bswap32(tmp);
  memcpy(res, &tmp, 4);
  if (!dap_read_word(0x00806010, &tmp)) {
    return false;
  }
  // tmp = dap_read_read_buf();
  tmp=bswap32(tmp);
  memcpy(&res[4], &tmp, 4);
  if (!dap_read_word(0x00806014, &tmp)) {
    return false;
  }
  // tmp = dap_read_read_buf();
  tmp=bswap32(tmp);
  memcpy(&res[8], &tmp, 4);
  if (!dap_read_word(0x00806018, &tmp)) {
    return false;
  }
  // tmp = dap_read_read_buf();
  tmp=bswap32(tmp);
  memcpy(&res[12], &tmp, 4);
  return true;
}

/*
 * OTHER FUNCTIONS
 */

bool samd_perform_mbist(uint32_t mem_start_address, uint32_t mem_length) {
  // write address start to DSU ADDR
  dap_write_word(DAP_DSU_ADDR, mem_start_address << 2); // AMOD=0: exit on error
  // write memory length to DSU LENGTH
  dap_write_word(DAP_DSU_LENGTH, mem_length<<2);
  // write 1 to DSU MBIST
  uint32_t ctrl_status = samd_read_dsu_ctrl_status();
  ctrl_status |= 1 << 3;
  //Serial.print("Set DSU CTRL/STATUS: 0x"); Serial.println(ctrl_status, HEX);
  samd_write_dsu_ctrl_status(ctrl_status);

  while (true) {
    //delay(1);
    ctrl_status = samd_read_dsu_ctrl_status();
    //Serial.print("CST: 0x"); Serial.println(ctrl_status, HEX);
    if (ctrl_status & (1 << 8)) { // check the DONE bit
      //Serial.println("Done");
      uint32_t tmp;
      dap_read_word(DAP_DSU_ADDR, &tmp);
      //Serial.print("ADDR reg contents: 0x"); Serial.println(tmp, HEX);
      dap_read_word(DAP_DSU_DATA, &tmp);
      tmp = dap_read_read_buf();
      //Serial.print("DATA reg contents: 0x"); Serial.println(tmp, HEX);
      if (ctrl_status & ( 1 << 11)) {
        //Serial.println("MBIST detected errors");
        return false;
      } else {
        //Serial.println("MBIST found no problems");
//        if (ctrl_status & (1 << 10)) {
//          Serial.println("However there is DSU.BERR (bus error) flag raised");
//        }
        return true;
      }
    }
  } 
}
