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
 
#include "line_write.h"
#include "reg_definitions.h"

#define SWDIO 9
#define SWCLK 8
#define SWRST 10

static uint32_t calc_parity(uint32_t value) {
  value ^= value >> 16;
  value ^= value >> 8;
  value ^= value >> 4;
  value &= 0x0f;
  return (0x6996 >> value) & 1;
}

static inline uint32_t build_request(uint8_t addr, bool is_ap_reg, bool is_read) {
  uint8_t request = 0; ;
  if (is_ap_reg) {
    request |= 1 << 1;            // set APnDP bit
  }
  if (is_read) {
    request |= 1 << 2;            // set RnW bit
  }
  request |= (addr & 0x0C) << 1;  // set ADDR[2:3] bits
  request |= calc_parity(request) << 5; //set parity
  request |= (1 << 7) | (1);      // set start & stop bits
  return request;
}

bool read_reg(uint8_t addr, bool is_ap_reg, uint32_t * res) {
  int attempts_to_read_reg = 0;
retry_read_reg:
  uint8_t request = build_request(addr, is_ap_reg, true);
  *res = 0;
  
  swd_write_line(request, 8);  // Write the request
  swd_turn_around_to_input();

  uint8_t ack = swd_read_ack(); // read the ACK
  if (ack != SWD_ACK_OK) {
    swd_turn_around_to_output();
    
    if (ack == SWD_ACK_WAIT) {
      delayMicroseconds(10);
      if (attempts_to_read_reg < 10) {
        Serial.print("wait... ");
        attempts_to_read_reg ++;
        goto retry_read_reg;
      }
    } else {
      // ack == SWD_ACK_ERR
      Serial.print("ACK ERR in reg read ");
      Serial.print(addr, HEX);
      Serial.print(" AP: "); Serial.print(is_ap_reg ? "Y" : "N");
      Serial.print(" ack: ");
      Serial.println(ack);
    }
    return false;
  }
  
  *res = swd_read_line(32);
  
  uint8_t parity_bit = swd_read_line(1); // read parity
  swd_turn_around_to_output();

  // Serial.println("ACK OK");
  if (parity_bit != calc_parity(*res)) {
    Serial.println("Parity mismatch");
  }
  // Serial.println(*res, HEX);
  return true;
}

bool write_reg(uint8_t addr, bool is_ap_reg, uint32_t val) {
  int attempts_to_write_reg = 0;
retry_write_reg:
  uint8_t request = build_request(addr, is_ap_reg, false);
  
  swd_write_line(request, 8);  // Write the request
  swd_turn_around_to_input();

  uint8_t ack = swd_read_ack(); // read the ACK
  if (ack != SWD_ACK_OK) {
    if (ack == SWD_ACK_WAIT) {
      delayMicroseconds(10);
      if (attempts_to_write_reg < 10) {
        Serial.print("wait... ");
        attempts_to_write_reg ++;
        goto retry_write_reg;
      }
    } else {
      Serial.print("ACK ERR in reg write ");
      Serial.print(addr, HEX);
      Serial.print(" AP: "); Serial.print(is_ap_reg ? "Y" : "N");
      Serial.print(" val: "); Serial.print(val, HEX);
      Serial.print(" ack: ");
      Serial.println(ack);
      return false;
    }
  }
  
  swd_turn_around_to_output();
  uint8_t parity_bit = calc_parity(val);
  swd_write_line(val, 32);
  swd_write_line(parity_bit, 1);
  
  // Serial.println("ACK OK"); 
  return ack == 1;
}

uint32_t read_id_code() {
  Serial.print("Read ID Code: ");
  uint32_t id_code = 0;
  read_reg(SWD_DP_REG_IDCODE, false, &id_code);
  return id_code;
}

uint32_t read_ctrl_stat() {
  Serial.print("Read CTRL/STAT: ");
  uint32_t ctrl_stat = 0;
  read_reg(SWD_DP_REG_CTRL_STAT, false, &ctrl_stat);
  Serial.println(ctrl_stat, HEX);
  return ctrl_stat;
}

uint32_t read_read_buf() {
  Serial.print("Read READ BUF: ");
  uint32_t read_buf = 0;
  if (!read_reg(SWD_DP_REG_RDBUFF, false, &read_buf)) {
    Serial.println(read_ctrl_stat());
    while(1);
  }
  Serial.print("0x"); Serial.println(read_buf, HEX);
  return read_buf;
}

bool write_ctrl_stat(uint32_t val) {
  Serial.print("Write CTRL STAT 0x");
  Serial.println(val, HEX);
  return write_reg(SWD_DP_REG_CTRL_STAT, false, val);
}

bool write_abort(uint32_t val) {
  Serial.print("Write ABORT 0x");
  Serial.println(val, HEX);
  return write_reg(SWD_DP_REG_ABORT, false, val);
}

bool write_select(uint32_t val) {
  Serial.print("Write SELECT 0x");
  Serial.println(val, HEX);
  return write_reg(SWD_DP_REG_SELECT, false, val);
}

///

bool write_csw(uint32_t val) {
  Serial.print("Write AP CSW 0x");
  Serial.println(val, HEX);
  return write_reg(SWD_AP_REG_CSW, true, val);
}
uint32_t read_csw() {
  uint32_t val;
  if (read_reg(SWD_AP_REG_CSW, true, &val)) {
    val = read_read_buf();
  }
  Serial.print("Read CSW: 0x"); Serial.println(val, HEX);
  return val;
}

//

bool read_word(uint32_t addr, uint32_t * res) {
  return (write_reg(SWD_AP_REG_TAR, true, addr) && read_reg(SWD_AP_REG_DRW, true, res));
}

bool write_word(uint32_t addr, uint32_t data) {
  return write_reg(SWD_AP_REG_TAR, true, addr) && write_reg(SWD_AP_REG_DRW, true, data);
}

///

void stop_core() {
  Serial.println("Stop core");
  
  write_word(DHCSR, 0xa05f0003);
  read_read_buf();
  write_word(DEMCR, 0x00100501); // ICE writes 1000001 to DEMCR?
  read_read_buf();
  write_word(AIRCR, 0x05fa0004);
  
  Serial.println("Core stopped\r\n");
}

uint32_t read_did() {
  Serial.print("Read DSU DID: ");
  uint32_t tmp;
  if (read_word(DAP_DSU_DID, &tmp)) {
    uint32_t did = read_read_buf();
    Serial.println(did, HEX);
    return did;
  } else {
    return 0;
  }
}

uint32_t read_dsu_ctrl_status() {
  Serial.print("Read DSU CTRL Status: ");
  uint32_t res = 0;
  read_word(DAP_DSU_CTRL_STATUS, &res);
  Serial.println(res, HEX);
  res = read_read_buf();
  Serial.print("DSU Ctrl Status: 0x"); Serial.println(res, HEX);
  return res;
}

/// Block Operations

// IMPORTANT: Use sizes, multiples of 4
bool read_block(uint32_t address, uint8_t * res, int size) {
  //write_csw(0x23000052); // increment single, size = 32;    // 
  write_csw(SWD_REG_MEM_AP_CSW_Prot(35) | SWD_REG_MEM_AP_CSW_AddrInc_Single | SWD_REG_MEM_AP_CSW_Size_Word);

  if (!write_reg(SWD_AP_REG_TAR, true, address)) {
    return false;
  }
  
  int read_bytes = 0;
  uint32_t cr;

  // I don't understand why but we need to do one read first that contains some old information. After that it all goes well. I think
  read_reg(SWD_AP_REG_DRW, true, &cr);
  
  while (read_bytes < size) {
    read_reg(SWD_AP_REG_DRW, true, &cr);

    memcpy(&(res[read_bytes]), &cr, 4);
    read_bytes += 4;
  }

  read_read_buf();

  return true;
}

void read_user_mem() {
  Serial.println("Read user mem");
  uint8_t user_bytes[512];
  memset(user_bytes, 0, 512);
  if (!read_block(USER_ROW_ADDR, user_bytes, 512)) {
    return;
  }
  Serial.println("Contents:");
  for (int i = 0; i < 512; i ++) {
    if (i%4 == 0) {
      Serial.print(" 0x");
    }
    Serial.print(user_bytes[i], HEX);
    if (i%8 == 7) {
      Serial.println();
    }
  }
}

void read_fuses() {
  Serial.println("Read fuses");
  uint8_t fuse_bytes[32];
  memset(fuse_bytes, 0, 32);
  if (!read_block(USER_ROW_ADDR, fuse_bytes, 32)) {
    return;
  }
  Serial.println("Fuses contents:");
  for (int i = 0; i < 32; i ++) {
    if (i%4 == 0) {
      Serial.print(" 0x");
    }
    Serial.print(fuse_bytes[i], HEX);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {
    delay(1);         // will pause the chip until it opens serial console
  }
/*
  uint32_t tmp = build_request(0x00, false, true);
  Serial.println(tmp, HEX); 
  while (1);*/
  
  // Prepare registers we will be using, allowing us to switch pins quickly, instead of using the slow Arduino functions
  // Reset the target, prep clk/data pins
  swd_prepare_pin_registers_and_reset_target(SWDIO, SWCLK, SWRST);
  swd_line_rst_switch_to_swd();
  
  delay(1); // give some time the target to wake up. not really needed...

  // let's get busy
  read_id_code();
  uint32_t ctrl_stat = read_ctrl_stat();
  read_read_buf();

  write_select(0x00); // select the mem/ap
  read_read_buf();  // should return 0

  Serial.println("Power-up the system and debug domains");

  // power-up and debug power-up request
  write_ctrl_stat(SWD_REG_CTRL_STAT_CSYSPWRUPREQ | SWD_REG_CTRL_STAT_CDBGPWRUPREQ);
  read_read_buf();  // should return 0
  ctrl_stat = read_ctrl_stat();
  for (int i = 0; i < 100; i ++){
    if ((ctrl_stat & SWD_REG_CTRL_STAT_CSYSPWRUPACK) && (ctrl_stat & SWD_REG_CTRL_STAT_CDBGPWRUPACK)) {
      Serial.println("Power-up requests complete");
      break;
    } else {
      delay(1);
    }
  }

  // check if there are any stick errors set and clear them if needed
  ctrl_stat = read_ctrl_stat();
  if (ctrl_stat & (SWD_REG_CTRL_STAT_WDATAERR|SWD_REG_CTRL_STAT_STICKYERR|SWD_REG_CTRL_STAT_STICKYCMP|SWD_REG_CTRL_STAT_STICKYORUN)) {
    write_abort(SWD_REG_ABORT_Clear_All);
  }
  
  // Debug start request
  // Technically what we should do is: write bit 26 (CDBGRSTREQ) of the CTRL/STAT, wait until bit 27 (CDBGRSTACK) is set and then clear bit 26. 
  // However it seems the Atmel ICE gives up after few ms
  // It would appear this is NOT actually implemented in the CPU.
  /*
  Serial.println("Attempt debugger core reset\r\n");
  for (int i = 0; i < 20; i ++){
    write_ctrl_stat(SWD_REG_CTRL_STAT_CSYSPWRUPREQ | SWD_REG_CTRL_STAT_CDBGPWRUPREQ | SWD_REG_CTRL_STAT_CDBGRSTREQ);
    read_read_buf();  // should return 0
    ctrl_stat = read_ctrl_stat();
    if ((ctrl_stat & SWD_REG_CTRL_STAT_CDBGRSTACK)) {
      Serial.println("Debugger reset request complete");
      break;
    } else {
      delayMicroseconds(100);
    }
  }
  if (!(ctrl_stat & SWD_REG_CTRL_STAT_CDBGRSTACK)) {
    Serial.println("Debugger reset request doesn't seem to work...");
  }
  
  // clear the CDBGRSTREQ - whether it was successfull or not we need to release the reset
  write_ctrl_stat(SWD_REG_CTRL_STAT_CSYSPWRUPREQ | SWD_REG_CTRL_STAT_CDBGPWRUPREQ); // | SWD_REG_CTRL_STAT_MASKLANE(0xF)
  read_read_buf();
  ctrl_stat = read_ctrl_stat();
  */
  
  write_csw(SWD_REG_MEM_AP_CSW_Prot(35) | SWD_REG_MEM_AP_CSW_AddrInc_Single | SWD_REG_MEM_AP_CSW_Size_Word);
  read_read_buf();

  stop_core();

  // Read device ID. Takes few attempts sometimes and raises sticy error. Don't know why
  for (int i = 0; i < 10; i ++) {
    if (read_did() == 0) { // should be 0x60060004 for SAMD51J20A
      ctrl_stat = read_ctrl_stat();
      if (ctrl_stat & SWD_REG_CTRL_STAT_STICKYERR) {
        write_abort(SWD_REG_ABORT_Clear_All);
        read_ctrl_stat();
      }
    } else {
      break;
    }
  }

  // Check if DeviceEn bit is set 
  if (! (read_csw() & SWD_REG_MEM_AP_CSW_DeviceEn)) {
    Serial.println("CSW.DeviceEn bit not set - error");
    while(1);
  }

  // Check if device security bit is set
  if (read_dsu_ctrl_status() & 0x00010000) {
    // Note that the result is flipped. 0x8120000 is what I usually get. First byte of that 0x03 offset, second is 0x02, etc. So that value means DBGPRES and HPE are set (and something in the reserved space)
    Serial.println("Security bit is set");
    while(1);
  }

  //read_user_mem();
  read_fuses();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
}
