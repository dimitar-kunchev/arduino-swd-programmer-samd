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

 #include "dap.h"
 #include "swd_io.h"

uint32_t dap_begin_and_identify(int swdio_pin, int swclk_pin, int nrst_pin) {
  swd_prepare_pin_registers_and_reset_target(swdio_pin, swclk_pin, nrst_pin);
  swd_line_rst_switch_to_swd();
  delay(1); // give some time the target to wake up. not really needed...
  uint32_t id_code = 0;
  dap_read_id_code(&id_code);
  return id_code;
}

bool dap_read_reg(uint8_t addr, bool is_ap_reg, uint32_t * res) {
  int attempts_to_read_reg = 0;
retry_read_reg:
  uint8_t request = swd_build_request(addr, is_ap_reg, true);
  *res = 0;
  
  swd_write_line(request, 8);  // Write the request
  swd_turn_around_to_input();

  uint8_t ack = swd_read_ack(); // read the ACK
  if (ack != SWD_ACK_OK) {
    swd_turn_around_to_output();
    
    if (ack == SWD_ACK_WAIT) {
      //delayMicroseconds(10);
      if (attempts_to_read_reg < 10) {
        // Serial.print(". ");
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
  if (parity_bit != swd_calc_parity(*res)) {
    Serial.println("Parity mismatch");
    attempts_to_read_reg ++;
    goto retry_read_reg;
    return false;
  }
  // Serial.println(*res, HEX);
  return true;
}

bool dap_write_reg(uint8_t addr, bool is_ap_reg, uint32_t val) {
  int attempts_to_write_reg = 0;
retry_write_reg:
  uint8_t request = swd_build_request(addr, is_ap_reg, false);
  
  swd_write_line(request, 8);  // Write the request
  swd_turn_around_to_input();

  uint8_t ack = swd_read_ack(); // read the ACK
  if (ack != SWD_ACK_OK) {
    if (ack == SWD_ACK_WAIT) {
      // delayMicroseconds(10);
      if (attempts_to_write_reg < 10) {
        // Serial.print(". ");
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
  uint8_t parity_bit = swd_calc_parity(val);
  swd_write_line(val, 32);
  swd_write_line(parity_bit, 1);
  
  // Serial.println("ACK OK"); 
  return ack == 1;
}

///
/// Functions to read/write certain registers
/// 

bool dap_read_id_code(uint32_t * id_code) {
  return dap_read_reg(SWD_DP_REG_IDCODE, false, id_code);
}

uint32_t dap_read_ctrl_stat() {
  // Serial.print("Read CTRL/STAT: ");
  uint32_t ctrl_stat = 0;
  dap_read_reg(SWD_DP_REG_CTRL_STAT, false, &ctrl_stat);
  // Serial.println(ctrl_stat, HEX);
  return ctrl_stat;
}

uint32_t dap_read_read_buf() {
  // Serial.print("Read READ BUF: ");
  uint32_t read_buf = 0;
  dap_read_reg(SWD_DP_REG_RDBUFF, false, &read_buf);
  // Serial.print("0x"); Serial.println(read_buf, HEX);
  return read_buf;
}

bool dap_write_ctrl_stat(uint32_t val) {
  // Serial.print("Write CTRL STAT 0x");
  // Serial.println(val, HEX);
  return dap_write_reg(SWD_DP_REG_CTRL_STAT, false, val);
}

bool dap_write_abort(uint32_t val) {
  // Serial.print("Write ABORT 0x");
  // Serial.println(val, HEX);
  return dap_write_reg(SWD_DP_REG_ABORT, false, val);
}

bool dap_write_select(uint32_t val) {
  // Serial.print("Write SELECT 0x");
  // Serial.println(val, HEX);
  return dap_write_reg(SWD_DP_REG_SELECT, false, val);
}
