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
 
//#include "dap.h"
//#include "dap_mem_ap.h"
#include "samd51_dap.h"
#include "test_fw.h"

#define SWDIO 9
#define SWCLK 8
#define SWRST 10

///

void test_fuses_read() {
  Serial.println("Read fuses");
  uint8_t fuse_bytes[32];
  samd_read_fuses(fuse_bytes);
  Serial.println("Fuses contents:");
  for (int i = 0; i < 32; i ++) {
    if (i%4 == 0) {
      Serial.print(" 0x");
    }
    Serial.print(fuse_bytes[i], HEX);
  }
  Serial.println();
}

void test_chip_erase() {
  Serial.println("Erasing chip...");
  samd_chip_erase();
  Serial.println("Done, chip erased");
}

void test_read_nvm_cpu_sn() {
  uint8_t serial_number[16];
  if (samd_read_serial_number(serial_number)) {
    Serial.print("S/N: ");
    for (uint8_t i = 0; i < 16; i ++) {
      if (serial_number[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(serial_number[i], HEX);
    }
    Serial.println();
  } else {
    Serial.println("Error reading serial number");
  }
}

void test_rw_user_memory(){ 

  //samd_perform_mbist(0x20000000, 256*1024);

  samd_erase_user_mem();

  const int bytes_to_test = 32;
  const int offset_to_test = 0x100;
  uint8_t tmp_user_area_buff[bytes_to_test];
  dap_read_mem_block(USER_ROW_ADDR + offset_to_test, tmp_user_area_buff, bytes_to_test);

  // read
  Serial.print("Sample user memory, offset 0x"); Serial.println(offset_to_test, HEX);
  for (int i = 0; i < bytes_to_test; i ++) {
    if (tmp_user_area_buff[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(tmp_user_area_buff[i], HEX);
    if (i % 4 == 3) { Serial.print (" "); }
  }
  Serial.println();

  // flip the words
  for (int i = 0; i < bytes_to_test; i +=4) {
    uint32_t t;
    memcpy(&t, &(tmp_user_area_buff[i]), 4);
    t = bswap32(t);
    memcpy(&(tmp_user_area_buff[i]), &t, 4);
  }

  // change
  memset(tmp_user_area_buff, 0x00, 4);
  memset(tmp_user_area_buff+4, 0x10, 4);
  memset(tmp_user_area_buff+8, 0x01, 4);
  memset(tmp_user_area_buff+12, 0b01010101, 4);
  memset(tmp_user_area_buff+16, 0b10101010, 4);
  memset(tmp_user_area_buff+20, 0b10101010, 4);
  memset(tmp_user_area_buff+24, 0xFFFFFFFF, 4);
  memset(tmp_user_area_buff+28, 0x00000000, 4);

  // write
  samd_write_user_mem(offset_to_test, tmp_user_area_buff, bytes_to_test);

  // read 

  dap_read_mem_block(USER_ROW_ADDR+offset_to_test, tmp_user_area_buff, bytes_to_test);

  Serial.print("Sample user memory, offset 0x"); Serial.println(offset_to_test, HEX);
  for (int i = 0; i < bytes_to_test; i ++) {
    if (tmp_user_area_buff[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(tmp_user_area_buff[i], HEX);
    if (i % 4 == 3) { Serial.print (" "); }
  }
  Serial.println();
}

void test_mbist() {
  if (samd_perform_mbist(0x20000000, 256*1024)) {
    Serial.println("MBIST ran OK");
  } else {
    Serial.println("MBIST RETURNED ERROR!");
  }
}

void test_read_firmware() {
  const int test_size = 4096;
  uint8_t fw_data[test_size];
  samd_read_flash_memory(0x00, fw_data, test_size);
  Serial.println(" ");
  for (int i = 0; i < test_size; i ++) {
    if (fw_data[i] < 0x10) { Serial.print("0"); };
    Serial.print(fw_data[i], HEX);
    if (i%16 == 15) {
      Serial.println();
    }
  }
}

void test_write_firmware() {
  samd_prepare_for_programming();
  //
  // REPLACE THIS WITH ACTUAL FIRMWARE!
  // const int fw_contents_len = 128;
  // uint8_t fw_contents[fw_contents_len];
  // memset(fw_contents, 0, fw_contents_len);
  //
  samd_write_flash(0, fw_contents, fw_contents_len);
  Serial.println("Done. Verifying");
  uint32_t crc;
  samd_end_programming(&crc, 0, fw_contents_len);
  Serial.print("CRC is 0x");Serial.println(crc, HEX);
  Serial.print("CRC xor 0xFFFFFFFF is 0x");Serial.println(crc^0xFFFFFFFF, HEX);
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
//  swd_prepare_pin_registers_and_reset_target(SWDIO, SWCLK, SWRST);
//  swd_line_rst_switch_to_swd();
//  
//  delay(1); // give some time the target to wake up. not really needed...
//
//  // let's get busy
//  read_id_code();
  uint32_t target_id_code = dap_begin_and_identify(SWDIO, SWCLK, SWRST);
  if (target_id_code != DAP_ID_CORTEX_M4) {
    Serial.println("DAP startup error");
    while (1);
  } else {
    Serial.println("Cortex M4 DAP connected");
  }
  
  uint32_t ctrl_stat = dap_read_ctrl_stat();
  dap_read_read_buf();

  dap_write_select(0x00); // select the mem/ap
  dap_read_read_buf();  // should return 0

  Serial.println("Power-up the system and debug domains");
  if (!samd_power_up_debug_domains()) {
    Serial.println("Failed to power up the system and debug domains");
    while (1);
  }

  // check if there are any stick errors set and clear them if needed
  ctrl_stat = dap_read_ctrl_stat();
  if (ctrl_stat & (SWD_REG_CTRL_STAT_WDATAERR|SWD_REG_CTRL_STAT_STICKYERR|SWD_REG_CTRL_STAT_STICKYCMP|SWD_REG_CTRL_STAT_STICKYORUN)) {
    dap_write_abort(SWD_REG_ABORT_Clear_All);
  }
  
  /*
  // Debug start request
  // Technically what we should do is: write bit 26 (CDBGRSTREQ) of the CTRL/STAT, wait until bit 27 (CDBGRSTACK) is set and then clear bit 26. 
  // However it seems the Atmel ICE gives up after few ms
  // It would appear this is NOT actually implemented in the CPU.
  Serial.println("Attempt debugger core reset\r\n");
  for (int i = 0; i < 20; i ++){
    dap_write_ctrl_stat(SWD_REG_CTRL_STAT_CSYSPWRUPREQ | SWD_REG_CTRL_STAT_CDBGPWRUPREQ | SWD_REG_CTRL_STAT_CDBGRSTREQ);
    dap_read_read_buf();  // should return 0
    ctrl_stat = dap_read_ctrl_stat();
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
  dap_write_ctrl_stat(SWD_REG_CTRL_STAT_CSYSPWRUPREQ | SWD_REG_CTRL_STAT_CDBGPWRUPREQ); // | SWD_REG_CTRL_STAT_MASKLANE(0xF)
  dap_read_read_buf();
  ctrl_stat = dap_read_ctrl_stat();
  */
  
  dap_write_csw(SWD_REG_MEM_AP_CSW_Prot(35) | SWD_REG_MEM_AP_CSW_AddrInc_Single | SWD_REG_MEM_AP_CSW_Size_Word);
  dap_read_read_buf();

  samd_stop_core();
  Serial.println("Core stopped");
  
  // Read device ID. Takes few attempts sometimes and raises sticky error. Don't know why
  uint32_t device_id = 0;
  for (int i = 0; i < 10; i ++) {
    device_id = samd_read_dsu_did();
    if (device_id == 0) { // should be 0x60060004 for SAMD51J20A
      Serial.println("DID read failed, retry");
      ctrl_stat = dap_read_ctrl_stat();
      if (ctrl_stat & SWD_REG_CTRL_STAT_STICKYERR) {
        dap_write_abort(SWD_REG_ABORT_Clear_All);
        dap_read_ctrl_stat();
      }
    } else {
      break;
    }
  }
  if (device_id == 0) {
    Serial.println("DID read failed, giving up");
    while (1);
  }
  Serial.print("DSU Device ID: 0x"); Serial.println(device_id);

  // Check if DeviceEn bit is set 
  uint32_t dap_csw = dap_read_csw();
  Serial.print("CSW: 0x"); Serial.println(dap_csw);
  if (! (dap_csw & SWD_REG_MEM_AP_CSW_DeviceEn)) {
    Serial.println("CSW.DeviceEn bit not set - error");
    while(1);
  }

  // Check if device security bit is set
  uint8_t dsu_status_b = samd_read_dsu_status_b();
  if (dsu_status_b & 0x01) {
    Serial.println("Security bit is set");
    while(1);
  }
  if (dsu_status_b & (0x01 << 1)) {
    Serial.println("DSU reports debugger connected - OK");
  }

  // test_fuses_read();
  // test_chip_erase();
  // test_read_nvm_cpu_sn();
  // test_rw_user_memory();

  //test_mbist();

  //test_read_firmware();
  test_chip_erase();
  test_write_firmware();
  // test_read_firmware();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
}
