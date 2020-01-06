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
 #include "dap_mem_ap.h"

#ifndef __SAMD51_DAP__
#define __SAMD51_DAP__

#ifdef __cplusplus
extern "C" {
#endif

#define SAMD_PAGE_SIZE  512

 /**
  * Functions to halt, erase and program a SAMD51 controller. Should work for SAME51 too really
  */

// Power-up the target and its debug domain
bool samd_power_up_debug_domains();

// Stop the core
void samd_stop_core();

/// DSU FUNCTIONS

// Read the device identifier from DSU memory area
uint32_t samd_read_dsu_did();

// Read the CTRL/StatusA/StatusB registers from the DSU memory area
// The returned uint32 has as its right-most octet the CTRL bits (LSB is CTRL)
uint32_t samd_read_dsu_ctrl_status();
uint32_t samd_write_dsu_ctrl_status(uint32_t val);
inline uint8_t samd_read_dsu_ctrl() {
  return samd_read_dsu_ctrl_status() & 0xff;
}
inline uint8_t samd_read_dsu_status_a() {
  return (samd_read_dsu_ctrl_status() >> 8) & 0xff;
}
inline uint8_t samd_read_dsu_status_b() {
  return (samd_read_dsu_ctrl_status() >> 16) & 0xff;
}

/// I cannot tell if this is working or not. I think it completes too quickly really. Also at the end there is BUS ERROR flag in the DSU
bool samd_perform_mbist(uint32_t mem_start_address, uint32_t mem_length);

/// MEMORY FUNCTIONS

// Read the user memory block from the 
bool samd_read_user_mem(uint8_t * user_bytes);
/// ERASE THE USER MEMORY AREA! PLEASE MAKE SURE YOU READ THE FUSES AND CALIBRATION DATA AND WRITE THEM BACK AGAIN AFTER ERASING!
bool samd_erase_user_mem();
// Write the the user area of the memory. writes are quad-word, so bytes length MUST BE MULTIPLE OF 16!
bool samd_write_user_mem(uint32_t address_offset, uint8_t * data, uint32_t length);

bool samd_read_fuses(uint8_t * fuse_bytes);

// This is just and example really.
inline bool samd_write_fuses(uint8_t * fuse_bytes) {
  samd_write_user_mem(0, fuse_bytes, 16);
}

bool samd_chip_erase();

/// read the unique serial number - 128 bits (16 bytes)
bool samd_read_serial_number(uint8_t * res);

inline bool samd_read_flash_memory(uint32_t offset_address, uint8_t * buf, uint32_t length) {
  return dap_read_mem_block(0x00 + offset_address, buf, length);
}

bool samd_prepare_for_programming();
bool samd_write_flash_page(uint32_t address, const uint8_t * buf, uint32_t length); // Write up to a page (SAMD_PAGE_SIZE) in flash memory. Actually any address is accepted, just make sure you use it with a flash address.
bool samd_write_flash(uint32_t offset_address, const uint8_t * buf, uint32_t length);
bool samd_end_programming(uint32_t * computed_crc, uint32_t start_memory_address, uint32_t total_memory_size);  // end programming and calculate CRC32 of the size of memory you need (usually the size of the upload)

// helper function

inline uint32_t bswap32(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF ); 
  return (val << 16) | (val >> 16);
}

void samd_start_core(); // used when you have finished tyour business

#ifdef __cplusplus
}
#endif


#endif // __SAMD51_DAP__
