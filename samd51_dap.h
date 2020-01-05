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


 /**
  * Functions to halt, erase and program a SAMD51 controller. Should work for SAME51 too really
  */

// Power-up the target and its debug domain
bool samd_power_up_debug_domains();

// Stop the core
void samd_stop_core();

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

// Read the user memory block from the 
bool samd_read_user_mem(uint8_t * user_bytes);
bool samd_read_fuses(uint8_t * fuse_bytes);

bool samd_chip_erase();

/// read the unique serial number - 128 bits (16 bytes)
bool samd_read_serial_number(uint8_t * res);

// helper function

inline uint32_t bswap32(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF ); 
  return (val << 16) | (val >> 16);
}

/// NOT WORKING. YET.
bool samd_perform_mbist(uint32_t mem_start_address, uint32_t mem_length);

#ifdef __cplusplus
}
#endif


#endif // __SAMD51_DAP__
