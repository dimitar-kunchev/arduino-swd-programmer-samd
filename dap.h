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
#include "reg_definitions.h"

#ifndef __DAP__
#define __DAP__

/*
 * The following functions provide access over SWD to the DAP DP registers. There are
 * functions to directly read/write registers and some are wrapped in easier-to-use
 * way
 * 
 * Note that these are used to further access the APs in the target, but this file only 
 * aims to provide the base DP registers access (to keep things neat and tidy)
 */

#ifdef __cplusplus
extern "C" {
#endif

#define DAP_ID_CORTEX_M4      0x2BA01477
#define DAP_ID_CORTEX_M3      0x2BA01477
#define DAP_ID_CORTEX_M0PLUS  0x0BC11477

// Initialise DAP, reset target, enter SWD mode, etc
uint32_t dap_begin_and_identify(int swdio_pin, int swclk_pin, int nrst_pin);

/// Raw register access functions

// Read a register
bool dap_read_reg(uint8_t addr, bool is_ap_reg, uint32_t * res);
// write a register
bool dap_write_reg(uint8_t addr, bool is_ap_reg, uint32_t val);


/// DP Registers access functions

// Read target IDCODE
bool dap_read_id_code(uint32_t * id_code);

// Read CTRL/STAT DP Register
uint32_t dap_read_ctrl_stat();

// Read RDBUFF DP Register
uint32_t dap_read_read_buf();

// Write CTRL/STAT DP Register
bool dap_write_ctrl_stat(uint32_t val);

// Write ABORT DP Register
bool dap_write_abort(uint32_t val);

// Write the SELECT DP Register
bool dap_write_select(uint32_t val);

// End and optionally reset the target
void dap_end (bool reset_target);

#ifdef __cplusplus
};
#endif 

#endif // __DAP__
