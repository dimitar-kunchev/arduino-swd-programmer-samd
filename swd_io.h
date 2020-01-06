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
#ifndef __SWD_IO__
#define __SWD_IO__

/*
 * The following functions directly control the IO. They wrap to some very basic-level 
 * functions such as sending and receiving bits on the line.
 * 
 * The only more complicated functions are the line reset and the pin registers preparation
 * 
 * In addition there are helper functions to build a register request and calculate parity for the SWD protocol
 * 
 * Note that this code is "decoupled" from the standard Arduino functions 
 */

#ifdef __cplusplus
extern "C" {
#endif

// Perform a line reset and the JTAG-SWD switching sequence
void swd_line_rst_switch_to_swd();

// Send up to 32 bits over the line
void swd_write_line(uint32_t val, uint8_t bit_length);

// Turn around from writing to reading
void swd_turn_around_to_input();

// Turn around from reading to writing
void swd_turn_around_to_output();

// Read 3-bit ACK response. 1 is OK, 2 is WAIT, 4 is ERROR
uint8_t swd_read_ack();

// Read up to 32 bits from the line
uint32_t swd_read_line(uint8_t bits);

//void set_clock_delay_us(int us);

// Setup the pins to use for the SWD and reset the target. IMPORTANT: swdio and swclk must be in the same port group (PA, PB, etc) - this lets us use the TGL register to match the rising/falling edges
void swd_prepare_pin_registers_and_reset_target (int swdio_pin, int swclk_pin, int nrst_pin);

// Calculate parity bit over a value (up to 32 bits)
uint32_t swd_calc_parity(uint32_t value);

// Build a request header for sending over the wire. addr is the register address (lowest 2 bits are ignored!), is_ap_reg - whether request is for AP (or DP) register, is_read - whether it is read request
uint32_t swd_build_request(uint8_t addr, bool is_ap_reg, bool is_read);

// Release the pins (set them back to input, optionally reset the target
void swd_end(bool reset_target);

#endif // __SWD_IO__

#ifdef __cplusplus
}
#endif
