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
#include <assert.h>

int cfg_swclk_pin, cfg_swdio_pin, cfg_swrst_pin;

volatile uint32_t * reg_clk_set;
volatile uint32_t * reg_clk_clr;
volatile uint32_t * reg_dat_set;
volatile uint32_t * reg_dat_clr;
volatile uint32_t * reg_dat_in_val;

volatile uint32_t * reg_out_clk_tgl;
uint32_t reg_out_clk_tgl_mask;
uint32_t reg_clk_mask;
uint32_t reg_dat_mask;

// uint32_t clk_delay_time = 0;

#define CLK_HIGH() *reg_clk_set=reg_clk_mask
#define CLK_LOW()  *reg_clk_clr=reg_clk_mask
#define CLK_DELAY()  __asm("nop\n\t""nop\n\t""nop\n\t"); // delayMicroseconds(clk_delay_time);

#define DAT_HIGH()  *reg_dat_set=reg_dat_mask; data_is_high=true;
#define DAT_LOW()   *reg_dat_clr=reg_dat_mask; data_is_high=false;

// @TODO: These should use direct registers instead
#define DAT_SET_OUTPUT() pinMode(cfg_swdio_pin, OUTPUT); data_line_is_input = false;
#define DAT_SET_INPUT()  pinMode(cfg_swdio_pin, INPUT_PULLUP); data_line_is_input = true;

#define DAT_GET_VAL() ((*reg_dat_in_val & reg_dat_mask)!=0)

#define DAT_CLK_TGL() *reg_out_clk_tgl=reg_out_clk_tgl_mask; data_is_high=!data_is_high;

volatile bool data_is_high = false;
volatile bool data_line_is_input = false;

///
/// The following functions are used only during the initial reset and JTAG-SWD switching. Writing is done a little different there
///

// Write a bit on the line
static inline void write_val_clock_falling(bool high) {
  if (high) {
    DAT_HIGH();
  } else {
    DAT_LOW();
  }
  CLK_LOW();
  data_is_high = high;
  CLK_DELAY();
  CLK_HIGH();
  CLK_DELAY();
}

// Write multiple bits over the line. This should be used only for the JTAG-SWD switching
void write_line_falling(char * buf, uint8_t bit_length) {
  char * bl = buf;
  char bit_index = 0;
  DAT_SET_OUTPUT();
  for (int i = 0; i < bit_length; i ++) {
    write_val_clock_falling ((buf[i / 8] >> (i % 8)) & 0x01);
  }
}

// Clock multiple bits with fixed data line. This is used for the line reset sequence only
void write_line_fixed(int clocks, bool high) {
  if (high) {
    DAT_HIGH();
  } else {
    DAT_LOW();
  }
  DAT_SET_OUTPUT();
  for (int i = 0; i < clocks; i ++) {
    CLK_HIGH();
    CLK_DELAY();
    CLK_LOW();
    CLK_DELAY();
  }
}

///
/// The following functions are used for the normal read/write process. These are the low-order functions that control the line directly
///

// Write a val at the risong edge of the clock, wait and bring it down again so the target can read it with the falling edge
static inline void write_val_clock_rising(bool high) {
  if (high == data_is_high) {
    CLK_HIGH();
  } else {
    DAT_CLK_TGL();
  }
  data_is_high = high;
  CLK_DELAY();
  CLK_LOW();
  CLK_DELAY();
}

// Write multiple bits on the line (up to 32)
void write_line(uint32_t val, uint8_t bit_length) {
  if (data_line_is_input) {
    DAT_SET_OUTPUT();
    DAT_HIGH();
    CLK_HIGH();
    CLK_DELAY();
  }
  for (int i = 0; i < bit_length; i ++) {
    write_val_clock_rising (val & 0x1);
    val >>= 1;
  }
}

// Read a bit from the target right after the falling edge of the clock
static uint8_t inline read_dat_falling_edge() {
  CLK_HIGH();
  CLK_DELAY();
  CLK_LOW();
  bool res = DAT_GET_VAL();
  CLK_DELAY();
  return res ? 1 : 0;
}

/*
static uint8_t inline read_dat_rising_edge() {
  CLK_LOW();
  CLK_DELAY();
  bool res = DAT_GET_VAL();
  CLK_HIGH();
  CLK_DELAY();
  return res ? 1 : 0;
}*/

// Read up to 32 bits from the line
uint32_t read_line(uint8_t bits) {
  uint32_t res = 0;
  for (int i = 0; i < bits; i ++) {
    res |= read_dat_falling_edge() << i;
  }
  return res;
}

///
/// 
///

// Execute the line reset sequence: 
// clock 51 high bits, write the JTAG-SWD switching sequence, clock 51 more high bits, clock 8 low bits
void line_rst_switch_to_swd() {
  write_line_fixed(51, true);
  CLK_HIGH();
  delay(1);
  char buf[2] = {0x9E, 0xE7};
  write_line_falling(buf, 16);
  CLK_HIGH();
  delay(1);
  write_line_fixed(51, true);
  CLK_HIGH();
  delay(1);
  write_line_fixed(8, false);
  CLK_HIGH();
}

void turn_around_to_input() {
  CLK_HIGH(); 
  CLK_DELAY();
  DAT_SET_INPUT();
  CLK_LOW();
  CLK_DELAY();
}

void turn_around_to_output() {
  CLK_HIGH();
  CLK_DELAY();
  DAT_SET_OUTPUT();
  CLK_LOW();
}

////

uint8_t read_ack() {
  return read_line(3);
}
//void set_clock_delay_us(int us) {
  //clk_delay_time = us;
//}

/// Prepare the IO pins and reset the target
void prepare_pin_registers_and_reset_target (int swdio_pin, int swclk_pin, int nrst_pin) {
  cfg_swdio_pin = swdio_pin;
  cfg_swclk_pin = swclk_pin;
  cfg_swrst_pin = nrst_pin;
  
  reg_clk_set = &(PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].OUTSET.reg);
  reg_clk_clr = &(PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].OUTCLR.reg);
  reg_clk_mask = (1ul << g_APinDescription[cfg_swclk_pin].ulPin);

  reg_dat_set = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].OUTSET.reg);
  reg_dat_clr = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].OUTCLR.reg);
  reg_dat_in_val = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].IN.reg);
  reg_dat_mask = (1ul << g_APinDescription[cfg_swdio_pin].ulPin);

  // We use the toggle register to match the edges of the clock and data 
  assert(g_APinDescription[cfg_swdio_pin].ulPort == g_APinDescription[cfg_swclk_pin].ulPort); 
  reg_out_clk_tgl = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].OUTTGL.reg);
  reg_out_clk_tgl_mask = (1ul << g_APinDescription[cfg_swdio_pin].ulPin) | (1ul << g_APinDescription[cfg_swclk_pin].ulPin);

  pinMode(cfg_swrst_pin, OUTPUT);
  digitalWrite(cfg_swrst_pin, LOW);
  
  pinMode(cfg_swclk_pin, OUTPUT);
  // Use high drive strength for the clock! The internal pull-up of the target is sometimes too strong for us
  PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].PINCFG[g_APinDescription[cfg_swclk_pin].ulPin].bit.DRVSTR = 1;
  
  CLK_LOW(); // set it low for cold-plugging of the debugger. We can set it high just as well - doesn't really matter
  DAT_LOW();
  DAT_SET_OUTPUT();

  delay(125);
  digitalWrite(cfg_swrst_pin, HIGH);
  delay(125);
}
