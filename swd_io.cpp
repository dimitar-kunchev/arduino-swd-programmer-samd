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
 
#include "swd_io.h"
#include <assert.h>

int cfg_swclk_pin, cfg_swdio_pin, cfg_swrst_pin;

volatile uint32_t * reg_clk_set;
volatile uint32_t * reg_clk_clr;
volatile uint8_t * reg_clk_pincfg;
volatile uint32_t * reg_clk_dirset;
volatile uint32_t * reg_clk_dirclr;

volatile uint32_t * reg_dat_set;
volatile uint32_t * reg_dat_clr;
volatile uint32_t * reg_dat_in_val;
volatile uint8_t * reg_dat_pincfg;
volatile uint32_t * reg_dat_dirset;
volatile uint32_t * reg_dat_dirclr;

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
#define DAT_SET_OUTPUT() *reg_dat_pincfg = PORT_PINCFG_INEN /*|PORT_PINCFG_DRVSTR*/; *reg_dat_dirset=reg_dat_mask; data_line_is_input = false;    /* pinMode(cfg_swdio_pin, OUTPUT);*/ 
#define DAT_SET_INPUT()  *reg_dat_pincfg = PORT_PINCFG_INEN|PORT_PINCFG_PULLEN; *reg_dat_dirclr=reg_dat_mask; *reg_dat_set=reg_dat_mask; data_line_is_input = true; /* pinMode(cfg_swdio_pin, INPUT_PULLUP); */ 

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
static void write_line_falling(char * buf, uint8_t bit_length) {
  char * bl = buf;
  char bit_index = 0;
  DAT_SET_OUTPUT();
  for (int i = 0; i < bit_length; i ++) {
    write_val_clock_falling ((buf[i / 8] >> (i % 8)) & 0x01);
  }
}

// Clock multiple bits with fixed data line. This is used for the line reset sequence only
static void write_line_fixed(int clocks, bool high) {
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
void swd_write_line(uint32_t val, uint8_t bit_length) {
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
uint32_t swd_read_line(uint8_t bits) {
  uint32_t res = 0;
  for (int i = 0; i < bits; i ++) {
    res |= read_dat_falling_edge() << i;
  }
  return res;
}

///
/// 
///

// a simple delay used while we are resetting the line
static inline void delay_during_rst() {
  for (int i = 0; i < 1000; i ++) {
    asm("nop");
  }
}

// Execute the line reset sequence: 
// clock 51 high bits, write the JTAG-SWD switching sequence, clock 51 more high bits, clock 8 low bits
void swd_line_rst_switch_to_swd() {
  write_line_fixed(51, true);
  CLK_HIGH();
  delay_during_rst();
  char buf[2] = {0x9E, 0xE7};
  write_line_falling(buf, 16);
  CLK_HIGH();
  delay_during_rst();
  write_line_fixed(51, true);
  CLK_HIGH();
  delay_during_rst();
  write_line_fixed(8, false);
  CLK_HIGH();
}

void swd_turn_around_to_input() {
  CLK_HIGH(); 
  CLK_DELAY();
  DAT_SET_INPUT();
  CLK_LOW();
  CLK_DELAY();
}

void swd_turn_around_to_output() {
  CLK_HIGH();
  CLK_DELAY();
  DAT_SET_OUTPUT();
  CLK_LOW();
}

////

uint8_t swd_read_ack() {
  return swd_read_line(3);
}

uint32_t swd_calc_parity(uint32_t value) {
  value ^= value >> 16;
  value ^= value >> 8;
  value ^= value >> 4;
  value &= 0x0f;
  return (0x6996 >> value) & 1;
}

uint32_t swd_build_request(uint8_t addr, bool is_ap_reg, bool is_read) {
  uint8_t request = 0; ;
  if (is_ap_reg) {
    request |= 1 << 1;            // set APnDP bit
  }
  if (is_read) {
    request |= 1 << 2;            // set RnW bit
  }
  request |= (addr & 0x0C) << 1;  // set ADDR[2:3] bits
  request |= swd_calc_parity(request) << 5; //set parity
  request |= (1 << 7) | (1);      // set start & stop bits
  return request;
}

//void set_clock_delay_us(int us) {
  //clk_delay_time = us;
//}

/// Prepare the IO pins and reset the target
void swd_prepare_pin_registers_and_reset_target (int swdio_pin, int swclk_pin, int nrst_pin) {
  cfg_swdio_pin = swdio_pin;
  cfg_swclk_pin = swclk_pin;
  cfg_swrst_pin = nrst_pin;
  
  reg_clk_set = &(PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].OUTSET.reg);
  reg_clk_clr = &(PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].OUTCLR.reg);
  reg_clk_mask = (1ul << g_APinDescription[cfg_swclk_pin].ulPin);
  reg_clk_pincfg = &(PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].PINCFG[g_APinDescription[cfg_swclk_pin].ulPin].reg);
  reg_clk_dirset = &(PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].DIRSET.reg);
  reg_clk_dirclr = &(PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].DIRCLR.reg);

  reg_dat_set = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].OUTSET.reg);
  reg_dat_clr = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].OUTCLR.reg);
  reg_dat_in_val = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].IN.reg);
  reg_dat_mask = (1ul << g_APinDescription[cfg_swdio_pin].ulPin);
  reg_dat_pincfg = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].PINCFG[g_APinDescription[cfg_swdio_pin].ulPin].reg);
  reg_dat_dirset = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].DIRSET.reg);
  reg_dat_dirclr = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].DIRCLR.reg);

  // We use the toggle register to match the edges of the clock and data 
  assert(g_APinDescription[cfg_swdio_pin].ulPort == g_APinDescription[cfg_swclk_pin].ulPort); 
  reg_out_clk_tgl = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].OUTTGL.reg);
  reg_out_clk_tgl_mask = (1ul << g_APinDescription[cfg_swdio_pin].ulPin) | (1ul << g_APinDescription[cfg_swclk_pin].ulPin);

  //pinMode(cfg_swrst_pin, OUTPUT);
  PORT->Group[g_APinDescription[cfg_swrst_pin].ulPort].DIRSET.reg = 1 << g_APinDescription[cfg_swrst_pin].ulPin;
  //digitalWrite(cfg_swrst_pin, LOW);
  PORT->Group[g_APinDescription[cfg_swrst_pin].ulPort].OUTCLR.reg = 1 << g_APinDescription[cfg_swrst_pin].ulPin;
  
  //pinMode(cfg_swclk_pin, OUTPUT);
  // Use high drive strength for the clock! The internal pull-up of the target is sometimes too strong for us
  //PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].PINCFG[g_APinDescription[cfg_swclk_pin].ulPin].bit.DRVSTR = 1;
  *reg_clk_pincfg = PORT_PINCFG_INEN|PORT_PINCFG_DRVSTR; // no pull, no mux; input enable to allow read-backs
  *reg_clk_dirset = reg_clk_mask;
  
  CLK_LOW(); // set it low for cold-plugging of the debugger. We can set it high just as well - doesn't really matter
  DAT_LOW();
  DAT_SET_OUTPUT();

  delay(125);
  //digitalWrite(cfg_swrst_pin, HIGH);
  PORT->Group[g_APinDescription[cfg_swrst_pin].ulPort].OUTSET.reg = 1 << g_APinDescription[cfg_swrst_pin].ulPin;
  delay(125);
}
