
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

#define DAT_SET_OUTPUT() pinMode(cfg_swdio_pin, OUTPUT); data_line_is_input = false;
#define DAT_SET_INPUT()  pinMode(cfg_swdio_pin, INPUT_PULLUP); data_line_is_input = true;

#define DAT_GET_VAL() ((*reg_dat_in_val & reg_dat_mask)!=0)

#define DAT_CLK_TGL() *reg_out_clk_tgl=reg_out_clk_tgl_mask; data_is_high=!data_is_high;

volatile bool data_is_high = false;
volatile bool data_line_is_input = false;

//used only during initial setup
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

void write_line_falling(char * buf, uint8_t bit_length) {
  char * bl = buf;
  char bit_index = 0;
  DAT_SET_OUTPUT();
  for (int i = 0; i < bit_length; i ++) {
    write_val_clock_falling ((buf[i / 8] >> (i % 8)) & 0x01);
  }
}

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

// the usual write functions

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

uint32_t read_line(uint8_t bits) {
  uint32_t res = 0;
  for (int i = 0; i < bits; i ++) {
    res |= read_dat_falling_edge() << i;
  }
  return res;
}

///

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

uint8_t read_ack() {
  return read_line(3);
}
//void set_clock_delay_us(int us) {
  //clk_delay_time = us;
//}

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

  assert(g_APinDescription[cfg_swdio_pin].ulPort == g_APinDescription[cfg_swclk_pin].ulPort); 
  reg_out_clk_tgl = &(PORT->Group[g_APinDescription[cfg_swdio_pin].ulPort].OUTTGL.reg);
  reg_out_clk_tgl_mask = (1ul << g_APinDescription[cfg_swdio_pin].ulPin) | (1ul << g_APinDescription[cfg_swclk_pin].ulPin);

  pinMode(cfg_swrst_pin, OUTPUT);
  digitalWrite(cfg_swrst_pin, LOW);
  
  pinMode(cfg_swclk_pin, OUTPUT);
  PORT->Group[g_APinDescription[cfg_swclk_pin].ulPort].PINCFG[g_APinDescription[cfg_swclk_pin].ulPin].bit.DRVSTR = 1;
  
  CLK_HIGH();
  DAT_LOW();
  DAT_SET_OUTPUT();

  delay(125);
  digitalWrite(cfg_swrst_pin, HIGH);
  delay(125);
}
