#include <Arduino.h>
#ifndef __LINE_WRITE__
#define __LINE_WRITE__


#ifdef __cplusplus
extern "C" {
#endif

void line_rst_switch_to_swd();

void write_line(uint32_t val, uint8_t bit_length);

void turn_around_to_input();
void turn_around_to_output();

uint8_t read_ack();

uint32_t read_line(uint8_t bits);

//void set_clock_delay_us(int us);

void prepare_pin_registers_and_reset_target (int swdio_pin, int swclk_pin, int nrst_pin);

#endif // __LINE_WRITE__

#ifdef __cplusplus
}
#endif
