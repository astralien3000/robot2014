#include "fpga.hpp"

#if defined (__AVR_ATmega128__)
#define F_CPU 16000000l

#include <util/delay.h>
#include <avr/io.h>

#include <hardware/xmem.hpp>
#include <hardware/interrupts.hpp>
#endif

void fpga_reset(void) {
#if defined (__AVR_ATmega128__)
  DDRB |= (1<<0);
  PORTB &= ~(1<<0);
  _delay_ms(500);
  PORTB |= (1<<0);
  _delay_ms(1);
  PORTB &= ~(1<<0);
  _delay_ms(1);
#endif
}

void fpga_init(void) {
#if defined (__AVR_ATmega128__)
  // External memory initialization
  Xmem::instance().init();
  fpga_reset();
  fpga_config();
#endif
}

void fpga_config(void) {
#if defined (__AVR_ATmega128__)
  RELATION = 15100;
#endif
}

void fpga_position_reset(void) {
  RESET = (1 << 1) | (1 << 4) | (1 << 7);
}

///////////////////////////////////////////////
// On UNIOC
#if defined (__AVR_ATmega128__)

///////////////////////////////////////////////
// NO UNIOC
#else

#warning "FPGA is not managed"

volatile u32 ENC_R     = 0;
volatile u32 ENC_L     = 0;
volatile u32 ENC_MOT_R = 0;
volatile u32 ENC_MOT_L = 0;

volatile s8 MOT_R      = 0;
volatile s8 MOT_L      = 0;
volatile u8 RESET_FPGA = 0;

volatile u32 RELATION  = 0;

// READ-ONLY
volatile u16 FPGA_US   = 0;
volatile u16 FPGA_MS   = 0;
volatile u16 FPGA_S    = 0;
volatile u16 POSX_FPGA = 0;
volatile u16 POSY_FPGA = 0;
volatile u16 ROT_FPGA  = 0;

#endif//UNIOC
