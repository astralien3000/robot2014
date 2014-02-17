#include "fpga.hpp"

void fpga_init(void) {
#if defined (__AVR_ATmega128__)
  // External memory initialization
  Xmem::instance().init();
  // FPGA manual reste
  DDRB |= (1<<0); 
  PORTB &= ~(1<<0);
  _delay_ms(500);
  PORTB |= (1<<0);
  _delay_ms(1);
  PORTB &= ~(1<<0);
#endif
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
