#include <aversive.hpp>
#include <device/stream/buffered_uart_stream.hpp>
#include <hardware/interrupts.hpp>
#include <hardware/timer.hpp>

#include <avr/io.h>

inline bool isOn(u16 p) {
  return (PORTB & (1<<p));
}

inline void turnOn(u16 p) {
  PORTB = PORTB | (1<<p);
  __asm__ __volatile__ ("NOP\n");
}

inline void turnOff(u16 p) {
  PORTB = PORTB & ~(1<<p);
  __asm__ __volatile__ ("NOP\n");
}

void toggle(u16 p) {
  if(isOn(p)) {
    turnOff(p);
    return;
  }
  else {
    turnOn(p);
    return;
  }
}

void synchronize(InternalBufferedStream& str) {
  u8 c;
  do {
    for(c = 0; c < 4; c++) {
      str << c;
    }
    str.flushOutput();
    str >> c;
    for(u32 i = 0; i < 400000; i++) {
      __asm__ __volatile__ ("NOP\n");
    }
  } while(c != 'K');
  c = 'K';
  str << c;
}

volatile s32 x;
volatile s32 y;
volatile s32 dest_x;
volatile s32 dest_y;

void update(void) {
  if(y > dest_y) {
    y -= 5;
  }
  else if(y < dest_y) {
    y += 5;
  }
  else if(x > dest_x) {
    x -= 5;
  }
  else if(x < dest_x) {
    x += 5;
  }
}

int main(int argc, char** argv) {
  (void) argc;
  (void) argv;
  
  DDRB = (1<<PB7) | (1<<PB4) | (1<<PB5);
  Aversive::init();
  BufferedUartStream<0>& str = BufferedUartStream<0>::instance();
  str.setMode(Stream::BINARY);
  Interrupts::init();
  
  synchronize(str);
  
  x = -1300;
  y = 950;
  
  Timer<0>& t = Timer<0>::instance();
  t.init();
  t.setPrescaler<1024>();
  t.overflowEvent().setFunction(update);
  t.overflowEvent().start();
  
  s32 tmp_x, tmp_y;
  while(Aversive::sync()) {
    str >> tmp_x >> tmp_y;
    Interrupts::lock();
    dest_x = tmp_x;
    dest_y = tmp_y;
    tmp_x = x;
    tmp_y = y;
    Interrupts::unlock();
    str << tmp_x << tmp_y;
    str.flushOutput();
  }
  
  Aversive::setReturnCode(0);
  return Aversive::exit();
}
