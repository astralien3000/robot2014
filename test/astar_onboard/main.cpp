#include "astar.hpp"
#include <math/vect.hpp>
//#include "../../src/devices.hpp"
#include <device/stream/buffered_uart_stream.hpp>
#include <device/stream/uart_stream.hpp>

#define F_CPU 16000000l
#include <util/delay.h>

int main(int argc, char* argv[]) {
  (void) argc;
  (void) argv;

 
  _delay_ms(2000);
  //BufferedUartStream<0>& io = BufferedUartStream<0>::instance();
  UartStream<0> io(0);
  _delay_ms(2000);
  // io >> argc;
  io << "Begin astar\n";

  Astar astar(11); //11 veut rien dire :)
  Vect<2, s32> source, target;
  source[0] = -1200;
  source[1] = -700;
  target[0] = 1100;
  target[1] = 600;
  Vect<2, s32>* path = astar.getTrajectory(source, target);

  for (uint8_t i=astar.getPathLengh(); i > 0; i--) {
    //io << i << "\n";
    io << path[i-1][0] << " " << path[i-1][1] << "\n";
  }

  return 0;
}
