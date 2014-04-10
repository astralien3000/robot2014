#include "astar.hpp"
#include <math/vect.hpp>
#include "../../src/devices.hpp"

UartStream<0> io("");

int main(void) {
  Astar astar(11); //11 veut rien dire :)
  Vect<2, s32> source, target;
  source[0] = -1200;
  source[1] = -700;
  target[0] = 1200;
  target[1] = 600;
  Vect<2, s32>* path = astar.getTrajectory(source, target);

  for (uint8_t i=astar.getPathLengh(); i > 0; i--) {
    io << path[i-1][0] << " " << path[i-1][1] << "\n";
  }

  return 0;
}
