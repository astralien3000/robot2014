#include "rds.hpp"

#include "devices.hpp"
#include <math/trigo.hpp>
#include "fpga.hpp"

FpgaUartStream rds_io("rds_stream", UART_TX_1_DATA, UART_TX_1_OCUP, UART_RX_1_DATA, UART_RX_1_AVA);

Rds rds("", rds_io);

void rds_init(void) {
  //rds_io.setMode(Stream::BINARY);
}

void check_for_collision(void) {
  rds.update();
  List<2, Vect<2, s32> > adv = rds.getValue();
  bool can_unlock = true;
  io << "detected : " << adv.usedSpace() << "\n";
  for (int i=0; i< (int)adv.usedSpace(); i++) {
    io << i << " : " << adv.get(i).coord(0) << " , " << adv.get(i).coord(1) << "\n";
    s32 abs_angle = 180 - adv.get(i).coord(1) + (odo.getValue().coord(1) >> 4);
    s32 rel_x = (s32)(adv.get(i).coord(0) * Math::cos<Math::DEGREE, double>(abs_angle));
    s32 rel_y = (s32)(adv.get(i).coord(0) * Math::sin<Math::DEGREE, double>(abs_angle));
    s32 abs_x = pos.getValue().coord(0) + rel_x * 10;
    s32 abs_y = pos.getValue().coord(1) + rel_y * 10;
    io << "rel_x = " << rel_x << " \trel_y = " << rel_y << "\n";
    io << "abs_x = " << abs_x << " \tabs_y = " << abs_y << "\n";

    if (adv.get(i).coord(0) < 60 &&
	(adv.get(i).coord(1) < 60 || adv.get(i).coord(1) > 300)) {
      //robot.lock();
      can_unlock = false;
    }
  }
  if (robot.getValue() && can_unlock) {
    //    trajectory_reset();
    //robot.unlock();
  }
}
