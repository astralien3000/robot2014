#include <aversive.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"

static Scheduler& sched = Scheduler::instance();

Vect<2, s32> cmd(0, 0);

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  fpga_init();
  MOT_R = 0;
  MOT_L = 0;

  Aversive::init();
  asserv_init();

  robot.lock();

  mot_l.inverse();
  enc_r.inverse();

  Task t([](void) {
      robot.setValue(cmd);
    });

  t.setPeriod(64000);
  t.setRepeat();
  sched.addTask(t);

  io << "----------------\n";
  io << "angle (odo) = " << (odo.getValue().coord(1) >> 4) << "\n";
  io << "angle (pos) = " << pos.angle() << "\n";
  io << "dist  = " << odo.getValue().coord(0) << "\n";
  io << "x = " << pos.getValue().coord(0) << "\n";
  io << "y  = " << pos.getValue().coord(1) << "\n";

  cmd.coord(0) = 1900 - 300 - 250;
  robot.unlock();
  Interrupts::init();

  while(Aversive::sync()) {
    s16 dummy = 0;
    io >> dummy;
    io << "----------------\n";
    io << "angle (odo) = " << (odo.getValue().coord(1) >> 4) << "\n";
    io << "angle (pos) = " << pos.angle() << "\n";
    io << "dist  = " << odo.getValue().coord(0) << "\n";
    io << "x = " << pos.getValue().coord(0) << "\n";
    io << "y  = " << pos.getValue().coord(1) << "\n";
  }

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
