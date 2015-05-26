#include <aversive.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"

#include <math/trigo.hpp>

#define F_CPU 16000000l
#include <util/delay.h>

static Scheduler& sched = Scheduler::instance();

static Vect<2, s32> cmd;

static void control_init(void) {
  Task t([]() {
      Vect<2, s32> _cmd = cmd;
      s32 angle_err = (cmd.coord(1) - odo.getValue().coord(1));
      // if(angle_err != 0) {
      // 	_cmd.coord(0) = Math::abs((_cmd.coord(0)) / (angle_err >> 5));
      // }
      _cmd.coord(0) = Math::abs(_cmd.coord(0));
      _cmd.coord(1) = - cmd.coord(1);
      robot.setValue(_cmd);
    });

  t.setPeriod(64000);
  t.setRepeat();
  
  sched.addTask(t);
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  // Need to be the first thing to be done (for security)
  fpga_init();
  MOT_R = 0;
  MOT_L = 0;

  asserv_init();

  robot.lock();

  mot_l.inverse();
  enc_r.inverse();

  control_init();
  Interrupts::set();

  robot.setLockable(false);
  robot.unlock();

  asserv_speed_slow();

  Vect<2, s32> dir;
  s32 last_angle = 0;
  while(1) {
    io << "CMD ?\n";

    io >> dir.coord(0);
    io << dir.coord(0) << "\n";

    io >> dir.coord(1);
    io << dir.coord(1) << "\n";

    if(dir.norm() != 0) {
      s32 angle = Math::atan2<Math::DEGREE, s32>(dir.coord(1), dir.coord(0));
      cmd.coord(1) = angle << 4;
      cmd.coord(0) = dir.norm();
      last_angle = angle;
    }
    else {
      cmd.coord(1) = last_angle;
      cmd.coord(0) = 0;
    }

  }

  while(1);

  return 0;
}
