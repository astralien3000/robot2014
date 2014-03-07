// #include <filter/pid_filter.hpp>
// #include <filter/diff_filter.hpp>
// #include <filter/quadramp_filter.hpp>
// #include <filter/feedback_loop_filter.hpp>
// #include <filter/composed_filter.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"

Scheduler& sched = Scheduler::instance();

#include <filter/diff_filter.hpp>
bool val = false;
DiffFilter diff;
s32 enc_ext = 0;

void control_init(void) {
#if defined (__AVR_ATmega128__)
  Task t([](void) {
      robot.setValue(cmd);
      //pos.update();
    });

  t.setPeriod(8000);
  t.setRepeat();
  sched.addTask(t);

  Interrupts::set();
#endif
}

#if defined (__AVR_ATmega128__)
extern "C" void __cxa_pure_virtual() { while(1); }
#endif

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  Aversive::init();
  asserv_init();
  fpga_init();

  file.setMode(Stream::BINARY);
  
  MOT_R = 0;
  MOT_L = 0;

  cmd.coord(0) = 0;
  cmd.coord(1) = 0;

  control_init();

  RELATION = 15100;

  motc_l.inverse();
  enc_r.inverse();
  
  //motc_l.setValue(50);
  //motc_r.setValue(50);
  
  //cmd.coord(0) = 300;
  //cmd.coord(1) = 90;
  
  while(Aversive::sync()) {
    //cmd_print_infos();
    //cmd_print_pos();
    //cmd_pid_set();
    cmd_dist_angle();
    //cmd_odo_config();
    
    if(robot.getValue()) {
      io << "Skating !!! \n";
      robot.unlock();
    }
  }

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
