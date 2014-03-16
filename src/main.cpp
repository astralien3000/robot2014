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

#include "trajectory_manager.hpp"

Scheduler& sched = Scheduler::instance();

PidFilter t_pid;
TrajectoryManager traj(robot, odo, pos, t_pid);

FpgaUartStream rds_stream("rds_stream", UART_TX_1_DATA, UART_TX_1_OCUP, UART_RX_1_DATA, UART_RX_1_AVA);

#include <filter/diff_filter.hpp>

void control_init(void) {
#if defined (__AVR_ATmega128__)
  Task t([](void) {
      //robot.setValue(cmd);
      traj.update();
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

bool toggle = false;

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  Aversive::init();
  asserv_init();
  fpga_init();

  rds_stream.setMode(Stream::BINARY);
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
  
  t_pid.setGains(150, 2, 1);
  t_pid.setMaxIntegral(100);
  t_pid.setOutShift(8);

  //traj.gotoPosition(Vect<2, s32>(1000, 0), 500);

  while(Aversive::sync()) {

    if(traj.isEnded()) {
      if(toggle) {
	traj.gotoPosition(Vect<2, s32>(0, 0), 0);
      }
      else {
	traj.gotoPosition(Vect<2, s32>(0, -800), 0);
      }
      toggle = !toggle;
      rds_stream << 'p';
      //cmd_trajectory();
    }

    //cmd_print_infos();
    //cmd_print_pos();
    //cmd_pid_set();
    //cmd_dist_angle();
    //cmd_odo_config();
    
    if(robot.getValue()) {
      io << "Skating !!! \n";
      s32 dummy;
      io >> dummy;
      robot.unlock();
    }
  }

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
