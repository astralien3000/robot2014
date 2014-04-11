#include <aversive.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"

#include "curv_trajectory_manager.hpp"

#include "strategy.hpp"

#define F_CPU 16000000l
#include <util/delay.h>

Scheduler& sched = Scheduler::instance();

PidFilter pid_rt;
PidFilter pid_ct;
CurvTrajectoryManager traj(robot, odo, pos, pid_rt, pid_ct);

FpgaUartStream rds_io("rds_stream", UART_TX_1_DATA, UART_TX_1_OCUP, UART_RX_1_DATA, UART_RX_1_AVA);

Vect<2, s32> _cmd(0,0);
bool traj_mode = true;
s32 cmd_l = 0;
s32 cmd_r = 0;

void control_init(void) {
  Task t([](void) {
      //motc_l.setValue(cmd_l);
      //motc_r.setValue(cmd_r);
      //robot.setValue(_cmd);
      traj.update();
    });

  t.setPeriod(64000);
  t.setRepeat();
  sched.addTask(t);

  Interrupts::set();
}

void trajectory_reset(void) {
  pid_l.reset();
  pid_r.reset();
  pid_d.reset();
  pid_a.reset();

  qramp_d.reset(odo.getValue().coord(0));
  qramp_a.reset(odo.getValue().coord(1));

  pid_ct.reset();
  pid_rt.reset();

  traj.reset();
}

bool toggle = false;

void set_angle(s32 new_angle) {
  s32 old_angle = odo.getValue().coord(1) >> 4;
  s32 new_zero = new_angle - old_angle;
  //s32 dist = odo.getValue().coord(0);

  //s32 cur_dist = odo.getValue().coord(0);

  io << "Show zero\n";
  traj.gotoAngle(new_zero);
  while((odo.getValue().coord(1) >> 4) != -new_zero);

  io << "reset...\n";
  robot.lock();
  fpga_reset();
  fpga_config();
  trajectory_reset();
  robot.unlock();  

  while(odo.getValue().coord(1) != 0) {
  }
  
  io << "Go back to angle\n";
  io << new_angle << "\n";
  traj.gotoAngle(-new_angle);
  while((odo.getValue().coord(1) >> 4) != new_angle);
  io << "angle ok\n";
}

enum CoordCalib {
  X = 0,
  Y = 1
};

void goto_wall(s32 dist) {
  io << "goto wall\n";

  traj.gotoDistance(dist);

  while(!robot.getValue());

  io << "wall touched\n";
  
  traj.gotoDistance(-dist/100);

  robot.unlock();

  while(!robot.getValue());

  io << "touching wall\n";
  traj.reset();
  robot.unlock();
}

void match_init(bool red_side) {
  (void) red_side;

  traj_mode = false;

  io << "Goto wall\n";
  goto_wall(DIST_INIT);
  io << "Reset X and angle\n";
  set_angle(A_INIT);
  pos.setX(X_INIT);
  
  io << "Go far from the wall\n";
  traj.gotoDistance(DIST_2_INIT);
  while(!traj.isEnded()) {
    robot.unlock();
  }

  io << "Face buffet\n";
  traj.gotoAngle(A_INIT);
  while(!traj.isEnded());

  io << "Goto buffet\n";
  goto_wall(-3000);
  
  io << "Reset Y\n";
  pos.setY(Y_INIT);

  io << "Go far from the wall\n";
  traj.gotoDistance(500);
  while(!traj.isEnded()) {
    robot.unlock();
  }
  
  io << "Done !\n";
}

void side_init(bool red_side) {
  if(red_side) {
    io << "Init RED side\n";
    DIST_INIT = RED_DIST_INIT;
    DIST_2_INIT = RED_DIST_2_INIT;
    A_INIT = RED_A_INIT;
    X_INIT = RED_X_INIT;
    Y_INIT = RED_Y_INIT;
    ANGLE_CALIB = RED_ANGLE_CALIB;
  }
  else {
    io << "Init YELLOW side\n";
    DIST_INIT = YELLOW_DIST_INIT;
    DIST_2_INIT = YELLOW_DIST_2_INIT;
    A_INIT = YELLOW_A_INIT;
    X_INIT = YELLOW_X_INIT;
    Y_INIT = YELLOW_Y_INIT;
    ANGLE_CALIB = YELLOW_ANGLE_CALIB;
  }
}


int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  // Need to be the first thing to be done (for security)
  fpga_init();
  MOT_R = 0;
  MOT_L = 0;

  Aversive::init();
  asserv_init();

  robot.lock();

  //io.setMode(Stream::BINARY);
  rds_io.setMode(Stream::BINARY);
  file.setMode(Stream::BINARY);
  
  cmd.coord(0) = 0;
  cmd.coord(1) = 0;

  control_init();

  mot_l.inverse();
  enc_r.inverse();
    
  pid_ct.setGains(1000, 50, 100);
  pid_ct.setMaxIntegral(25600);
  pid_ct.setOutShift(10);

  pid_rt.setGains(20, 0, 0);
  pid_rt.setMaxIntegral(100000);
  pid_rt.setOutShift(14);

  robot.unlock();

  qramp_a.setFirstOrderLimit(15,15);
  qramp_a.setSecondOrderLimit(2,2);

  qramp_d.setFirstOrderLimit(5,5);
  qramp_d.setSecondOrderLimit(1,2);

  traj.setMode(TrajectoryManager::FASTER);

  s16 dummy = 0;
  while(!dummy) {
    io << "GO ?\n";
    io >> dummy;
  }

  side_init(dummy==1);
  match_init(dummy==1);
  
  robot.lock();
  io << "Place me please <3\n";
  io >> dummy;

  qramp_a.setFirstOrderLimit(30, 30);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(10,10);
  qramp_d.setSecondOrderLimit(2,2);

  trajectory_reset();
  robot.unlock();

  traj.gotoPosition(BEGIN_POINT);
  while(!traj.isEnded());

  traj.gotoPosition(FIRST_FIRE_POINT);
  while(!traj.isEnded());

  traj.gotoPosition(BEGIN_POINT);
  while(!traj.isEnded());

  traj.gotoPosition(SECOND_FIRE_POINT);
  while(!traj.isEnded());  

  traj.gotoPosition(FRONT_FRESQ_POINT);
  while(!traj.isEnded());  

  traj.setMode(TrajectoryManager::BACKWARD);
  traj.gotoPosition(FRESQ_POINT);
  while(!robot.getValue());  

  traj.setMode(TrajectoryManager::FASTER);
  traj.gotoDistance(500);
  while(!traj.isEnded()) {
    robot.unlock();
  } 

  while(Aversive::sync());

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
