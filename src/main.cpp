#include <aversive.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"
#include "astar.hpp"
#include "calibrate.hpp"
#include "trajectory.hpp"
#include "rds.hpp"
#include "avoidance.hpp"

#include "curv_trajectory_manager.hpp"

#include "master_action.hpp"
#include "harvest_action.hpp"
#include "hunt_action.hpp"

MasterAction act1(yellow_top_fire.p(), 180);
HarvestAction act2(red_tree.centre(), -90);
HuntAction act3(red_tree.centre(), 4);

#define F_CPU 16000000l
#include <util/delay.h>

static Scheduler& sched = Scheduler::instance();

void print_pos(void) {
  io << "(x " << pos.getValue().coord(0) << ", ";
  io << "y " << pos.getValue().coord(1) << ")\n";
}

void print_sp(void) {
  io << "STACK POINTER = " << SPH << " " << SPL << "\n";
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

  //rds_init();

  control_init();
  Interrupts::set();

  mot_l.inverse();
  enc_r.inverse();

//TEST ALARACHE BENOIT LANCEBALLE
  s16 wait = 0;
  io << "Enter a number\n";
  io >> wait;
  io << "Do action\n";
  act3.doAction();
  while(1);


  robot.unlock();

  

  qramp_a.setFirstOrderLimit(15,15);
  qramp_a.setSecondOrderLimit(2,2);

  qramp_d.setFirstOrderLimit(5,5);
  qramp_d.setSecondOrderLimit(1,2);

  traj.setMode(TrajectoryManager::FASTER);
  
  s16 dummy = 0;
  io << "Side \n";
  io >> dummy;
  io << dummy << "\n";

  side_init(dummy==1);
  match_init(dummy==1);

  print_pos();

  robot.lock();
  print_sp();
  io << "Place me please <3\n";
  io >> dummy;
  
  traj.setMode(TrajectoryManager::FASTER);
  traj.reset();
  trajectory_reset();
  robot.unlock();

  qramp_a.setFirstOrderLimit(40,40);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(13,13);
  qramp_d.setSecondOrderLimit(2,2);

  print_pos();
  
  traj.gotoDistance(300);
  while(!traj.isEnded()) {
    //check_for_collision();
  }

  print_pos();

  io << "Conf\n";
  MasterAction::setPositionManager(pos);
  MasterAction::setRobot(robot);
  MasterAction::setTrajectoryManager(traj);
  MasterAction::side = YELLOW;

  io << "Goto\n";
  traj.gotoPosition(act1.controlPoint());
  while(!traj.isEnded());

  print_pos();

  io << "Do action\n";
  act1.doAction();
  io << "DONE\n";

  print_pos();
  avoidance_goto(Vect<2, s32>(-600, 0));
  print_pos();
  while(1);
  // io << "Goto2\n";
  // traj.gotoPosition(act2.controlPoint());
  // while(!traj.isEnded());

  io << "Do action2\n";
  act2.doAction();
  io << "DONE2\n";

  while(1);

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
